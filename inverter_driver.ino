/*
 * Inverter driver app using a cycle-by-cycle FET switching algorithm.
 * This implementation is tailored for a split-phase system, which is
 * predominate in the North American residential market.
 * 
 * This app generates 2 reference sine waves and a FET switching clock.
 * Each line in the split-phase output is implemented with a gate driver
 * board that also includes a feedback circuit, where the output voltage
 * is passed through a resistor divider and compared with the reference
 * sine wave to generate a signal indicating whether the output voltage
 * is too high or too low.  That signal is used to switch the FETs during
 * a FET switching clock low-to-high transition.
 * 
 * This app istargeted specifically for the SAMD51 processor models that
 * have at least 2 DACs.  We go deep down the rabbit hole of SAMD51
 * specific peripheral code with this app.
 * 
 * The driver controller's main priority is to deliver quality reference sine
 * waves and the clock signal, which is very timing critical.  A separate
 * component called the interface controller communicates to this inverter
 * driver, and also battery charge controllers, BMS instances, user display
 * terminals, and smart load balancers.
 * 
 * This app runs in the Arduino environment.
 * 
 * This code has been tested and verified on an Adafruit Feather M4 Express
 * (SAMD51J19A) and an Adafruit ItsyBitsy M4 Express (SAMD51G19A).
 * For other SAMD51 boards, some code changes may be necessary.
 * 
 * This app does 3 tasks:
 * done 1. Generates a clock signal that determines the timing of MOSFET
 *         switching in the inverter.  The clock timing ranges from a few kHz
 *         to 1 MHz.
 * done 2. For each output line, generate a reference sine wave.  For split
 *         phase inverters, there will be 2 output sine waves with 180 degrees
 *         phase difference.  To control the output voltage, the reference
 *         sine wave can be throttled down.  Since the sine wave data initially
 *         uses the maximum limit of the DAC, the since wave cannot be throttled
 *         up.
 *      3. Responds to requests from a separate processor that controls the UI,
 *         UVLO and OVLO checks, and gets temperature readings from sensors on
 *         critical components.  The communication between this app and the
 *         separate processor can be UART (slow) or SPI (fast).
 * 
 * 
 * SWITCHING CLOCK
 * 
 * The output clock signal is generated one of two ways (slight preference for #1):
 * 
 *     1. Using the Si5351 clock chip.  This will provide a more precise clock
 *        than using the TC method described below.  For higher frequencies that
 *        might interfere with broadcast radio, the Si5351 can do spread spectrum,
 *        which can keep the communications authorities from knocking on your
 *        door.  Assuming you can find a library that implements it.  (Another
 *        item on the TODO list.)  The Adafruit breakout board is used for testing.
 * 
 *     2. Using a TC instance with the 48 MHz clock input with appropriate
 *        settings to get one of the following output frequencies.  Since the
 *        output is toggled in each TC match, the counter value should be 1/2
 *        of the total number of clock pulses in each cycle.
 * 
 *        Other frequencies are possible, these are just the most common.
 * 
 *           48 MHz
 *        /24  =   1 MHz
 *        /32  = 750 kHz
 *        /40  = 600 kHz
 *        /48  = 500 kHz
 *        /60  = 400 kHz
 *        /80  = 300 kHz
 *        /96  = 250 kHz
 *        /120 = 200 kHz
 *        /192 = 125 kHz
 *        /240 = 100 kHz
 *        /320 =  75 kHz
 *        /400 =  60 kHz
 *        /480 =  50 kHz
 *        /600 =  40 kHz
 * 
 * 
 * REFERENCE SINE WAVE
 * 
 * To generate the reference sine waves, we use a 4000 entry sine wave table
 * that is filled in at startup time.  The SAMD51 supports native floating 
 * point instructions, so this is pretty fast.  We generate interrupts at a
 * frequency range of 160-280 kHz to drive the DACs to generate sine waves
 * at frequencies from 40-70 Hz.
 * 
 * One sine wave table works for multiple output reference waves, by
 * giving each output a different index into the table.  If we ever
 * want to generate 3-phase outputs, the sine wave table size should
 * be divisible by 3, but we don't consider it for now, since the
 * devices we use only have two DACs.
 * 
 * Like with the switching clock, the sine wave generation can be driven by
 * either the Si5351 or a TC instance.  In this case, I have a strong preference
 * for using the Si5351, since it can be used to generate sine wave frequencies
 * accurate down to 0.01 Hz.  The TC instance is way less precise.
 * 
 * Note that we do not yet support synchronizing to an external grid.  The best
 * way to do that is with a dual second order generalized integrator phase lock
 * loop (DSOGI PLL).  This, too, is on the TODO list.
 * 
 * For an example of grid-tie synchronization using a SOGI PLL, see
 *     https://www.instructables.com/STM32-Duino-Grid-Tie-PLL/
 * 
 * 
 * CONTROL COMMUNICATIONS
 * 
 * The dialog between this machine (the inverter driver) and the main
 * controller (the interface controller) will support the following
 * requests:
 *  .  1. [aA] Get/set L1 throttle (1..2047)
 *  .  2. [bB] Get/set L2 throttle (1..2047)
 *  .  3. [cC] Get/set L1 on/off signal (0..1)
 *  .  4. [dD] Get/set L2 on/off signal (0..1)
 *  .  5. [nN] Get/set neutral bias (+- 200)
 *  .  6. [oO] Get/set main on/off signal (0..1)
 *  .  7. [fF] Get/set output frequency (4000..7000 cHz) (40..70 Hz)
 *  .  8. [sS] Get/set switching frequency (1..2000) kHz
 *  .  9. [zZ] Get/set switching frequency dither (-25..15)
 *             [-25..-1] are for down spread of -2.5% to -0.1%
 *             [1..15] are for center spread of +/-0.1% to +/-1.5%
 *  . 10. [gG] Get/set switching clock src; s or S for Si5351, t or T for TC
 *  . 11. [hH] Get/set sine clock src; s or S for Si5351, t or T for TC
 * 
 * TODO: Devise a start up sequence that will prime the bootstrap
 * capacitor.  To do this, generate a low reference, and enable the
 * inverter for 5-10 clock cycles.  This should cause the phase leg
 * hardware to turn on the lower MOSFET Q2 long enough to charge up
 * the bootstrap capacitor, after which the inverter should be able
 * to start and run normally.
 * 
 * If you wanted to use another processor, like the SAMD21:
 * SAMD21 processors have only 1 DAC and would thus be able to generate
 * only 1 reference sine wave.  You could, however, use an inverting unity
 * gain op amp configuration to generate the 2nd sine wave for the split
 * phase inverter.  However, as mentioned above, porting this SAMD51
 * specific code to another processor would be work.
 * 
 */

int firstVariable = 4321;

#include <Arduino.h>
#include <SAMD51_Dumpster.h>      // Debug code,  comment out for production
#include "si5351.h"               // Etherkit_Si351 works, no spread spectrum
                                  // PU2REO_Si5351ArduinoLite is similar (Si5351A only 3 outputs)
#include <CRC.h>
#include <Base64.h>

bool si5351Ready      = false;
bool si5351ClockReady = false;
bool si5351SineReady  = false;
bool tcClockReady     = false;
bool tcSineReady      = false;

Si5351 si5351;

// clock definitions
uint8_t switchClkSrc = 's';       // s/S for Si5351, t/T for TC instance
uint8_t sineClkSrc   = 's';       // s/S for Si5351, t/T for TC instance

const uint32_t kHz                      = 1000;
const uint32_t MHz                      = 1000000;

const double cpuClkFrequency            = 120 * MHz;
const uint32_t intrClockSourceFrequency = 48 * MHz;
const uint32_t clkClockSourceFrequency  = 48 * MHz;

// The output and PWM frequencies
const uint16_t outRmsVoltage = 120;
double outputFreq = 60.05;                 // 50 or 60 Hz, usually
uint16_t outputFreq_cHz = 6005;
const uint16_t minOutputFreq = 4000;       // 40 Hz in cHz
const uint16_t maxOutputFreq = 7000;       // 70 Hz in cHz

uint64_t switchFreq = 500 * kHz;
uint16_t switchFreq_kHz = 500;
const uint16_t minSwitchFreq = 1;          // 1 kHz
const uint16_t maxSwitchFreq = 2000;       // 2000 kHz

// Input/Output pins.  These work for both M4 Feather and M4 ItsyBitsy
uint32_t l1RefPin     = DAC0;          // reference sine wave for L1
uint32_t l2RefPin     = DAC1;          // reference sine wave for L2
uint32_t clockPin     = A5;            // output clock signal
uint32_t sineClkPin   = 9;             // sine clock input pin
uint32_t l1OnOffPin   = 10;            // on/off signal for L1
uint32_t l2OnOffPin   = 11;            // on/off signal for L2
uint32_t mainOnOffPin = 12;            // on/off signal for entire inverter

// loop timing variables
unsigned long numSeconds = 0;
unsigned long nextLoopMs = 0;
const unsigned long intervalMs = 1000;

// on/off signal values
uint8_t mainOnOff = 0;

// Since wave generation
const uint16_t numSamples = 4000;
const uint16_t numBits = 12;
const uint16_t maxDacValue = 1 << numBits;
const double midDacValue = maxDacValue / 2;
const uint16_t maxDacAmplitude = midDacValue - 1;
const uint16_t minDacAmplitude = 0 - maxDacAmplitude;
double rawSineData[numSamples];

/*
 * One design assumption is that we have top, mid, and bottom connections to the
 * battery powering the inverter.  Generally, the mid point of the battery is
 * connected to neutral of the output.  The neutral bias shifts the reference sine
 * wave up or down relative to neutral, and can be used as a mechanism to balance
 * the top and bottom halves of the battery.  An active BMS balancer then only has
 * to balance across cells in each battery half, rather than across cells in the
 * entire pack.
 * 
 * The neutral bias is applied equally to both L1 and L2.
 */
const uint16_t minNeutralBias = -200;
const uint16_t maxNeutralBias = 200;
uint16_t neutralBias = 0;

/*
 * Data for generating an output reference sine wave for a single channel.
 * Since each line may have a different load, it will have a different throttle
 * setting, so we need separate arrays for L1 and L2.  By having two sine data
 * arrays, we can calculate a new array without unexpected interaction with the
 * generation of the sine wave.  After calculation, we change the pointner
 * sineData to reference the updated data array.
 */
typedef struct {
  uint16_t throttle;
  uint16_t index;
  uint16_t sineDataA[numSamples];
  uint16_t sineDataB[numSamples];
  uint16_t* sineData;
  bool usingDataA;
  uint8_t onOff;
  uint8_t onOffPin;
} LineData;

LineData l1;
LineData l2;
  
const uint16_t maxThrottle = 2047;
const uint16_t defaultThrottle = 1850;

// Interrupt request handler stats for the timer driving the sine wave generation
volatile uint32_t tc2IrqNumRaw = 0;
volatile uint32_t tc2IrqElapsed = 0;
volatile uint8_t  tc2IrqFlagsLast = 0;
volatile uint32_t tc2IrqFlagErrors = 0;

volatile uint32_t si5351IrqNumRaw = 0;
volatile uint32_t si5351IrqElapsed = 0;

/*
 * Definition of requests and response formats between the interface controller
 * and this driver controller.  The driver controller's main priority is to
 * deliver quality reference sine waves and clock signal, which is very timing
 * critical.  The interface controller communicates to this inverter driver, 
 * battery charge controllers, BMS instances, user display terminals, and smart
 * load balancers.
 * 
 * Requests and responses have the same format and are always 32 bits long in
 * binary form.  If the communication between the two controllers is over UART,
 * the in-transit form of the request/response may be converted to ASCII, but
 * it is converted back to 32-bit binary in the receiver.
 */
typedef struct {
  uint8_t cmd;
  uint8_t err;
  int32_t value;  // or int16_t[2]?
} RemoteCmd;


volatile uint32_t invalidCmdErr = 0;

// Buffer for communicating to/from the main controller.
const uint8_t cbufSize = 64;
const uint8_t cmdSizeEnc = (sizeof(RemoteCmd) + 2) / 3 * 4;
const uint8_t crcSize = sizeof(uint32_t);
const uint8_t crcSizeEnc = (crcSize + 2) / 3 * 4;
const uint8_t cmdSizeChk = cmdSizeEnc + crcSizeEnc;

char cbuf[cbufSize];
char *cbufPtr = &cbuf[0];

RemoteCmd inCmd;

SAMD51_Dumpster ilsd;

/*
 * Initialize the inverter.
 */
void setup() {
  setupSerial();

  dumpMemoryAddresses();
  dumpSamdPeripherals();

  setupDigitalPins();
  setupReferenceWaves();
  setupClockOutput();
  setupComms();

  dumpSamdPeripherals();

  testSerial();
  
  Serial.printf("Setup complete:\n\n");
}

/**
 * Set up the serial interface.  Delay units are in milliseconds.
 */
void setupSerial() {
  Serial.begin(115200);
  
  const int delayIncr = 100;
  const int maxDelay = 2000;
  for (int i = 0; i < maxDelay; i += delayIncr) {
    delay(delayIncr);
    Serial.print(".");
    Serial.flush();
  }
  
  Serial.println(" ");
  Serial.println("\n\nInverterV8\n");
}

/*
 * Setup the digital output pins for the various on/off signals.
 */
void setupDigitalPins() {
  pinMode(l1OnOffPin, OUTPUT);
  pinMode(l2OnOffPin, OUTPUT);
  pinMode(mainOnOffPin, OUTPUT);
  
  digitalWrite(l1OnOffPin, LOW);
  digitalWrite(l2OnOffPin, LOW);
  digitalWrite(mainOnOffPin, LOW);
}

/*
 * Set up generating the reference sine waves.
 */
void setupReferenceWaves() {
  setupSineData();
  initDac();
  analogWriteDac(0, 0);
  setupSineInterrupt();
}

/*
 * Set up the raw sine wave data.
 * Raw sine wave data over the range 0..2*pi is -1..+1.
 * Adjust those real values -1..+1 to the integer range 0..4095, centered at 2048.
 */
void setupSineData() {
  uint32_t startTicks = DWT->CYCCNT;

  // Constants to save calculations in the loops
  const double pi = 3.14159265359;
  const double pi2 = 2.0 * pi;
  const double dNumSamples = numSamples;
  
  // initialize the line structs for L1 and L2
  l1.index = 0;
  l2.index = numSamples / 2;
  l1.throttle = defaultThrottle;
  l2.throttle = defaultThrottle;
  l1.usingDataA = 1;
  l2.usingDataA = 1;
  l1.sineData = l1.sineDataA;
  l2.sineData = l2.sineDataA;
  l1.onOff = 0;
  l2.onOff = 0;
  l1.onOffPin = l1OnOffPin;
  l2.onOffPin = l2OnOffPin;
  
  for (int i = 0; i < numSamples; i++) {
    rawSineData[i] = sin((double) i * pi2 / dNumSamples);
  }
  
  updateSineData(&l1);
  updateSineData(&l2);
  
  uint32_t elapsedTicks = DWT->CYCCNT - startTicks;
  double elapsedMs = (double) elapsedTicks * 1000.0 / cpuClkFrequency;
  Serial.printf("Elapsed time ticks=%d  ms=", elapsedTicks);
  Serial.println(elapsedMs, 3);
}

/*
 * Update the sine wave array based on current throttle and bias settings.
 * Note that there are separate throttle settings for each output line.
 * There is only one bias setting used by both lines.
 * 
 * TODO: Add an additional per-line bias setting?
 * 
 * The throttle adjusts the amplitude of the sine wave.  With this mechanism
 * one can adjust the output voltage in software without having to make any
 * hardware changes.
 * 
 * The bias shifts the sine wave up or down by some (usually very small) amount.
 * The bias has two functions:
 *     1. To compensate for slight hardware tolerances that may result in
 *        the output sine wave not being centered at zero volts.
 *     2. To force the sine wave up or down slightly, to be used as a very
 *        coarse battery balance mechanism between the top and bottom halves
 *        of the battery pack.  The BMS can then balance the two halves of
 *        the battery independently, which is really helpful for active
 *        balancers moving charge up and down the cell stacks.
 * 
 * Currently, the bias is a straight shifting of the sine wave.  This preserves
 * the harmonic shape of the wave, but it introduces frequency jitter for any
 * components that are detecting zero crossing of their power source.  If this
 * frequency jitter is an issue, a solution would be to calculate the upper and
 * lower lobes of the sine wave by adding the bias to one, and subtracting it
 * from the other.  This would preserve the zero crossing frequency integrity at
 * the expense of some minor total harmonic distortion of the sine wave.
 * 
 * A final option would be to implement both the shift bias and the amplitude bias.
 */
void updateSineData(LineData* ld) {
  uint16_t* nextSineData = ld->usingDataA ? ld->sineDataB : ld->sineDataA;
  
  uint16_t effectiveNeutralBias = neutralBias;
  if (effectiveNeutralBias + ld->throttle > maxDacAmplitude) {
    effectiveNeutralBias = maxDacAmplitude - ld->throttle;
  } else if (effectiveNeutralBias - ld->throttle < minDacAmplitude) {
    effectiveNeutralBias = minDacAmplitude + ld->throttle;
  }
  
  for (int i = 0; i < numSamples; i++) {
    nextSineData[i] = rawSineData[i] * ld->throttle + midDacValue + effectiveNeutralBias;
  }
  
  ld->usingDataA = !ld->usingDataA;
  ld->sineData = nextSineData;
}

/*
 * Slight modification of the SAMD51 code in wiring.c.
 */
void initDac() {
  GCLK->PCHCTRL[DAC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 4 (12mhz)
  while (GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN == 0);
	
  while ( DAC->SYNCBUSY.bit.SWRST == 1 );
  DAC->CTRLA.bit.SWRST = 1;
  while ( DAC->SYNCBUSY.bit.SWRST == 1 );
	
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPU; // TODO: fix this once silicon bug is fixed
	
  // set refresh rates and bump the current control up a little bit
  DAC->DACCTRL[0].bit.REFRESH = 2;
  DAC->DACCTRL[1].bit.REFRESH = 2;
  
  // bump the current control up a little bit
  DAC->DACCTRL[0].bit.CCTRL = DAC_DACCTRL_CCTRL_CC1M_Val;
  DAC->DACCTRL[1].bit.CCTRL = DAC_DACCTRL_CCTRL_CC1M_Val;
}

/*
 * Extracted from wiring_analog.c
 */
void myPinPeripheral(uint32_t ulPin, EPioType ulPeripheral) {
  if ( g_APinDescription[ulPin].ulPin & 1 ) // is pin odd?
  {
    uint32_t temp ;

    // Get whole current setup for both odd and even pins and remove odd one
    temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
    // Set new muxing
    PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXO( ulPeripheral ) ;
    // Enable port mux
    PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
  }
  else // even pin
  {
    uint32_t temp ;

    temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
    PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXE( ulPeripheral ) ;
    PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR ; // Enable port mux
  }
}

/*
 * Extracted the SAMD51-specific code from wiring_analog.c, for just the
 * two pins (A0 and A1) which have DAC support.
 * 
 * We want all the functionality of this code without the incredibly long
 * 10 ms delay when enabling the DACs.  Is that really neccessary?
 * 
 * Set up output pins for both DAC0 and DAC1 in 1 call, and suffer the long
 * timeout only once.
 * 
 * This method should be called only once when starting the ref sine waves.
 * After this call, we update the DAC values from the interrupt handler.
 */
void analogWriteDac(uint16_t value0, uint16_t value1)
{
  PinDescription pinDesc0 = g_APinDescription[l1RefPin];
  PinDescription pinDesc1 = g_APinDescription[l2RefPin];
  uint32_t attr0 = pinDesc0.ulPinAttribute;
  uint32_t attr1 = pinDesc1.ulPinAttribute;

  // value0 = mapResolution(value0, _writeResolution, _dacResolution);
  // value1 = mapResolution(value1, _writeResolution, _dacResolution);

  myPinPeripheral(l1RefPin, PIO_ANALOG);
  myPinPeripheral(l2RefPin, PIO_ANALOG);

  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
  DAC->CTRLA.bit.ENABLE = 0;                     // disable DAC

  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
  DAC->DACCTRL[0].bit.ENABLE = 1;
  DAC->DACCTRL[1].bit.ENABLE = 1;

  while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
  DAC->CTRLA.bit.ENABLE = 1;                     // enable DAC

  while ( !DAC->STATUS.bit.READY0 );             // do the throw-away write, without waiting for enable?
  while (DAC->SYNCBUSY.bit.DATA0);
  DAC->DATA[0].reg = value0;
      
  while ( !DAC->STATUS.bit.READY1 );
  while (DAC->SYNCBUSY.bit.DATA1);
  DAC->DATA[1].reg = value1;

  delayMicroseconds(10000);                      // 10 msec is an eternity

  //ERROR!
  while(!DAC->DACCTRL[0].bit.ENABLE);            // why wait here, instead of above?
  while(!DAC->DACCTRL[1].bit.ENABLE);

  while ( !DAC->STATUS.bit.READY0 );             // do the real write
  while (DAC->SYNCBUSY.bit.DATA0);
  DAC->DATA[0].reg = value0;
      
  while ( !DAC->STATUS.bit.READY1 );
  while (DAC->SYNCBUSY.bit.DATA1);
  DAC->DATA[1].reg = value1;
}

/*
 * Set up the TC2 instance to generate an interrupt at the intervals
 * required for the configured output sine wave frequency.
 */
void setupSineInterrupt() {
  if (sineClkSrc == 't' || sineClkSrc == 'T') {
    // use a TC
    setupTcSineInterrupt();
  } else if (sineClkSrc == 's' || sineClkSrc == 'S'){
    // use the Si5351 chip
    setupSi5351SineInterrupt();
  } else {
    Serial.printf("Error: invalid sineClkSrc value = 0x%x\n", sineClkSrc);
  }
}

/*
 * Set up the TC2 instance to generate an interrupt at the intervals
 * required for the configured output sine wave frequency.
 */
void setupTcSineInterrupt() {
  uint16_t period = (uint16_t) ((double) intrClockSourceFrequency / ((double) numSamples * outputFreq));
  Serial.printf("numSamples=%d  period=%d  outputFreq=", numSamples, period);
  Serial.println(outputFreq, 3);
  
  // First, enable TC2 MCLK APBB interface
  if (! MCLK->APBBMASK.bit.TC2_) {
    MCLK->APBBMASK.bit.TC2_ = 1;
  }

  // Config the GCLK peripheral controller to use clock generator 1, and enable it
  GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

  Tc* TCx = TC2;                                         // Configure TC2

  TCx->COUNT16.CTRLA.bit.SWRST = 1;                      //reset
  while (TCx->COUNT16.SYNCBUSY.bit.SWRST);

  TCx->COUNT16.CTRLA.bit.ENABLE = 0;                     // Disable
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
  
  TCx->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;        // Set Timer counter Mode to 16 bits
  TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;          // Set normal frequency mode

  while (TCx->COUNT16.SYNCBUSY.bit.CC0);                 // Set the initial value
  TCx->COUNT16.CC[0].reg = period;
  while (TCx->COUNT16.SYNCBUSY.bit.CC0);

  NVIC_SetPriority(TC2_IRQn, 1);                         // Enable NVIC interrupts for TC2
  NVIC_EnableIRQ(TC2_IRQn);
  
  TCx->COUNT16.INTENSET.bit.OVF = 1;                     // Enable overflow interrupt

  TCx->COUNT16.CTRLA.bit.ENABLE = 1;                     // Enable TCx
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
  
  tcSineReady = true;
}

/*
 * Handler for TC2 interrupts.
 */
void TC2_Handler() {
  uint32_t startTicks = DWT->CYCCNT;
  tc2IrqNumRaw += 1;
  
  uint8_t intFlags = TC2->COUNT16.INTFLAG.reg;
  tc2IrqFlagsLast = intFlags;
  
  if (intFlags & TC_INTENSET_OVF) {
    while ( !DAC->STATUS.bit.READY0 );         // update DAC0 value for line 1
    while (DAC->SYNCBUSY.bit.DATA0);
    DAC->DATA[0].reg = l1.sineData[l1.index];
    l1.index += 1;
    if (l1.index >= numSamples) {
      l1.index = 0;
    }
      
    while ( !DAC->STATUS.bit.READY1 );         // update DAC1 value for line 2
    while (DAC->SYNCBUSY.bit.DATA1);
    DAC->DATA[1].reg = l2.sineData[l2.index];
    l2.index += 1;
    if (l2.index >= numSamples) {
      l2.index = 0;
    }
    
    intFlags &= !TC_INTENSET_OVF;
  }
  if (intFlags) {
    // It is unexpected to have any interrupt other than OVF
    tc2IrqFlagErrors += 1;
  }
  
  // clear all the pending interrupts
  TC2->COUNT16.INTFLAG.reg = tc2IrqFlagsLast;
  
  tc2IrqElapsed += DWT->CYCCNT - startTicks;
}

/*
 * Setup the Si5351 to generate interrupts to drive the DAC.
 */
void setupSi5351SineInterrupt() {
  setupSi5351();
  
  if (! si5351Ready) {
    Serial.printf("ERROR: Si5351 is not ready in setupSi5351SineInterrupt\n");
    return;
  }

  // ask for 100x higher frequency that we really want.
  uint64_t sineIntrFreq = (uint64_t) outputFreq_cHz * (uint64_t) numSamples;
  si5351.set_freq(sineIntrFreq, SI5351_CLK2);       // set up clock 2
  
  attachInterrupt(digitalPinToInterrupt(sineClkPin), si5351Sine_Handler, RISING);
  
  si5351SineReady = true;
}

/*
 * Handler for Si5351 sine interrupts.
 */
void si5351Sine_Handler() {
  uint32_t startTicks = DWT->CYCCNT;
  si5351IrqNumRaw += 1;
  
  while ( !DAC->STATUS.bit.READY0 );         // update DAC0 value for line 1
  while (DAC->SYNCBUSY.bit.DATA0);
  DAC->DATA[0].reg = l1.sineData[l1.index];
  l1.index += 1;
  if (l1.index >= numSamples) {
    l1.index = 0;
  }
      
  while ( !DAC->STATUS.bit.READY1 );         // update DAC1 value for line 2
  while (DAC->SYNCBUSY.bit.DATA1);
  DAC->DATA[1].reg = l2.sineData[l2.index];
  l2.index += 1;
  if (l2.index >= numSamples) {
    l2.index = 0;
  }
      
  si5351IrqElapsed += DWT->CYCCNT - startTicks;
}

/*
 * Disable the sine interrupt.
 */
void disableSineInterrupt() {
  if (sineClkSrc == 't' || sineClkSrc == 'T') {
    disableTcSineInterrupt();
  } else if (sineClkSrc == 's' || sineClkSrc == 'S'){
    disableSi5351SineInterrupt();
  } else {
    Serial.printf("Error: invalid sineClkSrc value = 0x%x\n", sineClkSrc);
  }
}

/*
 * Disable the TC based sine interrupt.
 */
void disableTcSineInterrupt() {
  if (tcSineReady) {
    NVIC_DisableIRQ(TC2_IRQn);
    tcSineReady = false;
  }
}

/*
 * Disable the Si5351 based sine interrupt by detaching it.
 */
void disableSi5351SineInterrupt() {
  if (si5351SineReady) {
    detachInterrupt(digitalPinToInterrupt(sineClkPin));
    // si5351.set_clock_disable(SI5351_CLK2, SI5351_CLK_DISABLE_HI_Z);
    si5351SineReady = false;
  }
}


/*
 * Set up a switching clock output signal using either:
 * a. A TC peripheral
 * b. A Si5351 clock chip
 */
void setupClockOutput() {
  if (switchClkSrc == 't' || switchClkSrc == 'T') {
    // use a TC
    setupTcClockOutput();
  } else if (switchClkSrc == 's' || switchClkSrc == 'S'){
    // use the Si5351
    setupSi5351ClockOutput();
  } else {
    Serial.printf("Error: invalid switchClkSrc value = 0x%x\n", switchClkSrc);
  }
}

/*
 * Setup a TC instance for the clock output signal.
 * 
 * These values are for the Feather M4 Express (SAMD51J19A):
 *     TCC0 IOSET 6
 *     TCC1 IOSET 1
 *     TC0 IOSET 1 -- PA04, PA05 pins A4, A1     bus APBA
 *     TC1 IOSET 1 -- PA06, PA07 pins A5, xx     bus APBA
 *     TC2 IOSET 2 -- PA12, PA13 pins SDA, SCL   bus APBB
 *     TC3 IOSET 1 -- PA14, PA15 pins 4, xx      bus APBB
 *     TC4 IOSET 1 -- PB08, PB09 pins A2, A3     bus APBC
 *     TC5                                       bus APBC
 */
void setupTcClockOutput() {
  // The switch frequency will be the clock freq (48MHz) divided by the period
  uint16_t period = clkClockSourceFrequency / (switchFreq * 2);
  Serial.printf("Setting up clock output: period=%d\n", period);

  // Call analogWrite to set up the output pin, then manually set up the TC the way we want it.
  // If you change the clockPin, you must also update the TC instance configured for that pin.
  analogWrite(clockPin, 100);
  
  // Then, enable TC MCLK APB interface
  if (! MCLK->APBAMASK.bit.TC1_) {
    MCLK->APBAMASK.bit.TC1_ = 1;
  }

  // Config the GCLK peripheral controller to use clock generator 1 (48 MHz), and enable it
  volatile GCLK_PCHCTRL_Type* pch = &(GCLK->PCHCTRL[TC1_GCLK_ID]);
  pch->bit.CHEN = 0;
  while (pch->bit.CHEN);
  
  pch->bit.GEN = GCLK_PCHCTRL_GEN_GCLK1_Val;
  pch->bit.CHEN = 1;
  while (pch->bit.CHEN == 0);

  Tc* TCx = TC1;	                                   // Configure TC

  TCx->COUNT16.CTRLA.bit.SWRST = 1;                  //reset
  while (TCx->COUNT16.SYNCBUSY.bit.SWRST);

  TCx->COUNT16.CTRLA.bit.ENABLE = 0;                 // Disable
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
  
  TCx->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;    // Set Timer counter Mode to 16 bits
  TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;      // Set match frequency mode

  while (TCx->COUNT16.SYNCBUSY.bit.CC0);	           // Set the period
  TCx->COUNT16.CC[0].reg = period;
  while (TCx->COUNT16.SYNCBUSY.bit.CC0);

	TCx->COUNT16.CTRLBCLR.bit.LUPD = 1;                // Enable runtime update of the period
	while (TCx->COUNT16.SYNCBUSY.bit.CTRLB);

  TCx->COUNT16.CTRLA.bit.ENABLE = 1;	               // Enable TCx
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
  
  tcClockReady = true;
}

/*
 * Initialize a Si5351 clock generator chip.  This function is idempotent,
 * and may be called for clock output and/or sine wave interrupts.
 */
void setupSi5351() {
  if (si5351Ready) {
    // already initialized
    return;
  }
  
  Serial.println("Setting up Si5351 device");

  // Initialize the chip. Zero in the 2nd param below is interpreted
  // as a 25 MHz crystal input.
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);
  if (!i2c_found) {
    Serial.println("Error: Si5351 device not found on I2C bus!");
    return;
  }
  
  // Always use clock 2 for the sine wave and configure it to run on PLLB
  // so that if we set spread spectrum (only available on PLLA) for the
  // output clock, it won't perturb the sine wave output.
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  
  si5351Ready = true;
}

  
/*
 * Set up a Si5351 clock generator chip to generate the switching clock
 * output. The advantage of this method is greater flexibility in selecting
 * the output frequency, more precise output, and the ability to turn
 * on spread spectrum dithering of the clock signal to reduce EMi.
 * 
 * Clock output is always on Si5351 clock 0.
 */
void setupSi5351ClockOutput() {
  // If we're switching from TC to Si5351 at run time, configure the
  // TC output pin to input to force it to high impedance state, so
  // that it can be wired directly to the Si5351 clock output.
  pinMode(clockPin, INPUT);
  
  setupSi5351();
  updateSi5351ClockFrequency();
}

/*
 * Update the output clock frequency.
 */
void updateOutputClockFrequency() {
  if (switchClkSrc == 't' || switchClkSrc == 'T') {
    updateTcClockFrequency();
  } else if (switchClkSrc == 's' || switchClkSrc == 'S'){
    updateSi5351ClockFrequency();
  } else {
    Serial.printf("Error: invalid switchClkSrc value = 0x%x\n", switchClkSrc);
  }
}

/*
 * Update the period in the output clock TC, which will change the output
 * clock frequency that is generated.  Write the new period to the CCBUF[0]
 * field, which will get transferred to CC[0] at the next UPDATE condition.
 * This allows jitter-free updates of the clock frequency at run time.
 */
void updateTcClockFrequency() {
  if (tcClockReady) {
    // The switch frequency will be the clock freq (48MHz) divided by the period
    uint16_t period = clkClockSourceFrequency / (switchFreq * 2);
    Serial.printf("Updating clock output: period=%d\n", period);

    Tc* TCx = TC1;	                                   // Configure TC

	  if (! TCx->COUNT16.STATUS.bit.CCBUFV0) {
	    TCx->COUNT16.CCBUF[0].reg = period;
	  }
  }
}

/*
 * If the Si5351 has been set up and is running, update the clock output frequency.
 */
void updateSi5351ClockFrequency() {
  if (si5351Ready) {
    uint64_t freq0 = switchFreq * 100ULL;      // library sets up for divide by 100
  
    si5351.set_freq(freq0, SI5351_CLK0);       // set up clock 0
  }
}

void disableClockOutput() {
  if (switchClkSrc == 't' || switchClkSrc == 'T') {
    disableTcClockOutput();
  } else if (switchClkSrc == 's' || switchClkSrc == 'S'){
    disableSi5351ClockOutput();
  } else {
    Serial.printf("Error: invalid switchClkSrc value = 0x%x\n", switchClkSrc);
  }
}

void disableTcClockOutput() {
  // TODO: implement me
}

void disableSi5351ClockOutput() {
  // TODO: implement me
}

/*
 * Dump an address of code, memory and the stack, just to get a feel for where the
 * segments are being located.  According to the SAMD51 datasheet:
 *     code/flash:  00000000 - flash size
 *     SRAM:        20000000 - 20040000
 *     peripherals: 40000000 - 48000000
 *     system:      e0000000 - ffffffff
 */
void dumpMemoryAddresses() {
  int32_t x = 1234;
  Serial.printf("Memory map:\n    Code:  %08x\n    Data:  %08x\n    Stack: %08x\n",
                (uint32_t) &setup, (uint32_t) &firstVariable, (uint32_t) &x);
}

void dumpSamdPeripherals() {
  ilsd.dumpGCLK(NULL);
  ilsd.dumpMCLK(NULL);
  ilsd.dumpTC(NULL);
  ilsd.dumpPort(NULL);
  ilsd.dumpSERCOM(NULL);
}

/*
 * Setup the communication channel to the interface controller.
 */
void setupComms() {
  Serial1.begin(115200);
}

/*
 * Handle receiving request byte(s) from the inverter interface controller.
 */
void recvReqByte_Handler(int numBytes) {
  Serial.printf("reading %d bytes into buffer containing %d bytes\n", numBytes, cbufPtr - cbuf);
  // read numBytes into cbuf
  int recvdBytes = Serial1.readBytes(cbufPtr, numBytes);
  cbufPtr += recvdBytes;
  int bufBytes = cbufPtr - cbuf;
  
  // if we have enough bytes for a command, process it
  if (bufBytes >= cmdSizeChk) {
    recvRequest(&cbuf[0]);
    
    // Copy any additional chars from the end to the front of the buffer,
    // in case we get multiple commands at once.
    char* src = &cbuf[cmdSizeChk];
    char* dst = &cbuf[0];
    while (src < cbufPtr) {
      char ch = *src++;
      // stop at null char
      if (ch == '\0') {
        break;
      }
      // don't copy CR, LF or space
      if (ch != '\n' && ch != '\r' && ch != ' ') {
        *dst++ = ch;
      }
    }
    cbufPtr = dst;
  }
}

/*
 * Process a received request.
 *     1. [aA] Get/set L1 throttle (1..2047)
 *     2. [bB] Get/set L2 throttle (1..2047)
 *     3. [cC] Get/set L1 on/off signal (0..1)
 *     4. [dD] Get/set L2 on/off signal (0..1)
 *     5. [nN] Get/set neutral bias (+- 200)
 *     6. [oO] Get/set main on/off signal (0..1)
 *     7. [fF] Get/set output frequency (4000..7000 cHz) (40.00 .. 70.00 Hz)
 *     8. [sS] Get/set switching frequency (1..2000) kHz
 *     9. [zZ] Get/set switching frequency dither (-25..15)
 *             [-25..-1] are for down spread of -2.5% to -0.1%
 *             [1..15] are for center spread of +/-0.1% to +/-1.5%
 *    10. [gG] Get/set switching clock src; s or S for Si5351, t or T for TC
 *    11. [hH] Get/set sine clock src; s or S for Si5351, t or T for TC
 */
void processRequest(RemoteCmd *cmd) {
  uint8_t cmdval = cmd->value | 0x20;     // convert to lower case
  uint8_t oldval = sineClkSrc | 0x20;
  
  switch (cmd->cmd) {
    case 'a':                            // get L1 throttle
      cmd->err = 0;
      cmd->value = l1.throttle;
      sendResponse(cmd);
      break;
    case 'A':                            // set L1 throttle
      if (cmd->value < 1 || cmd->value > maxThrottle) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        l1.throttle = cmd->value;
        updateSineData(&l1);
      }
      sendResponse(cmd);
      break;
    case 'b':                            // get L2 throttle
      cmd->err = 0;
      cmd->value = l2.throttle;
      sendResponse(cmd);
      break;
    case 'B':                            // set L2 throttle
      if (cmd->value < 1 || cmd->value > maxThrottle) {
        cmd->err = 1;
      } else {

        cmd->err = 0;
        l2.throttle = cmd->value;
        updateSineData(&l2);
      }
      sendResponse(cmd);
      break;
    case 'c':                            // get L1 on/off signal value
      cmd->err = 0;
      cmd->value = l1.onOff;
      sendResponse(cmd);
      break;
    case 'C':                            // set L1 on/off signal value
      if (cmd->value > 1) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        l1.onOff = cmd->value;
        digitalWrite(l1.onOffPin, l1.onOff);
      }
      sendResponse(cmd);
      break;
    case 'd':                            // get L2 on/off signal value
      cmd->err = 0;
      cmd->value = l2.onOff;
      sendResponse(cmd);
      break;
    case 'D':                            // set L2 on/off signal value
      if (cmd->value > 1) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        l2.onOff = cmd->value;
        digitalWrite(l2.onOffPin, l2.onOff);
      }
      sendResponse(cmd);
      break;
    case 'n':                            // get neutral bias
      cmd->err = 0;
      cmd->value = neutralBias;
      sendResponse(cmd);
      break;
    case 'N':                            // set neutral bias
      if (cmd->value < minNeutralBias || cmd->value > maxNeutralBias) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        neutralBias = cmd->value;
        updateSineData(&l1);
        updateSineData(&l2);
      }
      sendResponse(cmd);
      break;
    case 'o':                            // get main on/off signal value
      cmd->err = 0;
      cmd->value = mainOnOff;
      sendResponse(cmd);
      break;
    case 'O':                            // set main on/off signal value
      // when changing from 0 to 1, do special startup sequence to prime bootstrap capacitor
      if (cmd->value > 1) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        mainOnOff = cmd->value;
        digitalWrite(mainOnOffPin, mainOnOff);
      }
      sendResponse(cmd);
      break;
    case 'f':                            // get output frequency
      cmd->err = 0;
      cmd->value = outputFreq_cHz;
      sendResponse(cmd);
      break;
    case 'F':                            // set output frequency
      if (cmd->value < minOutputFreq || cmd->value > maxOutputFreq) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        outputFreq_cHz = cmd->value;
        outputFreq = (double) outputFreq_cHz / 100.0;
        // updateSineClock();
        // setupSineInterrupt();
      }
      sendResponse(cmd);
      break;
    case 's':                            // get switching frequency
      cmd->err = 0;
      cmd->value = switchFreq_kHz;
      sendResponse(cmd);
      break;
    case 'S':                            // set switching frequency
      if (cmd->value < minSwitchFreq || cmd->value > maxSwitchFreq) {
        cmd->err = 1;
      } else {
        cmd->err = 0;
        switchFreq_kHz = cmd->value;
        switchFreq = switchFreq_kHz * 1000;
        updateOutputClockFrequency();
      }
      sendResponse(cmd);
      break;
    case 'g':                            // get switching clock source
      cmd->err = 0;
      cmd->value = switchClkSrc;
      sendResponse(cmd);
      break;
    case 'G':                            // set switching clock source
      oldval = switchClkSrc & 0x20;      // convert to lower case;
      
      if (mainOnOff || (cmdval != 's' && cmdval != 't')) {
        cmd->err = 1;
      } else if (cmdval != oldval) {
        cmd->err = 0;
        disableClockOutput();
        switchClkSrc = cmd->value;
        setupClockOutput();
      }
      sendResponse(cmd);
      break;
    case 'h':                            // get sine clock source
      cmd->err = 0;
      cmd->value = sineClkSrc;
      sendResponse(cmd);
      break;
    case 'H':                            // set sine clock source
      if (mainOnOff || (cmdval != 's' && cmdval != 't')) {
        cmd->err = 1;
      } else if (cmdval != oldval) {
        cmd->err = 0;
        disableSineInterrupt();
        sineClkSrc = cmd->value;
        setupSineInterrupt();
      }
      break;
    default:
      invalidCmdErr += 1;
      cmd->err = 1;
      cmd->value = cmd->cmd;
      sendResponse(cmd);
      break;
  }
}

/*
 * Recieves a request from the interface controller.
 */
void recvRequest(char* buf) {
  dumpEncodedCmd(buf, "Request Enc");
  
  uint32_t crcVal = calcCRC32((uint8_t*)buf, cmdSizeEnc);
  if (crcVal != buf[cmdSizeEnc]) {
    invalidCmdErr += 1;
    Serial.printf("checksum validation error\n");
  } else {
    // checksum is good, decode the command and process it
    Base64.decode((char*) &inCmd, buf, cmdSizeEnc);
    dumpRemoteCmd(&inCmd, "Request Raw");
    processRequest(&inCmd);
  }
}

/*
 * Sends a response to the interface controller.
 */
void sendResponse(RemoteCmd *cmd) {
  dumpRemoteCmd(cmd, "Response Raw");
  
  char outBuf[16];
  
  memset(outBuf, 0, sizeof(outBuf));                                 // zero the buffer
  Base64.encode(outBuf, (char*) cmd, sizeof(RemoteCmd));             // encode to base64
  outBuf[cmdSizeEnc] = calcCRC32((uint8_t *)outBuf, cmdSizeEnc);     // add checksum
  
  dumpEncodedCmd(outBuf, "Response Enc");
  
  Serial1.print(outBuf);                                             // write data
  Serial1.flush();
}

/*
 * Print a remote command.
 */
void dumpRemoteCmd(RemoteCmd* rc, const char* msg) {
  Serial.printf("%s: ", msg != NULL ? msg : "RemoteCmd");
  
  uint8_t ch = rc->cmd;
  if (ch < ' ' || ch > '~') {
    ch = '.';
  }
  Serial.printf(" cmd=%c [%02x] err=%d val=%d\n", ch, rc->cmd, rc->err, rc->value);
}

/*
 * Dump a RemoteCmd that is base64-encoded, and has a CRC checksum byte.
 */
void dumpEncodedCmd(char* ec, const char* msg) {
  printf("%s: ", msg != NULL ? msg : "EncodedCmd");
  
  for (char* p = cbuf; p < cbuf + cmdSizeChk; p++) {
    if (*p >= ' ' && *p <= '~') {
      printf("%c", *p);
    } else {
      printf(" %02x ", *p);
    }
  }
  printf("\n");
}

/*
 * Test serialization and CRC for serial I/O
 */
void testSerial() {
  uint8_t errCnt = 0;
  
  Serial.printf("sizeof(RemoteCmd)=%d enc=%d chk=%d\n", sizeof(RemoteCmd), cmdSizeEnc, cmdSizeChk);
  
  // Build test command
  inCmd.cmd = 'S';    // set output frequency
  inCmd.err = 0;
  inCmd.value = 5432; // 54.32 Hz in cHz
  dumpRemoteCmd(&inCmd, "TestCmd");
  
  // Test serialization
  char outputBuf[32];
  memset(outputBuf, 0, sizeof(outputBuf));
  
  int result = Base64.encode(outputBuf, (char*) &inCmd, sizeof(RemoteCmd));          // encode to base64
  Serial.printf("encode result=%d input=", result);
  Serial.printf(" output=%s\n", outputBuf);
  if (result != cmdSizeEnc) {
    Serial.printf("Error: encode1 length mismatch: expected %d got %d\n", cmdSizeEnc, result);
    errCnt += 1;
  }

  uint32_t crcVal = calcCRC32((uint8_t*)outputBuf, cmdSizeEnc);
  
  Serial.printf("crcVal=%d %08x\n", crcVal, crcVal);
  result = Base64.encode(outputBuf + cmdSizeEnc, (char*) &crcVal, sizeof(uint32_t));
  Serial.printf("result=%d enc+crc=%s\n", result, outputBuf);
  
  if (result != crcSizeEnc) {
    Serial.printf("Error: encode2 length mismatch: expected %d got %d\n", cmdSizeEnc, result);
    errCnt += 1;
  }

  if (strlen(outputBuf) != cmdSizeChk) {
    Serial.printf("Error: enc+crc length mismatch: expected %d got %d\n", cmdSizeChk, strlen(outputBuf));
    errCnt += 1;
  }

  // Test decoding
  uint32_t inCrc;
  result = Base64.decode((char*) &inCrc, outputBuf + cmdSizeEnc, crcSizeEnc);
  if (result != sizeof(inCrc)) {
    Serial.printf("Error: decode1 length mismatch: expected %d got %d\n", sizeof(inCrc), result);
    errCnt += 1;
  }
  if (inCrc != crcVal) {
    Serial.printf("Error: decoded CRC mismatch: expected %d got %d\n", crcVal, inCrc);
    errCnt += 1;
  }
  
  RemoteCmd xCmd;
  memset(&xCmd, 0, sizeof(RemoteCmd));

  result = Base64.decode((char*) &xCmd, outputBuf, cmdSizeEnc);
  if (sizeof(RemoteCmd) != result) {
    Serial.printf("Error: decode2 length mismatch: expected %d got %d\n", sizeof(RemoteCmd), result);
    errCnt += 1;
  }
  
  dumpRemoteCmd(&xCmd, "Result");

  if (xCmd.cmd != inCmd.cmd ||
      xCmd.err != inCmd.err ||
      xCmd.value != inCmd.value) {
    Serial.printf("Error: deserialized cmd mismatch\n");
    errCnt += 1;
  }

  Serial.printf("test %s  errCnt=%d\n", errCnt == 0 ? "passed" : "failed", errCnt);
}

/*
 * The loop function gets called repeatedly.
 * 
 * Once a second, dump the stats from the interrupt handler, and reset the counters.
 */
void loop() {
  unsigned long curMs = millis();
  int numBytes = Serial1.available();
  if (numBytes) {
    recvReqByte_Handler(numBytes);
  }
  if (curMs >= nextLoopMs) {
    nextLoopMs = nextLoopMs + intervalMs;
    numSeconds += 1;

    uint32_t irqNumRaw  = si5351IrqNumRaw;
    uint32_t irqElapsed = si5351IrqElapsed;
    if (sineClkSrc == 't' || sineClkSrc == 'T') {
      irqNumRaw  = tc2IrqNumRaw;
      irqElapsed = tc2IrqElapsed;
    }
    
    tc2IrqNumRaw = 0;
    tc2IrqElapsed = 0;
    si5351IrqNumRaw = 0;
    si5351IrqElapsed = 0;

    // print the average number of instructions executed by the timer interrupt handler
    // and the average time to execute the handler
    uint32_t dHours   = numSeconds / 3600;
    uint32_t dSeconds = numSeconds - dHours * 3600;
    uint32_t dMinutes = dSeconds / 60;
    dSeconds = dSeconds - dMinutes * 60;
    double avgElapsedInstr = (double) irqElapsed / (double) irqNumRaw;
    double avgElapsedUsec  = avgElapsedInstr * 1e6 / cpuClkFrequency;
    Serial.printf("In loop:  %d:%02d:%02d numIrq=%d  numErrs=%d  avgElapsed=",
                  dHours, dMinutes, dSeconds, irqNumRaw, tc2IrqFlagErrors);
    Serial.print(avgElapsedInstr,3);
    Serial.printf(" instr  ");
    Serial.print(avgElapsedUsec, 6);
    Serial.printf(" us\n");
    
    Serial1.printf("In loop:  %d:%02d:%02d numIrq=%d\n",
                  dHours, dMinutes, dSeconds, irqNumRaw);

  }
}
