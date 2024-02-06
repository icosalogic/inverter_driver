/*
 * Inverter driver app targeted specifically for the SAMD51 processor
 * models that have at least 2 DACs.  SAMD21 processors have only 1 DAC
 * and would thus be able to generate only 1 reference sine wave.
 * 
 * This app runs in the Arduino environment.
 * 
 * This code has been tested and verified on an Adafruit Feather M4 Express.
 * For other SAMD51 boards, some code changes may be necessary.
 * 
 * This app does 3 tasks:
 * done 1. Generates a clock signal that determines the timing of MOSFET
 *         switching in the inverter.  The clock timing ranges from 10s kHz
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
 * The output clock signal is generated using a TC instance with the 48 MHz
 * clock input with appropriate settings to get one of the following output
 * frequencies.  Since the output is toggled in each TC match, the counter
 * value should be 1/2 of the total number of clock pulses in each cycle.
 * Other frequencies are possible, these are just the most common.
 * 
 *        48 MHz
 *     /24  =   1 MHz
 *     /32  = 750 kHz
 *     /40  = 600 kHz
 *     /48  = 500 kHz
 *     /60  = 400 kHz
 *     /80  = 300 kHz
 *     /96  = 250 kHz
 *     /120 = 200 kHz
 *     /192 = 125 kHz
 *     /240 = 100 kHz
 *     /320 =  75 kHz
 *     /400 =  60 kHz
 *     /480 =  50 kHz
 *     /600 =  40 kHz
 * 
 * To generate the reference sine waves, we configure another 
 * TC instance to run at 240 kHz (for 50 Hz output) or 200 kHz (for
 * 60 Hz output).  We generate an interrupt on each tick, which is
 * used to update the DAC output value.  At startup time, generate a
 * 4000 entry sine wave table to be used for setting the DAC outputs.
 * One sine wave table works for multiple output reference waves, by
 * giving each output a different index into the table.  If we ever
 * want to generate 3-phase outputs, the sine wave table size should
 * be divisible by 3, but we don't consider it for now, since the
 * devices we use only have two DACs.
 * 
 * The dialog between this machine (the inverter driver) and the main
 * controller (the interface controller) will support the following
 * RPCs:
 *     1. Get/set L1 throttle (1..2047)
 *     2. Get/set L2 throttle (1..2047)
 *     3. Get/set neutral bias (+- 200)
 *     4. Get/set main on/off signal (0..1)
 *     5. Get/set L1 on/off signal (0..1)
 *     6. Get/set L2 on/off signal (0..1)
 * 
 * TODO: Devise a start up sequence that will prime the bootstrap
 * capacitor.  To do this, generate a low reference, and enable the
 * inverter for 5-10 clock cycles.  This should cause the phase leg
 * hardware to turn on the lower MOSFET Q2 long enough to charge up
 * the bootstrap capacitor, after which the inverter should be able
 * to start and run normally.
 */

int firstVariable = 4321;

#include <Arduino.h>
// #include <IcosaLogic_SAMD_Dumpster.h>          // this is not released yet

// clock definitions
const uint32_t kHz                      = 1000;
const uint32_t MHz                      = 1000000;

const double cpuClkFrequency            = 120 * MHz;
const uint32_t intrClockSourceFrequency = 48 * MHz;
const uint32_t clkClockSourceFrequency  = 48 * MHz;

// The output and PWM frequencies
const uint16_t outRmsVoltage = 120;
const double outputFreq = 60.25;       // 50 or 60, usually
const uint32_t pwmFreq = 500 * kHz;

// Output pins
uint32_t l1RefPin = DAC0;
uint32_t l2RefPin = DAC1;
uint32_t clockPin = A5;

// loop timing variables
unsigned long nextLoopMs = 0;
const unsigned long intervalMs = 1000;

// on/off signal values
uint8_t mainOnOff = 0;
uint8_t l1OnOff = 0;
uint8_t l2OnOff = 0;

// Since wave generation
const uint16_t numSamples = 4000;
const uint16_t numBits = 12;
const uint16_t maxDacValue = 1 << numBits;
const double midDacValue = maxDacValue / 2;
const uint16_t maxDacAmplitude = midDacValue - 1;
const uint16_t minDacAmplitude = 0 - maxDacAmplitude;
double rawSineData[numSamples];
uint16_t neutralBias = 0;

/*
 * Data for generating an output reference sine wave for a single channel.
 * Since each line may have a different load, it will have a different throttle
 * setting, so we need separate arrays for L1 and L2.
 */
typedef struct {
  uint16_t throttle;
  uint16_t index;
  uint16_t sineDataA[numSamples];
  uint16_t sineDataB[numSamples];
  uint16_t* sineData;
  bool usingDataA;
} LineData;

LineData l1;
LineData l2;
  
const uint16_t maxThrottle = 2047;
const uint16_t defaultThrottle = 1850;


volatile uint32_t tc5IrqNumRaw = 0;
volatile uint32_t tc5IrqElapsed = 0;
volatile uint8_t  tc5IrqFlagsLast = 0;
volatile uint32_t tc5IrqFlagErrors = 0;

// IcosaLogic_SAMD_Dumpster ilsd;

/*
 * Initialize the inverter.
 */
void setup() {
  setupSerial();

  dumpMemoryAddresses();
  dumpSamdPeripherals();

  setupReferenceWaves();
  setupClockOutput();

  dumpSamdPeripherals();

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
 * Adjust those real values to the integer range 0..4095, centered at 2048.
 */
void setupSineData() {
  uint32_t startTicks = DWT->CYCCNT;

  // Constants to save calculations in the loops
  const double pi = 3.14159265359;
  const double pi2 = 2.0 * pi;
  const double dNumSamples = numSamples;
  
  // initialize the line struct for L1 and L2
  l1.index = 0;
  l2.index = numSamples / 2;
  l1.throttle = defaultThrottle;
  l2.throttle = defaultThrottle;
  l1.usingDataA = 1;
  l2.usingDataA = 1;
  l1.sineData = l1.sineDataA;
  l2.sineData = l2.sineDataA;
  
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
  
  Serial.printf("usingDataA=%d  !usingDataA=%d\n", ld->usingDataA, !ld->usingDataA);
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
 * Set up the TC5 instance to generate an interrupt at the intervals
 * required for the configured output sine wave frequency.
 */
void setupSineInterrupt() {
  uint16_t period = (uint16_t) ((double) intrClockSourceFrequency / ((double) numSamples * outputFreq));
  
  // First, enable TC5 MCLK APBC interface
  if (! MCLK->APBCMASK.bit.TC5_) {
    MCLK->APBCMASK.bit.TC5_ = 1;
  }

  // Config the GCLK peripheral controller to use clock generator 1, and enable it
  GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

  Tc* TCx = TC5;                                         // Configure TC5

  TCx->COUNT16.CTRLA.bit.SWRST = 1;                      //reset
  while (TCx->COUNT16.SYNCBUSY.bit.SWRST);

  TCx->COUNT16.CTRLA.bit.ENABLE = 0;                     // Disable
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
  
  TCx->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;        // Set Timer counter Mode to 16 bits
  TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;          // Set normal frequency mode

  while (TCx->COUNT16.SYNCBUSY.bit.CC0);                 // Set the initial value
  TCx->COUNT16.CC[0].reg = period;
  while (TCx->COUNT16.SYNCBUSY.bit.CC0);

  NVIC_SetPriority(TC5_IRQn, 1);                         // Enable NVIC interrupts for TC5
  NVIC_EnableIRQ(TC5_IRQn);
  
  TCx->COUNT16.INTENSET.bit.OVF = 1;                     // Enable overflow interrupt

  TCx->COUNT16.CTRLA.bit.ENABLE = 1;                     // Enable TCx
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
}

/*
 * Handler for TC5 interrupts.
 */
void TC5_Handler() {
  uint32_t startTicks = DWT->CYCCNT;
  tc5IrqNumRaw += 1;
  
  uint8_t intFlags = TC5->COUNT16.INTFLAG.reg;
  tc5IrqFlagsLast = intFlags;
  
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
    tc5IrqFlagErrors += 1;
  }
  
  // clear all the pending interrupts
  TC5->COUNT16.INTFLAG.reg = tc5IrqFlagsLast;
  
  tc5IrqElapsed += DWT->CYCCNT - startTicks;
}


/*
 * Setup a TC instance for the clock output signal.
 * 
 * These values are for the Feather M4 Express:
 *     TCC0 IOSET 6
 *     TCC1 IOSET 1
 *     TC0 IOSET 1 -- PA04, PA05 pins A4, A1     bus APBA
 *     TC1 IOSET 1 -- PA06, PA07 pins A5, xx     bus APBA
 *     TC2 IOSET 2 -- PA12, PA13 pins SDA, SCL   bus APBB
 *     TC3 IOSET 1 -- PA14, PA15 pins 4, xx      bus APBB
 *     TC4 IOSET 1 -- PB08, PB09 pins A2, A3     bus APBC
 *     TC5                                       bus APBC
 */
void setupClockOutput() {
  uint16_t period = clkClockSourceFrequency / (pwmFreq * 2);
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

  Tc* TCx = TC1;	                               // Configure TC

  TCx->COUNT16.CTRLA.bit.SWRST = 1;                    //reset
  while (TCx->COUNT16.SYNCBUSY.bit.SWRST);

  TCx->COUNT16.CTRLA.bit.ENABLE = 0;                   // Disable
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
  
  TCx->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;      // Set Timer counter Mode to 16 bits
  TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;        // Set match frequency mode

  while (TCx->COUNT16.SYNCBUSY.bit.CC0);	       // Set the period
  TCx->COUNT16.CC[0].reg = period;
  while (TCx->COUNT16.SYNCBUSY.bit.CC0);

  TCx->COUNT16.CTRLA.bit.ENABLE = 1;	               // Enable TCx
  while (TCx->COUNT16.SYNCBUSY.bit.ENABLE);
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
  // This is not released yet.
  // ilsd.dumpGCLK(NULL);
  // ilsd.dumpTC();
}

/*
 * The loop function gets called repeatedly.
 * 
 * Once a second, dump the stats from the interrupt handler, and reset the counters.
 */
void loop() {
  unsigned long curMs = millis();
  if (curMs >= nextLoopMs) {
    nextLoopMs = curMs + intervalMs;

    // 120M instr / sec
    double avgElapsedInstr = (double) tc5IrqElapsed / (double) tc5IrqNumRaw;
    double avgElapsedUsec  = avgElapsedInstr * 1e6 / cpuClkFrequency;
    Serial.printf("In loop:  numIrq=%d  numErrs=%d  avgElapsed=", tc5IrqNumRaw, tc5IrqFlagErrors);
    Serial.print(avgElapsedInstr,3);
    Serial.printf(" instr  ");
    Serial.print(avgElapsedUsec, 6);
    Serial.printf(" us\n");
    tc5IrqElapsed = 0;
    tc5IrqNumRaw = 0;

    // delay until the next interval to reduce CPU utilization
    /* */
    unsigned long msDelay = nextLoopMs - millis();
    if (msDelay > 1000) {
      msDelay = 1000;
    }
    delay(msDelay);
    /* */
  }
}
