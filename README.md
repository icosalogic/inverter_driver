# 1.0 Inverter Driver

This app generates reference sine waves and the MOSFET switching clock for a split-phase inverter.
This implementation is targeted specifically to SAMD51 boards, and is rather hardware specific.

# 2.0 Reference Sine Waves

The SAMD51 processors have two 12-bit DACs (Digital to Analog Converters) that are used to generate
the reference sine waves.
This is used to implement a split-phase inverter, where there are two sine waves 180 degrees out-of-phase.
Each of these output sine waves is called a line.

## 2.1 Sine Wave Amplitude
The 12-bit DACs can generate analog output signals corresponding to input values in the range
from 0 to 4095.
At system startup, we generate a 4000-entry table of raw sine value for a complete sine wave over
360 degrees or 2 pi radians.
The double precision floating point value for each sine value is in the range -1 to +1.

Each line has two 4000 entry arrays of uint16_t (unsigned 16-bit integers).
The 180 degree phase difference between the two output lines is created by initializing
line 1 index into the array with the value 0, and the line 2 index with 2000.

The values in these integer arrays is computed as follows:
1. By multiplying the raw sine value by a throttle value ranging from 1..2047
2. Adding 2048 (to center the result at 2048, the midpoint of the12-bit DAC output range)
3. Adding a small bias value (usually within -200 to +200) to shift the entire sine wave up or down
4. Converting the result to uint16_t.

The throttle is used to adjust the output voltage of each line, and to compensate for minor
hardware tolerances that may cause the output voltage to not match the desired value.

The bias can adjust the center of the output sine wave up or down, also to compensate for
potential hardware tolerances, and to provide a coarse mechanism to balance the top and
bottom halves of the battery

There are two arrays for each line, A and B.  If a line is using one of the arrays, and
the bias or throttle changes, which requires new array values to be computed, the new
values are computed into the other array.
Once all the values are recomputed, the line will be pointed to the other array.

## 2.2 Sine Wave Frequency

The sine wave amplitude is modified by adjusting the DAC output value in an interrupt handler.
The driver application arranges for interrupts to be generated at the frequency K = F * 4000,
where F is the output frequency.
Common inverter output frequencies are 50 and 60 Hz.
The inverter driver will support output frequencies from 40 to 70 Hz, adjustable in 0.01 Hz
increments.
For example, if the desired output frequency is 51.5 Hz, we generate interrupts at 206 kHz.

The interrupts driving the sine wave generation can be created in two ways:
1. Using a Si5351 clock generator chip
2. Using a SAMD51 TC instance.

If you want precise control over your output frequency, we strongly recommend using the
Si5351 clock chip.
The parts are inexpensive if you want to build your own board, or you can buy an inexpensive
breakout board from Adafruit.
Or you can buy an unbelievably cheap breakout board from AliExpress
that appears to be created from Adafruit's open source board design.
Our testing was done with the Adafruit Si5351 breakout board.
We use the Etherkit_Si5351 Arduino library.
There appears to be no functionally complete Arduino library for this chip, as none
of the libraries implement the spread spectrum functionality.

Note that we configure the sine wave clock to use Si5351 Clock 2, which is set to use PLLB.
PLLA is used for the MOSFET switching clock (see below), and it supports spread spectrum,
PLLB does not.
Assuming spread spectrum is ever enabled for the switching clock, we would not want it to affect
the reference sine waves.

Alternately, the driver can generate the sine wave interrupts using a SAMD51 TC instance doing
a divide of the 48 MHz clock.
The TC input clock frequency is 48 MHz, so to generate a 60 Hz sine wave the period
configured for the TC divisor is 48000000 / (4000 * 60) = 200.
The problem here is that the sine wave frequencies generated using the TC clock are accurate
to about 2 digits, while the Si5351 is accurate to approximately 4 digits.
This may not matter much in real world environments, but it is worth mentioning.

# 3.0 MOSFET Switching Clock

The clock used for MOSFET switching can be created in two ways:
1. Using a Si5351 clock generator chip
2. Using a SAMD51 TC instance.

Using the Si5351 offers benefits of more precise control, and the eventual option of
enabling spread spectrum, which may be important for switching large MOSFETs at frequencies
that could interfere with radio broadcasts.

Having said that, precise control of the switching frequencies is not critical, and for
lower switching speeds, the SAMD51 TC instance is both flexible and sufficiently accurate.

# 4.0 Communications with UI Controller

Precise control of the inverter is critical, so this inverter driver is not burdened
with providing a UI to the end user, nor monitoring slowly changing values like
temperatures, etc.
Those functions are provided by a separate UI controller, and this section documents
the messages exchanged between these two machines that allow them to coordinate their activities.

The messages exchanged between the controllers is described with this structure:

`
typedef struct {
  uint8_t cmd;
  uint8_t err;
  int16_t value;
} RemoteCmd;
`

Request messages are sent by the UI controller to the inverter driver, and response
messages are sent by the inverter driver back to the UI controller.

The cmd field describes what operation is being performed.
All commands have get and set variants; the get variant alloows the UI controller to
read settings from the inverter driver, and the set variant allows the UI controller
to change those settings.

The err field enables the driver controller to indicate there was an anomaly attempting
to process the sent command.

The value field is set on the response to a get command, and on a set command.

In the table below, the lower case command letter is always a get command, and the
upper case letter is used for a set command.

1. aA Get/set L1 throttle (1..2047)
2. bB Get/set L2 throttle (1..2047)
3. cC Get/set L1 on/off signal (0..1)
4. dD Get/set L2 on/off signal (0..1)
5. nN Get/set neutral bias (+- 200)
6. oO Get/set main on/off signal (0..1)
7. fF Get/set output frequency (4000..7000 cHz) (40..70 Hz)
8. sS Get/set switching frequency (1..2000) kHz
9. zZ Get/set switching frequency dither (-25..15)
        (-25..-1) are for down spread of -2.5% to -0.1%
        (1..15) are for center spread of +/- 0.1% to +/- 1.5%
10. gG Get/set switching clock src; s or S for Si5351, t or T for TC
11. hH Get/set sine clock src; s or S for Si5351, t or T for TC
