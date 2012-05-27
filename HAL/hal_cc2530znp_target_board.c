/**
* @file hal_cc2530znp_target_board.c
*
* @brief Hardware Abstraction Layer (HAL) for the cc2530znp target board
*
* This file must be modified if changing hardware platforms.
*
* The ZNP library & examples require the following methods to be implemented. 
* Refer to the individual method descriptions for more information.
* - halInit()
* - putchar()
* - halSpiInitZnp()
* - delayMs()
* - toggleLed()
*
* Also see hal_cc2530znp_target_board.h for macros that must be defined.
*
* @see hal_helper.c for utilities to assist when changing hardware platforms
*
* @see http://processors.wiki.ti.com/index.php/Tutorial_on_the_Examples and http://e2e.ti.com/support/low_power_rf/default.aspx
*
* $Rev: 658 $
* $Author: dsmith $
* $Date: 2010-07-13 10:23:50 -0700 (Tue, 13 Jul 2010) $
*
* YOU ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY 
* OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, 
* TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL TEXAS INSTRUMENTS 
* OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, 
* BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
* INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
* LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY 
* CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*/

#include "hal_cc2530znp_target_board.h"
#include "hal_Osram_SFH5711_ambient_light_sensor.c"

#define HARDWARE_UART

/** This is a function pointer for the Interrupt Service Routine called when a debug console character is received.
To use it, declare it with
<code> extern void (*debugConsoleIsr)(char);  </code> 
and then point it to a function you created, e.g.
<code> debugConsoleIsr = &handleDebugConsoleInterrupt;  </code>
and your function handleDebugConsoleInterrupt() will be called when a byte is received.
*/
void (*debugConsoleIsr)(char);

/** Function pointer for the ISR called when the button is pressed */
void (*buttonIsr)(void);

/** Function pointer for the ISR called when a timer generates an interrupt*/
void (*timerIsr)(void);

/** Function pointer for the ISR called when the accelerometer generates an interrupt*/
void (*accelerometerIsr)(void);

/** Function pointer for the ISR called when a SRDY interrupt occurs*/
void (*srdyIsr)(void);

/** Flags to indicate when to wake up the processor. These are read in the various interrupt service routines 
and if the flag is set then the processor will be woken up with HAL_WAKEUP() at the end of the ISR. 
This is required because HAL_WAKEUP() cannot be called anywhere except in an ISR. */
unsigned int wakeupFlags = 0;

/** The post-calibrated frequency of the Very Low Oscillator (VLO). Set with calibrateVlo() and read by initTimer(). */
unsigned int vloFrequency = 0;

/** Debug console interrupt service routine */
#pragma vector = USCIAB0RX_VECTOR   //0xFFEE
__interrupt void USCIAB0RX_ISR(void)
{
    if (IFG2 & UCA0RXIFG)  //debug console character received
    {
        debugConsoleIsr(UCA0RXBUF);    //reading this register clears the interrupt flag
    } 
}

/** Port 1 interrupt service routine. */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if (P1IFG & BIT2)                
        buttonIsr();

    if (P1IFG & BIT3)
    {
        accelerometerIsr();
        if (wakeupFlags & WAKEUP_AFTER_ACCELEROMETER)    
            HAL_WAKEUP();           
    }
    P1IFG = 0;                          // clear the interrupt
}

/** Port 2 interrupt service routine. */
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if (P2IFG & BIT6)
    {
        srdyIsr();
        if (wakeupFlags & WAKEUP_AFTER_SRDY)    
            HAL_WAKEUP();          
    }
    P2IFG = 0;                          // clear the interrupt
}

/** 
Configures hardware for the particular hardware platform:
- Ports: sets direction, interrupts, pullup/pulldown resistors etc. 
- Oscillator: turns off WDT, configures MCLK = 8MHz using internal DCO & sets SMCLK = 4MHz
- Holds radio in reset (active-low)
*/
void halInit()
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    
    //
    //    Initialize Oscillator
    //
    if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)                                     
    {  
        while(1); // Stop if calibration constants erased
    }   
    BCSCTL1 = CALBC1_8MHZ; // Set DCO = 8MHz for MCLK
    DCOCTL = CALDCO_8MHZ;
    BCSCTL2 |= DIVS_1;     //SMCLK = DCO/2 (4MHz) 
    BCSCTL3 |= LFXT1S_2;                   // Use VLO for ACLK
    //
    //    Initialize Ports
    //
    /*PORT 1:
    1.0     LED 1
    1.1     LED 2
    1.2     Button - connects to GND when closed
    1.3     Accelerometer Interrupt
    1.4     
    1.5     
    1.6     
    1.7     
    */
    P1DIR = BIT0+BIT1; 
    P1OUT &= ~BIT2;
    P1IE  = BIT2;   //Enable Button Interrupt
    P1IES = BIT2;   //Interrupt on high-to-low transition
    P1REN = BIT2;   //enable resistor for button
    P1OUT = BIT2;   //make resistor a PULL-UP
    P1SEL = 0;      //NOTE: default value is NOT 0!
    P1IFG = 0;
    /*PORT 2
    2.0     
    2.1     Light Sensor power
    2.2     TEST
    2.3     
    2.4     
    2.5     
    2.6     SRDY
    2.7     Accelerometer SS
    */
    P2DIR = BIT1+BIT2+BIT7;
    P2SEL = 0;
    
    /*PORT 3
    3.0     ZNP CS
    3.1     SPI MOSI
    3.2     SPI MISO
    3.3     SPI SCLK
    3.4     Debug UART Tx
    3.5     Debug UART Rx
    3.6     MRDY
    3.7     ZNP Reset
    */
    P3DIR = BIT0+BIT6+BIT7;
    P3SEL = (BIT1+BIT2+BIT3+BIT4+BIT5);  //p3.1,2,3=USCI_B0; P3.4,5 = USCI_A0 TXD/RXD;
    P3OUT &= ~BIT7;                      //turn off radio
    
    /*PORT 4
    4.0     CFG 0
    4.1     CFG 1
    4.2     CC2530 programming DC
    4.3     CC2530 programming DD
    4.4     
    4.5     
    4.6     
    4.7     
    */
    P4DIR = BIT0+BIT1 + BIT4;  //BIT4 = DEBUGGING
    P4SEL = 0;
    P4OUT &= ~BIT4;
    //
    //    Initialize UART debug console:
    //
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 26; UCA0BR1 = 0;                // 4mHz smclk w/modulation for 9,600bps, table 15-5 
    UCA0MCTL = UCBRS_0 + +UCBRF_1 + UCOS16;   // Modulation UCBRSx=1, over sampling      
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
    
    //
    //   Deselect SPI peripherals:
    //
    SPI_SS_CLEAR();                       // Deselect ZNP
    ACCELEROMETER_SS_CLEAR();             // Deselect acceleration sensor 
    
    //  Stop Timer A:
    TACTL = MC_0; 
    
    clearLeds();
}



/** Configures SRDY interrupt. Does NOT enable the interrupt.
On the CC2530 target board, SRDY is P2.6
@param flags the options for the SRDY interrupt: 
- SRDY_INTERRUPT_FALLING_EDGE or SRDY_INTERRUPT_RISING_EDGE
- NO_WAKEUP or WAKEUP_AFTER_SRDY

@return 0 if success, -1 if bad flag
*/
signed int halConfigureSrdyInterrrupt(unsigned char flags)
{
    if (flags & (~(SRDY_INTERRUPT_FALLING_EDGE | WAKEUP_AFTER_SRDY)))   //if any other bit is set in flags
        return -1;                                                      //then error
    
    if (flags & SRDY_INTERRUPT_FALLING_EDGE)
        P2IES |= BIT6;      //Interrupt on high-to-low transition
    else
        P2IES &= ~BIT6;     //Interrupt on low-to-high transition        
    
    if (flags & WAKEUP_AFTER_SRDY)
        wakeupFlags |= WAKEUP_AFTER_SRDY;
    else
        wakeupFlags &= ~WAKEUP_AFTER_SRDY;
    return 0;
}

/** Send one byte via hardware UART. Required for printf() etc. in stdio.h */
int putchar(int c)
{	
    while (!(IFG2 & UCA0TXIFG));   // Wait for ready
    UCA0TXBUF = (unsigned char) (c & 0xFF); 
    return c;
}

/**
Initializes the Serial Peripheral Interface (SPI) interface to the Zigbee Network Processor (ZNP).
@note Maximum CC2530ZNP SPI clock speed is 4MHz. SPI port configured for clock polarity of 0, clock phase of 0, and MSB first.
@note On the CC2530ZNP target board the MSP430 uses USCIB0 SPI port to communicate with the ZNP
@pre SPI pins configured correctly: Clock, MOSI, MISO configured as SPI function; Chip Select configured as an output; SRDY configured as an input.
@post SPI port is configured for ZNP communications.
*/
void halSpiInitZnp()
{
    UCB0CTL1 |= UCSSEL_2 | UCSWRST;                 //serial clock source = SMCLK, hold SPI interface in reset
    UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;     //clock polarity = inactive is LOW (CPOL=0); Clock Phase = 0; MSB first; Master Mode; Synchronous Mode    
    UCB0BR0 = 2;  UCB0BR1 = 0;                      //SPI running at 2MHz (SMCLK / 2)
    UCB0CTL1 &= ~UCSWRST;                           //start USCI_B1 state machine  
}

/**
Sends a message over SPI to the ZNP. Based on hal_board.c in CC2480 example application ZASA.
The ZNP uses a "write-to-read" approach: to read data out, you must write data in.
This is a private method that gets wrapped by other methods, e.g. spiSreq(), spiAreq, etc.
To Write, set *bytes and numBytes. To Read, set *bytes only. Don't need to set numBytes because CC2530ZNP will stop when no more bytes are received.
@param bytes the data to be sent or received.
@param numBytes the number of bytes to be sent. This same buffer will be overwritten with the received data.
@pre SPI port configured for the ZNP
@pre CC2530ZNP has been initialized
@post bytes contains received data, if any
*/
void spiWrite(unsigned char *bytes, unsigned char numBytes)
{
    while (numBytes--)
    {  
        UCB0TXBUF = *bytes;
        while (!(IFG2 & UCB0RXIFG)) ;     //WAIT for a character to be received, if any
        *bytes++ = UCB0RXBUF;             //read bytes
    }
}

/**
* Blocking Delay in Milliseconds
* delays by at least the specified number of milliseconds (ms)
* @pre TICKS_PER_MS set
* @param ms number of milliseconds to delay
*/
void delayMs(unsigned int ms)    
{
    unsigned int a, b;
    for (a = ms; a > 0; a--)         // outer loop takes 5 ck per round  
        for (b = TICKS_PER_MS; b > 0; b--)  // inner loop takes 5 ck per round 
            asm("nop"); 
}

/** Blocking Delay in Microseconds
delays by at least the specified number of microseconds (us)
@pre TICKS_PER_US set
@param us number of microseconds to delay
*/
void delayUs(unsigned int us)
{
    unsigned int a; 
    us *= TICKS_PER_US; 
    for (a = us; a > 0; a--)  // loop takes 5 ck per round 
        asm("nop"); 
}

/** Turns ON the specified LED. 
@param led the LED to turn on, must be 0 or 1.
@post The specified LED is turned on. 
@return 0 if success, -1 if invalid LED specified
*/
signed int setLed(unsigned char led)
{
    if (led > 1) 
        return -1;
    P1OUT |= (led) ? BIT1 : BIT0;
    return 0;
}

/** Turns OFF LEDs. 
@post LEDs are turned off. 
*/
void clearLeds()
{
    P1OUT &= ~(BIT0 + BIT1);
}

/** Toggles the specified LED. 
@param led the LED to toggle, must be 0 or 1.
@post The specified LED is toggled. 
@return 0 if success, -1 if invalid LED specified
*/
signed int toggleLed(unsigned char led)
{
    if (led > 1) 
        return -1;
    P1OUT ^= (1 << led);
    return 0;
}

//
//
//          CC2530ZNP TARGET BOARD PERIPHERALS
//          NOT REQUIRED FOR ZNP OPERATION
//
//


/** Reads the MSP430 supply voltage using the Analog to Digital Converter (ADC). 
On CC2530ZNP target board, this is approx. 3600mV
@return Vcc supply voltage, in millivolts
*/
unsigned int getVcc3()
{
    ADC10CTL0 = SREF_1 + REFON + REF2_5V + ADC10ON + ADC10SHT_3;  // use internal ref, turn on 2.5V ref, set samp time = 64 cycles
    ADC10CTL1 = INCH_11;                         
    delayMs(1);                                     // Allow internal reference to stabilize
    ADC10CTL0 |= ENC + ADC10SC;                     // Enable conversions
    while (!(ADC10CTL0 & ADC10IFG));                // Conversion done?
    unsigned long temp = (ADC10MEM * 5000l);        // Convert raw ADC value to millivolts
    return ((unsigned int) (temp / 1024l));
}

/** Reads the light sensor.
Turns on Light sensor, configures ADC, reads the value, and converts it to lux.
@note Light sensor outputs current between 0 to 50uA. On the target board, load resistor = 47k Ohms.
@note 0 to 100,000lx will generate 0 to 2350mV input into our ADC.
@note Light sensor analog input is on P2.0, or analog input A0.
@note This light sensor is binned for a sensitivity of 27 to 30 uA @ 1000lux = 1269-1410 mV = ADC readings of 520-578counts 
@return Light Sensor reading in lux/10.
*/
unsigned int getLightSensor()
{
    LIGHT_SENSOR_ON();
    ADC10AE0 = 0x01;
    ADC10CTL0 = SREF_1 + REFON + REF2_5V + ADC10ON + ADC10SHT_3;  // use internal ref, turn on 2.5V ref, set samp time = 64 cycles
    ADC10CTL1 = INCH_0;                             // Analog input A0
    delayMs(1);                                     // Allow internal reference to stabilize, also light sensor needs time to stabilize
    ADC10CTL0 |= ENC + ADC10SC;                     // Enable conversions
    while (!(ADC10CTL0 & ADC10IFG));                // Conversion done?
    LIGHT_SENSOR_OFF();
    return (convertAdcToLux(ADC10MEM));
}

//
//      ONLY REQUIRED FOR TARGET BOARD
//
/** Initializes the SPI port for the Accelerometer.
Required because accelerometer SPI port uses different settings than the ZNP SPI port
*/
void halSpiInitAccelerometer()
{
    UCB0CTL1 |= UCSSEL_2 | UCSWRST;                 //serial clock source = SMCLK, hold SPI interface in reset. NOTE: SMCLK is 4MHz
    UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;     //clock polarity = inactive is LOW (CPOL=0); Clock Phase = 0; MSB first; Master Mode; Synchronous Mode    
    UCB0BR0 = 16;  UCB0BR1 = 0;                     //SPI running at 250kHz (SMCLK / 16)
    UCB0CTL1 &= ~UCSWRST;                           //start USCI_B1 state machine 
    wakeupFlags |= WAKEUP_AFTER_ACCELEROMETER;
}

/** Enables the interrupt on the accelerometer (P1.3). 
@param wakeOnAccelerometer whether to wakeup after this int is triggered. Must be either NO_WAKEUP or WAKEUP_AFTER_ACCELEROMETER.
@return 0 if success; -1 if invalid wakeOnAccelerometer parameter
*/
signed int halEnableAccelerometerInterrupt(unsigned char wakeOnAccelerometer)
{
    if ((wakeOnAccelerometer != NO_WAKEUP) && (wakeOnAccelerometer != WAKEUP_AFTER_ACCELEROMETER))
        return -1;       
    P1IE |= BIT3;
    P1IES &= ~BIT3;  //Generate interrupt on Low to High Edge
    P1IFG = 0;
    return 0;
}

/* Simple utility method used in accelerometer methods. 
@param b the byte to write
*/
unsigned char spiWriteAccelerometer(unsigned char b)
{
    UCB0TXBUF = b;                            // Write address to TX buffer 
    while (!(IFG2 & UCB0RXIFG)) ;             // Wait until new data was written into RX buffer 
    return(UCB0RXBUF);                        // Read RX buffer just to clear interrupt flag      
}

/** Configures all ZNP interface signals as inputs to allow the ZNP to be programmed.
Toggles LED0 quickly to indicate application is running. 
*/
void setZnpInterfaceToInputs(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)                                     
    {  
        while(1); // Stop if calibration constants erased
    }   
    BCSCTL1 = CALBC1_8MHZ; // Set DCO = 8MHz for MCLK
    DCOCTL = CALDCO_8MHZ;
    BCSCTL2 |= DIVS_1;     //SMCLK = DCO/2 (4MHz)    
    
    P1DIR = 1; P1REN = 0; P1IE = 0; P1SEL = 0; 
    P2DIR = 0; P2REN = 0; P2IE = 0; P2SEL = 0; 
    P3DIR = 0;
    P4DIR = 0; P4REN = 0; P4SEL = 0;   
    for (;;)
    {
        toggleLed(0);
        delayMs(100);   
    }
}

#define TIMER_MAX_SECONDS 4
/** Configures timer.
@pre ACLK sourced from VLO
@pre VLO has been calibrated; number of VLO counts in one second is in vloFrequency.
@param seconds period of the timer. Maximum is 0xFFFF / vloFrequency; or about 4 since VLO varies between 9kHz - 15kHz. 
Use a prescaler on timer (e.g. set IDx bits in TACTL register) for longer times. 
Maximum prescaling of Timer A is divide by 8. Even longer times can be obtained by prescaling ACLK if this doesn't affect other system peripherals.
@param wakeOnTimer whether to wake the processor after the timer interrupt. Must be either NO_WAKEUP or WAKEUP_AFTER_TIMER.
@return 0 if success; -1 if illegal parameter or -2 if VLO not calibrated
*/
signed int initTimer(unsigned char seconds, unsigned char wakeOnTimer)
{  
    if (seconds > TIMER_MAX_SECONDS)
        return -1;
    if (vloFrequency == 0)
        return -2;
    if ((wakeOnTimer != NO_WAKEUP) && (wakeOnTimer != WAKEUP_AFTER_TIMER))
        return -3;
    wakeupFlags |= WAKEUP_AFTER_TIMER;
    CCTL0 = CCIE;                             // CCR0 interrupt enabled
    CCR0 = vloFrequency * (seconds);                      // generate int
    TACTL = TASSEL_1 + MC_1;           // ACLK, upmode
    return 0;
}

void stopTimer()
{
    TACTL = MC_0; 
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
    timerIsr();
    if (wakeupFlags & WAKEUP_AFTER_TIMER)    
    {
        HAL_WAKEUP();     
    }
}

/** Calibrate VLO. Once this is done, the VLO can be used semi-accurately for timers etc. 
Once calibrated, VLO is within ~2% of actual when using a 1% calibrated DCO frequency and temperature and supply voltage remain unchanged.
@return VLO frequency (number of VLO counts in 1sec), or -1 if out of range
@pre SMCLK is 4MHz
@pre MCLK is 8MHz
@pre ACLK sourced by VLO (BCSCTL3 = LFXT1S_2; in MSP430F2xxx)
@note Calibration is only as good as MCLK source. Obviously, if using the internal DCO (+/- 1%) then this value will only be as good as +/- 1%. YMMV.
@note On MSP430F248 or MSP430F22x2 or MSP430F22x4, must use TACCR2. On MSP430F20x2, must use TACCR0.
Check device-specific datasheet to see which module block has ACLK as a compare input.
For example, see page 23 of the MSP430F24x datasheet or page 17 of the MSP430F20x2 datasheet, or page 18 of the MSP430F22x4 datasheet.
@note If application will require accuracy over change in temperature or supply voltage, recommend calibrating VLO more often.
@post Timer A settings changed
@post ACLK divide by 8 bit cleared
*/

signed int calibrateVlo()
{
    WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
    delayMs(1000);
    
    BCSCTL1 |= DIVA_3;                    // Divide ACLK by 8
    TACCTL2 = CM_1 + CCIS_1 + CAP;        // Capture on ACLK
    TACTL = TASSEL_2 + MC_2 + TACLR;      // Start TA, SMCLK(DCO), Continuous
    while ((TACCTL0 & CCIFG) == 0);       // Wait until capture
    
    TACCR2 = 0;                           // Ignore first capture
    TACCTL2 &= ~CCIFG;                    // Clear CCIFG
    
    while ((TACCTL2 & CCIFG) == 0);       // Wait for next capture
    unsigned int firstCapture = TACCR2;   // Save first capture
    TACCTL2 &= ~CCIFG;                    // Clear CCIFG
    
    while ((TACCTL2 & CCIFG) ==0);        // Wait for next capture
    
    unsigned long counts = (TACCR2 - firstCapture);        // # of VLO clocks in 8Mhz
    BCSCTL1 &= ~DIVA_3;                   // Clear ACLK/8 settings
    
    vloFrequency = ((unsigned int) (32000000l / counts));
    if ((vloFrequency > VLO_MIN) && (vloFrequency < VLO_MAX))
        return vloFrequency;
    else
        return -1;
}
