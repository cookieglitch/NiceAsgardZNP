/**
* @file hal_mdb1.c
*
* @brief Hardware Abstraction Layer (HAL) for the Smith Electronics Module Development Board (MDB1)
*
* This development board has an MSP430F248 and CC2530ZNP and was the initial development platform for these ZNP Examples.
* This file is included to show how easy it is to port the ZNP libraries to a different hardware platform.
*
* Peripherals:
* - ZNP Interface: Connects to the ZNP via jumper-selectable SPI or UART 
* - LEDs: Six general purpose LEDs, one bi-color (Red/Yellow/Green) Status LED, and one power LED
* - Buttons: Five general purpose buttons, one reset button
* - Interfaces: MSP430 & CC2530 programming connectors, RS-232 on DB-9 
* - Crystals: External 32kHz and 16MHz crystal options
* - I2C Serial EEPROM
* - Other I/O: Two Opto-Isolated inputs, Four relay drivers, I/O pins on headers
* - Configurability: All on-board peripherals may be disabled with cuttable jumpers
* See www.smith-electronics.com for more information.
*
* $Rev: 709 $
* $Author: dsmith $
* $Date: 2010-08-12 19:00:00 -0700 (Thu, 12 Aug 2010) $
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
#include "hal_mdb1.h"

/** This is a function pointer for the Interrupt Service Routine called when a debug console character is received.
To use it, declare it with
<code> extern void (*debugConsoleIsr)(char);  </code> 
and then point it to a function you created, e.g.
<code> debugConsoleIsr = &handleDebugConsoleInterrupt;  </code>
and your function handleDebugConsoleInterrupt() will be called when a byte is received. */
void (*debugConsoleIsr)(char);

/** Function pointer for the ISR called when a button is pressed.*/
void (*buttonIsr)(char);

/** Function pointer for the ISR called when a timer generates an interrupt*/
void (*timerIsr)(void);

/** Function pointer for the ISR called when a byte is received on the aux. serial port */
void (*auxSerialPort)(char);

/** Function pointer for the ISR called when a byte is received on the bit-bang serial port. 
Param is which bit-bang interface (0 or 1) */
void (*bitBangSerialIsr)(char);

signed int oscInit(unsigned char mainOscConfiguration, unsigned char auxOscConfiguration);

unsigned long mclk, smclk;

/** Debug console interrupt service routine, called when a byte is received on USCIA0 or USCIB0. */
#pragma vector = USCIAB0RX_VECTOR   //0xFFEE
__interrupt void USCIAB0RX_ISR(void)
{
    if (IFG2 & UCA0RXIFG)
    {
        debugConsoleIsr(UCA0RXBUF);    //reading this register clears the interrupt flag
    } 
    if (UCB0STAT & UCNACKIFG)             //If I2C bus received a NACK...
    {                                     
        UCB0CTL1 |= UCTXSTP;                // ... then send a STOP
        UCB0STAT &= ~UCNACKIFG;             // and clear the status bit.
    }
}


/** Auxilliary Serial Port interrupt service routine, called when a byte is received on USCIA1 or USCIB1. */
#pragma vector=USCIAB1RX_VECTOR
__interrupt void USCI1RX_ISR(void)
{
    if (UC1IFG & UCA1RXIFG)
    {
        auxSerialPort(UCA1RXBUF);    //reading this register clears the interrupt flag                  
    }
}

/** Reads the current status of the buttons
@return 0x01 if button 1 is pressed, 0x02 if button 2 is pressed, 0x04 if button 3 is pressed, etc.
*/
unsigned char getButtons()
{
    return ((~P2IN) & (BIT0+BIT1+BIT2+BIT3+BIT4)); //only return buttons
}

/** Port P2 interrupt service routine. 
@pre Port 2 pins are configured as interrupts appropriately.*/
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if (P2IFG & BIT0) buttonIsr(0); 
    if (P2IFG & BIT1) buttonIsr(1); 
    if (P2IFG & BIT2) buttonIsr(2);
    if (P2IFG & BIT3) buttonIsr(3);
    if (P2IFG & BIT4) buttonIsr(4);
    //if (P2IFG & BIT6) bitBangSerialIsr(0);  //character received on bit-bang serial interface 0
    P2IFG = 0;  // clear all P2 interrupts
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if (P1IFG & BIT4) bitBangSerialIsr(0);  //character received on bit-bang serial interface 0
    P1IFG = 0;  // clear all P2 interrupts
}

/** 
* Configures hardware for the particular board
* - Oscillator: turns off WDT, configures MCLK & SMCLK
* - Ports: including purpose, direction, pullup/pulldown resistors etc. 
* - Holds radio in reset (active-low)
*/
void halInit()
{
    oscInit(MCLK_8_DCO, ACLK_VLO);
    
    /*PORT 1:
    1.0   CTS
    1.1   RTS, input, no interrupt
    1.2   Radio Reset
    1.3   SRDY
    1.4   opto in1 (note: inverted by opto-isolator)
    1.5   opto in2 (note: inverted by opto-isolator)
    1.6   Radio Debug Data (not used)
    1.7   Radio Debug Clock (not used)
    */
    P1IES = 0;
    P1DIR = BIT2; 
    P1OUT &= ~BIT2;
    
    /*PORT 2
    2.0     Button 0
    2.1     Button 1
    2.2     Button 2
    2.3     Button 3
    2.4     Button 4
    2.5     RS-232 Bit-Bang UART Receive (not used, leave as input)
    2.6     DIP Switch 0 - used for expander bit-bang input0
    2.7     DIP Switch 1 - used for expander bit-bang input1
    */
    P2DIR = 0x00;                                   // All inputs
    P2IE  = BIT0+BIT1+BIT2+BIT3+BIT4;               // Port 2 interrupts on pushbuttons
    P2REN = BIT0+BIT1+BIT2+BIT3+BIT4;   // NOT on UART receive or bit-bang input0,1
    P2OUT = BIT0+BIT1+BIT2+BIT3+BIT4;   // Port 2 resistors = pull-UP   (0=pull-down, 1=pull-up) 
    P2IFG = 0;                                      // clear all P2 interrupts
    
    /*PORT 3
    3.0
    3.1     I2C Data
    3.2     I2C Clock
    3.3
    3.4     Debug UART Tx
    3.5     Debug UART Rx
    3.6     Second (ZNP/Aux) UART Tx
    3.7     Second (ZNP/Aux) UART Rx
    */
    P3SEL = BIT1+BIT2+BIT4+BIT5+BIT6+BIT7;  //p3.1,2=USCI_B0 i2C P3.4,5 = USCI_A0 TXD/RXD; 
    
    /*PORT 4
    4.0     LED 0
    4.1     LED 1
    4.2     LED 2
    4.3     LED 3
    4.4     LED 4
    4.5     LED 5
    4.6     Status LED RED
    4.7     Status LED GREEN
    */
    P4DIR = 0xFF;                             // Set P4.0 to output direction for LED control
    
    /*PORT 5
    5.0     SPI CS
    5.1     SPI MO
    5.2     SPI MI
    5.3     SPI SCLK
    5.4     Relay Drive 0
    5.5     Relay Drive 1
    5.6     Relay Drive 2
    5.7     Relay Drive 3
    */
    P5SEL = BIT1+BIT2+BIT3;
    P5DIR = BIT0 + BIT4+BIT5+BIT6+BIT7;
    P5OUT &= ~(BIT4+BIT5+BIT6+BIT7);  //turn off all relay drive coils
    
    /*PORT 6
    6.0     Radio MRDY (optional)
    6.1     used for expander bit-bang output0
    6.2     used for expander bit-bang output1
    6.3     Vunreg through voltage divider
    6.4     Reserved for external current sensor 0
    6.5     Reserved for external current sensor 1
    6.6     debugging pin
    6.7     RS-232 Bit-Bang UART Tx (not used, leave as input)
    */
    P6DIR = BIT1 + BIT2 + BIT6 + BIT7;
    P6SEL = BIT3;     // P6.3,4,5,6-ADC option select  
    
    //debugConsoleInit(BAUD_RATE_115200);   //Use as fast baud rate as we can
    debugConsoleInit(BAUD_RATE_19200);
    auxSerialPortInit(BAUD_RATE_19200);
    
}

/** Configures the debug console UART (USCIA0) for the specified baud rate, with oversampling
Also enables the Rx interrupt for this UART.
@pre SMCLK is 4MHz
@see Table 15-5 of MSP430F2xxx Family User's Guide, slau144
@return 0 if success, -1 if invalid parameter
*/
signed int debugConsoleInit(unsigned char baudRate)
{
    if (baudRate > BAUD_RATE_115200)
        return -1;
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    switch (baudRate)
    {
    case BAUD_RATE_9600:
        UCA0BR0 = 26; UCA0BR1 = 0;                
        UCA0MCTL = UCBRS_0 + UCBRF_1 + UCOS16;                   
        break;     
    case BAUD_RATE_19200:
        UCA0BR0 = 13; UCA0BR1 = 0;                
        UCA0MCTL = UCBRS_0 + UCBRF_0 + UCOS16;    
        break;    
    case BAUD_RATE_38400:
        UCA0BR0 = 6; UCA0BR1 = 0;                 
        UCA0MCTL = UCBRS_0 + UCBRF_8 + UCOS16;   
        break;      
    case BAUD_RATE_115200:
        UCA0BR0 = 2; UCA0BR1 = 0;                  
        UCA0MCTL = UCBRS_3 + UCBRF_2 + UCOS16;           
        break;         
    default:
        printf("Error - Unsupported Baud Rate\r\n");
        return -1;
    }
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt  
    return 0;
}


/** Configures the Auxilliary Serial Port UART (USCIA1) with the specified baud rate.
Also enables the Rx interrupt for this UART.
@pre SMCLK = 4MHz
@return 0 if success, -1 if invalid parameter
*/
signed int auxSerialPortInit(unsigned char baudRate)
{
    if ((baudRate != BAUD_RATE_9600) && (baudRate != BAUD_RATE_19200) && (baudRate != BAUD_RATE_115200))
        return -1;
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    switch (baudRate)
    {
    case BAUD_RATE_19200:
        UCA1BR0 = 13; UCA1BR1 = 0;                // 4mHz smclk w/modulation, table 15-5 
        UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16;   // Modulation UCBRSx=1, over sampling         
        break;
    case BAUD_RATE_9600:
        UCA1BR0 = 26; UCA1BR1 = 0;                // 4mHz smclk w/modulation, table 15-5 
        UCA1MCTL = UCBRS_0 + UCBRF_1 + UCOS16;   // Modulation UCBRSx=1, over sampling         
        break;      
    case BAUD_RATE_115200:
        UCA1BR0 = 2; UCA1BR1 = 3;                // 4mHz smclk w/modulation, table 15-5 
        UCA1MCTL = UCBRS_0 + UCBRF_2 + UCOS16;   // Modulation UCBRSx=1, over sampling         
        break;         
    default:
        printf("Error - Unsupported Baud Rate\r\n");
        return -1;
    }
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UC1IE |= UCA1RXIE;                          // Enable USCI_A1 RX interrupt
    return 0;
}


/** Send one byte via hardware UART. Called by printf() etc. in stdio.h */
int putchar(int c)
{	
    while (!(IFG2 & UCA0TXIFG));   // Wait for ready
    UCA0TXBUF = (unsigned char) (c & 0xFF);  
    return c;
}

/** Send one byte via hardware UART to the auxilliary serial port. Aux Serial Port is USCIA1*/
int putcharAux(int c)
{	
    while (!(UC1IFG&UCA1TXIFG));               // USCI_A1 TX buffer ready?
    UCA1TXBUF = (unsigned char) (c & 0xFF);  
    return c;
}


/**
* Initializes the SPI interface to the ZNP. 
* @note CC2530 SPI clock speed < 4MHz. SPI port configured for clock polarity of 0, clock phase of 0, and MSB first.
* @note On MDB the RFIC SPI port is USCIB1
* @note Modify this method for other hardware implementations.
* @pre SPI pins configured correctly: Clock, MOSI, MISO configured as SPI function; Chip Select configured as an output; SRDY configured as an input.
* @post SPI port is configured for RFIC communications.
*/
void halSpiInitZnp()
{
    UCB1CTL1 |= UCSSEL_2 | UCSWRST;                 //serial clock source = SMCLK, hold SPI interface in reset
    UCB1CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;     //clock polarity = inactive is LOW (CPOL=0); Clock Phase = 0; MSB first; Master Mode; Synchronous Mode
    UCB1BR0 = 2;  UCB1BR1 = 0;                      //SPI running at 2MHz (SMCLK / 2)
    UCB1CTL1 &= ~UCSWRST;                           //start USCI_B1 state machine
    SPI_SS_CLEAR(); 
}


/**
* Sends a message over SPI to the radio IC on USCI_B1. Based on hal_board.c in CC2480 example application ZASA.
* The ZNP uses a "write-to-read" approach to read data out, you must write data in.
* This is a private method that gets wrapped by other methods, e.g. spiSreq(), spiAreq, etc.
* To Write, set *bytes, numBytes.
* To Read, set *bytes only. Don't need to set numBytes because CC2530ZNP will stop when no more bytes read.
* @param bytes the data to be sent or received.
* @param numBytes the number of bytes to be sent. This same buffer will be overwritten with the received data.
* @note Modify this method for other hardware implementations.
* @pre SPI port configured for writing
* @pre CC2530ZNP has been initialized
* @post bytes contains received data, if any
*/
void spiWrite(unsigned char *bytes, unsigned char numBytes)
{
    //SPI_SS_SET(); 
    while (numBytes--)
    {  
        UCB1TXBUF = *bytes;
        while (!(UC1IFG & UCB1RXIFG)) ; //WAIT for a character to be received, if any
        *bytes++ = UCB1RXBUF;  //read bytes
    }
    //SPI_SS_CLEAR();
}


/*
* Blocking Delay in Milliseconds
* delays by at least the specified number of milliseconds (mSec)
* @pre MCLK = 8MHz
* @param ms number of milliseconds to delay
*/
void delayMs(unsigned int ms)    
{
    for (int i = ms; i; i--)
    {
        for (int j = TICKS_IN_ONE_MS; j; j--);   //delay 1mSec
    }
}

/** Sets the status LED to a particular color. Leaves the other LEDs unchanged.
The status LED is a two-element LED (red+green) that is capable of displaying red, green, or yellow.
@note Button LEDs are active-LOW, Status LED is active-HIGH.
@param color the color to set, must be STATUS_LED_RED, STATUS_LED_GREEN, or STATUS_LED_YELLOW. */
void setStatusLed(unsigned char color)   //status LED on P4.6, P4.7
{
    if (color > STATUS_LED_YELLOW) {printf("Error - no such color\r\n"); return; }    
#define EXCLUDE_STATUS_LEDS 0x3F
    P4OUT &= EXCLUDE_STATUS_LEDS;
    P4OUT |= color;
}

/** Turns on the specified button LED. Leaves status LED unchanged.
@note Button LEDs are active-LOW, Status LED is active-HIGH.
@param whichLed the LED to turn on, must be 0-4.
*/
void setButtonLeds(unsigned char led)
{
    if (led > 4) 
        return;
    unsigned char oldStatusLeds = ((BIT6+BIT7) & P4OUT);
    unsigned char newStatus = (1 << led) + BIT6+BIT7;    
    P4OUT = ~(newStatus & (~oldStatusLeds));
}

/** Turns ON the specified LED. Required for ZNP examples.
@param led the LED to turn on, must be 0,1,2,3,4. 
@return 0 if success, -1 if invalid LED specified
*/
signed int setLed(unsigned char led)
{
    if (led > 4) 
        return -1;
    setButtonLeds(led);
    return 0;
}

/** Turns OFF the specified LED. Required for ZNP examples.
@param led the LED to turn off, must be 0,1,2,3,4.
@return 0 if success, -1 if invalid LED specified
*/
void clearLeds()
{
    clearButtonLeds();
}

/** Turns off the button LEDs and leaves status LED unchanged. 
@post button LEDs are all off. Status LED is in the same state as it was before the method was called.*/
void clearButtonLeds()
{
    unsigned char oldStatusLeds = ((BIT6+BIT7) & P4OUT);
    P4OUT = BIT0+BIT1+BIT2+BIT3+BIT4+BIT5 + oldStatusLeds;
}

/** Toggles the specified button LED. 
@param whichLed the LED to toggle, must be 0-4.
@post The specified button LED is toggled. Status LED is unchanged. */
void toggleLed(unsigned char whichLed)
{
    if (whichLed > 4) {printf("Error - no such LED\r\n"); return; }
    P4OUT ^= (1 << whichLed);
}

/** Simple test of all LEDs */
void testLEDs()
{
#define LED_TEST_DELAY 250    
    printf("Testing LEDs3 ");
    setButtonLeds(BUTTON_LEDS_OFF);  
    setStatusLed(STATUS_LED_GREEN);   
    delayMs(LED_TEST_DELAY);
    setStatusLed(STATUS_LED_RED);   
    delayMs(LED_TEST_DELAY);
    setStatusLed(STATUS_LED_YELLOW);   
    delayMs(LED_TEST_DELAY);
    for (int i=0; i<5; i++)
    {
        setButtonLeds(i);
        delayMs(LED_TEST_DELAY);
    }
    setButtonLeds(BUTTON_LEDS_OFF);
}


/** 
* Read value of the two Dual In-Line Package (DIP) switches.
* @return the state of the switches as a number from 0 to 3.
* @pre DIP Switches (P2.6, P2.7) are configured as digital inputs with pull-DOWNs
*/
unsigned char getSwitches()
{ 
    unsigned char value = 0;
    value += (P2IN & BIT6) ? BIT1 : 0;  //switch 1 on P2.6
    value += (P2IN & BIT7) ? BIT2 : 0;  //switch 2 on P2.7
    return value; 
}

unsigned int aClk = 0;

/** Stops the Watchdog timer and starts Oscillator based on the selected option. 
Halts if calibration constants erased.
@param mainOscConfiguration the main oscillator (MCLK) configuration - must be MCLK_1_DCO or MCLK_8_DCO or MCLK_8_XTAL
@param auxOscConfiguration the aux oscillator (ACLK) configuration - must be ACLK_VLO or ACLK_32_XTAL or ACLK_32_EXT
@return 0 if success, else error code if invalid parameter.
*/
signed int oscInit(unsigned char mainOscConfiguration, unsigned char auxOscConfiguration)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    
    switch (auxOscConfiguration)
    {
    case ACLK_VLO:
        BCSCTL3 = LFXT1S_2;                   // Use VLO for ACLK
        break;
    case ACLK_32_XTAL:
        BCSCTL3 = LFXT1S_0;                   // Use 32kHz crystal for ACLK
        BCSCTL3 = XCAP_3;  //12.5pF capacitors on 32kHz crystal. Epson MC-306 requires 12.5pF
        aClk = 32767;
        break;
    case ACLK_32_EXT:
        BCSCTL3 = LFXT1S_3; //setup ACLK to source from external oscillator
        aClk = 32767;        
        break;
    default:
        printf("oscInit main osc FATAL ERROR\r\n");
        return -1;
    }
    
    switch (mainOscConfiguration)
    {
    case MCLK_1_DCO:
        if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)      // Stop if calibration constants erased                                
            while(1); 
        BCSCTL1 = CALBC1_1MHZ; DCOCTL = CALDCO_1MHZ;        // Set DCO = 1MHz
        mclk = 1000000; smclk = 1000000;
        break;
    case MCLK_8_DCO: 
        if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)      // Stop if calibration constants erased                                     
            while(1); 
        BCSCTL1 = CALBC1_8MHZ; DCOCTL = CALDCO_8MHZ;        // Set DCO = 8MHz for MCLK
        BCSCTL2 |= DIVS_1;                                  // SMCLK = DCO/2 (4MHz)  
        mclk = 8000000; smclk = 4000000;       
        break;
    case MCLK_8_XTAL:  //uses external 16MHz xtal for operation at 8MHz MCLK and 4MHz SMCLK
        BCSCTL1 &= ~XT2OFF;                                 // Activate XT2 high freq xtal
        BCSCTL3 |= XT2S_2;                                  // 3 – 16MHz crystal or resonator
        do
        {
            IFG1 &= ~OFIFG;                                 // Clear OSCFault flag
            for (unsigned int i = 0xFFFF; i > 0; i--);      // Time for flag to set
        }
        while (IFG1 & OFIFG);                               // if XT2 OSCFault, then hang here; watchdog will reset
        BCSCTL2 = (SELM_2 + DIVM_1 + SELS + DIVS_2);        // MCLK = XT2 HF XTAL and MCLK div/2 from 16MHz crystal to get 8MHz; source SMCLK from XT2 and div/4 to get 4MHz
        mclk = 8000000; smclk = 4000000;   
        break;
    default:
        printf("oscInit main osc FATAL ERROR\r\n");
        return -2;
    }
    return 0;
}


/** Enable interrupts on the processor */
void halEnableInterrupts()
{
    __bis_SR_register(GIE);  
}

//#define st(x)      do { x } while (__LINE__ == -1)

/** Configures all ZNP interface signals as inputs to allow the ZNP to be programmed.
Toggles LED0 quickly to indicate application is running. */
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
    
    P1DIR = 0;      P1REN = 0;      P1SEL = 0;  P1IE = 0; 
    P2DIR = 0;      P2REN = 0;      P2SEL = 0;  P2IE = 0; 
    P3DIR = 0;      P3REN = 0;      P3SEL = 0;
    P4DIR = BIT0;   P4REN = 0;      P4SEL = 0;   
    P5DIR = 0;      P5REN = 0;      P5SEL = 0; 
    P6DIR = 0;      P6REN = 0;      P6SEL = 0;   
    
    for (;;)
    {
        toggleLed(0);
        delayMs(100);
    }
}

void stopTimerA()
{
    TACTL = MC_0;                                   // Stop timer    
}

unsigned char wakeupAfterTimer = 0;

#define TIMER_MAX_SECONDS 4
/** Configures timer for source = ACLK
@pre ACLK configured
@pre VLO has been calibrated if using VLO; number of VLO counts in one second is in vloFrequency.
@param seconds period of the timer. Maximum is 0xFFFF / aClk; about 2 if using 32kHz xtal or about 4 if using VLO since VLO varies between 9kHz - 15kHz. 
Use a prescaler on timer (e.g. set IDx bits in TACTL register) for longer times. 
Maximum prescaling of Timer A is divide by 8. 
Even longer times can be obtained by prescaling ACLK if this doesn't affect other system peripherals.
@return 0 if success; -1 if illegal parameter or -2 if aClk not set
*/
signed int initTimer(unsigned char seconds, unsigned char wakeOnTimer)
{  
    if ((seconds > TIMER_MAX_SECONDS) || (seconds == 0))
        return -1;
    if (aClk == 0)
        return -2;
    if ((wakeOnTimer != NO_WAKEUP) && (wakeOnTimer != WAKEUP_AFTER_TIMER))
        return -3;
    stopTimerA();                                   // Stop timer
    wakeupAfterTimer = wakeOnTimer;
    CCTL0 = CCIE;                                   // CCR0 interrupt enabled
    CCR0 = aClk * seconds;            
    TACTL = TASSEL_1 + MC_1;                        // ACLK, upmode
    return 0;
}


#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
    timerIsr();
    if (wakeupAfterTimer)
    {
        HAL_WAKEUP();     
    }
}

/** Calibrate VLO. Once this is done, the VLO can be used semi-accurately for timers etc. 
Once calibrated, VLO is within ~2% of actual when using a 1% calibrated DCO frequency and temperature 
and supply voltage remain unchanged. At room temperature, typical VLO frequencies seen are 9kHz (MSP430F248) and 12kHz (MSP430F2274)
@return VLO frequency (number of VLO counts in 1sec)
@pre SMCLK is 4MHz
@pre MCLK is 8MHz
@pre ACLK sourced by VLO (BCSCTL3 = LFXT1S_2; in MSP430F2xxx)
@note calibration is only as good as MCLK source. Obviously, if using the internal DCO (+/- 1%) then this value will only be as good as +/- 1%. YMMV.
@note on MSP430F248 or MSP430F22x2 or MSP430F22x4, must use TACCR2. On MSP430F20x2, must use TACCR0.
Check device-specific datasheet to see which module block has ACLK as a compare input.
For example, see page 23 of the MSP430F24x datasheet or page 17 of the MSP430F20x2 datasheet, or page 18 of the MSP430F22x4 datasheet.
@note if application will require accuracy over change in temperature or supply voltage, recommend calibrating VLO more often.
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
    
    aClk = ((unsigned int) (32000000l / counts));
    if ((aClk > VLO_MIN) && (aClk < VLO_MAX))
        return aClk;
    else
        return -1;
}


/** Private helper method to setup ADC for one-shot conversion and read out value according to registers.
Inserts a delay before beginning conversion if REFON
@return the raw ADC value with the specified commands.
@todo move the VREF warmup to startup and leave on to avoid 17mSec blocking delay each time?
*/
unsigned int getAnalogInput(unsigned int adc12ctl0, unsigned int adc12ctl1, unsigned char adc12mctl0)
{
#define ADC_VREF_DELAY_MS 17    
    ADC12CTL0 = adc12ctl0;
    ADC12CTL1 = adc12ctl1;
    ADC12MCTL0 = adc12mctl0;
    if (adc12ctl0 & REFON)                    // if internal reference is used...
        delayMs(ADC_VREF_DELAY_MS);           // 17mSec delay required to Vref capacitors
    ADC12CTL0 |= ENC;                         // Enable conversions
    ADC12CTL0 |= ADC12SC;                     // Start conversions
    while (!(ADC12IFG & 0x01));               // Conversion done?
    return ADC12MEM0;    // Read out 1st ADC value
}



/** Measures Vcc to the MSP430, nominally 3300mV
- ADC measures VCC/2 compared to 2.5V reference
- If Vcc = 3.3V, ADC output should be (1.65/2.5)*4095 = 2703
- (halfVcc/2.5)*4095 = ADC reading and (Vcc/2.5)*4095 = 2*ADC
- Vcc*4096 = 5*ADC --> and VCC=5*ADC/4095

@return Vcc in millivolts
*/
unsigned int getVcc3()
{
  unsigned int ctl0 = REFON + REF2_5V + ADC12ON + SHT0_15;  // turn on 2.5V ref, set samp time=1024 cycles
  unsigned int ctl1 = SHP;                                  // Use sampling timer, internal ADC12OSC
  unsigned char mctl0 = SREF_1 + INCH_11;                   // Channel A10, Vcc/2 
  //unsigned int analog = getAnalogInput(ctl0, ctl1, mctl0);
#define ADC_COUNT_TO_MILLIVOLT_MULTIPLIER 1.220703125   //(2500mV * 2) / 4096
    return (unsigned int) (getAnalogInput(ctl0, ctl1, mctl0) * ADC_COUNT_TO_MILLIVOLT_MULTIPLIER);

}


/* Working:
  unsigned int ctl0 = REFON + REF2_5V + ADC12ON + SHT0_15;  // turn on 2.5V ref, set samp time=1024 cycles
  unsigned int ctl1 = SHP;                                  // Use sampling timer, internal ADC12OSC
  unsigned char mctl0 = SREF_1 + INCH_11;                   // Channel A10, Vcc/2 
  unsigned int analog = getAnalogInput(ctl0, ctl1, mctl0);
    float multiplier = 5000.0/4096.0;
    float volt = ((float) analog) * multiplier;
    printf("volt = %f\r\n", volt);
    return analog;
*/