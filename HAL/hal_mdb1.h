/**
*  @file hal_mdb1.h
*
*  @brief  public methods for hal_mdb1.c
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
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "msp430x24x.h"
#include <stdio.h>
#include "../Common/utilities.h"

#define STATUS_LED_RED BIT6
#define STATUS_LED_GREEN BIT7
#define STATUS_LED_YELLOW (BIT6+BIT7)
#define ALL_LEDS_OFF 0x3F 

#define HAL_SLEEP()     ( __bis_SR_register(LPM3_bits + GIE))
#define HAL_WAKEUP()    ( __bic_SR_register_on_exit(LPM3_bits))

#define HAL_ENABLE_INTERRUPTS()         (_EINT())
#define HAL_DISABLE_INTERRUPTS()        (_DINT())
#define HAL_CLEAR_INTERRUPTS()          (      P2IFG = 0, P1IFG = 0, IFG2 &= ~UCA0RXIFG, UC1IFG &= ~UCA1RXIFG ) 
//options for halConfigureSrdyInterrrupt()
#define SRDY_INTERRUPT_FALLING_EDGE 0x01
#define SRDY_INTERRUPT_RISING_EDGE  0x00

void halInit();
void delayMs(unsigned int ms);
void toggleLed(unsigned char whichLed);
signed int setLed(unsigned char led);
void clearLeds();
void halSpiInitZnp();
void spiWrite(unsigned char *bytes, unsigned char numBytes);
void setZnpInterfaceToInputs(void);
signed int calibrateVlo();
signed int initTimer(unsigned char seconds, unsigned char wakeOnTimer);


//NOT REQUIRED FOR ZNP LIBRARY:
signed int oscInit(unsigned char mainOscConfiguration, unsigned char auxOscConfiguration);
unsigned char getSwitches();
unsigned char getButtons();
void testLEDs();
void setStatusLed(unsigned char color);
void setButtonLeds(unsigned char buttonId);
void clearButtonLeds();
int putcharAux(int c);
signed int debugConsoleInit(unsigned char baudRate);
signed int auxSerialPortInit(unsigned char baudRate);
void stopTimerA();
void delayUs(unsigned char hundredsOfMicroSeconds);
unsigned int getVcc3();


//REQUIRED FOR ZNP LIBRARY:
#define RADIO_ON()                  (P1OUT |= BIT2)
#define RADIO_OFF()                 (P1OUT &= ~BIT2)

#define SPI_SS_SET()                (P5OUT &= ~BIT0)  //active low
#define SPI_SS_CLEAR()              (P5OUT |= BIT0)  
#define SRDY_IS_HIGH()              (P1IN & BIT3)
#define SRDY_IS_LOW()               ((~P1IN) & BIT3)

//NOT REQUIRED FOR ZNP LIBRARY:
#define DEBUG_ON()                  (P6OUT |= BIT6)
#define DEBUG_OFF()                 (P6OUT &= BIT6)
#define DEBUG_TOGGLE()              (P6OUT ^= BIT6)

#define GET_MCLK_FREQ()             8000000L 

#define XTAL 8000000L
#define TICKS_IN_ONE_MS_8MHZ        1980   //derived experimentally
#define TICKS_IN_ONE_MS             TICKS_IN_ONE_MS_8MHZ  
#define TICKS_IN_100_US             155

#define NO_WAKEUP 0
#define WAKEUP_AFTER_TIMER 1
#define WAKEUP_AFTER_SRDY           4

#define MASTER_BUTTON_0 0x00
#define MASTER_BUTTON_1 0x01
#define MASTER_BUTTON_2 0x02
#define MASTER_BUTTON_3 0x03
#define MASTER_BUTTON_4 0x04

#define BUTTON_LEDS_OFF 0xFF

//baud rate options for serial port init
#define BAUD_RATE_9600      0x00
#define BAUD_RATE_19200     0x01
#define BAUD_RATE_38400     0x02
#define BAUD_RATE_115200    0x03


//oscInit MCLK options:
#define MCLK_1_DCO      0x00
#define MCLK_8_DCO      0x01
#define MCLK_8_XTAL     0x02
//oscInit ACLK options:
#define ACLK_VLO        0x00
#define ACLK_32_XTAL    0x01
#define ACLK_32_EXT     0x02

//Required for using VLO
#define VLO_NOMINAL 12000
#define VLO_MIN (VLO_NOMINAL - (VLO_NOMINAL >> 2)) //VLO min max = +/- 25% of nominal
#define VLO_MAX (VLO_NOMINAL + (VLO_NOMINAL >> 2))

//Required for bit-bang UART
//Note: be sure to also modify the ISR in Port1 or Port2 for bitBangSerialIsr

/*
#define BIT_BANG_RX0_PORT                   P2IN
#define BIT_BANG_RX0_BIT                    BIT6 //BIT5
#define DISABLE_BIT_BANG_RX0_INTERRUPT()    (P2IE &= ~BIT6) //BIT5)
#define ENABLE_BIT_BANG_RX0_INTERRUPT()     (P2IE |= BIT6)   //BIT5)
*/
//Note: Signal is inverted through opto!
#define BIT_BANG_TX0_PORT                   P6OUT
#define BIT_BANG_TX0_BIT                    BIT1  //BIT7
#define BIT_BANG_RX0_PORT                   P1IN
#define BIT_BANG_RX0_BIT                    BIT4 //BIT5
#define DISABLE_BIT_BANG_RX0_INTERRUPT()    (P1IE &= ~BIT4) //BIT5
#define ENABLE_BIT_BANG_RX0_INTERRUPT()     (P1IE |= BIT4)   //BIT5

#define BIT_BANG_TX1_PORT                   P6OUT
#define BIT_BANG_TX1_BIT                    BIT2
#define BIT_BANG_RX1_PORT                   P1IN
#define BIT_BANG_RX1_BIT                    BIT5
#define DISABLE_BIT_BANG_RX1_INTERRUPT()    (P1IE &= ~BIT5)
#define ENABLE_BIT_BANG_RX1_INTERRUPT()     (P1IE |= BIT5)

#define TOGGLE_DEBUG_PIN()                  (P4OUT ^= BIT5)  //debugging only


#endif
