/**
*  @file hal_cc2530znp_target_board.h
*
*  @brief  public methods for hal_cc2530znp_target_board.c
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
#ifndef hal_cc2530znp_target_board_H
#define hal_cc2530znp_target_board_H

//
//  Common Includes (will be included in all projects)
//
#include "msp430x22x4.h"
#include "../Common/printf.h"
#include "../Common/utilities.h"

//
//  METHODS REQUIRED FOR ZNP
//
void halInit();
void delayMs(unsigned int ms);
void delayUs(unsigned int us);
signed int toggleLed(unsigned char led);
signed int setLed(unsigned char led);
void clearLeds();
void halSpiInitZnp();
void spiWrite(unsigned char *bytes, unsigned char numBytes);
signed int calibrateVlo();
signed int initTimer(unsigned char seconds, unsigned char wakeOnTimer);

//
//  METHODS NOT REQUIRED FOR ZNP
//
unsigned int getVcc3();
unsigned int getLightSensor();

void setZnpInterfaceToInputs(void);
signed int halConfigureSrdyInterrrupt(unsigned char flags);
void halSpiInitAccelerometer();
signed int halEnableAccelerometerInterrupt(unsigned char wakeOnAccelerometer);
unsigned char spiWriteAccelerometer(unsigned char b);

#define HAL_SLEEP()     ( __bis_SR_register(LPM3_bits + GIE))
#define HAL_WAKEUP()    ( __bic_SR_register_on_exit(LPM3_bits))

#define ENABLE_SRDY_INTERRUPT()     (P2IE |= BIT6)
#define DISABLE_SRDY_INTERRUPT()    (P2IE &= ~BIT6)

#define HAL_ENABLE_INTERRUPTS()         (_EINT())
#define HAL_DISABLE_INTERRUPTS()        (_DINT())

//options for halConfigureSrdyInterrrupt()
#define SRDY_INTERRUPT_FALLING_EDGE 0x01
#define SRDY_INTERRUPT_RISING_EDGE  0x00

//
//  MACROS REQUIRED FOR ZNP
//
#define RADIO_ON()                  (P3OUT |= BIT7)  //ZNP Reset Line
#define RADIO_OFF()                 (P3OUT &= ~BIT7)
//  ZNP SPI
#define SPI_SS_SET()                (P3OUT &= ~(BIT0 | BIT6))  //active low, control SS and MRDY
#define SPI_SS_CLEAR()              (P3OUT |= (BIT0 | BIT6))  
#define SRDY_IS_HIGH()              (P2IN & BIT6)
#define SRDY_IS_LOW()               ((~P2IN) & BIT6)

#define DEBUG_ON()                  (P4OUT |= BIT4)
#define DEBUG_OFF()                 (P4OUT &= BIT4)

#define NO_WAKEUP                   0
#define WAKEUP_AFTER_TIMER          1
#define WAKEUP_AFTER_ACCELEROMETER  2
#define WAKEUP_AFTER_SRDY           4

//
//  MISC OTHER DEFINES
//
#define XTAL 8000000L
#define TICKS_PER_MS (XTAL / 1000 / 5 - 1)
#define TICKS_PER_US (TICKS_PER_MS / 1000)
#define VLO_NOMINAL 12000
#define VLO_MIN (VLO_NOMINAL - (VLO_NOMINAL/4)) //VLO min max = +/- 25% of nominal
#define VLO_MAX (VLO_NOMINAL + (VLO_NOMINAL/4))

#define GET_MCLK_FREQ()     8000000L    

//Required for Light Sensor - NOT REQUIRED FOR ZNP
#define LIGHT_SENSOR_ON()                  (P2OUT |= BIT1)  //ZNP Reset Line
#define LIGHT_SENSOR_OFF()                 (P2OUT &= ~BIT1)

//Required for Accelerometer - NOT REQUIRED FOR ZNP
#define ACCELEROMETER_SS_SET()                (P2OUT &= ~(BIT2|BIT7))  //active low
#define ACCELEROMETER_SS_CLEAR()              (P2OUT |= (BIT2|BIT7))  
#define ACCELEROMETER_RX_BUF    UCB0RXBUF




#endif