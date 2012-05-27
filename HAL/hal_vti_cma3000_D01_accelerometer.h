/**
*  @file hal_vti_cma3000_D01_accelerometer.h
*
*  @brief  public methods for hal_vti_cma3000_D01_accelerometer.c
*
* include this file in applications where you want to use the accelerometer.
*
* $Rev: 545 $
* $Author: dsmith $
* $Date: 2010-05-27 17:09:05 -0700 (Thu, 27 May 2010) $
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
#ifndef hal_vti_cma3000_d01_H
#define hal_vti_cma3000_d01_H

unsigned char readAccelerometerRegister(unsigned char Address);
unsigned char writeAccelerometerRegister(unsigned char Address, unsigned char Data);
char* getAccelerometerInterruptReason(unsigned char interruptReason);

// CMA3000 Registers 
#define ACCEL_WHO_AM_I        0x00 
#define ACCEL_REVID           0x01 
#define ACCEL_CTRL            0x02 
#define ACCEL_STATUS          0x03 
#define ACCEL_RSTR            0x04 
#define ACCEL_INT_STATUS      0x05 
#define ACCEL_DOUTX           0x06 
#define ACCEL_DOUTY           0x07 
#define ACCEL_DOUTZ           0x08 
#define ACCEL_MDTHR           0x09 
#define ACCEL_MDFFTMR         0x0A 
#define ACCEL_FFTHR           0x0B 
#define ACCEL_I2C_ADDR        0x0C 

// Control Register setup 
#define G_RANGE_2       0x80  // 2g range 
#define INT_LEVEL_LOW   0x40  // INT active high 
#define MDET_NO_EXIT    0x20  // Remain in motion detection mode 
#define I2C_DIS         0x10  // I2C disabled 
#define MODE_PD         0x00  // Power Down 
#define MODE_100        0x02  // Measurement mode 100 Hz ODR
#define MODE_400        0x04  // Measurement mode 400 Hz ODR 
#define MODE_40         0x06  // Measurement mode 40 Hz ODR 
#define MODE_MD_10      0x08  // Motion detection mode 10 Hz ODR 
#define MODE_FF_100     0x0A  // Free fall detection mode 100 Hz ODR 
#define MODE_FF_400     0x0C  // Free fall detection mode 400 Hz ODR 
#define INT_DIS         0x01  // Interrupts enabled 

//Note: According to app note "Interface CMA3000-D01 to MSP430", there must be at least 11 SCK clock periods between SPI frames.
#define ACCELEROMETER_DELAY_BETWEEN_OPERATIONS_US  44



#endif