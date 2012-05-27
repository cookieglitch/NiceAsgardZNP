/**
* @ingroup hal
* @{ 
* @file hal_vti_cma3000_D01_accelerometer.c
* 
* @brief Hardware Abstraction Layer (HAL) support file for VTI Technologies CMA3000-D01 Accelerometer.
* 
* @note ACCELEROMETER_RX_BUF and ACCELEROMETER_SS_SET and ACCELEROMETER_SS_CLEAR, and spiWriteAccelerometer() must be defined
* @see VTI Technologies document: CMA3000-D01 datasheet
* @see VTI Technologies document: Product Family Specification, CMA3000-D0X Series 3-axis accelerometer
* @see VTI Technologies document: Interfacing CMA3000-D01 to an MSP430 ultra low-power microcontroller
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

#include "../HAL/hal.h"   

/** Read a byte from the accelerometer over SPI
@pre SPI port configured for the accelerometer correctly
@param reg which register to read from
@return the value read from the register
*/
unsigned char readAccelerometerRegister(unsigned char reg) 
{ 
    unsigned char result = 0; 
    reg <<= 2;                            // Address to be shifted left by 2 
    ACCELEROMETER_SS_SET();               // Select acceleration sensor 
    result = ACCELEROMETER_RX_BUF;        // Read RX buffer just to clear interrupt flag 
    spiWriteAccelerometer(reg);           // Set which register to read from
    result = spiWriteAccelerometer(0);    // Read the selected register
    ACCELEROMETER_SS_CLEAR();             // Deselect acceleration sensor 
    return result; 
} 

/** Write a byte to the acceleration sensor over SPI
@pre SPI port configured correctly
@param reg which register to read from
@return the value returned by the write operation
*/
unsigned char writeAccelerometerRegister(unsigned char reg, unsigned char data) 
{ 
    unsigned char result = 0; 
    reg <<= 2;                            // Address to be shifted left by 2 
    reg  |= 2;                            // RW bit to be set 
    ACCELEROMETER_SS_SET();               // Select acceleration sensor 
    result = ACCELEROMETER_RX_BUF;        // Read RX buffer just to clear interrupt flag 
    spiWriteAccelerometer(reg);           // Set which register to write to
    result = spiWriteAccelerometer(data); // Write the data
    ACCELEROMETER_SS_CLEAR();             // Deselect acceleration sensor 
    return result; 
} 

/** Returns the name of the Accelerometer Interrupt Status Register that caused the interrupt.
@param interruptReason the value read from the ACCEL_INT_STATUS register.
@return the name of the reason for the interrupt, e.g. "Motion on X" or "Motion on Y" or "Free Fall" etc.
*/
char* getAccelerometerInterruptReason(unsigned char interruptReason)
{
    switch (interruptReason)
    {
    case 0: return "None";
    case 1: return "Motion on X"; 
    case 2: return "Motion on Y"; 
    case 3: return "Motion on Z";
    case 4: return "Free fall";
    default: return "Unknown";
    }
}


/* @} */