/**
*  @file utilities.h
*
*  @brief  public methods for utilities.c
*
* $Rev: 725 $
* $Author: dsmith $
* $Date: 2010-08-19 11:36:50 -0700 (Thu, 19 Aug 2010) $
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
#ifndef UTILITIES_H
#define UTILITIES_H

/** Convert two unsigned chars to an unsigned int, LSB first*/
#define CONVERT_TO_INT(lsb,msb) ((lsb) + 0x0100*(msb))   // ((lsb) + (((unsigned int) (msb)) << 8))

/** Get the Least Significant Byte (LSB) of an unsigned int*/
#define LSB(num) ((num) & 0xFF)

/** Get the Most Significant Byte (MSB) of an unsigned int*/
#define MSB(num) ((num) >> 8)

#define BYTES_TO_LONG(byteArray) (( ((unsigned long)byteArray[0] << 24) + ((unsigned long)byteArray[1] << 16) + ((unsigned long)byteArray[2] << 8) + ((unsigned long)byteArray[3] ) ) );

void initializeBuffer(unsigned char* buf, unsigned char len);
void printBinary(unsigned char n);
void printHexBytes(unsigned char* toPrint, unsigned char numBytes);
char isAsciiPrintableCharacter(unsigned char c);
#endif