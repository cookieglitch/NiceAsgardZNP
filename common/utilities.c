/**
*  @file utilities.c
*
*  @brief Miscellaneous utility methods
*
*  Miscellaneous helper utilities.
* 
* $Rev: 669 $
* $Author: dsmith $
* $Date: 2010-07-13 20:23:40 -0700 (Tue, 13 Jul 2010) $
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
#include <stdlib.h>
#ifdef USE_STDIO
#include <stdio.h>
#else
#include "printf.h"
#endif
#ifdef GW0
#include "../HAL/hal.h"
//#define printf UARTprintf
#endif

void printHexBytes(unsigned char* toPrint, unsigned char numBytes)
{
    for (int i=0; i<numBytes; i++)
        printf("%02X ", toPrint[i]);
    printf("\r\n");
}

/** Fills the messagePartBuffer with DEADBEEF to aid in debugging */
void initializeBuffer(unsigned char* buf, unsigned char len)
{
  for (int i=0; i<len; i+=4)  // initialize buffer
  {
    buf[i] = 0xDE;
    buf[(i+1)] = 0xAD;
    buf[(i+2)] = 0xBE;
    buf[(i+3)] = 0xEF;
  }
}


/** Prints a number in its binary form */
void printBinary(unsigned char n) 
{
    unsigned int i;
    i = 1<<(sizeof(n) * 8 - 1);
    while (i > 0) {
        if (n & i)
            printf("1");
        else
            printf("0");
        i >>= 1;
    }
}

/** returns >1 if is printable: ASCII values 0x20 (space) through 0x7E (~) */
char isAsciiPrintableCharacter(unsigned char c)
{
    return ((c >= 0x20) && (c <= 0x7E));
}
