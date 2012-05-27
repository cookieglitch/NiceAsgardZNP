/**
 * @ingroup hal
 * @{
 * @file hal_helper.c
 *
 * @brief Utilities to assist changing hardware platforms
 *
 * We designed the ZNP library and examples to be easy to port to different hardware platforms.
 * All interfaces to the hardware is done through one Hardware Abstraction Library (HAL) file. 
 * To change hardware platforms you need to create a HAL file for your hardware platform.
 * 
 * These utility methods will help you if you are changing hardware platforms by providing simple test
 * methods for the HAL methods required by the ZNP library. You will need to supply your own hal_xxxx.c 
 * and hal_xxxx.h files, where xxxx is the name of your hardware platform. If you are changing hardware 
 * it is assumed that you are very familiar with your destination hardware platform.
 *
 * There are several tests, #define the one you want to run.
 *
 * To change hardware platforms:
 * - You will need access to an oscilloscope and/or logic analyzer. Good low cost ($150) logic analyzer: www.saleae.com
 * - Copy and rename the hal_cc2530znp_target_board.h and .c files for your hardware.
 * - Include your new hal_xxxx.c file in this project, and remove the existing hal_cc2530znp_target_board.c file
 * - Using the schematic of your target board, update the MACROS REQUIRED FOR ZNP in the .h file and all the methods in the .c file.
 * - Go through the tests below in order.
 * - Run the basic examples: button_blink, hello_world, and reset_radio.
 * - Be sure that the target device is configured in your compiler correctly.
 * - Next, run the basic communications examples.
 * - We recommend you get the examples working first, and then do any optimizations, rather than the other way around.
 * - Refer to the TI E2E forums for support.
 *
 * $Rev: 600 $
 * $Author: dsmith $
 * $Date: 2010-06-16 13:08:09 -0700 (Wed, 16 Jun 2010) $
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
#include "hal.h"            //point this to your HAL file

/* Uncomment only ONE of the tests below to run that test */
//#define TEST_DELAY            //STEP 1 - get basic delay, LEDs working
//#define TEST_PUTCHAR          //STEP 2 - get putchar() working so that printf() will work
#define TEST_PRINTF           //STEP 3 - test printf() with a simple hello world application
//#define TEST_ZNP_RESET        //STEP 4 - verify ZNP is being reset
//#define TEST_SPI_WRITE        //STEP 5 - verify the SPI port is configured correctly (if using SPI)
//#define TEST_SYS_RESET_IND    //STEP 6 - Do something useful - reset the ZNP and display the response
//#define TEST_SYS_GET_MAC      //STEP 7 - Do something even more useful - query the ZNP for the MAC Address

#ifdef TEST_DELAY
/** Use this to verify that you've configured halInit(), toggleLed() and delayMs() correctly. 
Monitor LED with oscilloscope or logic analyzer to verify that delay is equal to ~50mSec. */
int main( void )
{
    halInit();
    while (1) 
    { 
        setLed(0);
        delayMs(50);
        clearLeds();
        delayMs(50);
    }
}
#endif

#ifdef TEST_PUTCHAR
/** Use this to verify that putchar() is working. printf() requires a working putchar() method. 
You should see a steady stream of 'U' output to your terminal.
This test is also useful if you are implementing a software UART and need to verify bit timing */
int main( void )
{
    halInit();
    while (1)
    {
        toggleLed(0);
        printf("U");
        delayMs(1000);
    }
}
#endif

#ifdef TEST_PRINTF
/** Use this to verify that printf() is working.
@pre working putchar() method.
 */
int main( void )
{
    halInit();
    int num = 0;
    while (1)
    {
        printf("Hello World %u\r\n", num++);
        delayMs(1000);
    }
}
#endif

#ifdef TEST_ZNP_RESET
/** This utility will toggle the reset line of the ZNP and determine whether the ZNP firmware is loaded.
When using SPI, toggling the reset line should cause SRDY to go high after approx. 400mSec to indicate a SYS_RESET_IND message.
This utility doesn't read the SPI bytes, just checks the SRDY line. 
Verify that the pin connected to the hardware reset of the ZNP is pulled down for 1mSec.
Configure logic analyzer to trigger on a 1->0 transition on ZNP reset line*/
int main( void )
{
    halInit();
    printf("Test ZNP Reset\r\n");
    RADIO_OFF();
    delayMs(1);
    RADIO_ON();

#define TEST_SRDY_INTERVAL_MS 1  //check SRDY every 100 mSec
#define TEST_SRDY_TIMEOUT_MS  1000
    unsigned int elapsedTime = 0;       //now, poll for SRDY going low...
    do
    {
        delayMs(TEST_SRDY_INTERVAL_MS);
        elapsedTime += TEST_SRDY_INTERVAL_MS;
    }
    while ((elapsedTime < TEST_SRDY_TIMEOUT_MS) && (SRDY_IS_HIGH()));
    if (SRDY_IS_LOW())
    {
        printf("Test PASSED - SRDY went low after approximately %umS\r\n", elapsedTime);
    }
    else
    {
        printf("ERROR - SRDY never went low\r\n");
    }
}
#endif


#ifdef TEST_SPI_WRITE
/** Use this to verify that the SPI port is configured correctly and writing bytes
Use a logic analyzer or scope and view the bytes being written.
The ZNP won't do anything but you'll be able to determine whether the SPI port is working.
Configure logic analyzer to trigger on a 1->0 transition on CS.
Verify that you're seeing the bytes written out the SPI port.
 */
int main( void )
{
    halInit();                          //Initialize hardware    
    halSpiInitZnp();
    halInit();
    printf("Test ZNP Reset\r\n");
    RADIO_OFF();
    delayMs(1);
    RADIO_ON();
#define TEST_SRDY_INTERVAL_MS 1  //check SRDY every 100 mSec
#define TEST_SRDY_TIMEOUT_MS  1000
    unsigned int elapsedTime = 0;       //now, poll for SRDY going low...
    do
    {
        delayMs(TEST_SRDY_INTERVAL_MS);
        elapsedTime += TEST_SRDY_INTERVAL_MS;
    }
    while ((elapsedTime < TEST_SRDY_TIMEOUT_MS) && (SRDY_IS_HIGH()));
    if (SRDY_IS_LOW())
    {
        printf("Test PASSED - SRDY went low after approximately %umS\r\n", elapsedTime);
    }
    else
    {
        printf("ERROR - SRDY never went low\r\n");
    }


    printf("SPI Write Test\r\n");
#define TEST_DATA {0x05, 0x04, 0x03, 0x02, 0x01, 0x00}
#define TEST_DATA_LENGTH 6
    unsigned char test[] = TEST_DATA;
    spiWrite(test, TEST_DATA_LENGTH);
    printf("Done!\r\n");
}
#endif

#if defined(TEST_SYS_RESET_IND) || defined(TEST_SYS_GET_MAC)

unsigned char znpBuf[100];

/** From znp_interface_spi.c, implemented here to avoid pesky dependencies */
signed int sendSreq()
{
    SPI_SS_SET();   
    while (SRDY_IS_HIGH()) ;                    //wait until SRDY goes low     
    spiWrite(znpBuf, (*znpBuf + 3));              // *bytes (first byte) is length after the first 3 bytes, all frames have at least the first 3 bytes
    *znpBuf = 0; *(znpBuf+1) = 0; *(znpBuf+2) = 0; //poll message is 0,0,0
    //SPI_SS_CLEAR();    //NOTE: MRDY must remain asserted here, but can de-assert SS if the two signals are separate
    while (SRDY_IS_LOW()) ;                     //wait for data
    //SPI_SS_SET();      //NOTE: if SS & MRDY are separate signals then can re-assert SS here.
    spiWrite(znpBuf, 3);
    if (*znpBuf > 0)                             // *bytes (first byte) contains number of bytes to receive
        spiWrite(znpBuf+3, *znpBuf);              //write-to-read: read data into buffer    
    SPI_SS_CLEAR();
    return 0;  
}

/** From znp_interface_spi.c */
signed int spiPoll()
{
    *znpBuf = 0; *(znpBuf+1) = 0; *(znpBuf+2) = 0;  //poll message is 0,0,0 
    return(sendSreq());
}
#endif

#ifdef TEST_SYS_RESET_IND
/** Reset the ZNP, poll for the SYS_RESET_IND message and display the contents.
 * Similar to the method znpReset() in znp_interface.c */
int main( void )
{
    halInit();                          //Initialize hardware
    halSpiInitZnp();
    printf("Resetting ZNP\r\n");
    RADIO_OFF();
    delayMs(1);
    RADIO_ON();
    spiPoll();
    for (int i=0; i< (znpBuf[0] + 3); i++)
        printf("%02X ", znpBuf[i]);
    printf("\r\n");
}
#endif

#ifdef TEST_SYS_GET_MAC
/** Reset the ZNP and then get the MAC. */
int main( void )
{
    halInit();                          //Initialize hardware
    halSpiInitZnp();
    printf("Resetting ZNP to get MAC address\r\n");
    RADIO_OFF();
    delayMs(1);
    RADIO_ON();
    spiPoll();
    for (int i=0; i< (znpBuf[0] + 3); i++)
        printf("%02X ", znpBuf[i]);
    printf("\r\n");
    
    znpBuf[0] = 1;      //ZB_GET_DEVICE_INFO_PAYLOAD_LEN;
    znpBuf[1] = 0x26;   //MSB(ZB_GET_DEVICE_INFO);
    znpBuf[2] = 0x06;   //LSB(ZB_GET_DEVICE_INFO);
    znpBuf[3] = 1;      //DIP_MAC_ADDRESS;
    sendSreq();
    printf("MAC Address, LSB first:");
    for (int i=4; i< (znpBuf[0] + 3); i++)
        printf("%02X ", znpBuf[i]);
    printf("\r\n");
}
#endif
/* @} */
