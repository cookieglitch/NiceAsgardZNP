
/**
* @file hal.h
*
* @brief Hardware Abstraction Layer (HAL) file
*
* Point this file to your Hardware Abstraction Layer file
*
* The ZNP library & examples require the following methods to be implemented. 
* Refer to the individual method descriptions for more information.
* - halInit()
* - putchar()
* - halSpiInit()
* - delayMs()
* - halResetZnp()
* - toggleLed()
*
*
* @note you must also set the processor version in Project Properties and be sure to 
* add your hal file to the Project. I recommend creating 
* @see hal_helper.c for utilities to assist when changing hardware platforms
*
* $Rev: 545 $
* $Author: dsmith $
* $Date: 2010-05-27 17:09:05 -0700 (Thu, 27 May 2010) $
*/

#ifdef MDB1                       //Smith Electronics ZM-1 Module Development Board
#include "../HAL/hal_mdb1.h"
#elif defined GW0
#include "../HAL/hal_gw0.h"	  //Smith Electronics Zigbee to Ethernet Gateway GW0
#else
#include "../HAL/hal_cc2530znp_target_board.H"
#endif
