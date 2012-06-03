/*
 * remotedefs.h
 *
 *  Created on: Apr 1, 2012
 *      Author: John Tiernan
 */

/*
 * TODO Least efficient way of defining messages. Clean it up
 */

#ifndef REMOTEDEFS_H_
#define REMOTEDEFS_H_


//Format: 0x[device][msgID]

//Room 0/Base
#define CLOSE0				0x01
#define OPEN0				0x02
#define FILM0				0x03
#define DAY0				0x04
#define NIGHT0				0x05

//Room 1
#define CLOSE1				0x11
#define OPEN1				0x12
#define FILM1				0x13
#define DAY1				0x14
#define NIGHT1				0x15

//Room 2
#define CLOSE2				0x21
#define OPEN2				0x22
#define FILM2				0x23
#define DAY2				0x24
#define NIGHT2				0x25


#endif /* REMOTEDEFS_H_ */
