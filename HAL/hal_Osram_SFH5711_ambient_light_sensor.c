/**
* @ingroup hal
* @{ 
* @file hal_Osram_SFH5711_ambient_light_sensor.c
*
* @brief Conversion routines for the Osram SFH5711 Ambient Light Sensor.
*
* @note Light sensor outputs current between 0 to 50uA. On the target board, load resistor = 47k Ohms.
* @note 0 to 100,000lx will generate 0 to 2350mV input into an Analog to Digital Converter (ADC).
* @note This particular light sensor is binned for a sensitivity of 27 to 30 uA @ 1000lux = 1269-1410 mV = ADC readings of 520-578counts 
* @note there are two versions of the convertAdcToLux() method: 
* - Use a lookup table. Faster and less code, but also less accurate.
* - Compute the value directly based on equation supplied by the manufacturer. 
* This is much more accurate but requires approximately 4.2kB more code for the pow() function in math.h.
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

#define LIGHT_SENSOR_USE_LOOKUP_TABLE

#ifdef LIGHT_SENSOR_USE_LOOKUP_TABLE

#define ADC_TO_LUX_LOOKUP_TABLE_ROWS 47

/** Lookup table used if LIGHT_SENSOR_USE_LOOKUP_TABLE is defined in lieu of calculating the value with math. 
First value is ADC reading, second is lux/10 */
const unsigned int adcToLuxLookupTable[ADC_TO_LUX_LOOKUP_TABLE_ROWS][2] = { 
    {193,1},     {250,2},     {284,3},    {308,4},    {327,5},    {342,6},    {355,7},     {366,8},     {376,9},
    {385,10},    {443,20},    {477,30},   {501,40},   {520,50},   {535,60},   {548,70},    {559,80},    {569,90},
    {578,100},   {635,200},   {669,300},  {693,400},  {712,500},  {727,600},  {740,700},   {751,800},   {761,900},
    {770,1000},  {828,2000},  {862,3000}, {886,4000}, {905,5000}, {920,6000}, {933,7000},  {944,8000},  {954,9000},
    {963,10000}, {970,11000}, {978,12000},{985,13000},{991,14000},{997,15000},{1002,16000},{1007,17000},{1012,18000},{1016,19000},{1020,20000}}; 

/** Converts the raw ADC value to lux for the Osram SFH5711 light sensor using adcToLuxLookupTable[].
Iterates through the table until nearest value is found. Does not interpolate.
@param adc the raw reading from the Analog to Digital converter
@return the amount of light, in lux/10, or 0xFFFF if the adc value was larger than the table. 
This value should be nearly impossible to obtain othererwise unless you are on the sun.
*/
unsigned int convertAdcToLux(unsigned int adc)
{
    unsigned char iterator = 0;
    while (adc > adcToLuxLookupTable[iterator][0])
        iterator++;
    if (iterator == ADC_TO_LUX_LOOKUP_TABLE_ROWS)
        return 0xFFFF;  //ADC value not found in table; error. 
    return (adcToLuxLookupTable[iterator][1]);
    
}

#else   //don't use the lookup table; calculate the value with math
#include <math.h>
/**
Converts the raw ADC value to lux for the Osram SFH 5711 light sensor using pow() method in math.h.
Conversion:
- Convert ADC to millivolts: mV=(ADC/1024) * 2500  (10bit ADC = 1024counts; we're using the 2.5V reference)
- Convert millivolts to microAmps: uA = mV / 47 (since the design uses a 47kOhm load resistor)
- Convert microAmps to lux: lux = 10^(uA/10)
@param adc the raw reading from the Analog to Digital converter
@return the amount of light, in lux/10.
*/
unsigned int convertAdcToLux(unsigned int adc)
{
    const float multiplier =(2500.0/47.0)/1024.0;
    return ((unsigned int) pow(10, ((adc*multiplier)/10) - 1));
}
#endif







/* @} */
