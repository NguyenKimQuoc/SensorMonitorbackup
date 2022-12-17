/*
 * bh1750.c
 *
 *	The MIT License.
 *  Created on: 02.12.2018
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	https://msalamon.pl/pomiar-natezenia-swiatla-z-wykorzystaniem-stm32/
 *	https://github.com/lamik/Light_Sensors_STM32
 */

#include "hal_i2c.h"

#include "bh1750.h"
#include "delay.h"
//I2C_HandleTypeDef 	*bh1750_i2c;	// Handler to I2C interface
bh1750_mode 		Bh1750_Mode = UNCONFIGURED;	// Current sensor mode
uint8 			Bh1750_Mtreg;	// Current MT register value

//
//	Initialization.
//
void BH1750_Init(bh1750_mode Mode)
{
  BH1750_Reset();
  BH1750_SetMode(Mode);
  _delay_ms(10);
  BH1750_SetMtreg(BH1750_DEFAULT_MTREG); // Set default value;
}

//
//	Reset all registers to default value.
//
void BH1750_Reset(void)
{
  uint8 tmp = 0x07;
  HalI2CSend(BH1750_ADDRESS, &tmp, 1);
}

//
//	Set the power state.
//	0 - sleep, low power.
//	1 - running.
//
void BH1750_PowerState(uint8 PowerOn)
{
  PowerOn = (PowerOn? 1:0);
  HalI2CSend(BH1750_ADDRESS, &PowerOn, 1);
}

//
//	Set the mode of converting. Look into bh1750_mode enum.
//
void BH1750_SetMode(bh1750_mode Mode)
{
//  if(!((Mode >> 4) || (Mode >> 5))) return BH1750_ERROR;
//  if((Mode & 0x0F) > 3) return BH1750_ERROR;

  Bh1750_Mode = Mode;
  HalI2CSend(BH1750_ADDRESS, (uint8 *)&Mode, 1);
}

//
//	Set the Measurement Time register. It allows to increase or decrease the sensitivity.
//
void BH1750_SetMtreg(uint8 Mtreg)
{
  //	if (Mtreg < 31 || Mtreg > 254) {
  //		return BH1750_ERROR;
  //	}

  Bh1750_Mtreg = Mtreg;

  uint8 tmp[2];

  tmp[0] = (0x40 | (Mtreg >> 5));
  tmp[1] = (0x60 | (Mtreg & 0x1F));

  HalI2CSend(BH1750_ADDRESS, &tmp[0], 1);
  
  HalI2CSend(BH1750_ADDRESS, &tmp[1], 1);
  
  HalI2CSend(BH1750_ADDRESS, (uint8 *)&Bh1750_Mode, 1);
  _delay_ms(10);
}

//
//	Trigger the conversion in manual modes.
//	For low resolution conversion time is typical 16 ms,
//	for high resolution 120 ms. You need to wait until read the measurement value.
//	There is no need to exit low power mode for manual conversion. It makes automatically.
//
void BH1750_TriggerManualConversion(void)
{
  BH1750_SetMode(Bh1750_Mode);
}

void waitMeasurementReady(bool maxWait) {
  unsigned long delaytime = 0;
  switch (Bh1750_Mode) {
  case CONTINUOUS_HIGH_RES_MODE:
  case CONTINUOUS_HIGH_RES_MODE_2:
  case ONE_TIME_HIGH_RES_MODE:
  case ONE_TIME_HIGH_RES_MODE_2:
    maxWait ? (delaytime = (180 * Bh1750_Mtreg / (byte)BH1750_DEFAULT_MTREG))
            : (delaytime = (120 * Bh1750_Mtreg / (byte)BH1750_DEFAULT_MTREG));
    break;
  case CONTINUOUS_LOW_RES_MODE:
  case ONE_TIME_LOW_RES_MODE:
    // Send mode to sensor
    maxWait ? (delaytime = (24 * Bh1750_Mtreg / (byte)BH1750_DEFAULT_MTREG))
            : (delaytime = (16 * Bh1750_Mtreg / (byte)BH1750_DEFAULT_MTREG));
    break;
  default:
    break;
  }
  // Wait for new measurement to be possible.
  // Measurements have a maximum measurement time and a typical measurement
  // time. The maxWait argument determines which measurement wait time is
  // used when a one-time mode is being used. The typical (shorter)
  // measurement time is used by default and if maxWait is set to True then
  // the maximum measurement time will be used. See data sheet pages 2, 5
  // and 7 for more details.
  _delay_ms(delaytime);
}

//
//	Read the converted value and calculate the result.
//
void BH1750_ReadLight(double *Result)
{
  double result;
  uint8 tmp[2];
  HalI2CStartCondition();
  HalI2CSendByte(BH1750_ADDRESS|0x01);
  HalI2CRead(tmp, 2);
  HalI2CStop();
  result = (tmp[0] << 8) | (tmp[1]);

  if(Bh1750_Mtreg != BH1750_DEFAULT_MTREG)
  {
          result *= (double)((uint8)BH1750_DEFAULT_MTREG/(double)Bh1750_Mtreg);
  }

  if(Bh1750_Mode == ONE_TIME_HIGH_RES_MODE_2 || Bh1750_Mode == CONTINUOUS_HIGH_RES_MODE_2)
  {
          result /= 2.0;
  }

  *Result = result / (double)BH1750_CONVERSION_FACTOR;
}