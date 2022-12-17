/*
 * bh1750.h
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

#ifndef BH1750_H_
#define BH1750_H_

#define BH1750_ADDRESS			(0x23<<1)

#define	BH1750_POWER_DOWN		0x00
#define	BH1750_POWER_ON			0x01
#define	BH1750_RESET			0x07
#define	BH1750_DEFAULT_MTREG	69

#define BH1750_CONVERSION_FACTOR	1.2

//typedef enum {
//	BH1750_OK		= 0,
//	BH1750_ERROR	= 1
//} void;

typedef enum
{
    // same as Power Down
    UNCONFIGURED = 0,
    // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_HIGH_RES_MODE = 0x10,
    // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
    // Measurement at 4 lux resolution. Measurement time is approx 16ms.
    CONTINUOUS_LOW_RES_MODE = 0x13,
    // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    ONE_TIME_HIGH_RES_MODE = 0x20,
    // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    ONE_TIME_HIGH_RES_MODE_2 = 0x21,
    // Measurement at 4 lux resolution. Measurement time is approx 16ms.
    ONE_TIME_LOW_RES_MODE = 0x23		
}bh1750_mode;

void BH1750_Init(bh1750_mode Mode);
void BH1750_Reset(void);
void BH1750_PowerState(uint8 PowerOn);
void BH1750_SetMtreg(uint8 Mtreg);
void BH1750_SetMode(bh1750_mode Mode);
void BH1750_TriggerManualConversion(void);
void BH1750_ReadLight(double *Result);
void waitMeasurementReady(bool maxWait);
#endif /* BH1750_H_ */