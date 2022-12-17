//==============================================================================
// S E N S I R I O N AG, Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project : SHT2x Sample Code (V1.2)
// File : SHT2x.c
// Author : MST
// Controller: NEC V850/SG3 (uPD70F3740)
// Compiler : IAR compiler for V850 (3.50A)
// Brief : Sensor layer. Functions for sensor access
//==============================================================================
//---------- Includes ----------------------------------------------------------
#include "sht20.h"
static uint16 POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001
//==============================================================================
uint8 SHT2x_CheckCrc(uint8 data[], uint8 nbrOfBytes, uint8 checksum)
//==============================================================================
{
  uint8 crc = 0;
  uint8 byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { 
    crc ^= (data[byteCtr]);
    for (uint8 bit = 8; bit > 0; --bit)
    { 
      if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) 
    return CHECKSUM_ERROR;
  else return 0;
}
//===========================================================================
uint8 SHT2x_ReadUserRegister(uint8 *pRegisterValue)
//===========================================================================
{
  uint8 checksum; //variable for checksum byte
  uint8 error=0; //variable for error code
  uint8 data[2];
  char buff[100];
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W);
//  error |= I2c_WriteByte (USER_REG_R);
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R);
//  *pRegisterValue = I2c_ReadByte(ACK);
//  checksum=I2c_ReadByte(NO_ACK);
//  error |= I2C_ReadMultByte(I2C_ADR, USER_REG_R, data, 2);
  HalI2CStart(I2C_ADR_W);
  HalI2CSendByte(USER_REG_R);
  HalI2CReceive(I2C_ADR_R, data, 2);
  *pRegisterValue = data[0];
  checksum = data[1];
  error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
  if(error != 0)
  {  
    sprintf(buff, "0: %d", data[0]);
    UART_String(buff);
    sprintf(buff, "1: %d", data[1]);
    UART_String(buff);
  }
  //I2c_StopCondition();
  return error;
}
//===========================================================================
uint8 SHT2x_WriteUserRegister(uint8 *pRegisterValue)
//===========================================================================
{
  uint8 error=0; //variable for error code
// I2c_StartCondition();
// error |= I2c_WriteByte (I2C_ADR_W);
// error |= I2c_WriteByte (USER_REG_W);
// error |= I2c_WriteByte (*pRegisterValue);
// I2c_StopCondition();

  error |= I2C_WriteMultByte(I2C_ADR, USER_REG_W, (uint8 []){*pRegisterValue}, 1);
  return error;
}
////===========================================================================
//uint8 SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, int16 *pMeasurand)
////===========================================================================
//{
// uint8 checksum; //checksum
// uint8 data[2]; //data array for checksum verification
// uint8 error=0; //error variable
// uint16 i; //counting variable
// //-- write I2C sensor address and command --
// I2c_StartCondition();
// error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
// switch(eSHT2xMeasureType)
// { case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM); break;
// case TEMP : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM); break;
// default: break;
// }
// //-- wait until hold master is released --
// I2c_StartCondition();
// error |= I2c_WriteByte (I2C_ADR_R);
// SCL=HIGH; // set SCL I/O port as input
// for(i=0; i<1000; i++) // wait until master hold is released or
// { _delay_us(1000); // a timeout (~1s) is reached
// if (SCL_CONF==1) break;
// }
// //-- check for timeout --
// if(SCL_CONF==0) error |= TIME_OUT_ERROR;
// //-- read two data bytes and one checksum byte --
// pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
// pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
// checksum=I2c_ReadByte(NO_ACK);
// //-- verify checksum --
// error |= SHT2x_CheckCrc (data,2,checksum);
// I2c_StopCondition();
// return error;
//}
//===========================================================================
uint8 SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  uint8 checksum; //checksum
  uint8 data[2]; //data array for checksum verification
  uint8 buffer[3];
  uint8 error=0; //error variable
  char debugBuff[100];
//  uint8 REG;
  uint8 i = 0;
//-- write I2C sensor address and command --
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  
  HalI2CStart(I2C_ADR_W);
  
  
  switch(eSHT2xMeasureType)
  { 
    case HUMI: 
//      error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_POLL);
      HalI2CSendByteLoop(TRIG_RH_MEASUREMENT_POLL);
//      error |= I2C_ReadMultByte(I2C_ADR, TRIG_RH_MEASUREMENT_POLL, buffer, 3);
//      REG = TRIG_RH_MEASUREMENT_POLL;
      break;
    case TEMP : 
//      error |= I2c_WriteByte (TRIG_T_MEASUREMENT_POLL); 
      HalI2CSendByteLoop(TRIG_T_MEASUREMENT_POLL);
//      error |= I2C_ReadMultByte(I2C_ADR, TRIG_T_MEASUREMENT_POLL, buffer, 3);
//      REG = TRIG_T_MEASUREMENT_POLL;
      break;
    default: 
      break;
  }
//  HalI2CReceive(I2C_ADR_R, buffer, 3);
  
  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
  do
  { 
    HalI2CStartCondition();
    _delay_us(10000); //delay 10ms
    if(i++ >= 20) 
      break;
  } while(HalI2CSendByte( I2C_ADR_R ) == 0);
  HalI2CRead(buffer, 3);
//  if (i>=20) 
//    error |= TIME_OUT_ERROR;
  //-- read two data bytes and one checksum byte --
  pMeasurand->s16.u8H = data[0] = buffer[0];
  pMeasurand->s16.u8L = data[1] = buffer[1];
  checksum = buffer[2];
  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  if(error != 0)
  {  
    sprintf(debugBuff, "0x: %d", buffer[0]);
    UART_String(debugBuff);
    sprintf(debugBuff, "1x: %d", buffer[1]);
    UART_String(debugBuff);
    sprintf(debugBuff, "2x: %d", buffer[2]);
    UART_String(debugBuff);
  }
  HalI2CStop();
//  I2c_StopCondition();
  return error;
}
//===========================================================================
uint8 SHT2x_SoftReset()
//===========================================================================
{
  uint8 error=0; //error variable
  // I2c_StartCondition();
  // error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  // error |= I2c_WriteByte (SOFT_RESET); // Command
  // I2c_StopCondition();
  //error |= I2C_WriteMultByte(I2C_ADR, SOFT_RESET, NULL, 0);
  HalI2CSend(I2C_ADR_W, (uint8 []){SOFT_RESET},1);
  _delay_us(15000); // wait till sensor has restarted
  return error;
}
//==============================================================================
float SHT2x_CalcRH(uint16 u16sRH)
//==============================================================================
{
  float humidityRH; // variable for result
  u16sRH &= ~0x0003; // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --
  humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}
//==============================================================================
float SHT2x_CalcTemperatureC(uint16 u16sT)
//==============================================================================
{
  float temperatureC; // variable for result
  u16sT &= ~0x0003; // clear bits [1..0] (status bits)
  //-- calculate temperature [°C] --
  temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}
//==============================================================================
uint8 SHT2x_GetSerialNumber(uint8 u8SerialNumber[])
//==============================================================================
{
  uint8 error=0; //error variable
  uint8 data[7];
  //Read from memory location 1
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); //I2C address
//  error |= I2c_WriteByte (0xFA); //Command for readout on-chip memory
//  error |= I2c_WriteByte (0x0F); //on-chip memory address
  HalI2CStart(I2C_ADR_W);
  HalI2CSendByte(0xFA);
  HalI2CSendByte(0x0F);
  HalI2CStop();
  
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R); //I2C address
//  u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
//  I2c_ReadByte(ACK); //Read CRC SNB_3 (CRC is not analyzed)
//  u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
//  I2c_ReadByte(ACK); //Read CRC SNB_2 (CRC is not analyzed)
//  u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
//  I2c_ReadByte(ACK); //Read CRC SNB_1 (CRC is not analyzed)
//  u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
//  I2c_ReadByte(NO_ACK); //Read CRC SNB_0 (CRC is not analyzed)
//  I2c_StopCondition();
  HalI2CReceive(I2C_ADR_R, data, 5); 
  u8SerialNumber[5] = data[0];
  u8SerialNumber[4] = data[2];
  u8SerialNumber[3] = data[4];
  u8SerialNumber[2] = data[6];
  
  //Read from memory location 2
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); //I2C address
//  error |= I2c_WriteByte (0xFC); //Command for readout on-chip memory
//  error |= I2c_WriteByte (0xC9); //on-chip memory address
  HalI2CStart(I2C_ADR_W);
  HalI2CSendByte(0xFC);
  HalI2CSendByte(0xC9);
  HalI2CStop();
  
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R); //I2C address
//  u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
//  u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
//  I2c_ReadByte(ACK); //Read CRC SNC0/1 (CRC is not analyzed)
//  u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
//  u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
//  I2c_ReadByte(NO_ACK); //Read CRC SNA0/1 (CRC is not analyzed)
//  I2c_StopCondition();
  HalI2CReceive(I2C_ADR_R, data, 5); 
  u8SerialNumber[1] = data[0];
  u8SerialNumber[0] = data[1];
  u8SerialNumber[7] = data[3];
  u8SerialNumber[6] = data[4];
  return error;
}
