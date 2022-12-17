
#include "delay.h"

void _delay_us(uint32 microSecs)
{
//  while(microSecs--)
//  {
//    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
//  }
  MicroWait(microSecs);
}

void _delay_ms(uint32 milliSecs)
{
//  while(milliSecs--)
//  {
//    _delay_us(1000);
//  }
  MicroWait(milliSecs*1000);
}