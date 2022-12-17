#include "ZComDef.h"
#include "onboard.h"
#include "uart.h"
#ifdef DO_DEBUG_UART
void UART_Init(void)
{
  P0SEL |= BV(2)|BV(3);
  U0CSR |= (1<<7);
  
  U0GCR = 11;	// 115200 Baud
  U0BAUD = 216;	
}

void UART_Transmit(char data)
{
  U0DBUF = data;
  while (U0CSR & (1<<0)); 
}

void UART_String(const char *s)
{
  while (*s)
  {
    UART_Transmit(*s++);
  }
  UART_Transmit('\r');
  UART_Transmit('\n');
}
#elseif
void UART_Init(void){return;};
void UART_Transmit(char data){return;}
void UART_String(const char *s){return;};
#endif