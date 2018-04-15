#ifndef __UART_H
#define __UART_H

#include <sys.h>

#define USART_PRINT 3
#define EN_USART1_RX 0
#define EN_USART3_RX 1
#define USART_REV_LEN 1024

extern u16 USART_RX_STA;
extern u8 USART_RX_BUF[];

void uart1_init(u32 bound);
void uart3_init(u32 bound);

#endif
