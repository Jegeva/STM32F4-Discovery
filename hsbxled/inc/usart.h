#ifndef __USART_H
#define __USART_H
#include "main.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

void usart_send_string(char *);
void usart_send_byte(char);
void usart_CRLF(void);
void usart_clrscrn(void);
void usart_init(void);

void usart_send_uint(uint32_t);



#endif
