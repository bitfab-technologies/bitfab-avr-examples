#ifndef _UARTCOMM_H
#define _UARTCOMM_H

#include <avr/io.h>

#define _UBRR(F_CPU, BAUD)  (F_CPU/16/BAUD-1)

void uart_init(unsigned int ubrr);
void put_char(char data);
void put_string(char *str);
char get_char(void);

#endif
