#include "uartcomm.h"

void uart_init(unsigned int ubrr) {
	UBRR0H = (unsigned char) (0xf & (ubrr >> 8));
	UBRR0L = (unsigned char) (0xff & ubrr);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = 3 << UCSZ0;
}

void put_char(char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void put_string(char *str) {
	while (*str) put_char(*str++);
}

char get_char(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}
