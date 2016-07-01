#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU  32000000  // 32MHz
#endif


int main(void) {
    PORTB.DIR  = 0xff;
    for(;;) { PORTB.OUT++; _delay_ms(PORTB.OUT>>1); }
}
