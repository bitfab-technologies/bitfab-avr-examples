#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU  32000000  // 32MHz
#endif

void err_dwell() {
    // twiddle-flash all eight LEDs forever to indicate error
    for(;;) {
        PORTB.OUT = ~PORTB.OUT;
        _delay_ms(997);
    }
}

int main(void) {
    // Set all bits of PORT A for OUTPUT.
    // LEDs are on PORT A when using DVK90CAN1.
    PORTB.DIR  = 0xff;
    err_dwell();
}
