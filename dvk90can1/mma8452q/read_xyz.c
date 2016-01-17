#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "uartcomm.h"

#ifndef F_CPU
#define F_CPU  8000000  // 8MHz
#endif

#define MMA8452Q_TWI_ADDRESS  0x3a  // 0x1d << 1 for r/w bit


int main(void)
{
    // Set all bits of PORT A for OUTPUT.
    // LEDs are on PORT A when using DVK90CAN1.
    DDRA  = 0xff;

    // Turn on LEDs in sequence to show progress.
    PORTA = 0b00000001;

    // send a power-on message to anyone listening on UART
    uart_init( _UBRR( F_CPU, 9600) );
    put_string( "[Bitfab Technologies LLC. EXAMPLE]\r\nProgram: read_xyz\r\nHardware: DVK90CAN1 + MMA8452Q (0x1D)\r\nLicense: GPL v3\r\n\0" );

    i2c_init();

    // start condition, device address, write mode
    unsigned char ret;
    ret = i2c_start( MMA8452Q_TWI_ADDRESS + I2C_WRITE );

    if ( ret ) {
        // failed to issue start condition, possibly no device found
        i2c_stop();
        // send an error message to anyone listening on UART
        put_string( "[ERROR] TWI: failed to issue start condition, possibly no device found.\r\n\0" );
        // flash all eight LEDs forever to indicate error
        for(;;) {
            PORTA = 0xff;
            _delay_ms(997);
            PORTA = 0x00;
            _delay_ms(997);
        }
        
    }
    else {
        put_string( "TWI: issued start condition.\r\n\0" );
        i2c_stop();       
    }

    for(;;);    
}
