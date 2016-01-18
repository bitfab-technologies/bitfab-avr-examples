#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "uartcomm.h"

#ifndef F_CPU
#define F_CPU  8000000  // 8MHz
#endif

#define L3G4200D_TWI_ADDRESS  0xd0  // 0x68 << 1 for r/w bit
#define REG_WHO_AM_I  0x0f
#define L3G4200D_DID  0xd3
#define REG_CTRL_REG1  0x20  // default value 0b00000111
#define REG_FIFO_CTL_REG  0x2e  // default value 0b00000000
#define REG_OUT_X_L  0x28
#define REG_OUT_X_H  0x29
#define REG_OUT_Y_L  0x2a
#define REG_OUT_Y_H  0x2b
#define REG_OUT_Z_L  0x2c
#define REG_OUT_Z_H  0x2d

static FILE uartout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);

void err_dwell() {
    // twiddle-flash all eight LEDs forever to indicate error
    for(;;) {
        PORTA = ~PORTA;
        _delay_ms(666);
    }
}

int main(void) {
    /*****************************************************************************
    ** First, we set up the plumbing for working with the DVK90CAN1. It's worth
    ** noting that the DVK90CAN1 has PORT_A hard-wired with 8 led's. Also, the
    ** AVR compiler will automagically link a `PORTA` variable in our code
    ** to the internal PORTA register. */
    stdout = &uartout;  // required for printf
    DDRA  = 0xff;  // Set all bits of PORT A for OUTPUT.
    PORTA = 0b00000001; // we will increment this 
    
    // send a power-on message to anyone listening on UART
    uart_init( _UBRR( F_CPU, 9600) );
    put_string( "[Bitfab Technologies LLC. EXAMPLE]\r\nProgram: read_xyz\r\nHardware: DVK90CAN1 + L3G4200D (0x1D)\r\nLicense: GPL v3\r\n\0" );

    

    /*****************************************************************************
    ** We'll kick off our i2c stuff here with an i2c init, and then we'll see if
    ** we can cammunicate with our gyro. :) */
    i2c_init();
    unsigned char ret; // we will use this for return values from our i2c commands

    // start condition, device address, write mode
    ret = i2c_start( L3G4200D_TWI_ADDRESS + I2C_WRITE );
    if ( ret ) {
        // failed to issue start condition, possibly no device found
        i2c_stop();
        // send an error message to anyone listening on UART
        put_string( "[ERROR] TWI: failed to issue start condition, possibly no device found.\r\n\0" );        
        err_dwell();
    }
    else {
        put_string( "TWI: issued start condition.\r\n\0" );
        PORTA += 1;  // increment progress LEDs.



        /*****************************************************************************
        ** Read the WHO_AM_I register and check if the returned data is as expected */
        // Write WHO_AM_I register to the bus, then... 
        ret = i2c_write( REG_WHO_AM_I );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the WHO_AM_I register.\r\n\0" );
            err_dwell();
        }
        // ... prepare to read a value ...
        ret = i2c_rep_start( L3G4200D_TWI_ADDRESS + I2C_READ );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for read.\r\n\0" );
            err_dwell();
        }
        // ... And then read the value. :)
        ret = i2c_readNak();
        // We are expecting a default value, if it isn't correct, we throw an error.
        if( L3G4200D_DID != ret ) {
            put_string( "[ERROR] TWI: failed to identify the L3G4200D, identifier value mismatch.\r\n\0" );
            err_dwell();
        }
        put_string( "TWI: identified L3G4200D.\r\n\0" );
        PORTA += 1;  // increment progress LEDs.



        /*****************************************************************************
        ** Now, let's read CTL_REG_1 register */
        ret = i2c_rep_start( L3G4200D_TWI_ADDRESS + I2C_WRITE );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for write.\r\n\0" );
            err_dwell();
        }
        ret = i2c_write( REG_CTRL_REG1 );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }
        ret = i2c_rep_start( L3G4200D_TWI_ADDRESS + I2C_READ );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for read.\r\n\0" );
            err_dwell();
        }
        ret = i2c_readNak();
        put_string( "TWI: retrieved the value of CTRL_REG1.\r\n\0" );
        unsigned char ctrl_reg1_value = ret;
        PORTA += 1;  // increment progress LEDs.



        /*****************************************************************************
        ** Now, let's try changing the value of REG_CTL_REG_1. There is a tiny bit of 
        ** magic in here, as we want to set the 4th bit of this register to a 1. By
        ** doing this, we are setting the device into `normal` mode, as opposed to
        ** the default `power down` mode */
        ret = i2c_rep_start( L3G4200D_TWI_ADDRESS + I2C_WRITE );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for write.\r\n\0" );
            err_dwell();
        }
        ret = i2c_write( REG_CTRL_REG1 );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }
        i2c_writeNak( ctrl_reg1_value | 8 ); // here is said magic
        PORTA += 1;  // increment progress LEDs.


        
        /*****************************************************************************
        ** I think we're done with setting up our gyro. Now, we should be able to sit
        ** in a data-read loop and constantly print our pitch, yaw, and roll data! */
        i2c_stop();
        put_string( "MCU: starting device data read, parse, and present loop.\r\n\0" );
        for(;;) {
            PORTA += 1;  // increment progress LEDs.
            
            unsigned char raw_data[6];

            unsigned char ret;
            ret = i2c_start( L3G4200D_TWI_ADDRESS + I2C_WRITE );
            if ( ret ) {
                // failed to issue start condition, possibly no device found
                i2c_stop();
                // send an error message to anyone listening on UART
                put_string( "[ERROR] TWI: failed to issue start condition, possibly no device found.\r\n\0" );                
                err_dwell();
            }
            else {
                put_string( "TWI: issued start condition.\r\n\0" );

                

                /*****************************************************************************
                ** Here, we can read contiguous registers because of an auto-increment feature
                */
                ret = i2c_write( REG_OUT_X_L );
                if( ret ) {
                    put_string( "[ERROR] TWI: failed to write the address of the REG_OUT_X_L register.\r\n\0" );
                    err_dwell();
                }
                ret = i2c_rep_start( L3G4200D_TWI_ADDRESS + I2C_READ );
                if( ret ) {
                    put_string( "[ERROR] TWI: failed to issue repeat start for read.\r\n\0" );
                    err_dwell();
                }
                raw_data[0] = i2c_readAck();
                raw_data[1] = i2c_readAck();
                raw_data[2] = i2c_readAck();
                raw_data[3] = i2c_readAck();
                raw_data[4] = i2c_readAck();
                raw_data[5] = i2c_readNak(); // last byte read issues NAK

                i2c_stop();
            }



            /*****************************************************************************
            ** Let's combine these six 8-bit values into three 16-bit 2's compliment */
            int16_t pitch_yaw_roll[3] = {0, 0, 0};  // x, y, z
            uint8_t i;
            for(i = 0; i < 3 ; ++i)
            {
                // combine the two 8 bit registers
                int16_t value = (8 << raw_data[i*2]) | raw_data[(i*2)+1];
                // If the value is 2's compliment
                if ( value >> 15 ) {
                    value = ~value + 1; // let's make it nice to look at
                }
                pitch_yaw_roll[i] = value;
            }
            // print the data over serial for humanoids
            printf( "pitch_yaw_roll = (%d, %d, %d)\r\n", pitch_yaw_roll[0], pitch_yaw_roll[1], pitch_yaw_roll[2] );
            _delay_ms(997);
        }
    }

    for(;;);    
}
