#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "uartcomm.h"

#ifndef F_CPU
#define F_CPU  8000000  // 8MHz
#endif

#define MPL3115a2_TWI_ADDRESS  0xC0
#define REG_WHO_AM_I  0x0C
#define MPL3115a2_DID  0xC4
#define REG_CTRL_REG1  0x26
#define REG_PT_DATA_CFG  0x13
#define REG_OUT_P_MSB  0x01


static FILE uartout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);

void err_dwell() {
    // twiddle-flash all eight LEDs forever to indicate error
    for(;;) {
        PORTA = ~PORTA;
        _delay_ms(997);
    }
}




int main(void) {
    stdout = &uartout;  // required for printf

    // Set all bits of PORT A for OUTPUT.
    // LEDs are on PORT A when using DVK90CAN1.
    DDRA  = 0xff;

    // Turn on LEDs in sequence to show progress.
    PORTA = 0b00000001;

    // send a power-on message to anyone listening on UART
    uart_init( _UBRR( F_CPU, 9600) );
    put_string( "[Bitfab Technologies LLC. EXAMPLE]\r\nProgram: read_xyz\r\nHardware: DVK90CAN1 + MPL3115a2 (0xC0)\r\nLicense: GPL v3\r\n\0" );

    i2c_init();


    // start condition, device address, write mode
    unsigned char ret;
    ret = i2c_start( MPL3115a2_TWI_ADDRESS + I2C_WRITE );

    #define STANDARD_PRESSURE 101325 // kPa at STP


    if ( ret ) {
        // failed to issue start condition, possibly no device found
        i2c_stop();
        // send an error message to anyone listening on UART
        put_string( "[ERROR] TWI: failed to issue start condition, possibly no device found.\r\n\0" );
        
        err_dwell();
    }
    else {
        put_string( "TWI: issued start condition.\r\n\0" );

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_write( REG_WHO_AM_I );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the WHO_AM_I register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_READ );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for read.\r\n\0" );
            err_dwell();
        }
        
        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_readNak();
        if( MPL3115a2_DID != ret ) {
            put_string( "[ERROR] TWI: failed to identify the MPL3115a2, identifier value mismatch.\r\n\0" );
            err_dwell();
        }

        put_string( "TWI: identified MPL3115a2.\r\n\0" );

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_WRITE );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for write.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_write( REG_CTRL_REG1 );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }
        
        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_READ );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for read.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_readNak();
        put_string( "TWI: retrieved the value of CTRL_REG1.\r\n\0" );


        // clear the active bit, set to ALT mode
        unsigned char ctrl_reg1_value = ret & ~(0x01) | (0x80) ;


        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_WRITE );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for write.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_write( REG_CTRL_REG1 );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        put_string( "MCU: putting device into standby mode to accept configuration changes.\r\n\0" );
        ret = i2c_write( ctrl_reg1_value );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write new value of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        put_string( "MCU: attempting to write configuration changes to device.\r\n\0" );
        ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_WRITE );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for write.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_write( REG_PT_DATA_CFG );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the REG_XYZ_DATA_CFG register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        // enable altitude and temp measurement
        unsigned char PT_flag_enable = 0x07;
        ret = i2c_write( PT_flag_enable );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the new value of the REG_XYZ_DATA_CFG register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_WRITE );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to issue repeat start for write.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        ret = i2c_write( REG_CTRL_REG1 );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write the address of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        // set the active bit
        ctrl_reg1_value = ret | 0x01;

        put_string( "MCU: putting device into active mode.\r\n\0" );
        ret = i2c_write( ctrl_reg1_value );
        if( ret ) {
            put_string( "[ERROR] TWI: failed to write new value of the CTRL_REG1 register.\r\n\0" );
            err_dwell();
        }

        // increment progress LEDs.
        PORTA += 1;

        // relinquish the bus / device to allow others to read data, as well.
        i2c_stop();

        put_string( "MCU: starting device data read, parse, and present loop.\r\n\0" );

        // data read loop
        for(;;) {
            // increment progress LEDs.
            PORTA += 1;

            unsigned char raw_data[6];

            // start condition, device address, write mode
            unsigned char ret;
            ret = i2c_start( MPL3115a2_TWI_ADDRESS + I2C_WRITE );

            if ( ret ) {
                // failed to issue start condition, possibly no device found
                i2c_stop();
                // send an error message to anyone listening on UART
                put_string( "[ERROR] TWI: failed to issue start condition, possibly no device found.\r\n\0" );
                
                err_dwell();
            }
            else {
                put_string( "TWI: issued start condition.\r\n\0" );

                ret = i2c_write( REG_OUT_P_MSB );
                if( ret ) {
                    put_string( "[ERROR] TWI: failed to write the address of the OUT_X_MSB register.\r\n\0" );
                    err_dwell();
                }

                ret = i2c_rep_start( MPL3115a2_TWI_ADDRESS + I2C_READ );
                if( ret ) {
                    put_string( "[ERROR] TWI: failed to issue repeat start for read.\r\n\0" );
                    err_dwell();
                }

                // multibyte read of contiguous data registers from device
                // this takes advantage of built-in read address auto-increment
                raw_data[0] = i2c_readAck();
                raw_data[1] = i2c_readAck();
                raw_data[2] = i2c_readAck();
                raw_data[3] = i2c_readAck();
                raw_data[4] = i2c_readAck();
                // last byte read issues NAK
                raw_data[5] = i2c_readNak();

                i2c_stop();
            }

            int16_t accel3[3] = {0, 0, 0};  // x, y, z

            // Loop to calculate 12-bit ADC and g value for each axis
            uint8_t i;
            for(i = 0; i < 3 ; ++i)
            {
                // combine the two 8 bit registers into one 12-bit number
                int16_t accel_value = (8 << raw_data[i*2]) | raw_data[(i*2)+1];
                // the two registers represent a right-aligned 12-bit value
                // therefore, shift right appropriately in this case
                accel_value >>= 4;

                // determine if the value would have been negative if it
                // were actually 16 bit instead of 12 bit
                if (0x7F < raw_data[i*2])
                {  
                  accel_value -= 0x1000;
                }

                accel3[i] = accel_value;
            }

            // print the data over serial for humanoids
            printf( "accel3 = (%d, %d, %d)\r\n", accel3[0], accel3[1], accel3[2] );

            _delay_ms(997);
        }
    }

    for(;;);    
}
