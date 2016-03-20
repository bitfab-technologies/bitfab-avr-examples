

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "uartcomm.h"

#ifndef F_CPU
#define F_CPU  8000000  // 8MHz
#endif

// Accelerometer, MMA8452Q
#define MMA8452Q_TWI_ADDRESS  0x3a  // 0x1d << 1 for r/w bit
#define MMA8452Q_REG_WHO_AM_I  0x0d
#define MMA8452Q_DID  0x2a
#define MMA8452Q_REG_CTRL_REG1  0x2a
#define MMA8452Q_REG_XYZ_DATA_CFG  0x0e
#define MMA8452Q_REG_OUT_X_MSB  0x01

// Gyroscope, L3G4200D
#define L3G4200D_TWI_ADDRESS  0xd0  // 0x68 << 1 for r/w bit
#define L3G4200D_REG_WHO_AM_I  0x0f
#define L3G4200D_DID  0xd3
#define L3G4200D_REG_CTRL_REG1  0x20  // default value 0b00000111
#define L3G4200D_REG_FIFO_CTL_REG  0x2e  // default value 0b00000000
#define L3G4200D_REG_OUT_X_L  0x28
#define L3G4200D_REG_OUT_X_H  0x29
#define L3G4200D_REG_OUT_Y_L  0x2a
#define L3G4200D_REG_OUT_Y_H  0x2b
#define L3G4200D_REG_OUT_Z_L  0x2c
#define L3G4200D_REG_OUT_Z_H  0x2d

// Magnetometer, HMC5983
#define HMC5983_TWI_ADDRESS  0x3c
#define HMC5983_REG_ID_A  0x0a
#define HMC5983_REG_ID_B  0x0b
#define HMC5983_REG_ID_C  0x0c
#define HMC5983_DID_A  0x48  // 'H'
#define HMC5983_DID_B  0x34  // '4'
#define HMC5983_DID_C  0x33  // '3'

// Altimeter, MPL3115A2
#define MPL3115A2_TWI_ADDRESS  0xc0
#define MPL3115A2_REG_WHO_AM_I  0x0c
#define MPL3115A2_DID  0xc4
#define MPL3115A2_REG_CTRL_REG1  0x26
#define MPL3115A2_REG_PT_DATA_CFG  0x13
#define MPL3115A2_REG_OUT_P_MSB  0x01

#define STANDARD_PRESSURE 101325 //kPa

#define FULL_SCALE_RANGE_G  2  // 2g, 4g, 8g

static FILE uartout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);

typedef enum { false, true } bool;

typedef enum {

	kUnknown,
	kStart,
	kReset,
	kPauseAndRetryCurrent,
	kSelfTest,
	kTestAccelerometer,
	kReadAccelerometer,
	kTestGyroscope,
	kReadGyroscope,
	kTestMagnetometer,
	kReadMagnetometer,
	kTestAltimeter,
	kReadAltimeter,
	kPrintCurrentData,
	kFinish

} MasterState;

typedef enum {

	kAccelerometer,
	kGyroscope,
	kMagnetometer,
	kAltimeter

} Subsystem;

#define SUBSYSTEM_COUNT 4

void err_dwell() {
    // twiddle-flash all eight LEDs forever to indicate error
    for(;;) {
        PORTA = ~PORTA;
        _delay_ms(997);
    }
}

void success_dwell() {
	// scroll a single LED forever to indicate success
	PORTA = 0x01;
	for(;;) {

		if(0x80 == PORTA) {
			PORTA = 0x01;
		}
		else {
			PORTA = PORTA << 1;
		}
		_delay_ms(98);

	}
}

int main(void) {
    stdout = &uartout;  // required for printf
    uart_init( _UBRR( F_CPU, 9600) );
    i2c_init();

    // Set all bits of PORT A for OUTPUT.
    // LEDs are on PORT A when using DVK90CAN1.
    DDRA  = 0xff;

    // Turn on LEDs in sequence to show progress.
    PORTA = 0b00000001;

    // FSM
    MasterState previousState = kUnknown;
    MasterState currentState = kUnknown;
    MasterState nextState = kStart;

    // System Data
    bool subsystemPassedTest[SUBSYSTEM_COUNT];

    int i;
    for(i=0; i < SUBSYSTEM_COUNT; ++i)
    {
    	subsystemPassedTest[i] = false;
    }
    unsigned char twiReturnValue = 0;
    unsigned int pauseDuration = 1000;  // ms
    unsigned int retryCounter = 0;

    for(;;)
    {
    	previousState = currentState;
    	currentState = nextState;
    	nextState = kUnknown;

    	PORTA += 1;

    	switch( currentState )
    	{
    		case kUnknown:
    		{
    			printf( "[ERROR]: Master state machine traversed into the unknown state! Previous: %d, Next: %d\n", previousState, nextState );
    			err_dwell();
    		}
    		break;

    		case kStart:
    		{
			    // send a power-on message to anyone listening on UART
			    printf( "[Bitfab Technologies LLC. EXAMPLE]\r\nProgram: imu_serial_stream\r\nHardware: BitFab Lief, R1\r\nLicense: GPL v3\r\n" );

			    nextState = kSelfTest;
			}
			break;

			case kReset:
			{
				previousState = kUnknown;
			    currentState = kUnknown;
			    nextState = kStart;
			}
			break;

			case kPauseAndRetryCurrent:
			{
				++retryCounter;
				_delay_ms(pauseDuration);

				nextState = previousState;
			}
			break;

			case kSelfTest:
			{
				if( pauseDuration < 1000 ) {
					pauseDuration = 1000;
				}

				int i;
				for(i=0; i < SUBSYSTEM_COUNT; ++i)
				{
					if( ! subsystemPassedTest[i] ) 
					{
						// we need to test the subsystem (again)
						switch( i )
						{
							case kAccelerometer:
							{
								// setup
								twiReturnValue = 0;

								// test
								twiReturnValue |= i2c_start( MMA8452Q_TWI_ADDRESS + I2C_WRITE );

								twiReturnValue |= i2c_write( MMA8452Q_REG_WHO_AM_I );
								twiReturnValue |= i2c_rep_start( MMA8452Q_TWI_ADDRESS + I2C_READ );

								if( 0 == twiReturnValue ) {
									twiReturnValue = ( MMA8452Q_DID == i2c_readNak() ) ? 0 : twiReturnValue;
								}

								i2c_stop();

								// handle issue(s) / error(s)
								if( twiReturnValue ) {
									// fail
									printf( "Accelerometer\t\t\t\t[FAIL]\n" );
									subsystemPassedTest[kAccelerometer] = false;
									break;
								}
								else {
									// pass
									printf( "Accelerometer\t\t\t\t[PASS]\n" );
									subsystemPassedTest[kAccelerometer] = true;
								}
							}
							break;

							case kGyroscope:
							{
								// setup
								twiReturnValue = 0;

								// test
								twiReturnValue |= i2c_start( L3G4200D_TWI_ADDRESS + I2C_WRITE );

								twiReturnValue |= i2c_write( L3G4200D_REG_WHO_AM_I );
								twiReturnValue |= i2c_rep_start( L3G4200D_TWI_ADDRESS + I2C_READ );

								if( 0 == twiReturnValue ) {
									twiReturnValue = ( L3G4200D_DID == i2c_readNak() ) ? 0 : twiReturnValue;
								}

								i2c_stop();

								// handle issue(s) / error(s)
								if( twiReturnValue ) {
									// fail
									printf( "Gyroscope\t\t\t\t[FAIL]\n" );
									subsystemPassedTest[kGyroscope] = false;
									break;
								}
								else {
									// pass
									printf( "Gyroscope\t\t\t\t[PASS]\n" );
									subsystemPassedTest[kGyroscope] = true;
								}
							}
							break;

							case kMagnetometer:
							{
								// setup
								twiReturnValue = 0;

								// test
								twiReturnValue |= i2c_start( HMC5983_TWI_ADDRESS + I2C_WRITE );

								twiReturnValue |= i2c_write( HMC5983_REG_ID_A );
								twiReturnValue |= i2c_rep_start( HMC5983_TWI_ADDRESS + I2C_READ );

								if( 0 == twiReturnValue ) {
									twiReturnValue = ( HMC5983_DID_A == i2c_readAck() ) ? 0 : twiReturnValue;
								}
								if( 0 == twiReturnValue ) {
									twiReturnValue = ( HMC5983_DID_B == i2c_readAck() ) ? 0 : twiReturnValue;
								}
								if( 0 == twiReturnValue ) {
									twiReturnValue = ( HMC5983_DID_C == i2c_readNak() ) ? 0 : twiReturnValue;
								}

								i2c_stop();

								// handle issue(s) / error(s)
								if( twiReturnValue ) {
									// fail
									printf( "Magnetometer\t\t\t\t[FAIL]\n" );
									subsystemPassedTest[kMagnetometer] = false;
									break;
								}
								else {
									// pass
									printf( "Magnetometer\t\t\t\t[PASS]\n" );
									subsystemPassedTest[kMagnetometer] = true;
								}
							}
							break;

							case kAltimeter:
							{
								// setup
								twiReturnValue = 0;

								// test
								twiReturnValue |= i2c_start( MPL3115A2_TWI_ADDRESS + I2C_WRITE );

								twiReturnValue |= i2c_write( MPL3115A2_REG_WHO_AM_I );
								twiReturnValue |= i2c_rep_start( MPL3115A2_TWI_ADDRESS + I2C_READ );

								if( 0 == twiReturnValue ) {
									twiReturnValue = ( MPL3115A2_DID == i2c_readNak() ) ? 0 : twiReturnValue;
								}

								i2c_stop();

								// handle issue(s) / error(s)
								if( twiReturnValue ) {
									// fail
									printf( "Altimeter\t\t\t\t[FAIL]\n" );
									subsystemPassedTest[kAltimeter] = false;
									break;
								}
								else {
									// pass
									printf( "Altimeter\t\t\t\t[PASS]\n" );
									subsystemPassedTest[kAltimeter] = true;
								}
							}
							break;
						}
					}
				}

				nextState = kFinish;
				for(i=0; i < SUBSYSTEM_COUNT; ++i)
				{
					if( ! subsystemPassedTest[i] ) nextState = kPauseAndRetryCurrent;
				}
			}
			break;

			case kFinish:
			{
				success_dwell();
			}

		}
    }

    for(;;);    
}
