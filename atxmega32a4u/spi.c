#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define PORT_SPI PORTC
#define DDR_SPI  PORTC.DIR
#define DD_MISO  PC6
#define DD_MOSI  PC5
#define DD_SCK   PC7

// SS1,2,3 are MPU9250 sensors, SS4 is uSDMMC card
#define DD_SS1   PA3
#define DD_SS2   PA2
#define DD_SS3   PA1
#define DD_SS4   PA0

void spi_init()
// initialize spi pins for communication
{
	PORTC.DIR = 0x50; // set MOSI and SCK as outputs, MISO as input
	PORTA.DIR = 0x0F; // port a bits 0,1,2,3 as SS pins, set to output

	// SPI Master, clock idle low, data setup on trailing edge,
  // data sampled on leading edge, double speed mode enabled
	SPIC.CTRL = 0xD0;

	SPIC.INTCTRL = 0x00; // ensure SPI interrupts as disabled

}
