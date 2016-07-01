#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU  32000000  // 32MHz
#endif

void clock_setup();
void io_setup();
void setup();
void spi_setup();

volatile int SPI_SS1 = PORTA.PIN3_bm; // MPU9250 on SS1,2,3
volatile int SPI_SS2 = PORTA.PIN2_bm;
volatile int SPI_SS3 = PORTA.PIN1_bm;
volatile int SPI_SS4 = PORTA.PIN0_bm; // SDMMC on SS4

int main(void) {
	setup();
	for(;;) {
		PORTB.OUT = ~PORTB.OUT;
		_delay_ms(1000);
	}
}

void setup() {
	io_setup();
	clock_setup();
	spi_setup();
}

void clock_setup() {
	//configure clock to 32MHz
	OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;
	while (!(OSC.STATUS & OSC_RC32KRDY_bm));
	while (!(OSC.STATUS & OSC_RC32MRDY_bm));
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	OSC.CTRL &= ~OSC_RC2MEN_bm;
}

void io_setup() {
	PORTB.OUT = 0x00;
	PORTB.DIR = 0x0F;
	//PORTB.PIN0CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc);
	//PORTB.PIN1CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc);
	//PORTB.PIN2CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc);
	//PORTB.PIN3CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc);
	PORTB.INT0MASK = 0x0;
	PORTB.INT1MASK = 0x0;
}

void spi_setup() {
	SPIC.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_bm | SPI_PRESCALER0_bm;
}
