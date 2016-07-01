#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU  32000000  // 32MHz
#endif

void clock_setup();
void io_setup();
void dac_setup();
void setup();

int main(void) {
	int bright = 0;
	setup();
	for(;;) {
		DACB_CH0DATA = (bright++)&0xFFF;
		DACB_CH1DATA = ~bright&0xFFF;
		_delay_us(10);
		PORTB.OUT=~PORTB.OUT;
	}
}

void setup() {
	clock_setup();
	io_setup();
	dac_setup();
}

void clock_setup() {
	OSC_CTRL |= OSC_RC32KEN_bm;
	while (!(OSC_STATUS & OSC_RC32KRDY_bm));
	OSC_CTRL |= OSC_RC32MEN_bm;
	CPU_CCP = CCP_IOREG_gc;
	CLK_PSCTRL &= ~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm);
	CLK_PSCTRL |= CLK_PSADIV_1_gc | CLK_PSBCDIV_2_2_gc;
	//OSC_DFLLCTRL &= ~(OSC_RC32MENCREF_gm | OSC_RC2MCREF_bm);
	//OSC_DFLLCTRL |= OSC_RC32MCREF_RC32K_gc;

	DFLLRC32M_CTRL |= DFLL_ENABLE_bm;
	while (!(OSC_STATUS & OSC_RC32MRDY_bm));
	CPU_CCP = CCP_IOREG_gc;
	CLK_CTRL &= ~CLK_SCLKSEL_gm;
	CLK_CTRL |= CLK_SCLKSEL_RC32M_gc;
	OSC_CTRL &= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);
	PORTCFG_CLKEVOUT = 0x00;
}

void io_setup() {
	PORTB.OUT = 0x00;
	PORTB.DIR = 0x0C;
	PORTB.PIN0CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc);
	PORTB.PIN1CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc);
	PORTB.PIN2CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc);
	PORTB.PIN3CTRL = (PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc);
	PORTB.INT0MASK = 0x0;
	PORTB.INT1MASK = 0x0;
}

void dac_setup() {
	DACB.CTRLA &= ~(DAC_IDOEN_bm | DAC_CH0EN_bm | DAC_CH1EN_bm);
	DACB.CTRLA |= (DAC_CH0EN_bm | DAC_CH1EN_bm | DAC_ENABLE_bm);

	DACB.CTRLB &= ~(DAC_CHSEL_gm | DAC_CH0TRIG_bm | DAC_CH1TRIG_bm);
	DACB.CTRLB |= DAC_CHSEL_DUAL_gc;

	DACB.CTRLC &= ~(DAC_REFSEL_gm | DAC_LEFTADJ_bm);
	DACB.CTRLC |= DAC_REFSEL_AVCC_gc;
}
