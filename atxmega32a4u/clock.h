#include <avr/io.h>

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
