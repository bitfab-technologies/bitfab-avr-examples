#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU  32000000  // 32MHz
#endif

#include "clock.h"
#include "spi.h"
#include "MPU9250.h"
#include "sdmmc.h"
#include "ui.h"

#define SPI_PORT PORTC
#define SS1 PA3
#define SS2 PA2
#define SS3 PA1
#define SS4 PA0

#define MPU_DATA_gc (MPU_GYRO_EN_bm | MPU_ACC_EN_bm | MPU_MAG_EN_bm)

void setup();

void main()
{
  setup();

  while(1)
  {
    if (mpu1.exist)
    { mpu1_data = mpu1.get_data(MPU_DATA_gc); }
    
    if (mpu2.exist)
    { mpu2_data = mpu2.get_data(MPU_DATA_gc); }
    
    if (mpu3.exist)
    { mpu3_data = mpu3.get_data(MPU_DATA_gc); }
    
    if (sdmmc.exist)
    {
      sdmmc.log_data(millis(), mpu1_data);
      sdmmc.log_data(millis(), mpu2_data);
      sdmmc.log_data(millis(), mpu3_data);
    }
  }
}

void setup()
{
  clock_setup(); //32MHz
  io_setup();
  spi_init(&SPI_PORT, SPI_BAUD_1MHz_gc);

  mpu1 =  mpu9250_init(SS1);
  mpu2 =  mpu9250_init(SS2);
  mpu3 =  mpu9250_init(SS3);
  sdmmc = sdmmc_init(SS4, SDMMC_NEW_FILE_EVERY_RESET_bm);
}

void io_setup() {
  PORTB.OUT = 0x0;
  PORTB.DIR = 0xF;
  PORTB.INT0MASK = 0x0;
  PORTB.INT1MASK = 0x0;
  PORTR.DIR = 0x0;
}