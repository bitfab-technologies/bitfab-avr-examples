
###############################################################################
# Makefile for the project at90can128::read_xyz
###############################################################################

## General Flags
PROJECT = read_xyz
MCU = at90can128
F_CPU = 8000000
TARGET = $(PROJECT).elf
CC = avr-gcc

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2  -O0 -funsigned-char -funsigned-bitfields
CFLAGS += -DF_CPU=$(F_CPU)
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=$(PROJECT).map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"../" -I"../../" -I"../../common/libraries/include/uartcomm" -I"../../common/libraries/include/i2cmaster"

## Objects that must be built in order to link
OBJECTS = $(PROJECT).o uartcomm.o i2cmaster.o

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) $(PROJECT).hex $(PROJECT).eep $(PROJECT).lss size

## Compile
$(PROJECT).o: $(PROJECT).c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uartcomm.o: ../../common/libraries/src/uartcomm/uartcomm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2cmaster.o: ../../common/libraries/src/i2cmaster/twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -o i2cmaster.o -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	rm -rf $(OBJECTS) $(PROJECT).elf dep $(PROJECT).hex $(PROJECT).eep $(PROJECT).lss $(PROJECT).map

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

