# put your *.o targets here, make should handle the rest!

SRCS = stm32f4xx_it.c system_stm32f4xx.c usart.c main.c mic.c audio_output.c ucsx912.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main
FLASH_SIZE =1048576
# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

CFLAGS  = -g -Os -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

vpath %.c src

ROOT=$(shell pwd)

CFLAGS += -Iinc -I../STM32F4-Discovery_lib/inc -I../../CMSIS/Include/ -I PDM_filter
CFLAGS += -DARM_MATH_CM4 -lc -specs=nosys.specs

SRCS += ../STM32F4-Discovery_lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

B = main

###################################################

.PHONY: lib proj

all: lib dsp_lib pdm_filter proj

lib:
	$(MAKE) -C ../STM32F4-Discovery_lib/

dsp_lib:
	$(MAKE) -C ../../CMSIS/DSP_Lib/

pdm_filter:
	$(MAKE) -C PDM_filter

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -fdata-sections -ffunction-sections -L PDM_filter -L../STM32F4-Discovery_lib -L../../CMSIS/DSP_Lib/Lib/ -lstm32f4_discovery -lpdm_filter -larm_cortexM4lf_math -lm -Wl,--gc-sections
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(SIZE) $(PROJ_NAME).elf
	@perl -e 'printf "\nUsage : %0.2f%%\n",100*('`ls -l main.bin |cut -d' ' -f5`'/'$(FLASH_SIZE)');'

prog: 
	st-flash write $(PROJ_NAME).bin 0x8000000

gdb :
	openocd --file /usr/share/openocd/scripts/board/stm32f4discovery.cfg  &
	sleep 1
	echo "target extended localhost:3333\nmonitor reset halt\nmonitor reset_nag enable\nb "$(B)"\nset directories ../STM32F4-Discovery_lib/src\nc\n" >.gdbinit
	gdb-multiarch main.elf && killall openocd
gdbr :
	openocd --file /usr/share/openocd/scripts/board/stm32f4discovery.cfg  &
	sleep 1
	echo "target extended localhost:3333\nmonitor reset halt\nmonitor reset_nag enable\nb NMI_Handler\nb HardFault_Handler\nb MemManage_Handler\nb BusFault_Handler\nb UsageFault_Handler\nset directories ../STM32F4-Discovery_lib/src\nc\n" >.gdbinit
	gdb-multiarch main.elf && killall openocd
clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

allclean: clean
	$(MAKE) -C ../STM32F4-Discovery_lib/ clean
	$(MAKE) -C ../../CMSIS/DSP_Lib/ clean
	$(MAKE) -C ./PDM_filter clean
