# Makefile for release build.
# Uses gcc-arm-none-eabi 9-2020-q2-update, available from:
# https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2?revision=05382cca-1721-44e1-ae19-1e7c3dc96118&rev=05382cca172144e1ae191e7c3dc96118&hash=FDE675133A099796BD1507A3FF215AC4

CC = ../gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc
STARTUP_FILE = startup/startup_stm32.s
NAME = decibel_meter

#############################################

SOURCES = $(shell find . -type d \( -path ./tests -o -path ./unity \) -prune -o -name '*.c' -print)
OBJECTS = $(foreach x, $(basename $(SOURCES)), $(x).o)
STARTUP_OBJ = $(subst .s,.o,$(STARTUP_FILE))
TARGET_ELF = $(NAME).elf
TARGET_MAP = $(NAME).map
ASM_FLAGS =

release: $(STARTUP_OBJ) $(OBJECTS)
	$(CC) -o $(TARGET_ELF) $^ -larm_cortexM0l_math -mcpu=cortex-m0plus -T"LinkerScript.ld" --specs=nosys.specs -Wl,-Map=$(TARGET_MAP) -Wl,--gc-sections -static -Wl,--start-group -larm_cortexM0l_math -Wl,--end-group -L./ --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group

compile: $(STARTUP_OBJ) $(OBJECTS)

$(STARTUP_OBJ):
	$(CC) -mcpu=cortex-m0plus $(ASM_FLAGS) -c -x assembler-with-cpp -MMD -MP -MF $(subst .s,.d,$(STARTUP_FILE)) -MT $(STARTUP_OBJ) --specs=nano.specs -mfloat-abi=soft -mthumb -o $(STARTUP_OBJ) $(STARTUP_FILE)

%.o: %.c
	$(CC) $< -mcpu=cortex-m0plus -std=gnu11 -DSTM32G071xx '-D__weak=__attribute__((weak))' -DARM_MATH_CM0PLUS '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -c -I./inc -I./Drivers/STM32G0xx_HAL_Driver/Inc -I./Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I./Drivers/CMSIS/Device/ST/STM32G0xx/Include -I./Drivers/CMSIS/Include -O3 -ffunction-sections -Wall -Wextra -fstack-usage -MMD -MP -MF $(subst .o,.d,$@) -MT $@ --specs=nano.specs -mfloat-abi=soft -mthumb -o $@

clean:
	rm -f $(TARGET_ELF) $(TARGET_MAP) $(OBJECTS) $(STARTUP_OBJ) $(foreach x, $(basename $(SOURCES)), $(x).d) $(foreach x, $(basename $(SOURCES)), $(x).su) $(subst .s,.d,$(STARTUP_FILE))
