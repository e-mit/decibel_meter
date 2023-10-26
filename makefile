# Makefile for release, debug and system-tests builds.
# Builds for the STM32G071xx chip.
# Uses gcc-arm-none-eabi 9-2020-q2-update, available from:
# https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2?revision=05382cca-1721-44e1-ae19-1e7c3dc96118&rev=05382cca172144e1ae191e7c3dc96118&hash=FDE675133A099796BD1507A3FF215AC4

NAME = decibel_meter
CC_VER = gcc-arm-none-eabi-9-2020-q2-update
INC_DIRS = -I./inc -I./Drivers/STM32G0xx_HAL_Driver/Inc -I./Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I./Drivers/CMSIS/Device/ST/STM32G0xx/Include -I./Drivers/CMSIS/Include
LNK_SCRIPT = LinkerScript.ld
STARTUP_FILE = startup/startup_stm32.s

#############################################

CC = $(CC_VER)/bin/arm-none-eabi-gcc
SOURCES = $(shell find . -type d \( -path ./tests -o -path ./$(CC_VER) -o -path ./unity \) -prune -o -name '*.c' -print)
OBJECTS = $(foreach x, $(basename $(SOURCES)), $(x).o)
STARTUP_OBJ = $(subst .s,.o,$(STARTUP_FILE))
TARGET_ELF = $(NAME).elf
TARGET_MAP = $(NAME).map

ifeq ($(MAKECMDGOALS),release)
	ASM_FLAGS =
	C_FLAGS = -O3
	LNK_FLAGS =
else ifeq ($(MAKECMDGOALS),debug)
	ASM_FLAGS = -g3
	C_FLAGS = -g3 -DDEBUG -O0
	LNK_FLAGS = -u_printf_float
else ifeq ($(MAKECMDGOALS),tests)
	ASM_FLAGS = -g3
	C_FLAGS = -g3 -DTESTS -DDEBUG -O0
	LNK_FLAGS = -u_printf_float
endif

#############################################

release: build

debug: build

tests: build

build: $(STARTUP_OBJ) $(OBJECTS)
	$(CC) -o $(TARGET_ELF) $^ -larm_cortexM0l_math -mcpu=cortex-m0plus -T $(LNK_SCRIPT) --specs=nosys.specs -Wl,-Map=$(TARGET_MAP) -Wl,--gc-sections -static -Wl,--start-group -larm_cortexM0l_math -Wl,--end-group -L./ $(LNK_FLAGS) --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group

compile: $(STARTUP_OBJ) $(OBJECTS)

$(STARTUP_OBJ):
	$(CC) -mcpu=cortex-m0plus $(ASM_FLAGS) -c -x assembler-with-cpp -MMD -MP -MF $(subst .s,.d,$(STARTUP_FILE)) -MT $(STARTUP_OBJ) --specs=nano.specs -mfloat-abi=soft -mthumb -o $(STARTUP_OBJ) $(STARTUP_FILE)

%.o: %.c
	$(CC) $< -mcpu=cortex-m0plus -std=gnu11 $(C_FLAGS) -DSTM32G071xx '-D__weak=__attribute__((weak))' -DARM_MATH_CM0PLUS '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -c $(INC_DIRS) -ffunction-sections -Wall -Wextra -fstack-usage -MMD -MP -MF $(subst .o,.d,$@) -MT $@ --specs=nano.specs -mfloat-abi=soft -mthumb -o $@

clean:
	rm -f $(TARGET_ELF) $(TARGET_MAP) $(OBJECTS) $(STARTUP_OBJ) $(foreach x, $(basename $(SOURCES)), $(x).d) $(foreach x, $(basename $(SOURCES)), $(x).su) $(subst .s,.d,$(STARTUP_FILE))
