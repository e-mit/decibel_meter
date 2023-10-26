# Makefile for release, debug and system-tests builds.
#
# These builds target the STM32G071xx chip (Cortex M0+).
# This uses gcc-arm-none-eabi 9-2020-q2-update, available from:
# https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2

NAME = decibel_meter
CC_VER = gcc-arm-none-eabi-9-2020-q2-update
INC_FLAGS = -I./inc -I./Drivers/STM32G0xx_HAL_Driver/Inc -I./Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I./Drivers/CMSIS/Device/ST/STM32G0xx/Include -I./Drivers/CMSIS/Include
LNK_SCRIPT = LinkerScript.ld
STARTUP_FILE = startup/startup_stm32.s
LIB_DIR = ./lib

#############################################

ifeq ($(MAKECMDGOALS),release)
	ASM_FLAGS =
	C_FLAGS = -O3
	LNK_FLAGS =
	CLEAN_DIRS = $(MAKECMDGOALS)
else ifeq ($(MAKECMDGOALS),debug)
	ASM_FLAGS = -g3
	C_FLAGS = -g3 -DDEBUG -O0
	LNK_FLAGS = -u_printf_float
	CLEAN_DIRS = $(MAKECMDGOALS)
else ifeq ($(MAKECMDGOALS),system-tests)
	ASM_FLAGS = -g3
	C_FLAGS = -g3 -DTESTS -DDEBUG -O0
	LNK_FLAGS = -u_printf_float
	CLEAN_DIRS = $(MAKECMDGOALS)
else ifeq ($(MAKECMDGOALS),clean)
	CLEAN_DIRS = release debug system-tests
endif

CC = $(CC_VER)/bin/arm-none-eabi-gcc
SOURCES = $(shell find src/ Drivers/ -name "*.c" -type f)
OBJECTS = $(foreach x, $(basename $(SOURCES)), $(MAKECMDGOALS)/$(x).o)
STARTUP_OBJ = $(MAKECMDGOALS)/$(subst .s,.o,$(STARTUP_FILE))
TARGET_ELF = $(MAKECMDGOALS)/$(NAME).elf
TARGET_MAP = $(MAKECMDGOALS)/$(NAME).map

#############################################

release: clean mkdirs build

debug: clean mkdirs build

system-tests: clean mkdirs build

build: $(STARTUP_OBJ) $(OBJECTS)
	$(CC) -o $(TARGET_ELF) $^ -larm_cortexM0l_math -mcpu=cortex-m0plus -T $(LNK_SCRIPT) --specs=nosys.specs -Wl,-Map=$(TARGET_MAP) -Wl,--gc-sections -static -Wl,--start-group -larm_cortexM0l_math -Wl,--end-group -L$(LIB_DIR) $(LNK_FLAGS) --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group

$(STARTUP_OBJ):
	$(CC) -mcpu=cortex-m0plus $(ASM_FLAGS) -c -x assembler-with-cpp -MMD -MP -MF $(subst .o,.d,$(STARTUP_OBJ)) -MT $(STARTUP_OBJ) --specs=nano.specs -mfloat-abi=soft -mthumb -o $(STARTUP_OBJ) $(STARTUP_FILE)

$(MAKECMDGOALS)/%.o: %.c
	$(CC) $< -mcpu=cortex-m0plus -std=gnu11 $(C_FLAGS) -DSTM32G071xx '-D__weak=__attribute__((weak))' -DARM_MATH_CM0PLUS '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -c $(INC_FLAGS) -ffunction-sections -Wall -Wextra -fstack-usage -MMD -MP -MF $(subst .o,.d,$@) -MT $@ --specs=nano.specs -mfloat-abi=soft -mthumb -o $@

mkdirs:
	$(shell mkdir -p $(MAKECMDGOALS)/$(dir $(STARTUP_FILE)))
	$(foreach src, $(dir $(SOURCES)), $(shell mkdir -p $(MAKECMDGOALS)/$(src)))

clean:
	rm -rf $(CLEAN_DIRS)
