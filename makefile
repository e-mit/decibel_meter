# Makefile for release, debug and system-tests builds.
#
# These builds target the STM32G071xx chip (Cortex M0+).
# This uses gcc-arm-none-eabi 9-2020-q2-update, available from:
# https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/
#         gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2

NAME = decibel_meter
CC_VER = gcc-arm-none-eabi-9-2020-q2-update
INC_FLAGS = -I./inc -I./drivers/STM32G0xx_HAL_Driver/Inc \
            -I./drivers/STM32G0xx_HAL_Driver/Inc/Legacy \
            -I./drivers/CMSIS/Device/ST/STM32G0xx/Include \
            -I./drivers/CMSIS/Include
LNK_SCRIPT = LinkerScript.ld
STARTUP_FILE = startup/startup_stm32.s
LIB_DIR = ./lib

#############################################

ifeq ($(MAKECMDGOALS),release)
	ASM_FLAGS =
	CFLAGS = -O3
	LNK_FLAGS =
	CLEAN_DIRS = $(MAKECMDGOALS)
else ifeq ($(MAKECMDGOALS),debug)
	ASM_FLAGS = -g3
	CFLAGS = -g3 -DDEBUG -O0
	LNK_FLAGS = -u_printf_float
	CLEAN_DIRS = $(MAKECMDGOALS)
else ifeq ($(MAKECMDGOALS),system-tests)
	ASM_FLAGS = -g3
	CFLAGS = -g3 -DTESTS -DDEBUG -O0
	LNK_FLAGS = -u_printf_float
	CLEAN_DIRS = $(MAKECMDGOALS)
else ifeq ($(MAKECMDGOALS),clean)
	CLEAN_DIRS = release debug system-tests
endif

CC = $(CC_VER)/bin/arm-none-eabi-gcc
SOURCES = $(shell find src/ drivers/ -name "*.c" -type f)
OBJECTS = $(foreach x, $(basename $(SOURCES)), $(MAKECMDGOALS)/$(x).o)
STARTUP_OBJ = $(MAKECMDGOALS)/$(subst .s,.o,$(STARTUP_FILE))
TARGET_ELF = $(MAKECMDGOALS)/$(NAME).elf
TARGET_MAP = $(MAKECMDGOALS)/$(NAME).map

CFLAGS += -Wall
CFLAGS += -Wextra
CFLAGS += -Werror
CFLAGS += -Wshadow
CFLAGS += -Wformat-overflow
CFLAGS += -Wformat-truncation
CFLAGS += -Wpointer-arith
CFLAGS += -Wwrite-strings
CFLAGS += -Wswitch-default
CFLAGS += -Wunreachable-code
CFLAGS += -Winit-self
CFLAGS += -Wmissing-field-initializers
CFLAGS += -Wno-unknown-pragmas
CFLAGS += -Wstrict-prototypes
CFLAGS += -Wold-style-definition
CFLAGS += -Wno-misleading-indentation

#############################################

release: clean mkdirs build

debug: clean mkdirs build

system-tests: clean mkdirs build

build: $(STARTUP_OBJ) $(OBJECTS)
	$(CC) -o $(TARGET_ELF) $^ -larm_cortexM0l_math -mcpu=cortex-m0plus -T $(LNK_SCRIPT) \
          --specs=nosys.specs -Wl,-Map=$(TARGET_MAP) -Wl,--gc-sections -static -Wl,--start-group \
          -larm_cortexM0l_math -Wl,--end-group -L$(LIB_DIR) $(LNK_FLAGS) --specs=nano.specs \
          -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group

$(STARTUP_OBJ):
	$(CC) -mcpu=cortex-m0plus $(ASM_FLAGS) -c -x assembler-with-cpp -MMD -MP -MF \
          $(subst .o,.d,$(STARTUP_OBJ)) -MT $(STARTUP_OBJ) --specs=nano.specs -mfloat-abi=soft \
          -mthumb -o $(STARTUP_OBJ) $(STARTUP_FILE)

$(MAKECMDGOALS)/%.o: %.c
	$(CC) $< -mcpu=cortex-m0plus -std=gnu11 $(CFLAGS) -DSTM32G071xx \
          '-D__weak=__attribute__((weak))' -DARM_MATH_CM0PLUS \
          '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -c $(INC_FLAGS) \
          -ffunction-sections -fstack-usage -MMD -MP -MF $(subst .o,.d,$@) \
          -MT $@ --specs=nano.specs -mfloat-abi=soft -mthumb -o $@

mkdirs:
	$(shell mkdir -p $(MAKECMDGOALS)/$(dir $(STARTUP_FILE)))
	$(foreach src, $(dir $(SOURCES)), $(shell mkdir -p $(MAKECMDGOALS)/$(src)))

clean:
	rm -rf $(CLEAN_DIRS)
