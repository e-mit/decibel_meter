CLEANUP = rm -f
TARGET_EXTENSION=.out
C_COMPILER=gcc
UNITY_ROOT=../unity

CFLAGS=-std=gnu11
CFLAGS += -Wall
CFLAGS += -Wextra
CFLAGS += -Werror
CFLAGS += -Wshadow
CFLAGS += -Wpedantic
CFLAGS += -Wdouble-promotion
CFLAGS += -Wformat-overflow
CFLAGS += -Wformat-truncation
CFLAGS += -Wpointer-arith
CFLAGS += -Wcast-align
CFLAGS += -Wwrite-strings
CFLAGS += -Wswitch-default
CFLAGS += -Wunreachable-code
CFLAGS += -Winit-self
CFLAGS += -Wmissing-field-initializers
CFLAGS += -Wno-unknown-pragmas
CFLAGS += -Wstrict-prototypes
CFLAGS += -Wundef
CFLAGS += -Wold-style-definition
CFLAGS += -Wno-misleading-indentation

TARGET_BASE1=test_efficient_10log10
TARGET1 = $(TARGET_BASE1)$(TARGET_EXTENSION)
SRC_FILES1=$(UNITY_ROOT)/unity.c ../src/efficient_10log10.c $(TARGET_BASE1).c

TARGET_BASE2=test_sound_utilities
TARGET2 = $(TARGET_BASE2)$(TARGET_EXTENSION)
SRC_FILES2=$(UNITY_ROOT)/unity.c ../src/efficient_10log10.c ../src/sound_utilities.c $(TARGET_BASE2).c

INC_DIRS=-I$(UNITY_ROOT) -I../inc

all: clean build run

build: 
	$(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SRC_FILES1) -o $(TARGET1)
	$(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SRC_FILES2) -o $(TARGET2) -lm

run:
	- @./$(TARGET1)
	- @./$(TARGET2)

clean:
	$(CLEANUP) $(TARGET1) $(TARGET2)
