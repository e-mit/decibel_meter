CLEANUP = rm -f
TARGET_EXTENSION=.out
C_COMPILER=gcc
UNITY_ROOT=../unity

CFLAGS=-std=gnu11
CFLAGS += -Wall
CFLAGS += -Wextra
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
#CFLAGS += -Wno-misleading-indentation

TARGET_BASE1=test1
TARGET1 = $(TARGET_BASE1)$(TARGET_EXTENSION)
SRC_FILES1=$(UNITY_ROOT)/unity.c ../src/efficient_10log10.c test_efficient_10log10.c
TARGET_BASE2=test2
TARGET2 = $(TARGET_BASE2)$(TARGET_EXTENSION)
SRC_FILES2=$(UNITY_ROOT)/unity.c ../src/efficient_10log10.c ../src/sound_utilities.c test_sound_utilities.c
INC_DIRS=-I$(UNITY_ROOT) -I../inc
SYMBOLS=

all: clean default

default: 
	$(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES1) -o $(TARGET1)
	$(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES2) -o $(TARGET2) -lm
	- ./$(TARGET1)
	- ./$(TARGET2)

clean:
	$(CLEANUP) $(TARGET1) $(TARGET2)
