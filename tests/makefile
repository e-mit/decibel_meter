# Build unit tests using the unity framework.
#
# Note that these unit tests are intended to run on any computer.
# The separate system-tests are intended to run on the STM32 target.

TARGET_EXT = out
CC = gcc
UNITY_ROOT = ../Unity/src
TEST_PREFIX = unitytest_

CFLAGS = -std=gnu11
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

#####################################################################

TEST_SRCS = $(shell find -name "$(TEST_PREFIX)*.c" -type f)
TARGETS = $(foreach x, $(basename $(TEST_SRCS)), $(x).$(TARGET_EXT))
SRC = $(foreach x, $(notdir $(TEST_SRCS)), ../src/$(subst $(TEST_PREFIX),,$(x)))
INC_DIRS = -I$(UNITY_ROOT) -I../inc

all: clean $(TARGETS)

%.$(TARGET_EXT): %.c
	$(CC) $(CFLAGS) $(INC_DIRS) $(UNITY_ROOT)/unity.c $(SRC) $< -o $@ -lm

clean:
	rm -f *.$(TARGET_EXT)
