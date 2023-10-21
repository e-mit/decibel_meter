#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include "project_config.h"
#include "print_functions.h"

extern void printString(char * str, uint16_t len); // this must be supplied in external code

static char strbuf[512] = {0};

// Format a string for printing. Call this just like printf().
void print(const char* format, ...) {
    va_list argptr;
    va_start(argptr, format);
    vsnprintf(strbuf, sizeof strbuf, format, argptr);
    va_end(argptr);
    printString(strbuf, strlen(strbuf));
}

// Print a 64-bit signed integer in hex.
// This is needed because the Arm nano lib cannot do it with printf, sprintf etc
// Just prints the number with apostrophes between each 16-bit chunk,
// no newlines or spaces included.
void print64hex(int64_t n) {
	uint64_t x = llabs(n);
	uint16_t a = (uint16_t) (x >> 48);
	uint16_t b = (uint16_t) ((x >> 32) & 0xFFFF);
	uint16_t c = (uint16_t) ((x >> 16) & 0xFFFF);
	uint16_t d = (uint16_t) (x & 0xFFFF);
	if (n < 0) {
		snprintf(strbuf, sizeof strbuf, "-0x%04X'%04X'%04X'%04X", a, b, c, d);
	}
	else {
		snprintf(strbuf, sizeof strbuf, "0x%04X'%04X'%04X'%04X", a, b, c, d);
	}
	printString(strbuf, strlen(strbuf));
}

// Print a 64-bit unsigned integer in hex.
// This is needed because the Arm nano lib cannot do it with printf, sprintf etc
// Just prints the number with apostrophes between each 16-bit chunk,
// no newlines or spaces included.
void printU64hex(uint64_t x) {
	uint16_t a = (uint16_t) (x >> 48);
	uint16_t b = (uint16_t) ((x >> 32) & 0xFFFF);
	uint16_t c = (uint16_t) ((x >> 16) & 0xFFFF);
	uint16_t d = (uint16_t) (x & 0xFFFF);
	snprintf(strbuf, sizeof strbuf, "0x%04X'%04X'%04X'%04X", a, b, c, d);
	printString(strbuf, strlen(strbuf));
}

