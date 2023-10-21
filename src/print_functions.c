#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include "project_config.h"
#include "print_functions.h"

#ifdef HAL_UART_MODULE_ENABLED
 
extern void UARTprint(char * str, uint16_t len); // this must be supplied in project code as it relies on HAL etc
// the above is supplied by UART.c/h

// expects a null-terminated string (no \n is needed; this is added)
void UARTprintLine(char * str) {
	#define BUFLEN 512
	char buf[BUFLEN] = {0};
	size_t len = strlen(str);
	if ((len+1) > BUFLEN) {
		// cut short:
		len = BUFLEN-1;
	}
	strcpy(buf,str);
	buf[len] = '\n';
	UARTprint(buf, len+1);
}

// call this just like printf()
#define buflen 512
static char buf[buflen] = {0};
void printSerial(const char* format, ...) {
    va_list argptr;
    va_start(argptr, format);
    vsnprintf (buf, buflen, format, argptr);
    // if the string is bigger than the buffer, the excess is discarded and no overflow results.
    va_end(argptr);
    UARTprint(buf, strlen(buf));
}
#undef buflen

// print a 64-bit (un)signed integer over the serial port in hex
// this is because, using the nano lib, cannot do it with printf, sprintf etc
// this just prints the number with apostrophes between each 16-bit chunk
// no newlines, spaces etc. Do get a negative sign if relevant.
void print64hex(int64_t n) {
	char buf[23] = {0};
	uint64_t x = llabs(n);
	uint16_t a = (uint16_t) (x>>48);
	uint16_t b = (uint16_t) ((x>>32) & 0xFFFF);
	uint16_t c = (uint16_t) ((x>>16) & 0xFFFF);
	uint16_t d = (uint16_t) (x & 0xFFFF);
	if (n < 0) {
		sprintf(buf,"-0x%04X'%04X'%04X'%04X",a,b,c,d);
	}
	else {
		sprintf(buf,"0x%04X'%04X'%04X'%04X",a,b,c,d);
	}
	UARTprint(buf,strlen(buf));
}

void printU64hex(uint64_t x) {
	char buf[23] = {0};
	uint16_t a = (uint16_t) (x>>48);
	uint16_t b = (uint16_t) ((x>>32) & 0xFFFF);
	uint16_t c = (uint16_t) ((x>>16) & 0xFFFF);
	uint16_t d = (uint16_t) (x & 0xFFFF);
	sprintf(buf,"0x%04X'%04X'%04X'%04X",a,b,c,d);
	UARTprint(buf,strlen(buf));
}

#endif
