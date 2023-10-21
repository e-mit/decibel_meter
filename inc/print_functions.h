#ifndef PRINT_FUNCTIONS_H
#define PRINT_FUNCTIONS_H

#include <stdint.h>

void UARTprintLine(char * str);
void printSerial(const char* format, ...);
void print64hex(int64_t n);
void printU64hex(uint64_t x);

#ifdef DEBUG_PRINT
#define DEBUG_LOG(x) printSerial(x)
#else
#define DEBUG_LOG(x)
#endif

#endif

