#ifndef PRINT_FUNCTIONS_H
#define PRINT_FUNCTIONS_H

#include <stdint.h>

void print(const char* format, ...);
void print64hex(int64_t n);
void printU64hex(uint64_t x);

#ifdef DEBUG_PRINT
#define DEBUG_LOG(x) print(x)
#else
#define DEBUG_LOG(x)
#endif

#endif

