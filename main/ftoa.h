#ifndef FTOA_H
#define FTOA_H

#include <stdint.h>
#include <stdio.h>

int power(int base, int exp);
char* ftoa(const float num, char* output, uint8_t decimals);

#endif
