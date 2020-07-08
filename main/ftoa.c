#include "ftoa.h"

int power(int base, int exp)
{
  int result = 1;
  while (exp) {
    result *= base;
    exp--;
  }

  return result;
}

char* ftoa(const float num, char* output, uint8_t decimals){
    int rounded = (int) num;
    int decimal = (int) ((num - rounded) * power(10, decimals));
    if (decimal < 0) {
        // get rid of sign on decimal portion
        decimal = -decimal;
    }

    char *pattern[10]; // setup printf pattern for decimal portion
    sprintf((char *) pattern, "%%d.%%0%dd", decimals);
    sprintf((char *) output, (const char *) pattern, rounded, decimal);
    return output;
}
