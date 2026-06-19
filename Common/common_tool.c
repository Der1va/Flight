#include "common_tool.h"

int16_t Com_limit_int16(int16_t x, int16_t min, int16_t max)
{
    if (x < min)
        return min;
    else if (x > max)
        return max;
    else
        return x;
}