#ifndef COMMON_TOOL_H
#define COMMON_TOOL_H

#include "stdint.h"

/**
    限幅函数
    @param x 输入值
    @param min 最小值
    @param max 最大值
    @return 限幅后的值
**/
int16_t Com_limit_int16(int16_t x, int16_t min, int16_t max);

#endif /* COMMON_TOOL_H */