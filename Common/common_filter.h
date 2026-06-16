#ifndef __COMMON_FILTER_H
#define __COMMON_FILTER_H

#include "common_debug.h"

typedef struct
{
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
} KalmanFilter_Struct;

extern KalmanFilter_Struct kfs[3];
int16_t Common_Filter_LowPass(int16_t newValue, int16_t preFilteredValue);

double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input);

#endif
