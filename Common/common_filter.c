#include "common_filter.h"

#define ALPHA 0.15

int16_t Common_Filter_LowPass(int16_t newValue, int16_t preFilteredValue)
{
    return ALPHA * newValue + (1 - ALPHA) * preFilteredValue;
}

KalmanFilter_Struct kfs[3] = {
    {0.02, 0, 0, 0, 0.001, 0.543},
    {0.02, 0, 0, 0, 0.001, 0.543},
    {0.02, 0, 0, 0, 0.001, 0.543}};
double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input)
{
    kf->Now_P = kf->LastP + kf->Q;
    kf->Kg = kf->Now_P / (kf->Now_P + kf->R);
    kf->out = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    return kf->out;
}
