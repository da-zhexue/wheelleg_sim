#ifndef DAMIAO_LQR_H
#define DAMIAO_LQR_H

float LQR_K_calc(const float *coe, float len);
void LQR_Calc(float *T, const float *K, const float *err);
void LQR_Calc_air(float *T, const float *K, const float *err);

#endif //DAMIAO_LQR_H