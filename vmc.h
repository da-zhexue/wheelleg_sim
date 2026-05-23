#ifndef DAMIAO_VMC_H
#define DAMIAO_VMC_H

#define LEFT 0
#define RIGHT 1
#define PI 3.14159265358979323846f
#include <stdbool.h>

typedef struct
{
    /*左右两腿的公共参数，固定不变*/
    float l5;//AE长度 //单位为m
    float l1;//单位为m
    float l2;//单位为m
    float l3;//单位为m
    float l4;//单位为m

    float XB, YB;//B点的坐标
    float XD, YD;//D点的坐标

    float XC, YC;//C点的直角坐标
    float L0, phi0;//C点的极坐标
    float alpha;
    float d_alpha;

    float lBD; // BD两点的距离

    float d_phi0; // 现在C点角度phi0的变换率
    float last_phi0; // 上一次C点角度，用于计算角度phi0的变换率d_phi0

    float A0, B0, C0; // 中间变量
    float phi2, phi3;
    float phi1, phi4;

    float j11, j12, j21, j22; // 笛卡尔空间力到关节空间的力的雅可比矩阵系数
    float torque_set[2];

    float F0;
    float Tp;
    float F02;

    float theta;
    float d_theta; // theta的一阶导数
    float last_d_theta;
    float dd_theta; // theta的二阶导数

    float d_L0; // L0的一阶导数
    float dd_L0; // L0的二阶导数
    float last_L0;
    float last_d_L0;

    float FN; // 支持力

    bool first_flag;
    bool leg_flag; // 腿长完成标志
} vmc_leg_t;

void VMC_init(vmc_leg_t *vmc, float l1, float l2);
void VMC_calc_1(vmc_leg_t *vmc, float Pitch, float PithGyro, float dt);
void VMC_calc_2(vmc_leg_t *vmc);
bool ground_detection(vmc_leg_t *vmc, float g);
vmc_leg_t *Get_VMC_Leg(bool leg);

#endif //DAMIAO_VMC_H