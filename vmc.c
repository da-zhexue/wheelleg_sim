#include "vmc.h"
#include <math.h>
#include <stddef.h>

static vmc_leg_t leg_left, leg_right;

void VMC_init(vmc_leg_t *vmc, const float l1, const float l2)
{
    vmc->l1 = l1;
    vmc->l2 = l2;
    vmc->l3 = l2;
    vmc->l4 = l1;
    vmc->l5 = 0.0f;
    vmc->first_flag = 0;
    vmc->last_phi0 = 0.0f;
    vmc->last_d_theta = 0.0f;
    vmc->last_L0 = 0.0f;
    vmc->last_d_L0 = 0.0f;
}

void VMC_calc_1(vmc_leg_t *vmc, const float pitch, const float pitchgyro, const float dt)
{
    static float Pitch=0.0f;
    static float PithGyro=0.0f;
    Pitch = pitch;
    PithGyro =  pitchgyro;

    vmc->YD = vmc->l4 * sinf(vmc->phi4);//D的y坐标
    vmc->YB = vmc->l1 * sinf(vmc->phi1);//B的y坐标
    vmc->XD = vmc->l5 + vmc->l4 * cosf(vmc->phi4);//D的x坐标
    vmc->XB = vmc->l1 * cosf(vmc->phi1); //B的x坐标

    vmc->lBD = sqrtf((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB) * (vmc->YD - vmc->YB));

    vmc->A0 = 2.0f * vmc->l2*(vmc->XD - vmc->XB);
    vmc->B0 = 2.0f * vmc->l2*(vmc->YD - vmc->YB);
    vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;

    float inner_sqrt = vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0;
    if (inner_sqrt < 0.0f) inner_sqrt = 0.0f; // Prevent NaN

    vmc->phi2 = 2.0f * atan2f((vmc->B0 + sqrtf(inner_sqrt)), vmc->A0 + vmc->C0);
    vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * sinf(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * cosf(vmc->phi2));
    //C点直角坐标
    vmc->XC = vmc->l1*cosf(vmc->phi1) + vmc->l2 * cosf(vmc->phi2);
    vmc->YC = vmc->l1*sinf(vmc->phi1) + vmc->l2 * sinf(vmc->phi2);
    //C点极坐标
    vmc->L0 = sqrtf((vmc->XC - vmc->l5/2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);

    vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5/2.0f));//phi0用于计算lqr需要的theta
    vmc->alpha = PI/2.0f - vmc->phi0 ;

    if(vmc->first_flag == 0)
    {
        vmc->last_phi0 = vmc->phi0 ;
        vmc->first_flag = 1;
    }
    vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    vmc->d_alpha = 0.0f - vmc->d_phi0 ;

    vmc->theta = PI/2.0f - Pitch - vmc->phi0;//得到状态变量1
    vmc->d_theta = (-PithGyro - vmc->d_phi0);//得到状态变量2

    vmc->last_phi0 = vmc->phi0 ;

    vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;//腿长L0的一阶导数
    vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;//腿长L0的二阶导数

    vmc->last_d_L0 = vmc->d_L0;
    vmc->last_L0 = vmc->L0;

    vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    vmc->last_d_theta = vmc->d_theta;
}

void VMC_calc_2(vmc_leg_t *vmc)
{
    vmc->j11 = (vmc->l1 * sinf(vmc->phi0-vmc->phi3) * sinf(vmc->phi1-vmc->phi2)) / sinf(vmc->phi3-vmc->phi2);
    vmc->j12 = (vmc->l1 * cosf(vmc->phi0-vmc->phi3) * sinf(vmc->phi1-vmc->phi2)) / (vmc->L0*sinf(vmc->phi3-vmc->phi2));
    vmc->j21 = (vmc->l4 * sinf(vmc->phi0-vmc->phi2) * sinf(vmc->phi3-vmc->phi4)) / sinf(vmc->phi3-vmc->phi2);
    vmc->j22 = (vmc->l4 * cosf(vmc->phi0-vmc->phi2) * sinf(vmc->phi3-vmc->phi4)) / (vmc->L0*sinf(vmc->phi3-vmc->phi2));

    vmc->torque_set[0]=vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp;
    vmc->torque_set[1]=vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp;
}

static float aver[2][4];
bool ground_detection(vmc_leg_t *vmc, const float g) // 离地检测
{
    const bool leg = (vmc == &leg_left) ? LEFT : RIGHT;
    vmc->FN = vmc->F0 * cosf(vmc->theta) + vmc->Tp * sinf(vmc->theta) / vmc->L0 +
        0.6f * (g-vmc->dd_L0 * cosf(vmc->theta) + 2.0f * vmc->d_L0 * vmc->d_theta * sinf(vmc->theta) +
            vmc->L0 * vmc->dd_theta * sinf(vmc->theta) + vmc->L0 * vmc->d_theta * vmc->d_theta * cosf(vmc->theta));


    aver[leg][0]=aver[leg][1];
    aver[leg][1]=aver[leg][2];
    aver[leg][2]=aver[leg][3];
    aver[leg][3]=vmc->FN;

    const float aver_fn=0.25f*aver[leg][0]+0.25f*aver[leg][1]+0.25f*aver[leg][2]+0.25f*aver[leg][3];//对支持力进行均值滤波

    if(aver_fn<3.0f)
        return 1;

    return 0;
}

vmc_leg_t *Get_VMC_Leg(const bool leg)
{
    if (leg == LEFT)
        return &leg_left;
    if (leg == RIGHT)
        return &leg_right;

    return NULL;
}