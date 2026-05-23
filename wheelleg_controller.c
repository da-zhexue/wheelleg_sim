#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>

#include "vmc.h"
#include "lqr.h"

#define TIME_STEP 64 // ms (0.064 s)
#define DT ((float)TIME_STEP / 1000.0f)

#define L1 0.8f // 大腿长
#define L2 0.8f // 小腿长
#define TORCH_MAX 10.0f
#define WHEEL_TORCH_MAX 5.0f
#define MG 1.5f // 机器人总重力的一半(单腿承重)

typedef struct {
    float kp, ki, kd;
    float err, last_err, integral;
} PID_Controller;

float PID_Calc(PID_Controller *pid, float current, float target) {
    pid->err = target - current;
    pid->integral += pid->err * DT;
    float derivative = (pid->err - pid->last_err) / DT;
    pid->last_err = pid->err;
    return pid->kp * pid->err + pid->ki * pid->integral + pid->kd * derivative;
}

const float lqr_K[12] = {
    -0.7850f, -0.1545f, -0.8625f, -1.0791f, -5.7511f, -1.2732f, 
    2.7737f, 0.3880f, -0.1227f, -0.1532f, -1.0004f, -0.1956f
};

int main(int argc, char **argv) {
    wb_robot_init();

    // 1. 获取设备
    WbDeviceTag wheel_L = wb_robot_get_device("wheel_motor_L");
    WbDeviceTag wheel_R = wb_robot_get_device("wheel_motor_R");
    WbDeviceTag joint_LF = wb_robot_get_device("joint_LF");
    WbDeviceTag joint_LB = wb_robot_get_device("joint_LB");
    WbDeviceTag joint_RF = wb_robot_get_device("joint_RF");
    WbDeviceTag joint_RB = wb_robot_get_device("joint_RB");

    WbDeviceTag inertial = wb_robot_get_device("inertial");
    wb_inertial_unit_enable(inertial, TIME_STEP);
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, TIME_STEP);

    WbDeviceTag ecd_LF = wb_robot_get_device("ecd_LF");
    WbDeviceTag ecd_RF = wb_robot_get_device("ecd_RF");
    WbDeviceTag ecd_LB = wb_robot_get_device("ecd_LB");
    WbDeviceTag ecd_RB = wb_robot_get_device("ecd_RB");
    wb_position_sensor_enable(ecd_LF, TIME_STEP);
    wb_position_sensor_enable(ecd_RF, TIME_STEP);
    wb_position_sensor_enable(ecd_LB, TIME_STEP);
    wb_position_sensor_enable(ecd_RB, TIME_STEP);

    // 2. 将轮毂电机也设置为纯力矩控制模式 (取消位置和速度控制)
    wb_motor_set_position(wheel_L, INFINITY);
    wb_motor_set_position(wheel_R, INFINITY);
    wb_motor_set_torque(wheel_L, 0.0);
    wb_motor_set_torque(wheel_R, 0.0);

    wb_motor_set_position(joint_LF, INFINITY);
    wb_motor_set_position(joint_LB, INFINITY);
    wb_motor_set_position(joint_RF, INFINITY);
    wb_motor_set_position(joint_RB, INFINITY);

    // 3. 初始化 VMC 与 PID
    vmc_leg_t *left_leg = Get_VMC_Leg(LEFT);
    vmc_leg_t *right_leg = Get_VMC_Leg(RIGHT);
    VMC_init(left_leg, L1, L2);
    VMC_init(right_leg, L1, L2);

    PID_Controller leg_l_pid = {30.0f, 0.0f, 3.0f, 0, 0, 0}; // 腿长PID参数参考硬件
    PID_Controller leg_r_pid = {30.0f, 0.0f, 3.0f, 0, 0, 0};

    float lqr_out_L[2], lqr_out_R[2];
    float err_L[6] = {0}, err_R[6] = {0};

    float target_L0 = 0.2f; // 目标腿长
    float x_filter = 0.0f;  // 模拟底盘位移(需靠里程计或轮速积分得到，此处作简化)
    float v_filter = 0.0f;  // 模拟底盘速度

    while (wb_robot_step(TIME_STEP) != -1) {
        // 读取传感器数据
        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial);
        double pitch = rpy[0];
        double pitch_rate = gyro ? wb_gyro_get_values(gyro)[0] : 0.0;
        printf("pitch: %.2f, pitch_rate: %2f\n", pitch * 180.0f / PI, pitch_rate);
        left_leg->phi1 = wb_position_sensor_get_value(ecd_LF);
        left_leg->phi4 = wb_position_sensor_get_value(ecd_LB);
        right_leg->phi1 = wb_position_sensor_get_value(ecd_RF);
        right_leg->phi4 = wb_position_sensor_get_value(ecd_RB);

        // ================= 左腿控制 =================
        float pitch_L = -pitch;
        float pitch_rate_L = -pitch_rate;
        VMC_calc_1(left_leg, pitch_L, pitch_rate_L, DT);

        // 1. 构建 LQR 状态误差矩阵
        err_L[0] = left_leg->theta - 0.0f;
        err_L[1] = left_leg->d_theta - 0.0f;
        err_L[2] = 0.0f - x_filter;        // X位移误差
        err_L[3] = 0.0f - v_filter;        // 速度误差
        err_L[4] = pitch_L - 0.0f;         // 机身俯仰角误差
        err_L[5] = pitch_rate_L - 0.0f;    // 机身俯仰角速度误差

        // 2. 计算 LQR 输出 (0: 轮毂力矩， 1: 髋关节虚拟力矩Tp)
        LQR_Calc(lqr_out_L, lqr_K, err_L);
        left_leg->Tp = lqr_out_L[1];

        // 3. 计算腿部推力 F0 (重力补偿 + 腿长PID)
        left_leg->F0 = MG / cos(left_leg->theta) + PID_Calc(&leg_l_pid, left_leg->L0, target_L0);
        printf("left Tp: %.2f, F0: %.2f\n", left_leg->Tp, left_leg->F0);
        VMC_calc_2(left_leg); // VMC逆解到关节力矩

        // ================= 右腿控制 =================
        float pitch_R = pitch;
        float pitch_rate_R = pitch_rate;
        VMC_calc_1(right_leg, pitch_R, pitch_rate_R, DT);

        err_R[0] = right_leg->theta - 0.0f;
        err_R[1] = right_leg->d_theta - 0.0f;
        err_R[2] = x_filter - 0.0f;
        err_R[3] = v_filter - 0.0f;
        err_R[4] = pitch_R - 0.0f;
        err_R[5] = pitch_rate_R - 0.0f;

        LQR_Calc(lqr_out_R, lqr_K, err_R);
        right_leg->Tp = lqr_out_R[1];

        right_leg->F0 = MG / cos(right_leg->theta) + PID_Calc(&leg_r_pid, right_leg->L0, target_L0);
        printf("right Tp: %.2f, F0: %.2f\n", right_leg->Tp, right_leg->F0);
        VMC_calc_2(right_leg);

        // ================= 力矩限幅与下发 =================
        if (left_leg->torque_set[0] > TORCH_MAX) left_leg->torque_set[0] = TORCH_MAX;
        if (left_leg->torque_set[0] < -TORCH_MAX) left_leg->torque_set[0] = -TORCH_MAX;
        if (left_leg->torque_set[1] > TORCH_MAX) left_leg->torque_set[1] = TORCH_MAX;
        if (left_leg->torque_set[1] < -TORCH_MAX) left_leg->torque_set[1] = -TORCH_MAX;

        if (right_leg->torque_set[0] > TORCH_MAX) right_leg->torque_set[0] = TORCH_MAX;
        if (right_leg->torque_set[0] < -TORCH_MAX) right_leg->torque_set[0] = -TORCH_MAX;
        if (right_leg->torque_set[1] > TORCH_MAX) right_leg->torque_set[1] = TORCH_MAX;
        if (right_leg->torque_set[1] < -TORCH_MAX) right_leg->torque_set[1] = -TORCH_MAX;

        // 下发关节力矩
        wb_motor_set_torque(joint_LF, left_leg->torque_set[0]);
        wb_motor_set_torque(joint_LB, left_leg->torque_set[1]);
        wb_motor_set_torque(joint_RF, right_leg->torque_set[0]);
        wb_motor_set_torque(joint_RB, right_leg->torque_set[1]);

        // 下发轮毂力矩
        float wheel_torque_L = lqr_out_L[0];
        float wheel_torque_R = lqr_out_R[0];

        // 限幅处理
        if (wheel_torque_L > WHEEL_TORCH_MAX) wheel_torque_L = WHEEL_TORCH_MAX;
        if (wheel_torque_L < -WHEEL_TORCH_MAX) wheel_torque_L = -WHEEL_TORCH_MAX;
        if (wheel_torque_R > WHEEL_TORCH_MAX) wheel_torque_R = WHEEL_TORCH_MAX;
        if (wheel_torque_R < -WHEEL_TORCH_MAX) wheel_torque_R = -WHEEL_TORCH_MAX;

        wb_motor_set_torque(wheel_L, wheel_torque_L);
        wb_motor_set_torque(wheel_R, wheel_torque_R);
    }
    
    wb_robot_cleanup();
    return 0;
}