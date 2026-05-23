#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "vmc.h"
#include "lqr.h"
#include "kalman_filter.h"
#include "keyboard.h"

const float lqr_K[12] = {
    -36.7965f, -1.1603f, -2.20387f, -0.6839f, -9.7831f, -1.3954f,
    6.7737f, 0.9880f, -0.1227f, -0.1532f, -3.0004f, -0.1956f
};
// const float lqr_K[12] = {
//     -0.7965f, -0.1603f, -0.7387f, -1.1839f, -7.7831f, -1.3954f,
//     2.7737f, 0.3880f, -0.1227f, -0.1532f, -1.0004f, -0.1956f
// };
// const float lqr_K[12] = {
//     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
//     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
// };

#define TIME_STEP 4
#define DT ((float)TIME_STEP / 1000.0f)

#define MAX_POS_ERR 0.5f
#define ACCEL_LIMIT 1.0f

#define L1 0.8f // 大腿长
#define L2 0.8f // 小腿长
#define TORCH_MAX 30.0f
#define WHEEL_TORCH_MAX 5.0f
#define MG 1.5f // 机器人总重力的一半(单腿承重)
#define WHEEL_RAD 0.1f
#define ACCEL_LPF 0.0089f // 加速度低通滤波系数（参考INS_task）

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

KalmanFilter_t vaEstimateKF;  // 滤波器实例
float vel_acc[2];             // 输出：vel_acc[0]是滤波后的速度

// 卡尔曼滤波器矩阵（参考observe_task.c）
float vaEstimateKF_F[4] = {1.0f, DT, 0.0f, 1.0f};
float vaEstimateKF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f};
float vaEstimateKF_Q[4] = {1.0f, 0.0f, 0.0f, 1.0f};
float vaEstimateKF_R[4] = {200.0f, 0.0f, 0.0f, 200.0f};
const float vaEstimateKF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f};

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);
    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    EstimateKF->MeasuredVector[0] = vel;
    EstimateKF->MeasuredVector[1] = acc;
    Kalman_Filter_Update(EstimateKF);
    vel_acc[0] = EstimateKF->FilteredValue[0]; // v_filter
}

float yaw_rate_cmd = 0.0f;      // 期望偏航角速度
float pitch_compensation = 0.0f; // 旋转时的俯仰角补偿
float centrifugal_force = 0.0f;  // 离心力补偿

// 重力加速度（地球坐标系，Z轴向上，参考INS_task）
static const float GRAVITY[3] = {0.0f, 0.0f, 9.81f};
float MotionAccel_b[3] = {0};  // 机体坐标系运动加速度
float MotionAccel_n[3] = {0};  // 地球坐标系运动加速度

/**
 * @brief 欧拉角转四元数（Z-Y-X旋转顺序：yaw→pitch→roll）
 * @param yaw   偏航角 (绕Z轴)
 * @param pitch 俯仰角 (绕Y轴)
 * @param roll  横滚角 (绕X轴)
 * @param q     输出四元数 [w, x, y, z]
 */
void EulerToQuaternion(float yaw, float pitch, float roll, float q[4])
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q[0] = cy * cp * cr + sy * sp * sr;  // w
    q[1] = cy * cp * sr - sy * sp * cr;  // x
    q[2] = cy * sp * cr + sy * cp * sr;  // y
    q[3] = sy * cp * cr - cy * sp * sr;  // z
}

/**
 * @brief 地球坐标系 -> 机体坐标系（参考INS_task.c）
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, const float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief 机体坐标系 -> 地球坐标系（参考INS_task.c）
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, const float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

int main(int argc, char **argv) {
    wb_robot_init();

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
    WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(accelerometer, TIME_STEP);

    WbDeviceTag ecd_LF = wb_robot_get_device("ecd_LF");
    WbDeviceTag ecd_RF = wb_robot_get_device("ecd_RF");
    WbDeviceTag ecd_LB = wb_robot_get_device("ecd_LB");
    WbDeviceTag ecd_RB = wb_robot_get_device("ecd_RB");
    wb_position_sensor_enable(ecd_LF, TIME_STEP);
    wb_position_sensor_enable(ecd_RF, TIME_STEP);
    wb_position_sensor_enable(ecd_LB, TIME_STEP);
    wb_position_sensor_enable(ecd_RB, TIME_STEP);

    WbDeviceTag ecd_wheel_L = wb_robot_get_device("ecd_wheel_L");
    WbDeviceTag ecd_wheel_R = wb_robot_get_device("ecd_wheel_R");
    wb_position_sensor_enable(ecd_wheel_L, TIME_STEP);
    wb_position_sensor_enable(ecd_wheel_R, TIME_STEP);

    wb_motor_set_position(wheel_L, INFINITY);
    wb_motor_set_position(wheel_R, INFINITY);
    wb_motor_set_torque(wheel_L, 0.0);
    wb_motor_set_torque(wheel_R, 0.0);

    wb_motor_set_position(joint_LF, INFINITY);
    wb_motor_set_position(joint_LB, INFINITY);
    wb_motor_set_position(joint_RF, INFINITY);
    wb_motor_set_position(joint_RB, INFINITY);

    // wb_motor_set_position(joint_LF, 0);
    // wb_motor_set_position(joint_LB, 0);
    // wb_motor_set_position(joint_RF, 0);
    // wb_motor_set_position(joint_RB, 0);

    vmc_leg_t *left_leg = Get_VMC_Leg(LEFT);
    vmc_leg_t *right_leg = Get_VMC_Leg(RIGHT);
    VMC_init(left_leg, L1, L2);
    VMC_init(right_leg, L1, L2);

    PID_Controller leg_l_pid = {500.0f, 0.0f, 10.0f, 0, 0, 0}; // 腿长PID参数参考硬件
    PID_Controller leg_r_pid = {500.0f, 0.0f, 10.0f, 0, 0, 0};

    float lqr_out_L[2], lqr_out_R[2];
    float err_L[6] = {0}, err_R[6] = {0};

    float target_L0 = 0.8f;
    float target_v = 0.0f;
    float smooth_target_v = 0.0f;
    float target_x_ref = 0.0f;
    float x_filter = 0.0f;
    float x_l = 0.0f, x_r = 0.0f;
    float v_l = 0.0f, v_r = 0.0f;
    float v_filter = 0.0f;
    float last_x_l = 0.0f, last_x_r = 0.0f;

    float turn_set = 0.0f;

    float roll_set = 0.0f;

    PID_Controller turn_pid = {1.0f, 0.0f, 0.0f, 0, 0, 0};
    PID_Controller roll_pid = {10.0f, 0.0f, 0.5f, 0, 0, 0};
    PID_Controller tp_pid = {3.0f, 0.0f, 0.0f, 0, 0, 0};

    float current_time = 0.0f;

    Keyboard_Init(TIME_STEP);

    xvEstimateKF_Init(&vaEstimateKF); // 初始化卡尔曼滤波器（参考observe_task.c）

    float filter_alpha = 0.03f; // 一阶滤波系数，取值0~1，越小越平滑

    while (wb_robot_step(TIME_STEP) != -1) {
        current_time += (float)TIME_STEP / 1000.0f;
        Keyboard_Update(&target_v, &target_L0, &turn_set);

        // 一阶滤波实现平滑加减速
        smooth_target_v = filter_alpha * target_v + (1.0f - filter_alpha) * smooth_target_v;

        target_x_ref += smooth_target_v * DT;

        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial);
        double pitch = rpy[0];
        double roll = rpy[1];
        double yaw = rpy[2];

        const double *gyro_vals = wb_gyro_get_values(gyro);
        double pitch_rate = gyro_vals[0];
        double roll_rate = gyro_vals[1];
        double yaw_rate = gyro_vals[2];

        float w_pos_L = wb_position_sensor_get_value(ecd_wheel_L);
        float w_pos_R = wb_position_sensor_get_value(ecd_wheel_R);

        printf("pitch: %.2f, pitch_rate: %2f\n", pitch * 180.0f / PI, pitch_rate);
        printf("yaw_target: %.2f, yaw: %.2f, yaw_rate: %2f\n", turn_set * 180.0f / PI, yaw * 180.0f / PI, yaw_rate);
        left_leg->phi1 = wb_position_sensor_get_value(ecd_LF) + PI * 150.0f / 180.0f;
        left_leg->phi4 = wb_position_sensor_get_value(ecd_LB) + PI * 30.0f / 180.0f;
        right_leg->phi4 = wb_position_sensor_get_value(ecd_RF) + PI * 30.0f / 180.0f;
        right_leg->phi1 = wb_position_sensor_get_value(ecd_RB) + PI * 150.0f / 180.0f;

        // 轮子速度计算（用于卡尔曼滤波测量）
        x_l = w_pos_L / 2.0f * WHEEL_RAD;
        x_r = w_pos_R / 2.0f * WHEEL_RAD;
        v_l = (x_l - last_x_l) / DT;
        v_r = (x_r - last_x_r) / DT;
        float wheel_v = (v_l - v_r) / 2.0f;
        last_x_l = x_l;
        last_x_r = x_r;

        // 坐标系变换：用确认的yaw/pitch/roll计算四元数（不直接用webots的quaternion）
        float q[4];
        EulerToQuaternion((float)yaw, (float)pitch, (float)roll, q);

        const double *accel_vals = wb_accelerometer_get_values(accelerometer);
        float Accel_b[3] = {(float)accel_vals[1], (float)accel_vals[0], (float)accel_vals[2]};

        float gravity_b[3];
        EarthFrameToBodyFrame(GRAVITY, gravity_b, q);

        // 运动加速度 = 加速度计读数 - 重力投影（一阶低通滤波，同INS_task.c）
        for (uint8_t i = 0; i < 3; i++)
        {
            MotionAccel_b[i] = (Accel_b[i] - gravity_b[i]) * DT / (ACCEL_LPF + DT)
                             + MotionAccel_b[i] * ACCEL_LPF / (ACCEL_LPF + DT);
        }
        BodyFrameToEarthFrame(MotionAccel_b, MotionAccel_n, q);

        // 死区滤波（参考INS_task.c）
        if (fabsf(MotionAccel_n[0]) < 0.02f) MotionAccel_n[0] = 0.0f;
        if (fabsf(MotionAccel_n[1]) < 0.02f) MotionAccel_n[1] = 0.0f;
        if (fabsf(MotionAccel_n[2]) < 0.04f) MotionAccel_n[2] = 0.0f;

        printf("motionacc 0: %.2f, 1: %.2f, 2: %.2f\n", MotionAccel_b[0], MotionAccel_b[1], MotionAccel_b[2]);
        // 卡尔曼滤波融合加速度计与轮速（参考observe_task.c）
        xvEstimateKF_Update(&vaEstimateKF, -MotionAccel_b[0], wheel_v);
        v_filter = vel_acc[0];                  // 滤波后速度
        x_filter += v_filter * DT;              // 积分得位移

        printf("target_x: %.3f x: %.3f\n", target_x_ref, x_filter);
        printf("target_v: %.3f v: %.3f\n", smooth_target_v, v_filter);

        float yaw_error = turn_set - yaw;
        if (yaw_error > PI) yaw_error -= 2.0f * PI;
        else if (yaw_error < -PI) yaw_error += 2.0f * PI;

        // 期望偏航角速度（rad/s）
        yaw_rate_cmd = turn_pid.kp * yaw_error - turn_pid.kd * yaw_rate;

        // 计算旋转时的离心力补偿
        float mass = 1.0f;  // 机器人总质量（需要根据实际调整）
        float com_height = target_L0;  // 质心高度（用腿长近似）
        centrifugal_force = mass * com_height * yaw_rate_cmd * yaw_rate_cmd;

        // 计算俯仰角补偿（用于抵抗离心力）
        // 当向左转时，机器人需要向右倾斜一点
        pitch_compensation = atan2f(centrifugal_force, mass * 9.81f);

        // 根据旋转方向调整符号
        if (yaw_rate_cmd > 0) {
            // 向左转，左腿需要更负，右腿需要更正
            pitch_compensation = -pitch_compensation;
        }

        float pitch_L = -pitch;
        float pitch_rate_L = -pitch_rate;
        VMC_calc_1(left_leg, pitch_L, pitch_rate_L, DT);
        err_L[0] = left_leg->theta - 0.0f;
        err_L[1] = left_leg->d_theta - 0.0f;
        err_L[2] = (target_x_ref - x_filter);
        err_L[3] = (smooth_target_v - v_filter);
        err_L[4] = (pitch_L - pitch_compensation);
        err_L[5] = (pitch_rate_L - 0.0f);

        LQR_Calc(lqr_out_L, lqr_K, err_L);
        left_leg->Tp = lqr_out_L[1];

        float pitch_R = pitch;
        float pitch_rate_R = pitch_rate;
        VMC_calc_1(right_leg, pitch_R, pitch_rate_R, DT);
        printf("left theta: %.3f, right theta: %.3f\n", left_leg->theta * 180.0f / PI, right_leg->theta * 180.0f / PI);

        // 防劈叉补偿
        float theta_err = 0.0f - (left_leg->theta + right_leg->theta);
        float leg_tp = PID_Calc(&tp_pid, theta_err, 0.0f);
        // left_leg->Tp += leg_tp;
        // right_leg->Tp += leg_tp;

        // 横滚角补偿
        float roll_f0 = roll_pid.kp * (roll_set - roll) - roll_pid.kd * roll_rate;

        float cos_theta_L = cosf(left_leg->theta);
        left_leg->F0 = MG / cos_theta_L + PID_Calc(&leg_l_pid, left_leg->L0, target_L0);
        left_leg->F0 -= roll_f0; // 左腿减去roll_f0

        VMC_calc_2(left_leg);

        err_R[0] = right_leg->theta - 0.0f;
        err_R[1] = right_leg->d_theta - 0.0f;
        err_R[2] = (x_filter - target_x_ref);
        err_R[3] = (v_filter - smooth_target_v);
        err_R[4] = (pitch_R + pitch_compensation);
        err_R[5] = (pitch_rate_R - 0.0f);

        LQR_Calc(lqr_out_R, lqr_K, err_R);
        right_leg->Tp = lqr_out_R[1] + leg_tp;

        float cos_theta_R = cosf(right_leg->theta);
        right_leg->F0 = MG / cos_theta_R + PID_Calc(&leg_r_pid, right_leg->L0, target_L0);
        right_leg->F0 += roll_f0; // 右腿加上roll_f0

        VMC_calc_2(right_leg);

        if (left_leg->torque_set[0] > TORCH_MAX) left_leg->torque_set[0] = TORCH_MAX;
        if (left_leg->torque_set[0] < -TORCH_MAX) left_leg->torque_set[0] = -TORCH_MAX;
        if (left_leg->torque_set[1] > TORCH_MAX) left_leg->torque_set[1] = TORCH_MAX;
        if (left_leg->torque_set[1] < -TORCH_MAX) left_leg->torque_set[1] = -TORCH_MAX;

        if (right_leg->torque_set[0] > TORCH_MAX) right_leg->torque_set[0] = TORCH_MAX;
        if (right_leg->torque_set[0] < -TORCH_MAX) right_leg->torque_set[0] = -TORCH_MAX;
        if (right_leg->torque_set[1] > TORCH_MAX) right_leg->torque_set[1] = TORCH_MAX;
        if (right_leg->torque_set[1] < -TORCH_MAX) right_leg->torque_set[1] = -TORCH_MAX;

        wb_motor_set_torque(joint_LF, left_leg->torque_set[0]);
        wb_motor_set_torque(joint_LB, left_leg->torque_set[1]);
        wb_motor_set_torque(joint_RF, right_leg->torque_set[1]);
        wb_motor_set_torque(joint_RB, right_leg->torque_set[0]);

        // 偏航角补偿 (Turn)
        float yaw_err = turn_set - yaw;
        // 应对 yaw_err 的跳变（循环限幅求最短路径）
        if (yaw_err > PI) yaw_err -= 2.0f * PI;
        else if (yaw_err < -PI) yaw_err += 2.0f * PI;

        float turn_T = turn_pid.kp * yaw_err - turn_pid.kd * yaw_rate;

        printf("target len: %.3f, left len: %.3f, right len: %.3f\n", target_L0, left_leg->L0, right_leg->L0);
        float wheel_torque_L, wheel_torque_R;

        if (fabs(yaw_rate_cmd) > 0.01f) {  // 有旋转指令时
            // 前馈力矩：用于产生旋转
            float feedforward_torque = 0.5f * yaw_rate_cmd;  // 比例系数需要调整

            // 反馈力矩：LQR输出用于平衡
            float feedback_L = lqr_out_L[0];
            float feedback_R = lqr_out_R[0];

            // 组合：前馈用于旋转，反馈用于平衡
            wheel_torque_L = feedback_L + feedforward_torque;
            wheel_torque_R = feedback_R - feedforward_torque;  // 差速
        } else {
            // 无旋转时，只用LQR输出
            wheel_torque_L = lqr_out_L[0];
            wheel_torque_R = lqr_out_R[0];
        }

        if (wheel_torque_L > WHEEL_TORCH_MAX) wheel_torque_L = WHEEL_TORCH_MAX;
        if (wheel_torque_L < -WHEEL_TORCH_MAX) wheel_torque_L = -WHEEL_TORCH_MAX;
        if (wheel_torque_R > WHEEL_TORCH_MAX) wheel_torque_R = WHEEL_TORCH_MAX;
        if (wheel_torque_R < -WHEEL_TORCH_MAX) wheel_torque_R = -WHEEL_TORCH_MAX;

        printf("L_err:[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n", err_L[0], err_L[1], err_L[2], err_L[3], err_L[4], err_L[5]);
        printf("R_err:[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n", err_R[0], err_R[1], err_R[2], err_R[3], err_R[4], err_R[5]);
        printf("wheel_L_torque: %.3f, wheel_R_torque: %.3f, turn_T: %.3f, raw: %.3f,%.3f\n", wheel_torque_L, wheel_torque_R, turn_T, lqr_out_L[0], lqr_out_R[0]);
        wb_motor_set_torque(wheel_L, wheel_torque_L);
        wb_motor_set_torque(wheel_R, wheel_torque_R);

        printf("\n\n");
    }

    wb_robot_cleanup();
    return 0;
}