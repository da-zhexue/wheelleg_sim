#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "vmc.h"
#include "lqr.h"
#include "kalman_filter.h"
#include "keyboard.h"

const float lqr_K[12] = {
    -18.5289f, -3.8021f, -1.2500f, -40.6604f, -270.6452f, -67.8242f,
   62.5455f, 2.3431f, 0.0385f, 1.2460f, 17.6498f, 1.7696f
};
// const float lqr_K[12] = {
//     -18.5289f, -3.8021f, -1.2500f, -40.6604f, -270.6452f, -67.8242f,
//    62.5455f, 2.3431f, 0.0385f, 1.2460f, 17.6498f, 1.7696f
// };
// const float lqr_K[12] = {
//     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
//     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
// };
#define FILTER_ALPHA 0.02f // 控制速度低通滤波
#define V_MAX 2.0f // 控制速度
#define W_MAX 0.005f // 控制转向角速度
#define L_DELTA_MAX 0.002f // 腿长变化速度

#define TIME_STEP 4 // 控制周期
#define DT ((float)TIME_STEP / 1000.0f)

#define L1 0.64f // 大腿长
#define L2 0.8f // 小腿长
#define TORQUE_MAX 30.0f // 关节电机力矩限制
#define WHEEL_TORQUE_MAX 3.0f // 轮毂电机力矩限制
#define MG 6.5f // 机器人总重力的一半(单腿承重)
#define WHEEL_RAD 0.15f // 轮子半径
#define ACCEL_LPF 0.0089f // 加速度低通滤波系数

float pitch_compensation = 0.028f; // 旋转时的俯仰角补偿

// 一个简单的PID实现
typedef struct {
    float kp, ki, kd;
    float err, last_err, integral;
    float max;
} PID_Controller;

float PID_Calc(PID_Controller *pid, float current, float target) {
    pid->err = target - current;
    pid->integral += pid->err * DT;
    float derivative = (pid->err - pid->last_err) / DT;
    pid->last_err = pid->err;
    float out = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * derivative;
    if (out > pid->max) out = pid->max;
    if (out < -pid->max) out = -pid->max;
    return out;
}

// x、v卡尔曼滤波器，用于获得一个较为准确的x、v
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

// 重力加速度（地球坐标系，Z轴向上）
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
 * @brief 地球坐标系 -> 机体坐标系
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
 * @brief 机体坐标系 -> 地球坐标系
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

FILE *csv_file = NULL;
int csv_initialized = 0;
char csv_filename[256];

// 新增：初始化CSV文件
void init_csv_logging() {
    // 生成带时间戳的文件名
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(csv_filename, sizeof(csv_filename),
             "log/robot_log_%Y%m%d_%H%M%S.csv", tm_info);

    csv_file = fopen(csv_filename, "w");
    if (csv_file == NULL) {
        printf("无法创建CSV文件!\n");
        return;
    }

    // 写入CSV表头
    fprintf(csv_file, "Time,Pitch(deg),PitchRate(rad/s),");
    //fprintf(csv_file, "YawTarget(deg),Yaw(deg),YawRate(rad/s),");
    fprintf(csv_file, "WheelVelocity,");
    fprintf(csv_file, "Target_X,Filtered_X,Target_V,Filtered_V,");
    fprintf(csv_file, "LeftTheta(deg),RightTheta(deg),LeftL0,RightL0,");
    //fprintf(csv_file, "JumpFlag,FallFlag,Turn_T,");
    fprintf(csv_file, "LeftTp,LeftF0,LeftTorque0,LeftTorque1,");
    fprintf(csv_file, "RightTp,RightF0,RightTorque0,RightTorque1,");
    fprintf(csv_file, "WheelTorque_L,WheelTorque_R,");
    fprintf(csv_file, "LQR0,LQR3,LQR4\n");

    csv_initialized = 1;
    printf("CSV日志文件已创建: %s\n", csv_filename);
}

// 新增：写入一行数据到CSV
void write_csv_row(float current_time,
                   float pitch_deg, float pitch_rate,
                   //float yaw_target_deg, float yaw_deg, float yaw_rate,
                   float wheel_v,
                   float target_x_ref, float x_filter,
                   float smooth_target_v, float v_filter,
                   float left_theta_deg, float right_theta_deg,
                   float left_L0, float right_L0,
                   //int jump_flag, int fall_flag, float turn_T,
                   float left_Tp, float left_F0, float left_torque0, float left_torque1,
                   float right_Tp, float right_F0, float right_torque0, float right_torque1,
                   float wheel_torque_L, float wheel_torque_R) {

    if (!csv_initialized || csv_file == NULL) return;

    fprintf(csv_file, "%.3f,", current_time);
    fprintf(csv_file, "%.2f,%.2f,", pitch_deg, pitch_rate);
    //fprintf(csv_file, "%.2f,%.2f,%.2f,", yaw_target_deg, yaw_deg, yaw_rate);
    fprintf(csv_file, "%.3f,", wheel_v);
    fprintf(csv_file, "%.3f,%.3f,%.3f,%.3f,", target_x_ref, x_filter, smooth_target_v, v_filter);
    fprintf(csv_file, "%.2f,%.2f,%.3f,%.3f,", left_theta_deg, right_theta_deg, left_L0, right_L0);
    //fprintf(csv_file, "%d,%d,%.3f,", jump_flag, fall_flag, turn_T);
    fprintf(csv_file, "%.2f,%.2f,%.2f,%.2f,", left_Tp, left_F0, left_torque0, left_torque1);
    fprintf(csv_file, "%.2f,%.2f,%.2f,%.2f,", right_Tp, right_F0, right_torque0, right_torque1);
    fprintf(csv_file, "%.3f,%.3f,", wheel_torque_L, wheel_torque_R);
    fprintf(csv_file, "%.3f,%.3f,%.3f\n", pitch_deg * lqr_K[6], v_filter * lqr_K[9], right_theta_deg* lqr_K[10]);

    // 刷新缓冲区，确保数据及时写入文件
    fflush(csv_file);
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

    // webots中使用力矩控制或者速度控制需要将位置设为INFINITY
    wb_motor_set_position(wheel_L, INFINITY);
    wb_motor_set_position(wheel_R, INFINITY);
    wb_motor_set_torque(wheel_L, 0.0);
    wb_motor_set_torque(wheel_R, 0.0);

    wb_motor_set_position(joint_LF, INFINITY);
    wb_motor_set_position(joint_LB, INFINITY);
    wb_motor_set_position(joint_RF, INFINITY);
    wb_motor_set_position(joint_RB, INFINITY);

    // 下面设定位置模式用于测试极性
    // wb_motor_set_position(joint_LF, 0);
    // wb_motor_set_position(joint_LB, 0);
    // wb_motor_set_position(joint_RF, 0);
    // wb_motor_set_position(joint_RB, 0);

    vmc_leg_t *left_leg = Get_VMC_Leg(LEFT);
    vmc_leg_t *right_leg = Get_VMC_Leg(RIGHT);
    VMC_init(left_leg, L1, L2);
    VMC_init(right_leg, L1, L2);

    PID_Controller leg_l_pid = {100.0f, 0.0f, 10.0f, 0, 0, 0, 30}; // 腿长PID
    PID_Controller leg_r_pid = {100.0f, 0.0f, 10.0f, 0, 0, 0, 30};

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

    init_csv_logging();
    // 跳跃状态机
    int jump_flag = 0;    // 0=正常, 1=下蹲压缩, 2=上升加速, 3=空中缩腿
    int jump_time = 0;
    float last_target_L0 = 0.9f;

    PID_Controller turn_pid = {1.5f, 0.0f, 0.3f, 0, 0, 0, 50};
    PID_Controller roll_pid = {1.0f, 0.0f, 0.0f, 0, 0, 0, 100};
    PID_Controller tp_pid = {30.0f, 0.0f, 1.0f, 0, 0, 0, 100};

    float current_time = 0.0f;
    int fall_time = 0;
    int fall_flag = 0;

    //ws前进后退，ad旋转，qe腿长调节
    Keyboard_Init(TIME_STEP, V_MAX, W_MAX, L_DELTA_MAX);

    xvEstimateKF_Init(&vaEstimateKF); // 初始化卡尔曼滤波器

    float filter_alpha = FILTER_ALPHA; // 一阶滤波系数，取值0~1，越小越平滑

    while (wb_robot_step(TIME_STEP) != -1) {
        current_time += (float)TIME_STEP / 1000.0f;
        int jump_trigger = 0;
        Keyboard_Update(&target_v, &target_L0, &turn_set, &jump_trigger);

        // 平滑加减速
        smooth_target_v = filter_alpha * target_v + (1.0f - filter_alpha) * smooth_target_v;

        target_x_ref += smooth_target_v * DT;

        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial); 
        // 至于为什么实际数据顺序和函数名不一样可能与安装方向有关
        double pitch = -rpy[0];
        double roll = rpy[1];
        double yaw = rpy[2];

        const double *gyro_vals = wb_gyro_get_values(gyro);
        double pitch_rate = -gyro_vals[0];
        double roll_rate = gyro_vals[1];
        double yaw_rate = gyro_vals[2];

        float w_pos_L = wb_position_sensor_get_value(ecd_wheel_L);
        float w_pos_R = wb_position_sensor_get_value(ecd_wheel_R);

        printf("pitch: %.2f, pitch_rate: %2f\n", pitch * 180.0f / PI, pitch_rate);
        printf("yaw_target: %.2f, yaw: %.2f, yaw_rate: %2f\n", turn_set * 180.0f / PI, yaw * 180.0f / PI, yaw_rate);
        left_leg->phi4 = wb_position_sensor_get_value(ecd_LF) + PI * 30.0f / 180.0f;
        left_leg->phi1 = wb_position_sensor_get_value(ecd_LB) + PI * 150.0f / 180.0f;
        right_leg->phi1 = wb_position_sensor_get_value(ecd_RF) + PI * 150.0f / 180.0f;
        right_leg->phi4 = wb_position_sensor_get_value(ecd_RB) + PI * 30.0f / 180.0f;
        // 轮腿左右腿建模是翻转的而不是对称，所以右腿phi1是前腿，左腿phi1是后腿

        // 轮子速度计算（用于卡尔曼滤波测量）
        // 这里不能用webots的获得轮子速度的函数，该函数似乎只能获得绝对值？ 
        x_l = w_pos_L / 2.0f * WHEEL_RAD;
        x_r = w_pos_R / 2.0f * WHEEL_RAD;
        v_l = (x_l - last_x_l) / DT;
        v_r = (x_r - last_x_r) / DT;
        float wheel_v = (v_l - v_r) / 2.0f;
        last_x_l = x_l;
        last_x_r = x_r;

        // 坐标系变换：用确认的yaw/pitch/roll计算四元数（因为欧拉角顺序不一样，不直接用webots的quaternion）
        float q[4];
        EulerToQuaternion((float)yaw, (float)pitch, (float)roll, q);

        const double *accel_vals = wb_accelerometer_get_values(accelerometer);
        float Accel_b[3] = {(float)accel_vals[1], (float)accel_vals[0], (float)accel_vals[2]};

        float gravity_b[3];
        EarthFrameToBodyFrame(GRAVITY, gravity_b, q);

        // 运动加速度 = 加速度计读数 - 重力投影（一阶低通滤波）
        for (uint8_t i = 0; i < 3; i++)
        {
            MotionAccel_b[i] = (Accel_b[i] - gravity_b[i]) * DT / (ACCEL_LPF + DT)
                             + MotionAccel_b[i] * ACCEL_LPF / (ACCEL_LPF + DT);
        }
        BodyFrameToEarthFrame(MotionAccel_b, MotionAccel_n, q);

        // 死区滤波
        if (fabsf(MotionAccel_n[0]) < 0.02f) MotionAccel_n[0] = 0.0f;
        if (fabsf(MotionAccel_n[1]) < 0.02f) MotionAccel_n[1] = 0.0f;
        if (fabsf(MotionAccel_n[2]) < 0.04f) MotionAccel_n[2] = 0.0f;

        // 卡尔曼滤波融合加速度计与轮速
        xvEstimateKF_Update(&vaEstimateKF, -MotionAccel_b[0], wheel_v);
        v_filter = vel_acc[0];                  // 滤波后速度
        x_filter += v_filter * DT;              // 积分得位移

        printf("accel: %.2f", MotionAccel_b[0]);
        printf("target_x: %.3f x: %.3f\n", target_x_ref, x_filter);
        printf("target_v: %.3f v: %.3f\n", smooth_target_v, v_filter);

        // 左腿VMC->LQR
        float pitch_L = -pitch;
        float pitch_rate_L = -pitch_rate;
        VMC_calc_1(left_leg, pitch_L, pitch_rate_L, DT);
        err_L[0] = left_leg->theta - 0.0f;
        err_L[1] = left_leg->d_theta - 0.0f;
        err_L[2] = (target_x_ref - x_filter);
        err_L[3] = (smooth_target_v - v_filter);
        err_L[4] = (pitch_L - pitch_compensation);
        err_L[5] = (pitch_rate_L - 0.0f);

        // 自创自救措施，倒地但是轮子还能够地时，LQR不计算速度误差
        // 目前自救过程还是有点抽象的，后续看看怎么改进
        // 后续还要增加判断倒地角度更大时要用别的自救策略
        if(pitch > 0.32f || pitch < -0.32f)
        {
            fall_flag = 1;
            fall_time = 0;
        }
        else
        {
            fall_time++;
            if(fall_time > 100)
            {
                fall_flag = 0;
            }
        }
        if(fall_flag)
        {

            err_L[2] = 0.0f;
            err_L[3] = 0.0f;
        }

        LQR_Calc(lqr_out_L, lqr_K, err_L);
        left_leg->Tp = lqr_out_L[1];

        // 右腿VMC->LQR
        float pitch_R = pitch;
        float pitch_rate_R = pitch_rate;
        VMC_calc_1(right_leg, pitch_R, pitch_rate_R, DT);
        printf("left theta: %.3f, right theta: %.3f\n", left_leg->theta * 180.0f / PI, right_leg->theta * 180.0f / PI);

        err_R[0] = right_leg->theta - 0.0f;
        err_R[1] = right_leg->d_theta - 0.0f;
        err_R[2] = (x_filter - target_x_ref);
        err_R[3] = (v_filter - smooth_target_v);
        err_R[4] = (pitch_R + pitch_compensation);
        err_R[5] = (pitch_rate_R - 0.0f);

        if(fall_flag)
        {
            err_R[2] = 0.0f;
            err_R[3] = 0.0f;
        }

        LQR_Calc(lqr_out_R, lqr_K, err_R);
        printf("theta: %.2f, v: %.2f, pitch: %.2f\n", lqr_K[0] * err_R[0], lqr_K[3] * err_R[3], lqr_K[4] * err_R[4]);
        right_leg->Tp = lqr_out_R[1];

        // 防劈叉补偿
        float theta_err = 0.0f - (left_leg->theta + right_leg->theta);
        float leg_tp = PID_Calc(&tp_pid, theta_err, 0.0f);
        // leg_tp = 0;
        left_leg->Tp += leg_tp;
        right_leg->Tp += leg_tp;

        // 横滚角补偿
        float roll_f0 = roll_pid.kp * (roll_set - roll) - roll_pid.kd * roll_rate;

        // 跳跃状态机，照搬达妙
        if (jump_trigger && jump_flag == 0) {
            jump_flag = 1;
            jump_time = 0;
            last_target_L0 = target_L0;  // 保存跳跃前目标腿长
        }

        float jump_target_L0;

        if (jump_flag == 1) {
            // 下蹲压缩阶段
            jump_target_L0 = 0.70f;
            left_leg->F0 = MG / cosf(left_leg->theta) + PID_Calc(&leg_l_pid, left_leg->L0, jump_target_L0);
            right_leg->F0 = MG / cosf(right_leg->theta) + PID_Calc(&leg_r_pid, right_leg->L0, jump_target_L0);

            if (left_leg->L0 < 0.70f && right_leg->L0 < 0.70f)
                jump_time++;
            if (jump_time >= 10) {
                jump_time = 0;
                jump_flag = 2;  // 进入上升加速阶段
            }
        } else if (jump_flag == 2) {
            // 上升加速阶段
            jump_target_L0 = 1.10f;
            left_leg->F0 = MG / cosf(left_leg->theta) + PID_Calc(&leg_l_pid, left_leg->L0, jump_target_L0);
            right_leg->F0 = MG / cosf(right_leg->theta) + PID_Calc(&leg_r_pid, right_leg->L0, jump_target_L0);

            if (left_leg->L0 > 1.00f && right_leg->L0 > 1.00f)
                jump_time++;
            if (jump_time >= 2) {
                jump_time = 0;
                jump_flag = 3;  // 进入空中缩腿阶段
            }
        } else if (jump_flag == 3) {
            // 空中缩腿阶段
            jump_target_L0 = 0.70f;
            left_leg->F0 = PID_Calc(&leg_l_pid, left_leg->L0, jump_target_L0);
            right_leg->F0 = PID_Calc(&leg_r_pid, right_leg->L0, jump_target_L0);

            if (left_leg->L0 < 0.70f && right_leg->L0 < 0.70f)
                jump_time++;
            if (jump_time >= 3) {
                jump_time = 0;
                target_L0 = last_target_L0;  // 恢复目标腿长
                jump_flag = 0;  // 跳跃结束
            }
        } else {
            // 正常模式
            left_leg->F0 = MG / cosf(left_leg->theta) + PID_Calc(&leg_l_pid, left_leg->L0, target_L0);
            right_leg->F0 = MG / cosf(right_leg->theta) + PID_Calc(&leg_r_pid, right_leg->L0, target_L0);
        }

        // 离地检测 
        int left_ground = ground_detection(left_leg, Accel_b[2]);
        int right_ground = ground_detection(right_leg, Accel_b[2]);

        // 离地特殊处理
        if ((left_ground && right_ground && jump_flag != 1 && jump_flag != 2) || jump_flag == 3) {
            // 两腿同时离地时（排除跳跃压缩和上升阶段），或跳跃缩腿阶段
            // 轮子扭矩清零，髋关节只保留theta/d_theta项
            lqr_out_L[0] = 0.0f;
            lqr_out_R[0] = 0.0f;
            left_leg->Tp = lqr_K[6] * (left_leg->theta - 0.0f) + lqr_K[7] * (left_leg->d_theta - 0.0f) + leg_tp;
            right_leg->Tp = lqr_K[6] * (right_leg->theta - 0.0f) + lqr_K[7] * (right_leg->d_theta - 0.0f) + leg_tp;
            // 重置位移积分防止饱和
            x_filter = 0.0f;
            target_x_ref = 0.0f;
        } else {
            // 没有离地时施加横滚角补偿
            if (jump_flag == 0) {
                left_leg->F0 -= roll_f0;
                right_leg->F0 += roll_f0;
            }
        }

        // F0限幅
        if (left_leg->F0 > 100.0f) left_leg->F0 = 100.0f;
        if (left_leg->F0 < -100.0f) left_leg->F0 = -100.0f;
        if (right_leg->F0 > 100.0f) right_leg->F0 = 100.0f;
        if (right_leg->F0 < -100.0f) right_leg->F0 = -100.0f;

        VMC_calc_2(left_leg);
        VMC_calc_2(right_leg);

        // 力矩限幅（跳跃时允许更大扭矩，但我懒得改电机里设置的最大力矩，所以这里力矩都限制为30）
        float torque_limit = (jump_flag >= 1 && jump_flag <= 3) ? TORQUE_MAX  : TORQUE_MAX;
        if (left_leg->torque_set[0] > torque_limit) left_leg->torque_set[0] = torque_limit;
        if (left_leg->torque_set[0] < -torque_limit) left_leg->torque_set[0] = -torque_limit;
        if (left_leg->torque_set[1] > torque_limit) left_leg->torque_set[1] = torque_limit;
        if (left_leg->torque_set[1] < -torque_limit) left_leg->torque_set[1] = -torque_limit;

        if (right_leg->torque_set[0] > torque_limit) right_leg->torque_set[0] = torque_limit;
        if (right_leg->torque_set[0] < -torque_limit) right_leg->torque_set[0] = -torque_limit;
        if (right_leg->torque_set[1] > torque_limit) right_leg->torque_set[1] = torque_limit;
        if (right_leg->torque_set[1] < -torque_limit) right_leg->torque_set[1] = -torque_limit;

        printf("left Tp: %.2f, F0: %.2f, torque0: %.2f, torque1: %.2f\n", left_leg->Tp, left_leg->F0, left_leg->torque_set[0], left_leg->torque_set[1]);
        printf("right Tp: %.2f, F0: %.2f, torque0: %.2f, torque1: %.2f\n", right_leg->Tp, right_leg->F0, right_leg->torque_set[0], right_leg->torque_set[1]);

        wb_motor_set_torque(joint_LF, left_leg->torque_set[1]);
        wb_motor_set_torque(joint_LB, left_leg->torque_set[0]);
        wb_motor_set_torque(joint_RF, right_leg->torque_set[0]);
        wb_motor_set_torque(joint_RB, right_leg->torque_set[1]);


        // 偏航角补偿 (Turn)
        float yaw_err = turn_set - yaw;
        // 应对 yaw_err 的跳变
        if (yaw_err > PI) yaw_err -= 2.0f * PI;
        else if (yaw_err < -PI) yaw_err += 2.0f * PI;

        float turn_T = turn_pid.kp * yaw_err - turn_pid.kd * yaw_rate;
        // turn_T = 0;

        printf("target len: %.3f, left len: %.3f, right len: %.3f\n", target_L0, left_leg->L0, right_leg->L0);
        //printf("jump_flag: %d, left_ground: %d, right_ground: %d\n", jump_flag, left_ground, right_ground);
        float wheel_torque_L, wheel_torque_R;

        if(fall_flag)
        {
            turn_T = 0.0f;
        }
        wheel_torque_L = lqr_out_L[0] - turn_T;
        wheel_torque_R = lqr_out_R[0] - turn_T;

        if (wheel_torque_L > WHEEL_TORQUE_MAX) wheel_torque_L = WHEEL_TORQUE_MAX;
        if (wheel_torque_L < -WHEEL_TORQUE_MAX) wheel_torque_L = -WHEEL_TORQUE_MAX;
        if (wheel_torque_R > WHEEL_TORQUE_MAX) wheel_torque_R = WHEEL_TORQUE_MAX;
        if (wheel_torque_R < -WHEEL_TORQUE_MAX) wheel_torque_R = -WHEEL_TORQUE_MAX;

        //printf("wheel_L_torque: %.3f, wheel_R_torque: %.3f, turn_T: %.3f, raw: %.3f,%.3f\n", wheel_torque_L, wheel_torque_R, turn_T, lqr_out_L[0], lqr_out_R[0]);
        wb_motor_set_torque(wheel_L, wheel_torque_L);
        wb_motor_set_torque(wheel_R, wheel_torque_R);

        printf("\n\n");
        // 写入CSV行
        write_csv_row(current_time,
                     pitch * 180.0f / PI, pitch_rate,
                     // turn_set * 180.0f / PI, yaw * 180.0f / PI, yaw_rate,
                     wheel_v,
                     target_x_ref, x_filter,
                     smooth_target_v, v_filter,
                     left_leg->theta * 180.0f / PI, right_leg->theta * 180.0f / PI,
                     left_leg->L0, right_leg->L0,
                     // jump_flag, fall_flag, turn_T,
                     left_leg->Tp, left_leg->F0, left_leg->torque_set[0], left_leg->torque_set[1],
                     right_leg->Tp, right_leg->F0, right_leg->torque_set[0], right_leg->torque_set[1],
                     wheel_torque_L, wheel_torque_R);
    }
    if (csv_file != NULL) {
        fclose(csv_file);
        printf("CSV日志文件已保存: %s\n", csv_filename);
    }
    wb_robot_cleanup();
    return 0;
}