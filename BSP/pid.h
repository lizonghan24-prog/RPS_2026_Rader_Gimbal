#ifndef __PID_H
#define __PID_H

#include "main.h"

// 18-bit 编码器参数
#define ENCODER_MAX_RANGE  65535.0
#define ENCODER_HALF_RANGE 32767.5
#define PI 3.14159265

// === 调试模式枚举 ===
typedef enum {
    DEBUG_STOP = 0,         // 0: 停止
    DEBUG_SPEED_LOOP,       // 1: 调速度环
    DEBUG_ANGLE_LOOP,
	DEBUG_SWEEP,	// 2: 调角度环
	DEBUG_POINTMODE
} Debug_Mode_e;

// === 全局调试参数包 (把这个加入 Watch Window 就能控制一切) ===
typedef struct {
    Debug_Mode_e Control_Mode; // 改这个切换模式 (0, 1, 2)
//    float Speed_Sine_Amp;      // 正弦波幅度 (最大速度, 单位 dps) - 对应之前的 Target_Speed
//    float Speed_Sine_Freq;     // 正弦波频率 (摆动快慢, 单位 Hz)
	double target_speed;
    double Pitch_Target_Angle;        // 模式2时，改这里控制角度
    double Yaw_Target_Angle;
    // Pitch 轴 PID 参数 (实时修改生效)
    double Pitch_Speed_Kp, Pitch_Speed_Ki, Pitch_Speed_Kd,Pitch_Speed_kk,Pitch_Speed_D_filter;
    double Pitch_Angle_Kp, Pitch_Angle_Ki, Pitch_Angle_Kd,Pitch_Angle_kk;
    
    // Yaw 轴 PID 参数
    double Yaw_Speed_Kp,   Yaw_Speed_Ki,   Yaw_Speed_Kd ,Yaw_Speed_KK,Yaw_Speed_D_filter;
    double Yaw_Angle_Kp,   Yaw_Angle_Ki,   Yaw_Angle_Kd ,Yaw_Angle_KK;
    double Sweep_Freq; // 目标频率 (Hz)
    double Sweep_Amp;  // 电流/力矩幅值 (对于 MF7025，建议从小开始)
    double Sweep_Time; // 运行时间
	double sine_target_speed;
	double Actual_Speed;
} Gimbal_Debug_t;

// 内部 PID 结构体
typedef struct {
    double Kp, Ki, Kd,kk;
    double Error,Last_Error,P_Out, I_Out, D_Out, Output,last_target,target,last_D_out,D_filter;
    double Last_Measure, I_Limit, Max_Output;
} Gimbal_PID_t;

// 电机对象
typedef struct {
    double Angle_Total;     // 当前总角度 (监视这个看电机转到了哪)
    double Last_Raw_Enc;
    int32_t Round_Count;
    Gimbal_PID_t Angle_PID;
    Gimbal_PID_t Speed_PID;
} Gimbal_Motor_Handle_t;

extern Gimbal_Motor_Handle_t Pitch_Motor;
extern Gimbal_Motor_Handle_t Yaw_Motor;
extern Gimbal_Debug_t Gimbal_Debug;

void Gimbal_Init(void);
double Gimbal_Control_Loop(Gimbal_Motor_Handle_t *motor, int axis_id, double current_raw_enc, double current_speed);

#endif
