#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#include "stm32h7xx.h"                  // Device header

// 电机反馈数据与状态结构体
typedef struct {
    int16_t Angle_Kp;
    int16_t Angle_Ki;
    int16_t Speed_Kp;
    int16_t Speed_Ki;
    int16_t Iq_Kp;
    int16_t Iq_Ki;
    
    int32_t Accel;
    
    uint16_t Encoder;       // 编码器原始值 0~65535
    uint16_t Encoder_raw;
    uint16_t Encoder_Offset;
    
    double Motor_Angle;    // 多圈角度
    double Circle_Angle;     // 单圈角度 0-360 度
    
    int8_t Temp;            // 温度
    double Voltage;          // 电压
    uint8_t Error_state;    // 错误标志位
    
    // 状态枚举
    enum {Vol_Normal = 0, Vol_Low} Voltage_State;
    enum {Temp_Normal = 0, Over_Temp} Temp_State;
    
    double T;            // 当前转矩
    double LK_Speed;     // 当前转速 (dps)
    int16_t Power;      // 功率
} LK_M_t;

// 云台 Yaw 轴电机全局变量
extern LK_M_t LK_M_Gimbal_Yaw;

// ================== 参数读取指令 ==================

// 读取 PID 参数
void LK_M_Read_Pid_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 读取加速度数据
void LK_M_Read_Acc_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 读取编码器原始数据
void LK_M_Read_Encoder_Data_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 读取多圈角度
void LK_M_Read_MulAngle_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 读取单圈角度
void LK_M_Read_circleAngle_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);

// ================== 状态读取指令 ==================

// 读取电机状态 1 (温度、电压、错误)
void LK_Read_Motor_State_1_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 读取电机状态 2 (温度、转矩、速度、编码器)
void LK_Read_Motor_State_2_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 读取电机状态 3 (温度、相电流)
void LK_Read_Motor_State_3_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);

// ================== 维护与设置指令 ==================

// 多圈角度清零 (设置当前位置为零点)
void LK_M_Angle_Clear_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 清除错误标志
void LK_M_Wrong_Flag_Clear_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 写入 PID 参数到 RAM (掉电丢失)
void LK_M_Write_Pid_2_RAM(FDCAN_HandleTypeDef *hfdcan, int16_t id, int8_t P_p, int8_t I_p, int8_t P_s, int8_t I_s, int8_t P_t, int8_t I_t);
// 写入 PID 参数到 ROM (掉电保存)
void LK_M_Write_Pid_2_ROM(FDCAN_HandleTypeDef *hfdcan, int16_t id, int8_t P_p, int8_t I_p, int8_t P_s, int8_t I_s, int8_t P_t, int8_t I_t);
// 写入加速度到 RAM
void LK_M_Write_Acc_2_RAM(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t Accel);
// 设置特定编码器偏移值到 ROM
void LK_M_Write_POS_0_2_ROM(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t encoderOffset);
// 把当前位置设为零点并写入 ROM
void LK_M_Write_Current_POS_As_0_2_ROM(FDCAN_HandleTypeDef *hfdcan, int16_t id);

// ================== 启停控制指令 ==================

// 关闭电机输出
void LK_M_Power_Off(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 停止电机 (保持刹车状态)
void LK_M_Stop(FDCAN_HandleTypeDef *hfdcan, int16_t id);
// 开启电机输出
void LK_M_Power_On(FDCAN_HandleTypeDef *hfdcan, int16_t id);

// ================== 闭环控制指令 ==================

// 转矩闭环控制 (输入目标转矩值)
void LK_M_Close_Loop_Control_T(FDCAN_HandleTypeDef *hfdcan, int16_t id, double TControl);
// 转矩闭环控制 (输入目标 Iq 电流值)
void LK_M_Close_Loop_Control_iq(FDCAN_HandleTypeDef *hfdcan, int16_t id, double iqControl);
// 多电机转矩控制 (一次控制4个电机)
void LK_M_Mul_Motor_Close_Loop_Control_T(FDCAN_HandleTypeDef *hfdcan, int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4);
// 速度闭环控制
void LK_M_Close_Loop_Control_Speed(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t speedControl);
// 位置闭环控制 1 (多圈绝对位置)
void LK_M_Close_Loop_Control_Angle_1(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t angleControl);
// 位置闭环控制 2 (多圈绝对位置 + 最大速度限制)
void LK_M_Close_Loop_Control_Angle_2(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t maxSpeed, int32_t angleControl);
// 位置闭环控制 3 (单圈绝对位置)
void LK_M_Close_Loop_Control_Angle_3(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t angleControl);
// 位置闭环控制 4 (单圈绝对位置 + 最大速度限制)
void LK_M_Close_Loop_Control_Angle_4(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t max_Speed, uint16_t angleControl);
// 位置闭环控制 5 (增量位置)
void LK_M_Close_Loop_Control_Angle_5(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t angleControl);
// 位置闭环控制 6 (增量位置 + 最大速度限制)
void LK_M_Close_Loop_Control_Angle_6(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t max_Speed, int32_t angleControl);

// ================== 数据处理 ==================

// 接收数据处理函数，在 FDCAN 回调中调用
void LK_M_Data_Process(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data, int16_t id, LK_M_t* LK_M);

#endif
