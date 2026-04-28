#ifndef __GIMBAL_CALIBRATION_H
#define __GIMBAL_CALIBRATION_H

#include <stdint.h>

#define CALIB_MAX_IMU_ANGLE    10.0   // 物理最高角度 
#define CALIB_MIN_IMU_ANGLE   -20.0   // 物理最低角度 
#define CALIB_SWEEP_SPEED       2.0   
#define CALIB_CTRL_FREQ      1000.0   // 控制循环频率 1000Hz

#define CALIB_KP   40.0  
#define CALIB_KD    2.0  

// 降采样记录：每 10ms 记录一次数据 (100Hz)
#define CALIB_RECORD_DIV   10
// 数组总大小
#define CALIB_MAX_POINTS   4000 

// 标定状态机
typedef enum {
    CALIB_STATE_IDLE = 0,
    CALIB_STATE_INIT,            
    CALIB_STATE_MOVE_TO_TOP,     
    CALIB_STATE_PAUSE_TOP,      
    CALIB_STATE_SWEEP_DOWN,     
    CALIB_STATE_PAUSE_BOTTOM,    
    CALIB_STATE_SWEEP_UP,        
    CALIB_STATE_DONE            
} CalibState_e;

// ozone数组
extern double calib_record_enc[CALIB_MAX_POINTS];
extern double calib_record_imu[CALIB_MAX_POINTS];
extern double calib_record_torque[CALIB_MAX_POINTS];
extern uint32_t calib_record_index;
extern CalibState_e calib_current_state;

// 函数声明
void Gimbal_Calib_Init(void);
void Gimbal_Calib_Start(void);
void Gimbal_Calib_Stop(void);

// 核心 1kHz 标定刷新函数
double Gimbal_Calib_Update_1kHz(double current_enc, double current_spd, double current_imu);

#endif