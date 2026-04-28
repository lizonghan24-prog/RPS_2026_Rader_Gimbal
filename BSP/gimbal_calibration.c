#include "gimbal_calibration.h"

// 数据记录数组
double calib_record_enc[CALIB_MAX_POINTS];
double calib_record_imu[CALIB_MAX_POINTS];
double calib_record_torque[CALIB_MAX_POINTS];
uint32_t calib_record_index = 0;

CalibState_e calib_current_state = CALIB_STATE_IDLE;

// 内部运动规划变量
static double target_enc_angle = 0.0;
static double target_enc_speed = 0.0;
static uint16_t record_div_counter = 0;
static uint16_t pause_counter = 0;

// 坐标系转换临时变量
static double temp_enc_offset = 0.0; 
static double target_max_enc = 0.0;
static double target_min_enc = 0.0;

// 每毫秒的角度步进量
static const double angle_step_per_ms = CALIB_SWEEP_SPEED / CALIB_CTRL_FREQ;

void Gimbal_Calib_Init(void) {
    calib_current_state = CALIB_STATE_IDLE;
    calib_record_index = 0;
    record_div_counter = 0;
    target_enc_angle = 0.0;
    target_enc_speed = 0.0;
}

void Gimbal_Calib_Start(void) {
    if (calib_current_state == CALIB_STATE_IDLE || calib_current_state == CALIB_STATE_DONE) {
        calib_current_state = CALIB_STATE_INIT;
        calib_record_index = 0;
    }
}

void Gimbal_Calib_Stop(void) {
    calib_current_state = CALIB_STATE_DONE;
}

// 核心函数 (必须放在 TIM6 1000Hz 中断里执行)
double Gimbal_Calib_Update_1kHz(double current_enc, double current_spd, double current_imu) {
    double out_torque = 0.0;
    double err_p, err_d;

    switch (calib_current_state) {
        case CALIB_STATE_IDLE:
        case CALIB_STATE_DONE:
            out_torque = 0.0;
            target_enc_speed = 0.0;
            break;

        case CALIB_STATE_INIT:
            // 自动抓取此时的坐标系差值
            
            temp_enc_offset = current_enc - current_imu;
            target_max_enc = CALIB_MAX_IMU_ANGLE + temp_enc_offset;
            target_min_enc = CALIB_MIN_IMU_ANGLE + temp_enc_offset;

            
            target_enc_angle = current_enc;
            target_enc_speed = 0.0;
            calib_current_state = CALIB_STATE_MOVE_TO_TOP;
            break;

        case CALIB_STATE_MOVE_TO_TOP:
            // 引导雷达到最高点 (不记录数据)
            if (target_enc_angle < target_max_enc) {
                target_enc_angle += angle_step_per_ms;
                target_enc_speed = CALIB_SWEEP_SPEED;
                if (target_enc_angle >= target_max_enc) {
                    target_enc_angle = target_max_enc;
                    pause_counter = 0;
                    calib_current_state = CALIB_STATE_PAUSE_TOP;
                }
            } else {
                target_enc_angle -= angle_step_per_ms;
                target_enc_speed = -CALIB_SWEEP_SPEED;
                if (target_enc_angle <= target_max_enc) {
                    target_enc_angle = target_max_enc;
                    pause_counter = 0;
                    calib_current_state = CALIB_STATE_PAUSE_TOP;
                }
            }
            break;

        case CALIB_STATE_PAUSE_TOP:
            target_enc_speed = 0.0; // 悬停静压
            pause_counter++;
            // 停留 2 秒 (2000 ms) 让机械震荡和线缆弹性完全平息
            if (pause_counter >= 2000) {
                target_enc_speed = -CALIB_SWEEP_SPEED;
                calib_current_state = CALIB_STATE_SWEEP_DOWN;
            }
            break;

        case CALIB_STATE_SWEEP_DOWN:
            target_enc_speed = -CALIB_SWEEP_SPEED;
            target_enc_angle -= angle_step_per_ms;
            
            if (target_enc_angle <= target_min_enc) {
                target_enc_angle = target_min_enc;
                pause_counter = 0;
                calib_current_state = CALIB_STATE_PAUSE_BOTTOM;
            }
            break;

        case CALIB_STATE_PAUSE_BOTTOM:
            target_enc_speed = 0.0;
            pause_counter++;
            if (pause_counter >= 2000) {
                target_enc_speed = CALIB_SWEEP_SPEED;
                calib_current_state = CALIB_STATE_SWEEP_UP;
            }
            break;

        case CALIB_STATE_SWEEP_UP:
            target_enc_speed = CALIB_SWEEP_SPEED;
            target_enc_angle += angle_step_per_ms;
            
            if (target_enc_angle >= target_max_enc) {
                target_enc_angle = target_max_enc;
                calib_current_state = CALIB_STATE_DONE; 
            }
            break;
    }

    
    if (calib_current_state != CALIB_STATE_IDLE && calib_current_state != CALIB_STATE_DONE) {
        err_p = target_enc_angle - current_enc;
        err_d = target_enc_speed - current_spd; 
        
        
        out_torque = (CALIB_KP * err_p) + (CALIB_KD * err_d);

        
        if (calib_current_state == CALIB_STATE_SWEEP_UP || calib_current_state == CALIB_STATE_SWEEP_DOWN) {
            record_div_counter++;
            if (record_div_counter >= CALIB_RECORD_DIV) {
                record_div_counter = 0;
                if (calib_record_index < CALIB_MAX_POINTS) {
                    // 同步截取同一时刻的绝对编码器值、绝对IMU值、输出扭矩
                    calib_record_enc[calib_record_index]    = current_enc;
                    calib_record_imu[calib_record_index]    = current_imu;
                    calib_record_torque[calib_record_index] = out_torque;
                    calib_record_index++;
                }
            }
        }
    }

    return out_torque;
}
