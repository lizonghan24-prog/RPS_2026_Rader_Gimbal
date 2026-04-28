#ifndef __HI_IMU_H__
#define __HI_IMU_H__

#include "main.h"   
#include <stdint.h>

#define HI_IMU_USE_H7_DCACHE  1



/**
 * @brief 供应用层调用的IMU 数据结构
 */
#pragma pack(push, 1)  // 强制单字节对齐
typedef struct {
    uint8_t  tag;           // 偏移 0 (0x91)
    uint16_t main_status;   // 偏移 1-2
    int8_t   temperature;   // 偏移 3
    float    air_pressure;  // 偏移 4-7
    uint32_t system_time;   // 偏移 8-11
    
    float    acc_x;         // 偏移 12-15
    float    acc_y;         // 偏移 16-19
    float    acc_z;         // 偏移 20-23
    
    float    gyr_x;         // 偏移 24-27
    float    gyr_y;         // 偏移 28-31
    float    gyr_z;         // 偏移 32-35
    
    uint8_t  reserved[12];  // 偏移 36-47 
    
    float    roll;          // 偏移 48-51
    float    pitch;         // 偏移 52-55
    float    yaw;           // 偏移 56-59
} CH040_Raw_t;
#pragma pack(pop)


/**
 * @brief 原始协议包负载结构体 
 * @note  
 */
#pragma pack(push, 1) 
typedef struct {
    uint8_t  tag;         // 帧头标识 
    
       
    // uint8_t  id;       // 预留
    
    float    eul[3];      // 欧拉角: Pitch, Roll, Yaw
    float    acc[3];      // 加速度: X, Y, Z
    float    gyr[3];      // 角速度: X, Y, Z
    float    mag[3];      // 磁力计: X, Y, Z
    float    quat[4];     // 四元数: q0, q1, q2, q3
    float    pres;        // 气压计
    uint32_t timestamp;   // 时间戳
} Hi91_Raw_t;
#pragma pack(pop)    


extern CH040_Raw_t hi_imu_data;




void HiIMU_Init(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t buffer_size);


void HiIMU_RxIdleCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* __HI_IMU_H__ */
