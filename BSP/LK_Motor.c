#include "main.h"
#include <math.h>



// ================================================================
#define LK_ID_BASE              0x140
#define LK_TORQUE_COEF          403.44 // 转矩转电流系数

// LK电机命令字
#define CMD_READ_PID            0x30
#define CMD_WRITE_PID_RAM       0x31
#define CMD_WRITE_PID_ROM       0x32
#define CMD_READ_ACC            0x33
#define CMD_WRITE_ACC_RAM       0x34
#define CMD_DATA_WRITE          0x40
#define CMD_MOTOR_OFF           0x80
#define CMD_MOTOR_STOP          0x81
#define CMD_MOTOR_ON            0x88
#define CMD_READ_ENC            0x90
#define CMD_WRITE_POS_ROM       0x91
#define CMD_READ_MULTI_ANGLE    0x92
#define CMD_READ_SINGLE_ANGLE   0x94
#define CMD_CLEAR_ANGLE         0x95
#define CMD_READ_STATE_1        0x9A
#define CMD_CLEAR_ERROR         0x9B
#define CMD_READ_STATE_2        0x9C
#define CMD_READ_STATE_3        0x9D
#define CMD_CONTROL_TORQUE      0xA1
#define CMD_CONTROL_SPEED       0xA2
#define CMD_CONTROL_POS_1       0xA3
#define CMD_CONTROL_POS_2       0xA4
#define CMD_CONTROL_POS_3       0xA5
#define CMD_CONTROL_POS_4       0xA6
#define CMD_CONTROL_POS_5       0xA7
#define CMD_CONTROL_POS_6       0xA8

// 浮点数限幅
static double double_Constrain(double val, double min, double max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief  FDCAN 底层发送函数 (核心适配部分)
 * @param  hfdcan: FDCAN 句柄
 * @param  std_id: 标准帧 ID
 * @param  data:   发送数据指针
 * @param  len:    数据长度
 */
static void LK_Send_Packet(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id, uint8_t *data, uint8_t len) {
    FDCAN_TxHeaderTypeDef TxHeader;
    
    // 配置发送头
    TxHeader.Identifier = std_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    
    switch(len) {
        case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
        case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
    }
    
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 不开启波特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 经典 CAN 模式 (LK电机通常不支持 CANFD)
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 等待 FIFO 有空闲空间 (模拟原代码的 while wait)
    // 防止发送过快导致丢包
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) 
{
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}
}


void LK_M_Read_Pid_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_PID;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Read_Acc_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_ACC;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Read_Encoder_Data_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_ENC;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Read_MulAngle_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_MULTI_ANGLE;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Read_circleAngle_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_SINGLE_ANGLE;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_Read_Motor_State_1_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_STATE_1;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_Read_Motor_State_2_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_STATE_2;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_Read_Motor_State_3_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_READ_STATE_3;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Angle_Clear_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_CLEAR_ANGLE;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);	
}

void LK_M_Wrong_Flag_Clear_Request(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_CLEAR_ERROR;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Write_Pid_2_RAM(FDCAN_HandleTypeDef *hfdcan, int16_t id, int8_t P_p, int8_t I_p, int8_t P_s, int8_t I_s, int8_t P_t, int8_t I_t) {
    uint8_t data[8];
    data[0] = CMD_WRITE_PID_RAM;
    data[1] = 0;
    data[2] = (uint8_t)P_p; data[3] = (uint8_t)I_p;
    data[4] = (uint8_t)P_s; data[5] = (uint8_t)I_s;
    data[6] = (uint8_t)P_t; data[7] = (uint8_t)I_t;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Write_Pid_2_ROM(FDCAN_HandleTypeDef *hfdcan, int16_t id, int8_t P_p, int8_t I_p, int8_t P_s, int8_t I_s, int8_t P_t, int8_t I_t) {
    uint8_t data[8];
    data[0] = CMD_WRITE_PID_ROM;
    data[1] = 0;
    data[2] = (uint8_t)P_p; data[3] = (uint8_t)I_p;
    data[4] = (uint8_t)P_s; data[5] = (uint8_t)I_s;
    data[6] = (uint8_t)P_t; data[7] = (uint8_t)I_t;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Write_Acc_2_RAM(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t Accel) {
    uint8_t data[8];
    data[0] = CMD_WRITE_ACC_RAM;
    data[1] = 0; data[2] = 0; data[3] = 0;
    data[4] = (uint8_t)(Accel & 0xff);
    data[5] = (uint8_t)((Accel >> 8) & 0xff);
    data[6] = (uint8_t)((Accel >> 16) & 0xff);
    data[7] = (uint8_t)((Accel >> 24) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Write_POS_0_2_ROM(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t encoderOffset) {
    uint8_t data[8] = {0};
    data[0] = CMD_WRITE_POS_ROM;
    data[6] = (uint8_t)(encoderOffset & 0xff);
    data[7] = (uint8_t)((encoderOffset >> 8) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Write_Current_POS_As_0_2_ROM(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = 0x19;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Power_Off(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_MOTOR_OFF;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Stop(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_MOTOR_STOP;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Power_On(FDCAN_HandleTypeDef *hfdcan, int16_t id) {
    uint8_t data[8] = {0};
    data[0] = CMD_MOTOR_ON;
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}


void LK_M_Close_Loop_Control_T(FDCAN_HandleTypeDef *hfdcan, int16_t id, double TControl) {
    // 限制 Iq 范围 -2000 ~ 2000
    int16_t iq = (int16_t)(LK_TORQUE_COEF * TControl);
    if (iq > 2000) iq = 2000;
    else if (iq < -2000) iq = -2000;

    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_TORQUE;
    data[4] = (uint8_t)(iq & 0xff);
    data[5] = (uint8_t)((iq >> 8) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_iq(FDCAN_HandleTypeDef *hfdcan, int16_t id, double iqControl) {
    int32_t iq = (int32_t)iqControl;
    if (iq > 1000) iq = 1000;
    else if (iq < -1000) iq = -1000;

    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_TORQUE;
    data[4] = (uint8_t)(iq & 0xff);
    data[5] = (uint8_t)((iq >> 8) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Mul_Motor_Close_Loop_Control_T(FDCAN_HandleTypeDef *hfdcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {
    uint8_t data[8];
    data[0] = (uint8_t)(iq1 & 0xff);       data[1] = (uint8_t)((iq1 >> 8) & 0xff);
    data[2] = (uint8_t)(iq2 & 0xff);       data[3] = (uint8_t)((iq2 >> 8) & 0xff);
    data[4] = (uint8_t)(iq3 & 0xff);       data[5] = (uint8_t)((iq3 >> 8) & 0xff);
    data[6] = (uint8_t)(iq4 & 0xff);       data[7] = (uint8_t)((iq4 >> 8) & 0xff);
    LK_Send_Packet(hfdcan, 0x280, data, 8);
}

void LK_M_Close_Loop_Control_Speed(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t speedControl) {
    int32_t ctrl = speedControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_SPEED;
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    data[6] = (uint8_t)((ctrl >> 16) & 0xff);
    data[7] = (uint8_t)((ctrl >> 24) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_Angle_1(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t angleControl) {
    int32_t ctrl = angleControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_POS_1;
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    data[6] = (uint8_t)((ctrl >> 16) & 0xff);
    data[7] = (uint8_t)((ctrl >> 24) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_Angle_2(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t maxSpeed, int32_t angleControl) {
    int32_t ctrl = angleControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_POS_2;
    data[2] = (uint8_t)(maxSpeed & 0xff);
    data[3] = (uint8_t)((maxSpeed >> 8) & 0xff);
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    data[6] = (uint8_t)((ctrl >> 16) & 0xff);
    data[7] = (uint8_t)((ctrl >> 24) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_Angle_3(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t angleControl) {
    uint16_t ctrl = angleControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_POS_3;
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_Angle_4(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t maxSpeed, uint16_t angleControl) {
    uint16_t ctrl = angleControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_POS_4;
    data[2] = (uint8_t)(maxSpeed & 0xff);
    data[3] = (uint8_t)((maxSpeed >> 8) & 0xff);
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_Angle_5(FDCAN_HandleTypeDef *hfdcan, int16_t id, int32_t angleControl) {
    int32_t ctrl = angleControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_POS_5;
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    data[6] = (uint8_t)((ctrl >> 16) & 0xff);
    data[7] = (uint8_t)((ctrl >> 24) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}

void LK_M_Close_Loop_Control_Angle_6(FDCAN_HandleTypeDef *hfdcan, int16_t id, uint16_t maxSpeed, int32_t angleControl) {
    int32_t ctrl = angleControl * 100;
    uint8_t data[8] = {0};
    data[0] = CMD_CONTROL_POS_6;
    data[2] = (uint8_t)(maxSpeed & 0xff);
    data[3] = (uint8_t)((maxSpeed >> 8) & 0xff);
    data[4] = (uint8_t)(ctrl & 0xff);
    data[5] = (uint8_t)((ctrl >> 8) & 0xff);
    data[6] = (uint8_t)((ctrl >> 16) & 0xff);
    data[7] = (uint8_t)((ctrl >> 24) & 0xff);
    LK_Send_Packet(hfdcan, LK_ID_BASE + id, data, 8);
}


/**
 * @brief  LK电机数据解析 (需在 FDCAN 接收回调中调用)
 * @param  rx_header: 接收头指针
 * @param  rx_data:   接收数据指针
 * @param  id:        电机ID (1-N)
 * @param  LK_M:      电机数据结构体指针
 */
void LK_M_Data_Process(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data, int16_t id, LK_M_t* LK_M)
{
    // ID 匹配检查 (0x140 + id)
    if ((rx_header->Identifier - LK_ID_BASE) != id) return;

    switch (rx_data[0]) {
        case CMD_READ_PID:
            LK_M->Angle_Kp = rx_data[2];
            LK_M->Angle_Ki = rx_data[3];
            LK_M->Speed_Kp = rx_data[4];
            LK_M->Speed_Ki = rx_data[5];
            LK_M->Iq_Kp    = rx_data[6];
            LK_M->Iq_Ki    = rx_data[7];
            break;

        case CMD_READ_ACC:
            LK_M->Accel = (int32_t)(rx_data[4] | (rx_data[5] << 8) | (rx_data[6] << 16) | (rx_data[7] << 24));
            break;

        case CMD_READ_ENC:
            LK_M->Encoder        = (uint16_t)(rx_data[2] | (rx_data[3] << 8));
            LK_M->Encoder_raw    = (uint16_t)(rx_data[4] | (rx_data[5] << 8));
            LK_M->Encoder_Offset = (uint16_t)(rx_data[6] | (rx_data[7] << 8));
            LK_M->Circle_Angle   = (double)LK_M->Encoder / 65535.0 * 360.0;
            break;

        case CMD_READ_MULTI_ANGLE:
            LK_M->Motor_Angle = (int64_t)rx_data[1] |
                                ((int64_t)rx_data[2] << 8)  |
                                ((int64_t)rx_data[3] << 16) |
                                ((int64_t)rx_data[4] << 24) |
                                ((int64_t)rx_data[5] << 32) |
                                ((int64_t)rx_data[6] << 40) |
                                ((int64_t)rx_data[7] << 48);
            LK_M->Motor_Angle *= 0.01;
            break;

        case CMD_READ_SINGLE_ANGLE:
            LK_M->Circle_Angle = (int32_t)(rx_data[4] | (rx_data[5] << 8) | (rx_data[6] << 16) | (rx_data[7] << 24));
            LK_M->Circle_Angle *= 0.01;
            break;

        case CMD_READ_STATE_1:
            LK_M->Temp = (int8_t)rx_data[1];
            LK_M->Voltage = (int16_t)(rx_data[3] | (rx_data[4] << 8));
            LK_M->Voltage *= 0.1f;
            LK_M->Error_state = rx_data[7];
            
            LK_M->Voltage_State = (rx_data[7] & 0x01) ? Vol_Low : Vol_Normal;
            LK_M->Temp_State = ((rx_data[7] >> 3) & 0x01) ? Over_Temp : Temp_Normal;
            break;

        case CMD_READ_STATE_2:
        case 0xA0: 
        case CMD_CONTROL_TORQUE:
        case CMD_CONTROL_SPEED:
        case CMD_CONTROL_POS_1:
        case CMD_CONTROL_POS_2:
        case CMD_CONTROL_POS_3:
        case CMD_CONTROL_POS_4:
        case CMD_CONTROL_POS_5:
        case CMD_CONTROL_POS_6:
            LK_M->Temp = (int8_t)rx_data[1];
            
            if (rx_data[0] == 0xA0) {
                LK_M->Power = (int16_t)(rx_data[2] | (rx_data[3] << 8));
            } else {
                LK_M->T = (int16_t)(rx_data[2] | (rx_data[3] << 8));
                LK_M->T *= 0.0025;
            }

            LK_M->LK_Speed = (int16_t)(rx_data[4] | (rx_data[5] << 8));
            LK_M->LK_Speed /= 8.0; // dps

            LK_M->Encoder = (uint16_t)(rx_data[6] | (rx_data[7] << 8));
            LK_M->Circle_Angle = (double)LK_M->Encoder / 65535.0 * 360.0;
            break;
            
        case CMD_READ_STATE_3:
             
             LK_M->Temp = (int8_t)rx_data[1];
             break;

        default:
            break;
    }
}
