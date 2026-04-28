#include "main.h"

/* ================= 私有变量 ================= */
static UART_HandleTypeDef *HiIMU_HUART;
static uint8_t *p_rx_buf_ptr = NULL;
static uint16_t rx_buf_len = 0;

/* 全局用户数据 */
CH040_Raw_t hi_imu_data = {0};

/* ================= 内部函数声明 ================= */
static void HiIMU_Process_Directly(uint8_t *RX, uint16_t Size);
/**
  * @brief  初始化 IMU 接收
  * @param  huart: 串口句柄 (例如 &huart5)
  * @param  rx_buffer: DMA 接收缓存区首地址
  * @param  buffer_size: 缓存区大小
  */
void HiIMU_Init(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t buffer_size)
{
    HiIMU_HUART = huart;
    p_rx_buf_ptr = rx_buffer;
    rx_buf_len = buffer_size;
    
    // 【关键新增】强行清除上电真空期可能产生的溢出、帧错误等标志位
    __HAL_UART_CLEAR_FLAG(HiIMU_HUART, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    
    // 启动空闲中断 DMA 接收 (最好检查一下返回值是不是 HAL_OK)
    if (HAL_UARTEx_ReceiveToIdle_DMA(HiIMU_HUART, p_rx_buf_ptr, rx_buf_len) != HAL_OK)
    {
        // 如果卡在这里说明 DMA 配置有问题或者被占用了
        Error_Handler(); 
    }
	__HAL_DMA_DISABLE_IT(HiIMU_HUART->hdmarx, DMA_IT_HT);
}

/**
  * @brief  串口空闲中断回调 (在 HAL_UARTEx_RxEventCallback 中调用)
  * @param  huart: 串口句柄
  * @param  Size: 本次接收到的总字节数
  */
void HiIMU_RxIdleCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == HiIMU_HUART->Instance && p_rx_buf_ptr != NULL)
    {
        /* H7 Cache 维护：无效化 D-Cache，确保 CPU 读到的是 DMA 搬运的最新数据 */
        // 注意：传入的 Size 或 rx_buf_len 最好是 32 的整数倍
        SCB_InvalidateDCache_by_Addr((uint32_t*)p_rx_buf_ptr, rx_buf_len);

        /* 将 DMA 接收到的数组直接扔给按位解析逻辑 */
        HiIMU_Process_Directly(p_rx_buf_ptr, Size);

        /* 重新启动接收 (覆盖旧数据，准备下一次接收) */
        HAL_UARTEx_ReceiveToIdle_DMA(HiIMU_HUART, p_rx_buf_ptr, rx_buf_len);
		__HAL_DMA_DISABLE_IT(HiIMU_HUART->hdmarx, DMA_IT_HT);
    }
}
/**
  * @brief  直接从缓冲区寻找帧头并映射数据结构
  */
static void HiIMU_Process_Directly(uint8_t *RX, uint16_t Size)
{
    // 1. 扫描寻找帧头 0x5A 0xA5 (Size 至少要有 Header(2)+Len(2)+CRC(2)+Tag(1) = 7 个字节)
    for (uint16_t i = 0; i < Size - 7; i++) 
    {
        // 匹配帧头
        if (RX[i] == 0x5A && RX[i+1] == 0xA5) 
        {
            // 2. 检查偏移 6 个字节后的 Tag 是否为 0x91 
            // (跳过 5A A5 LEN_L LEN_H CRC_L CRC_H 共6字节)
           if (RX[i + 6] == 0x91) 
            {
                // 用新的、严丝合缝的结构体映射
                CH040_Raw_t *raw = (CH040_Raw_t *)(&RX[i + 6]);

                /* ================= 解析与赋值 ================= */
                hi_imu_data.roll  = raw->roll;
                hi_imu_data.pitch = raw->pitch;
                hi_imu_data.yaw   = raw->yaw;

                hi_imu_data.acc_x = raw->acc_x;
                hi_imu_data.acc_y = raw->acc_y;
                hi_imu_data.acc_z = raw->acc_z;

                hi_imu_data.gyr_x = raw->gyr_x;
                hi_imu_data.gyr_y = raw->gyr_y;
                hi_imu_data.gyr_z = raw->gyr_z;

                hi_imu_data.air_pressure = raw->air_pressure;
                
                
                break; 
            
            }
        }
    }
}