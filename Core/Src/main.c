/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2026 STMicroelectronics.
	  * All rights reserved.
	  *
	  * This software is licensed under terms that can be found in the LICENSE file
	  * in the root directory of this software component.
	  * If no LICENSE file comes with this software, it is provided AS-IS.
	  *
	  ******************************************************************************
	  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "LK_Motor.h"
	#include "pid.h"
	#include "VOFA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	LK_M_t LK_M_Gimbal_Yaw;
	LK_M_t LK_M_Gimbal_Pitch;
	extern FDCAN_HandleTypeDef hfdcan1;
	extern TIM_HandleTypeDef htim6;
	extern TIM_HandleTypeDef htim7;
	extern UART_HandleTypeDef huart5;
	#define IMU_RX_BUF_SIZE 256            // DMA 接收缓冲区大小
	__attribute__((section(".data.ARM.__at_0x30000000"))) uint8_t imu_rx_buf[IMU_RX_BUF_SIZE] ;
	volatile uint32_t last_motor_rx_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Process_USB_Command(void)
{
    static char line_buf[64];
    static uint8_t line_idx = 0;

    // 当 FIFO 中有数据时
    while (usb_rx_fifo.tail != usb_rx_fifo.head) 
    {
        // 读出一个字符
        char c = usb_rx_fifo.buffer[usb_rx_fifo.tail];
        usb_rx_fifo.tail = (usb_rx_fifo.tail + 1) % 512; // 512 是 USB_RX_BUF_SIZE

        // 判断是否是一帧的结束 (回车或换行)
        if (c == '\n' || c == '\r') 
        {
            line_buf[line_idx] = '\0'; // 字符串封尾
            if (line_idx > 0) 
            {
                float p_val = 0.0f, y_val = 0.0f;
                // 解析格式: "P:12.5,Y:45.0"
                if (sscanf(line_buf, "P:%f,Y:%f", &p_val, &y_val) == 2) 
                {
                    // 赋值给目标结构体
                    Gimbal_Debug.Pitch_Target_Angle = p_val;
                    Gimbal_Debug.Yaw_Target_Angle = y_val;
                    
                    Gimbal_Debug.Control_Mode = DEBUG_ANGLE_LOOP;
                }
            }
            line_idx = 0; // 重置索引，准备接收下一帧
        } 
        else 
        {
            // 存入行缓冲区，防止溢出
            if (line_idx < sizeof(line_buf) - 1) {
                line_buf[line_idx++] = c;
            }
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
HAL_PWREx_EnableUSBVoltageDetector();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	 
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	} 
	  Gimbal_Init();
	  
	  LK_M_Power_On(&hfdcan1, 2); // 开启 Pitch ( ID=2)
	  LK_M_Power_On(&hfdcan1, 1); // 开启 Yaw   ( ID=1)
	  HAL_Delay(100);
	  LK_M_Close_Loop_Control_iq(&hfdcan1, 1, 0);
	  LK_M_Close_Loop_Control_iq(&hfdcan1, 2, 0);
	  HAL_Delay(10);
	  HAL_TIM_Base_Start_IT(&htim6);
	HiIMU_Init(&huart5,imu_rx_buf,IMU_RX_BUF_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	uint32_t last_probe_time = 0;
//	static uint8_t motor_was_offline = 1; // 记录电机的上下线状态
		while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Process_USB_Command();
//        uint32_t current_time = HAL_GetTick();

        }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

if (htim->Instance == TIM6)

{
	
	static uint32_t flag_tim = 0;
//	if (HAL_GetTick() - last_motor_rx_time <= 500)
//        {


if(flag_tim%1 == 0)
{
float pitch_iq = Gimbal_Control_Loop(&Pitch_Motor, 2,(float)LK_M_Gimbal_Pitch.Encoder, LK_M_Gimbal_Pitch.LK_Speed);




float yaw_iq = Gimbal_Control_Loop(&Yaw_Motor, 1, (float)LK_M_Gimbal_Yaw.Encoder, LK_M_Gimbal_Yaw.LK_Speed);




if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
                LK_M_Close_Loop_Control_iq(&hfdcan1, 2, pitch_iq);
            }
if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
                LK_M_Close_Loop_Control_iq(&hfdcan1, 1, yaw_iq);
		}
}
//	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
//                LK_M_Close_Loop_Control_iq(&hfdcan1, 2, 0);
//            }
//if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
//                LK_M_Close_Loop_Control_iq(&hfdcan1, 1, 0);
//	}
if(flag_tim%1000 == 0)
{
	LK_M_Power_On(&hfdcan1, 2); // 开启 Pitch ( ID=1)
	LK_M_Power_On(&hfdcan1, 1); // 开启 Yaw   ( ID=2)
}
flag_tim++;
}
}



/**

  * @brief  FDCAN FIFO0 接收回调函数

  * @note   当 FDCAN 的 FIFO0 收到新消息时，HAL 库调用这个函数

  * @param  



  */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)

{


if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)

{

FDCAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];



if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)

{


if (hfdcan->Instance == FDCAN1) 

{
	last_motor_rx_time = HAL_GetTick();



LK_M_Data_Process(&RxHeader, RxData, 2, &LK_M_Gimbal_Pitch);

LK_M_Data_Process(&RxHeader, RxData, 1, &LK_M_Gimbal_Yaw);

}

}

}

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        
        HiIMU_RxIdleCallback(huart, Size);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        
        HAL_UART_AbortReceive(huart);
        
        //重新启动下一次空闲中断 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, imu_rx_buf, IMU_RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
