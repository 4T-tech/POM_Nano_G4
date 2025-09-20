/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "crc.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "POM_ESP32_C3.h"
#include "POM_lsm6dso.h"
#include "POM_lis2mdl.h"
#include "POM_MotionFX.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t printf_Buf[100];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t FIFO_NUM;
uint8_t FIFO_BUF[30][7];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_Device_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  Disable_ESP32_C3();
  HAL_Delay(1000);
  CDC_Transmit_FS("Disable_ESP32_C3:OK",19);
  sprintf(printf_Buf,"Disable_ESP32_C3:OK");
  HAL_UART_Transmit(&huart1,printf_Buf,19,200);
  if(LSM6DSO_Initialization() != POM_LSM6DSO_OK)
  {
        CDC_Transmit_FS("LSM6DSO Init Failed",19);
        sprintf(printf_Buf,"LSM6DSO Init Failed");
        HAL_UART_Transmit(&huart1,printf_Buf,19,200);
  }
  if(LSM6DSO_Set_FIFO_Mode() != POM_LSM6DSO_OK)
  {
        CDC_Transmit_FS("LSM6DSO Set Failed",18);
        sprintf(printf_Buf,"LSM6DSO Set Failed");
        HAL_UART_Transmit(&huart1,printf_Buf,18,200);
  }
  MotionFX_6x_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(FIFO_NUM)
    {
        MotionFX_6x_Pro(FIFO_BUF,FIFO_NUM);
        printf_Buf[0] = 0xAB;
        printf_Buf[1] = 0xFD;
        printf_Buf[2] = 0xFE;
        printf_Buf[3] = 0x03;
        printf_Buf[4] = 0x07;
        printf_Buf[5] = 0x00;
        printf_Buf[6] = (int16_t)(sensor_hub_data.mfx_6x.rotation[2]*100)&0xFF;
        printf_Buf[7] = (int16_t)(sensor_hub_data.mfx_6x.rotation[2]*100)>>8;
        printf_Buf[8] = (int16_t)(sensor_hub_data.mfx_6x.rotation[1]*100)&0xFF;
        printf_Buf[9] = (int16_t)(sensor_hub_data.mfx_6x.rotation[1]*100)>>8;
        printf_Buf[10] = (int16_t)((sensor_hub_data.mfx_6x.rotation[0]-180)*100)&0xFF;
        printf_Buf[11] = (int16_t)((sensor_hub_data.mfx_6x.rotation[0]-180)*100)>>8;
        printf_Buf[12] = 0x01;
        printf_Buf[13] = 0;
        printf_Buf[14] = 0;
        for(uint8_t temp_num = 0;temp_num < 13; temp_num++)
        {
            printf_Buf[13] += printf_Buf[temp_num];
            printf_Buf[14] += printf_Buf[13];
        }
        HAL_UART_Transmit(&huart1,printf_Buf,15,200);
        CDC_Transmit_FS(printf_Buf,15);
        FIFO_NUM = 0;
    }
//    if(LSM6DSO_Poll_ACC_Value_Get(&LSM6DSO_ACC_Num) == POM_LSM6DSO_OK && LSM6DSO_Poll_GYRO_Value_Get(&LSM6DSO_GYRO_Num) == POM_LSM6DSO_OK)
//    {
//        sprintf(printf_Buf,"%d",LSM6DSO_ACC_Num.x);
//        HAL_UART_Transmit(&huart1,printf_Buf,5,200);
//    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        FIFO_NUM = LSM6DSO_FIFO_Buf_Get(FIFO_BUF);
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
