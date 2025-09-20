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
#include "i2c.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "G431_Series.h"
#include "lis2mdl_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_DEBUG
#define USE_INFO
#define USE_PRINT

#if defined(USE_DEBUG) || defined(USE_INFO) || defined(USE_PRINT)
uint8_t printfBuf[100];
#endif
#ifdef USE_DEBUG
#define LOG_DEBUG(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[DEBUG] %s:%d %s(): " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__))
#else
#define LOG_DEBUG(format, ...)
#endif
#ifdef USE_INFO
#define LOG_INFO(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[INFO] %d %s(): " format "\n", __LINE__, __func__, ##__VA_ARGS__))
#else
#define LOG_INFO(format, ...)
#endif
#ifdef USE_PRINT
#define LOG_PRINT(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[PRINT] %s(): " format "\n", __func__, ##__VA_ARGS__))
#else
#define LOG_PRINT(format, ...)
#endif

 #define SENSOR_BUS hi2c1
 #define MAG_THRESHOLD_MIN    -500.0f  // 磁力计范围：后续可根据情况自定义
 #define MAG_THRESHOLD_MAX     500.0f	//+-500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t data_raw_magnetic[3];
float magnetic_mG[3];
uint8_t mag_whoamI, mag_rst;
stmdev_ctx_t dev_ctx;


uint8_t data[15]={0xab,0xfd,0xfe,0x01,0x07,0x00};
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
int16_t	mag_int16[3]	={0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t mag_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t mag_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void MAG_Init(void);
uint8_t get_mag(void);

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
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  Disable_ESP32_C3();
  HAL_Delay(1000);
  LOG_INFO("Disable_ESP32_C3:OK");
  MAG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    get_mag();
    
    /* Read magnetic field data */
    memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
    lis2mdl_magnetic_raw_get(&dev_ctx, data_raw_magnetic);
    magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[0]);
    magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[1]);
    magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[2]);
    
    mag_int16[0]=(int16_t)(magnetic_mG[0]);
    mag_int16[1]=(int16_t)(magnetic_mG[1]);
    mag_int16[2]=(int16_t)(magnetic_mG[2]);
    
    data[7]=mag_int16[0]>>8;//MAG_X
    data[6]=mag_int16[0];
    data[9]=mag_int16[1]>>8;//MAG_Y
    data[8]=mag_int16[1];
    data[11]=mag_int16[2]>>8;//MAG_Z
    data[10]=mag_int16[2];
    
    data[12]=0;
    
    sumcheck = 0;
    addcheck = 0;
    for(uint16_t i=0; i < 13; i++)
    {
    sumcheck += data[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
    addcheck += sumcheck; //每一字节的求和操作，进行一次 sumcheck 的累加
    }
    data[13]=sumcheck;
    data[14]=addcheck;
    
    CDC_Transmit_FS(data,15);
    
    HAL_Delay(10);
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
int32_t mag_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_I2C_Mem_Write(handle, LIS2MDL_I2C_ADD, reg,I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
    return 0;
}

int32_t mag_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_I2C_Mem_Read(handle, LIS2MDL_I2C_ADD, reg,I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}
void MAG_Init(void) 
{
  /* Initialize mag driver interface */
  dev_ctx.write_reg = mag_platform_write;
  dev_ctx.read_reg = mag_platform_read;
  dev_ctx.mdelay = HAL_Delay;
  dev_ctx.handle = &SENSOR_BUS;
  
  while (1) {
    lis2mdl_device_id_get(&dev_ctx, &mag_whoamI);
    LOG_INFO("LIS2MDL_ID=0x%x, whoamI=0x%x\r\n", LIS2MDL_ID, mag_whoamI);
    if (mag_whoamI == LIS2MDL_ID)
        break;
    else {
        LOG_INFO("MAG ID mismatch! Retrying...\r\n");
        HAL_Delay(100);
    }
  };
  lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);
  
  do
  {
      lis2mdl_reset_get(&dev_ctx, &mag_rst);
      HAL_Delay(10);
  } while (mag_rst);
  lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lis2mdl_drdy_on_pin_set(&dev_ctx, PROPERTY_ENABLE);
  lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);
  
  lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);
  
  lis2mdl_set_rst_mode_set(&dev_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
  
  lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
  
  lis2mdl_power_mode_set(&dev_ctx, LIS2MDL_HIGH_RESOLUTION);
  
  LOG_INFO("MAG Init Complete\r\n");
}

uint8_t get_mag(void)
{
  uint8_t reg;
  /* Read output only if new mag value is available */
  lis2mdl_mag_data_ready_get(&dev_ctx, &reg);
  if (reg) {
      /* Read magnetic field data */
      memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
      lis2mdl_magnetic_raw_get(&dev_ctx, data_raw_magnetic);
  }
  return reg;
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

#ifdef  USE_FULL_ASSERT
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
