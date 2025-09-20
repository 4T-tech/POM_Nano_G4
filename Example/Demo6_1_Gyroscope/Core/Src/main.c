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
#include "lsm6dso_reg.h"
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

#define ACCEL_THRESHOLD_MIN   -2500.0f  // 加速度范围：后续可根据情况自定义
#define ACCEL_THRESHOLD_MAX   	2500.0f  //	+-2500
#define GYRO_THRESHOLD_MIN    -2200.0f  // 陀螺仪范围：后续可根据情况自定义
#define GYRO_THRESHOLD_MAX     2200.0f	 // +-2200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static stmdev_ctx_t dev_ctx;

uint8_t data[21]={0xab,0xfd,0xfe,0x01,0x0d,0x00};
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
int16_t	acc_int16[3]	={0,0,0};
int16_t	gyr_int16[3]		={0,0,0};	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t imu_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t imu_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
void IMU_Init();
uint8_t get_xl();
uint8_t get_gy();

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
  IMU_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    get_xl();
    get_gy();
    
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    acceleration_mg[0] =
    lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] =
    lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
    acceleration_mg[2] =
    lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);
    
    acc_int16[0]=(int16_t)(acceleration_mg[0]);
    acc_int16[1]=(int16_t)(acceleration_mg[1]);
    acc_int16[2]=(int16_t)(acceleration_mg[2]);
    
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
    angular_rate_mdps[0] =
    lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[0]);
    angular_rate_mdps[1] =
    lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[1]);
    angular_rate_mdps[2] =
    lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[2]);
    
    gyr_int16[0]=(int16_t)(angular_rate_mdps[0]/1000);
    gyr_int16[1]=(int16_t)(angular_rate_mdps[1]/1000);
    gyr_int16[2]=(int16_t)(angular_rate_mdps[2]/1000);
    
    data[7]=acc_int16[0]>>8;//ACC_X
    data[6]=acc_int16[0];
    data[9]=acc_int16[1]>>8;//ACC_Y
    data[8]=acc_int16[1];
    data[11]=acc_int16[2]>>8;//ACC_Z
    data[10]=acc_int16[2];

    data[13]=gyr_int16[0]>>8;//GYR_X 
    data[12]=gyr_int16[0];		
    data[15]=gyr_int16[1]>>8;//GYR_Y 
    data[14]=gyr_int16[1];			
    data[17]=gyr_int16[2]>>8;//GYR_Z 
    data[16]=gyr_int16[2];	
    
    data[18]=0;	
    
    sumcheck = 0;
    addcheck = 0;
    for(uint16_t i=0; i < 19; i++)
    {
    sumcheck += data[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
    addcheck += sumcheck; //每一字节的求和操作，进行一次 sumcheck 的累加
    }
    data[19]=sumcheck;
    data[20]=addcheck;
    
    CDC_Transmit_FS(data,21);
    
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
static int32_t imu_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
    return 0;
}

static int32_t imu_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}
void IMU_Init()
{
  dev_ctx.write_reg = imu_platform_write;
  dev_ctx.read_reg = imu_platform_read;
  dev_ctx.mdelay = HAL_Delay;
  dev_ctx.handle = &SENSOR_BUS;
  while (1)
  {
    lsm6dso_device_id_get(&dev_ctx, &whoamI);
    LOG_INFO("LSM6DSO_ID=0x%x,whoamI=0x%x\r\n",LSM6DSO_ID,whoamI);
    if (whoamI == LSM6DSO_ID)
    break;
  };
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&dev_ctx, &rst);
  } while (rst);
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);
  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);
  lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
  lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
}

uint8_t get_xl()
{
  uint8_t reg;
  /* Read output only if new xl value is available */
  lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);
  if (reg) 
  {
    /* Read acceleration field data */
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  }
  return reg;
}

uint8_t get_gy()
{
  uint8_t reg;
  lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
  if (reg) 
  {
    /* Read angular rate field data */
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
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
