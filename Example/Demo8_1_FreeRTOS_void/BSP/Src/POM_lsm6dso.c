#include "POM_lsm6dso.h"


#define USE_LSM6DSO_DEBUG       
#ifdef USE_LSM6DSO_DEBUG        
uint8_t printf_lsm6dso_Buf[100];
#define LOG_LSM6DSO_DEBUG(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_lsm6dso_Buf, sprintf((char*)printf_lsm6dso_Buf, "[DEBUG] %s:%d %s(): " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__), 200)
#define LOG_LSM6DSO_INFO(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_lsm6dso_Buf, sprintf((char*)printf_lsm6dso_Buf, "[INFO] %d %s(): " format "\n", __LINE__, __func__, ##__VA_ARGS__), 200)
#define LOG_LSM6DSO_PRINTF(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_lsm6dso_Buf, sprintf((char*)printf_lsm6dso_Buf, format "\n", ##__VA_ARGS__), 200)

#else
#define LOG_LSM6DSO_DEBUG(format, ...)
#define LOG_LSM6DSO_INFO(format, ...)
#define LOG_LSM6DSO_PRINTF(format, ...)
#endif




/*************
�÷���:

LSM6DSO_Axes_t LSM6DSO_ACC_Num;
LSM6DSO_Axes_t LSM6DSO_GYRO_Num;
mian()
{
    LSM6DSO_Initialization();
    LSM6DSO_Set_Poll_Mode();
    while(1)
    {
        if(LSM6DSO_Poll_ACC_Value_Get(&LSM6DSO_ACC_Num) == POM_LSM6DSO_OK && LSM6DSO_Poll_GYRO_Value_Get(&LSM6DSO_GYRO_Num) == POM_LSM6DSO_OK)
        {
        
        }
    }
}

�÷���:
uint8_t FIFO_NUM;
uint8_t FIFO_BUF[30][7];
mian()
{
    LSM6DSO_Initialization();
    LSM6DSO_Set_FIFO_Mode();
    MotionFX_6x_init();
    while(1)
    {
        if(FIFO_NUM)
        {
            MotionFX_6x_Pro(FIFO_BUF,FIFO_NUM);
            FIFO_NUM = 0;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        FIFO_NUM = LSM6DSO_FIFO_Buf_Get(FIFO_BUF);
    }
}
*************/




LSM6DSO_Object_t lsm6dso_obj;
LSM6DSO_Capabilities_t lsm6dso_cap;

/**
  *@brief Initialize the LSM6DSO sensor
  *@retval POM_LSM6DSO_OK in case of success, POM_LSM6DSO_ERROR otherwise
  */
int8_t LSM6DSO_Initialization(void)
{
    /* ��������ڴ� */
    memset(&lsm6dso_obj, 0, sizeof(LSM6DSO_Object_t));
    
    /* ע��IO�ӿ� */
    LSM6DSO_IO_t io_ctx = {
        .Init = BSP_I2C1_Init,
        .DeInit = BSP_I2C1_DeInit,
        .BusType = LSM6DSO_I2C_BUS,
        .Address = LSM6DSO_I2C_ADD_H,
        .WriteReg = BSP_I2C1_WriteReg,
        .ReadReg = BSP_I2C1_ReadReg,
        .GetTick = BSP_GetTick,
        .Delay = HAL_Delay
    };
    
    if(LSM6DSO_RegisterBusIO(&lsm6dso_obj, &io_ctx) != LSM6DSO_OK)
    {
        LOG_LSM6DSO_INFO("Failed to register bus IO");
        return POM_LSM6DSO_ERROR;
    }
    
    /* ��ʼ�������� */
    if(LSM6DSO_Init(&lsm6dso_obj) != LSM6DSO_OK)
    {
        LOG_LSM6DSO_INFO("Failed to initialize LSM6DSO");
        return POM_LSM6DSO_ERROR;
    }
    /* ��ȡWHO_AM_I ȷ����������������*/
    uint8_t WHO_AM_I_Reg;
    if(LSM6DSO_ReadID(&lsm6dso_obj,&WHO_AM_I_Reg) != LSM6DSO_OK)
    {
        LOG_LSM6DSO_INFO("Failed to LSM6DSO Read WHO_AM_I_Reg");
        return POM_LSM6DSO_ERROR;
    }
    if(WHO_AM_I_Reg != LSM6DSO_ID)
    {
        LOG_LSM6DSO_INFO("Failed to LSM6DSO WHO_AM_I_Reg = 0x%x",WHO_AM_I_Reg);
    }
    //LOG_LSM6DSO_INFO("LSM6DSO initialized successfully");
    return POM_LSM6DSO_OK;
}

/**
  * @brief  Configure LSM6DSO sensor in polling mode
  * @details This function configures the accelerometer and gyroscope with:
  *          - Accelerometer full scale ��2g
  *          - Gyroscope full scale ��2000dps  
  *          - ODR 12.5Hz for both sensors
  *          - Basic analog filter configuration
  *          - Enables both sensors
  * @retval POM_LSM6DSO_OK if configuration successful, POM_LSM6DSO_ERROR otherwise
  */
int8_t LSM6DSO_Set_Poll_Mode(void)
{
    //���ü��ٶȼ�����Ϊ��2g
    if (LSM6DSO_ACC_SetFullScale(&lsm6dso_obj,2) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //��������������Ϊ��2000dps
    if (LSM6DSO_GYRO_SetFullScale(&lsm6dso_obj,2000) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���ü��ٶȼƲ���Ƶ��Ϊ12.5Hz
    if (LSM6DSO_ACC_SetOutputDataRate(&lsm6dso_obj,12.5) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���������ǲ���Ƶ��Ϊ12.5Hz
    if (LSM6DSO_GYRO_SetOutputDataRate(&lsm6dso_obj,12.5) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���ü��ٶȼ��˲���Ϊ ��ͨ�˲��� LSM6DSO_LP_ODR_DIV_100
    if (LSM6DSO_ACC_Set_Filter_Mode(&lsm6dso_obj,0,0x04) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //�����������˲���Ϊ ��ͨ�˲��� �е��˲�
    if(LSM6DSO_GYRO_Set_Filter_Mode(&lsm6dso_obj,0,0x03) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //ʹ�ܼ��ٶȼ�
    if (LSM6DSO_ACC_Enable(&lsm6dso_obj) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //ʹ��������
    if (LSM6DSO_GYRO_Enable(&lsm6dso_obj) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    return POM_LSM6DSO_OK;
}

/**
  * @brief  Configure LSM6DSO sensor in FIFO mode
  * @details This function configures the accelerometer and gyroscope with:
  *          - Accelerometer full scale ��2g
  *          - Gyroscope full scale ��2000dps  
  *          - FIFO Watermark_Level scale 30
  *          - FIFO 417Hz for both sensors
  *          - ODR 417Hz for both sensors
  *          - DRDY Set Mode to LSM6DSO_DRDY_PULSED
  *          - Trigger INT1 jump when FIFO reaches threshold
  *          - Enables timestamp
  *          - Enables both sensors
  * @retval POM_LSM6DSO_OK if configuration successful, POM_LSM6DSO_ERROR otherwise
  */
int8_t LSM6DSO_Set_FIFO_Mode(void)
{
    lsm6dso_pin_int1_route_t int1_route;
    
    //���ü��ٶȼ�����Ϊ��2g
    if (LSM6DSO_ACC_SetFullScale(&lsm6dso_obj,2) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //��������������Ϊ��2000dps
    if (LSM6DSO_GYRO_SetFullScale(&lsm6dso_obj,2000) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //����FIFO��СΪ30 ���ٶȼ� ������ ʱ��� �������ݸ�ʮ��ʱ����FIFO���ж�
    if(LSM6DSO_FIFO_Set_Watermark_Level(&lsm6dso_obj,30)!= LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���ü��ٶȼ�FIFO�ٶ�417
    if(LSM6DSO_FIFO_ACC_Set_BDR(&lsm6dso_obj,417) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //����������FIFO�ٶ�417
    if(LSM6DSO_FIFO_GYRO_Set_BDR(&lsm6dso_obj,417) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���ü��ٶȼƲ���Ƶ��Ϊ417Hz
    if (LSM6DSO_ACC_SetOutputDataRate(&lsm6dso_obj,417) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���������ǲ���Ƶ��Ϊ417Hz
    if (LSM6DSO_GYRO_SetOutputDataRate(&lsm6dso_obj,417) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //����FIFOΪ LSM6DSO_STREAM_MODE
    if(LSM6DSO_FIFO_Set_Mode(&lsm6dso_obj,6) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //����DRDYģʽΪ����(FIFO��ʱ����75us�͵�ƽ)
    if(LSM6DSO_DRDY_Set_Mode(&lsm6dso_obj,1) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //����FIFO������ֵʱ����INT1����
    lsm6dso_pin_int1_route_get(&(lsm6dso_obj.Ctx), &int1_route);
    int1_route.fifo_th = PROPERTY_ENABLE;
    lsm6dso_pin_int1_route_set(&(lsm6dso_obj.Ctx), int1_route);
    //���ÿ���ʱ�������
    lsm6dso_fifo_timestamp_decimation_set(&(lsm6dso_obj.Ctx),LSM6DSO_DEC_1);
    /* Enable timestamp */
    lsm6dso_timestamp_set(&(lsm6dso_obj.Ctx),1);
    //���ü��ٶȼƵ�ԴģʽΪ LSM6DSO_HIGH_PERFORMANCE_MD
    if(LSM6DSO_ACC_Set_Power_Mode(&lsm6dso_obj,0) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //���������ǵ�ԴģʽΪ LSM6DSO_GY_HIGH_PERFORMANCE
    if(LSM6DSO_GYRO_Set_Power_Mode(&lsm6dso_obj,0) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //ʹ�ܼ��ٶȼ�
    if (LSM6DSO_ACC_Enable(&lsm6dso_obj) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    //ʹ��������
    if (LSM6DSO_GYRO_Enable(&lsm6dso_obj) != LSM6DSO_OK)
    {
        return POM_LSM6DSO_ERROR;
    }
    return POM_LSM6DSO_OK;
}


/**
  * @brief  Get accelerometer data from LSM6DSO sensor (polling mode)
  * @param  ACC_Buf Pointer to structure for storing accelerometer axes data (x/y/z)
  * @retval POM_LSM6DSO_OK New data successfully read
  * @retval POM_LSM6DSO_ERROR No new data available
  * @note   Uses polling approach - checks DRDY status before reading
  *         Ensures only fresh data is retrieved
  */
int8_t LSM6DSO_Poll_ACC_Value_Get(LSM6DSO_Axes_t *ACC_Buf)
{
    uint8_t reg;
    //�������״̬
    LSM6DSO_ACC_Get_DRDY_Status(&lsm6dso_obj,&reg);
    if (reg) 
    {
        //�õ�����
        LSM6DSO_ACC_GetAxes(&lsm6dso_obj,ACC_Buf);
        return POM_LSM6DSO_OK;
    }
    else
    {
        return POM_LSM6DSO_ERROR;
    }
}

/**
  * @brief  Get gyroscope data from LSM6DSO sensor (polling mode)
  * @param  GYRO_Buf Pointer to structure for storing gyroscope axes data (x/y/z in dps)
  * @retval POM_LSM6DSO_OK New data successfully read
  * @retval POM_LSM6DSO_ERROR No new data available
  * @note   Uses polling approach - checks DRDY status before reading
  *         Non-blocking function - returns immediately if no new data
  */
int8_t LSM6DSO_Poll_GYRO_Value_Get(LSM6DSO_Axes_t *GYRO_Buf)
{
    uint8_t reg;
    //�������״̬
    LSM6DSO_GYRO_Get_DRDY_Status(&lsm6dso_obj,&reg);
    if (reg) 
    {
        //�õ�����
        LSM6DSO_GYRO_GetAxes(&lsm6dso_obj,GYRO_Buf);
        return POM_LSM6DSO_OK;
    }
    else
    {
        return POM_LSM6DSO_ERROR;
    }
}



/**
  * @brief  Retrieve FIFO data from LSM6DSO sensor when Watermark (WTM) threshold is reached
  * @param  FIFO_data Pointer to a 2D array for storing FIFO data (each row contains [tag, data0..data5])
  * @retval uint8_t Number of samples read if WTM flag is set
  * @retval POM_LSM6DSO_ERROR If WTM flag is not set (no data available)
  * @note   - Checks FIFO Watermark interrupt flag before reading
  *         - Reads all available samples up to the WTM threshold
  *         - Each sample consists of: 
  *             - 1-byte TAG (FIFO_data[i][0]) 
  *             - 6-byte data payload (FIFO_data[i][1..6])
  *         - Typical usage: Call in interrupt handler when WTM event occurs
  *         - FIFO_data array must be pre-allocated with sufficient rows (��fifo_num)
  * @use    Place it in the interrupt for optimal use
  */
int8_t LSM6DSO_FIFO_Buf_Get(uint8_t FIFO_data[][7])
{
    uint8_t wtmflag;
    uint16_t fifo_num;
    uint16_t temp_num = 0;
    uint8_t fifo_tag;
    uint8_t temp_buf[6] = {0,0,0,0,0,0};
    //����Ƿ�ﵽ���õ���ֵ  1:�ﵽ      0:δ�ﵽ
    lsm6dso_fifo_wtm_flag_get(&(lsm6dso_obj.Ctx),&wtmflag);
    
    if(wtmflag != 0)
    {
        //���FIFO���ж���������Ҫ������
        LSM6DSO_FIFO_Get_Num_Samples(&lsm6dso_obj,&fifo_num);
        for(temp_num = 0;temp_num<fifo_num;temp_num++)
        {
            //��ȡFIFO��ǩ
            LSM6DSO_FIFO_Get_Tag(&lsm6dso_obj,&fifo_tag);
            FIFO_data[temp_num][0] = fifo_tag;
            //��ȡ����
            LSM6DSO_FIFO_Get_Data(&lsm6dso_obj,&temp_buf[0]);
            FIFO_data[temp_num][1] = temp_buf[0];
            FIFO_data[temp_num][2] = temp_buf[1];
            FIFO_data[temp_num][3] = temp_buf[2];
            FIFO_data[temp_num][4] = temp_buf[3];
            FIFO_data[temp_num][5] = temp_buf[4];
            FIFO_data[temp_num][6] = temp_buf[5];
        }
        return fifo_num;
    }
    return POM_LSM6DSO_ERROR;
}





