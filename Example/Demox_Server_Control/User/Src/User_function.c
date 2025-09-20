#include "User_function.h"

/**
  * @brief  从 USB CDC 环形缓冲区读取数据到指定内存
  *
  * @note   该函数从全局环形缓冲区 `cdc_rx_buf` 中读取所有待处理数据，
  *         读取后的数据会从缓冲区移除（移动尾指针）。
  *         适用于主循环中非中断环境下处理 USB 接收数据。
  *
  * @warning
  *         - 缓冲区操作未加中断保护，若在中断和主循环中同时访问缓冲区，
  *           需调用前禁用中断（如 `__disable_irq()`）
  *         - 需确保 `cdc_rx_buf`、`cdc_rx_head`、`cdc_rx_tail` 和
  *           `CDC_RX_BUFSIZE` 已正确定义
  *
  * @param  Buf : 目标存储缓冲区指针，读取的数据将拷贝到此内存
  * @retval uint16_t : 实际读取的数据长度（字节数），返回 0 表示无数据可读
  *
  * @usage
  * @code
  * uint8_t data[64];
  * uint16_t len = Read_USB_CDC_Buf(data);
  * if(len > 0) {
  *     // 处理数据...
  * }
  * @endcode
  */
uint16_t Read_USB_CDC_Buf(uint8_t *Buf)
{
    uint32_t len = 0;
    while(cdc_rx_tail != cdc_rx_head) 
    {
        Buf[len++] = cdc_rx_buf[cdc_rx_tail];
        cdc_rx_tail = (cdc_rx_tail + 1) % CDC_RX_BUFSIZE;
    }
    return len;
}


