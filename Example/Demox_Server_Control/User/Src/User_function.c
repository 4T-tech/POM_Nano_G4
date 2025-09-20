#include "User_function.h"

/**
  * @brief  �� USB CDC ���λ�������ȡ���ݵ�ָ���ڴ�
  *
  * @note   �ú�����ȫ�ֻ��λ����� `cdc_rx_buf` �ж�ȡ���д��������ݣ�
  *         ��ȡ������ݻ�ӻ������Ƴ����ƶ�βָ�룩��
  *         ��������ѭ���з��жϻ����´��� USB �������ݡ�
  *
  * @warning
  *         - ����������δ���жϱ����������жϺ���ѭ����ͬʱ���ʻ�������
  *           �����ǰ�����жϣ��� `__disable_irq()`��
  *         - ��ȷ�� `cdc_rx_buf`��`cdc_rx_head`��`cdc_rx_tail` ��
  *           `CDC_RX_BUFSIZE` ����ȷ����
  *
  * @param  Buf : Ŀ��洢������ָ�룬��ȡ�����ݽ����������ڴ�
  * @retval uint16_t : ʵ�ʶ�ȡ�����ݳ��ȣ��ֽ����������� 0 ��ʾ�����ݿɶ�
  *
  * @usage
  * @code
  * uint8_t data[64];
  * uint16_t len = Read_USB_CDC_Buf(data);
  * if(len > 0) {
  *     // ��������...
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


