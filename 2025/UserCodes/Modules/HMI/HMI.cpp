#include "HMI.h"
HMI hmi;
/**
 * @brief HMI初始化
 *
 * @param huart 串口句柄
 */
void HMI::Init(UART_HandleTypeDef* huart)
{
    this->huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, HMI_RX_BUFFER_SIZE_MAX);
}

/**
 * @brief 串口空闲中断回调函数（DMA）
 *
 * @param huart 串口句柄
 * @param Size 接收数据长度
 */
void HMI::UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == this->huart)
    {
        // 清除所有错误标志位
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        if (RxBuffer[0] == HEADER_BUFFER && RxBuffer[Size - 1] == FOOTER_BUFFER)
        {
            memcpy(&rxdata, RxBuffer, sizeof(data_packet_t));
            recvlist.push_back(rxdata);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, HMI_RX_BUFFER_SIZE_MAX);
    }
}
void HMI::UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart == this->huart)
    {
        // 停止当前的DMA传输
        HAL_UART_DMAStop(huart);

        // 清除所有错误标志位
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // 清空接收FIFO
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
        {
            volatile uint8_t temp = huart->Instance->DR;
            (void)temp;
        }

        // 清空缓冲区
        memset(this->RxBuffer, 0, HMI_RX_BUFFER_SIZE_MAX);

        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, HMI_RX_BUFFER_SIZE_MAX);
    }
}


void HMI::Set_Detect_Mode(uint8_t _mode, uint8_t _color)
{
    txdata.header = HEADER_BUFFER;
    txdata.type = DATA_TYPE_SET_DETECT_MODE;
    txdata.u_data1 = _mode;
    txdata.u_data2 = _color;
    txdata.f_data1 = 0.0f;
    txdata.f_data2 = 0.0f;
    txdata.footer = FOOTER_BUFFER;
    HAL_UART_Transmit(huart, (uint8_t*)&txdata, sizeof(data_packet_t), 1000);
}
