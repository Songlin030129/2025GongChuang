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
        if (RxBuffer[0] == HEADER_BUFFER && RxBuffer[Size - 1] == FOOTER_BUFFER)
        {
            memcpy(&rxdata, RxBuffer, sizeof(data_packet_t));
            recvlist.push_back(rxdata);
            printf("HMI Received type:%d, state:0x%x, udata1:%d, udata2:%d, fdata1:%f, fdata2:%f\r\n",
                rxdata.command_type, rxdata.command_state, rxdata.u_data1, rxdata.u_data2, rxdata.f_data1, rxdata.f_data2);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, HMI_RX_BUFFER_SIZE_MAX);
    }
}

void HMI::Transmit(uint8_t _command_type, uint8_t _command_state, uint8_t _u_data1, uint8_t _u_data2, float _f_data1, float _f_data2)
{
    txdata.header = HEADER_BUFFER;
    txdata.command_type = _command_type;
    txdata.command_state = _command_state;

    txdata.u_data1 = _u_data1;
    txdata.u_data2 = _u_data2;

    txdata.f_data1 = _f_data1;
    txdata.f_data2 = _f_data2;
    txdata.footer = FOOTER_BUFFER;
    HAL_UART_Transmit(huart, (uint8_t*)&txdata, sizeof(data_packet_t), 1000);
}
