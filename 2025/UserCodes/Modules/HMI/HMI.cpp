#include "HMI.h"
#include "LCD.h"
#include "Gimbal.h"
HMI hmi;
extern uint8_t qrcode_detected;
extern uint8_t first_round_color1, first_round_color2, first_round_color3;
extern uint8_t second_round_color1, second_round_color2, second_round_color3;

/**
 * @brief HMI初始化
 *
 * @param huart 串口句柄
 */
void HMI::Init(UART_HandleTypeDef* huart)
{
    this->huart = huart;
    this->valid_data_count = 0;  // 初始化计数器
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
            if (rxdata.type == HMI::DATA_TYPE_QRCODE)
            {
                switch (rxdata.u_data1)
                {
                case 1: // 123
                    first_round_color1 = 1;
                    first_round_color2 = 2;
                    first_round_color3 = 3;
                    break;
                case 2: // 132
                    first_round_color1 = 1;
                    first_round_color2 = 3;
                    first_round_color3 = 2;
                    break;
                case 3: // 213
                    first_round_color1 = 2;
                    first_round_color2 = 1;
                    first_round_color3 = 3;
                    break;
                case 4: // 231
                    first_round_color1 = 2;
                    first_round_color2 = 3;
                    first_round_color3 = 1;
                    break;
                case 5: // 312
                    first_round_color1 = 3;
                    first_round_color2 = 1;
                    first_round_color3 = 2;
                    break;
                case 6: // 321
                    first_round_color1 = 3;
                    first_round_color2 = 2;
                    first_round_color3 = 1;
                    break;
                default:
                    break;
                }
                switch (rxdata.u_data2)
                {
                case 1: // 123
                    second_round_color1 = 1;
                    second_round_color2 = 2;
                    second_round_color3 = 3;
                    break;
                case 2: // 132
                    second_round_color1 = 1;
                    second_round_color2 = 3;
                    second_round_color3 = 2;
                    break;
                case 3: // 213
                    second_round_color1 = 2;
                    second_round_color2 = 1;
                    second_round_color3 = 3;
                    break;
                case 4: // 231
                    second_round_color1 = 2;
                    second_round_color2 = 3;
                    second_round_color3 = 1;
                    break;
                case 5: // 312
                    second_round_color1 = 3;
                    second_round_color2 = 1;
                    second_round_color3 = 2;
                    break;
                case 6: // 321
                    second_round_color1 = 3;
                    second_round_color2 = 2;
                    second_round_color3 = 1;
                    break;
                default:
                    break;
                }

                lcd.SendData(rxdata.u_data1, rxdata.u_data2);
                lcd.SendData(rxdata.u_data1, rxdata.u_data2);
                qrcode_detected = 1;
            }
            else if (rxdata.type == HMI::DATA_TYPE_TARGET_ERR)
            {
                // 检查数据是否有效
                if (rxdata.f_data1 <= 9000.0f && rxdata.f_data2 <= 9000.0f)
                {
                    // 有效数据，计数器递增
                    if (valid_data_count < VALID_DATA_THRESHOLD)
                    {
                        valid_data_count++;
                    }

                    // 连续检测到足够次数的有效数据后才启用
                    if (valid_data_count >= VALID_DATA_THRESHOLD)
                    {
                        gimbal.camera_data_valid = 1;
                        gimbal.camera_detect_color = rxdata.u_data1;
                        gimbal.camera_raw_x_err = rxdata.f_data1;
                        gimbal.camera_raw_y_err = rxdata.f_data2;
                    }
                    else
                    {
                        // 还没达到阈值，暂时不启用但更新数据
                        gimbal.camera_data_valid = 0;
                        gimbal.camera_detect_color = rxdata.u_data1;
                        gimbal.camera_raw_x_err = rxdata.f_data1;
                        gimbal.camera_raw_y_err = rxdata.f_data2;
                    }
                }
                else
                {
                    // 无效数据，重置计数器和状态
                    valid_data_count = 0;
                    gimbal.camera_data_valid = 0;
                    gimbal.camera_raw_x_err = 0;
                    gimbal.camera_raw_y_err = 0;
                }
            }
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
