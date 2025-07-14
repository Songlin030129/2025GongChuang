#include "HWT101.h"
HWT101 imu;
/**
 * @brief HWT101初始化
 *
 * @param huart 串口句柄
 */
void HWT101::Init(UART_HandleTypeDef* huart)
{
    this->huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, HWT101_RX_BUFFER_SIZE_MAX);

    uint8_t temp[5] = { 0xFF, 0xAA, 0x76, 0x00, 0x00 };
    HAL_UART_Transmit(huart, temp, 5, 0xffff);
}

/**
 * @brief 串口空闲中断回调函数（DMA）
 *
 * @param huart 串口句柄
 * @param Size 接收数据长度
 */
void HWT101::UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == this->huart)
    {
        int16_t temp1;
        temp1 = (RxBuffer[18] << 8 | RxBuffer[17]);
        Yaw_raw = (float)temp1 / 32768 * 180;

        temp1 = (RxBuffer[7] << 8 | RxBuffer[6]);
        Gyro = -(float)temp1 / 32768 * 2000;

        float temp = Yaw_raw - Yaw_Last;
        if (temp > 180)
        {
            temp -= 360;
        }
        else if (temp < -180)
        {
            temp += 360;
        }
        Yaw -= temp;
        Yaw_Last = Yaw_raw;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, HWT101_RX_BUFFER_SIZE_MAX);
    }
}