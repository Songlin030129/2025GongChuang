#ifndef __HWT101_H
#define __HWT101_H

#include "common_inc.h"
#define HWT101_RX_BUFFER_SIZE_MAX 50
class HWT101
{
public:
    /**
     * @brief HWT101初始化
     *
     * @param huart 串口句柄
     */
    void Init(UART_HandleTypeDef* huart);
    /**
     * @brief 串口空闲中断回调函数（DMA）
     *
     * @param huart 串口句柄
     * @param Size 接收数据长度
     */
    void UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size);

    float Yaw;
    float Gyro;
    //   private:
    float Yaw_raw;
    float Yaw_Last;
    uint8_t RxBuffer[HWT101_RX_BUFFER_SIZE_MAX];
    UART_HandleTypeDef* huart;

};
extern HWT101 imu;

#endif