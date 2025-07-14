#ifndef __OPS_H
#define __OPS_H

#include "common_inc.h"
#define OPS_RX_BUFFER_SIZE_MAX 100
class OPS
{
public:
    /**
     * @brief OPS初始化
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
    void UART_ErrorCallback(UART_HandleTypeDef* huart);

    float Yaw, Pitch, Roll, W_Z;
    float Pos_X, Pos_Y, Last_Pos_X, Last_Pos_Y;
    float Vel_X, Vel_Y;

    float Yaw_raw;
    float Yaw_Last;

    void Data_Reset();

    //   private:
    uint8_t RxBuffer[OPS_RX_BUFFER_SIZE_MAX];
    UART_HandleTypeDef* huart;
    uint32_t Last_Time;
    float Ts;

};
extern OPS ops;

#endif