#include "bsp_uart.h"
#include "VOFA_Debug.h"
#include "OPS.h"
#include "FSUS_Servo.h"
#include "HMI.h"
/*------------------串口空闲中断回调函数-------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    debug.UartReceive_IDLE_DMA_Callback(huart, Size);
    ops.UartReceive_IDLE_DMA_Callback(huart, Size);
    servo_protocol.UartReceive_IDLE_DMA_Callback(huart, Size);
    hmi.UartReceive_IDLE_DMA_Callback(huart, Size);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    ops.UART_ErrorCallback(huart);
}