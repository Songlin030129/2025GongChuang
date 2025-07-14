#ifndef __HMI_H__
#define __HMI_H__

#include "common_inc.h"
#include <list>
#define HMI_RX_BUFFER_SIZE_MAX 50
#define HEADER_BUFFER 0xFF
#define FOOTER_BUFFER 0xFE
#pragma pack(push, 1) // 设置结构体对齐为 1 字节
typedef struct
{
    uint8_t header;
    uint8_t command_type;
    uint8_t command_state;
    uint8_t u_data1;
    uint8_t u_data2;
    float f_data1;
    float f_data2;
    uint8_t footer;
} data_packet_t;
#pragma pack(pop) // 恢复默认对齐方式

class HMI
{
public:
    /**
     * @brief HMI初始化
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

    void Transmit(uint8_t _command_type, uint8_t _command_state, uint8_t _u_data1, uint8_t _u_data2, float _f_data1, float _f_data2);

    std::list<data_packet_t> recvlist;

    data_packet_t rxdata, txdata;

private:
    uint8_t RxBuffer[HMI_RX_BUFFER_SIZE_MAX];
    UART_HandleTypeDef* huart;

};
extern HMI hmi;

#endif