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
    uint8_t type;
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
    void UART_ErrorCallback(UART_HandleTypeDef* huart);

    void Set_Detect_Mode(uint8_t _mode, uint8_t _color);

    uint8_t valid_data_count;        // 连续有效数据计数器
    static const uint8_t VALID_DATA_THRESHOLD = 10;  // 有效数据阈值

    std::list<data_packet_t> recvlist;

    data_packet_t rxdata, txdata;

    //数据类型定义
    static constexpr uint8_t DATA_TYPE_SET_DETECT_MODE = 1;
    static constexpr uint8_t DATA_TYPE_QRCODE = 2;
    static constexpr uint8_t DATA_TYPE_TARGET_ERR = 3;

    //识别模式定义
    static constexpr uint8_t DETECT_MODE_NONE = 0;
    static constexpr uint8_t DETECT_MODE_QRCODE = 1;
    static constexpr uint8_t DETECT_MODE_MATERIAL = 2;
    static constexpr uint8_t DETECT_MODE_TARGET = 3;
    static constexpr uint8_t DETECT_MODE_BLOCK = 4;


    uint8_t qrcode_detected;
    uint8_t first_round_color1, first_round_color2, first_round_color3;
    uint8_t second_round_color1, second_round_color2, second_round_color3;

    uint8_t camera_data_valid = 0;
    float camera_raw_x_err, camera_raw_y_err;
private:
    uint8_t RxBuffer[HMI_RX_BUFFER_SIZE_MAX];
    UART_HandleTypeDef* huart;

};
extern HMI hmi;

#endif