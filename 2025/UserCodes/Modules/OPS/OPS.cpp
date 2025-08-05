#include "OPS.h"
OPS ops;
// 传感器相对小车中心的偏移量 (单位：米)
// 根据实际安装位置调整这些数值
#define sensor_offset_x  0.0f  // 传感器相对小车中心的x偏移量
#define sensor_offset_y  -0.07f  // 传感器相对小车中心的y偏移量

// function approximating the sine calculation by using fixed size array
// uses a 65 element lookup table and interpolation
// thanks to @dekutree for his work on optimizing this
float _sin(float a)
{
    // 16bit integer array for sine lookup. interpolation is used for better precision
    // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
    // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -_PI,_PI for 3217 steps)
    static uint16_t sine_array[65] = { 0, 804, 1608, 2411, 3212, 4011, 4808, 5602, 6393, 7180, 7962, 8740, 9512, 10279, 11039, 11793, 12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531, 18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595, 23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791, 27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957, 30274, 30572, 30853, 31114, 31357, 31581, 31786, 31972, 32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758, 32768 };
    unsigned int i = (unsigned int)(a * (64 * 4 * 256.0f / _2PI));
    int t1, t2, frac = i & 0xff;
    i = (i >> 8) & 0xff;
    if (i < 64)
    {
        t1 = sine_array[i];
        t2 = sine_array[i + 1];
    }
    else if (i < 128)
    {
        t1 = sine_array[128 - i];
        t2 = sine_array[127 - i];
    }
    else if (i < 192)
    {
        t1 = -sine_array[-128 + i];
        t2 = -sine_array[-127 + i];
    }
    else
    {
        t1 = -sine_array[256 - i];
        t2 = -sine_array[255 - i];
    }
    return (1.0f / 32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a)
{
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}

/**
 * @brief OPS初始化
 *
 * @param huart 串口句柄
 */
void OPS::Init(UART_HandleTypeDef* huart)
{
    this->huart = huart;
    // 停止可能正在进行的DMA传输
    HAL_UART_DMAStop(huart);

    // 清除所有串口错误标志位
    __HAL_UART_CLEAR_OREFLAG(huart);  // 清除溢出错误
    __HAL_UART_CLEAR_FEFLAG(huart);   // 清除帧错误
    __HAL_UART_CLEAR_PEFLAG(huart);   // 清除奇偶校验错误
    __HAL_UART_CLEAR_NEFLAG(huart);   // 清除噪声错误
    __HAL_UART_CLEAR_IDLEFLAG(huart); // 清除空闲标志

    // 清空接收FIFO中的残留数据
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
    {
        volatile uint8_t temp = huart->Instance->DR;
        (void)temp; // 避免编译器警告
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, OPS_RX_BUFFER_SIZE_MAX);
    Last_Pos_X = 0;
    Last_Pos_Y = 0;
    // 初始化零点偏移相关变量
    vTaskDelay(200);
    // Data_Reset();
}

void OPS::Data_Reset()
{
    HAL_UART_Transmit(huart, (uint8_t*)"ACT0", 5, 1000);
    vTaskDelay(200);
}
void OPS::Update_XY(float _x, float _y)
{
    // 准备数据包结构
    struct {
        char header[4];     // "ACTD" 或其他标识符
        float x_data;       // X坐标数据
        float y_data;       // Y坐标数据
    } data_packet;

    // 填充数据包
    memcpy(data_packet.header, "ACTD", 4);
    data_packet.x_data = _x * 1000.0f;  // 转换为毫米
    data_packet.y_data = -_y * 1000.0f;  // 转换为毫米

    // 发送二进制数据
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)&data_packet, sizeof(data_packet), 1000);
    if (status != HAL_OK) {
        printf("Error: Failed to send float data: %d\r\n", status);
    }

    vTaskDelay(200);

}
void OPS::Data_Calibrate()
{
    HAL_UART_Transmit(huart, (uint8_t*)"ACTR", 5, 1000);
    vTaskDelay(200);
}

/**
 * @brief 串口空闲中断回调函数（DMA）
 *
 * @param huart 串口句柄
 * @param Size 接收数据长度
 */
void OPS::UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == this->huart)
    {
        // 清除错误标志位
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);

        //获取时间间隔Ts
        uint32_t currentTime = HAL_GetTick();
        if (Last_Time == 0)
        {
            Last_Time = currentTime;
        }
        else
        {
            Ts = (currentTime - Last_Time) / 1000.0f;
            Last_Time = currentTime;
        }

        static union
        {
            uint8_t data[24];
            float Val[6];
        }ReadVal;

        if (RxBuffer[0] == 0x0D && RxBuffer[1] == 0x0A && RxBuffer[26] == 0x0A && RxBuffer[27] == 0x0D)
        {
            memcpy(ReadVal.data, RxBuffer + 2, 24);
            Yaw_raw = ReadVal.Val[0] * _PI / 180.0f;
            Pitch = ReadVal.Val[1];
            Roll = ReadVal.Val[2];
            Pos_X_Raw = ReadVal.Val[3] / 1000.0f;
            Pos_Y_Raw = -ReadVal.Val[4] / 1000.0f;
            W_Z = ReadVal.Val[5] * _PI / 180.0f;

            float temp = Yaw_raw - Yaw_Last;
            if (temp > _PI)
            {
                temp -= _2PI;
            }
            else if (temp < -_PI)
            {
                temp += _2PI;
            }
            Yaw -= temp;
            Yaw_Last = Yaw_raw;

            float yaw_normalized = Yaw;
            while (yaw_normalized < 0) yaw_normalized += _2PI;
            while (yaw_normalized >= _2PI) yaw_normalized -= _2PI;
            // 计算小车中心的全局坐标
            // 将传感器偏移量从小车坐标系转换到全局坐标系
            float cos_yaw = _cos(yaw_normalized);  // 将角度转换为弧度
            float sin_yaw = _sin(yaw_normalized);

            // 坐标变换：从传感器位置计算小车中心位置
            float Car_Center_X = Pos_X_Raw - (sensor_offset_x * cos_yaw - sensor_offset_y * sin_yaw);
            float Car_Center_Y = Pos_Y_Raw - (sensor_offset_x * sin_yaw + sensor_offset_y * cos_yaw);

            // 更新位置为小车中心坐标
            Pos_X = Car_Center_X + sensor_offset_x;
            Pos_Y = Car_Center_Y + sensor_offset_y;

            Vel_X = (Pos_X - Last_Pos_X) / Ts;
            Vel_Y = (Pos_Y - Last_Pos_Y) / Ts;

            Last_Pos_X = Pos_X;
            Last_Pos_Y = Pos_Y;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, OPS_RX_BUFFER_SIZE_MAX);
    }
}

void OPS::UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart == this->huart)
    {
        // 停止当前的DMA传输
        HAL_UART_DMAStop(huart);

        // 清除错误标志位
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);

        // 清空接收FIFO
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
        {
            volatile uint8_t temp = huart->Instance->DR;
            (void)temp;
        }

        // 清空缓冲区
        memset(this->RxBuffer, 0, OPS_RX_BUFFER_SIZE_MAX);

        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, OPS_RX_BUFFER_SIZE_MAX);
    }
}