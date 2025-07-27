#ifndef __LCD_H__
#define __LCD_H__
#include "main.h"
#include "usart.h"
class LCD
{
public:
    UART_HandleTypeDef* huart;
    void SendData(uint16_t temp1, uint16_t temp2);
    void Init(UART_HandleTypeDef* _huart);
};
extern LCD lcd;
#endif
