#include "LED.h"
LED led1, led2;

/**
 * @brief LED初始化
 *
 * @param GPIOx
 * @param GPIO_Pin
 */
void LED::Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    this->GPIOx = GPIOx;
    this->GPIO_Pin = GPIO_Pin;
}

/**
 * @brief LED打开
 *
 */
void LED::On(void)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
/**
 * @brief LED关闭
 *
 */
void LED::Off(void)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}
/**
 * @brief LED翻转
 *
 */
void LED::Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}
