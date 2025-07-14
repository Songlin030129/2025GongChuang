#ifndef __LED_H__
#define __LED_H__

#include "common_inc.h"
class LED
{
public:
    /**
     * @brief LED初始化
     *
     * @param GPIOx
     * @param GPIO_Pin
     */
    void Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

    /**
     * @brief LED打开
     *
     */
    void  On(void);
    /**
     * @brief LED关闭
     *
     */
    void  Off(void);
    /**
     * @brief LED翻转
     *
     */
    void  Toggle(void);

private:
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};
extern LED led1, led2;

#endif // __LED_H__