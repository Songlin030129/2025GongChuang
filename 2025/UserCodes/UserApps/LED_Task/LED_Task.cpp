#include "LED_Task.h"
#include "LED.h"
void LED_Task()
{
    led1.Init(LED1_GPIO_Port, LED1_Pin);
    led2.Init(LED2_GPIO_Port, LED2_Pin);
    while (1) {
        led1.Toggle();
        led2.Toggle();
        vTaskDelay(1000);
    }

}