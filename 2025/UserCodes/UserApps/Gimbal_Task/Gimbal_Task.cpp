#include "Gimbal.h"
#include "Gimbal_Task.h"

void Gimbal_Task()
{
    vTaskDelay(500);
    gimbal.Init(&hcan2);
    while (1)
    {
        gimbal.loop_control();
        vTaskDelay(5);
    }
}