#include "Chassis_Task.h"
#include "Chassis.h"
extern float tar_vel;
void Chassis_Task()
{
    chassis.Init(&hcan1);
    while (1) {

        chassis.Loop_Control();
        vTaskDelay(10);
    }
}