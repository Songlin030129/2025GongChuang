#include "common_inc.h"
#include "Debug_Task.h"
#include "Chassis_Task.h"
#include "Key_Task.h"
#include "HMI_Task.h"
#include "Main_Task.h"
#include "Gimbal_Task.h"
#include "LED.h"
static TaskHandle_t Main_Task_Handle;
static TaskHandle_t KEY_Task_Handle;
static TaskHandle_t DBG_Task_Handle;
static TaskHandle_t Chassis_Task_Handle;
static TaskHandle_t HMI_Task_Handle;
static TaskHandle_t Gimbal_Task_Handle;

void MainTask(void* argument)
{
    Main_Task();
}
void KEYTask(void* argument)
{
    Key_Task();
}
void DBGTask(void* argument)
{
    Debug_Task();
}
void ChassisTask(void* argument)
{
    Chassis_Task();
}
void HMITask(void* arrgument)
{
    HMI_Task();
}
void GimbalTask(void* arrgument)
{
    Gimbal_Task();
}

// /*--------------------------主函数创建线程-------------------------*/
void Main()
{
    led1.Init(LED1_GPIO_Port, LED1_Pin);
    led2.Init(LED2_GPIO_Port, LED2_Pin);

    BaseType_t xReturn = pdTRUE;
    xReturn = xTaskCreate(MainTask, "MainTask", 1024, NULL, osPriorityNormal, &Main_Task_Handle);
    if (xReturn == pdTRUE)
        printf("Main Task Create Success!\r\n");
    else
        printf("Main Task Create Fail\r\n");

    xReturn = xTaskCreate(KEYTask, "KEYTask", 256, NULL, osPriorityNormal, &KEY_Task_Handle);
    if (xReturn == pdTRUE)
        printf("KEY Task Create Success!\r\n");
    else
        printf("KEY Task Create Fail\r\n");

    xReturn = xTaskCreate(DBGTask, "DBGTask", 256, NULL, osPriorityNormal, &DBG_Task_Handle);
    if (xReturn == pdTRUE)
        printf("DBG Task Create Success!\r\n");
    else
        printf("DBG Task Create Fail\r\n");

    xReturn = xTaskCreate(ChassisTask, "ChassisTask", 512, NULL, osPriorityNormal1, &Chassis_Task_Handle);
    if (xReturn == pdTRUE)
        printf("Chassis Task Create Success!\r\n");
    else
        printf("Chassis Task Create Fail\r\n");

    xReturn = xTaskCreate(GimbalTask, "GimbalTask", 512, NULL, osPriorityNormal1, &Gimbal_Task_Handle);
    if (xReturn == pdTRUE)
        printf("Gimbal Task Create Success!\r\n");
    else
        printf("Gimbal Task Create Fail\r\n");

    // xReturn = xTaskCreate(HMITask, "HMITask", 512, NULL, osPriorityNormal1, &HMI_Task_Handle);
// if (xReturn == pdTRUE)
//     printf("HMI Task Create Success!\r\n");
// else
//     printf("HMI Task Create Fail\r\n");


}
