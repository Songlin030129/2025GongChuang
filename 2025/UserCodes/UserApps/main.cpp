#include "common_inc.h"
#include "Debug_Task.h"
#include "Chassis_Task.h"
#include "Key_Task.h"
#include "Command_Task.h"
#include "LED_Task.h"
static TaskHandle_t LED_Task_Handle;
static TaskHandle_t KEY_Task_Handle;
static TaskHandle_t DBG_Task_Handle;
static TaskHandle_t Chassis_Task_Handle;
static TaskHandle_t Command_Task_Handle;

void LEDTask(void* argument)
{
    LED_Task();
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
void CommandTask(void* arrgument)
{
    Command_Task();
}
// /*--------------------------主函数创建线程-------------------------*/
void Main()
{
    BaseType_t xReturn = pdTRUE;
    xReturn = xTaskCreate(LEDTask, "LEDTask", 128, NULL, osPriorityNormal, &LED_Task_Handle);
    if (xReturn == pdTRUE)
        printf("LED Task Create Success!\r\n");
    else
        printf("LED Task Create Fail\r\n");

    xReturn = xTaskCreate(KEYTask, "KEYTask", 128, NULL, osPriorityNormal, &KEY_Task_Handle);
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

    xReturn = xTaskCreate(CommandTask, "CommandTask", 512, NULL, osPriorityNormal1, &Command_Task_Handle);
    if (xReturn == pdTRUE)
        printf("Command Task Create Success!\r\n");
    else
        printf("Command Task Create Fail\r\n");

}
