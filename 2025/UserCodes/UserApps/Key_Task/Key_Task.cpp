#include "Key_Task.h"
#include "Key.h"
#include "HMI.h"
#include "Chassis.h"
#include "Motion.h"
#include "DMMotor.h"
#include "Motion.h"
#include "Main_Task.h"
#include "Gimbal.h"
extern float run_enable;

void Key_Task()
{
    keys.AddKey(&key1, KEY1_GPIO_Port, KEY1_Pin);
    keys.AddKey(&key2, KEY2_GPIO_Port, KEY2_Pin);
    keys.AddKey(&key3, KEY3_GPIO_Port, KEY3_Pin);
    keys.AddKey(&key4, KEY4_GPIO_Port, KEY4_Pin);
    while (1) {
        keys.KeysHandler();
        vTaskDelay(30);
    }
}

extern uint8_t test_enable;
/*------------------按键扫描回调函数-------------------------*/
void KEY_KeyClickCallback(KEY* key)
{
    if (key == &key1) {
        // dm_gimbal.SaveZeroPoint();
        run_enable = 1;
    }
    if (key == &key2) {
        // gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_IN_2);
        // gimbal.Lift_Move(gimbal.LIFT_DISTANCE_GROUND);
        // gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_1);
    }
    if (key == &key3) {
        // gimbal.Lift_Move(gimbal.LIFT_DISTANCE_TOP);
        // gimbal.Extension_Move(gimbal.EXTENSION_DISTANCE_OUT_2);
        // gimbal.Rotate_Move(gimbal.ROTATE_ANGLE_OUT_2);
        // chassis.control_enable = 0;
        // gimbal.control_enable = 0;
    }
    if (key == &key4) {
        chassis.Pos_Rst();
        NVIC_SystemReset();
    }
}
void KEY_MultipleClickCallback(KEY* key)
{
    if (key == &key1) {
    }
    if (key == &key2) {
    }
    if (key == &key3) {

    }
}
void KEY_LongHoldCallback(KEY* key)
{
    if (key == &key1) {
    }
    if (key == &key2) {
    }
    if (key == &key3) {

    }
}
void KEY_HoldTriggerCallback(KEY* key)
{
    if (key == &key1) {
    }
    if (key == &key2) {
    }
    if (key == &key3) {

    }
}
