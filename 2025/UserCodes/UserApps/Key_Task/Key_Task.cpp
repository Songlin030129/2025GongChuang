#include "Key_Task.h"
#include "Key.h"
#include "HMI.h"
#include "Chassis.h"
#include "Motion.h"
void Key_Task()
{
    keys.AddKey(&key1, KEY1_GPIO_Port, KEY1_Pin);
    keys.AddKey(&key2, KEY2_GPIO_Port, KEY2_Pin);
    keys.AddKey(&key3, KEY3_GPIO_Port, KEY3_Pin);
    while (1) {
        keys.KeysHandler();
        vTaskDelay(20);
    }
}


/*------------------按键扫描回调函数-------------------------*/
void KEY_KeyClickCallback(KEY* key)
{
    if (key == &key1) {
        hmi.Transmit(0, 1, 0, 0, 0, 0);
        motion.Camera_Out();
        motion.Yuntai_Out();
    }
    if (key == &key2) {
        data_packet_t temp = { 0 };
        temp.u_data1 = 0;
        temp.u_data2 = 1;
        temp.command_type = 3;
        hmi.recvlist.push_back(temp);
    }
    if (key == &key3) {
        chassis.Pos_Rst();

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
