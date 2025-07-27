#include "Main_Task.h"
#include "LED.h"
#include "Paths.h"
#include "HMI.h"
#include "Motion.h"
uint8_t global_state = 0;
uint8_t run_enable = 0;
uint8_t qrcode_detected = 0;
uint8_t first_round_color1, first_round_color2, first_round_color3;
uint8_t second_round_color1, second_round_color2, second_round_color3;

void Main_Task()
{
    led1.Init(LED1_GPIO_Port, LED1_Pin);
    led2.Init(LED2_GPIO_Port, LED2_Pin);

    while (1)
    {
        if (global_state == 0) {
            if (run_enable == 1) {
                global_state = 1;
                run_enable = 0;
            }
        }
        else if (global_state == 1) {
            // uint8_t temp = paths.task_from_1_to_2();
            uint8_t temp = 1;
            if (temp == 1) {
                global_state = 2;
                hmi.Set_Detect_Mode(HMI::DETECT_MODE_QRCODE, 0);
            }
        }
        else if (global_state == 2) {
            if (qrcode_detected == 1) {
                qrcode_detected = 0;
                global_state = 3;
                hmi.Set_Detect_Mode(HMI::DETECT_MODE_MATERIAL, first_round_color1);
            }
        }
        else if (global_state == 3) {
            uint8_t temp = motion.load_from_material(1);
            if (temp == 1) {
                global_state = 4;
                hmi.Set_Detect_Mode(HMI::DETECT_MODE_MATERIAL, first_round_color2);
            }
        }
        else if (global_state == 4) {
            uint8_t temp = motion.load_from_material(2);
            if (temp == 1) {
                global_state = 5;
                hmi.Set_Detect_Mode(HMI::DETECT_MODE_MATERIAL, first_round_color3);
            }
        }
        else if (global_state == 5) {
            uint8_t temp = motion.load_from_material(3);
            if (temp == 1) {
                global_state = 6;
            }
        }

        vTaskDelay(5);
    }
}