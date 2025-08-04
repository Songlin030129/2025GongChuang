#include "HMI_Task.h"
#include "LCD.h"
#include "HMI.h"
#include "Gimbal.h"
extern uint8_t qrcode_detected;
extern uint8_t first_round_color1, first_round_color2, first_round_color3;
extern uint8_t second_round_color1, second_round_color2, second_round_color3;

void HMI_Task()
{
    while (1)
    {
        if (!hmi.recvlist.empty())
        {
            for (std::list<data_packet_t>::iterator it = hmi.recvlist.begin(); it != hmi.recvlist.end(); ++it) {
                if (it->type == HMI::DATA_TYPE_QRCODE)
                {
                    switch (it->u_data1)
                    {
                    case 1: // 123
                        first_round_color1 = 1;
                        first_round_color2 = 2;
                        first_round_color3 = 3;
                        break;
                    case 2: // 132
                        first_round_color1 = 1;
                        first_round_color2 = 3;
                        first_round_color3 = 2;
                        break;
                    case 3: // 213
                        first_round_color1 = 2;
                        first_round_color2 = 1;
                        first_round_color3 = 3;
                        break;
                    case 4: // 231
                        first_round_color1 = 2;
                        first_round_color2 = 3;
                        first_round_color3 = 1;
                        break;
                    case 5: // 312
                        first_round_color1 = 3;
                        first_round_color2 = 1;
                        first_round_color3 = 2;
                        break;
                    case 6: // 321
                        first_round_color1 = 3;
                        first_round_color2 = 2;
                        first_round_color3 = 1;
                        break;
                    default:
                        break;
                    }
                    switch (it->u_data2)
                    {
                    case 1: // 123
                        second_round_color1 = 1;
                        second_round_color2 = 2;
                        second_round_color3 = 3;
                        break;
                    case 2: // 132
                        second_round_color1 = 1;
                        second_round_color2 = 3;
                        second_round_color3 = 2;
                        break;
                    case 3: // 213
                        second_round_color1 = 2;
                        second_round_color2 = 1;
                        second_round_color3 = 3;
                        break;
                    case 4: // 231
                        second_round_color1 = 2;
                        second_round_color2 = 3;
                        second_round_color3 = 1;
                        break;
                    case 5: // 312
                        second_round_color1 = 3;
                        second_round_color2 = 1;
                        second_round_color3 = 2;
                        break;
                    case 6: // 321
                        second_round_color1 = 3;
                        second_round_color2 = 2;
                        second_round_color3 = 1;
                        break;
                    default:
                        break;
                    }

                    lcd.SendData(it->u_data1, it->u_data2);
                    hmi.recvlist.erase(it);
                    lcd.SendData(it->u_data1, it->u_data2);
                    qrcode_detected = 1;
                }
                else if (it->type == HMI::DATA_TYPE_TARGET_ERR)
                {
                    if (it->f_data1 <= 9000.0f && it->f_data2 <= 9000.0f)
                    {
                        gimbal.camera_data_valid = 1;
                        gimbal.camera_detect_color = it->u_data1;
                        gimbal.camera_raw_x_err = it->f_data1;
                        gimbal.camera_raw_y_err = it->f_data2;
                    }
                    else
                    {
                        gimbal.camera_data_valid = 0;
                    }
                    hmi.recvlist.erase(it);
                }
            }
        }
        vTaskDelay(10);
    }

}


