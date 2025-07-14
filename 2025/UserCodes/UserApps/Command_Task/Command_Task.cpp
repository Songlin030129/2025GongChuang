#include "Command_Task.h"
#include "Motion.h"
#include "Chassis.h"
#include "Paths.h"
#include "trapTraj.h"
#include "LCD.h"
#include "HMI.h"

#define COMMAND_TYPE_YUNTAI 1
#define COMMAND_TYPE_HUAGUI 2
#define COMMAND_TYPE_CHASSIS_PATH 3
#define COMMAND_TYPE_CHASSIS_VELOCITY 4
#define COMMAND_TYPE_CHASSIS_POSITION 5
#define COMMAND_TYPE_CHASSIS_ANGLE 6
#define COMMAND_TYPE_LCD_SHOW 7
#define COMMAND_TYPE_LOAD 8
#define COMMAND_TYPE_UNLOAD 9
#define COMMAND_TYPE_GET 10

#define CHASSIS_VEL 0.25f
#define CHASSIS_ACC 0.25f
#define CHASSIS_VEL_LOW 0.20f
#define CHASSIS_ACC_LOW 0.20f

uint8_t first_distance_command_flag = 0;
TrapezoidalTrajectory traj_x, traj_y;
uint32_t Last_Time;

void Command_Task()
{
    // hmi.Init(&huart1);
    // motion.Init();
    // LCD_Init();
    while (1)
    {
        //获取时间间隔Ts
        uint32_t currentTime = HAL_GetTick();
        if (Last_Time == 0)
        {
            Last_Time = currentTime;
        }
        float Ts = (currentTime - Last_Time) / 1000.0f;
        Last_Time = currentTime;

        if (!hmi.recvlist.empty())
        {
            for (std::list<data_packet_t>::iterator it = hmi.recvlist.begin(); it != hmi.recvlist.end(); ++it) {
                if (it->command_type == COMMAND_TYPE_CHASSIS_PATH)
                {
                    first_distance_command_flag = 1;

                    chassis.pid_pos_x.output_limit = 1;
                    chassis.pid_pos_y.output_limit = 1;

                    if (it->u_data1 == 0 && it->u_data2 == 1)
                    {
                        uint8_t temp = paths.task_from_1_to_2();
                        motion.Camera_Out();
                        motion.Yuntai_Out();
                        if (temp == 1)
                        {
                            hmi.recvlist.erase(it);
                            hmi.Transmit(COMMAND_TYPE_CHASSIS_PATH, 0, 0, 0, 0, 0);
                            printf("Finished!\r\n");
                            data_packet_t temp = { 0 };
                            temp.u_data1 = 1;
                            temp.u_data2 = 2;
                            temp.command_type = 3;
                            hmi.recvlist.push_back(temp);

                        }
                    }
                    else if (it->u_data1 == 1 && it->u_data2 == 2)
                    {
                        uint8_t temp = paths.task_from_2_to_3();
                        if (temp == 1)
                        {
                            hmi.recvlist.erase(it);
                            hmi.Transmit(COMMAND_TYPE_CHASSIS_PATH, 0, 0, 0, 0, 0);
                            printf("Finished!\r\n");
                            data_packet_t temp = { 0 };
                            temp.u_data1 = 2;
                            temp.u_data2 = 3;
                            temp.command_type = 3;
                            hmi.recvlist.push_back(temp);
                        }
                    }
                    else if (it->u_data1 == 2 && it->u_data2 == 3)
                    {
                        uint8_t des_pos = it->command_state & 0x0F;
                        uint8_t temp = paths.task_from_3_to_7(2);
                        if (temp == 1)
                        {
                            hmi.recvlist.erase(it);
                            hmi.Transmit(COMMAND_TYPE_CHASSIS_PATH, 0, 0, 0, 0, 0);
                            printf("Finished!\r\n");
                            data_packet_t temp = { 0 };
                            temp.u_data1 = 3;
                            temp.u_data2 = 4;
                            temp.command_type = 3;
                            hmi.recvlist.push_back(temp);

                        }
                    }
                    else if (it->u_data1 == 3 && it->u_data2 == 4)
                    {
                        uint8_t src_pos = (it->command_state & 0xF0) >> 4;
                        uint8_t des_pos = it->command_state & 0x0F;
                        uint8_t temp = paths.task_from_7_to_10(src_pos, des_pos);
                        if (temp == 1)
                        {
                            hmi.recvlist.erase(it);
                            hmi.Transmit(COMMAND_TYPE_CHASSIS_PATH, 0, 0, 0, 0, 0);
                            printf("Finished!\r\n");
                            data_packet_t temp = { 0 };
                            temp.u_data1 = 4;
                            temp.u_data2 = 0;
                            temp.command_type = 3;
                            hmi.recvlist.push_back(temp);

                        }
                    }
                    else if (it->u_data1 == 4 && it->u_data2 == 2)
                    {
                        uint8_t src_pos = (it->command_state & 0xF0) >> 4;
                        uint8_t temp = paths.task_from_10_to_3(src_pos);
                        if (temp == 1)
                        {
                            hmi.recvlist.erase(it);
                            hmi.Transmit(COMMAND_TYPE_CHASSIS_PATH, 0, 0, 0, 0, 0);
                            printf("Finished!\r\n");
                        }
                    }
                    else if (it->u_data1 == 4 && it->u_data2 == 0)
                    {
                        uint8_t src_pos = (it->command_state & 0xF0) >> 4;
                        uint8_t temp = paths.task_from_10_to_1(src_pos);
                        if (temp == 1)
                        {
                            hmi.recvlist.erase(it);
                            hmi.Transmit(COMMAND_TYPE_CHASSIS_PATH, 0, 0, 0, 0, 0);
                            printf("Finished!\r\n");
                        }
                    }
                }
            }

        }

        vTaskDelay(5);
    }

}


