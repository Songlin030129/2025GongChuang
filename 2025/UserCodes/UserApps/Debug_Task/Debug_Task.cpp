#include "Debug_Task.h"
#include "VOFA_Debug.h"
#include "OPS.h"
#include "DJIMotor.h"
#include "Chassis.h"
float tar_vel = 0;
void Debug_Task()
{
    debug.Init(&huart2);
    while (1) {
        debug.Run_Debug();
        // printf("0:%d, tar_vel:%f, vel:%f, angle:%f, P:%f, I:%f, OUT:%f\r\n",
        //     0,
        //     tar_vel,
        //     dji_mot3.motor_velocity,
        //     dji_mot3.motor_angle,
        //     dji_mot3.PID_Vel.P,
        //     dji_mot3.PID_Vel.I,
        //     dji_mot3.PID_Vel.output_value);
        printf(":%d, OPS_X:%f, OPS_Y:%f, OPS_Yaw:%f, tar_X:%f, tar_Y:%f, tar_Yaw:%f\r\n",
            0,
            ops.Pos_X, ops.Pos_Y, ops.Yaw,
            chassis.Tar_Pos_X, chassis.Tar_Pos_Y, chassis.Tar_Angle);
        vTaskDelay(10);
    }

}