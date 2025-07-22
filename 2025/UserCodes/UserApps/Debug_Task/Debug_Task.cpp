#include "Debug_Task.h"
#include "VOFA_Debug.h"
#include "OPS.h"
#include "DJIMotor.h"
#include "Gimbal.h"
#include "Chassis.h"
float tar_vel = 0;
void Debug_Task()
{
    debug.Init(&huart2);

    // debug.Add_ValueCommander("OP", &gimbal.PID_Omega.P);
    // debug.Add_ValueCommander("OI", &gimbal.PID_Omega.I);
    // debug.Add_ValueCommander("AP", &gimbal.PID_Angle.P);
    debug.Add_ValueCommander("PX", &gimbal.PID_CAM_X.P);
    debug.Add_ValueCommander("PY", &gimbal.PID_CAM_Y.P);


    while (1) {
        debug.Run_Debug();
        printf("gimbal_theta:%f, x_err:%f, x_out:%f, y_err:%f, y_out:%f\r\n",
            gimbal.gimbal_angle,
            gimbal.camera_x_err, gimbal.PID_CAM_X.output_value,
            gimbal.camera_y_err, gimbal.PID_CAM_Y.output_value);
        // printf(":%d, tar_vel:%f, vel:%f, angle:%f, P:%f, I:%f, OUT:%f\r\n",
        //     0,
        //     tar_vel,
        //     dji_mot3.motor_velocity,
        //     dji_mot3.motor_angle,
        //     dji_mot3.PID_Vel.P,
        //     dji_mot3.PID_Vel.I,
        //     dji_mot3.PID_Vel.output_value);
        // printf(":%d, OPS_X:%f, OPS_Y:%f, OPS_Yaw:%f, tar_X:%f, tar_Y:%f, tar_Yaw:%f\r\n",
        //     0,
        //     ops.Pos_X, ops.Pos_Y, ops.Yaw,
        //     chassis.Tar_Pos_X, chassis.Tar_Pos_Y, chassis.Tar_Angle);
        // printf(":%d, tar_angle:%f, angle:%f, vel:%f, OP:%f, OI:%f, AP:%f, out:%f\r\n",
        //     0,
        //     gimbal.tar_gimbal_angle, gimbal.gimbal_angle, gimbal.gimbal_omega,
        //     gimbal.PID_Omega.P, gimbal.PID_Omega.I, gimbal.PID_Angle.P,
        //     gimbal.PID_Omega.output_value);
        // printf(":%d, tar_omega:%f, omega:%f, P:%f, I:%f, out:%f\r\n",
        //     0,
        //     gimbal.tar_gimbal_omega, gimbal.gimbal_omega,
        //     gimbal.PID_Omega.P, gimbal.PID_Omega.I,
        //     gimbal.PID_Omega.output_value);
        vTaskDelay(10);
    }

}