#include "Debug_Task.h"
#include "VOFA_Debug.h"
#include "OPS.h"
#include "DJIMotor.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "Main_Task.h"
extern system_state_t global_state;
extern float run_enable;

void Debug_Task()
{
    debug.Init(&huart2);
    // debug.Add_ValueCommander("E", &run_enable);
    // debug.Add_ValueCommander("A", &gimbal.tar_jaw_angle);

    // debug.Add_ValueCommander("T", &gimbal.tar_rotate_angle);
    // debug.Add_ValueCommander("OP", &gimbal.PID_Omega.P);
    // debug.Add_ValueCommander("OI", &gimbal.PID_Omega.I);
    // debug.Add_ValueCommander("AP", &gimbal.PID_Angle.P);

    debug.Add_ValueCommander("PX", &gimbal.PID_CAM_X.P);
    debug.Add_ValueCommander("DX", &gimbal.PID_CAM_X.D);
    debug.Add_ValueCommander("OP", &gimbal.PID_CAM_Omega.P);
    debug.Add_ValueCommander("OI", &gimbal.PID_CAM_Omega.I);
    debug.Add_ValueCommander("PY", &gimbal.PID_CAM_Y.P);
    debug.Add_ValueCommander("DY", &gimbal.PID_CAM_Y.D);

    // debug.Add_ValueCommander("T", &gimbal.tar_vel_x);
    // debug.Add_ValueCommander("P", &gimbal.PID_CAM_Omega.P);
    // debug.Add_ValueCommander("I", &gimbal.PID_CAM_Omega.I);

    while (1) {
        debug.Run_Debug();
        // printf(":%d, lift_tar:%f, lift:%f, rotate_tar:%f, rotate:%f, extension_tar:%f, extension:%f\r\n",
        //     0,
        //     gimbal.trap_lift_tar, gimbal.lift_height,
        //     gimbal.trap_rotate_tar, gimbal.rotate_angle,
        //     gimbal.trap_extension_tar, gimbal.extension_distance);
        // printf(":%d, tar_dis:%f, dis:%f\r\n", 0, gimbal.tar_rotate_angle, gimbal.rotate_angle);
    // printf(":%d, ops_x:%f, ops_y:%f, chassis_tar_x:%f, chassis_tar_y:%f\r\n", 0,
    //     ops.Pos_X, ops.Pos_Y, chassis.Tar_Pos_X, chassis.Tar_Pos_Y);

    // printf(":%d, angle:%f, distance:%f, PX:%f, DX:%f, OP:%f, OI:%f, PY:%f, DY:%f, x_err:%f, x_out:%f, y_err:%f, y_out:%f\r\n", 0,
    //     gimbal.rotate_angle, gimbal.extension_distance,
    //     gimbal.PID_CAM_X.P, gimbal.PID_CAM_X.D,
    //     gimbal.PID_CAM_Omega.P, gimbal.PID_CAM_Omega.I,
    //     gimbal.PID_CAM_Y.P, gimbal.PID_CAM_Y.D,
    //     gimbal.camera_x_err, gimbal.PID_CAM_X.output_value,
    //     gimbal.camera_y_err, gimbal.PID_CAM_Y.output_value);

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
    //     gimbal.tar_rotate_angle, gimbal.rotate_angle, gimbal.rotate_omega,
    //     gimbal.PID_Omega.P, gimbal.PID_Omega.I, gimbal.PID_Angle.P,
    //     gimbal.PID_Omega.output_value);

    // printf(":%d, tar_omega:%f, omega:%f, P:%f, I:%f, OUT:%f\r\n", 0,
    //     gimbal.tar_vel_x, gimbal.rotate_omega,
    //     gimbal.PID_CAM_Omega.P, gimbal.PID_CAM_Omega.I,
    //     gimbal.PID_CAM_Omega.output_value);

        vTaskDelay(20);
    }

}