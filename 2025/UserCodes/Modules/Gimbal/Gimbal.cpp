#include "Gimbal.h"
Gimbal gimbal;
void Gimbal::Init(CAN_HandleTypeDef* _hcan)
{
    dm_gimbal.Init(_hcan, 6, 5, CONTROL_MODE_VEL);
    zdt_hori.Init(_hcan, 1, ZDTMotor::DIRECTION_POSITIVE);
    zdt_vert.Init(_hcan, 2, ZDTMotor::DIRECTION_NEGATIVE);
    dm_gimbal.Enable();
    vTaskDelay(1);
    zdt_hori.Enable();
    vTaskDelay(1);
    zdt_vert.Enable();
    vTaskDelay(1);
    dm_gimbal.Enable();
    vTaskDelay(100);

    tar_gimbal_angle = -3.14f;
}

void Gimbal::loop_control()
{
    zdt_hori.GetPosition();
    zdt_vert.GetPosition();
    tar_gimbal_angle = _constrain(tar_gimbal_angle, -4.71f, 1.57f);
    tar_gimbal_distance = _constrain(tar_gimbal_distance, 0.0f, 0.2f);
    tar_gimbal_height = _constrain(tar_gimbal_height, 0.0f, 0.2f);
    gimbal_distance = zdt_hori.Position / 9000.0f;
    gimbal_height = zdt_vert.Position / 9000.0f;
    gimbal_angle = dm_gimbal.Position;


    zdt_vert.SetVelocity(PID_Height.Cal(tar_gimbal_height - gimbal_height, 0.0f),
        10000, ZDTMotor::MULTI_MODE_ASYNC);
    if (GIMBAL_CONTROL_MODE == GIMBAL_POSITION_CONTROL)
    {
        zdt_hori.SetVelocity(PID_Distance.Cal(tar_gimbal_distance - gimbal_distance, 0.0f),
            10000, ZDTMotor::MULTI_MODE_ASYNC);

        dm_gimbal.Control(PID_Angle.Cal(tar_gimbal_angle - gimbal_angle, 0.0f));
    }
    if (GIMBAL_CONTROL_MODE == GIMBAL_CAMERA_CONTROL)
    {
        camera_x_err = LPF_ERR_X(camera_x_err);
        camera_y_err = LPF_ERR_Y(camera_y_err);
        float tar_vel_x = PID_CAM_X.Cal(-camera_x_err, 0.0f);
        if (gimbal_angle >= -2.0f)
        {
            if (tar_vel_x >= 0) tar_vel_x = 0;
        }
        else if (gimbal_angle <= -4.2f)
        {
            if (tar_vel_x <= 0)tar_vel_x = 0;
        }
        dm_gimbal.Control(tar_vel_x);
        float tar_vel_y = PID_CAM_Y.Cal(-camera_y_err, 0.0f);
        if (gimbal_distance <= 0.01f)
        {
            if (tar_vel_y <= 0) tar_vel_y = 0;
        }
        else if (gimbal_distance >= 0.18f)
        {
            if (tar_vel_y >= 0) tar_vel_y = 0;
        }
        zdt_hori.SetVelocity(tar_vel_y, 20000, ZDTMotor::MULTI_MODE_ASYNC);
    }
}
