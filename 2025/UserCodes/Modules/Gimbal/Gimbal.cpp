#include "Gimbal.h"
Gimbal gimbal;
void Gimbal::Init(CAN_HandleTypeDef *_hcan)
{
    dm_gimbal.Init(_hcan, 6, 5, CONTROL_MODE_MIT);
    zdt_hori.Init(_hcan, 1, ZDTMotor::DIRECTION_POSITIVE);
    zdt_vert.Init(_hcan, 2, ZDTMotor::DIRECTION_NEGATIVE);
    dm_gimbal.Enable();
    vTaskDelay(1);
    zdt_hori.Enable();
    vTaskDelay(1);
    zdt_vert.Enable();
    vTaskDelay(1);
    dm_gimbal.Enable();
    vTaskDelay(1);

    tar_gimbal_angle = ROTATE_ANGLE_OUT_2;

    servo_protocol.Init(&huart5);
    servo_jaw.init(3, &servo_protocol);
    if (servo_jaw.ping() != 1) printf("servo_jaw error\r\n");

    servo_jaw.setAngle(JAW_OPEN_ANGLE);
}

void Gimbal::loop_control()
{
    zdt_hori.GetPosition();
    zdt_vert.GetPosition();
    tar_gimbal_angle    = _constrain(tar_gimbal_angle, -4.71f, 1.57f);
    tar_gimbal_distance = _constrain(tar_gimbal_distance, 0.0f, 0.2f);
    tar_gimbal_height   = _constrain(tar_gimbal_height, 0.0f, 0.2f);
    gimbal_distance     = zdt_hori.Position / 9000.0f;
    gimbal_height       = zdt_vert.Position / 9000.0f;
    gimbal_angle        = dm_gimbal.Position;
    gimbal_omega        = dm_gimbal.Velocity;

    // 垂直电机位置闭环
    zdt_vert.SetVelocity(PID_Height.Cal(tar_gimbal_height - gimbal_height, 0.0f),
                         10000, ZDTMotor::MULTI_MODE_ASYNC);

    // 水平伸缩电机位置闭环
    zdt_hori.SetVelocity(PID_Distance.Cal(tar_gimbal_distance - gimbal_distance, 0.0f),
                         10000, ZDTMotor::MULTI_MODE_ASYNC);

    // 云台角度闭环
    dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f,
                      PID_Omega.Cal(PID_Angle.Cal(tar_gimbal_angle - gimbal_angle, 0.0f) - gimbal_omega, 0.0f));

    if (gimbal_control_mode == GIMBAL_POSITION_CONTROL) {
        // 云台位置控制
    } else if (gimbal_control_mode == GIMBAL_CAMERA_CONTROL) {
        // 云台锁定摄像头位置
        camera_x_err = LPF_ERR_X(camera_x_err);
        camera_y_err = LPF_ERR_Y(camera_y_err);

        float tar_vel_x = PID_CAM_X.Cal(-camera_x_err, 0.0f);
        if (gimbal_angle >= -2.0f) {
            if (tar_vel_x >= 0) tar_vel_x = 0;
        } else if (gimbal_angle <= -4.2f) {
            if (tar_vel_x <= 0) tar_vel_x = 0;
        }
        tar_gimbal_angle += tar_vel_x; // 云台角度自增

        float tar_vel_y = PID_CAM_Y.Cal(-camera_y_err, 0.0f);
        if (gimbal_distance <= 0.01f) {
            if (tar_vel_y <= 0) tar_vel_y = 0;
        } else if (gimbal_distance >= 0.18f) {
            if (tar_vel_y >= 0) tar_vel_y = 0;
        }
        tar_gimbal_distance += tar_vel_y; // 云台伸缩距离自增
    }
}
uint8_t Gimbal::Camera_Calibrated()
{
    return 0;
}

void Gimbal::Lift_Move(float _height)
{
    if (_height >= 0 && _height <= 0.2f)
        tar_gimbal_height = _height;
}
uint8_t Gimbal::Lift_Finished()
{
    if (fabs(tar_gimbal_height - gimbal_height) <= LIFT_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Gimbal::Extension_Move(float _distance)
{
    if (_distance >= 0.0f && _distance <= 0.2f)
        tar_gimbal_distance = _distance;
}
uint8_t Gimbal::Extension_Finished()
{
    if (fabs(tar_gimbal_distance - gimbal_distance) <= EXTENSION_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Gimbal::Rotate_Move(float _angle)
{
    if (_angle >= -4.71f && _angle <= 1.57f)
        tar_gimbal_angle = _angle;
}
uint8_t Gimbal::Rotate_Finished()
{
    if (fabs(tar_gimbal_angle - gimbal_angle) <= GIMBAL_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}
void Gimbal::Jaw_Move(float _angle)
{
    if (_angle >= JAW_OPEN_ANGLE && _angle <= JAW_CLOSE_ANGLE) {
        vTaskDelay(5);
        servo_jaw.setAngle(_angle, JAW_INTERVAL);
    }
}
uint8_t Gimbal::Jaw_Finished()
{
    vTaskDelay(5);
    if (fabs(servo_jaw.targetAngle - servo_jaw.queryAngle()) <= JAW_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}
