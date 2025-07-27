#include "Gimbal.h"
Gimbal gimbal;
void Gimbal::Init(CAN_HandleTypeDef* _hcan)
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

    tar_rotate_angle = ROTATE_ANGLE_OUT_2;
    tar_extension_distance = EXTENSION_DISTANCE_OUT_2;

    servo_protocol.Init(&huart4);
    servo_jaw.init(3, &servo_protocol);
    if (servo_jaw.ping() != 1) printf("servo_jaw error\r\n");

    tar_jaw_angle = JAW_ANGLE_OPEN;
}

void Gimbal::loop_control()
{
    //获取时间间隔Ts
    uint32_t current_tick = HAL_GetTick();
    if (last_tick == 0)
    {
        last_tick = current_tick;
        return;
    }
    ts = (current_tick - last_tick) * 0.001f;
    last_tick = current_tick;

    zdt_hori.GetPosition();
    zdt_vert.GetPosition();
    tar_rotate_angle = _constrain(tar_rotate_angle, -4.71f, 1.57f);
    tar_extension_distance = _constrain(tar_extension_distance, 0.0f, 0.2f);
    tar_lift_height = _constrain(tar_lift_height, 0.0f, 0.2f);

    rotate_angle = dm_gimbal.Position;
    rotate_omega = LPF_OMEGA((rotate_angle - last_rotate_angle) / ts);
    last_rotate_angle = rotate_angle;
    extension_distance = zdt_hori.Position / 9000.0f;
    lift_height = zdt_vert.Position / 9000.0f;
    jaw_angle = servo_jaw.queryAngle();
    camera_x_err = LPF_ERR_X(camera_raw_x_err);
    camera_y_err = LPF_ERR_Y(camera_raw_y_err);

    servo_jaw.setAngle(tar_jaw_angle, JAW_INTERVAL);

    // 垂直电机位置闭环
    zdt_vert.SetVelocity(PID_Height.Cal(tar_lift_height - lift_height, 0.0f),
        10000, ZDTMotor::MULTI_MODE_ASYNC);

    if (gimbal_control_mode == GIMBAL_POSITION_CONTROL) {
        // 云台位置控制
        // 水平伸缩电机位置闭环
        zdt_hori.SetVelocity(PID_Distance.Cal(tar_extension_distance - extension_distance, 0.0f),
            10000, ZDTMotor::MULTI_MODE_ASYNC);

        // 云台角度闭环
        dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f,
            PID_Omega.Cal(PID_Angle.Cal(tar_rotate_angle - rotate_angle, 0.0f) - rotate_omega, 0.0f));
    }
    else if (gimbal_control_mode == GIMBAL_CAMERA_CONTROL) {
        // 云台锁定摄像头位置闭环
        if (camera_data_enable) {
            tar_rotate_angle = rotate_angle;
            tar_extension_distance = extension_distance;

            float tar_vel_x = PID_CAM_X.Cal(-camera_x_err, 0.0f);
            if (rotate_angle >= -2.0f) {
                if (tar_vel_x >= 0) tar_vel_x = 0;
            }
            else if (rotate_angle <= -4.2f) {
                if (tar_vel_x <= 0) tar_vel_x = 0;
            }
            dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f,
                PID_CAM_Omega.Cal(tar_vel_x - rotate_omega, 0.0f));

            float tar_vel_y = PID_CAM_Y.Cal(-camera_y_err, 0.0f);
            if (extension_distance <= 0.01f) {
                if (tar_vel_y <= 0) tar_vel_y = 0;
            }
            else if (extension_distance >= 0.18f) {
                if (tar_vel_y >= 0) tar_vel_y = 0;
            }
            zdt_hori.SetVelocity(tar_vel_y, 10000, ZDTMotor::MULTI_MODE_ASYNC);

        }
    }
}
uint8_t Gimbal::Camera_Calibrated()
{
    static uint8_t continuous_count = 0;

    if (fabs(camera_x_err) <= 10 && fabs(camera_y_err) <= 10 && camera_data_enable == 1)
    {
        continuous_count++;
        if (continuous_count >= 10)
        {
            return 1;
        }
    }
    else
    {
        continuous_count = 0;
    }

    return 0;
}

void Gimbal::Lift_Move(float _height)
{
    if (_height >= 0 && _height <= 0.2f)
        tar_lift_height = _height;
}
uint8_t Gimbal::Lift_Finished()
{
    if (fabs(tar_lift_height - lift_height) <= LIFT_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Gimbal::Extension_Move(float _distance)
{
    if (_distance >= 0.0f && _distance <= 0.2f)
        tar_extension_distance = _distance;
}
uint8_t Gimbal::Extension_Finished()
{
    if (fabs(tar_extension_distance - extension_distance) <= EXTENSION_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Gimbal::Rotate_Move(float _angle)
{
    if (_angle >= -4.71f && _angle <= 1.57f)
        tar_rotate_angle = _angle;
}
uint8_t Gimbal::Rotate_Finished()
{
    if (fabs(tar_rotate_angle - rotate_angle) <= ROTATE_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}
void Gimbal::Jaw_Move(float _angle)
{
    if (_angle >= JAW_ANGLE_OPEN && _angle <= JAW_ANGLE_CLOSE) {
        vTaskDelay(5);
        tar_jaw_angle = _angle;
    }
}
uint8_t Gimbal::Jaw_Finished()
{
    vTaskDelay(5);
    if (fabs(tar_jaw_angle - jaw_angle) <= JAW_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}
