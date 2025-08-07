#include "Gimbal_Task.h"
#include "LED.h"
#include "HMI.h"
Gimbal gimbal;

void Gimbal_Task()
{
    vTaskDelay(500);
    gimbal.Init(&hcan2);
    while (1)
    {
        gimbal.loop_control();
        vTaskDelay(5);
    }
}

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

    tar_rotate_angle = ROTATE_ANGLE_IN_2;
    tar_extension_distance = EXTENSION_DISTANCE_IN_2;
    tar_lift_height = LIFT_DISTANCE_TOP;

    servo_protocol.Init(&huart4);
    servo_jaw.init(3, &servo_protocol);
    if (servo_jaw.ping() != 1) printf("servo_jaw error\r\n");

    tar_jaw_angle = JAW_ANGLE_OPEN;
    control_enable = 1;
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
    dt = (current_tick - last_tick) * 0.001f;
    last_tick = current_tick;

    zdt_hori.GetPosition();
    zdt_vert.GetPosition();
    tar_rotate_angle = _constrain(tar_rotate_angle, -4.71f, 1.57f);
    tar_extension_distance = _constrain(tar_extension_distance, 0.0f, 0.23f);
    tar_lift_height = _constrain(tar_lift_height, 0.0f, 0.2f);

    rotate_angle = dm_gimbal.Position;
    rotate_omega = ((rotate_angle - last_rotate_angle) / dt);
    last_rotate_angle = rotate_angle;
    extension_distance = zdt_hori.Position / 9000.0f;
    lift_height = zdt_vert.Position / 9000.0f;
    jaw_angle = servo_jaw.queryAngle();

    float raw_camera_x_err = LPF_ERR_X(hmi.camera_raw_x_err);
    float raw_camera_y_err = LPF_ERR_Y(hmi.camera_raw_y_err);

    // 对相机误差应用slope_following，添加缓慢跟随特性
    slope_following(&raw_camera_x_err, &camera_x_err, CAMERA_ERR_SLOPE_RATE);
    slope_following(&raw_camera_y_err, &camera_y_err, CAMERA_ERR_SLOPE_RATE);

    camera_angle_err = camera_x_err / (0.20f + extension_distance);
    // 执行升降轨迹
    if (lift_move_state == 1) {
        PID_Height.P = 10000.0f;
        float elapsed_time = (HAL_GetTick() - lift_move_start_tick) / 1000.0f;
        trap_lift.t_ = elapsed_time;
        TrapezoidalTrajectory::Step_t step = trap_lift.eval(trap_lift.t_);

        if (trap_lift.t_ <= trap_lift.Tf_) {
            tar_lift_height = step.Y;
            lift_ff = step.Yd;
        }
        else {
            PID_Height.P = 20000.0f;
            lift_move_state = 0;
        }
    }

    if (!control_enable) {
        dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        vTaskDelay(1);
        zdt_hori.Disable();
        vTaskDelay(1);
        zdt_vert.Disable();
        vTaskDelay(1);
        servo_jaw.setTorque(0);
        return;
    }

    servo_jaw.setAngle(tar_jaw_angle, JAW_INTERVAL);

    // 垂直电机位置闭环
    zdt_vert.SetVelocity(PID_Height.Cal(tar_lift_height - lift_height, lift_ff * 1500),
        10000, ZDTMotor::MULTI_MODE_ASYNC);

    if (gimbal_control_mode == GIMBAL_POSITION_CONTROL) {
        // 云台位置控制
        // 执行旋转轨迹
        if (rotate_move_state == 1) {
            PID_Angle.P = 5.0f;
            float elapsed_time = (HAL_GetTick() - rotate_move_start_tick) / 1000.0f;
            trap_rotate.t_ = elapsed_time;
            TrapezoidalTrajectory::Step_t step = trap_rotate.eval(trap_rotate.t_);

            if (trap_rotate.t_ <= trap_rotate.Tf_) {
                tar_rotate_angle = step.Y;
                rotate_ff = step.Yd;
            }
            else {
                rotate_move_state = 0;
                PID_Angle.P = 10.0f;
            }
        }
        // 执行伸缩轨迹
        if (extension_move_state == 1) {
            PID_Distance.P = 10000.0f;
            float elapsed_time = (HAL_GetTick() - extension_move_start_tick) / 1000.0f;
            trap_extension.t_ = elapsed_time;
            TrapezoidalTrajectory::Step_t step = trap_extension.eval(trap_extension.t_);

            if (trap_extension.t_ <= trap_extension.Tf_) {
                tar_extension_distance = step.Y;
                extension_ff = step.Yd;
            }
            else {
                PID_Distance.P = 20000.0f;
                extension_move_state = 0;
            }
        }

        // 水平伸缩电机位置闭环
        zdt_hori.SetVelocity(PID_Distance.Cal(tar_extension_distance - extension_distance, extension_ff * 1500),
            10000, ZDTMotor::MULTI_MODE_ASYNC);

        // 云台角度闭环
        dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f,
            PID_Omega.Cal(PID_Angle.Cal(tar_rotate_angle - rotate_angle, rotate_ff) - rotate_omega, 0.0f));
    }
    else if (gimbal_control_mode == GIMBAL_CAMERA_CONTROL) {
        // 云台锁定摄像头位置闭环
        if (hmi.camera_data_valid) {
            tar_rotate_angle = rotate_angle;
            tar_extension_distance = extension_distance;

            float tar_vel_x = PID_CAM_X.Cal(-camera_angle_err, 0.0f);
            float tar_torque = PID_CAM_Omega.Cal(tar_vel_x - rotate_omega, 0.0f);
            if (rotate_angle >= -2.0f) {
                if (tar_torque >= 0) tar_torque = 0;
            }
            else if (rotate_angle <= -4.2f) {
                if (tar_torque <= 0) tar_torque = 0;
            }
            dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f, tar_torque);
            tar_rotate_angle = rotate_angle;
            tar_extension_distance = extension_distance;

            float tar_vel_y = PID_CAM_Y.Cal(-camera_y_err, 0.0f);
            if (extension_distance <= 0.0f) {
                if (tar_vel_y <= 0) {
                    tar_vel_y = 0;
                    flag_out_of_range = 1;
                }
                else {
                    flag_out_of_range = 0;
                }
            }
            else if (extension_distance >= 0.22f) {
                if (tar_vel_y >= 0) {
                    tar_vel_y = 0;
                    flag_out_of_range = 1;
                }
                else {
                    flag_out_of_range = 0;
                }
            }
            else {
                flag_out_of_range = 0;
            }
            zdt_hori.SetVelocity(tar_vel_y, 10000, ZDTMotor::MULTI_MODE_ASYNC);

        }
        else {
            dm_gimbal.Control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            zdt_hori.SetVelocity(0, 10000, ZDTMotor::MULTI_MODE_ASYNC);
        }
    }
    static float cnt = 0.0f;
    cnt += dt;
    if (cnt >= 1.0f) {
        cnt = 0;
        led2.Toggle();
    }
}

void Gimbal::Set_ControlMode(gimbal_control_mode_e _mode)
{
    if (_mode == GIMBAL_POSITION_CONTROL && gimbal_control_mode != GIMBAL_POSITION_CONTROL) {
        gimbal_control_mode = GIMBAL_POSITION_CONTROL;
        tar_extension_distance = extension_distance;
        tar_rotate_angle = rotate_angle;
        rotate_ff = 0.0f;
        extension_ff = 0.0f;
        PID_Angle.reset();
        PID_Omega.reset();
        PID_Distance.reset();
        trap_extension.reset();
        trap_rotate.reset();
        hmi.camera_data_valid = 0;
        hmi.valid_data_count = 0;
        rotate_move_state = 0;
        extension_move_state = 0;
        PID_Angle.Enable = 1;
        PID_Omega.Enable = 1;
        PID_Distance.Enable = 1;
        PID_CAM_Omega.Enable = 0;
        PID_CAM_X.Enable = 0;
        PID_CAM_Y.Enable = 0;
        tar_extension_distance = extension_distance;
        tar_rotate_angle = rotate_angle;
    }
    else if (_mode == GIMBAL_CAMERA_CONTROL && gimbal_control_mode != GIMBAL_CAMERA_CONTROL) {
        gimbal_control_mode = GIMBAL_CAMERA_CONTROL;
        camera_x_err = 0;
        camera_y_err = 0;
        LPF_ERR_X.reset();
        LPF_ERR_Y.reset();
        hmi.camera_data_valid = 0;
        hmi.valid_data_count = 0;
        PID_CAM_Omega.reset();
        PID_CAM_X.reset();
        PID_CAM_Y.reset();
        PID_Angle.Enable = 0;
        PID_Omega.Enable = 0;
        PID_Distance.Enable = 0;
        PID_CAM_Omega.Enable = 1;
        PID_CAM_X.Enable = 1;
        PID_CAM_Y.Enable = 1;
    }
}

uint8_t Gimbal::Camera_Calibrated()
{
    static uint8_t continuous_count = 0;

    if (_ABS(camera_x_err) <= CAMERA_CALIBRATE_X_THRESHOLD && _ABS(camera_y_err) <= CAMERA_CALIBRATE_Y_THRESHOLD
        && hmi.camera_data_valid == 1)
    {
        continuous_count++;
        if (continuous_count >= 10)
        {
            continuous_count = 0;
            return 1;
        }
    }
    else
    {
        continuous_count = 0;
    }
    return 0;
}

void Gimbal::Jaw_Move(float _angle)
{
    if (_angle >= JAW_ANGLE_OPEN && _angle <= JAW_ANGLE_CLOSE) {
        tar_jaw_angle = _angle;
    }

}

uint8_t Gimbal::Jaw_Finished()
{
    if (_ABS(tar_jaw_angle - jaw_angle) <= JAW_FINISHED_THRESHOLD)
        return 1;
    else
        return 0;
}

void Gimbal::Lift_Move(float _distance)
{
    trap_lift_tar = _distance;
    if (lift_move_state == 0) {
        trap_lift.reset();
        lift_move_start_tick = HAL_GetTick();

        float Xi = lift_height;
        float Vi = 0.0f;
        float Xf = _distance;
        float Vmax = LIFT_MOVE_VEL;
        float Amax = LIFT_MOVE_ACC;
        float Dmax = LIFT_MOVE_ACC;

        bool success = trap_lift.planTrapezoidal(Xf, Xi, Vi, Vmax, Amax, Dmax);
        if (success) {
            lift_move_state = 1;
        }
        else {
            printf("Failed to plan\r\n");
        }
    }
    else {
        printf("trap state err\r\n");
    }
}
uint8_t Gimbal::Lift_Finished()
{
    if (lift_move_state == 0)
        return 1;
    else
        return 0;
}

void Gimbal::Rotate_Move(float _angle)
{
    trap_rotate_tar = _angle;
    if (rotate_move_state == 0) {
        trap_rotate.reset();
        rotate_move_start_tick = HAL_GetTick();

        float Xi = rotate_angle;
        float Vi = 0.0f;
        float Xf = _angle;
        float Vmax = ROTATE_MOVE_VEL;
        float Amax = ROTATE_MOVE_ACC;
        float Dmax = ROTATE_MOVE_ACC;

        bool success = trap_rotate.planTrapezoidal(Xf, Xi, Vi, Vmax, Amax, Dmax);
        if (success) {
            rotate_move_state = 1;
        }
        else {
            printf("Failed to plan\r\n");
        }
    }
    else {
        printf("trap state err\r\n");
    }
}
uint8_t Gimbal::Rotate_Finished()
{
    if (rotate_move_state == 0)
        return 1;
    else
        return 0;
}

void Gimbal::Extension_Move(float _distance)
{
    trap_extension_tar = _distance;
    if (extension_move_state == 0) {
        trap_extension.reset();
        extension_move_start_tick = HAL_GetTick();

        float Xi = extension_distance;
        float Vi = 0.0f;
        float Xf = _distance;
        float Vmax = EXTENSION_MOVE_VEL;
        float Amax = EXTENSION_MOVE_ACC;
        float Dmax = EXTENSION_MOVE_ACC;

        bool success = trap_extension.planTrapezoidal(Xf, Xi, Vi, Vmax, Amax, Dmax);
        if (success) {
            extension_move_state = 1;
        }
        else {
            printf("Failed to plan\r\n");
        }
    }
    else {
        printf("trap state err\r\n");
    }
}
uint8_t Gimbal::Extension_Finished()
{
    if (extension_move_state == 0)
        return 1;
    else
        return 0;
}

uint8_t Gimbal::All_Move_Finished()
{
    if (Jaw_Finished() && Extension_Finished() && Rotate_Finished() && Lift_Finished())
        return 1;
    else
        return 0;
}

void Gimbal::slope_following(float* target, float* set, float acc)
{
    if (*target > *set)
    {
        *set = *set + acc;
        if (*set >= *target)
            *set = *target;
    }
    else if (*target < *set)
    {
        *set = *set - acc;
        if (*set <= *target)
            *set = *target;
    }

}