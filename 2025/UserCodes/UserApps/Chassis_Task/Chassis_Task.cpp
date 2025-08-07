#include "Chassis_Task.h"
#include "LED.h"
#include "HMI.h"
Chassis chassis;

void Chassis_Task()
{
    chassis.Init(&hcan1);
    while (1) {

        chassis.Loop_Control();
        vTaskDelay(10);
    }
}

void Chassis::Init(CAN_HandleTypeDef* _hcan)
{
    dji_motor_group.init(_hcan);
    dji_motor_group.addMotor(&dji_mot1, M2006, 1, DJIMotor::DIRECTION_POSITIVE);
    dji_motor_group.addMotor(&dji_mot2, M2006, 2, DJIMotor::DIRECTION_POSITIVE);
    dji_motor_group.addMotor(&dji_mot3, M2006, 3, DJIMotor::DIRECTION_POSITIVE);
    dji_motor_group.addMotor(&dji_mot4, M2006, 4, DJIMotor::DIRECTION_POSITIVE);

    chassis.Set(0.0, 0.0, 0.0, 0);
    control_enable = 1;
    ops.Init(&huart6);

}
CarVel_s Chassis::Kinematics_Forward(MotorsVel_s* _input)
{
    static CarVel_s output = { 0, 0, 0 };
    static double temp = 0.707106781 * 2;
    output.vx = (_input->vel1 - _input->vel2) / temp;
    output.vy = (_input->vel2 - _input->vel3) / temp;
    output.omega = (_input->vel1 + _input->vel3) / 2 / WHEEL_DISTANCE_TO_CENTER;
    return output;
}
MotorsVel_s Chassis::Kinematics_Inverse(CarVel_s* _input)
{
    MotorsVel_s output;
    static double cos_45 = 0.707106781, sin_45 = 0.707106781;
    output.vel1 = (+_input->vx * cos_45 + _input->vy * sin_45 + _input->omega * WHEEL_DISTANCE_TO_CENTER) / WHEEL_RADIUS;
    output.vel2 = (-_input->vx * sin_45 + _input->vy * cos_45 + _input->omega * WHEEL_DISTANCE_TO_CENTER) / WHEEL_RADIUS;
    output.vel3 = (-_input->vx * sin_45 - _input->vy * cos_45 + _input->omega * WHEEL_DISTANCE_TO_CENTER) / WHEEL_RADIUS;
    output.vel4 = (+_input->vx * cos_45 - _input->vy * sin_45 + _input->omega * WHEEL_DISTANCE_TO_CENTER) / WHEEL_RADIUS;
    return output;
}
void Chassis::Set(float _Vx, float _Vy, float _Wz, float _Yaw)
{
    float _sinTheta = sin(-_Yaw);
    float _cosTheta = cos(-_Yaw);
    float Vtx = _Vx * _cosTheta - _Vy * _sinTheta;
    float Vty = _Vx * _sinTheta + _Vy * _cosTheta;

    static CarVel_s Car_Vel = { 0, 0, 0 };
    Car_Vel.vx = Vtx;
    Car_Vel.vy = Vty;
    Car_Vel.omega = _Wz;

    MotorsVel_s Motor_Vel = Kinematics_Inverse(&Car_Vel);

    dji_mot1.Velocity_Control(Motor_Vel.vel1);
    dji_mot2.Velocity_Control(Motor_Vel.vel2);
    dji_mot3.Velocity_Control(Motor_Vel.vel3);
    dji_mot4.Velocity_Control(Motor_Vel.vel4);
    dji_motor_group.canSendData();

}

void Chassis::Pos_Rst()
{
    chassis.Set(0.0, 0.0, 0.0, 0);
    ops.Data_Reset();
    Tar_Angle = 0;
    Tar_Pos_X = 0;
    Tar_Pos_Y = 0;

}
uint8_t Chassis::Camera_Calibrated()
{
    static uint8_t continuous_count = 0;

    if (_ABS(camera_x_err) <= CAMERA_CALIBRATE_THRESHOLD && _ABS(camera_y_err) <= CAMERA_CALIBRATE_THRESHOLD
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

void Chassis::Set_ControlMode(chassis_control_mode_e _mode)
{
    if (_mode == CHASSIS_POSITION_CONTROL && chassis_control_mode != CHASSIS_POSITION_CONTROL) {
        chassis_control_mode = CHASSIS_POSITION_CONTROL;
        Tar_Pos_X = ops.Pos_X;
        Tar_Pos_Y = ops.Pos_Y;
        pid_pos_x.reset();
        pid_pos_y.reset();
        pid_pos_x.Enable = 1;
        pid_pos_y.Enable = 1;
        pid_cam_x.Enable = 0;
        pid_cam_y.Enable = 0;
    }
    else if (_mode == CHASSIS_CAMERA_CONTROL && chassis_control_mode != CHASSIS_CAMERA_CONTROL) {
        chassis_control_mode = CHASSIS_CAMERA_CONTROL;
        camera_x_err = 0;
        camera_y_err = 0;
        LPF_ERR_X.reset();
        LPF_ERR_Y.reset();
        hmi.camera_data_valid = 0;
        hmi.valid_data_count = 0;
        pid_cam_x.reset();
        pid_cam_y.reset();
        pid_pos_x.Enable = 0;
        pid_pos_y.Enable = 0;
        pid_cam_x.Enable = 1;
        pid_cam_y.Enable = 1;
    }

}

void Chassis::Loop_Control()
{
    //获取时间间隔Ts
    uint32_t current_tick = HAL_GetTick();
    if (last_tick == 0)
    {
        last_tick = current_tick;
        return;
    }
    dt = (current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    static float Last_Pos_X = 0, Last_Pos_Y = 0, Last_Yaw = 0;
    Vel_X = (ops.Pos_X - Last_Pos_X) / dt;
    Vel_Y = (ops.Pos_Y - Last_Pos_Y) / dt;
    Vel_A = (ops.Yaw - Last_Yaw) / dt;
    Last_Pos_X = ops.Pos_X;
    Last_Pos_Y = ops.Pos_Y;
    Last_Yaw = ops.Yaw;
    camera_x_err = LPF_ERR_X(hmi.camera_raw_x_err);
    camera_y_err = LPF_ERR_Y(hmi.camera_raw_y_err);

    if (!control_enable) {
        dji_mot1.setCurrent(0.0f);
        dji_mot2.setCurrent(0.0f);
        dji_mot3.setCurrent(0.0f);
        dji_mot4.setCurrent(0.0f);
        dji_motor_group.canSendData();
        return;
    }

    if (chassis_control_mode == CHASSIS_POSITION_CONTROL) {
        Tar_Vel_X = pid_pos_x.Cal(Tar_Pos_X - ops.Pos_X, Vel_X_FF);
        Tar_Vel_Y = pid_pos_y.Cal(Tar_Pos_Y - ops.Pos_Y, Vel_Y_FF);
        Tar_Vel_A = pid_angle.Cal(Tar_Angle - ops.Yaw, 0);
        chassis.Set(Tar_Vel_X, Tar_Vel_Y, Tar_Vel_A, ops.Yaw);
    }
    else if (chassis_control_mode == CHASSIS_CAMERA_CONTROL) {
        if (hmi.camera_data_valid) {
            Tar_Vel_X = pid_cam_x.Cal(-camera_x_err, 0.0f);
            Tar_Vel_Y = pid_cam_y.Cal(-camera_y_err, 0.0f);
            Tar_Pos_X = ops.Pos_X;
            Tar_Pos_Y = ops.Pos_Y;
        }
        else {
            Tar_Vel_X = 0.0f;
            Tar_Vel_Y = 0.0f;
        }
        Tar_Vel_A = pid_angle.Cal(Tar_Angle - ops.Yaw, 0);
        chassis.Set(Tar_Vel_X, Tar_Vel_Y, Tar_Vel_A, 0.0f);
    }

    static float cnt = 0.0f;
    cnt += dt;
    if (cnt >= 1) {
        cnt = 0.0f;
        led1.Toggle();
    }
}