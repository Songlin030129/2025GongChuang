#include "Chassis.h"
Chassis chassis;
void Chassis::Init(CAN_HandleTypeDef* _hcan)
{
    dji_motor_group.init(_hcan);
    dji_motor_group.addMotor(&dji_mot1, M2006, 1, DJIMotor::DIRECTION_POSITIVE);
    dji_motor_group.addMotor(&dji_mot2, M2006, 2, DJIMotor::DIRECTION_POSITIVE);
    dji_motor_group.addMotor(&dji_mot3, M2006, 3, DJIMotor::DIRECTION_POSITIVE);
    dji_motor_group.addMotor(&dji_mot4, M2006, 4, DJIMotor::DIRECTION_POSITIVE);

    chassis.Set(0.0, 0.0, 0.0, 0);

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

void Chassis::Loop_Control()
{
    //获取时间间隔Ts
    uint32_t currentTime = HAL_GetTick();
    if (Last_Tick == 0)
    {
        Last_Tick = currentTime;
        return;
    }
    float Time_Step = (currentTime - Last_Tick) / 1000.0f;
    Last_Tick = currentTime;

    static float Last_Pos_X = 0, Last_Pos_Y = 0, Last_Yaw = 0;
    Vel_X = (ops.Pos_X - Last_Pos_X) / Time_Step;
    Vel_Y = (ops.Pos_Y - Last_Pos_Y) / Time_Step;
    Vel_A = (ops.Yaw - Last_Yaw) / Time_Step;
    Last_Pos_X = ops.Pos_X;
    Last_Pos_Y = ops.Pos_Y;
    Last_Yaw = ops.Yaw;

    Tar_Vel_X = pid_pos_x.Cal(Tar_Pos_X - ops.Pos_X, Vel_X_FF);
    Tar_Vel_Y = pid_pos_y.Cal(Tar_Pos_Y - ops.Pos_Y, Vel_Y_FF);
    Tar_Vel_A = pid_angle.Cal(Tar_Angle - ops.Yaw, 0);

    chassis.Set(Tar_Vel_X, Tar_Vel_Y, Tar_Vel_A, ops.Yaw);
}