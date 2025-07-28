#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "common_inc.h"
#include "DJIMotor.h"
#include "math.h"
#include "ops.h"
#include "PID.h"
#include "HWT101.h"
#define WHEEL_DISTANCE_TO_CENTER 0.15f
#define WHEEL_RADIUS 0.038f
struct CarVel_s
{
    float vx;
    float vy;
    float omega;
};
struct MotorsVel_s
{
    float vel1;
    float vel2;
    float vel3;
    float vel4;
};

class Chassis
{
public:
    void Init(CAN_HandleTypeDef* _hcan);

    CarVel_s Kinematics_Forward(MotorsVel_s* _input);

    MotorsVel_s Kinematics_Inverse(CarVel_s* _input);

    void Set(float _Vx, float _Vy, float _Wz, float _Yaw);

    void Loop_Control();

    void Pos_Rst();

    float Vel_X, Vel_Y, Vel_A;
    float Tar_Vel_X, Tar_Vel_Y, Tar_Vel_A;
    float Vel_X_FF, Vel_Y_FF;
    enum CHASSIS_CONTROL_MODE {
        CHASSIS_POSITION_CONTROL = 0,
        CHASSIS_NAVIGATE_CONTROL = 1,
    } chassis_control_mode = CHASSIS_POSITION_CONTROL;

    // PIDController pid_pos_x{ 10.0, 0, 0, 0, 1.0, 0 };
    // PIDController pid_pos_y{ 10.0, 0, 0, 0, 1.0, 0 };
    PIDController pid_angle{ 10.0, 0, 0, 0, 3.14, 0 };

    PIDController pid_pos_x{ 3.0, 0, 0, 0, 1.0, 0 };
    PIDController pid_pos_y{ 3.0, 0, 0, 0, 1.0, 0 };

    float Tar_Pos_X, Tar_Pos_Y, Tar_Angle;
    uint8_t control_enable;
    uint32_t Last_Tick;

};
extern Chassis chassis;

#endif