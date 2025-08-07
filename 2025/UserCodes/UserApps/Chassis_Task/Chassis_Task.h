#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "common_inc.h"
#include "DJIMotor.h"
#include "math.h"
#include "ops.h"
#include "PID.h"
#define WHEEL_DISTANCE_TO_CENTER 0.15f
#define WHEEL_RADIUS 0.038f

void Chassis_Task();

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
    typedef enum CHASSIS_CONTROL_MODE {
        CHASSIS_POSITION_CONTROL = 0,
        CHASSIS_CAMERA_CONTROL = 1,
    } chassis_control_mode_e;
    chassis_control_mode_e chassis_control_mode = CHASSIS_POSITION_CONTROL;
    void Set_ControlMode(chassis_control_mode_e _mode);

    static constexpr float NAVIGATION_CONTROL_KP = 5.0f;
    static constexpr float POSITION_CONTROL_KP = 8.0f;

    PIDController pid_angle{ 10.0f, 0.0f, 0.0f, 0.0f, 3.14f, 0.0f };
    PIDController pid_pos_x{ 8.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    PIDController pid_pos_y{ 8.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    float Tar_Pos_X, Tar_Pos_Y, Tar_Angle;
    uint8_t control_enable;

    float camera_x_err, camera_y_err;
    LowPassFilter LPF_ERR_X{ 0.005f };
    LowPassFilter LPF_ERR_Y{ 0.005f };

    PIDController pid_cam_x{ 0.001f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f };
    PIDController pid_cam_y{ 0.001f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f };

    float CAMERA_CALIBRATE_THRESHOLD = 20.0f;
    uint8_t Camera_Calibrated();

    uint32_t last_tick;
    float dt;

};
extern Chassis chassis;




#endif