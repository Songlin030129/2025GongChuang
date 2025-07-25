#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "common_inc.h"
#include "DMMotor.h"
#include "ZDTMotor.h"
#include "FSUS_Servo.h"
#include "PID.h"
#include "LPF.h"

class Gimbal
{
public:
    void Init(CAN_HandleTypeDef *_hcan);
    void loop_control();
    PIDController PID_Angle{0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f};
    PIDController PID_Omega{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    PIDController PID_Distance{20000.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f};
    PIDController PID_Height{20000.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f};
    float gimbal_distance     = 0;
    float gimbal_angle        = 0;
    float gimbal_omega        = 0;
    float gimbal_height       = 0;
    float tar_gimbal_distance = 0;
    float tar_gimbal_angle    = 0;
    float tar_gimbal_height   = 0;

    float camera_x_err = 0;
    float camera_y_err = 0;
    PIDController PID_CAM_X{0.002f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    PIDController PID_CAM_Y{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    LowPassFilter LPF_ERR_X{0.01f};
    LowPassFilter LPF_ERR_Y{0.01f};

    enum GIMBAL_CONTROL_MODE {
        GIMBAL_POSITION_CONTROL = 0,
        GIMBAL_CAMERA_CONTROL   = 1,
    } gimbal_control_mode = GIMBAL_CAMERA_CONTROL;

    uint8_t Camera_Calibrated();
    void Lift_Move(float _height);
    uint8_t Lift_Finished();
    void Extension_Move(float _distance);
    uint8_t Extension_Finished();
    void Rotate_Move(float _angle);
    uint8_t Rotate_Finished();
    void Jaw_Move(float _angle);
    uint8_t Jaw_Finished();

    // 旋转角度常量
    static constexpr float ROTATE_ANGLE_IN_1         = -0.0f;
    static constexpr float ROTATE_ANGLE_IN_2         = 0.0f;
    static constexpr float ROTATE_ANGLE_IN_3         = 0.0f;
    static constexpr float ROTATE_ANGLE_OUT_1        = -3.14f;
    static constexpr float ROTATE_ANGLE_OUT_2        = -3.14f;
    static constexpr float ROTATE_ANGLE_OUT_3        = -3.14f;
    static constexpr float ROTATE_FINISHED_THRESHOLD = 0.1f;

    // 夹爪角度常量
    static constexpr float JAW_ANGLE_CLOSE        = -82;
    static constexpr float JAW_ANGLE_OPEN         = -120;
    static constexpr float JAW_INTERVAL           = 200;
    static constexpr float JAW_FINISHED_THRESHOLD = 7;

    // 抬升距离常量
    static constexpr float LIFT_DISTANCE_TOP       = 0;
    static constexpr float LIFT_DISTANCE_GROUND    = 1460;
    static constexpr float LIFT_DISTANCE_ZHUANPAN  = 760;
    static constexpr float LIFT_DISTANCE_ZAIWU_PUT = 200;
    static constexpr float LIFT_DISTANCE_ZAIWU_GET = 330;
    static constexpr float LIFT_DISTANCE_SECOND    = 840;
    static constexpr float LIFT_VELOCITY           = 1500;
    static constexpr float LIFT_ACC                = 2000;
    static constexpr float LIFT_FINISHED_THRESHOLD = 5;

    // 伸缩角度常量
    static constexpr float EXTENSION_DISTANCE_1         = 0;
    static constexpr float EXTENSION_DISTANCE_2         = 0;
    static constexpr float EXTENSION_DISTANCE_3         = 0;
    static constexpr float EXTENSION_FINISHED_THRESHOLD = 5;
};

extern Gimbal gimbal;

#endif