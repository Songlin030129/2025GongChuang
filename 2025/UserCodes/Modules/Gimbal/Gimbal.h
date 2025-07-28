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
    float last_tick, ts;
    void Init(CAN_HandleTypeDef* _hcan);
    void loop_control();
    uint8_t control_enable;
    uint8_t flag_out_of_range;
    //运行状态量
    float extension_distance = 0;
    float rotate_angle = 0;
    float last_rotate_angle = 0;
    float rotate_omega = 0;
    float lift_height = 0;
    float jaw_angle = 0;
    LowPassFilter LPF_OMEGA{ 0.01f };

    //直线导轨升降闭环
    float tar_lift_height = 0;
    PIDController PID_Height{ 20000.0f, 0.0f, 0.0f, 0.0f, 500.0f, 0.0f };

    //舵机夹爪闭环
    float tar_jaw_angle = 0;

    //云台角度闭环
    float tar_extension_distance = 0;
    float tar_rotate_angle = 0;
    PIDController PID_Angle{ 25.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f };
    PIDController PID_Omega{ 0.9f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f };
    PIDController PID_Distance{ 20000.0f, 0.0f, 0.0f, 0.0f, 500.0f, 0.0f };

    //云台摄像头定位闭环
    uint8_t camera_data_enable = 0;
    uint8_t camera_detect_color = 0;
    float camera_raw_x_err = 0;
    float camera_raw_y_err = 0;
    float camera_x_err = 0;
    float camera_y_err = 0;
    LowPassFilter LPF_ERR_X{ 0.025f };
    LowPassFilter LPF_ERR_Y{ 0.025f };
    PIDController PID_CAM_X{ 0.002f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    PIDController PID_CAM_Y{ 0.9f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    PIDController PID_CAM_Omega{ 1.5f, 6.0f, 0.0f, 0.0f, 3.0f, 0.0f };

    enum GIMBAL_CONTROL_MODE {
        GIMBAL_POSITION_CONTROL = 0,
        GIMBAL_CAMERA_CONTROL = 1,
    } gimbal_control_mode = GIMBAL_POSITION_CONTROL;

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
    static constexpr float ROTATE_ANGLE_IN_1 = -0.51f;
    static constexpr float ROTATE_ANGLE_IN_2 = 0.0f;
    static constexpr float ROTATE_ANGLE_IN_3 = 0.51f;
    static constexpr float ROTATE_ANGLE_OUT_1 = -3.90f;
    static constexpr float ROTATE_ANGLE_OUT_2 = -3.14f;
    static constexpr float ROTATE_ANGLE_OUT_3 = -2.52f;
    static constexpr float ROTATE_FINISHED_THRESHOLD = 0.05f;

    // 夹爪角度常量
    static constexpr float JAW_ANGLE_CLOSE = 71.0f;
    static constexpr float JAW_ANGLE_OPEN = 0.0f;
    static constexpr float JAW_INTERVAL = 200.0f;
    static constexpr float JAW_FINISHED_THRESHOLD = 5.0f;

    // 抬升距离常量
    static constexpr float LIFT_DISTANCE_TOP = 0.0f;
    static constexpr float LIFT_DISTANCE_GROUND = 0.173f;
    static constexpr float LIFT_DISTANCE_ZHUANPAN = 0.095f;
    static constexpr float LIFT_DISTANCE_ZAIWU_PUT = 0.080f;
    static constexpr float LIFT_DISTANCE_ZAIWU_GET = 0.088f;
    static constexpr float LIFT_DISTANCE_SECOND = 0.104f;
    static constexpr float LIFT_FINISHED_THRESHOLD = 0.005f;

    // 伸缩角度常量
    static constexpr float EXTENSION_DISTANCE_IN_1 = 0.045f;
    static constexpr float EXTENSION_DISTANCE_IN_2 = 0.020f;
    static constexpr float EXTENSION_DISTANCE_IN_3 = 0.045f;
    static constexpr float EXTENSION_DISTANCE_OUT_1 = 0.10f;
    static constexpr float EXTENSION_DISTANCE_OUT_2 = 0.03f;
    static constexpr float EXTENSION_DISTANCE_OUT_3 = 0.10f;
    static constexpr float EXTENSION_FINISHED_THRESHOLD = 0.005f;
};

extern Gimbal gimbal;

#endif