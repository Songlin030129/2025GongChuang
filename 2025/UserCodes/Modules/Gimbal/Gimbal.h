#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "common_inc.h"
#include "DMMotor.h"
#include "ZDTMotor.h"
#include "FSUS_Servo.h"
#include "PID.h"
#include "LPF.h"
#include "trapTraj.h"
class Gimbal
{
public:
    float last_tick, dt;
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
    LowPassFilter LPF_OMEGA{ 0.00f };

    //舵机夹爪闭环
    float tar_jaw_angle = 0;

    //直线导轨升降闭环
    float tar_lift_height = 0, lift_ff = 0, trap_lift_tar = 0;
    PIDController PID_Height{ 10000.0f, 0.0f, 0.0f, 0.0f, 600.0f, 0.0f };
    void Lift_Move(float _distance);
    uint8_t Lift_Finished();
    uint8_t lift_move_state = 0;
    uint32_t lift_move_start_tick = 0;
    TrapezoidalTrajectory trap_lift;
    static constexpr float LIFT_MOVE_VEL = 0.7f;
    static constexpr float LIFT_MOVE_ACC = 2.0f;

    //水平导轨伸缩闭环
    float tar_extension_distance = 0, extension_ff = 0, trap_extension_tar = 0;
    PIDController PID_Distance{ 10000.0f, 0.0f, 0.0f, 0.0f, 600.0f, 0.0f };
    void Extension_Move(float _distance);
    uint8_t Extension_Finished();
    uint8_t extension_move_state = 0;
    uint32_t extension_move_start_tick = 0;
    TrapezoidalTrajectory trap_extension;
    static constexpr float EXTENSION_MOVE_VEL = 0.5f;
    static constexpr float EXTENSION_MOVE_ACC = 1.5f;

    //云台角度闭环
    float tar_rotate_angle = 0, rotate_ff = 0, trap_rotate_tar = 0;
    PIDController PID_Angle{ 10.0f, 0.0f, 0.0f, 0.0f, 4.71f, 0.0f };
    PIDController PID_Omega{ 0.6f, 10.0f, 0.0f, 0.0f, 2.5f, 0.0f };
    void Rotate_Move(float _angle);
    uint8_t Rotate_Finished();
    uint8_t rotate_move_state = 0;
    uint32_t rotate_move_start_tick = 0;
    TrapezoidalTrajectory trap_rotate;
    static constexpr float ROTATE_MOVE_VEL = 4.71f;
    static constexpr float ROTATE_MOVE_ACC = 10.0f;

    uint8_t All_Move_Finished();

    //云台摄像头定位闭环
    uint8_t camera_data_valid = 0;
    uint8_t camera_detect_color = 0;
    float camera_raw_x_err = 0;
    float camera_raw_y_err = 0;
    float camera_x_err = 0, camera_angle_err = 0;
    float camera_y_err = 0;
    LowPassFilter LPF_ERR_X{ 0.015f };
    LowPassFilter LPF_ERR_Y{ 0.015f };
    PIDController PID_CAM_X{ 0.0008f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    PIDController PID_CAM_Omega{ 0.7f, 11.0f, 0.0f, 0.0f, 2.5f, 0.0f };
    PIDController PID_CAM_Y{ 0.9f, 0.0f, 0.0f, 0.0f, 250.0f, 0.0f };

    typedef enum GIMBAL_CONTROL_MODE {
        GIMBAL_POSITION_CONTROL = 0,
        GIMBAL_CAMERA_CONTROL = 1,
    } gimbal_control_mode_e;

    gimbal_control_mode_e gimbal_control_mode = GIMBAL_POSITION_CONTROL;
    void Set_ControlMode(gimbal_control_mode_e _mode);

    uint8_t Camera_Calibrated();

    void Jaw_Move(float _angle);
    uint8_t Jaw_Finished();

    // 旋转角度常量
    float ROTATE_ANGLE_IN_1 = -0.49f;
    float ROTATE_ANGLE_IN_2 = 0.0f;
    float ROTATE_ANGLE_IN_3 = 0.51f;
    float ROTATE_ANGLE_OUT_1 = -3.80f;
    float ROTATE_ANGLE_OUT_2 = -3.14f;
    float ROTATE_ANGLE_OUT_3 = -2.50f;
    float ROTATE_FINISHED_THRESHOLD = 0.02f;

    // 夹爪角度常量
    float JAW_ANGLE_CLOSE = 73.0f;
    float JAW_ANGLE_OPEN = 0.0f;
    float JAW_INTERVAL = 200.0f;
    float JAW_FINISHED_THRESHOLD = 6.0f;

    // 抬升距离常量
    float LIFT_DISTANCE_TOP = 0.0f;
    float LIFT_DISTANCE_GROUND = 0.173f;
    float LIFT_DISTANCE_ZHUANPAN = 0.095f;
    float LIFT_DISTANCE_ZAIWU_PUT = 0.080f;
    float LIFT_DISTANCE_ZAIWU_GET = 0.088f;
    float LIFT_DISTANCE_SECOND = 0.10f;
    float LIFT_FINISHED_THRESHOLD = 0.002f;

    // 伸缩角度常量
    float EXTENSION_DISTANCE_IN_1 = 0.045f;
    float EXTENSION_DISTANCE_IN_2 = 0.020f;
    float EXTENSION_DISTANCE_IN_3 = 0.045f;
    float EXTENSION_DISTANCE_OUT_1 = 0.12f;
    float EXTENSION_DISTANCE_OUT_2 = 0.10f;
    float EXTENSION_DISTANCE_OUT_3 = 0.12f;
    float EXTENSION_DISTANCE_OUT_MATERIAL = 0.07f;
    float EXTENSION_FINISHED_THRESHOLD = 0.002f;

};

extern Gimbal gimbal;

#endif