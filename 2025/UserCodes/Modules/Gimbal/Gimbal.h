#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "common_inc.h"
#include "DMMotor.h"
#include "ZDTMotor.h"
#include "PID.h"
#include "LPF.h"

class Gimbal
{
public:
    void Init(CAN_HandleTypeDef* _hcan);
    void loop_control();
    PIDController PID_Angle{ 30.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f };
    PIDController PID_Distance{ 20000.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f };
    PIDController PID_Height{ 20000.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f };
    float gimbal_distance = 0;
    float gimbal_angle = 0;
    float gimbal_height = 0;
    float tar_gimbal_distance = 0;
    float tar_gimbal_angle = 0;
    float tar_gimbal_height = 0;

    float camera_x_err = 0;
    float camera_y_err = 0;
    PIDController PID_CAM_X{ 0.002f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    PIDController PID_CAM_Y{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    LowPassFilter LPF_ERR_X{ 0.01f };
    LowPassFilter LPF_ERR_Y{ 0.01f };


    enum GIMBAL_CONTROL_MODE
    {
        GIMBAL_POSITION_CONTROL = 0,
        GIMBAL_CAMERA_CONTROL = 1,
    }GIMBAL_CONTROL_MODE = GIMBAL_CAMERA_CONTROL;
};

extern Gimbal gimbal;

#endif