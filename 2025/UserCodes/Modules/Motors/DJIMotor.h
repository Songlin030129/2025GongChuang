#ifndef __DJIMOTOR_H__
#define __DJIMOTOR_H__

#include "common_inc.h"
#include <vector>
#include "PID.h"
#include "LPF.h"
typedef enum
{
    M2006,
    M3508,
} Motor_Type;
class DJIMotor
{
public:
    typedef enum {
        DIRECTION_POSITIVE = 1,
        DIRECTION_NEGATIVE = -1,
    } DIRECTION_e; // 电机方向枚举

    void setCurrent(float _current);
    void Recv_Callback(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf);
    void Velocity_Control(float _tar_vel);
    void Angle_Control(float  _tar_angle);
    CAN_HandleTypeDef* hcan;
    uint8_t motor_id;
    Motor_Type motor_type;
    DIRECTION_e dir;

    float motor_angle, motor_velocity; // rad, rad/s
    int16_t motor_current;
    uint8_t motor_temperature;
    PIDController PID_Vel{ 1.0f, 0.5f, 0.0f, 0.0f, 10.0f, 0.0f };
    PIDController PID_Angle{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    LowPassFilter LPF_Vel{ 0.01f };


    int16_t input_current;
    uint16_t raw_angle, last_raw_angle;
    int32_t total_raw_angle;
    int16_t raw_velocity;
    uint8_t init_angle_state;

};

class DJIMotorGroup
{
public:
    void init(CAN_HandleTypeDef* _hcan);
    uint8_t addMotor(DJIMotor* _motor, Motor_Type _type, uint8_t _id, DJIMotor::DIRECTION_e _dir);
    void canSendData();
    std::vector<DJIMotor*> motors;
    uint8_t data1[8];
    uint8_t data2[8];
    CAN_HandleTypeDef* hcan;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

};
extern DJIMotor dji_mot1, dji_mot2, dji_mot3, dji_mot4;
extern DJIMotorGroup dji_motor_group;

#endif