#ifndef __DMMOTOR_H__
#define __DMMOTOR_H__

#include "common_inc.h"
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define T_MIN -5.0f
#define T_MAX 5.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
typedef enum
{
    CONTROL_MODE_MIT = 0,
    CONTROL_MODE_POS_VEL = 0x100,
    CONTROL_MODE_VEL = 0x200,
} Control_Mode;
class DMMotor
{
public:
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);

    void Init(CAN_HandleTypeDef* _hcan, uint8_t _Motor_ID, uint8_t _Master_ID, Control_Mode _Mode);
    void CAN_Send_Data(uint32_t _Motor_ID, uint8_t* _Data, uint8_t _Size);
    void Recv_Callback(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf);

    void Enable();
    void Disable();
    void SaveZeroPoint();
    void ClearErr();
    void write_motor_data(uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
    void save_motor_data(uint8_t rid);

    void Control(float _pos, float _vel, float _KP, float _KD, float _torq);
    void Control(float _vel, float _pos);
    void Control(float _vel);

    float Position, Velocity, Torque;
    float Pos_des, Vel_des, Tor_des;

    uint8_t Motor_ID, Master_ID;
    Control_Mode Mode;
    uint8_t Recv_Cache[8];
    uint8_t Recv_Length;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = { 0 };
    CAN_HandleTypeDef* hcan;

};
extern DMMotor dm_gimbal;

#endif

