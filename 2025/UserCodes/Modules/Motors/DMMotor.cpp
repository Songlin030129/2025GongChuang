#include "DMMotor.h"
#include "string.h"
DMMotor dm_gimbal;
void DMMotor::Init(CAN_HandleTypeDef* _hcan, uint8_t _Motor_ID, uint8_t _Master_ID, Control_Mode _Mode)
{
    static uint8_t init_state = 0;
    if (init_state == 0)
    {
        CAN_FilterTypeDef can_Filter = { 0 };

        can_Filter.FilterIdHigh = 0;
        can_Filter.FilterIdLow = 0;
        can_Filter.FilterMaskIdHigh = 0;
        can_Filter.FilterMaskIdLow = 0;
        can_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        can_Filter.FilterBank = 0;
        can_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
        can_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
        can_Filter.FilterActivation = CAN_FILTER_ENABLE;
        HAL_CAN_ConfigFilter(_hcan, &can_Filter);
        HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

        HAL_CAN_Start(_hcan);

        init_state = 1;
    }

    hcan = _hcan;
    Motor_ID = _Motor_ID;
    Master_ID = _Master_ID;
    Mode = _Mode;
}

void DMMotor::CAN_Send_Data(uint32_t _Motor_ID, uint8_t* _Data, uint8_t _Size)
{
    memcpy(TxData, _Data, _Size);
    TxHeader.StdId = _Motor_ID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = _Size;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    vTaskDelay(1);
}

void DMMotor::Recv_Callback(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf)
{
    if (_hcan == hcan)
    {
        if (RxHeader.StdId == Master_ID)
        {
            int p_int = (RecvBuf[1] << 8) | RecvBuf[2];
            int v_int = (RecvBuf[3] << 4) | (RecvBuf[4] >> 4);
            int t_int = ((RecvBuf[4] & 0xF) << 8) | RecvBuf[5];
            Position = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
            Velocity = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
            Torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0) 
        }
    }
}

void DMMotor::Enable()
{
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFC;
    CAN_Send_Data(Motor_ID + Mode, Data, 8);

}

void DMMotor::Disable()
{
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFD;
    CAN_Send_Data(Motor_ID + Mode, Data, 8);

}
void DMMotor::write_motor_data(uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint8_t can_id_l = Motor_ID & 0x0F;
    uint8_t can_id_h = (Motor_ID >> 4) & 0x0F;

    uint8_t data[8] = { can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3 };
    CAN_Send_Data(0x7FF, data, 8);

}

void DMMotor::save_motor_data(uint8_t rid)
{
    uint8_t can_id_l = Motor_ID & 0xFF;       // 低 8 位
    uint8_t can_id_h = (Motor_ID >> 8) & 0x07; // 高 3 位

    uint8_t data[4] = { can_id_l, can_id_h, 0xAA, 0x01 };
    CAN_Send_Data(0x7FF, data, 4);

}
void DMMotor::SaveZeroPoint()
{
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFE;
    CAN_Send_Data(Motor_ID + Mode, Data, 8);

}

void DMMotor::ClearErr()
{
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFB;
    CAN_Send_Data(Motor_ID + Mode, Data, 8);

}


void DMMotor::Control(float _pos, float _vel, float _KP, float _KD, float _torq)
{
    if (Mode == CONTROL_MODE_MIT)
    {
        Pos_des = _pos;
        Vel_des = _vel;
        Tor_des = _torq;
        uint8_t Data[8];
        uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
        pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
        vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
        kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
        kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
        tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

        Data[0] = (pos_tmp >> 8);
        Data[1] = pos_tmp;
        Data[2] = (vel_tmp >> 4);
        Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        Data[4] = kp_tmp;
        Data[5] = (kd_tmp >> 4);
        Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        Data[7] = tor_tmp;
        CAN_Send_Data(Motor_ID + Mode, Data, 8);

    }
}

void DMMotor::Control(float _vel, float _pos)
{
    if (Mode == CONTROL_MODE_POS_VEL)
    {
        Pos_des = _pos;
        Vel_des = _vel;
        uint8_t* pbuf, * vbuf;
        pbuf = (uint8_t*)&_pos;
        vbuf = (uint8_t*)&_vel;
        uint8_t Data[8];
        Data[0] = *pbuf;
        Data[1] = *(pbuf + 1);
        Data[2] = *(pbuf + 2);
        Data[3] = *(pbuf + 3);
        Data[4] = *vbuf;
        Data[5] = *(vbuf + 1);
        Data[6] = *(vbuf + 2);
        Data[7] = *(vbuf + 3);
        CAN_Send_Data(Motor_ID + Mode, Data, 8);
    }
}

void DMMotor::Control(float _vel)
{
    if (Mode == CONTROL_MODE_VEL)
    {
        Vel_des = _vel;
        uint8_t Data[4];
        uint8_t* vbuf;
        vbuf = (uint8_t*)&_vel;
        Data[0] = *vbuf;
        Data[1] = *(vbuf + 1);
        Data[2] = *(vbuf + 2);
        Data[3] = *(vbuf + 3);
        CAN_Send_Data(Motor_ID + Mode, Data, 4);
    }
}

float DMMotor::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
int DMMotor::float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits

    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
