#include "DJIMotor.h"
DJIMotor dji_mot1, dji_mot2, dji_mot3, dji_mot4;
DJIMotorGroup dji_motor_group;

void DJIMotor::setCurrent(float _current)
{
    float current = _current;
    switch (motor_type)
    {
    case M2006:
        if (current >= 10.0f) current = 10.0f;
        else if (current <= -10.0f) current = -10.0f;
        input_current = (int16_t)(current * 1000.0f);
        if (input_current >= 10000) input_current = 10000;
        else if (input_current <= -10000) input_current = -10000;
        input_current *= dir;
        break;
    case M3508:
        if (current >= 20.0f) current = 20.0f;
        else if (current <= -20.0f) current = -20.0f;
        input_current = (int16_t)(current * 819.2f);
        if (input_current >= 16384) input_current = 16384;
        else if (input_current <= -16384) input_current = -16384;
        input_current *= dir;
        break;
    default:
        break;
    }
}

void DJIMotor::Recv_Callback(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf)
{
    if (_hcan == hcan)
    {
        if (RxHeader.StdId == 0x200U + motor_id)
        {
            raw_angle = (RecvBuf[0] << 8) | RecvBuf[1];
            raw_velocity = (RecvBuf[2] << 8) | RecvBuf[3] * dir;
            motor_current = (RecvBuf[4] << 8) | RecvBuf[5] * dir;
            motor_temperature = RecvBuf[6];
            if (init_angle_state == 0) {
                init_angle_state = 1;
                last_raw_angle = raw_angle;
            }
            int temp = raw_angle - last_raw_angle;
            if (temp > 4096) {
                total_raw_angle -= 8192;
            }
            else if (temp < -4096) {
                total_raw_angle += 8192;
            }
            total_raw_angle += temp;
            total_raw_angle *= dir;
            last_raw_angle = raw_angle;
            switch (motor_type)
            {
            case M2006:
                motor_angle = total_raw_angle * 2 * _PI / 8192.0f / 36.0f;
                motor_velocity = raw_velocity * 2.0f * _PI / 60.0f / 36.0f;
                break;
            case M3508:
                motor_angle = total_raw_angle * 2 * _PI / 8192.0f / 16.0f;
                motor_velocity = raw_velocity * 2.0f * _PI / 60.0f / 16.0f;
                break;
            default:
                break;
            }
        }
    }
}

void DJIMotor::Velocity_Control(float _tar_vel)
{
    setCurrent(PID_Vel.Cal(_tar_vel - LPF_Vel(motor_velocity), 0.0f));
}

void DJIMotor::Angle_Control(float _tar_angle)
{
    float temp = PID_Angle.Cal(_tar_angle - motor_angle, 0.0f);
    setCurrent(PID_Vel.Cal(temp - motor_velocity, 0.0f));
}

void DJIMotorGroup::init(CAN_HandleTypeDef* _hcan)
{
    static uint8_t init_state = 0;
    if (init_state == 0)
    {
        CAN_FilterTypeDef can_Filter = {};

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
        vTaskDelay(1);
        init_state = 1;
    }
    hcan = _hcan;
}

uint8_t DJIMotorGroup::addMotor(DJIMotor* _motor, Motor_Type _type, uint8_t _id, DJIMotor::DIRECTION_e _dir)
{
    if (motors.size() == 8 || _id > 8)
        return 0;
    for (size_t i = 0; i < motors.size(); i++)
    {
        if (_id == motors[i]->motor_id) return 0;
    }
    motors.push_back(_motor);
    _motor->motor_id = _id;
    _motor->motor_type = _type;
    _motor->hcan = hcan;
    _motor->last_raw_angle = _motor->raw_angle;
    _motor->dir = _dir;
    switch (_type)
    {
    case M2006:
        _motor->PID_Vel.output_limit = 10.0f;
        break;
    case M3508:
        _motor->PID_Vel.output_limit = 20.0f;
        break;
    default:
        break;
    }
    return 1;
}

void DJIMotorGroup::canSendData()
{
    for (size_t i = 0; i < motors.size(); i++)
    {
        if (motors[i]->motor_id <= 4)
        {
            data1[motors[i]->motor_id * 2 - 2] = (motors[i]->input_current >> 8);
            data1[motors[i]->motor_id * 2 - 1] = motors[i]->input_current;

        }
        else
        {
            data2[(motors[i]->motor_id - 4) * 2 - 2] = (motors[i]->input_current >> 8);
            data2[(motors[i]->motor_id - 4) * 2 - 1] = motors[i]->input_current;

        }
    }
    TxHeader.StdId = 0x200;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hcan, &TxHeader, data1, &TxMailbox);
    vTaskDelay(1);
    for (size_t i = 0; i < motors.size(); i++)
    {
        if (motors[i]->motor_id > 4)
        {
            TxHeader.StdId = 0x1FF;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.DLC = 8;
            TxHeader.TransmitGlobalTime = DISABLE;

            HAL_CAN_AddTxMessage(hcan, &TxHeader, data2, &TxMailbox);
            vTaskDelay(1);
            break;
        }
    }
}

