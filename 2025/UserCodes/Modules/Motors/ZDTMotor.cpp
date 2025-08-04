#include "ZDTMotor.h"
ZDTMotor zdt_hori, zdt_vert;
/**
 * @brief CAN总线发送数据
 * @param _Motor_ID 电机ID
 * @param Data 要发送的数据
 * @param Byte_Count 数据长度
 * @retval void
 *
 */
void ZDTMotor::CAN_Send_Data(uint8_t _Motor_ID, uint8_t* _Data, uint8_t _Size)
{
    uint8_t segments = (_Size + 7) / 8; // 计算需要发送的段数
    uint8_t segment = 0;
    uint8_t Command = _Data[0];
    _Size = _Size + segments - 1;
    for (segment = 0; segment < segments; segment++) {
        // 清空TxData
        for (uint8_t i = 0; i < 8; i++) {
            TxData[i] = 0;
        }
        // 填充TxData
        TxData[0] = Command;
        for (uint8_t i = 0; i < zdt_min(7, _Size - segment * 8); i++) {
            TxData[i + 1] = _Data[i - segment + 1 + segment * 8];
        }
        // 设置TxHeader
        TxHeader.StdId = 0;
        TxHeader.ExtId = (_Motor_ID << 8) | segment;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = zdt_min(8, _Size - segment * 8);
        TxHeader.TransmitGlobalTime = DISABLE;

        // 发送数据
        HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);

        vTaskDelay(1);
    }
}

/**
 * @brief CAN总线接收回调函数(放在HAL_CAN_RxFifo0MsgPendingCallback中)
 * @param RxHeader 接收到的数据头
 * @param RecvBuf 接收到的数据
 * @retval void
 *
 */
void ZDTMotor::Recv_Callback(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf)
{
    if (_hcan == hcan) {
        if (RxHeader.ExtId >> 8 == Motor_ID) {
            if (RecvBuf[RxHeader.DLC - 1] == DATA_FRAME_END) {
                Clear_Cache();
                Recv_Cache[0] = RxHeader.ExtId >> 8;
                for (uint32_t i = 0; i < RxHeader.DLC; i++) {
                    Recv_Cache[i + 1] = RecvBuf[i];
                }
                Recv_Length = RxHeader.DLC + 1;

                if (Recv_Cache[1] == COMMAND_GET_VELOCITY) {
                    int32_t temp = 0;
                    temp = Recv_Cache[3] << 8 | Recv_Cache[4];
                    if (Recv_Cache[2] == 0x01) {
                        temp = -temp;
                    }
                    Velocity = temp * Direction / 10.0f;
                }
                else if (Recv_Cache[1] == COMMAND_GET_POSITION) {
                    int32_t temp = 0;
                    temp = Recv_Cache[3] << 24 | Recv_Cache[4] << 16 | Recv_Cache[5] << 8 | Recv_Cache[6];
                    if (Recv_Cache[2] == 0x01) {
                        temp = -temp;
                    }
                    Position = temp * Direction / 10.0f;
                }
                else if (Recv_Cache[1] == COMMAND_GET_POS_ERR) {
                    int32_t temp = 0;
                    temp = Recv_Cache[3] << 24 | Recv_Cache[4] << 16 | Recv_Cache[5] << 8 | Recv_Cache[6];
                    if (Recv_Cache[2] == 0x01) {
                        temp = -temp;
                    }
                    PosErr = temp * Direction / 100.0f;
                }
                else if (Recv_Cache[1] == COMMAND_SET_POSITION_TRAP || Recv_Cache[1] == COMMAND_SET_POSITION_PASS) {
                    if (Recv_Cache[2] == 0x9F) {
                        this->StopFlag_PosCtrl = 1;
                    }
                }
            }
        }
    }
}
/**
 * @brief 电机CAN初始化函数
 * @param _hcan can外设句柄
 * @param _Motor_ID 电机ID
 * @param _Direction 电机方向
 * @retval void
 *
 */
void ZDTMotor::Init(CAN_HandleTypeDef* _hcan, uint8_t _Motor_ID, DIRECTION _Direction)
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

        init_state = 1;
    }
    hcan = _hcan;
    Motor_ID = _Motor_ID;
    Direction = _Direction;
}
/**
 * @brief 使能电机
 * @param void
 * @retval void
 *
 */
void ZDTMotor::Enable()
{
    uint8_t Data[5];
    Data[0] = COMMAND_ENABLE_DISABLE;
    Data[1] = 0xAB;
    Data[2] = 0x01;
    Data[3] = 0x00;
    Data[4] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 5);
}
/**
 * @brief 失能电机
 * @param void
 * @retval void
 *
 */
void ZDTMotor::Disable()
{
    uint8_t Data[5];
    Data[0] = COMMAND_ENABLE_DISABLE;
    Data[1] = 0xAB;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 5);
}
/**
 * @brief 设置电机力矩模式转矩
 * @param _Current 电机转矩（单位：mA）
 * @param _CurrentRamp 电机转矩斜坡
 * @param _MultiMode 多电机同步模式
 * @retval void
 *
 */
void ZDTMotor::SetTorque(int32_t _Current, uint16_t _CurrentRamp, MULTI_MODE _MultiMode)
{
    Tar_Cur = _Current;
    _Current *= Direction;
    uint16_t Current = My_ABS(_Current);
    uint8_t Data[9];
    Data[0] = COMMAND_SET_TORQUE;
    Data[1] = _Current >= 0 ? 0x00 : 0x01;
    Data[2] = _CurrentRamp >> 8;
    Data[3] = (_CurrentRamp << 8) >> 8;
    Data[4] = Current >> 8;
    Data[5] = (Current << 8) >> 8;
    Data[6] = _MultiMode;
    Data[7] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 8);
}
/**
 * @brief 设置电机速度模式转速
 * @param _Velocity 电机速度（单位：RPM）
 * @param _VelocityRamp 电机速度斜坡（单位：RPM/s）
 * @param _MultiMode 多电机同步模式
 * @retval void
 *
 */
void ZDTMotor::SetVelocity(float _Velocity, uint16_t _VelocityRamp, MULTI_MODE _MultiMode)
{
    Tar_Vel = _Velocity;
    _Velocity *= Direction;
    uint16_t Velocity = My_ABS(_Velocity * 10);
    uint8_t Data[9];
    Data[0] = COMMAND_SET_VELOCITY;
    Data[1] = _Velocity >= 0 ? 0x00 : 0x01;
    Data[2] = _VelocityRamp >> 8;
    Data[3] = (_VelocityRamp << 8) >> 8;
    Data[4] = Velocity >> 8;
    Data[5] = (Velocity << 8) >> 8;
    Data[6] = _MultiMode;
    Data[7] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 8);
}
/**
 * @brief 设置电机直通位置模式位置
 * @param _Position 电机位置（单位：°）
 * @param _Velocity 电机速度（单位：RPM/s）
 * @param _PositionMode 位置模式（相对模式或绝对模式）
 * @param _MultiMode 多电机同步模式
 * @retval void
 *
 */
void ZDTMotor::SetPosition_PASS(float _Position, float _Velocity, POSITION_MODE _PositionMode, MULTI_MODE _MultiMode)
{
    Tar_Pos = _Position;
    this->StopFlag_PosCtrl = 0;
    _Position *= Direction;
    uint32_t Position = My_ABS(_Position * 10);
    uint16_t Velocity = My_ABS(_Velocity * 10);
    uint8_t Data[11];
    Data[0] = COMMAND_SET_POSITION_PASS;
    Data[1] = _Position >= 0 ? 0x00 : 0x01;
    Data[2] = Velocity >> 8;
    Data[3] = (Velocity << 8) >> 8;
    Data[4] = Position >> 24;
    Data[5] = (Position << 8) >> 24;
    Data[6] = (Position << 16) >> 24;
    Data[7] = (Position << 24) >> 24;
    Data[8] = _PositionMode;
    Data[9] = _MultiMode;
    Data[10] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 11);
}
/**
 * @brief 设置电机梯形曲线位置模式位置
 * @param _Position 电机位置（单位：°）
 * @param _Acc 加速度（单位：RPM/s）
 * @param _Dec 减速度（单位：RPM/s）
 * @param _MaxVel 最大速度（单位：RPM）
 * @param _PositionMode 位置模式（相对模式或绝对模式）
 * @param _MultiMode 多电机同步模式
 * @retval void
 *
 */
void ZDTMotor::SetPosition_TRAP(float _Position, uint16_t _Acc, uint16_t _Dec, float _MaxVel, POSITION_MODE _PositionMode, MULTI_MODE _MultiMode)
{
    Tar_Pos = _Position;
    this->StopFlag_PosCtrl = 0;

    _Position *= Direction;
    uint32_t Position = My_ABS(_Position * 10);
    uint16_t MaxVel = My_ABS(_MaxVel * 10);
    uint8_t Data[15];
    Data[0] = COMMAND_SET_POSITION_TRAP;
    Data[1] = _Position >= 0 ? 0x00 : 0x01;
    Data[2] = _Acc >> 8;
    Data[3] = (_Acc << 8) >> 8;
    Data[4] = _Dec >> 8;
    Data[5] = (_Dec << 8) >> 8;
    Data[6] = MaxVel >> 8;
    Data[7] = (MaxVel << 8) >> 8;
    Data[8] = Position >> 24;
    Data[9] = (Position << 8) >> 24;
    Data[10] = (Position << 16) >> 24;
    Data[11] = (Position << 24) >> 24;
    Data[12] = _PositionMode;
    Data[13] = _MultiMode;
    Data[14] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 15);
}
/**
 * @brief 停止电机
 * @param _MultiMode 多电机同步模式
 * @retval void
 *
 */
void ZDTMotor::StopMotor(MULTI_MODE _MultiMode)
{
    uint8_t Data[4];
    Data[0] = COMMAND_STOP_MOTOR;
    Data[1] = 0x98;
    Data[2] = _MultiMode;
    Data[3] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 4);
}
/**
 * @brief 同步移动
 * @param void
 * @retval void
 *
 */
void ZDTMotor::SyncMove()
{
    uint8_t Data[3];
    Data[0] = COMMAND_SYNC_MOVE;
    Data[1] = 0x66;
    Data[2] = DATA_FRAME_END;
    CAN_Send_Data(0, Data, 3);
}
/**
 * @brief 清空编码器累计值
 * @param void
 * @retval void
 *
 */
void ZDTMotor::ClearEncoder()
{
    uint8_t Data[3];
    Data[0] = COMMAND_CLEAR_ENCODER;
    Data[1] = 0x6D;
    Data[2] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 3);
}
/**
 * @brief 设定电机PID参数
 *
 * @param _save 是否保存参数
 * @param _trap_position_KP 梯形曲线位置环KP
 * @param _pass_position_KP 直通位置环KP
 * @param _velocity_KP 速度环KP
 * @param _veloicty_KI 速度环KI
 */
void ZDTMotor::SetPID(uint8_t _save, uint32_t _trap_position_KP, uint32_t _pass_position_KP, uint32_t _velocity_KP, uint32_t _veloicty_KI)
{
    uint8_t Data[20];
    Data[0] = COMMAND_SET_PID_1;
    Data[1] = COMMAND_SET_PID_2;
    Data[2] = _save;

    Data[3] = _trap_position_KP >> 24;
    Data[4] = (_trap_position_KP << 8) >> 24;
    Data[5] = (_trap_position_KP << 16) >> 24;
    Data[6] = (_trap_position_KP << 24) >> 24;

    Data[7] = _pass_position_KP >> 24;
    Data[8] = (_pass_position_KP << 8) >> 24;
    Data[9] = (_pass_position_KP << 16) >> 24;
    Data[10] = (_pass_position_KP << 24) >> 24;

    Data[11] = _velocity_KP >> 24;
    Data[12] = (_velocity_KP << 8) >> 24;
    Data[13] = (_velocity_KP << 16) >> 24;
    Data[14] = (_velocity_KP << 24) >> 24;

    Data[15] = _veloicty_KI >> 24;
    Data[16] = (_veloicty_KI << 8) >> 24;
    Data[17] = (_veloicty_KI << 16) >> 24;
    Data[18] = (_veloicty_KI << 24) >> 24;

    Data[19] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 20);
}
/**
 * @brief 获取电机速度（放在定时中断中）
 * @param void
 * @retval void
 *
 */
void ZDTMotor::GetVelocity()
{
    uint8_t Data[2];
    Data[0] = COMMAND_GET_VELOCITY;
    Data[1] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 2);
}
/**
 * @brief 获取电机位置（放在定时中断中）
 * @param void
 * @retval void
 *
 */
void ZDTMotor::GetPosition()
{
    uint8_t Data[2];
    Data[0] = COMMAND_GET_POSITION;
    Data[1] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 2);
}
/**
 * @brief 获取电机位置误差
 * @param void
 * @retval void
 *
 */
void ZDTMotor::GetPosErr()
{
    uint8_t Data[2];
    Data[0] = COMMAND_GET_POS_ERR;
    Data[1] = DATA_FRAME_END;
    CAN_Send_Data(this->Motor_ID, Data, 2);
}
/**
 * @brief 清空接收缓存
 * @param void
 * @retval void
 *
 */
void ZDTMotor::Clear_Cache()
{
    for (int i = 0; i < 8; i++) {
        Recv_Cache[i] = 0;
    }
    Recv_Length = 0;
}