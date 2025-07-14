#ifndef __ZDTMOTOR_H
#define __ZDTMOTOR_H
#include "common_inc.h"
#define min(a, b)                 ((a) < (b) ? (a) : (b))          // 取最小值
#define My_ABS(temp)              ((temp) >= 0 ? (temp) : -(temp)) // 取绝对值

#define COMMAND_ENABLE_DISABLE    0xF3 // 使能/失能命令
#define COMMAND_SET_TORQUE        0xF5 // 力矩模式命令
#define COMMAND_SET_VELOCITY      0xF6 // 速度模式命令
#define COMMAND_SET_POSITION_PASS 0xFB // 直通位置模式命令
#define COMMAND_SET_POSITION_TRAP 0xFD // 梯形位置模式命令
#define COMMAND_STOP_MOTOR        0xFE // 停止命令
#define COMMAND_SYNC_MOVE         0xFF // 同步移动命令
#define COMMAND_CLEAR_ENCODER     0x0A // 清空编码器命令
#define COMMAND_GET_VELOCITY      0x35 // 获取速度命令
#define COMMAND_GET_POSITION      0x36 // 获取位置命令
#define COMMAND_GET_POS_ERR       0x37 // 获取位置误差
#define COMMAND_SET_PID_1         0x4A // 设定电机PID参数命令1
#define COMMAND_SET_PID_2         0xC3 // 设定电机PID参数命令2
#define DATA_FRAME_END            0x6B // 帧尾

class ZDTMotor
{
public:
    typedef enum DIRECTION {
        DIRECTION_POSITIVE = 1,
        DIRECTION_NEGATIVE = -1,
    } DIRECTION; // 电机方向枚举
    typedef enum MULTI_MODE {
        MULTI_MODE_SYNC = 1,
        MULTI_MODE_ASYNC = 0,
    } MULTI_MODE; // 多机同步模式枚举
    typedef enum POSITION_MODE {
        POSITION_MODE_RELATIVE = 0,
        POSITION_MODE_ABSOLUT = 1,
    } POSITION_MODE; // 位置闭环模式枚举

    /**
     * @brief 电机CAN初始化函数
     * @param _hcan can外设句柄
     * @param _Motor_ID 电机ID
     * @param _Direction 电机方向
     * @retval void
     *
     */
    void Init(CAN_HandleTypeDef* _hcan, uint8_t _Motor_ID, DIRECTION _Direction);
    /**
     * @brief CAN总线接收回调函数(放在HAL_CAN_RxFifo0MsgPendingCallback中)
     * @param RxHeader 接收到的数据头
     * @param RecvBuf 接收到的数据
     * @retval void
     *
     */
    void Recv_Callback(CAN_HandleTypeDef* _hcan, CAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf);
    /**
     * @brief 使能电机
     * @param void
     * @retval void
     *
     */
    void Enable();
    /**
     * @brief 失能电机
     * @param void
     * @retval void
     *
     */
    void Disable();
    /**
     * @brief 设置电机力矩模式转矩
     * @param _Current 电机转矩（单位：mA）
     * @param _CurrentRamp 电机转矩斜坡
     * @param _MultiMode 多电机同步模式
     * @retval void
     *
     */
    void SetTorque(int32_t _Current, uint16_t _CurrentRamp, MULTI_MODE _MultiMode);
    /**
     * @brief 设置电机速度模式转速
     * @param _Velocity 电机速度（单位：RPM）
     * @param _VelocityRamp 电机速度斜坡（单位：RPM/s）
     * @param _MultiMode 多电机同步模式
     * @retval void
     *
     */
    void SetVelocity(float _Velocity, uint16_t _VelocityRamp, MULTI_MODE _MultiMode);
    /**
     * @brief 设置电机直通位置模式位置
     * @param _Position 电机位置（单位：°）
     * @param _Velocity 电机速度（单位：RPM/s）
     * @param _PositionMode 位置模式（相对模式或绝对模式）
     * @param _MultiMode 多电机同步模式
     * @retval void
     *
     */
    void SetPosition_PASS(float _Position, float _Velocity, POSITION_MODE _PositionMode, MULTI_MODE _MultiMode);
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
    void SetPosition_TRAP(float _Position, uint16_t _Acc, uint16_t _Dec, float _MaxVel, POSITION_MODE _PositionMode, MULTI_MODE _MultiMode);
    /**
     * @brief 停止电机
     * @param _MultiMode 多电机同步模式
     * @retval void
     *
     */
    void StopMotor(MULTI_MODE _MultiMode);
    /**
     * @brief 同步移动
     * @param void
     * @retval void
     *
     */
    void SyncMove();
    /**
     * @brief 清空编码器累计值
     * @param void
     * @retval void
     *
     */
    void ClearEncoder();
    /**
     * @brief 设定电机PID参数
     *
     * @param _save 是否保存参数
     * @param _trap_position_KP 梯形曲线位置环KP
     * @param _pass_position_KP 直通位置环KP
     * @param _velocity_KP 速度环KP
     * @param _veloicty_KI 速度环KI
     */
    void SetPID(uint8_t _save, uint32_t _trap_position_KP, uint32_t _pass_position_KP, uint32_t _velocity_KP, uint32_t _veloicty_KI);
    /**
     * @brief 获取电机速度（放在定时中断中）
     * @param void
     * @retval void
     *
     */
    void GetVelocity();
    /**
     * @brief 获取电机位置（放在定时中断中）
     * @param void
     * @retval void
     *
     */
    void GetPosition();
    /**
     * @brief 获取电机位置误差
     * @param void
     * @retval void
     *
     */
    void GetPosErr();
    float Tar_Pos, Tar_Vel, Tar_Cur;
    float Position, Velocity, PosErr;
    uint8_t StopFlag_PosCtrl = 1;

private:
    /**
     * @brief 清空接收缓存
     * @param void
     * @retval void
     *
     */
    void Clear_Cache();
    /**
     * @brief CAN总线发送数据
     * @param _Motor_ID 电机ID
     * @param Data 要发送的数据
     * @param Byte_Count 数据长度
     * @retval void
     *
     */
    void CAN_Send_Data(uint8_t _Motor_ID, uint8_t* _Data, uint8_t _Size);

    uint8_t Motor_ID;
    int8_t Direction;
    uint8_t Recv_Cache[8];
    uint8_t Recv_Length;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = { 0 };
    CAN_HandleTypeDef* hcan;
};
extern ZDTMotor zdt_hori, zdt_vert;


#endif // __ZDTMotor_H