#include "bsp_can.h"
#include "DMMotor.h"
#include "DJIMotor.h"
#include "ZDTMotor.h"
/*------------------CAN总线接收中断回调函数-------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RecvBuf[8] = { 0 };
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RecvBuf);
    dm_gimbal.Recv_Callback(hcan, RxHeader, RecvBuf);
    dji_mot1.Recv_Callback(hcan, RxHeader, RecvBuf);
    dji_mot2.Recv_Callback(hcan, RxHeader, RecvBuf);
    dji_mot3.Recv_Callback(hcan, RxHeader, RecvBuf);
    dji_mot4.Recv_Callback(hcan, RxHeader, RecvBuf);
    zdt_hori.Recv_Callback(hcan, RxHeader, RecvBuf);
    zdt_vert.Recv_Callback(hcan, RxHeader, RecvBuf);

}
