#ifndef __MOVE_H__
#define __MOVE_H__

#include "common_inc.h"

#define CAMERA_IN_ANGLE -18
#define CAMERA_OUT_ANGLE -244
#define CAMERA_INTERVAL 1000

#define JIAZHUA_CLOSE_ANGLE -82
#define JIAZHUA_OPEN_ANGLE -120
#define JIAZHUA_INTERVAL 200
#define JIAZHUA_FINISHED_THRESHOLD 7

#define ZAIWU_FRONT_ANGLE 180
#define ZAIWU_MID_ANGLE 90
#define ZAIWU_BACK_ANGLE 0
#define ZAIWU_INTERVAL 500
#define ZAIWU_FINISHED_THRESHOLD 5

#define HUAGUI_TOP_ANGLE 0
#define HUAGUI_GROUND_ANGLE 1460
#define HUAGUI_ZHUANPAN_ANGLE 760
#define HUAGUI_ZAIWU_PUT_ANGLE 200
#define HUAGUI_ZAIWU_GET_ANGLE 330
#define HUAGUI_SECOND_ANGLE 840
#define HUAGUI_VELOCITY 1500
#define HUAGUI_ACC 2000
#define HUAGUI_FINISHED_THRESHOLD 5

#define YUNTAI_VELOCITY 7
#define YUNTAI_OUT_ANGLE 0
#define YUNTAI_IN_ANGLE 3.08
#define YUNTAI_FINISHED_THRESHOLD 0.1

class Motion
{
public:
    void Init();
    void Huagui_Move(float _angle, float _acc, float _vel);
    uint8_t Huagui_Finished();
    void Yuntai_Out();
    void Yuntai_In();
    uint8_t Yuntai_Finished();
    void Jiazhua_Close();
    void Jiazhua_Open();
    uint8_t Jiazhua_Finished();
    void Zaiwu_Front();
    void Zaiwu_Mid();
    void Zaiwu_Back();
    uint8_t Zaiwu_Finished();
    void Camera_In();
    void Camera_Out();

    uint8_t load_from_material(uint8_t _loadDir);
    uint8_t load_from_ground(uint8_t _loadDir);
    uint8_t ready_to_load = 0;

    uint8_t get_from_car(uint8_t _loadDir);
    uint8_t ready_to_unload = 0;

    uint8_t unload_to_ground();
    uint8_t unload_to_second();

};
extern Motion motion;
#endif