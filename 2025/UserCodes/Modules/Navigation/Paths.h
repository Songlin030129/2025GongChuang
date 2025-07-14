#ifndef __PATHS_H__
#define __PATHS_H__
#include "common_inc.h"
#include "Navigation.h"
//                        pos_x   pos_y  theta
const position_t pos_1 = { 0.00f, -0.00f, 0.00f };
const position_t pos_2 = { 0.70f, -0.20f, 0.00f };
const position_t pos_3 = { 1.43f, -0.16f, 0.00f };
const position_t pos_4 = { 1.25f, -0.20f, 0.00f };
const position_t pos_5 = { 1.00f, -0.45f, 0.00f };
const position_t pos_6 = { 1.02f, -1.85f, -3.14f };
const position_t pos_7 = { 0.98f, -1.95f, -3.14f };
const position_t pos_8 = { 1.62f, -1.95f, -1.57f };
const position_t pos_9 = { 1.87f, -1.70f, -1.57f };
const position_t pos_10 = { 1.87f, -1.12f, -1.57f };
const position_t pos_11 = { 1.90f, -0.40f, -0.00f };
const position_t pos_12 = { 1.65f, -0.15f, 0.00f };
const position_t pos_13 = { 0.30f, -0.15f, 0.00f };

const position_t pos_7_1 = { 1.08f, -1.95f, -3.14f };
const position_t pos_7_2 = pos_7;
const position_t pos_7_3 = { 0.80f, -1.95f, -3.14f };

const position_t pos_10_1 = { 1.87f, -0.97f, -1.57f };
const position_t pos_10_2 = pos_10;
const position_t pos_10_3 = { 1.87f, -1.27f, -1.57f };

const arc_t circle_1 = { 1.25f, -0.45f, 0.25f };
const arc_t circle_2 = { 1.62f, -1.70f, 0.25f };
const arc_t circle_3 = { 1.65f, -0.40f, 0.25f };

class Paths
{
public:

    uint8_t task_from_1_to_2();
    uint8_t task_from_2_to_3();
    uint8_t task_from_3_to_7(uint8_t _des_flag);
    uint8_t task_from_7_to_10(uint8_t _src_flag, uint8_t _des_flag);
    uint8_t task_from_10_to_3(uint8_t _src_flag);
    uint8_t task_from_10_to_1(uint8_t _src_flag);

private:
    Navigation navi_1_to_2;
    path_t path_1_to_2 = {
         PATH_TYPE_START_END, PATH_SHAPE_LINE, pos_1, pos_2
    };
    Navigation navi_2_to_3;
    path_t path_2_to_3 = {
        PATH_TYPE_START_END, PATH_SHAPE_LINE, pos_2, pos_3
    };
    Navigation navi_3_to_7;
    path_t path_3_to_7[4] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_3, pos_4},
        { PATH_TYPE_MID, PATH_SHAPE_CIRCLE, pos_4, pos_5, circle_1},
        { PATH_TYPE_MID, PATH_SHAPE_LINE, pos_5, pos_6},
        { PATH_TYPE_END, PATH_SHAPE_LINE, pos_6, pos_7}
    };
    Navigation navi_7_to_10;
    path_t path_7_to_10[3] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_7, pos_8},
        { PATH_TYPE_MID, PATH_SHAPE_CIRCLE, pos_8, pos_9, circle_2},
        { PATH_TYPE_END, PATH_SHAPE_LINE, pos_9, pos_10}
    };
    Navigation navi_10_to_3;
    path_t path_10_to_3[3] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_10, pos_11},
        { PATH_TYPE_MID, PATH_SHAPE_CIRCLE, pos_11, pos_12, circle_3},
        { PATH_TYPE_END, PATH_SHAPE_LINE, pos_12, pos_3}
    };
    Navigation navi_10_to_1;
    path_t path_10_to_1[4] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_10, pos_11},
        { PATH_TYPE_MID, PATH_SHAPE_CIRCLE, pos_11, pos_12, circle_3},
        { PATH_TYPE_MID, PATH_SHAPE_LINE, pos_12, pos_13},
        { PATH_TYPE_END, PATH_SHAPE_LINE, pos_13, pos_1}
    };


};

extern Paths paths;

#endif