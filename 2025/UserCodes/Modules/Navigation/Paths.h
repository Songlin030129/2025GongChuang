#ifndef __PATHS_H__
#define __PATHS_H__
#include "common_inc.h"
#include "Navigation.h"
//                        pos_x   pos_y  theta
const position_t pos_start = { 0.00f, -0.00f, 0.00f };//起点
const position_t pos_qrcode = { 0.50f, -0.20f, 0.00f };//二维码区
const position_t pos_material = { 1.42f, -0.15f, 0.00f };//原料区
const position_t pos_material_process = { 1.03f, -0.15f, 0.00f };
// const position_t pos_material_process = { 1.03f, -0.51f, 0.00f };
const position_t pos_process = { 1.02f, -1.89f, -3.14f };//加工区
const position_t pos_process_storage_1 = { 1.49f, -1.87f, -1.57f };
const position_t pos_process_storage_2 = { 1.83f, -1.50f, -1.57f };
const position_t pos_storage = { 1.85f, -1.03f, -1.57f };//暂存区
const position_t pos_storage_start_1 = { 1.83f, -0.49f, -0.00f };
const position_t pos_storage_start_2 = { 0.30f, -0.20f, 0.00f };

// const arc_t circle_material_process = { 1.40f, -0.50f, 0.38f };
const arc_t circle_process_storage = { 1.49f, -1.50f, 0.35f };
const arc_t circle_storage_material = { 1.42f, -0.49f, 0.36f };

class Paths
{
public:

    uint8_t task_from_start_to_qrcode();
    uint8_t task_from_qrcode_to_material();
    uint8_t task_from_material_to_process();
    uint8_t task_from_process_to_storage();
    uint8_t task_from_storage_to_material();
    uint8_t task_from_storage_to_stop();

private:
    Navigation navi_start_to_qrcode;
    path_t path_start_to_qrcode[1] = {
        { PATH_TYPE_START_END, PATH_SHAPE_LINE, pos_start, pos_qrcode }
    };
    Navigation navi_qrcode_to_material;
    path_t path_qrcode_to_material[1] = {
        { PATH_TYPE_START_END, PATH_SHAPE_LINE, pos_qrcode, pos_material }
    };
    Navigation navi_material_to_process;
    path_t path_material_to_process[2] = {
        { PATH_TYPE_START_END, PATH_SHAPE_LINE, pos_material, pos_material_process},
        { PATH_TYPE_START_END, PATH_SHAPE_LINE, pos_material_process, pos_process}
    };
    Navigation navi_process_to_storage;
    path_t path_process_to_storage[3] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_process, pos_process_storage_1},
        { PATH_TYPE_MID, PATH_SHAPE_CIRCLE, pos_process_storage_1, pos_process_storage_2, circle_process_storage},
        { PATH_TYPE_END, PATH_SHAPE_LINE, pos_process_storage_2, pos_storage}
    };
    Navigation navi_storage_to_material;
    path_t path_storage_to_material[2] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_storage, pos_storage_start_1},
        { PATH_TYPE_END, PATH_SHAPE_CIRCLE, pos_storage_start_1, pos_material, circle_storage_material},
    };
    Navigation navi_storage_to_stop;
    path_t path_storage_to_stop[4] = {
        { PATH_TYPE_START, PATH_SHAPE_LINE, pos_storage, pos_storage_start_1},
        { PATH_TYPE_MID, PATH_SHAPE_CIRCLE, pos_storage_start_1, pos_material, circle_storage_material},
        { PATH_TYPE_MID, PATH_SHAPE_LINE, pos_material, pos_storage_start_2},
        { PATH_TYPE_END, PATH_SHAPE_LINE, pos_storage_start_2, pos_start}
    };


};

extern Paths paths;

#endif