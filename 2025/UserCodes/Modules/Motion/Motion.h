#ifndef __MOVE_H__
#define __MOVE_H__

#include "common_inc.h"

class Motion
{
public:
    void Init();

    uint8_t load_from_material(uint8_t _loadDir, uint8_t _last);

    uint8_t load_from_ground(uint8_t _loadDir, uint8_t _unloadDir, uint8_t _is_rotate_out, uint8_t _rotate_out_dir, uint8_t _is_calibrate);

    uint8_t unload_to_ground(uint8_t _loadDir, uint8_t _unloadDir);

    uint8_t unload_to_second(uint8_t _loadDir, uint8_t _unloadDir);

};
extern Motion motion;
#endif