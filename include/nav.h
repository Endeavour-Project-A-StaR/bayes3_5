#pragma once

#include "types.h"

    void nav_init();
    void nav_update_pid(FltData_t *fltdata, float dt);
    void nav_write_servo(FltData_t *fltdata);
