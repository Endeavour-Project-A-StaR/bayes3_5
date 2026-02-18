#pragma once

#include "types.h"

bool sd_init();
bool log_init();
bool log_write_frame(FltData_t* fltdata, FltStates_t fltstate, uint32_t ts);
