#pragma once

#include <stdio.h>
#include "types.h"

int serializer(char* buffer, size_t buf_size, uint32_t timestamp, FltStates_t state, const FltData_t* fltdata);
bool sd_init();
bool log_init();
bool log_write_frame(FltData_t *fltdata, FltStates_t fltstate, uint32_t ts);
