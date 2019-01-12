#pragma once

#include "pg/rx.h"

#include "rx/rx.h"

#include <stdint.h>
#include <stdbool.h>

bool srxlv2RxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
bool srxlv2RxIsActive(void);
void srxlv2RxWriteData(const void *data, int len);
