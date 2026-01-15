#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"

esp_err_t tmp117_init(void);

esp_err_t tmp117_read_celsius(float *degC);

#ifdef __cplusplus
}
#endif
