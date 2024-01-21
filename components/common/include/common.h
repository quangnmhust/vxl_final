#ifndef LIB_COMMON_H_
#define LIB_COMMON_H_
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

typedef enum {
    SAFE = 0,
	DANGER,
} status;

typedef struct
{
	status device_status;
} device_status_t;

typedef struct
{
	float temp;
	float humi;
	float flame_data;
	float LPG;
	float CO;
	float prob;

} data_type;

typedef enum {
    LINEAR = 1,
	EXPON = 2
} _regressionMethod;

typedef struct
{
	float MQ_correctionFactor;
	float ratioInCleanAir;

	float _VOL_RESOLUTION;
	float _RL;

	float _adc, _a_LPG, _b_LPG, _a_CO, _b_CO, _sensor_volt;
	float _R0, _Rs_air, _ratio, _PPM_LPG, _PPM_CO, _Rs_calc;

	_regressionMethod MQ_regressMethod;
} MQdata_st;

#endif
