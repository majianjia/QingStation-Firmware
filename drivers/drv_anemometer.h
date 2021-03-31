/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-06     Jianjia Ma       the first version
 */
#ifndef __DRV_ANEMOMETER_H__
#define __DRV_ANEMOMETER_H__

#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

// The output channel. When it is selected, the corresponding input channel will also be selected.
typedef enum {
    NORTH = 0,
    EAST,
    SOUTH,
    WEST
} ULTRASONIC_CHANNEL;

static char ane_ch_names[][2] = {
    "N",
    "E",
    "S",
    "W"
};

// pin for analog swith selection.
#define PIN_DRV_EN0    GET_PIN(A, 5)
#define PIN_DRV_EN1    GET_PIN(B, 2)
#define PIN_SWOPA_EN   GET_PIN(B, 1)  // this also enable the OPA (analog power supply pmos)
#define PIN_SW_A    GET_PIN(B, 0)
#define PIN_SW_B    GET_PIN(A, 7)

void ane_pwr_control(uint32_t freq, bool flag);

bool ane_check_busy();

int ane_measure_ch(ULTRASONIC_CHANNEL ch, uint16_t *pulse, uint16_t pulse_len, uint16_t* adc_buf, uint32_t adc_len);

int adc_sample(ULTRASONIC_CHANNEL ch, uint16_t* adc_buf, uint32_t adc_len);


#ifdef __cplusplus
}
#endif

#endif /* _DRV_ANEMOMETER_H_ */
