/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-07     Jianjia Ma       the first version
 */

#ifndef __DRV_BME280_H__
#define __DRV_BME280_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

int8_t bme280_get_data(double *h, double *t, double*p);

int bme280_initialization(rt_device_t bus);



#ifdef __cplusplus
}
#endif

#endif /* __DRV_BME280_H__ */
