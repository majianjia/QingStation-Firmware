/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-27     Jianjia Ma       the first version
 */

#ifndef __DRV_AS3935_H__
#define __DRV_AS3935_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#define AS3935_I2C_ADDR  (0x3)

#define AS3935_DEFAULT_REG      0x3C
#define AS3935_CALIB_REG        0x3D
#define AS3935_TUNECAP_REG      0x08
#define AS3935_NFLWDTH_REG      0x01
#define AS3935_DATA_REG         0x07
#define AS3935_TUNEANT_REG      0x03

int as3935_read_data(uint32_t* distance, uint32_t* energy);

int as3935_init(rt_device_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_APDS9250_H__ */
