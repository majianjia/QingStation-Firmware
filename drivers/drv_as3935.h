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


#define AS3935_LCO      0x80
#define AS3935_SRCO     0x40
#define AS3935_TRCO     0x20

#define AS3935_INT_NOISE 0x01
#define AS3935_INT_D     0x04       // Disturber detected
#define AS3935_INT_L     0x08       // lightning
#define AS3935_INT_NONE     0x00

int as3935_enable_clock_output(uint8_t which);
int as3935_interrupt_set(uint8_t mask);
int as3935_noise_level_set(uint8_t value); // 0-7
uint8_t as3935_read_int();

int as3935_read_data(uint32_t* distance, uint32_t* energy);

int as3935_init(rt_device_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_APDS9250_H__ */
