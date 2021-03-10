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

#ifndef __DRV_APDS9250_H__
#define __DRV_APDS9250_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#define APDS_I2C_ADDR           0x52

#define APDS_MAIN_CTRL          0x00
#define APDS_LS_MEAS_RATE       0x04
#define APDS_LS_GAIN            0x05
#define APDS_PART_ID            0x06
#define APDS_MAIN_STATUS        0x07
#define APDS_LS_DATA_IR_0       0x0A
#define APDS_LS_DATA_IR_1       0x0B
#define APDS_LS_DATA_IR_2       0x0C
#define APDS_LS_DATA_GREEN_0    0x0D
#define APDS_LS_DATA_GREEN_1    0x0E
#define APDS_LS_DATA_GREEN_2    0x0F
#define APDS_LS_DATA_BLUE_0     0x10
#define APDS_LS_DATA_BLUE_1     0x11
#define APDS_LS_DATA_BLUE_2     0x12
#define APDS_LS_DATA_RED_0      0x13
#define APDS_LS_DATA_RED_1      0x14
#define APDS_LS_DATA_RED_2      0x15
#define APDS_INT_CFG            0x19
#define APDS_INT_PERSISTENCE    0x1A
#define APDS_LS_THRES_UP_0      0x21
#define APDS_LS_THRES_UP_1      0x22
#define APDS_LS_THRES_UP_2      0x23
#define APDS_LS_THRES_LOW_0     0x24
#define APDS_LS_THRES_LOW_1     0x25
#define APDS_LS_THRES_LOW_2     0x26
#define APDS_LS_THRES_VAR       0x27


#define APDS_SELECT_RGBIR   0x02
#define APDS_SELECT_ALSIR   0x00

int apds9250_read_rgbir(uint32_t* r, uint32_t* g, uint32_t* b, uint32_t* ir);

int apds9250_read_alsir(uint32_t* als,  uint32_t* ir);

int apds9250_select_sensor(uint8_t select);

int apds9250_init(rt_device_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_APDS9250_H__ */
