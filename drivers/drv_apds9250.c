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

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "drv_soft_i2c.h"

#include "drv_apds9250.h"

#define DRV_DEBUG
#define LOG_TAG     "drv.apds9250"
#include <rtdbg.h>

static rt_device_t i2c_bus = NULL;


static rt_err_t write_regs(void *bus, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t read_regs(void *bus, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}


int apds9250_read_rgbir(uint32_t* r, uint32_t* g, uint32_t* b, uint32_t* ir)
{
    uint8_t data[12];

    if(!i2c_bus)
        return -1;

    read_regs(i2c_bus, APDS_I2C_ADDR, APDS_LS_DATA_IR_0, data, 12);
    *ir = data[0] | (uint32_t)data[1] << 8 | (uint32_t) data[2] << 16;
    *g = data[3] | (uint32_t)data[4] << 8 | (uint32_t) data[5] << 16;
    *b = data[6] | (uint32_t)data[7] << 8 | (uint32_t) data[8] << 16;
    *r = data[9] | (uint32_t)data[10] << 8 | (uint32_t) data[11] << 16;

    return 0;
}

int apds9250_read_alsir(uint32_t* als, uint32_t* ir)
{
    uint8_t data[12];

    if(!i2c_bus)
        return -1;

    read_regs(i2c_bus, APDS_I2C_ADDR, APDS_LS_DATA_IR_0, data, 12);
    *ir = data[0] | (uint32_t)data[1] << 8 | (uint32_t) data[2] << 16;
    *als = data[3] | (uint32_t)data[4] << 8 | (uint32_t) data[5] << 16;

    return 0;
}



int apds9250_select_sensor(uint8_t select)
{
    if(!i2c_bus)
        return -1;
    uint8_t temp;
    temp = 0x04 | select;
    write_regs(i2c_bus, APDS_I2C_ADDR, APDS_MAIN_CTRL, &temp, 1);

    return 0;
}

int apds9250_init(rt_device_t bus)
{
    uint8_t temp;
    if(!bus)
        return -1;
    i2c_bus = bus;

    // read id = 0xB2
    read_regs(bus, APDS_I2C_ADDR, APDS_PART_ID, &temp, 1);
    LOG_D("APDS9250 ID reg: 0x%x", temp);

    // reset
    temp = 0x10;
    write_regs(bus, APDS_I2C_ADDR, APDS_MAIN_CTRL, &temp, 1);
    rt_thread_mdelay(10);

    // all sensor on
    temp = 0x02 | 0x04;
    write_regs(bus, APDS_I2C_ADDR, APDS_MAIN_CTRL, &temp, 1);

    // measurement rate, 1000ms. 20 bits.
    temp = 0x05;
    write_regs(bus, APDS_I2C_ADDR, APDS_LS_MEAS_RATE, &temp, 1);

    // gain, set to minimum.
    temp = 0x00;
    write_regs(bus, APDS_I2C_ADDR, APDS_LS_GAIN, &temp, 1);

    return 0;
}



