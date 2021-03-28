/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-27     Jianjia Ma       the first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "drv_soft_i2c.h"

#include "drv_as3935.h"

#define DRV_DEBUG
#define LOG_TAG     "drv.as3935"
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

int as3935_read_data(uint32_t* distance, uint32_t* energy)
{
    uint8_t data[4];
    if(!i2c_bus)
        return -1;

    read_regs(i2c_bus, AS3935_I2C_ADDR, 0x4, data, 4);
    *energy = data[0] | ((uint32_t)data[1]<<8) | ((uint32_t)data[2]<<16);
    *distance = data[3];
    return 0;
}


int as3935_init(rt_device_t bus)
{
    uint8_t temp;
    if(!bus)
        return -1;
    i2c_bus = bus;

    // print all the registor for test
    for(int i=0; i<9; i++)
    {
        read_regs(i2c_bus, AS3935_I2C_ADDR, i, &temp, 1);
        printf("REG: 0x%02X, 0x%02X\n", i, temp);
    }

    // reset
    temp = 0x1;
    write_regs(bus, AS3935_I2C_ADDR, 0x00 , &temp, 1);
    rt_thread_mdelay(10);
    temp = 0x0;
    write_regs(bus, AS3935_I2C_ADDR, 0x00, &temp, 1);

    // recalibrate
    temp = 0x96;
    write_regs(bus, AS3935_I2C_ADDR, AS3935_DEFAULT_REG, &temp, 1);
    write_regs(bus, AS3935_I2C_ADDR, AS3935_CALIB_REG, &temp, 1);

    temp = 0x40;
    write_regs(bus, AS3935_I2C_ADDR, AS3935_TUNECAP_REG, &temp, 1);
    rt_thread_mdelay(10);
    temp = 0x00;
    write_regs(bus, AS3935_I2C_ADDR, AS3935_TUNECAP_REG, &temp, 1);

    return 0;
}




