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

#include "BME280/bme280.h"

#include "drv_bme280.h"

#define DRV_DEBUG
#define LOG_TAG     "drv.bme280"
#include <rtdbg.h>

static rt_device_t i2c_bus = NULL;
static struct bme280_dev bme_dev;


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

static int8_t bme_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return read_regs(i2c_bus, *(uint8_t*)intf_ptr, reg_addr, reg_data, len);
}

static int8_t bme_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,void *intf_ptr)
{
    return write_regs(i2c_bus, *(uint8_t*)intf_ptr, reg_addr, reg_data, len);
}

// it is safe to use ms delay, since the bme drive only use twice of it and each of them is the multiply of 1000/.
static void bme_delay_us(uint32_t us)
{
    rt_thread_mdelay(us/1000);
}


int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;
    dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
    return rslt;
}

int8_t bme280_sleep_mode(struct bme280_dev *dev)
{
    return bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
}

int8_t bme280_get_data(double *h, double *t, double*p)
{
    int8_t rslt;
    struct bme280_data comp_data;
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme_dev);

    *h = comp_data.humidity;
    *t = comp_data.temperature;
    *p = comp_data.pressure;

    return rslt;
}


int bme280_initialization(rt_device_t bus)
{
    int8_t rslt = BME280_OK;
    static uint8_t dev_addr = BME280_I2C_ADDR_SEC;// BME280_I2C_ADDR_PRIM;

    i2c_bus = bus;

    bme_dev.intf_ptr = &dev_addr;
    bme_dev.intf = BME280_I2C_INTF;
    bme_dev.read = bme_i2c_read;
    bme_dev.write = bme_i2c_write;
    bme_dev.delay_us = bme_delay_us;

    rslt = bme280_init(&bme_dev);

    // set to continual mode for test.
    stream_sensor_data_normal_mode(&bme_dev);

    return 0;
}



