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

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "drv_soft_i2c.h"
#include "imu.h"

#include "BMI160/bmi160.h"     // using bmi160 driver for bmx160
#include "BMM150/bmm150.h"
#include "drv_bmx160.h"

#define DRV_DEBUG
#define LOG_TAG         "drv.bmx160"
#include <rtdbg.h>

typedef struct _bmx_imu_dev_t
{
    imu_dev_t super;
    struct _bmx_imu_dev_t *next;    // a simple list for id based search.
    struct bmi160_dev bmi;
    struct bmm150_dev bmm;

    /* Buffer to store the Mag data from 0x42 to 0x48 */
    uint8_t mag_data[8];
} bmx_imu_dev_t;

static rt_device_t i2c_bus = 0;

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

int8_t bmx_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return read_regs(i2c_bus, dev_addr, reg_addr, data, len);
}

int8_t bmx_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return write_regs(i2c_bus, dev_addr, reg_addr, data, len);
}

static bmx_imu_dev_t * bmx_instance_copy; // that is not a good design by bosch.
// this address is fixed to BMI's address.
int8_t bmm_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return bmi160_aux_read(reg_addr, data, len, &bmx_instance_copy->bmi); // use a device but didnt pass in??
    //return read_regs(i2c_bus, BMI160_I2C_ADDR, reg_addr, data, len);
}

int8_t bmm_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return bmi160_aux_write(reg_addr, data, len, &bmx_instance_copy->bmi);
    //return write_regs(i2c_bus, BMI160_I2C_ADDR, reg_addr, data, len);
}


static int bmx160_read_raw(imu_dev_t *imu)
{
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
    bmx_imu_dev_t *bmx = (bmx_imu_dev_t *) imu;
    uint8_t data[2];

    /* To read both Accel and Gyro data along with time*/
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &bmx->bmi);
    bmi160_read_aux_data_auto_mode(bmx->mag_data, &bmx->bmi);
    bmm150_aux_mag_data(bmx->mag_data, &bmx->bmm);

    // write to
    imu->raw.acc_x = accel.x;
    imu->raw.acc_y = accel.y;
    imu->raw.acc_z = accel.z;
    imu->raw.gyro_x = gyro.x;
    imu->raw.gyro_y = gyro.y;
    imu->raw.gyro_z = gyro.z;
    imu->raw.mag_x = bmx->bmm.data.x;
    imu->raw.mag_y = bmx->bmm.data.y;
    imu->raw.mag_z = bmx->bmm.data.z;
    imu->raw.temperature = ((uint16_t)data[1])<<8 | data[0];

    return 0;
}

static void bmx_delay_ms(uint32_t ms)
{
    rt_thread_mdelay(ms);
}
//
//// setting source
//// https://github.com/BoschSensortec/BMI160_driver#accessing-auxiliary-bmm150-with-bmm150-apis-via-bmi160-secondary-interface
//int bmx160_init(struct imu_dev_t * dev)
//{
//    bmx_imu_dev_t * imu = (bmx_imu_dev_t *)dev;
//
//    // init bmx 160
//    imu->bmi.id = BMI160_I2C_ADDR;
//    imu->bmi.interface = BMI160_I2C_INTF;
//    imu->bmi.read = bmx_i2c_read;
//    imu->bmi.write = bmx_i2c_write;
//    imu->bmi.delay_ms = bmx_delay_ms;
//
//    int8_t rslt = BMI160_OK;
//    rslt = bmi160_init(&imu->bmi);
//
//    /* Select the Output data rate, range of accelerometer sensor */
//    imu->bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
//    imu->bmi.accel_cfg.range = BMI160_ACCEL_RANGE_16G ;
//    imu->bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
//
//    /* Select the power mode of accelerometer sensor */
//    imu->bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
//
//    /* Select the Output data rate, range of Gyroscope sensor */
//    imu->bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
//    imu->bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
//    imu->bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
//
//    /* Select the power mode of Gyroscope sensor */
//    imu->bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
//
//    /* Set the sensor configuration */
//    rslt = bmi160_set_sens_conf(&imu->bmi);
//
//    /* Configure device structure for auxiliary sensor parameter */
//    imu->bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE; // auxiliary sensor enable
//    imu->bmi.aux_cfg.aux_i2c_addr = BMI160_AUX_BMM150_I2C_ADDR; // auxiliary sensor address
//    imu->bmi.aux_cfg.manual_enable = BMI160_ENABLE; // setup mode enable
//    imu->bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3;// burst read of 8 byte
//
//    /* Configure the BMM150 device structure by
//    mapping user_aux_read and user_aux_write */
//    imu->bmm.read = bmm_i2c_read;
//    imu->bmm.write = bmm_i2c_write;
//    imu->bmm.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
//    /* Ensure that sensor.aux_cfg.aux_i2c_addr = bmm150.id
//       for proper sensor operation */
//    imu->bmm.delay_ms = bmx_delay_ms;
//    imu->bmm.intf = BMM150_I2C_INTF;  //test
//
//    /* Initialize the auxiliary sensor interface */
//    rslt = bmi160_aux_init(&imu->bmi);
//
//    /* Auxiliary sensor is enabled and can be accessed from this point */
//
//    /* Configure the desired settings in auxiliary BMM150 sensor
//     * using the bmm150 APIs */
//
//    /* Initialising the bmm150 sensor */
//    rslt = bmm150_init(&imu->bmm);
//
//    /* Set the power mode and preset mode to enable Mag data sampling */
////    imu->bmm.settings.pwr_mode = BMM150_NORMAL_MODE;//BMM150_NORMAL_MODE;
////    rslt = bmm150_set_op_mode(&imu->bmm);
//    imu->bmm.settings.pwr_mode = BMM150_FORCED_MODE;
//    rslt = bmm150_set_op_mode(&imu->bmm);
//
//    imu->bmm.settings.preset_mode= BMM150_PRESETMODE_LOWPOWER;//BMM150_PRESETMODE_LOWPOWER;
//    rslt = bmm150_set_presetmode(&imu->bmm);
//
//    /* In BMM150 Mag data starts from register address 0x42 */
//    uint8_t aux_addr = 0x42;
//
//    /* Configure the Auxiliary sensor either in auto/manual modes and set the
//    polling frequency for the Auxiliary interface */
//    imu->bmi.aux_cfg.aux_odr = 6; /* 8 polling rate in 100 Hz*/ //6 for 25
//    rslt = bmi160_config_aux_mode(&imu->bmi);
//
//    /* Set the auxiliary sensor to auto mode */
//    rslt = bmi160_set_aux_auto_mode(&aux_addr, &imu->bmi);
//
//    /* Reading data from BMI160 data registers */
//    rslt = bmi160_read_aux_data_auto_mode(imu->mag_data, &imu->bmi);
//
//    // marked init done then return the instance.
//    imu->super.is_inited = 1;
//
//    return 0;
//}


// setting source
// https://github.com/BoschSensortec/BMI160_driver#accessing-auxiliary-bmm150-with-bmm150-apis-via-bmi160-secondary-interface
int bmx160_init(struct imu_dev_t * dev)
{
    bmx_imu_dev_t * imu = (bmx_imu_dev_t *)dev;
    // for i2c
    bmx_instance_copy = dev;
    // init bmx 160
    imu->bmi.id = BMI160_I2C_ADDR;
    imu->bmi.interface = BMI160_I2C_INTF;
    imu->bmi.read = bmx_i2c_read;
    imu->bmi.write = bmx_i2c_write;
    imu->bmi.delay_ms = bmx_delay_ms;

    /* Configure the BMM150 device structure by mapping user_aux_read and user_aux_write */
    imu->bmm.read = bmm_i2c_read;
    imu->bmm.write = bmm_i2c_write;
    imu->bmm.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
    imu->bmm.delay_ms = bmx_delay_ms;
    imu->bmm.intf = BMM150_I2C_INTF;  //test

    int8_t rslt = BMI160_OK;
    rslt = bmi160_init(&imu->bmi);

    /* Configure device structure for auxiliary sensor parameter */
    imu->bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE; // auxiliary sensor enable
    imu->bmi.aux_cfg.aux_i2c_addr = imu->bmm.dev_id; // auxiliary sensor address
    imu->bmi.aux_cfg.manual_enable = BMI160_ENABLE; // setup mode enable
    imu->bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3;// burst read of 8 byte

    /* Initialize the auxiliary sensor interface */
    rslt = bmi160_aux_init(&imu->bmi);

    /* Initialising the bmm150 sensor */
    rslt = bmm150_init(&imu->bmm);

    /* Select the Output data rate, range of accelerometer sensor */
    imu->bmi.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
    imu->bmi.accel_cfg.range = BMI160_ACCEL_RANGE_16G ;
    imu->bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    /* Select the power mode of accelerometer sensor */
    imu->bmi.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    imu->bmi.gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;
    imu->bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    imu->bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    /* Select the power mode of Gyroscope sensor */
    imu->bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&imu->bmi);

    /* Auxiliary sensor is enabled and can be accessed from this point */

    /* Configure the desired settings in auxiliary BMM150 sensor
     * using the bmm150 APIs */

    /* Set the power mode and preset mode to enable Mag data sampling */
    imu->bmm.settings.pwr_mode = BMM150_NORMAL_MODE;
    //imu->bmm.settings.pwr_mode = BMM150_FORCED_MODE; // why this is not working.
    rslt = bmm150_set_op_mode(&imu->bmm);

    imu->bmm.settings.preset_mode= BMM150_PRESETMODE_LOWPOWER;//BMM150_PRESETMODE_LOWPOWER;
    rslt = bmm150_set_presetmode(&imu->bmm);

    /* In BMM150 Mag data starts from register address 0x42 */
    uint8_t aux_addr = BMM150_DATA_X_LSB;

    /* Configure the Auxiliary sensor either in auto/manual modes and set the
    polling frequency for the Auxiliary interface */
    imu->bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_12_5HZ; /* 8 polling rate in 100 Hz*/ //6 for 25
    rslt = bmi160_config_aux_mode(&imu->bmi);

    /* Set the auxiliary sensor to auto mode */
    rslt = bmi160_set_aux_auto_mode(&aux_addr, &imu->bmi);

//    /* Reading data from BMI160 data registers */
//    rslt = bmi160_read_aux_data_auto_mode(imu->mag_data, &imu->bmi);

    // marked init done then return the instance.
    imu->super.is_inited = 1;

    return 0;
}

imu_dev_t * bmx160_register(rt_device_t bus)
{
    if(bus == NULL)
        return NULL;

    bmx_imu_dev_t * imu = rt_malloc(sizeof(bmx_imu_dev_t));
    if(imu == NULL)
        return NULL;

    // save a local copy for RW use.
    i2c_bus = bus;

    // set operation interfance
    imu->super.bus_dev = bus;
    imu->super.read_raw = bmx160_read_raw;
    imu->super.detect = NULL;
    imu->super.init = bmx160_init;

    imu->super.is_inited = 0;
    return (imu_dev_t*) imu;
}
