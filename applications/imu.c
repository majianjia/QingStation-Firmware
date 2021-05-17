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
#include <rtdevice.h>
#include <board.h>
#include "drv_anemometer.h"
#include "recorder.h"
#include "drv_bmx160.h"
#include "data_pool.h"
#include "configuration.h"
#include "MadgwickAHRS.h"
#include <math.h>

#define DBG_TAG "imu"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

void quaternion_to_eular(float quat[], triaxis_t *euler)
{
    float q0, q1, q2, q3;
    q0 = quat[0];
    q1 = quat[1];
    q2 = quat[2];
    q3 = quat[3];
    euler->y = (float)(asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f); // pitch
    euler->x = (float)(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f); // roll
    euler->z = (float)(atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f);
}

typedef struct _mag_calibration_t {
    triaxis_int_t offset;
    triaxis_t scale;

    triaxis_int_t max;
    triaxis_int_t min;
} mag_calibration_t;

void mag_max_min(triaxis_int_t * newdata, mag_calibration_t* calib )
{
    calib->max.x = MAX(calib->max.x, newdata->x);
    calib->max.y = MAX(calib->max.y, newdata->y);
    calib->max.z = MAX(calib->max.z, newdata->z);
    calib->min.x = MIN(calib->min.x, newdata->x);
    calib->min.y = MIN(calib->min.y, newdata->y);
    calib->min.z = MIN(calib->min.z, newdata->z);
}

void thread_imu(void* parameters)
{
    rt_device_t i2c_bus;
    sensor_config_t * cfg;
    bmx160_config_t * user_cfg;
    rt_tick_t period = 0;
    mag_calibration_t mag_calib = {0};

    // wait and load the configuration
    do{
        rt_thread_delay(100);
        cfg = get_sensor_config("BMX160");
    }while(cfg == NULL && !is_system_cfg_valid());
    // device specific data.
    user_cfg = cfg->user_data;

    i2c_bus = rt_device_find(cfg->interface_name);
    if(!i2c_bus)
    {
        LOG_E("cannot find %s bus for IMU.", cfg->interface_name);
        return;
    }

    // set up madgwick.
    MadgwickAHRS_Init(1000 / cfg->data_period, 0.1f);

    // register and init the bmx160
    imu_dev_t *imu = bmx160_register(i2c_bus);
    imu->init(imu);

    //
    period = cfg->data_period;

    // load mag data
    // offset = (max+min)/2
    // scale = x/y. or x/z
//    mag_calib.offset.x = 9;
//    mag_calib.offset.y = -10;
//    mag_calib.offset.z = 15;
//    mag_calib.scale.x = 1; //80
//    mag_calib.scale.y = 0.93f; //86
//    mag_calib.scale.z = 0.987f; //81

    // load calibration from user's configuration file.
    mag_calib.offset.x = user_cfg->mag_offset_x;
    mag_calib.offset.y = user_cfg->mag_offset_y;
    mag_calib.offset.z = user_cfg->mag_offset_z;
    mag_calib.scale.x = user_cfg->mag_scale_x;
    mag_calib.scale.y = user_cfg->mag_scale_x;
    mag_calib.scale.z = user_cfg->mag_scale_x;

    while(1)
    {
        // add small delay in case of over lapping.
        rt_thread_mdelay(period/8);
        rt_thread_mdelay(period - rt_tick_get() % period);
        if(!cfg->is_enable)
            continue;

        imu->read_raw(imu);

        // mag calibration
        triaxis_int_t new_mag = {imu->raw.mag_x, imu->raw.mag_y, imu->raw.mag_z};
        mag_max_min(&new_mag, &mag_calib);

        // temporary
        imu->unit.gyro_x = imu->raw.gyro_x / 16.4f;
        imu->unit.gyro_y = imu->raw.gyro_y / 16.4f;
        imu->unit.gyro_z = imu->raw.gyro_z / 16.4f;
        imu->unit.acc_x = imu->raw.acc_x / 2048.f;
        imu->unit.acc_y = imu->raw.acc_y / 2048.f;
        imu->unit.acc_z = imu->raw.acc_z / 2048.f;
        imu->unit.mag_x = (imu->raw.mag_x - mag_calib.offset.x) * mag_calib.scale.x;
        imu->unit.mag_y = (imu->raw.mag_y - mag_calib.offset.y) * mag_calib.scale.y;
        imu->unit.mag_z = (imu->raw.mag_z - mag_calib.offset.z) * mag_calib.scale.z;
        imu->unit.temperature = imu->raw.temperature * 0.002 + 23;

        // write to global data pool
        gyro.raw.x = imu->raw.gyro_x;
        gyro.raw.y = imu->raw.gyro_y;
        gyro.raw.z = imu->raw.gyro_z;
        gyro.unit.x = imu->unit.gyro_x;
        gyro.unit.y = imu->unit.gyro_y;
        gyro.unit.z = imu->unit.gyro_z;
        gyro.temperature = imu->unit.temperature;
        data_updated(&gyro.info);

        acc.raw.x = imu->raw.acc_x;
        acc.raw.y = imu->raw.acc_y;
        acc.raw.z = imu->raw.acc_z;
        acc.unit.x = imu->unit.acc_x;
        acc.unit.y = imu->unit.acc_y;
        acc.unit.z = imu->unit.acc_z;
        acc.temperature = imu->unit.temperature;
        data_updated(&acc.info);

        mag.raw.x = imu->raw.mag_x;
        mag.raw.y = imu->raw.mag_y;
        mag.raw.z = imu->raw.mag_z;
        mag.unit.x = imu->unit.mag_x;
        mag.unit.y = imu->unit.mag_y;
        mag.unit.z = imu->unit.mag_z;
        mag.temperature = imu->unit.temperature;
        data_updated(&mag.info);

        // AHRS
        #define PI 3.1415926f
        #define RAD (PI/180)
        MadgwickAHRSupdate(gyro.unit.x*RAD, gyro.unit.y*RAD, gyro.unit.z*RAD,
                acc.unit.x, acc.unit.y, acc.unit.z,
                //0,0,0);
                mag.unit.x, mag.unit.y, mag.unit.z);

        orientation.q[0] = q0;
        orientation.q[1] = q1;
        orientation.q[2] = q2;
        orientation.q[3] = q3;
        quaternion_to_eular(orientation.q, &orientation.euler);
        data_updated(&orientation.info);

        //printf("%d, %d, %d\n", (int)(orientation.euler.x*10), (int)(orientation.euler.y*10), (int)(orientation.euler.z*10));
        //LOG_I("%d, %d, %d", (int)gyro.unit.x , (int)gyro.unit.y , (int)gyro.unit.z );
        //LOG_I("%d, %d, %d", (int)mag.raw.x , (int)mag.raw.y , (int)mag.raw.z );
    }
}

int thread_imu_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("imu", thread_imu, RT_NULL, 1024+128, 15, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_imu_init);// small noise
