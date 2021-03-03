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

#ifndef __QINGSTATION_IMU_H__
#define __QINGSTATION_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif


typedef struct _imu_dev_t
{
    rt_device_t * bus_dev;
    union{
        int16_t data[10];
        struct {
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
            int16_t mag_x;
            int16_t mag_y;
            int16_t mag_z;
            int16_t temperature;
        };
    } raw;

    struct {
        float acc_x;
        float acc_y;
        float acc_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float mag_x;
        float mag_y;
        float mag_z;
        float temperature;
    } unit;

    int (*init)(struct _imu_dev_t *dev);
    int (*read_raw)(struct _imu_dev_t *dev);
    int (*detect)(struct _imu_dev_t * dev);
    char is_inited;
} imu_dev_t;


#ifdef __cplusplus
}
#endif

#endif /* __QINGSTATION_IMU_H__ */
