/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-07     majia       the first version
 */
#ifndef __DATA_POOL_H__
#define __DATA_POOL_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <rtthread.h>


typedef struct _triaxis_t
{
    float x;
    float y;
    float z;
} triaxis_t;

typedef struct _triaxis_int_t
{
    int16_t x;
    int16_t y;
    int16_t z;
} triaxis_int_t;


typedef struct _sensor_info_t
{
    float update_rate;
    float update_timestamp;
    uint32_t count;
}sensor_info_t;


typedef struct _gyro_t {
    sensor_info_t info;
    triaxis_int_t raw;
    triaxis_t unit;
    float temperature;
} gyro_t;
extern gyro_t gyro;

typedef struct _acc_t {
    sensor_info_t info;
    triaxis_int_t raw;
    triaxis_t unit;
    float temperature;
} acc_t;
extern acc_t acc;

typedef struct _mag_t {
    sensor_info_t info;
    triaxis_int_t raw;
    triaxis_t unit;
    float temperature;
} mag_t;
extern mag_t mag;

typedef struct _orientation_t {
    sensor_info_t info;
    triaxis_t euler;
    float q[4];
} orientation_t;
extern orientation_t orientation;

typedef struct _anemometer_t {
    sensor_info_t info;
    float course;
    float speed;
    float speed30savg;
    float speed30smax;
    float soundspeed;
    int err_code;
} anemometer_t;
extern anemometer_t anemometer;

typedef struct _air_info_t {
    sensor_info_t info;
    float pressure;
    float humidity;
    float temperature;
} air_info_t;
extern air_info_t air_info;

typedef struct _light_info_t {
    sensor_info_t info;
    int R;
    int G;
    int B;
    int IR;
    int ALS;  // all light sensing?
}light_info_t;
extern light_info_t light_info;

typedef struct _rain_t {
    sensor_info_t info;
    int level;
    int raw;
    float var;
}rain_t;
extern rain_t rain;

typedef struct _lightning_t
{
    sensor_info_t info;
    float distance;
}lightning_t;
extern lightning_t lightning;

typedef struct _gnss_t
{
    sensor_info_t info;
    float latitude;     // a double can be more accurate
    float longitude;
    float speed;        // m/s
    float altitude;     // sea altitude
    float course;
    int   num_sat;      // number of satellites
    bool is_fixed;         // location is fixed
}gnss_t;
extern gnss_t gnss;

typedef struct _sys_t
{
    sensor_info_t info;
    float bat_voltage;
    float sys_voltage;
    float mcu_temp;
}sys_t;
extern sys_t sys;


void data_updated(sensor_info_t *info);


#define DATA_NAME_MAX_LEN   (16)

/*  names and getters for availalbe recording and data */
extern float (*get_data[])();
extern int (*print_data[])(char* );
extern const char data_name[][DATA_NAME_MAX_LEN];

extern const int EXPORT_DATA_SIZE;

/* get data getter index by its name*/
uint32_t get_data_index(char* name);

/* get a list of names return the order of their getters
 * This will use strtok which will destroy the input strings. */
uint32_t get_data_orders(char* names, char* delim, uint16_t *orders, uint16_t max_order_len);


#endif /* __DATA_POOL_H__ */
