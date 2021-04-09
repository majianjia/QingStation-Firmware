/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-06     majia       the first version
 */
#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "cjson/cjson.h"

#define MAX_NAME_LEN    (16)
#define MAX_HEADER_LEN  (512)
#define MAX_PATH_LEN    (32)

typedef struct sensor_config
{
    void *next;                     // sensor list.
    bool is_enable;
    char name[MAX_NAME_LEN];        // part number
    char interface_name[MAX_NAME_LEN];   // interface device name
    uint8_t addr;                   // i2c address
    uint32_t data_period;           // in ms
    uint32_t oversampling;          // num of over sampling
    void *user_data;
    void (*create_json)(struct sensor_config*, cJSON*);
    void (*load_json)(struct sensor_config*, cJSON*);
} sensor_config_t;

typedef struct uart_config
{
    bool is_enable;
    int bitrate;                // target bit rate. for GPS, it can auto detect the bit rate.
    char device[MAX_NAME_LEN];  // the type of device (GPS, AT client, AT server, log) that connected to this uart.
} uart_config_t;

typedef struct record_config
{
    // recording
    bool is_enable;
    bool is_split_file;
    char root_path[MAX_PATH_LEN]; // the root of recording file. no longer than 31 chars.
    char header[MAX_HEADER_LEN];    // the header of recording, it also control what data will be recorded.
    uint32_t period;            // millisecond
} record_config_t;

typedef struct log_config
{
    // recording
    bool is_enable;
    bool is_repeat_header; // add header before data, for better reading.
    char header[MAX_HEADER_LEN];    // the header of recording, it also control what data will be recorded.
    uint32_t period;            // millisecond
} log_config_t;

// configuration of the station
// align with json config file
typedef struct system_config {
    int32_t version;
    bool is_valid;      // whether it is valid to use.
    sensor_config_t *sensors;

    uart_config_t uart1;
    uart_config_t uart2;

    log_config_t log;        // logging to uart.
    record_config_t record;  // record to file.

    // debugging
    bool ane_record_pulse;

} system_config_t;


typedef struct _bmx160_config_t
{
    // magnetometer calibration value
    int32_t mag_offset_x;
    int32_t mag_offset_y;
    int32_t mag_offset_z;
    float mag_scale_x;
    float mag_scale_y;
    float mag_scale_z;
} bmx160_config_t;

typedef struct _anemometer_config_t
{
    float height;   // height of reflective plate to transducer.
    float pitch;    // pitch size between transducer.
    float pulse_offset[4]; // time offset for each channel.
    bool is_dump_error; // save error
} anemometer_config_t;

extern system_config_t system_config;
bool is_system_cfg_valid();
int save_system_cfg_to_file();
sensor_config_t* get_sensor_config(char *name);
sensor_config_t* get_sensor_config_wait(char *name);

#endif /* __CONFIGURATION_H__ */
