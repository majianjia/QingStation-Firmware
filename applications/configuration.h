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
#define MAX_HEADER_LEN  (256)
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


typedef struct record_config
{
    // recording
    bool is_enable;
    bool is_split_file;
    char data_path[MAX_PATH_LEN]; // the root of recording file. no longer than 31 chars.
    char header[MAX_HEADER_LEN];    // the header of recording, it also control what data will be recorded.
    uint32_t period;            // millisecond
    uint32_t max_file_size;     // when is_split is enable.
} record_config_t;

typedef struct log_config
{
    // recording
    bool is_enable;
    bool is_repeat_header; // add header before data, for better reading.
    char header[MAX_HEADER_LEN];    // the header of recording, it also control what data will be recorded.
    uint32_t period;            // millisecond
} log_config_t;

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

typedef struct _rain_config_t
{
    // intensity of rain fall. threshold of variance.
    int light;
    int moderate;
    int heavy;
    int violent;
} rain_config_t;


typedef struct _mqtt_config_t
{
    char interface[12];      // name of uart device
    int baudrate;           // baudrate for the uart.
    char module[12];    // type of module,
    char wifi_ssid[16];      // if wifi, the ssid
    char wifi_password[32];  // if wifi, password
    char mqtt_username[16];
    char mqtt_password[32];
    char pub_data[MAX_HEADER_LEN];  // select the data to publish
    char topic_prefix[32];          // prefix of the topic (e.g. "qing/" will make data topics to be a sub topic under 'qing')
    char uri[64];       // uri
    int port;           // port
    int period;         // update period in ms
    bool is_enable;
} mqtt_config_t;

typedef struct _gnss_config_t
{
    char interface[12];      // name of uart device
    int baudrate;           // baudrate for the uart.
    int period;         // update period in ms 1,2,4,5,10 HZ = 1000/500/250/200/100
    bool is_enable;
} gnss_config_t;

typedef struct _ntp_config_t
{
    char server[3][32];      // support 3 servers
    int startup_delay;       // start up delay
    int update_period;       // update time in sec
    bool is_enable;
} ntp_config_t;


// configuration of the station
// align with json config file
typedef struct system_config {
    int32_t version;
    bool is_valid;      // whether it is valid to use.
    sensor_config_t *sensors;

    log_config_t log;        // logging to uart.
    record_config_t record;  // record to file.

    mqtt_config_t mqtt;
    gnss_config_t gnss;
    ntp_config_t ntp;
} system_config_t;



extern system_config_t system_config;
bool is_system_cfg_valid();
int save_system_cfg_to_file();
sensor_config_t* get_sensor_config(char *name);
sensor_config_t* get_sensor_config_wait(char *name);

#endif /* __CONFIGURATION_H__ */
