/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-06     Jianjia Ma       the first version
 */

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <data_pool.h>
#include <rtdevice.h>
#include "configuration.h"
#include <string.h>
#include <stdlib.h>
#include <dfs.h>
#include <dfs_posix.h>
#include <stdbool.h>

#include "drv_anemometer.h"

#include "cjson/cjson.h"

#define DBG_TAG "config"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


void add_sensor(sensor_config_t* list, sensor_config_t* new_sensor)
{
    if(new_sensor)
    {
        sensor_config_t* p = list;
        while(p){
            if(p == new_sensor) return;
            p = p->next;
        }

        while(list->next != NULL)
            list = list->next;
        list->next = new_sensor;
    }
}

sensor_config_t* find_sensor_config(sensor_config_t* list, char* name)
{
    if(list)
    {
        do{
            if(strcmp(list->name, name) == 0)
                return list;
            list = list->next;
        }while(list != NULL);
    }
    return NULL;
}

// this doesnt work if the first sensor is the one to delete.
void delete_sensor(sensor_config_t* list, char* name)
{
    sensor_config_t* previous = list;
    if(strcmp(list->name, name) == 0)
        return;
    if(list)
    {
        do{
            if(strcmp(list->name, name) == 0)
            {
                previous->next = list->next;
                free(list);
                return;
            }
            previous = list;
            list = list->next;
        }while(list != NULL);
    }
}

#include "cjson/cjson.h"


#define DEFAULT_SENSOR_CONFIG { \
    .next = NULL,               \
    .is_enable = true,          \
    .name = "",                 \
    .interface_name = "i2c2",   \
    .addr = 0,                  \
    .data_period = 1000,       \
    .oversampling = 1          \
}

/* it control the system, should be a single instance*/
system_config_t system_config;

sensor_config_t* get_sensor_config_wait(char *name)
{
    sensor_config_t* sensor = system_config.sensors;

    while(!system_config.is_valid)
        rt_thread_delay(10);

    if(sensor == NULL)
        return NULL;
    for(;sensor != NULL; sensor=sensor->next)
    {
        if(! strcasecmp(name, sensor->name))
            return sensor;
    }
    return NULL;
}


sensor_config_t* get_sensor_config(char *name)
{
    sensor_config_t* sensor = system_config.sensors;
    if(sensor == NULL)
        return NULL;
    for(;sensor != NULL; sensor=sensor->next)
    {
        if(! strcasecmp(name, sensor->name))
            return sensor;
    }
    return NULL;
}

sensor_config_t* new_sensor(char* name, char* interface)
{
    sensor_config_t def_sensor_cfg = DEFAULT_SENSOR_CONFIG;
    sensor_config_t* s;
    s = get_sensor_config(name); // search the sensor list first
    if(!s){
        s = malloc(sizeof(sensor_config_t));
        if(!s)
            return NULL;
        memset(s, 0, sizeof(sensor_config_t));
        memcpy(s, &def_sensor_cfg, sizeof(sensor_config_t));
        strncpy(s->name, name, MAX_NAME_LEN);
        strncpy(s->interface_name, interface, MAX_NAME_LEN);
    }
    return s;
}

void bmx_create_json(sensor_config_t* config, cJSON* json)
{
    bmx160_config_t * bmx;
    if(!config->user_data)
        return;
    bmx = config->user_data;

    cJSON * temp = cJSON_CreateObject();
    if(!temp) return;
    if(!cJSON_AddItemToObject(json, "settings", temp)) return;

    if(!cJSON_AddNumberToObject(temp, "mag_offset_x", bmx->mag_offset_x)) return;
    if(!cJSON_AddNumberToObject(temp, "mag_offset_y", bmx->mag_offset_y)) return;
    if(!cJSON_AddNumberToObject(temp, "mag_offset_z", bmx->mag_offset_z)) return;
    if(!cJSON_AddNumberToObject(temp, "mag_scale_x", bmx->mag_scale_x)) return;
    if(!cJSON_AddNumberToObject(temp, "mag_scale_y", bmx->mag_scale_y)) return;
    if(!cJSON_AddNumberToObject(temp, "mag_scale_z", bmx->mag_scale_z)) return;
}

void bmx_load_json(sensor_config_t* config, cJSON* json)
{
    bmx160_config_t * bmx;
    if(!config->user_data){
        config->user_data = malloc(sizeof(bmx160_config_t));
        memset(config->user_data, 0, sizeof(bmx160_config_t));
    }
    bmx = config->user_data;

    cJSON * temp;
    json = cJSON_GetObjectItem(json, "settings");
    if(!cJSON_IsObject(json))
        return;
    temp = cJSON_GetObjectItem(json, "mag_offset_x");
    if(cJSON_IsNumber(temp))
        bmx->mag_offset_x = temp->valueint;
    temp = cJSON_GetObjectItem(json, "mag_offset_y");
    if(cJSON_IsNumber(temp))
        bmx->mag_offset_y = temp->valueint;
    temp = cJSON_GetObjectItem(json, "mag_offset_z");
    if(cJSON_IsNumber(temp))
        bmx->mag_offset_z = temp->valueint;
    temp = cJSON_GetObjectItem(json, "mag_scale_x");
    if(cJSON_IsNumber(temp))
        bmx->mag_scale_x = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "mag_scale_y");
    if(cJSON_IsNumber(temp))
        bmx->mag_scale_y = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "mag_scale_z");
    if(cJSON_IsNumber(temp))
        bmx->mag_scale_z = temp->valuedouble;
}

void anemo_create_json(sensor_config_t* config, cJSON* json)
{
    anemometer_config_t * ane;
    if(!config->user_data)
        return;
    ane = config->user_data;

    cJSON * temp = cJSON_CreateObject();
    if(!temp) return;
    if(!cJSON_AddItemToObject(json, "settings", temp)) return;
    if(!cJSON_AddNumberToObject(temp, "height", ane->height)) return;
    if(!cJSON_AddNumberToObject(temp, "pitch", ane->pitch)) return;
    if(!cJSON_AddNumberToObject(temp, "offset_n", ane->pulse_offset[NORTH])) return;
    if(!cJSON_AddNumberToObject(temp, "offset_e", ane->pulse_offset[EAST])) return;
    if(!cJSON_AddNumberToObject(temp, "offset_s", ane->pulse_offset[SOUTH])) return;
    if(!cJSON_AddNumberToObject(temp, "offset_w", ane->pulse_offset[WEST])) return;
    if(!cJSON_AddBoolToObject(temp, "is_dump_error", ane->is_dump_error)) return;
}

void anemo_load_json(sensor_config_t* config, cJSON* json)
{
    anemometer_config_t * ane;
    if(!config->user_data){
        config->user_data = malloc(sizeof(anemometer_config_t));
        memset(config->user_data, 0, sizeof(anemometer_config_t));
    }
    ane = config->user_data;

    cJSON * temp;
    json = cJSON_GetObjectItem(json, "settings");
    if(!cJSON_IsObject(json))
        return;
    temp = cJSON_GetObjectItem(json, "height");
    if(cJSON_IsNumber(temp))
        ane->height = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "pitch");
    if(cJSON_IsNumber(temp))
        ane->pitch = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "offset_n");
    if(cJSON_IsNumber(temp))
        ane->pulse_offset[NORTH] = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "offset_e");
    if(cJSON_IsNumber(temp))
        ane->pulse_offset[EAST] = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "offset_s");
    if(cJSON_IsNumber(temp))
        ane->pulse_offset[SOUTH] = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "offset_w");
    if(cJSON_IsNumber(temp))
        ane->pulse_offset[WEST] = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "is_dump_error");
    if(cJSON_IsBool(temp))
        ane->is_dump_error = temp->valueint;
}


void rain_create_json(sensor_config_t* config, cJSON* json)
{
    rain_config_t * rain;
    if(!config->user_data)
        return;
    rain = config->user_data;

    cJSON * temp = cJSON_CreateObject();
    if(!temp) return;
    if(!cJSON_AddItemToObject(json, "settings", temp)) return;

    if(!cJSON_AddNumberToObject(temp, "light", rain->light)) return;
    if(!cJSON_AddNumberToObject(temp, "moderate", rain->moderate)) return;
    if(!cJSON_AddNumberToObject(temp, "heavy", rain->heavy)) return;
    if(!cJSON_AddNumberToObject(temp, "violent", rain->violent)) return;
}

void rain_load_json(sensor_config_t* config, cJSON* json)
{
    rain_config_t * rain;
    if(!config->user_data){
        config->user_data = malloc(sizeof(bmx160_config_t));
        memset(config->user_data, 0, sizeof(bmx160_config_t));
    }
    rain = config->user_data;

    cJSON * temp;
    json = cJSON_GetObjectItem(json, "settings");
    if(!cJSON_IsObject(json))
        return;
    temp = cJSON_GetObjectItem(json, "light");
    if(cJSON_IsNumber(temp))
        rain->light = temp->valueint;
    temp = cJSON_GetObjectItem(json, "moderate");
    if(cJSON_IsNumber(temp))
        rain->moderate = temp->valueint;
    temp = cJSON_GetObjectItem(json, "heavy");
    if(cJSON_IsNumber(temp))
        rain->heavy = temp->valueint;
    temp = cJSON_GetObjectItem(json, "violent");
    if(cJSON_IsNumber(temp))
        rain->violent = temp->valueint;
}


void load_default_config(system_config_t* sys)
{
    sensor_config_t* s;
    sys->version = 100;

    // digital sensor.
    s = new_sensor("BMX160", "i2c2");
    if(s == NULL) return;
    bmx160_config_t* bmx_cfg = malloc(sizeof(bmx160_config_t));
    if(bmx_cfg == NULL) return;
    memset(bmx_cfg, 0, sizeof(bmx160_config_t));
    bmx_cfg->mag_offset_x = 9;
    bmx_cfg->mag_offset_y = -10;
    bmx_cfg->mag_offset_z = 15;
    bmx_cfg->mag_scale_x = 1;
    bmx_cfg->mag_scale_y = 0.93f;
    bmx_cfg->mag_scale_z = 0.987f;
    s->user_data = bmx_cfg;
    s->create_json = bmx_create_json;
    s->load_json = bmx_load_json;
    s->data_period = 1000/10; // 10Hz
    sys->sensors = s;       // the first cannot use add_sensor().

    s = new_sensor("APDS9250", "i2c2");
    if(s == NULL) return;
    add_sensor(sys->sensors, s);

    s = new_sensor("BME280", "i2c2");
    if(s == NULL) return;
    add_sensor(sys->sensors, s);

    s = new_sensor("AS3935", "i2c2");
    if(s == NULL) return;
    add_sensor(sys->sensors, s);

    s = new_sensor("MP34DT", "n/a");
    if(s == NULL) return;
    add_sensor(sys->sensors, s);

    // analog, channel is fixed on board. setting doesnt matter.
    s = new_sensor("Anemometer", "adc_ch5");
    if(s == NULL) return;
    s->data_period = 1000;
    s->oversampling = 2;
    anemometer_config_t* ane_cfg = malloc(sizeof(anemometer_config_t));
    if(ane_cfg == NULL) return;
    memset(ane_cfg, 0, sizeof(anemometer_config_t));
    ane_cfg->height = 0.05f;     // default, normally set to 5 cm
    ane_cfg->pitch = 0.04f;      // defualt pitch, based on the 3D file
    ane_cfg->pulse_offset[0] = 0;
    ane_cfg->pulse_offset[1] = 0;
    ane_cfg->pulse_offset[2] = 0;
    ane_cfg->pulse_offset[3] = 0;
    ane_cfg->is_dump_error = false; // dump adc data when error
    s->user_data = ane_cfg;
    s->create_json = anemo_create_json;
    s->load_json = anemo_load_json;
    add_sensor(sys->sensors, s);

    s = new_sensor("Rain", "adc_ch6");
    if(s == NULL) return;
    s->oversampling = 10;
    add_sensor(sys->sensors, s);
    rain_config_t* rain_cfg = malloc(sizeof(rain_config_t));
    if(rain_cfg == NULL) return;
    memset(rain_cfg, 0, sizeof(rain_config_t));
    rain_cfg->light     = 10;
    rain_cfg->moderate  = 50;
    rain_cfg->heavy     = 100;
    rain_cfg->violent   = 200;
    s->user_data = rain_cfg;
    s->create_json = rain_create_json;
    s->load_json = rain_load_json;
    add_sensor(sys->sensors, s);

    // recorder
    sys->record.is_enable = true;
    sys->record.is_split_file = true;
    strcpy(sys->record.header, "");
    sys->record.period = 1000;
    sys->record.max_file_size = 4096*1024; // 4MB
    strcpy(sys->record.data_path, "/");

    // log
    sys->log.is_enable = false;
    sys->log.is_repeat_header = true;
    strcpy(sys->log.header,"air_temp,humidity,pressure,bat_volt,als,gnss_sat,gnss_lat,gnss_long,wind_speed");
    sys->log.period = 10000;

    // mqtt
    sys->mqtt.is_enable = true;
    sys->mqtt.period = 1;
    sys->mqtt.baudrate = 57600;
    strcpy(sys->mqtt.interface,"uart2");
    //strcpy(sys->mqtt.module,"esp8266");
    strcpy(sys->mqtt.module,"esp32");
    //strcpy(sys->mqtt.module,"sim800c");
    strcpy(sys->mqtt.wifi_ssid,"");
    strcpy(sys->mqtt.wifi_password,"");
    strcpy(sys->mqtt.mqtt_username,"");
    strcpy(sys->mqtt.mqtt_password,"");
    strcpy(sys->mqtt.pub_data, ""); // empty means every data is published.
    strcpy(sys->mqtt.topic_prefix, ""); // allows you to add a super topic before data.
    strcpy(sys->mqtt.uri, "");
    sys->mqtt.port = 1883;

    // gnss
    sys->gnss.is_enable = true;
    sys->gnss.period = 1000;
    sys->gnss.baudrate = 38400;
    strcpy(sys->gnss.interface, "lpuart1");

    // ntp
    sys->ntp.is_enable = true;
    sys->ntp.startup_delay = 60;
    sys->ntp.update_period = 3600;
    memset(sys->ntp.server, 0, sizeof(sys->ntp.server));
    strncpy(sys->ntp.server[0], "uk.pool.ntp.org", sizeof(sys->ntp.server[0]));
    strncpy(sys->ntp.server[1], "us.pool.ntp.org", sizeof(sys->ntp.server[1]));
    strncpy(sys->ntp.server[2], "cn.pool.ntp.org", sizeof(sys->ntp.server[2]));
}


int load_config_from_json(system_config_t* sys, char* json_strings)
{
    cJSON *config = NULL;
    cJSON *sensors = NULL;
    cJSON *log = NULL;
    cJSON *record = NULL;
    cJSON *version = NULL;
    cJSON *mqtt = NULL;
    cJSON *gnss = NULL;
    cJSON *ntp = NULL;

    int status = 0;
    config = cJSON_Parse(json_strings);
    if (config == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            printf("Error before: %s\n", error_ptr);
        }
        status = 0;
        goto end;
    }

    // version
    version = cJSON_GetObjectItemCaseSensitive(config, "version");
    if (cJSON_IsNumber(version))
    {
        sys->version = version->valueint;
        printf("version %d\n", version->valueint);
    }

    // log
    log = cJSON_GetObjectItemCaseSensitive(config, "log");
    if (cJSON_IsObject(log))
    {
        cJSON * temp;
        temp = cJSON_GetObjectItem(log, "enable");
        if(cJSON_IsBool(temp))
            sys->log.is_enable = temp->valueint;

        temp = cJSON_GetObjectItem(log, "repeat_header");
        if(cJSON_IsBool(temp))
            sys->log.is_repeat_header = temp->valueint;

        temp = cJSON_GetObjectItem(log, "period");
        if(cJSON_IsNumber(temp))
            sys->log.period = temp->valueint;

        temp = cJSON_GetObjectItem(log, "header");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->log.header, temp->valuestring, MAX_HEADER_LEN);
    }

    // record
    record = cJSON_GetObjectItemCaseSensitive(config, "record");
    if (cJSON_IsObject(record))
    {
        cJSON * temp;
        temp = cJSON_GetObjectItem(record, "enable");
        if(cJSON_IsBool(temp))
            sys->record.is_enable = temp->valueint;

        temp = cJSON_GetObjectItem(record, "split_file");
        if(cJSON_IsBool(temp))
            sys->record.is_split_file = temp->valueint;

        temp = cJSON_GetObjectItem(record, "period");
        if(cJSON_IsNumber(temp))
            sys->record.period = temp->valueint;

        temp = cJSON_GetObjectItem(record, "max_file_size");
        if(cJSON_IsNumber(temp))
            sys->record.max_file_size = temp->valueint;

        temp = cJSON_GetObjectItem(record, "header");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->record.header, temp->valuestring, MAX_HEADER_LEN);

        temp = cJSON_GetObjectItem(record, "data_path");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->record.data_path, temp->valuestring, sizeof(sys->record.data_path));
    }

    // mqtt
    mqtt = cJSON_GetObjectItemCaseSensitive(config, "mqtt");
    if (cJSON_IsObject(mqtt))
    {
        cJSON * temp;
        temp = cJSON_GetObjectItem(mqtt, "enable");
        if(cJSON_IsBool(temp))
            sys->mqtt.is_enable = temp->valueint;

        temp = cJSON_GetObjectItem(mqtt, "period");
        if(cJSON_IsNumber(temp))
            sys->mqtt.period = temp->valueint;

        temp = cJSON_GetObjectItem(mqtt, "baudrate");
        if(cJSON_IsNumber(temp))
            sys->mqtt.baudrate = temp->valueint;

        temp = cJSON_GetObjectItem(mqtt, "interface");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.interface, temp->valuestring, sizeof(sys->mqtt.interface));

        temp = cJSON_GetObjectItem(mqtt, "module");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.module, temp->valuestring, sizeof(sys->mqtt.module));

        temp = cJSON_GetObjectItem(mqtt, "wifi_ssid");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.wifi_ssid, temp->valuestring, sizeof(sys->mqtt.wifi_ssid));

        temp = cJSON_GetObjectItem(mqtt, "wifi_password");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.wifi_password, temp->valuestring, sizeof(sys->mqtt.wifi_password));

        temp = cJSON_GetObjectItem(mqtt, "mqtt_username");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.mqtt_username, temp->valuestring, sizeof(sys->mqtt.mqtt_username));

        temp = cJSON_GetObjectItem(mqtt, "mqtt_password");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.mqtt_password, temp->valuestring, sizeof(sys->mqtt.mqtt_password));

        temp = cJSON_GetObjectItem(mqtt, "uri");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.uri, temp->valuestring, sizeof(sys->mqtt.uri));

        temp = cJSON_GetObjectItem(mqtt, "port");
        if(cJSON_IsNumber(temp))
            sys->mqtt.port = temp->valueint;

        temp = cJSON_GetObjectItem(mqtt, "pub_data");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.pub_data, temp->valuestring, sizeof(sys->mqtt.pub_data));

        temp = cJSON_GetObjectItem(mqtt, "topic_prefix");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->mqtt.topic_prefix, temp->valuestring, sizeof(sys->mqtt.topic_prefix));
    }

    // gnss
    gnss = cJSON_GetObjectItemCaseSensitive(config, "gnss");
    if (cJSON_IsObject(gnss))
    {
        cJSON * temp;
        temp = cJSON_GetObjectItem(gnss, "enable");
        if(cJSON_IsBool(temp))
            sys->gnss.is_enable = temp->valueint;

        temp = cJSON_GetObjectItem(gnss, "period");
        if(cJSON_IsNumber(temp))
            sys->gnss.period = temp->valueint;

        temp = cJSON_GetObjectItem(gnss, "baudrate");
        if(cJSON_IsNumber(temp))
            sys->gnss.baudrate = temp->valueint;

        temp = cJSON_GetObjectItem(gnss, "interface");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->gnss.interface, temp->valuestring, sizeof(sys->gnss.interface));
    }

    // ntp
    ntp = cJSON_GetObjectItemCaseSensitive(config, "ntp");
    if (cJSON_IsObject(ntp))
    {
        cJSON * temp;
        temp = cJSON_GetObjectItem(ntp, "enable");
        if(cJSON_IsBool(temp))
            sys->ntp.is_enable = temp->valueint;

        temp = cJSON_GetObjectItem(ntp, "startup_delay");
        if(cJSON_IsNumber(temp))
            sys->ntp.startup_delay = temp->valueint;

        temp = cJSON_GetObjectItem(ntp, "update_period");
        if(cJSON_IsNumber(temp))
            sys->ntp.update_period = temp->valueint;

        temp = cJSON_GetObjectItem(ntp, "server1");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->ntp.server[0], temp->valuestring, sizeof(sys->ntp.server[0]));

        temp = cJSON_GetObjectItem(ntp, "server2");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->ntp.server[1], temp->valuestring, sizeof(sys->ntp.server[1]));

        temp = cJSON_GetObjectItem(ntp, "server3");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->ntp.server[2], temp->valuestring, sizeof(sys->ntp.server[2]));
    }

    // sensor list
    sensors = cJSON_GetObjectItem(config, "sensors");
    if(cJSON_IsObject(sensors))
    {
        cJSON * sensor;
        cJSON_ArrayForEach(sensor, sensors)
        {
            cJSON * temp;
            sensor_config_t* sensor_cfg;
            temp = cJSON_GetObjectItem(sensor, "interface");
            if(cJSON_IsString(temp) && temp->string != NULL){
                sensor_cfg = new_sensor(sensor->string, temp->valuestring);
            }
            else {
                LOG_E("read json file error at %", sensor->string);
            }

            temp = cJSON_GetObjectItem(sensor, "enable");
            if(cJSON_IsBool(temp))
                sensor_cfg->is_enable = temp->valueint;

            temp = cJSON_GetObjectItem(sensor, "addr");
            if(cJSON_IsNumber(temp))
                sensor_cfg->addr = temp->valueint;

            temp = cJSON_GetObjectItem(sensor, "data_period");
            if(cJSON_IsNumber(temp))
                sensor_cfg->data_period = temp->valueint;

            temp = cJSON_GetObjectItem(sensor, "oversampling");
            if(cJSON_IsNumber(temp))
                sensor_cfg->oversampling = temp->valueint;

            // if there is private configuration, load.
            if(sensor_cfg->load_json)
                sensor_cfg->load_json(sensor_cfg, sensor);

            // store this sensor.
            LOG_I("read config for sensor %s", sensor->string);
            if(sys->sensors) // the following
                add_sensor(sys->sensors, sensor_cfg);
            else // the first
                sys->sensors = sensor_cfg;
        }
    }

end:
    cJSON_Delete(config);
    return status;
}


char* create_json_from_config(system_config_t* sys)
{
    char *ostring = NULL;
    cJSON *config = NULL;
    cJSON *sensors = NULL;
    cJSON *temp = NULL;
    cJSON *log = NULL;
    cJSON *record = NULL;
    cJSON *mqtt = NULL;
    cJSON *gnss = NULL;
    cJSON *ntp = NULL;

    // roots
    config = cJSON_CreateObject();
    if(!config) goto end;
    if(!cJSON_AddNumberToObject(config, "version", sys->version)) goto end;

    sensors = cJSON_CreateObject();
    if(!sensors) goto end;
    if(!cJSON_AddItemToObject(config, "sensors", sensors)) goto end;

    // write sensors
    sensor_config_t* sensor_cfg = sys->sensors;
    for(; sensor_cfg != NULL; sensor_cfg = sensor_cfg->next)
    {
        temp = cJSON_CreateObject();
        if(!temp) goto end;
        if(!cJSON_AddItemToObject(sensors, sensor_cfg->name, temp)) goto end;
        if(!cJSON_AddBoolToObject(temp, "enable", sensor_cfg->is_enable)) goto end;
        if(!cJSON_AddStringToObject(temp, "interface", sensor_cfg->interface_name)) goto end;
        if(!cJSON_AddNumberToObject(temp, "addr", sensor_cfg->addr)) goto end;
        if(!cJSON_AddNumberToObject(temp, "data_period", sensor_cfg->data_period)) goto end;
        if(!cJSON_AddNumberToObject(temp, "oversampling", sensor_cfg->oversampling)) goto end;

        // add private configuration.
        if(sensor_cfg->create_json)
            sensor_cfg->create_json(sensor_cfg, temp);
    }

    record = cJSON_CreateObject();
    if(!record) goto end;
    if(!cJSON_AddItemToObject(config, "record", record)) goto end;
    if(!cJSON_AddBoolToObject(record, "enable", sys->record.is_enable)) goto end;
    if(!cJSON_AddBoolToObject(record, "split_file", sys->record.is_split_file)) goto end;
    if(!cJSON_AddStringToObject(record, "header", sys->record.header)) goto end;
    if(!cJSON_AddStringToObject(record, "data_path", sys->record.data_path)) goto end;
    if(!cJSON_AddNumberToObject(record, "period", sys->record.period)) goto end;
    if(!cJSON_AddNumberToObject(record, "max_file_size", sys->record.max_file_size)) goto end;

    log = cJSON_CreateObject();
    if(!record) goto end;
    if(!cJSON_AddItemToObject(config, "log", log)) goto end;
    if(!cJSON_AddBoolToObject(log, "enable", sys->log.is_enable)) goto end;
    if(!cJSON_AddBoolToObject(log, "repeat_header", sys->log.is_repeat_header)) goto end;
    if(!cJSON_AddStringToObject(log, "header", sys->log.header)) goto end;
    if(!cJSON_AddNumberToObject(log, "period", sys->log.period)) goto end;

    mqtt = cJSON_CreateObject();
    if(!mqtt) goto end;
    if(!cJSON_AddItemToObject(config, "mqtt", mqtt)) goto end;
    if(!cJSON_AddBoolToObject(mqtt, "enable", sys->mqtt.is_enable)) goto end;
    if(!cJSON_AddNumberToObject(mqtt, "period", sys->mqtt.period)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "interface", sys->mqtt.interface)) goto end;
    if(!cJSON_AddNumberToObject(mqtt, "baudrate", sys->mqtt.baudrate)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "module", sys->mqtt.module)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "wifi_ssid", sys->mqtt.wifi_ssid)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "wifi_password", sys->mqtt.wifi_password)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "mqtt_username", sys->mqtt.mqtt_username)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "mqtt_password", sys->mqtt.mqtt_password)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "uri", sys->mqtt.uri)) goto end;
    if(!cJSON_AddNumberToObject(mqtt, "port", sys->mqtt.port)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "pub_data", sys->mqtt.pub_data)) goto end;
    if(!cJSON_AddStringToObject(mqtt, "topic_prefix", sys->mqtt.topic_prefix)) goto end;

    gnss = cJSON_CreateObject();
    if(!gnss) goto end;
    if(!cJSON_AddItemToObject(config, "gnss", gnss)) goto end;
    if(!cJSON_AddBoolToObject(gnss, "enable", sys->gnss.is_enable)) goto end;
    if(!cJSON_AddNumberToObject(gnss, "period", sys->gnss.period)) goto end;
    if(!cJSON_AddStringToObject(gnss, "interface", sys->gnss.interface)) goto end;
    if(!cJSON_AddNumberToObject(gnss, "baudrate", sys->gnss.baudrate)) goto end;

    ntp = cJSON_CreateObject();
    if(!ntp) goto end;
    if(!cJSON_AddItemToObject(config, "ntp", ntp)) goto end;
    if(!cJSON_AddBoolToObject(ntp, "enable", sys->ntp.is_enable)) goto end;
    if(!cJSON_AddNumberToObject(ntp, "startup_delay", sys->ntp.startup_delay)) goto end;
    if(!cJSON_AddNumberToObject(ntp, "update_period", sys->ntp.update_period)) goto end;
    if(!cJSON_AddStringToObject(ntp, "server1", sys->ntp.server[0])) goto end;
    if(!cJSON_AddStringToObject(ntp, "server2", sys->ntp.server[1])) goto end;
    if(!cJSON_AddStringToObject(ntp, "server3", sys->ntp.server[2])) goto end;

    ostring = cJSON_Print(config);

end:
    cJSON_Delete(config);
    return ostring;
}

int sys_config(int argc, void*argv)
{
    char *out = create_json_from_config(&system_config);
    printf(out);
    free(out);
    return 0;
}
MSH_CMD_EXPORT(sys_config, print system configuration)

int save_system_cfg_to_file()
{
    /* to file system */
    int fd = open("/config.json", O_CREAT| O_WRONLY | O_TRUNC);
    if(fd < 0){
       LOG_E("config file create failed");
       return -1;
    }
    else{
        char *out = create_json_from_config(&system_config);
        //printf(out);
        write(fd, out, strlen(out));
        close(fd);
        free(out);
        return 0;
    }
}

bool is_system_cfg_valid() {return system_config.is_valid;}

int init_load_default_config()
{
    int fd;
    // load default first.
    load_default_config(&system_config);

    // wait for filesystem
    rt_thread_delay(1000);

    // if config not in sd card, create one and store the default.
    if(access("/config.json", 0))
    {
        /* to file system */
        fd = open("/config.json", O_CREAT| O_WRONLY | O_TRUNC);
        if(fd < 0){
           LOG_E("config file create failed");
        }
        else{
            char *out = create_json_from_config(&system_config);
            printf("%s", out);
            write(fd, out, strlen(out));
            close(fd);
            free(out);
        }
    }

    // try to load config from sd card
    fd = open("/config.json", O_RDONLY);
    if (fd >= 0)
    {
        const int buffer_size = 4096;
        char *json_str = malloc(buffer_size);
        int size;
        memset(json_str, 0, buffer_size);
        size = read(fd, json_str, buffer_size);
        if(size >= buffer_size * 0.8)
            LOG_W("json config file size %d close to maximum buffer %d", size, buffer_size);
        json_str[size] = '\0';      // must end this str.
        //printf("%s",json_str);
        load_config_from_json(&system_config, json_str);
        close(fd);
        free(json_str);
    }
    else {
        LOG_W("default configuration is loaded, please check.");
    }

    // set it is valid
    system_config.is_valid = true;
    return 0;
}
INIT_APP_EXPORT(init_load_default_config);





