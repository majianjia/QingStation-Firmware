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


#include "cjson/cjson.h"

#define DBG_TAG "config"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


void add_sensor(sensor_config_t* list, sensor_config_t* new_sensor)
{
    if(list && new_sensor)
    {
        while(list->next != NULL){
            list = list->next;
        }
        list->next = new_sensor;
    }
    else if (list==NULL){
        list = new_sensor;
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
    .update_rate = 1,           \
    .oversampling = 0          \
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
    s = malloc(sizeof(sensor_config_t));
    if(!s)
        return NULL;
    memset(s, 0, sizeof(sensor_config_t));
    memcpy(s, &def_sensor_cfg, sizeof(sensor_config_t));
    strncpy(s->name, name, MAX_NAME_LEN);
    strncpy(s->interface_name, interface, MAX_NAME_LEN);
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
    if(!cJSON_AddItemToObject(json, "setting", temp)) return;

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
    if(!cJSON_AddItemToObject(json, "setting", temp)) return;
    if(!cJSON_AddNumberToObject(temp, "height", ane->height)) return;
    if(!cJSON_AddNumberToObject(temp, "pitch", ane->pitch)) return;
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
    temp = cJSON_GetObjectItem(json, "height");
    if(cJSON_IsNumber(temp))
        ane->height = temp->valuedouble;
    temp = cJSON_GetObjectItem(json, "pitch");
    if(cJSON_IsNumber(temp))
        ane->pitch = temp->valuedouble;
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
    s->update_rate = 25;
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
    s->update_rate = 1;
    s->oversampling = 1;
    if(s == NULL) return;
    anemometer_config_t* ane_cfg = malloc(sizeof(anemometer_config_t));
    if(ane_cfg == NULL) return;
    memset(ane_cfg, 0, sizeof(anemometer_config_t));
    ane_cfg->height = 0.048f;     // default, normally set to 5 cm
    ane_cfg->pitch = 0.04f;      // defualt pitch, based on the 3D file
    s->user_data = ane_cfg;
    s->create_json = anemo_create_json;
    s->load_json = anemo_load_json;
    add_sensor(sys->sensors, s);

    s = new_sensor("Rain", "adc_ch6");
    if(s == NULL) return;
    add_sensor(sys->sensors, s);

    // test
    sys->uart1.bitrate = 115200;
    sys->uart2.bitrate = 115200;

    // recorder
    sys->record.is_enable = true;
    sys->record.is_split_file = false;
    strcpy(sys->record.header, "temp,humidity,pressure,red,green,blue,infrared");
    sys->record.period = 1000;
    strcpy(sys->record.root_path, "/");

    // log
    sys->log.is_enable = true;
    sys->log.is_repeat_header = true;
    strcpy(sys->log.header,"temp,humidity,pressure,light,num_sat,latitude,longitude,windspeed");
    sys->log.period = 30000;

    // test
    sys->ane_record_pulse = false;
}


int load_config_from_json(system_config_t* sys, char* json_strings)
{
    cJSON *config = NULL;
    cJSON *sensors = NULL;
    cJSON *log = NULL;
    cJSON *record = NULL;
    cJSON *version = NULL;
    cJSON *uart = NULL;

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

        temp = cJSON_GetObjectItem(record, "header");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->record.header, temp->valuestring, MAX_HEADER_LEN);

        temp = cJSON_GetObjectItem(record, "root_path");
        if(cJSON_IsString(temp) && temp->string != NULL)
            strncpy(sys->record.root_path, temp->valuestring, 32);
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

            temp = cJSON_GetObjectItem(sensor, "update_rate");
            if(cJSON_IsNumber(temp))
                sensor_cfg->update_rate = temp->valueint;

            temp = cJSON_GetObjectItem(sensor, "oversampling");
            if(cJSON_IsNumber(temp))
                sensor_cfg->oversampling = temp->valueint;

//            temp = cJSON_GetObjectItem(sensor, "mode");
//            if(cJSON_IsNumber(temp))
//                sensor_cfg->mode = temp->valueint;

            // if there is private configuration, load.
            if(sensor_cfg->load_json)
                sensor_cfg->load_json(sensor_cfg, sensor);

            // store this sensor.
            LOG_I("read config for sensor %s", sensor->string);
            //rt_thread_delay(100);
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
        if(!cJSON_AddNumberToObject(temp, "update_rate", sensor_cfg->update_rate)) goto end;
        if(!cJSON_AddNumberToObject(temp, "oversampling", sensor_cfg->oversampling)) goto end;
        //if(!cJSON_AddNumberToObject(temp, "mode", sensor_cfg->mode)) goto end;

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
    if(!cJSON_AddStringToObject(record, "root_path", sys->record.root_path)) goto end;
    if(!cJSON_AddNumberToObject(record, "period", sys->record.period)) goto end;

    log = cJSON_CreateObject();
    if(!record) goto end;
    if(!cJSON_AddItemToObject(config, "log", log)) goto end;
    if(!cJSON_AddBoolToObject(log, "enable", sys->log.is_enable)) goto end;
    if(!cJSON_AddBoolToObject(log, "repeat_header", sys->log.is_repeat_header)) goto end;
    if(!cJSON_AddStringToObject(log, "header", sys->log.header)) goto end;
    if(!cJSON_AddNumberToObject(log, "period", sys->log.period)) goto end;

    ostring = cJSON_Print(config);

end:
    cJSON_Delete(config);
    return ostring;
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
        char *json_str = rt_malloc(4096);
        int size;
        memset(json_str, 0, 4096);
        size = read(fd, json_str, 4096);
        json_str[size] = '\0';      // must end this str.
        //printf("%s",json_str);
        load_config_from_json(&system_config, json_str);
        close(fd);
        free(json_str);
    }
    else {
        LOG_W("default configuration is loaded, please change.");
    }

    // set it is valid
    system_config.is_valid = true;
    return 0;
}
INIT_APP_EXPORT(init_load_default_config);





