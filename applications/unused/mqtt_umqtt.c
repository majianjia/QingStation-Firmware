
/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-20     Jianjia Ma       the first version
 */
#include <stdio.h>
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "data_pool.h"
#include "configuration.h"

#define DBG_TAG "mqtt"
#define DBG_LVL DBG_INFO
//#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "umqtt.h"
#include "umqtt_internal.h"

#include <at_device_esp8266.h>
#include "wifi_password.h"

#define TEST_WITH_LOCAL_BROKER
#define LOCAL_URI "tcp://192.168.1.85:1883"

// #define MQTT_URI                "tcp://test.mosquitto.org:1883"
// #define MQTT_URI                "tcp://mq.tongxinmao.com:18831"
//#define MQTT_URI                "tcp://broker.emqx.io:1883"
#define MQTT_URI                "tcp://qbe26b46.en.emqx.cloud:11523"

static int is_started = 0;
static umqtt_client_t m_umqtt_client = RT_NULL;




#define ESP8266_DEIVCE_NAME     "esp0"
#define ESP8266_CLIENT_NAME     "lpuart1"
#define ESP8266_RECV_BUFF_LEN    (512)

static struct at_device_esp8266 esp0;

// store locally for security. i.e. not export to default config cjson
static char ssid[sizeof(system_config.mqtt.wifi_ssid)] = {0};
static char password[sizeof(system_config.mqtt.wifi_password)] = {0};
static char mqtt_username[sizeof(system_config.mqtt.mqtt_username)] = {0};
static char mqtt_password[sizeof(system_config.mqtt.mqtt_password)] = {0};
static char uri[128] = {0};

static int esp8266_device_register(void)
{
    struct at_device_esp8266 *esp8266 = &esp0;

    if(strlen(system_config.mqtt.wifi_ssid) != 0)
    {
        strncpy(ssid, system_config.mqtt.wifi_ssid, sizeof(ssid));
        strncpy(password, system_config.mqtt.wifi_password, sizeof(password));
        if(strlen(ssid) == 0)
        {
            LOG_E("No ssid set, please set SSID for wifi connection (sd/config.json/mqtt/wifi_ssid)");
            return -1;
        }
        esp8266->wifi_ssid = ssid;
        esp8266->wifi_password = password;
    }
    else {
        LOG_I("SSID set to firmware default.");
        esp8266->wifi_ssid = ESP8266_WIFI_SSID;
        esp8266->wifi_password = ESP8266_WIFI_PASSWORD;
    }

    esp8266->client_name = system_config.mqtt.interface;
    esp8266->device_name = ESP8266_DEIVCE_NAME;
    esp8266->recv_line_num = ESP8266_RECV_BUFF_LEN;

    return at_device_register(&(esp8266->device),
                              esp8266->device_name,
                              esp8266->client_name,
                              AT_DEVICE_CLASS_ESP8266,
                              (void *) esp8266);
}

static int sim800c_device_register(void)
{
    return -1;
}


static int user_callback(struct umqtt_client *client, enum umqtt_evt event)
{
    RT_ASSERT(client);

    switch(event)
    {
    case UMQTT_EVT_LINK:
        LOG_D(" user callback, event - link!");
        break;
    case UMQTT_EVT_ONLINE:
        LOG_D(" user callback, event - online!");
        break;
    case UMQTT_EVT_OFFLINE:
        LOG_D(" user callback, event - offline!");
        break;
    case UMQTT_EVT_HEARTBEAT:
        LOG_D(" user callback, event - heartbeat!");
        break;
    default:
        LOG_D(" user callback, event:%d", event);
        break;
    }

    return 0;
}

static int mqtt_start()
{
    struct umqtt_info umqtt_info = { 0 };

#ifdef TEST_WITH_LOCAL_BROKER
    umqtt_info.uri = MQTT_URI;
    umqtt_info.uri = LOCAL_URI;
#else
    sprintf(uri, "tcp://%s:%d", system_config.mqtt.uri, system_config.mqtt.port);
    umqtt_info.uri = uri;
#endif

    if(strlen(system_config.mqtt.mqtt_username) != 0)
    {
        strncpy(mqtt_username, system_config.mqtt.mqtt_username, sizeof(mqtt_username));
        strncpy(mqtt_password, system_config.mqtt.mqtt_password, sizeof(mqtt_password));
        umqtt_info.user_name = mqtt_username;
        umqtt_info.password = mqtt_password;
    }
    else{
#ifndef TEST_WITH_LOCAL_BROKER
        umqtt_info.user_name = MQTT_USERNAME;
        umqtt_info.password = MQTT_PASSWORD;
#endif
    }

    m_umqtt_client = umqtt_create(&umqtt_info);
    if (m_umqtt_client == RT_NULL)
    {
        LOG_E(" umqtt client create failed!");
        return -1;
    }
    umqtt_control(m_umqtt_client, UMQTT_CMD_EVT_CB, user_callback);

    if (umqtt_start(m_umqtt_client) >= 0){
        LOG_I(" umqtt start success!");
        is_started = 1;
    }
    else{
        m_umqtt_client = RT_NULL;
        LOG_E(" umqtt start failed!");
    }
    return 0;
}

static int mqtt_stop()
{
    umqtt_stop(m_umqtt_client);
    umqtt_delete(m_umqtt_client);
    m_umqtt_client = RT_NULL;
    return 0;
}

static int is_mqtt_linked ()
{
    enum umqtt_client_state state = 0;
    if(!m_umqtt_client)
        return 0;
    state = umqtt_control(m_umqtt_client, UMQTT_CMD_GET_CLIENT_STA, NULL);
    return state & UMQTT_CS_LINKED;
}

static int mqtt_publish(const char topic[], const char value[])
{
    return umqtt_publish(m_umqtt_client, 2, topic, value, strlen(value), 1000);
}

void thread_mqtt(void* p)
{
    #define BUFSIZE  64
    char topic[BUFSIZE] = {0};
    char line[BUFSIZE] = {0};

    int data_len;
    uint16_t orders[64];
    char str_buf[256] = {0};
    mqtt_config_t *cfg;

    // wait until system cfg loaded
    // wait and load the configuration
    do{
        rt_thread_delay(100);
    }while(!is_system_cfg_valid());
    cfg = &system_config.mqtt;

    while(!cfg->is_enable)
        rt_thread_delay(1000);

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_t serial = rt_device_find(cfg->interface);
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);

    config.baud_rate  =  cfg->baudrate;
    if(RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config))
         LOG_E("change baudrate %d faile %s failed!", cfg->baudrate, cfg->interface );

    if(!strcasecmp("esp8266", system_config.mqtt.module))
        esp8266_device_register();
    else if(!strcasecmp("sim800c", system_config.mqtt.module))
        sim800c_device_register();
    else
        LOG_E("MQTT does not support %s module, please check config/mqtt/module.", system_config.mqtt.module);

    // find out the data to be export (publish)
    if(strlen(system_config.mqtt.pub_data) == 0)
    {
        data_len = EXPORT_DATA_SIZE-1; // data 0 is "unknown"
        for(int i=0; i<data_len; i++)
            orders[i] = i+1;
    }
    // if
    else{
        strncpy(str_buf, system_config.mqtt.pub_data, 256);
        data_len = get_data_orders(str_buf, ", ", orders, 32);
    }

    // wait for SAL and AT device.
    rt_thread_mdelay(5000);

    // start mqtt
    LOG_I("Start MQTT");
    mqtt_start();
    while(!is_mqtt_linked())
        rt_thread_delay(100);
    LOG_I("MQTT linked");


    // infinite loop
    while(1)
    {
        for(int i=0; i< data_len; i++)
        {
//            #define PREFIX "qing/"
//            snprintf(topic, sizeof(topic), "%s%s", PREFIX, data_name[i]);
//            print_data[i](line);
//            mqtt_publish(topic, line);
//            rt_thread_mdelay(50);

            print_data[orders[i]](line);
            if(mqtt_publish(data_name[orders[i]], line) != 0)
            {
                // reconnected needed.
                LOG_E("publish fail");
                mqtt_stop();
                rt_thread_delay(1000);
                mqtt_start();
                rt_thread_delay(1000);
            }
            rt_thread_mdelay(500); // this must be large enough.
        }
    }
}


int thread_mqtt_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("mqtt", thread_mqtt, RT_NULL, 2048+512, 8, 1000); // it need to be higher than at_clnt thread
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
 INIT_APP_EXPORT(thread_mqtt_init);

