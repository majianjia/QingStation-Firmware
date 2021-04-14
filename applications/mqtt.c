
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
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "data_pool.h"

#define DBG_TAG "mqtt"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include <at_device_esp8266.h>
#include "wifi_password.h"

#define ESP8266_DEIVCE_NAME     "esp0"
#define ESP8266_CLIENT_NAME     "lpuart1"
#define ESP8266_RECV_BUFF_LEN    (512)

static struct at_device_esp8266 esp0 =
{
    ESP8266_DEIVCE_NAME,
    ESP8266_CLIENT_NAME,

    ESP8266_WIFI_SSID,
    ESP8266_WIFI_PASSWORD,
    ESP8266_RECV_BUFF_LEN,
};

static int esp8266_device_register(void)
{
    struct at_device_esp8266 *esp8266 = &esp0;

    return at_device_register(&(esp8266->device),
                              esp8266->device_name,
                              esp8266->client_name,
                              AT_DEVICE_CLASS_ESP8266,
                              (void *) esp8266);
}
INIT_APP_EXPORT(esp8266_device_register);


void thread_mqtt(void* p)
{
    #define BUFSIZE  256
    char line[BUFSIZE] = {0};

    // infinite loop
    while(1)
    {
        rt_thread_mdelay(1000);
    }
}


int thread_mqtt_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("mqtt", thread_mqtt, RT_NULL, 2048, 25, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
 INIT_APP_EXPORT(thread_mqtt_init);

