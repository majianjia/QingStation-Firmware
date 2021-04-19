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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_apds9250.h"
#include "data_pool.h"
#include "configuration.h"

#define DBG_TAG "light"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


void thread_light(void* parameters)
{
    uint32_t r,g,b,ir = 0;
    rt_device_t i2c_bus;
    sensor_config_t * cfg;
    rt_tick_t period = 0;

    // wait and load the configuration
    do{
        rt_thread_delay(100);
        cfg = get_sensor_config("APDS9250");
    }while(cfg == NULL || !is_system_cfg_valid());

    i2c_bus = rt_device_find(cfg->interface_name);
    if(!i2c_bus)
    {
        LOG_E("cannot find %s bus for APDS9250.", cfg->interface_name);
        return;
    }
    // update rate in tick
    apds9250_init(i2c_bus);
    apds9250_select_sensor(APDS_SELECT_RGBIR);

    while(1)
    {
        // add some delay in case the speed too fast, that case multiple run in 1ms
        rt_thread_delay(2);
        rt_thread_delay(cfg->data_period - rt_tick_get()%cfg->data_period);

        if(!cfg->is_enable)
            continue;

        apds9250_read_rgbir(&r, &g, &b, &ir);

        light_info.R = r;
        light_info.G = g;
        light_info.B = b;
        light_info.IR = ir;
        light_info.ALS = g;  // it is the same as green.
        data_updated(&light_info.info);

    }
}


int thread_light_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("light", thread_light, RT_NULL, 1024, 15, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_light_init);
