/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-26     Jianjia Ma       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_as3935.h"
#include "data_pool.h"
#include "configuration.h"

#define DBG_TAG "lightning"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


void thread_lightning(void* parameters)
{
    rt_device_t i2c_bus;
    sensor_config_t * cfg;
    rt_tick_t period = 0;
    uint32_t distance, energy;

    // wait and load the configuration
    do{
        rt_thread_delay(100);
        cfg = get_sensor_config("AS3935");
    }while(cfg == NULL || !is_system_cfg_valid());

    i2c_bus = rt_device_find(cfg->interface_name);
    if(!i2c_bus)
    {
        LOG_E("cannot find %s bus for AS3935.", cfg->interface_name);
        return;
    }
    // update rate in tick
    period = 1000/cfg->update_rate;

    as3935_init(i2c_bus);

    while(1)
    {
        // add some delay in case the speed too fast, that case multiple run in 1ms
        rt_thread_delay(2);
        rt_thread_delay(period - rt_tick_get()%period);

        as3935_read_data(&distance, &energy);

        //printf("%d, %d\n", distance, energy);

        lightning.distance = distance;
        data_updated(&lightning.info);
    }
}


int thread_lightning_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("lightni", thread_lightning, RT_NULL, 1024, 25, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_lightning_init);
