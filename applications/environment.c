/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-08     Jianjia Ma       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <data_pool.h>
#include "drv_bme280.h"
#include "recorder.h"
#include "stdio.h"
#include "stdlib.h"

#define DBG_TAG "env"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

void thread_environment(void* parameters)
{
    double T;
    double H;
    double P;
    int32_t period = 1000;

    rt_device_t i2c_bus = rt_device_find("i2c2");
    if(!i2c_bus)
    {
        LOG_E("cannot find i2c bus for environment.");
        return;
    }

    // init and set to continual mode.
    bme280_initialization(i2c_bus);

    rt_thread_mdelay(100);

    while(1)
    {
        rt_thread_mdelay(period - rt_tick_get()%period);
        bme280_get_data(&H, &T, &P);

        // write to global data pool
        air_info.humidity = H;
        air_info.pressure = P;
        air_info.temperature = T;
        data_updated(&air_info.info);

        rt_thread_delay(2);
        //LOG_D("Humidity: %3.1f%%, Temp: %2.2f Degree, Pressure: %.3f Pa", H, T, P);
    }
}



int thread_environment_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("env", thread_environment, RT_NULL, 1024, 12, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_environment_init); // no noise
