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
#include "drv_bme280.h"
#include "recorder.h"
#include "stdio.h"
#include "stdlib.h"

#define DBG_TAG "env"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_sem_t sem_ready;
static rt_timer_t timer;

static void timer_tick(void* parameter)
{
    rt_sem_release(sem_ready);
}

void thread_environment(void* parameters)
{
    double T;
    double H;
    double P;
    int8_t buf[128];

    rt_device_t i2c_bus = rt_device_find("i2c2");
    if(!i2c_bus)
    {
        LOG_E("cannot find i2c bus for environment.");
        return;
    }

    // init and set to continual mode.
    bme280_initialization(i2c_bus);

    rt_thread_delay(RT_TICK_PER_SECOND*2);

    recorder_t *recorder = recorder_create("/env.csv", "env", 128, RT_TICK_PER_SECOND*10);
    if(recorder)
        recorder_write(recorder, "timestamp,temperature,humidity,pressure\n");

    rt_timer_start(timer);
    while(1)
    {
        rt_sem_take(sem_ready, RT_WAITING_FOREVER);
        bme280_get_data(&H, &T, &P);

        if(recorder)
        {
           snprintf(buf, 128, "%d, %3.3f, %2.3f, %.3f\n", rt_tick_get(), T, H, P);
           if(recorder)
               recorder_write(recorder, buf);
        }

        LOG_D("Humidity: %3.1f%%, Temp: %2.2f Degree, Pressure: %.3f Pa", H, T, P);
    }
}



int thread_environment_init()
{
    rt_thread_t tid;

    sem_ready = rt_sem_create("env", 0, RT_IPC_FLAG_FIFO);
    timer = rt_timer_create("env", timer_tick, RT_NULL, 10000, RT_TIMER_FLAG_PERIODIC);
    tid = rt_thread_create("env", thread_environment, RT_NULL, 2048, 12, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_environment_init);
