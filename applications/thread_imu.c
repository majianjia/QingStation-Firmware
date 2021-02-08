/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-06     Jianjia Ma       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_anemometer.h"
#include "recorder.h"
#include "drv_bmx160.h"

#define DBG_TAG "imu"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_sem_t sem_ready;
static rt_timer_t timer;

static void timer_tick(void* parameter)
{
    rt_sem_release(sem_ready);
}

void thread_imu(void* parameters)
{
    rt_device_t i2c_bus = rt_device_find("i2c2");
    if(!i2c_bus)
    {
        LOG_E("cannot find i2c bus for IMU.");
        return;
    }

    // register and init the bmx160
    imu_dev_t *imu = bmx160_register(i2c_bus);
    imu->init(imu);

    while(1)
    {
        rt_sem_take(sem_ready, RT_WAITING_FOREVER);
        imu->read_raw(imu);
    }
}



int thread_imu_init()
{
    rt_thread_t tid;

    sem_ready = rt_sem_create("imu", 0, RT_IPC_FLAG_FIFO);
    timer = rt_timer_create("imu", timer_tick, RT_NULL, 100, RT_TIMER_FLAG_PERIODIC);
    tid = rt_thread_create("imu", thread_imu, RT_NULL, 1024, 10, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_imu_init);
