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

/* This file hold the upper logic of the station running
 * It decides:
 *  - Which functions is turned on/off.
 *  - The sampling moment.
 *  - Recording.
 *  - Communication.
 *  - State machine.
 */

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <manager.h>
#include "configuration.h"
#include "recorder.h"
#include "pm.h"

#define DBG_TAG "mngr"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

void thread_manager(void* parameters)
{
    rt_thread_delay(3000);
    rt_pm_request(PM_SLEEP_MODE_IDLE); // stop cpu when in idle
    rt_pm_release(PM_SLEEP_MODE_NONE); // release the default none sleep mode.

    while(1)
    {
        rt_thread_delay(1000);
    }
}

int thread_manager_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("mngr", thread_manager, RT_NULL, 1024, 12, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_manager_init);


