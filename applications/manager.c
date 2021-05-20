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


// cpu cycle count
void cpu_timer_init(void)
{
     /* Disable TRC */
     CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
     /* Enable TRC */
     CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
     /* Disable clock cycle counter */
     DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
     /* Enable clock cycle counter */
     DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
     /* Reset the clock cycle counter value */
     DWT->CYCCNT = 0;
}

unsigned int get_cpu_timer(){
    return DWT->CYCCNT;
}

#define MAX_THREAD 16

// test for thread hooks
typedef struct thread_info {
    struct rt_thread* tid;
    uint32_t prev_time;
    uint32_t time;
    rt_tick_t leave_tick;
    float cpu;
} thread_info_t;

thread_info_t threads[MAX_THREAD];

thread_info_t *find_or_add(thread_info_t *list, struct rt_thread* tid)
{
    int i = 0;
    for(i=0; i<MAX_THREAD; i++)
    {
        if(tid == list[i].tid)
            return &list[i];
    }
    for(i=0; i<MAX_THREAD; i++)
    {
        if(list[i].tid == NULL)
        {
            list[i].tid = tid;
            return &list[i];
        }
    }
    return NULL;

}

void delete_from_list(thread_info_t *list, struct rt_thread* tid)
{
    for(int i=0; i<MAX_THREAD; i++)
    {
        if(tid == list[i].tid)
        {
            memset(&list[i], 0, sizeof(thread_info_t));
            return;
        }
    }
}

// this hook takes times
// when thread = 32
// -o0 max 1976 normal 310-570
// -o2 max 702

static void hook_of_scheduler(struct rt_thread* from, struct rt_thread* to)
{
    thread_info_t *t = NULL;
    uint32_t ts = get_cpu_timer();
    // stop
    t = find_or_add(threads, from);
    if(t)
    {
        t->time += ts - t->prev_time;
        t->leave_tick = rt_tick_get();
    }

    t = find_or_add(threads, to);
    if(t)
        t->prev_time = ts;
    //rt_kprintf("from: %s -->  to: %s \n", from->name , to->name);
}

int cpuusage(int argc, void*argv[])
{
    for(int i=0; i<MAX_THREAD; i++)
    {
        if(threads[i].tid != NULL)
            printf("%8s : %.2f%%\n", threads[i].tid->name, threads[i].cpu);
    }
    return 0;
}
MSH_CMD_EXPORT(cpuusage, print cpu for every thread)

void thread_manager(void* parameters)
{
    rt_thread_delay(3000);
    rt_pm_request(PM_SLEEP_MODE_IDLE); // stop cpu when in idle
    rt_pm_release(PM_SLEEP_MODE_NONE); // release the default none sleep mode.

    cpu_timer_init();
    rt_scheduler_sethook(hook_of_scheduler);
    memset(threads, 0, sizeof(threads));

    while(1)
    {
        uint32_t total = 0;

        rt_thread_delay(1000);

        // detect if a thread is timeout or deleted.
        for(int i=0; i<MAX_THREAD; i++)
        {
            if(rt_tick_get() - threads[i].leave_tick >= 10000)
                delete_from_list(threads, threads[i].tid);
        }

        for(int i=0; i<MAX_THREAD; i++)
        {
           total += threads[i].time;
        }

        for(int i=0; i<MAX_THREAD; i++)
        {
            threads[i].cpu = (double)(threads[i].time) *100 / total;
            threads[i].time = 0;
        }
    }
}

int thread_manager_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("mngr", thread_manager, RT_NULL, 1024, 30, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_manager_init);


