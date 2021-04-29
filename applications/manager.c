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


// test for thread hooks
typedef struct thread_info {
    struct rt_thread* tid;
    uint32_t prev_time;
    uint32_t time;
    uint32_t cpu;
} thread_info_t;

thread_info_t threads[32];

thread_info_t *find_or_add(thread_info_t *list, struct rt_thread* tid)
{
    int i = 0;
    for(i=0; i<32; i++)
    {
        if(tid == list[i].tid)
            return &list[i];
    }
    for(i=0; i<32; i++)
    {
        if(list[i].tid == NULL)
        {
            list[i].tid = tid;
            break;
        }
    }
    return &list[i];
}

static void hook_of_scheduler(struct rt_thread* from, struct rt_thread* to)
{
    thread_info_t *t = NULL;
    uint32_t ts = get_cpu_timer();
    // stop
    t = find_or_add(threads, from);
    t->time += ts - t->prev_time;

    t = find_or_add(threads, to);
    t->prev_time = ts;

    //rt_kprintf("from: %s -->  to: %s \n", from->name , to->name);
}

int cpuusage(int argc, void*argv[])
{
    char line[32];
    for(int i=0; i<32; i++)
    {
        if(threads[i].tid != NULL)
        {
            strncpy(line, threads[i].tid->name, 8);
            line[8] = '\0';
            printf("%10s: ", line);
            printf("%3u%%\n", threads[i].cpu);
        }
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

        rt_thread_delay(500);

        for(int i=0; i<32; i++)
        {
           total += threads[i].time;
        }

        for(int i=0; i<32; i++)
        {
            if(threads[i].tid != NULL)
            {
                threads[i].cpu = threads[i].time *100 / total;
                threads[i].time = 0;
            }
        }

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


