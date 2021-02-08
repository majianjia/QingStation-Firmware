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
#include "stdio.h"

#define DBG_TAG "anemo"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_sem_t sem_ready;
static rt_timer_t timer;


// ADC = 1Msps, 500sample = 0.5ms ToF ~= 0.17m
// speed of sound: ~340m/s
// 0.5ms ~= 0.17m
#define ADC_SAMPLE_LEN (500)
uint16_t adc_buffer[4][ADC_SAMPLE_LEN] = {0};

static void timer_tick(void* parameter)
{
    rt_sem_release(sem_ready);
}



void thread_anemometer(void* parameters)
{
    recorder_t *recorder = NULL;
    char buf[128] = {0};
    ane_pwr_control(40*1000, true);

    while(1)
    {
        rt_sem_take(sem_ready, RT_WAITING_FOREVER);

        //
        ane_measure_ch(NORTH, 4, adc_buffer[0], ADC_SAMPLE_LEN);

        // only record a time to test
        if(recorder == NULL)
        {
            recorder = recorder_create("/sd/test.csv", "recorder", 128, 1000);
            recorder_write(recorder, "adc\n"); // header
            for(int i; i<ADC_SAMPLE_LEN; i++)
            {
                snprintf(buf, 128, "%d\n", (int)adc_buffer[i]);
                recorder_write(recorder, buf);
            }
            recorder_delete(recorder);
        }
    }
}



int thread_anemometer_init()
{
    rt_thread_t tid;

    sem_ready = rt_sem_create("anemo", 0, RT_IPC_FLAG_FIFO);
    timer = rt_timer_create("anemo", timer_tick, RT_NULL, 1000, RT_TIMER_FLAG_PERIODIC);
    tid = rt_thread_create("anemo", thread_anemometer, RT_NULL, 4096, 5, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_anemometer_init);
