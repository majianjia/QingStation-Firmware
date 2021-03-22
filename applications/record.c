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
#include <board.h>
#include <data_pool.h>
#include <rtdevice.h>
#include "configuration.h"
#include <string.h>
#include <stdlib.h>

#define DBG_TAG "record"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "data_pool.h"
#include "recorder.h"
#include "time.h"


void thread_record(void* parameters)
{
    uint16_t orders[32];
    uint32_t data_len = 0;
    char line[512] = {0};
    time_t timep;
    // wait until system cfg loaded
    while(!is_system_cfg_valid() && system_config.record.is_enable)
        rt_thread_mdelay(1000);

    //
    // timestamp
    time(&timep);
    strftime(line, 64, "%Y%m%d_%H%M%S", gmtime(&timep));
    char filepath[128];
    snprintf(filepath, 128, "%s/%s_%s", system_config.record.root_path, line,"test.csv");

    recorder_t* recorder = recorder_create(filepath, "rec", 256, 2000);

    if(recorder == NULL)
    {
        // temporary check.
        LOG_E("Cannot create recording file");
        while(1)
            rt_thread_delay(1000);
    }
    recorder_write(recorder, "timestamp,");
    recorder_write(recorder, system_config.record.header);
    recorder_write(recorder, "\n");

    // copy for us to destroy :p
    // get what data do we want.
    strncpy(line, system_config.record.header, 256);
    data_len = get_data_orders(line, ",", orders, 32);

    while(1)
    {
        rt_thread_mdelay(system_config.record.period - rt_tick_get() % system_config.record.period);
        //rt_tick_t timestamp = rt_tick_get();
        int index = 0;
        //index += sprintf(&line[index], "%d", timestamp);

        // timestamp
        time(&timep);
        index += strftime(&line[index], 32, "%H%M%S",  gmtime(&timep));

        // print each data to the str
        for(uint32_t i=0; i<data_len; i++ )
        {
            index+= sprintf(&line[index],",");
            index+= print_data[orders[i]](&line[index]);
        }

        // next line.
        index+= sprintf(&line[index],"\n");

        // now write to recorder
        recorder_write(recorder, line);

        // add some delay in case the speed too fast, that case multiple run in 1ms
        rt_thread_delay(20);
    }
}

int thread_record_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("rec", thread_record, RT_NULL, 2048, 20, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_record_init);

