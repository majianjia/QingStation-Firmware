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

recorder_t * new_file(char* line)
{
    time_t timep;
    char filepath[128];
    recorder_t* recorder;
    // timestamp
    time(&timep);
    strftime(line, 64, "%Y%m%d_%H%M%S", gmtime(&timep));
    snprintf(filepath, 128, "%s/%s_%s", system_config.record.data_path, line,"log.csv");
    recorder = recorder_create(filepath, "rec", 2000);
    if(recorder == NULL)
    {
        // temporary check.
        LOG_E("Cannot create recording file");
        while(1)
            rt_thread_delay(1000);
    }
    return recorder;
}

void thread_record(void* parameters)
{
    #define MSG_SIZE 512
    uint16_t orders[64];
    uint32_t data_len = 0;
    char line[MSG_SIZE] = {0};
    time_t timep;
    // wait until system cfg loaded
    while(!is_system_cfg_valid() && system_config.record.is_enable)
        rt_thread_mdelay(1000);

    if(access(system_config.record.data_path, 0)){
        mkdir(system_config.record.data_path, 777);
    }
    if(access(system_config.record.data_path, 0))
    {
        LOG_E("data recording folder cannot be create : %s", system_config.record.data_path);
    }

    // create one
    recorder_t* recorder = new_file(line);

    // find out the data to be export (publish)
    // if the field is empty, then we print all data.
    if(strlen(system_config.record.header) == 0)
    {
        data_len = EXPORT_DATA_SIZE-1; // data 0 is "unknown"
        for(int i=0; i<data_len; i++)
            orders[i] = i+1;
    }
    // not empty, use it.
    else{
        // copy for us to destroy :p
        // get what data do we want.
        strncpy(line, system_config.record.header, MSG_SIZE);
        data_len = get_data_orders(line, ", ", orders, 64);
    }
    // write header
    recorder_write(recorder, "timestamp");
    for(int i=0; i<data_len; i++)
    {
        sprintf(line, ",%s",data_name[orders[i]]);
        recorder_write(recorder, line);
    }
    recorder_write(recorder, "\n");

    while(1)
    {
        rt_thread_mdelay(system_config.record.period - rt_tick_get() % system_config.record.period);
        int index = 0;

        // timestamp
        time(&timep);
        index += strftime(&line[index], 64, "%Y%m%d%H%M%S",  gmtime(&timep));

        // print each data to the str
        for(uint32_t i=0; i<data_len; i++)
        {
            index+= sprintf(&line[index],",");
            index+= print_data[orders[i]](&line[index]);
        }

        // next line.
        index+= sprintf(&line[index],"\n");

        // now write to recorder
        recorder_write(recorder, line);

        // new file when needed.
        if(system_config.record.is_split_file &&
                recorder->file_size >= system_config.record.max_file_size)
        {
            recorder_delete_wait(recorder);
            recorder = new_file(line);
            // write header
            recorder_write(recorder, "timestamp");
            for(int i=0; i<data_len; i++)
            {
                sprintf(line, ",%s",data_name[orders[i]]);
                recorder_write(recorder, line);
            }
            recorder_write(recorder, "\n");
        }

        // add some delay in case the speed too fast, that case multiple runs in 1ms
        rt_thread_mdelay(system_config.record.period/16);
    }
}

int thread_record_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("rec", thread_record, RT_NULL, 2048+512, 18, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_record_init);

