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
#include <time.h>

#define DBG_TAG "log"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "data_pool.h"

void thread_log(void* parameters)
{
    uint16_t orders[32];
    uint32_t data_len = 0;
    char str_buf[256] = {0};
    uint32_t str_index = 0;
    bool is_repeated_header;
    rt_device_t usb_cdc = NULL;
    time_t timep;

    // wait until system cfg loaded
    while(!is_system_cfg_valid() && system_config.log.is_enable)
        rt_thread_mdelay(1000);

    // copy for us to destroy :p
    strncpy(str_buf, system_config.log.header, 256);
    data_len = get_data_orders(str_buf, ", ", orders, 32);

    //
    is_repeated_header = system_config.log.is_repeat_header;

    // try to use USB CDC for test.
    usb_cdc = rt_device_find("vcom");
    if(usb_cdc)
        rt_device_open(usb_cdc, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);//  | RT_DEVICE_FLAG_STREAM ?

    while(1)
    {
        rt_thread_mdelay(system_config.log.period - rt_tick_get() % system_config.log.period);
        if(!system_config.log.is_enable)
            continue;
        str_index = 0;

        // timestamp
        time(&timep);
        struct tm * timeinfo = gmtime(&timep);
        str_index += strftime(str_buf, 32, "%Y%m%d_%H%M%S", timeinfo);

        for(uint32_t i=0; i<data_len; i++ )
        {
            str_index += sprintf(str_buf+str_index, ", ");
            if(is_repeated_header)
                str_index += sprintf(str_buf+str_index,"%s:", data_name[orders[i]]);
            str_index += print_data[orders[i]](str_buf+str_index);
        }

        // print
        str_index += sprintf(str_buf+str_index, "\n");
        printf("%s", str_buf);

        // copy to USB cdc
        if(usb_cdc)
            rt_device_write(usb_cdc, 0, str_buf, strlen(str_buf));

        // add some delay in case the speed too fast, that case multiple run in 1ms
        rt_thread_delay(2);
    }
}

int thread_log_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("log", thread_log, RT_NULL, 2048, 20, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_log_init);



