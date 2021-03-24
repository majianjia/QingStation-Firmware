/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-20     Jianjia Ma       the first version
 */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "recorder.h"
#include "configuration.h"
#include "minmea.h"

#define DBG_TAG "gnss"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_semaphore rx_sem;
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    return RT_EOK;
}


struct minmea_sentence_rmc rmc_frame;
struct minmea_sentence_gga gga_frame;
struct minmea_sentence_gsv gsv_frame;
struct minmea_sentence_gsa gsa_frame;
struct minmea_sentence_gll gll_frame;
struct minmea_sentence_vtg vtg_frame;
struct minmea_sentence_zda zda_frame;

int parse_nmea_sentence(char *line)
{
    switch (minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_RMC:
            minmea_parse_rmc(&rmc_frame, line);
            break;
        case MINMEA_SENTENCE_GGA:
            minmea_parse_gga(&gga_frame, line);
            break;
        case MINMEA_SENTENCE_GSV:
            minmea_parse_gsv(&gsv_frame, line);
            break;
        case MINMEA_SENTENCE_GSA:
            minmea_parse_gsa(&gsa_frame, line);
            break;
        case MINMEA_SENTENCE_GLL:
            minmea_parse_gll(&gll_frame, line);
            break;
        case MINMEA_SENTENCE_VTG:
            minmea_parse_vtg(&vtg_frame, line);
            break;
        case MINMEA_SENTENCE_ZDA:
            minmea_parse_zda(&zda_frame, line);
            break;
        default:break;
    }
    return 0;
}

void thread_gnss(void* p)
{
    #define BUFSIZE  256
    char line[BUFSIZE] = {0};
    char buf [10];
    int32_t size = 0;
    int32_t index = 0;
    char ch;

    rt_thread_mdelay(3000);
    rt_device_t serial = rt_device_find("uart2");
    rt_device_set_rx_indicate(serial, uart_input);
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate  =  BAUD_RATE_9600;
    if(RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config))
         LOG_E("change bitrate failed!\n");

    while(1)
    {
        // wait for data.
        do{
           rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
           size = rt_device_read(serial, 0, buf, 10);
        }while(size == 0);

        // check one by one
        for(int i=0; i<size; i++)
        {
            ch = buf[i];
            line[index++] = ch;

            // if sentance completed, process.
            if(ch == '\n')
            {
                line[index] = '\0';
                //rt_kprintf("%s", line);
                parse_nmea_sentence(line);
                index = 0;
            }
            // buffer
            if(index >= BUFSIZE-1)
                index = 0;
        }
    }
}

int thread_gnss_init()
{
    rt_thread_t tid;
    rt_sem_init(&rx_sem, "gnss_rx", 0, RT_IPC_FLAG_FIFO);
    tid = rt_thread_create("gnss", thread_gnss, RT_NULL, 2048, 16, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_gnss_init);
