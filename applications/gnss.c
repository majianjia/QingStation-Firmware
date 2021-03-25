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
#include "ctype.h"
#include "math.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "recorder.h"
#include "configuration.h"
#include "minmea.h"
#include "data_pool.h"

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

enum minmea_sentence_id parse_nmea_sentence(char *line)
{
    enum minmea_sentence_id id = minmea_sentence_id(line, false);
    switch (id) {
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
    return id;
}

//0=4800bps
//1=9600bps
//2=19200bps
//3=38400bps
//4=57600bps
//5=115200bps
void gps_set_bitrate(rt_device_t serial, uint32_t bitrate)
{
    uint32_t opt = 0;
    switch(bitrate){
    case 4800: opt = 0; break;
    case 9600: opt = 1; break;
    case 19200: opt = 2; break;
    case 38400: opt = 3; break;
    case 57600: opt = 4; break;
    case 115200: opt = 5; break;
    default: LOG_E("Set gps bitrate %d is not supported", bitrate);
    }

    //$PCAS01,br*CS<CR><LF>
    char cmd[32] = "$PCAS01,";
    int cs = 0;
    int idx = strlen(cmd);
    idx += sprintf(&cmd[idx],"%d", opt );
    cs = cmd[1];
    for(int i=2; i<idx; i++)
        cs^=cmd[i];
    idx += sprintf(&cmd[idx],"*%X\r\n", cs);
    // send
    rt_device_write(serial, 0, cmd, strlen(cmd));
}

int detect_bitrate(rt_device_t serial, uint32_t num_of_try)
{
    // find the corrent bit rate
    const int bitrate_table[] = {4800, 9600, 19200, 38400, 56700, 115200};
    int gnss_bitrate = 9600;
    for(int i=0; i<sizeof(bitrate_table)/sizeof(int); i++)
    {
        int size;
        char temp;
        LOG_D("try bitrate %dbps", bitrate_table[i]);
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate  =  bitrate_table[i];
        rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
        rt_thread_mdelay(10);
        // flush
        do{
            size = rt_device_read(serial, 0, &temp, 1);
        }while(size !=0);
        // test if characters are printable
        int try = num_of_try;
        bool is_char = true;
        while(try--)
        {
            do{
                size = rt_device_read(serial, 0, &temp, 1);
            }while(size == 0);

            if(!isprint(temp) && (temp!='\n' && temp!='\r' && temp!='\0'))
                is_char = false;
        }
        if(is_char)
        {
            LOG_I("Gnss bitrate detected: %dbsp", bitrate_table[i]);
            return bitrate_table[i];
        }
    }
    return -1;
}

static bool is_echo = false;
void thread_gnss(void* p)
{
    #define BUFSIZE  256
    char line[BUFSIZE] = {0};
    char buf [10];
    int32_t size;
    int32_t index = 0;
    char ch;
    int gnsss_bitrate = 0;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    rt_thread_mdelay(3000);
    rt_device_t serial = rt_device_find("uart2");
    rt_device_set_rx_indicate(serial, uart_input);
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);

    // try to scan the bitrates
    gnsss_bitrate = detect_bitrate(serial, 30);
    if(gnsss_bitrate == -1)
    {
        LOG_E("Cannot detect gnss model bitrate, uart will be set to default %dbps", system_config.uart2.bitrate);
        config.baud_rate  =  system_config.uart2.bitrate;
        if(RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config))
             LOG_E("change bitrate failed!\n");
    }
    // the detected bitrate is different than the system setting
    else if(gnsss_bitrate != system_config.uart2.bitrate)
    {
        LOG_I("Set gnss model bitrate to %dbps per configuration file", system_config.uart2.bitrate);
        gnsss_bitrate = system_config.uart2.bitrate;
        gps_set_bitrate(serial, gnsss_bitrate);
        config.baud_rate  =  gnsss_bitrate;
        if(RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config))
             LOG_E("change bitrate failed!\n");
    }

    // infinite loop
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
                enum minmea_sentence_id id;
                line[index] = '\0';
                //rt_kprintf("%s", line);
                id = parse_nmea_sentence(line);

                if(id == MINMEA_SENTENCE_RMC)
                {
                    if(rmc_frame.time.seconds == 0 && rmc_frame.valid) // sync clock every minutes
                    {
                        set_date(rmc_frame.date.year+2000, rmc_frame.date.month, rmc_frame.date.day);
                        set_time(rmc_frame.time.hours, rmc_frame.time.minutes, rmc_frame.time.seconds);
                    }
                    gnss.latitude = minmea_tocoord(&rmc_frame.latitude);
                    gnss.longitude = minmea_tocoord(&rmc_frame.longitude);
                    gnss.course = minmea_tofloat(&rmc_frame.course);
                    gnss.speed = minmea_tofloat(&rmc_frame.speed);
                    gnss.num_sat = gga_frame.satellites_tracked;
                    gnss.is_fixed = rmc_frame.valid;
                }

                if(is_echo)
                    printf(line);

                index = 0;
            }
            // buffer
            if(index >= BUFSIZE-1)
                index = 0;
        }
    }
}

int echo_gnss(int argc, void* argv[])
{
    is_echo = !is_echo;
    return 0;
}
MSH_CMD_EXPORT(echo_gnss, echo gnss raw nmea message to cmd line)

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
