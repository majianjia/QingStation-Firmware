/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-26     Jianjia Ma       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_as3935.h"
#include "data_pool.h"
#include "configuration.h"

#define DBG_TAG "lightning"
//#define DBG_LVL DBG_INFO
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define INT_PIN GET_PIN(B, 12)

static int32_t cali_count = 0;
void irq_callback(void *args)
{
    cali_count++;
}

void io_irq_enable(void)
{
    rt_pin_mode(INT_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(INT_PIN, PIN_IRQ_MODE_RISING, irq_callback, RT_NULL);
    rt_pin_irq_enable(INT_PIN, PIN_IRQ_ENABLE);
}

void io_irq_disable(void)
{
    rt_pin_irq_enable(INT_PIN, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(INT_PIN);
}


static bool is_calibrating = true;
bool is_lightning_calibrating()
{
    return is_calibrating;
}

bool is_lightning_print = false;
int lightning_info(int argc, char * argv[])
{
    is_lightning_print = !is_lightning_print;
    return 0;
}
MSH_CMD_EXPORT(lightning_info, print lightning info)

void thread_lightning(void* parameters)
{
    rt_device_t i2c_bus;
    sensor_config_t * cfg;
    rt_tick_t period = 0;
    uint32_t distance, energy;
    uint8_t calib_cap = 0;
    float calib_f = 0;
    uint8_t noise_level = 2;

    // wait and load the configuration
    do{
        rt_thread_delay(100);
        cfg = get_sensor_config("AS3935");
    }while(cfg == NULL || !is_system_cfg_valid());

    i2c_bus = rt_device_find(cfg->interface_name);
    if(!i2c_bus)
    {
        LOG_E("cannot find %s bus for AS3935.", cfg->interface_name);
        return;
    }

    rt_thread_delay(500);

    as3935_init(i2c_bus);
    io_irq_enable();

    LOG_I("Antenna calibrating");
    is_calibrating = true;
    for(int i=0;i<16; i++)
    {
        calib_cap = i;
        as3935_enable_clock_output(AS3935_LCO | calib_cap);
        cali_count = 0;
        rt_tick_t t = rt_tick_get();
        rt_thread_mdelay(200);
        t = rt_tick_get() - t;
        calib_f = ((float)cali_count/t)*128;
        if(calib_f < 502)
            break;
        LOG_D("ant: pulse:%d, sample times:%dms, freq = %.1fKHz", cali_count, t, calib_f);
    }
    is_calibrating = false;
    as3935_enable_clock_output(calib_cap); // disable clock output.
    LOG_I("Calibration is done: freq: %.1fkHz, calib_cap:%d x 8pF", calib_f, calib_cap);

    period = cfg->data_period / cfg->oversampling;
    while(1)
    {
        // add some delay in case the speed too fast, that case multiple run in 1ms
        rt_thread_delay(2);
        rt_thread_delay(period - rt_tick_get()%period);

        as3935_read_data(&distance, &energy);

        if(rt_pin_read(INT_PIN))
        {
            uint8_t event = as3935_read_int();
            if(is_lightning_print)
                printf("interrupted, %d\n", event);
            if(event & AS3935_INT_NOISE)
            {
                as3935_noise_level_set(noise_level);
                LOG_D("Set noise level to %d", noise_level);
                noise_level++;
                if(noise_level >7)
                    noise_level = 7;
            }
        }

        if(is_lightning_print)
            printf("distance: %u, energy:%u\n", distance, energy);

        lightning.distance = distance;
        data_updated(&lightning.info);
    }
}


int thread_lightning_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("lightni", thread_lightning, RT_NULL, 1024+512, 25, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_lightning_init);


