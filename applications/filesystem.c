/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-07     Jianjia Ma       the first version
 */

#include <rtthread.h>
#include <stm32l4xx.h>
#include <rtdevice.h>
#include <board.h>
#include "dfs_fs.h"
#include "stdio.h"
#include "drv_sdio.h"

#include "ulog_file.h"

#define DBG_TAG "sdcard"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define SD_DETECT_PIN    GET_PIN(A, 10)
#define SD_POWER_PIN    GET_PIN(A, 15)

static int sdcard_mount(char path[])
{
    rt_device_t device;

    device = rt_device_find("sd0");
    if (device == NULL)
    {
        mmcsd_wait_cd_changed(0);
        stm32_mmcsd_change();
        mmcsd_wait_cd_changed(RT_WAITING_FOREVER);
        device = rt_device_find("sd0");
    }
    if (device != RT_NULL)
    {
        if (dfs_mount("sd0", "/", "elm", 0, 0) == RT_EOK)
        {
            LOG_I("sd card mount to '/'");
        }
        else
        {
            LOG_W("sd card mount to '/' failed!");
        }
    }
    return 0;
}

static void sdcard_unmount(char path[])
{
    rt_thread_mdelay(200);
    dfs_unmount("/");
    LOG_I("Unmount \"/\"");

    mmcsd_wait_cd_changed(0);
    stm32_mmcsd_change();
    mmcsd_wait_cd_changed(RT_WAITING_FOREVER);
}

void thread_sdcard(void *parameters)
{
    rt_uint8_t is_sd_inited = 0;

    rt_pin_mode(SD_DETECT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(SD_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SD_POWER_PIN, PIN_LOW); // enable power
    rt_thread_mdelay(500);               // this is needed, to delay our initilization

    while(1)
    {
        if(!rt_pin_read(SD_DETECT_PIN) && !is_sd_inited)
        {
            rt_thread_mdelay(100);
            if(!rt_pin_read(SD_DETECT_PIN) && !is_sd_inited)
            {
                // mount
                sdcard_mount("/");
                ulog_file_backend_init();
                is_sd_inited = 1;
            }
        }

        // card not present, sd inited. -> unmount
        if(is_sd_inited && rt_pin_read(SD_DETECT_PIN))
        {
            // unmount
            sdcard_unmount("/");
            ulog_file_backend_deinit();
            is_sd_inited = 0;
        }

        rt_thread_mdelay(500);
    }
}


int thread_sdcard_init(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("filesys", thread_sdcard, RT_NULL,
            2048, 20, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_ENV_EXPORT(thread_sdcard_init);





