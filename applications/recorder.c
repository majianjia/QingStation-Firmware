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
#include <board.h>
#include <rtdevice.h>
#include "string.h"
#include <dfs.h>
#include <dfs_posix.h>
#include <stdbool.h>

#include "recorder.h"

#define DRV_DEBUG
#define LOG_TAG         "recorder"
#include <rtdbg.h>

// in main.c temp
void led_indicate_busy();
void led_indicate_release();

static void thread_recorder(void* parameter)
{
    int32_t result;
    recorder_t *recorder = (recorder_t*) parameter;
    do
    {
        if(rt_mq_recv(recorder->msg, recorder->buf, recorder->max_msg_size, RT_WAITING_FOREVER) == RT_ETIMEOUT)
            continue;

        if(recorder->fd < 0)
        {
            recorder->fd = open(recorder->file_path, O_RDWR | O_APPEND);
            // close if file error
            if(recorder->fd < 0)
                break;
        }
        led_indicate_busy();
        result = write(recorder->fd, recorder->buf, strlen(recorder->buf));
        recorder->file_size += result;
        led_indicate_release();

        // reopen to flush the data
        if(recorder->reopen_after == 0 ||
           rt_tick_get() - recorder->_last_timestamp > recorder->reopen_after)
        {
            recorder->_last_timestamp = rt_tick_get();
            close(recorder->fd);
            recorder->fd = -1; // marked as closed. reopen required.
        }
    }
    while(recorder->is_open);

    if(recorder->fd >= 0)
        close(recorder->fd);
    rt_mq_delete(recorder->msg);
    memset(recorder, 0, sizeof(recorder_t));
    free(recorder);
}

void recorder_delete(recorder_t * recorder)
{
    if(recorder == NULL)
        return;
    if(recorder->magic != RECORDER_MAGIC) // assert
        return;
    recorder->is_open = false; // the thread will self-close
}

// return the num of byte written. error if return value < 0
int recorder_write(recorder_t * recorder, const char *str)
{
    if(recorder == NULL)
        return 0;
    if(recorder->magic != RECORDER_MAGIC) // assert
        return 0;
    if(!recorder->is_open)
        return 0;

    return rt_mq_send(recorder->msg, str, strlen(str)+1); // strlen+1 for '\0'
}
/* filepath: the path -> it will be overwrited
 * name: a short name < 8 for this recorder
 * msg_size: the maximum size of a message (strlen()+1)
 * reopen_after: >0: the file will reopen to flush the data after the time
 *               =0: the file will be closed immediately after a message.  */
recorder_t * recorder_create(const char file_path[], const char name[], uint32_t msg_size, rt_tick_t reopen_after_ticks)
{
    recorder_t * recorder;
    char tname[16];
    int fd;
    snprintf(tname, 16, "%s%s","r_", name);
    fd = open(file_path, O_CREAT| O_RDWR | O_TRUNC);
    if(fd < 0)
    {
       LOG_E("Log file create failed: %s.", file_path);
       return NULL;
    }
    LOG_D("Recorder data file created: %s.", file_path);
    recorder = malloc(sizeof(recorder_t) + msg_size);
    if(recorder == NULL)
    {
       close(fd);
       return NULL;
    }

    memset(recorder, 0, sizeof(recorder_t) + msg_size);
    recorder->magic = RECORDER_MAGIC;
    recorder->buf = (char*)recorder + sizeof(recorder_t);
    recorder->max_msg_size = msg_size;
    recorder->fd = fd;
    recorder->file_path = file_path;
    recorder->reopen_after = reopen_after_ticks;
    recorder->is_open = true;

    // message queue as a buffer to store data.
    recorder->msg = rt_mq_create(tname, recorder->max_msg_size, 8, RT_IPC_FLAG_FIFO);
    if(!recorder->msg)
    {
        close(fd); free(recorder);
        return NULL;
    }
    // thread who do the recording.
    recorder->tid = rt_thread_create(tname, thread_recorder, recorder, 2048, 25, 1000);
    if(recorder->tid)
        rt_thread_startup(recorder->tid);
    else
    {
        close(fd); rt_mq_delete(recorder->msg); free(recorder);
        return NULL;
    }

    rt_thread_delay(5); // let the thread (low priority) start up.
    return recorder;
}



