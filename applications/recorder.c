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
    rt_err_t rsl = 0;
    recorder_t *recorder = (recorder_t*) parameter;
    recorder_msg_t *rec_msg;
    do
    {
        rsl = rt_mb_recv(recorder->mailbox, (void*)&rec_msg, RT_TICK_PER_SECOND);
        if(rsl == -RT_ETIMEOUT)
            continue;

        if(recorder->fd < 0)
        {
            recorder->fd = open(recorder->file_path, O_RDWR | O_APPEND);
            // close if file error
            if(recorder->fd < 0)
                break;
        }
        led_indicate_busy();
        result = write(recorder->fd, rec_msg->msg, rec_msg->size);
        recorder->file_size += result;
        led_indicate_release();
        free(rec_msg);

        // reopen to flush the data
        if(recorder->reopen_after == 0 ||
           rt_tick_get() - recorder->_last_timestamp > recorder->reopen_after)
        {
            recorder->_last_timestamp = rt_tick_get();
            close(recorder->fd);
            recorder->fd = -1; // marked as closed. reopen required.
        }
    }while(recorder->is_open || rsl != -RT_ETIMEOUT); // wait for empty

    // clear mq
    while(rt_mb_recv(recorder->mailbox, (void*)&rec_msg, 1) != -RT_ETIMEOUT)
        free(rec_msg);

    if(recorder->fd >= 0)
        close(recorder->fd);
    rt_mb_delete(recorder->mailbox);
    memset(recorder, 0, sizeof(recorder_t)); // destroy magic word
    free(recorder);
}


void recorder_delete_wait(recorder_t * recorder)
{
    if(recorder == NULL)
        return;
    if(recorder->magic != RECORDER_MAGIC) // assert
        return;
    recorder->is_open = false;
    // wait until the handle is destroyed.
    while(recorder->magic != RECORDER_MAGIC)
        rt_thread_delay(10);
}

void recorder_delete(recorder_t * recorder)
{
    if(recorder == NULL)
        return;
    if(recorder->magic != RECORDER_MAGIC) // assert
        return;
    recorder->is_open = false;
    // the thread will self-close later
}

// return the num of byte written. error if return value < 0
int recorder_write(recorder_t * recorder, const char *str)
{
    if(recorder == NULL)
        return -1;
    if(recorder->magic != RECORDER_MAGIC) // assert
        return -1;
    if(!recorder->is_open)
        return -1;

    int len = strlen(str); // discard '\0'
    recorder_msg_t * msg = malloc(sizeof(recorder_msg_t) + len);
    if(!msg)
        return -1;
    msg->size = len;
    strncpy(msg->msg, str, len);
    return rt_mb_send_wait(recorder->mailbox, msg, RT_TICK_PER_SECOND);
}
/* filepath: the path -> it will be overwrited
 * name: a short name < 8 for this recorder
 * msg_size: the maximum size of a message (strlen()+1)
 * reopen_after: >0: the file will reopen to flush the data after the time
 *               =0: the file will be closed immediately after a message.  */
recorder_t * recorder_create(const char file_path[], const char name[], rt_tick_t reopen_after_ticks)
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
    recorder = malloc(sizeof(recorder_t));
    if(recorder == NULL)
    {
       close(fd);
       return NULL;
    }

    memset(recorder, 0, sizeof(recorder_t));
    recorder->magic = RECORDER_MAGIC;
    recorder->fd = fd;
    recorder->reopen_after = reopen_after_ticks;
    recorder->is_open = true;
    strncpy(recorder->file_path, file_path, 128);
    // message queue as a buffer to store data.
    recorder->mailbox = rt_mb_create(tname, 4, RT_IPC_FLAG_FIFO);
    if(!recorder->mailbox)
    {
        close(fd); free(recorder);
        return NULL;
    }
    // thread who do the recording.
    recorder->tid = rt_thread_create(tname, thread_recorder, recorder, 1500, 25, 1000);
    if(recorder->tid)
        rt_thread_startup(recorder->tid);
    else
    {
        close(fd); rt_mb_delete(recorder->mailbox); free(recorder);
        return NULL;
    }

    rt_thread_delay(5); // let the thread (low priority) start up.
    return recorder;
}



