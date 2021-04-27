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

#ifndef __RECORDER_H__
#define __RECORDER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <board.h>
#include <stdbool.h>
#include <rtdevice.h>
#include "string.h"
#include <dfs.h>
#include <dfs_posix.h>

#define RECORDER_MAGIC (0x787679AE)

// message container.
typedef struct _recorder_msg_t {
    int size;
    char msg[0];
}recorder_msg_t ;

typedef struct _recorder_t
{
   uint32_t magic;
   rt_thread_t tid;
   rt_mailbox_t mailbox;
   bool is_open;
   uint32_t file_size;
   int32_t error_code;
   int fd;                      // file handle
   char file_path[128];       // path of the file
   rt_tick_t reopen_after;      // save/reopen the file after a certain tick. 0 indicate do not reopen.
   rt_tick_t _last_timestamp;   // do not touch
} recorder_t;

recorder_t * recorder_create(const char file_path[], const char name[], rt_tick_t reopen_after_ticks);

void recorder_delete(recorder_t * recorder);
void recorder_delete_wait(recorder_t * recorder);

// return the num of byte written. errer if return value < 0
int recorder_write(recorder_t * recorder, const char *str);


#ifdef __cplusplus
}
#endif

#endif /* __RECORDER_H__ */
