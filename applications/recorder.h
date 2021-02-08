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

typedef struct _recorder_t
{
   rt_thread_t tid;
   rt_mq_t msg;
   char* buf;
   uint32_t max_msg_size;
   bool is_closed;
   uint32_t file_size;
   int32_t error_code;
   int fd;                      // file handle
   const char *file_path;       // path of the file
   rt_tick_t reopen_after;      // save/reopen the file after a certain tick. 0 indicate do not reopen.
   rt_tick_t _last_timestamp;   // do not touch
} recorder_t;


recorder_t * recorder_create(const char file_path[], const char name[], uint32_t msg_size, rt_tick_t reopen_after_ticks);

void recorder_delete(recorder_t * recorder);

// return the num of byte written. errer if return value < 0
int recorder_write(recorder_t * recorder, const char *str);


#ifdef __cplusplus
}
#endif

#endif /* __RECORDER_H__ */
