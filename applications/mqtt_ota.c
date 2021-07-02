
/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-04-24     Jianjia Ma       the first version
 */
#include <stdio.h>
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "data_pool.h"
#include "configuration.h"

#include "drv_flash.h"
#include "mqtt_ota.h"
#include "tiny_md5.h"

#define DBG_TAG "mqtt.ota"
//#define DBG_LVL DBG_INFO
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


// Initial pack (strings)
// "version,file size,pack size,[MD5]"

// Data pack
// Pack id(2 bytes) | data size(2 bytes) | data (n-bytes) | checksum (2 bytes)

// Ack pack
// Flag (byte) | Payload

// Close pack
// "Close"

/* for ack pack */
// flag                 |   payload
// Data OK              |   pack id
// Data CRC failed      |   pack id
// Ready                |   No
// Version              |   "version"

// start from second block.
// the first block for firmware info. firmware start from the second block
#define BASE_ADDRESS  (0x08080000)
#define ERASE_SIZE    (2048)
#define APP_BASE_ADDRESS  (BASE_ADDRESS + ERASE_SIZE)

// header, the first byte of the pack.
#define INITIAL_PACK    0x01
#define DATA_PACK       0x02
#define ACK_PACK        0x03
#define CLOSE_PACK      0x04

enum {
  OTA_IDEL = 0,
  OTA_INITED,
  OTA_TRANSFERING,
  OTA_FINISHED
};

struct ota_t {
    uint8_t *pack_id;
    uint32_t file_size;
    uint32_t pack_size;
    uint32_t pack_num;
    char version[16];
    char *md5;
    int state;
};
struct ota_t ota = {0};
static uint8_t recv_buf[384];
static uint16_t buf_len = 0;
static rt_sem_t sem_rec = NULL;

// called after the initial package received
int mqtt_ota_prepare(struct ota_t *ota, char *msg)
{
    char* saveptr = NULL;
    char *ver = NULL;
    char *fwsize = NULL;
    char *pksize = NULL;
    char *md5 = NULL;

    // write message to the first block first, buffer will be destroyed later
    stm32_flash_erase(BASE_ADDRESS, ERASE_SIZE);
    stm32_flash_write(BASE_ADDRESS, (uint8_t*)msg, MIN((strlen(msg)+1), ERASE_SIZE));

    // initial ota
    ver = strtok_r((char*)msg, ", ", &saveptr); // version
    fwsize = strtok_r(NULL, ", ", &saveptr);
    pksize = strtok_r(NULL, ", ", &saveptr);
    md5 = (strtok_r(NULL, ", ", &saveptr) - msg) + (void*)BASE_ADDRESS; // use the flash version.
    printf("Updating to version: %s, MD5:%s\n", ver, md5);

    // init new
    ota->pack_size = atoi(pksize);
    ota->file_size = atoi(fwsize);
    ota->pack_num = ota->pack_size/ota->file_size + MIN(ota->pack_size % ota->file_size, 1);
    ota->md5 = md5;
    ota->pack_id = malloc(ota->pack_num / 8 + 1);
    if(!ota->pack_id)
        return -1;
    memset(ota->pack_id, 0, ota->pack_num / 8 + 1);
    ota->state = OTA_INITED;
    return 0;
}

int mqtt_ota_receive(struct ota_t *ota, uint8_t *data, uint32_t len)
{
    uint16_t pack_id = ((uint16_t)data[2])<<8 | data[1];
    uint16_t size = ((uint16_t)data[4])<<8 | data[3];
    uint16_t cs = ((uint16_t)data[6+size])<<8 | data[5+size]; // check sum
    uint16_t sum = 0;
    for(int i=0; i<len-2; i++)
        sum += data[i];
    if(sum != cs)
        return -1;

    // start programming
    uint32_t start_addr = (pack_id * ota->pack_size) + APP_BASE_ADDRESS;
    if(start_addr % ERASE_SIZE == 0)
        stm32_flash_erase(start_addr, ERASE_SIZE);
    // write
    stm32_flash_write(start_addr, &data[5], size);
    printf("\rwriting data to 0x%x, size %d", start_addr, size);
    return 0;
}

int ota_invalidate(int a, char* av[])
{
   return stm32_flash_erase(BASE_ADDRESS, ERASE_SIZE);
}
MSH_CMD_EXPORT(ota_invalidate, destroy ota signiture)

int mqtt_ota_end(struct ota_t *ota)
{
    char* saveptr = NULL;
    char* md5;
    tiny_md5_context *ctx;
    int fw_size;
    uint8_t output[16] = {0};
    // validate the fw
    strncpy((char*)recv_buf, (char*)(BASE_ADDRESS), sizeof(recv_buf));
    strtok_r((char*)recv_buf, ", ", &saveptr); // version
    fw_size = atoi(strtok_r(NULL, ", ", &saveptr)); // filesize
    strtok_r(NULL, ", ", &saveptr); // pack size
    md5 = strtok_r(NULL, ", ", &saveptr); // md5

    printf("OTA FW tag: %s\n", (char*)(BASE_ADDRESS));
    // start validation
    ctx = malloc(sizeof(tiny_md5_context));
    if(ctx)
    {
        memset(ctx, 0, sizeof(tiny_md5_context));
        tiny_md5_starts(ctx);
        tiny_md5_update(ctx, (void*)APP_BASE_ADDRESS, fw_size);
        tiny_md5_finish(ctx, output);
        printf("OTA firmware    MD5: %s\n", md5);
        printf("Local validated MD5: ");
        for(int i=0; i<16; i++)
            printf("%02x", output[i]);
        printf("\n");
        free(ctx);
    }
    else {
        LOG_E("no memory to validate md5");
    }

    if(ota->pack_id)
        free(ota->pack_id);
    ota->state = OTA_IDEL;

    // test
    printf("Rebooting the device to update firmware.\n");
    rt_thread_delay(500);
    rt_hw_cpu_reset();

    return 0;
}

int mqtt_ota_ack(uint16_t packid, bool is_correct)
{
    int mqtt_publish_data(const char topic[], char value[], int qs);
    char buf[16];
    snprintf(buf, 16, "%d,%s", packid, is_correct==false?"false":"true");
    mqtt_publish_data(MQTT_OTA_UPSTREAM, buf, 0);
    return 0;
}

void mqtt_ota_receive_callback(uint8_t *data, uint32_t len)
{
    memcpy(recv_buf, data, MIN(len, sizeof(recv_buf)));
    buf_len = len;
    rt_sem_release(sem_rec);
}

void ota_thread(void *parameters)
{
    int rslt = 0;

    rt_thread_delay(1000);
    //mqtt_ota_end(&ota);

    while(1)
    {
        rt_sem_take(sem_rec, RT_WAITING_FOREVER);

        switch(recv_buf[0])
        {
        case INITIAL_PACK:{
            recv_buf[buf_len] = '\0';
            rslt = mqtt_ota_prepare(&ota, &recv_buf[1]);
            if(rslt == 0)
                mqtt_ota_ack(0, true);
        }break;
        case DATA_PACK:{
            uint16_t pack_id = ((uint16_t)recv_buf[2])<<8 | recv_buf[1];
            rslt = mqtt_ota_receive(&ota, recv_buf, buf_len);
            if(rslt == 0)
                mqtt_ota_ack(pack_id, true);
            else {
                mqtt_ota_ack(pack_id, false);
            }
        }break;
        case CLOSE_PACK:
            mqtt_ota_end(&ota);
            mqtt_ota_ack(0, true);
            break;
        default:break;
        }
    }
}

int ota_init(void)
{
    sem_rec = rt_sem_create("ota", 0, RT_IPC_FLAG_PRIO);
    rt_thread_t tid = rt_thread_create("ota", ota_thread, RT_NULL, 2048, 11, 1000);
    if(!tid)
        return -1;
    rt_thread_startup(tid);
    return 0;
}
INIT_APP_EXPORT(ota_init);

int md5(int argc, char* argv[])
{
    uint8_t output[16] = {0};
    if(argc != 2)
        return -1;

//    HASH_HandleTypeDef     HashHandle;
//    HAL_HASH_DeInit(&HashHandle);
//    HashHandle.Init.DataType = HASH_DATATYPE_8B;
//    HAL_HASH_Init(&HashHandle);
//
//    /* Compute SHA1 */
//    HAL_HASH_MD5_Start(&HashHandle, (uint8_t *)argv[1], strlen(argv[1]), output, 0xFF);
//
//    printf("%s \nMD5: \n", argv[1]);
//    for(int i=0; i<16; i++)
//        printf("%x", output[i]);
//    printf("\n");

    tiny_md5(argv[1], strlen(argv[1]), output);
    printf("%s \nMD5: \n", argv[1]);
    for(int i=0; i<16; i++)
        printf("%02x", output[i]);
    printf("\n");
    return 0;
}
MSH_CMD_EXPORT(md5, calculate md5 of a string)


