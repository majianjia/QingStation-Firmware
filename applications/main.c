/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-02-06     Jianjia Ma       the first version
 */


#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED1_PIN    GET_PIN(C, 7)
#define LED0_PIN    GET_PIN(C, 6)

int main(void)
{
    int count = 1;
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        //LOG_D("Hello RT-Thread!");
        rt_pin_write(LED0_PIN, count % 2);
        rt_pin_write(LED1_PIN, count % 2);
        rt_thread_delay(500);
        count++;
    }

    return RT_EOK;
}

#ifdef FINSH_USING_MSH

static rt_err_t read_regs(void *bus, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void i2c_scan(int argc, char *argv[])
{
    uint8_t temp = 0;
    uint8_t addr = 0;
    uint8_t found = 0;
    struct rt_i2c_bus_device *i2c_bus;

    if(argc == 2)
    {
        i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(argv[1]);
        rt_kprintf("Trying to find devices on %s\n", argv[1]);
    }
    else {
        i2c_bus = (struct rt_i2c_bus_device *)rt_device_find("i2c2");
        rt_kprintf("Trying to find devices on %s\n", "i2c2");
    }
    if(!i2c_bus)
    {
        LOG_E("cannot find the specified i2c bus device");
        return;
    }

    while(addr < 0x7f)
    {
        if(read_regs(i2c_bus, addr, 0x00, &temp, 1) != RT_EOK)
        {
            //LOG_W("address 0x%02x, no device", addr);
        }
        else {
            LOG_I("found i2c device at 0x%02x", addr);
            found ++;
        }
        addr ++;
    }
    rt_kprintf("Found %d i2c devices\n", found);

}
/* export to msh cmd list */
MSH_CMD_EXPORT(i2c_scan, scan i2c device);
#endif

