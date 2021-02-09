/*
 * Change Logs:
 * Date           Author        Notes
 * 2020-05-12     Jianjia Ma    first version
 */

#ifndef ___DRV_BMX160_H__
#define ___DRV_BMX160_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "imu.h"

imu_dev_t * bmx160_register(rt_device_t bus);

#ifdef __cplusplus
}
#endif

#endif /* ___DRV_BMX160_H__ */
