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

#ifndef APPLICATIONS_MQTT_OTA_H_
#define APPLICATIONS_MQTT_OTA_H_

#define MQTT_OTA_UPSTREAM     "ota_upstream"
#define MQTT_OTA_DOWNSTREAM   "ota_downstream"


void mqtt_ota_receive_callback(uint8_t *data, uint32_t len);

#endif /* APPLICATIONS_MQTT_OTA_H_ */
