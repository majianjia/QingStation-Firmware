
/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-03-20     Jianjia Ma       the first version
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "data_pool.h"
#include "configuration.h"
#include "stm32l4xx_ll_utils.h"

#define DBG_TAG "mqtt"
//#define DBG_LVL DBG_INFO
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "mqtt_client.h"
#include "mqtt_ota.h"

#include <at_device_esp32.h>
#include <at_device_esp8266.h>
#include <at_device_sim800c.h>

#include "wifi_password.h"

//#define TEST_WITH_LOCAL_BROKER
#define LOCAL_URI "tcp://192.168.1.87:1883"

// #define MQTT_URI                "tcp://test.mosquitto.org:1883"
// #define MQTT_URI                "tcp://mq.tongxinmao.com:18831"
//#define MQTT_URI                "tcp://broker.emqx.io:1883"
#define MQTT_URI                "tcp://qbe26b46.en.emqx.cloud:11523"
#define MQTT_PUBTOPIC           "state"
#define MQTT_SUBTOPIC           "state"
#define MQTT_WILLMSG            "Bye!"

#define ESP8266_DEIVCE_NAME     "esp0"
#define ESP8266_CLIENT_NAME     "lpuart1"
#define ESP8266_RECV_BUFF_LEN    (512)

static struct at_device_esp8266 esp0 = {0};
static struct at_device_esp32 esp32 = {0};

// store locally for security. i.e. not export to default config cjson
static char ssid[sizeof(system_config.mqtt.wifi_ssid)] = {0};
static char password[sizeof(system_config.mqtt.wifi_password)] = {0};
static char mqtt_username[sizeof(system_config.mqtt.mqtt_username)] = {0};
static char mqtt_password[sizeof(system_config.mqtt.mqtt_password)] = {0};
static char uri[128] = {0};

static int esp8266_device_register(void)
{
    struct at_device_esp8266 *esp8266 = &esp0;

    if(strlen(system_config.mqtt.wifi_ssid) != 0)
    {
        strncpy(ssid, system_config.mqtt.wifi_ssid, sizeof(ssid));
        strncpy(password, system_config.mqtt.wifi_password, sizeof(password));
        if(strlen(ssid) == 0)
        {
            LOG_E("No ssid set, please set SSID for wifi connection (sd/config.json/mqtt/wifi_ssid)");
            return -1;
        }
        esp8266->wifi_ssid = ssid;
        esp8266->wifi_password = password;
    }
    else {
        LOG_I("SSID set to firmware default.");
        esp8266->wifi_ssid = ESP8266_WIFI_SSID;
        esp8266->wifi_password = ESP8266_WIFI_PASSWORD;
    }

    esp8266->client_name = system_config.mqtt.interface;
    esp8266->device_name = ESP8266_DEIVCE_NAME;
    esp8266->recv_line_num = ESP8266_RECV_BUFF_LEN;

    return at_device_register(&(esp8266->device),
                              esp8266->device_name,
                              esp8266->client_name,
                              AT_DEVICE_CLASS_ESP8266,
                              (void *) esp8266);
}


static int esp32_device_register(void)
{
    struct at_device_esp32 *esp = &esp32;

    if(strlen(system_config.mqtt.wifi_ssid) != 0)
    {
        strncpy(ssid, system_config.mqtt.wifi_ssid, sizeof(ssid));
        strncpy(password, system_config.mqtt.wifi_password, sizeof(password));
        if(strlen(ssid) == 0)
        {
            LOG_E("No ssid set, please set SSID for wifi connection (sd/config.json/mqtt/wifi_ssid)");
            return -1;
        }
        esp->wifi_ssid = ssid;
        esp->wifi_password = password;
    }
    else {
        LOG_I("SSID set to firmware default.");
        esp->wifi_ssid = ESP8266_WIFI_SSID;
        esp->wifi_password = ESP8266_WIFI_PASSWORD;
    }

    esp->client_name = system_config.mqtt.interface;
    esp->device_name = ESP8266_DEIVCE_NAME;
    esp->recv_line_num = ESP8266_RECV_BUFF_LEN;

    return at_device_register(&(esp->device),
            esp->device_name,
            esp->client_name,
            AT_DEVICE_CLASS_ESP32,
            (void *) esp);
}


#define SIM800C_DEIVCE_NAME     "sim0"
#define SIM800C_POWER_PIN       (-1)
#define SIM800C_STATUS_PIN      (-1)
#define SIM800C_RECV_BUFF_LEN   (512)

static struct at_device_sim800c sim0 = {0};

static int sim800c_device_register(void)
{
    struct at_device_sim800c *sim800c = &sim0;
    sim800c->client_name = system_config.mqtt.interface;
    sim800c->device_name = SIM800C_DEIVCE_NAME;
    sim800c->recv_line_num = SIM800C_RECV_BUFF_LEN;
    sim800c->power_pin = SIM800C_POWER_PIN;
    sim800c->power_status_pin = SIM800C_STATUS_PIN;

    return at_device_register(&(sim800c->device),
                              sim800c->device_name,
                              sim800c->client_name,
                              AT_DEVICE_CLASS_SIM800C,
                              (void *) sim800c);
}

extern CRC_HandleTypeDef hcrc;
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  HAL_CRC_Init(&hcrc);
}


/* define MQTT client context */
static mqtt_client client;
static int is_started = 0;
static int is_connected = 0;

static void mqtt_sub_callback(mqtt_client *c, message_data *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    LOG_D("mqtt sub callback: %.*s %.*s",
               msg_data->topic_name->lenstring.len,
               msg_data->topic_name->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);
}

static void mqtt_ota_callback(mqtt_client *c, message_data *msg_data)
{
    mqtt_ota_receive_callback(msg_data->message->payload, msg_data->message->payloadlen);
}


static void mqtt_sub_default_callback(mqtt_client *c, message_data *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    LOG_D("mqtt sub default callback: %.*s %.*s",
               msg_data->topic_name->lenstring.len,
               msg_data->topic_name->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);
}

static void mqtt_connect_callback(mqtt_client *c)
{
    LOG_D("MQTT connecting");
}

static void mqtt_online_callback(mqtt_client *c)
{
    LOG_D("MQTT online");
    is_connected = 1;
}

static void mqtt_offline_callback(mqtt_client *c)
{
    LOG_D("MQTT offline");
    is_connected = 0;
}

static void mqtt_new_sub_callback(mqtt_client *client, message_data *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    LOG_D("mqtt new subscribe callback: %.*s %.*s",
               msg_data->topic_name->lenstring.len,
               msg_data->topic_name->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);
}

static int mqtt_subscribe(int argc, char **argv)
{
    if (argc != 2){
        rt_kprintf("mqtt_subscribe [topic]  --send an mqtt subscribe packet and wait for suback before returning.\n");
        return -1;
    }
    if (is_started == 0){
        LOG_E("mqtt client is not connected.");
        return -1;
    }
    return paho_mqtt_subscribe(&client, QOS1, argv[1], mqtt_new_sub_callback);
}

static int mqtt_unsubscribe(int argc, char **argv)
{
    if (argc != 2){
        rt_kprintf("mqtt_unsubscribe [topic]  --send an mqtt unsubscribe packet and wait for suback before returning.\n");
        return -1;
    }
    if (is_started == 0){
        LOG_E("mqtt client is not connected.");
        return -1;
    }
    return paho_mqtt_unsubscribe(&client, argv[1]);
}



static int mqtt_start(int argc, char **argv)
{
    /* init condata param by using MQTTPacket_connectData_initializer */
    MQTTPacket_connectData condata = MQTTPacket_connectData_initializer;
    static char cid[20] = { 0 };

    if (argc != 1)
    {
        rt_kprintf("mqtt_start    --start a mqtt worker thread.\n");
        return -1;
    }

    if (is_started)
    {
        LOG_E("mqtt client is already connected.");
        return -1;
    }
    /* config MQTT context param */
    {
        // UUID is too long, we use 32bit CRC as id.
        uint32_t uuid[3];
        uint32_t id;
        uuid[0] = LL_GetUID_Word0();
        uuid[1] = LL_GetUID_Word1();
        uuid[2] = LL_GetUID_Word2();
        MX_CRC_Init();
        id = HAL_CRC_Calculate(&hcrc, uuid, 3); // word

        client.isconnected = 0;
        /* generate the random client ID */
        rt_snprintf(cid, sizeof(cid), "Qing%u", id);
        /* config connect param */
        memcpy(&client.condata, &condata, sizeof(condata));
        client.condata.clientID.cstring = cid;
        client.condata.keepAliveInterval = 30;
        client.condata.cleansession = 1;

        #ifdef TEST_WITH_LOCAL_BROKER
            client.uri = LOCAL_URI;
        #else
            //client.uri = MQTT_URI;
            if(strlen(system_config.mqtt.uri) && system_config.mqtt.port){
                sprintf(uri, "tcp://%s:%d", system_config.mqtt.uri, system_config.mqtt.port);
                client.uri = uri;
            }
            else {
                LOG_W("Please config the MQTT uri and port in config.json");
                client.uri = MQTT_URI;
            }
        #endif
            if(strlen(system_config.mqtt.mqtt_username) != 0)
            {
                strncpy(mqtt_username, system_config.mqtt.mqtt_username, sizeof(mqtt_username));
                strncpy(mqtt_password, system_config.mqtt.mqtt_password, sizeof(mqtt_password));
                client.condata.username.cstring = mqtt_username;
                client.condata.password.cstring = mqtt_password;
            }
            else{
        #ifndef TEST_WITH_LOCAL_BROKER
                client.condata.username.cstring = MQTT_USERNAME;
                client.condata.password.cstring = MQTT_PASSWORD;
        #endif
            }

        /* config MQTT will param. */
        client.condata.willFlag = 1;
        client.condata.will.qos = 1;
        client.condata.will.retained = 0;
        client.condata.will.topicName.cstring = MQTT_PUBTOPIC;
        client.condata.will.message.cstring = MQTT_WILLMSG;

        /* malloc buffer. */
        client.buf_size = 512;     // send
        client.readbuf_size = 512; // receive
        client.buf = rt_calloc(1, client.buf_size);
        client.readbuf = rt_calloc(1, client.readbuf_size);
        if (!(client.buf && client.readbuf))
        {
            LOG_E("no memory for MQTT client buffer!");
            return -1;
        }

        /* set event callback function */
        client.connect_callback = mqtt_connect_callback;
        client.online_callback = mqtt_online_callback;
        client.offline_callback = mqtt_offline_callback;

        /* set subscribe table and event callback */
        client.message_handlers[0].topicFilter = rt_strdup(MQTT_SUBTOPIC);
        client.message_handlers[0].callback = mqtt_sub_callback;
        client.message_handlers[0].qos = QOS0; //

        /* for OTA */
        client.message_handlers[1].topicFilter = rt_strdup(MQTT_OTA_DOWNSTREAM);
        client.message_handlers[1].callback = mqtt_ota_callback;
        client.message_handlers[1].qos = QOS0;

        /* set default subscribe event callback */
        client.default_message_handlers = mqtt_sub_default_callback;
    }

    {
      int value;
      uint16_t u16Value;
      value = 5;
      paho_mqtt_control(&client, MQTT_CTRL_SET_CONN_TIMEO, &value);
      value = 5;
      paho_mqtt_control(&client, MQTT_CTRL_SET_MSG_TIMEO, &value);
      value = 5;
      paho_mqtt_control(&client, MQTT_CTRL_SET_RECONN_INTERVAL, &value);
      value = 30;
      paho_mqtt_control(&client, MQTT_CTRL_SET_KEEPALIVE_INTERVAL, &value);
      u16Value = 3;
      paho_mqtt_control(&client, MQTT_CTRL_SET_KEEPALIVE_COUNT, &u16Value);
    }

    /* run mqtt client */
    paho_mqtt_start(&client, 1536, 15); // priority need to be reasonable high. what happend if higher than AT clnt
    is_started = 1;
    return 0;
}

static int mqtt_stop(int argc, char **argv)
{
    if (argc != 1){
        rt_kprintf("mqtt_stop    --stop mqtt worker thread and free mqtt client object.\n");
    }
    is_started = 0;
    return paho_mqtt_stop(&client);
}

static int mqtt_publish(int argc, char **argv)
{
    if (is_started == 0)
    {
        LOG_E("mqtt client is not connected.");
        return -1;
    }
    if (argc == 2)
    {
        paho_mqtt_publish(&client, QOS1, MQTT_PUBTOPIC, argv[1], strlen(argv[1]));
    }
    else if (argc == 3)
    {
        paho_mqtt_publish(&client, QOS1, argv[1], argv[2],strlen(argv[2]));
    }
    else
    {
        rt_kprintf("mqtt_publish <topic> [message]  --mqtt publish message to specified topic.\n");
        return -1;
    }
    return 0;
}

int mqtt_publish_data(const char topic[], char value[], int qs)
{
    if(is_connected) // hope it can minimized the error rate.
        return paho_mqtt_publish(&client, qs, topic, value, MIN(strlen(value), 256));
    return -1;
}

#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(mqtt_start, startup mqtt client);
MSH_CMD_EXPORT(mqtt_stop, stop mqtt client);
MSH_CMD_EXPORT(mqtt_publish, mqtt publish message to specified topic);
MSH_CMD_EXPORT(mqtt_subscribe,  mqtt subscribe topic);
MSH_CMD_EXPORT(mqtt_unsubscribe, mqtt unsubscribe topic);
#endif /* FINSH_USING_MSH */

static uint64_t msg_count = 0;
static float msg_rate = 0;
static int  mqtt_state(int argc, char **argv)
{
    if(argc != 1)
        rt_kprintf("mqtt_state :print the state of mqtt\n");

    rt_kprintf(" connect timeout: %d\n", client.connect_timeout);
    rt_kprintf(" keep alive count: %d\n", client.keepalive_count);
    rt_kprintf(" keep alive counter: %d\n", client.keepalive_counter);
    rt_kprintf(" keep alive interval: %d\n", client.keepalive_interval);
    rt_kprintf(" reconnect interval: %d\n", client.reconnect_interval);
    rt_kprintf(" uri: %s\n", client.uri);
    rt_kprintf(" user ID: %s", client.condata.clientID);
    rt_kprintf(" username: %s password: %s\n", client.condata.username.cstring, client.condata.password.cstring);
    rt_kprintf(" is connected: %d\n", client.isconnected);

    rt_kprintf(" msg sent: %u\n", (uint32_t)msg_count);
    printf(" msg rate: %.1f/min\n", msg_rate);
    return 0;
}
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(mqtt_state, print the state of mqtt);
#endif



void thread_mqtt(void* p)
{
    #define BUFSIZE  64
    char topic[BUFSIZE] = {0};
    char line[BUFSIZE] = "test";
    int data_len = 0;
    uint16_t orders[64] = {0};
    float last_data[64] = {0}; // whether the data is updated.
    rt_tick_t last_full_update = rt_tick_get();
    bool is_full_update = false;
    mqtt_config_t *cfg;
    int rslt = 0;
    uint64_t msg_count_last = msg_count;
    rt_tick_t msg_count_tick;

    // wait until system cfg loaded
    // wait and load the configuration
    do{
        rt_thread_delay(100);
    }while(!is_system_cfg_valid());
    cfg = &system_config.mqtt;

    //cfg->is_enable = false;
    while(!cfg->is_enable)
        rt_thread_delay(1000);

    //rt_thread_delay(5000);

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_t serial = rt_device_find(cfg->interface);
    //rt_device_open(serial, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_DMA_RX);
    rt_device_open(serial, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);

    config.baud_rate  = 57600;// cfg->baudrate; // do not configer higher than this.
    if(config.baud_rate > 57600)
        LOG_W("Baudrate[%d], higher than 56700bps cause unstable AT links, please lower the baudrate.", cfg->baudrate);
    if(RT_EOK != rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config))
        LOG_E("change baudrate %d, %s failed!", cfg->baudrate, cfg->interface );
    else
        LOG_I("MQTT set to uart: %s, baudrate is set to %dbps", cfg->interface, cfg->baudrate);

    // clear buffer.
//    while(0 != rt_device_read(serial, 0, line, sizeof(line)))
//        printf("buffer left: %s\n", line);

    rt_thread_mdelay(2000);

    LOG_I("Setting up MQTT AT module: %s", system_config.mqtt.module);
    if(!strcasecmp("esp8266", system_config.mqtt.module))
        esp8266_device_register();
    else if(!strcasecmp("esp32", system_config.mqtt.module))
        esp32_device_register();
    else if(!strcasecmp("sim800c", system_config.mqtt.module))
        sim800c_device_register();
    else
        LOG_E("MQTT does not support %s module, please check config/mqtt/module.", system_config.mqtt.module);

    // find out the data to be export (publish)
    if(strlen(system_config.mqtt.pub_data) == 0)
    {
        data_len = EXPORT_DATA_SIZE-1; // data 0 is "unknown"
        for(int i=0; i<data_len; i++)
            orders[i] = i+1;
    }
    // if the field is empty, then we print all data.
    else{
        char *str_buf = malloc(512);
        strncpy(str_buf, system_config.mqtt.pub_data, 512);
        data_len = get_data_orders(str_buf, ", ", orders, 64);
        free(str_buf);
    }

    // wait for SAL and AT device.
    rt_thread_mdelay(5000);

    // start mqtt
    mqtt_start(1, NULL);
    rt_thread_mdelay(2000);

    msg_count_tick = rt_tick_get();

    int period = cfg->period;
    while(1)
    {
        rt_thread_mdelay(period - rt_tick_get()%period);

        // fully update data every minute
        if((int)(rt_tick_get() - last_full_update) > 60 * RT_TICK_PER_SECOND)
        {
            last_full_update = rt_tick_get();
            is_full_update = true;

            // message rate
            if(msg_count != msg_count_last){
                msg_rate = (float)(msg_count - msg_count_last)*60*RT_TICK_PER_SECOND
                        / (rt_tick_get() - msg_count_tick);
                msg_count_last = msg_count;
                msg_count_tick = rt_tick_get();
            }
        }
        else
            is_full_update = false;

        // now send data if they are updated.
        for(int i=0; i< data_len; i++)
        {
            // a simple check for whether the data is updated
            if(!is_full_update){
                //printf("%d, %d, %f\n",i, orders[i], get_data[orders[i]]());
                if(last_data[orders[i]] != get_data[orders[i]]())
                    last_data[orders[i]] = get_data[orders[i]]();
                else
                   continue;
            }

            if(is_connected)
            {
                snprintf(topic, sizeof(topic), "%s%s", cfg->topic_prefix, data_name[orders[i]]);
                print_data[orders[i]](line);
                rslt = mqtt_publish_data(topic, line, 0);
                //printf("%d, %d, %s\n", i, orders[i], line);
                if(rslt != 0)
                {
                    // reconnected needed.
                    is_connected = 0;
                    LOG_E("publish fail, wait for reconnect");
                    rt_thread_mdelay(2000);
                }
                else {
                    msg_count++;
                }
                //printf(line);
            }
            rt_thread_delay(1); // this is needed for more stable AT device
        }
    }
}


int thread_mqtt_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("mqtt", thread_mqtt, RT_NULL, 2048, 20, 1000);
    if(!tid)
        return -RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_mqtt_init);

