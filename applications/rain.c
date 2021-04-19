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
#include <rtdevice.h>
#include <board.h>
#include "configuration.h"
#include "data_pool.h"
#include <stdlib.h>
#include "string.h"
#include "math.h"

#define DBG_TAG "rain"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define IR_LED_PIN  GET_PIN(A, 4)

typedef struct data_buffer {
    int16_t *buf;
    int32_t idx;
    int32_t size;
    bool is_full;
} data_buffer_t;
data_buffer_t rain_data;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
ADC_ChannelConfTypeDef ADC_ChanConf;

static void MX_ADC1_Init(void)
{
    do{
        rt_thread_delay(100);
        hadc1.Instance = ADC1;
        HAL_ADC_DeInit(&hadc1);

        hadc1.Instance = ADC1;
        hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
        hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.ScanConvMode = DISABLE;
        hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        hadc1.Init.LowPowerAutoWait = DISABLE;
        hadc1.Init.ContinuousConvMode = DISABLE;
        hadc1.Init.NbrOfConversion = 1;
        hadc1.Init.DiscontinuousConvMode = DISABLE;
        hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc1.Init.DMAContinuousRequests = DISABLE;
        hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
        hadc1.Init.OversamplingMode = DISABLE;
    } while( HAL_ADC_Init(&hadc1) != HAL_OK); // sometimes this fail.
}

int get_adc_value(uint32_t channel)
{
    ADC_ChanConf.Channel =  channel;
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    ADC_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
    ADC_ChanConf.SingleDiff = LL_ADC_SINGLE_ENDED;
    HAL_ADC_ConfigChannel(&hadc1, &ADC_ChanConf);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);

    return HAL_ADC_GetValue(&hadc1);;
}


int measure_rain()
{
    uint16_t adc;
    rt_pin_write(IR_LED_PIN, GPIO_PIN_SET);
    rt_thread_mdelay(1);
    adc = get_adc_value(ADC_CHANNEL_6);
    rt_pin_write(IR_LED_PIN, GPIO_PIN_RESET);
    return adc;
}

int measure_sys_voltage()
{
    return get_adc_value(ADC_CHANNEL_4);
}


int add_to_buffer(data_buffer_t* data, int32_t new)
{
    data->buf[data->idx] = new;
    data->idx ++;
    if(!data->is_full && data->idx == data->size)
        data->is_full = true;
    if(data->idx >= data->size)
        data->idx = 0;
    return 0;
}

float compute_variance(data_buffer_t *data)
{
    float var = 0;
    float avg = 0;
    for(int i=0; i<data->size; i++)
        avg += data->buf[i];
    avg /= data->size;
    for(int i=0; i<data->size; i++)
    {
        float diff = data->buf[i] - avg;
        var += diff * diff;
    }
    var /= data->size - 1;
    var = sqrtf(var);
    return var;
}


void thread_rain(void* parameters)
{
    sensor_config_t * cfg;
    uint16_t rain_raw;
    uint16_t sys_vol_raw;

    rt_pin_mode(IR_LED_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(IR_LED_PIN, GPIO_PIN_RESET);

    // wait until system cfg loaded
    // wait and load the configuration
    do{
        rt_thread_delay(100);
        cfg = get_sensor_config("Rain");
    }while(cfg == NULL && !is_system_cfg_valid());

    // init the data
    int len = cfg->oversampling * 10 *cfg->data_period / 1000; // 10 second windows
    memset(&rain_data, 0, sizeof(data_buffer_t));
    rain_data.buf = malloc(len*2);
    rain_data.size = len;
    if(rain_data.buf == NULL){
        LOG_E("no memory for rain sensor data buffer, require %d bytes", len*2);
        return;
    }

    MX_ADC1_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    int period = cfg->data_period / cfg->oversampling;
    while(1)
    {
        rt_thread_mdelay(2);
        rt_thread_mdelay(period - rt_tick_get()%period);

        // calculate the variance
        rain_raw = measure_rain();
        add_to_buffer(&rain_data, rain_raw);
        if(rain_data.is_full)
        {
            rain.rain_var = compute_variance(&rain_data);
            data_updated(&rain.info);
        }
        //printf("measurement:%d, rain_var: %f\n", rain_raw, rain.rain_var);

        // temporary place the system voltage sending here
        // since our ADC is not using RTT's framework, so not thread safe yet.
        sys_vol_raw = measure_sys_voltage();
        sys.bat_voltage = sys_vol_raw / 4096.f * 2.f * 3.3f;
        data_updated(&sys.info);
    }
}


int thread_rain_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("rain", thread_rain, RT_NULL, 1024, 12, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_rain_init);


void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}



