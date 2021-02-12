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
#include <rtdevice.h>
#include <board.h>
#include "drv_anemometer.h"
#include "recorder.h"
#include "stdio.h"
#include "math.h"

#define DBG_TAG "anemo"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifndef MAX
#define MAX(a, b) (a>b?a:b)
#endif
#ifndef MIN
#define MIN(a, b) (a<b?a:b)
#endif

static rt_sem_t sem_ready;
static rt_timer_t timer;

static void timer_tick(void* parameter){
    rt_sem_release(sem_ready);
}

// return the offset of the pattern.
uint32_t match_filter_fixedpoint_idx(int16_t* pattern, uint32_t pattern_len, int16_t* signal, uint32_t signal_len)
{
   int32_t sum = 0;
   int32_t max = 0;
   uint32_t idx = 0;
   for(uint32_t i=0; i<signal_len-pattern_len; i++)
   {
       for(uint32_t j=0; j<pattern_len; j++)
           sum += pattern[j] * signal[i+j];

       if(sum > max)
       {
           max = sum;
           idx = i;
       }
       sum = 0;
   }
   return idx;
}

// return offset
uint32_t match_filter(float* signal, uint32_t signal_len, float* pattern, uint32_t pattern_len, float* output)
{
   float max = 0;
   float sum = 0;
   uint32_t idx = 0;
   for(uint32_t i=0; i<signal_len-pattern_len; i++)
   {
       for(uint32_t j=0; j<pattern_len; j++)
           sum += pattern[j] * signal[i+j];
       if(output)
           output[i] = sum;
       if(sum > max)
       {
           max = sum;
           idx = i;
       }
       sum = 0;
   }
   return idx;
}

// to normalize the pattern
void normalize(float* pattern, uint32_t len)
{
    float sum=0;
    for(uint32_t i=0; i<len; i++)
        sum += abs(pattern[i]); // need to decide which function to normalize here.
    sum = sum /len;
    for(uint32_t i=0; i<len; i++)
        pattern[i] = pattern[i] * sum;
}

// find the exact interpolation
// output the offset of the signal in sub-digit resolution.
int linear_interpolation_zerocrossing(float* sig, uint32_t sig_len, float* out, uint32_t num_zero_cross)
{
    #define IS_SIGN_DIFF(a, b) (a * b < 0)
    uint32_t cross = 0;
    for(uint32_t i=0; i<sig_len-1 && cross<num_zero_cross; i++)
    {
        if(IS_SIGN_DIFF(sig[i], sig[i+1]))
        {
            // do interpolation using y=ax+b
            float a = sig[i+1] - sig[i];    // a=(y2 - y1)/(x2 - x1) while x2 - x1 = 1
            float b = sig[i];               // b=y1
            float x = -b/a;                 // x=(y-b)/a zero crossing.
            out[cross] = x + i;
            // cross
            cross++;
        }
    }
    return  cross;
}

// see if needed. to filter where the 2 points are within a distance.
uint32_t match_point(float *p1, float *p2, uint32_t len, float threshold)
{
    bool is_matched = false;
    uint32_t matched_size;
    for(uint32_t i=0; i<len; i++)
    {
        for(uint32_t j=0; j<len; j++){
            if(abs(p1[i] - p2[j]) <= threshold){
                is_matched = true;
                break;
            }
        }
        // if not matched, delete the number in P1
        if(!is_matched){
            p1[i] = 0;
        }
        is_matched = false;
    }

    // delete all zero in p1;
    for(uint32_t i=0; i<len; )
    {
        if(p1[i] == 0)
        {
            memmove(&p1[i], &p1[i+1], (len-i-1)*sizeof(float));
            continue;
        }
        matched_size++;
        i++;
    }
    // if fully matched, nothing changed.
    if(matched_size == len)
        return matched_size;

    // now delete unmatched p2
    for(uint32_t i=0; i<len; )
    {
        if(abs(p1[i] - p2[i]) <= threshold){
            memmove(&p2[i], &p2[i+1], (len-i-1)*sizeof(float));
            continue;
        }
        i++;
    }
    return matched_size;
}

// find the max index
uint32_t arg_maxf(float* sig, uint32_t len)
{
    float max = sig[0];
    uint32_t arg = 0;

    for(uint32_t i=0; i<len; i++)
    {
        if(sig[i] > max)
        {
            arg = i;
            max = sig[i];
        }
    }
    return arg;
}

// insert sort
static void sort(float arr[], uint32_t len)
{
    uint32_t i,j;
    for (i=1; i<len;i++)
    {
        float temp = arr[i];
        for (j=i; j>0 && arr[j-1]>temp; j--)
            arr[j] = arr[j-1];
        arr[j] = temp;
    }
}

// Forward propagation signal, backward signal, both should be aligned and shifted to zero.
// signal should exclude the direct sound. -> only include the reflection wave. Offset need to be made before signal in.
// This function measure the delay different between 2 signals.
// return the delay in sub sample, positive = backward > forward.
float measure_signal_diff(float* forward, float* backward, uint32_t sig_len, float* pattern_buf, uint32_t pattern_len)
{
    // we use forward as pattern.
    // cut a few raising cycle before the maximum point
    int32_t pattern_start;
    float * pattern = pattern_buf;
    pattern_start = arg_maxf(forward, sig_len) - pattern_len;
    if(pattern_start < 0)
        return 0;
    memcpy(pattern, &forward[pattern_start], sizeof(float)*pattern_len);

    // find the signal match for the backward
    int32_t backward_start = match_filter(backward, sig_len, pattern, pattern_len, NULL);

    // the signal offset in digit resolution : offset = backward_matched - forward_matched.
    float sig_offset = backward_start - pattern_start;

    // next, we use interpolation on zero cross moment to improve the the offset to sub digit resolution.
    #define NUM_CROSS       10
    float fw_cross[NUM_CROSS];
    float bw_cross[NUM_CROSS];
    // get the find zero crossing
    uint32_t fw_cross_num = linear_interpolation_zerocrossing(
            &forward[pattern_start], sig_len-pattern_start, fw_cross, NUM_CROSS);
    uint32_t bw_cross_num = linear_interpolation_zerocrossing(
            &backward[backward_start], sig_len-backward_start, bw_cross, NUM_CROSS);

    // get the average of all crossing difference. (also want to add a median filter. )
    float sub_digit_diff = 0;
    uint32_t num_of_cross = MIN(bw_cross_num, fw_cross_num);
    float zero_cross[NUM_CROSS];
    for(uint32_t i=0; i<num_of_cross; i++){
        zero_cross[i] = bw_cross[i] - fw_cross[i];
    }
    // get rid of a few max and min, use the rest for average.
    sort(zero_cross, num_of_cross);
    // average
    for(uint32_t i=2; i<num_of_cross-2; i++){
        sub_digit_diff += zero_cross[i];
    }
    sub_digit_diff = sub_digit_diff/(num_of_cross - 4); // in a good match, this should be within -1 to 1

    return sig_offset + sub_digit_diff;
}

// this function measure the propagation timing t
// t: from first ADC measurement to the first matched zero cross of the pattern.
// a offset should be made if the pattern doest not started at the very first cycle.
float measure_signal_t(float* sig, uint32_t sig_len, float* pattern, uint32_t pattern_len)
{
    // find the signal match for the backward
    uint32_t matched_idx = match_filter(pattern, pattern_len, sig, sig_len, NULL);

    // using zero cross to gain sub-digit resolution
    #define NUM_CROSS       10
    float cross[NUM_CROSS];
    uint32_t num_of_cross = linear_interpolation_zerocrossing(&sig[matched_idx], sig_len-matched_idx, cross, NUM_CROSS);

    // get the average of all crossing difference. (also want to add a median filter. )
    float sub_digit_diff = 0;
    float period = (cross[num_of_cross-1] - cross[0])/num_of_cross;       // get the period.
    float errors[NUM_CROSS] = {0};

    for(uint32_t i=0; i<num_of_cross; i++){
        errors[i] = cross[0] + i*period - cross[i];
    }
    // get rid of a few max and min, use the rest for average.
    sort(errors, num_of_cross);
    // average
    for(uint32_t i=2; i<num_of_cross-2; i++){
        sub_digit_diff += errors[i];
    }
    sub_digit_diff = sub_digit_diff/(num_of_cross - 4); // in a good match, this should be within -1 to 1

    return matched_idx + sub_digit_diff;
}

// input the calibration signal (sampled when no signal)
float get_zero_level(uint16_t* raw, uint32_t len)
{
    float sum = 0;
    for(uint32_t i=0; i<len; i++)
        sum += raw[i];
    return sum/len;
}

// int16 -> float, also move data to zero_level. see if we need to scale it?
void preprocess(uint16_t *raw, float* out, float zero_level, uint32_t len)
{
    for(uint32_t i=0; i<len; i++)
        out[i] = (float)raw[i] - zero_level;
}


void thread_anemometer(void* parameters)
{
    recorder_t *recorder = NULL;
    char str_buf[128] = {0};

    // parameters.
    // D=distance to reflector; alpha=angle of reflection.
    // wind speed: v = d/sin(a)*cos(a)((1/T_forward) - (1/T_backward))
    // sound speed: c = d/sin(a)*((1/T_forward) + (1/T_backward)
    #define HEIGHT   0.07   // the height to the reflection panel.
    #define PITCH    0.05   // the distance between 2 transceivers.
    // const
    float alpha = atanf(2*HEIGHT/PITCH);
    float cos_alpha = cosf(alpha);
    float sin_alpha = sinf(alpha);

    // ADC = 1Msps, 500sample = 0.5ms ToF ~= 0.17m
    // speed of sound: ~340m/s
    // 500 sample  = 0.5ms ~= 0.17m
    // 1000 sample = 1ms   ~= 0.34m
    #define ADC_SAMPLE_LEN (1000)
    uint16_t adc_buffer[4][ADC_SAMPLE_LEN] = {0};

    // where the process started -> to avoid the direct sound propagation at the beginning. Depended on the mechanical structure.
    // Same unit of ADC sampling -> us.
    #define START_OFFSET (400)

    // allocate for floating-point signal
    #define PATTERN_LEN (100)
    float* sig_buf1;
    float* sig_buf2;
    float* pattern_buf;
    pattern_buf = malloc(sizeof(float)* PATTERN_LEN);
    sig_buf1 = malloc(sizeof(float) * ADC_SAMPLE_LEN);
    sig_buf2 = malloc(sizeof(float) * ADC_SAMPLE_LEN);

    // temporary for result.
    float north_delay, east_delay;
    float dt[4];

    // zero_level for all direction. see if needed.
    float zero_level[4]= {0};

    // hardware power_on
    ane_pwr_control(40*1000, true);

    // measure zero_level - calibration.
    for(uint32_t i=0; i<4; i++)
    {
        rt_thread_mdelay(10);
        adc_sample((ULTRASONIC_CHANNEL)i, adc_buffer[0], ADC_SAMPLE_LEN);
        rt_thread_mdelay(10);
        adc_sample((ULTRASONIC_CHANNEL)i, adc_buffer[0], ADC_SAMPLE_LEN); // sample twice for more stable measurement.
        zero_level[i] = get_zero_level(adc_buffer[0], ADC_SAMPLE_LEN);
    }

    while(1)
    {
        rt_sem_take(sem_ready, RT_WAITING_FOREVER);

        // measure.
        ane_measure_ch(NORTH, 4, adc_buffer[NORTH], ADC_SAMPLE_LEN);
        ane_measure_ch(SOUTH, 4, adc_buffer[SOUTH], ADC_SAMPLE_LEN);
        ane_measure_ch(EAST, 4, adc_buffer[EAST], ADC_SAMPLE_LEN);
        ane_measure_ch(WEST, 4, adc_buffer[WEST], ADC_SAMPLE_LEN);

        const uint32_t soff = START_OFFSET;
        const uint32_t len = ADC_SAMPLE_LEN - START_OFFSET;

        // process north-south
        preprocess(adc_buffer[NORTH], sig_buf1, zero_level[SOUTH], ADC_SAMPLE_LEN); // -> forward
        preprocess(adc_buffer[SOUTH], sig_buf2, zero_level[NORTH], ADC_SAMPLE_LEN); // -> backward
        north_delay = measure_signal_diff(&sig_buf1[soff], &sig_buf2[soff], len, pattern_buf, PATTERN_LEN);


        // process east-west
        preprocess(adc_buffer[EAST], sig_buf1, zero_level[WEST], ADC_SAMPLE_LEN); // -> forward
        preprocess(adc_buffer[WEST], sig_buf2, zero_level[EAST], ADC_SAMPLE_LEN); // -> backward
        east_delay = measure_signal_diff(&sig_buf1[soff], &sig_buf2[soff], len, pattern_buf, PATTERN_LEN);

        LOG_D("N-S: %2.3us, E-W: %2.3us", north_delay, east_delay);

        //
        float ns_speed, ew_speed;



        // only record a time to test
        if(recorder == NULL)
        {
            recorder = recorder_create("/sd/test.csv", "recorder", 128, 1000);
            recorder_write(recorder, "adc\n"); // header
            for(int i; i<ADC_SAMPLE_LEN; i++)
            {
                snprintf(str_buf, 128, "%d\n", (int)adc_buffer[0][i]);
                recorder_write(recorder, str_buf);
            }
            recorder_delete(recorder);
        }
    }

    free(sig_buf1);
    free(sig_buf2);
}



int thread_anemometer_init()
{
    rt_thread_t tid;

    sem_ready = rt_sem_create("anemo", 0, RT_IPC_FLAG_FIFO);
    timer = rt_timer_create("anemo", timer_tick, RT_NULL, 1000, RT_TIMER_FLAG_PERIODIC);
    tid = rt_thread_create("anemo", thread_anemometer, RT_NULL, 8192, 5, 1000);
    if(!tid)
        return RT_ERROR;

    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_anemometer_init);
