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
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_anemometer.h"
#include "recorder.h"
#include "configuration.h"
#include "data_pool.h"

#define DBG_TAG "anemo"
#define DBG_LVL LOG_LVL_INFO
#include <rtdbg.h>

#ifndef MAX
#define MAX(a, b) (a>b?a:b)
#endif
#ifndef MIN
#define MIN(a, b) (a<b?a:b)
#endif

//#define IS_SIGN_DIFF(a, b) (a * b < 0)            // this doesnt not cover 0
//#define IS_SIGN_DIFF(a, b) (!(((int)a^(int)b)>>(sizeof(int)-1))) // this cover 0, but have fixed width
#define IS_SIGN_DIFF(a, b) (!(signbit(a) == signbit(b))) // this cover 0, but have fixed width

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

float maxf(float* sig, int len)
{
    float max = sig[0];
    for(int i=1; i<len; i++)
        max = MAX(sig[i], max);
    return max;
}

// find the max index
int32_t argmaxf(float* sig, int32_t len)
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


float minf(float* sig, int len)
{
    float min = sig[0];
    for(int i=1; i<len; i++)
        min = MIN(sig[i], min);
    return min;
}

int arg_minf(float* sig, int len)
{
    float min = sig[0];
    int arg = 0;
    for(uint32_t i=0; i<len; i++){
        if(sig[i] < min){
            arg = i;
            min = sig[i];
        }
    }
    return arg;
}


// to normalize the signal to -1 ~1
void normalize(float* pattern, uint32_t len)
{
    float max = 0;
    for(int i=0; i<len; i++)
        max = MAX(abs(pattern[i]), max);
    for(int i=0; i<len; i++)
        pattern[i] = pattern[i] / max;
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

// find the exact interpolation
// output the offset of the signal in sub-digit resolution.
int linear_interpolation_zerocrossing(float* sig, uint32_t sig_len, float* out, uint32_t num_zero_cross)
{
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
        else if(sig[i] == 0) // in case there is a zero.
        {
            out[cross] = i;
            cross++;
        }
    }
    return  cross;
}

// input the calibration signal (sampled when no signal)
float get_zero_level(uint16_t* raw, uint32_t len)
{
    float sum = 0;
    for(uint32_t i=0; i<len; i++)
        sum += raw[i];
    return sum/len;
}

int find_next_turning(float *sig, int len)
{
    float pre_dt = sig[5]-sig[4]; // jump the first point for better stability
    float dt = 0;
    for(int i=5; i<len-1; i++)
    {
        dt = sig[i+1] - sig[i];
        if(IS_SIGN_DIFF(pre_dt, dt))
            return i;
        pre_dt = dt;
    }
    return 0;
}

int find_prev_turning(float *sig, int len)
{
    float *p = sig;
    float pre_dt = *(p-4) - *(p-5);
    float dt = 0;
    for(int i=5; i<len-1; i++)
    {
        dt = *(p-i) - *(p-i-1);
        if(IS_SIGN_DIFF(pre_dt, dt))
            return -i;
        pre_dt = dt;
    }
    return 0;
}

// capture a few peak moment around the peak points
// buffer size should equal to (peak_left_len + 1 + peak_right_len)
// buffer is stored in peaks[][0] = index, peaks[][1] = value.
int capture_peaks(float* sig, int sig_len, float peaks[][2], int peak_left_len, int peak_right_len, float threshold)
{
    int peak_detected_len = 0;
    int max_idx = argmaxf(sig, sig_len); // this is the middle peak (main).
    int peak_idx;
    int max_distance_left = 25 * (peak_left_len +2);
    int max_distance_right = 25 * (peak_right_len +2);
    threshold = sig[max_idx] * threshold;

    // main peak
    peak_idx = peak_left_len;
    peaks[peak_idx][0] = max_idx;
    peaks[peak_idx][1] = sig[max_idx];
    peak_detected_len ++;

    // scan for the peak after max. (right)
    int sig_idx = max_idx;          // signal start form the right of main peak.
    peak_idx = peak_left_len + 1;  // start from the right of main peak.
    for(int i=0; i<peak_right_len; i++)
    {
       int turning_idx = find_next_turning(&sig[sig_idx], sig_len - sig_idx);
       if(turning_idx == 0)
           break;
       sig_idx += turning_idx;
       if(sig_idx > sig_len || sig_idx - max_idx > max_distance_right) // max searching distance
           break;

       if(fabs(sig[sig_idx]) >= threshold)
       {
           peaks[peak_idx][0] = sig_idx;
           peaks[peak_idx][1] = sig[sig_idx];
           peak_idx ++;
           peak_detected_len ++;
       }
    }

    // scan for the peak before max. (left)
    sig_idx = max_idx;          // signal start form the left of main peak.
    peak_idx = peak_left_len - 1;  // start from the left of main peak.
    for(int i=peak_idx; i>=0 && peak_idx>=0; i--)
    {
       int turning_idx = find_prev_turning(&sig[sig_idx], sig_idx);
       if(turning_idx == 0)
           break;
       sig_idx += turning_idx;
       if(max_idx - sig_idx > max_distance_left) // maximum searching distance
           break;

       if(fabs(sig[sig_idx]) >= threshold)
       {
           peaks[peak_idx][0] = sig_idx;
           peaks[peak_idx][1] = sig[sig_idx];
           peak_idx --;
           peak_detected_len ++;
       }
    }
    return peak_detected_len;
}

// compare 2 peaks arrays, find offset of it.
int match_shape(float peaks1[][2], float peaks2[][2], int len, float mse[], int search_range)
{
    memset(mse, 0, sizeof(float)*search_range);
    for(int off = -search_range/2; off<=search_range/2; off++)
    {
        float sum = 0;
        float count = 0;
        int start_idx = -off;
        int stop_idx = len + off;
        if(start_idx < 0) start_idx = 0;
        if(stop_idx > len) stop_idx = len - off;
        for(int i=start_idx; i<stop_idx; i++)
        {
            if(peaks1[i][0] !=0 && peaks2[i][0] != 0) //
            {
                float v = peaks1[i][1] - peaks2[i+off][1];
                sum += v*v;
                count++;
            }
        }
        mse[off+search_range/2] = sum/count;
    }
    return arg_minf(mse, search_range) - (search_range/2);
}

// int16 -> float, also move data to zero_level. see if we need to scale it?
void preprocess(uint16_t *raw, float* out, float zero_level, uint32_t len)
{
    for(uint32_t i=0; i<len; i++)
        out[i] = (float)raw[i] - zero_level;
}

// ADC = 1Msps, 500sample = 0.5ms ToF ~= 0.17m
// speed of sound: ~340m/s
// 500 sample  = 0.5ms ~= 0.17m
// 1000 sample = 1ms   ~= 0.34m
#define ADC_SAMPLE_LEN (1000)
uint16_t adc_buffer[4][ADC_SAMPLE_LEN] = {0};

void test_channel(uint32_t ch)
{
#define PWM_PIN GET_PIN(A, 6)
    rt_pin_mode(PWM_PIN, PIN_MODE_OUTPUT);
    set_output_channel(ch);
    rt_pin_write(PWM_PIN, 1);
    rt_pin_write(PWM_PIN, 0);
}

void test_print_raw()
{
    for(int j=0; j<ADC_SAMPLE_LEN; j++)
        rt_kprintf("%d\n", adc_buffer[NORTH][j]);
    for(int j=0; j<ADC_SAMPLE_LEN; j++)
        rt_kprintf("%d\n", adc_buffer[SOUTH][j]);
    for(int j=0; j<ADC_SAMPLE_LEN; j++)
        rt_kprintf("%d\n", adc_buffer[EAST][j]);
    for(int j=0; j<ADC_SAMPLE_LEN; j++)
        rt_kprintf("%d\n", adc_buffer[WEST][j]);
}

// ref http://www.sengpielaudio.com/calculator-airpressure.htm
// Speed of sound c ≈ 331.3 + 0.6 ϑ  (m/s)
// Temperature ϑ ≈ (331.3 − c) / 0.6
float speed_of_sound_from_T(float temperature){
    return 331.3+0.6*temperature;
}

float average(float sig[], int num)
{
    float sum=0;
    for(int i=0; i<num; i++)
        sum += sig[i];
    return sum/num;
}

void thread_anemometer(void* parameters)
{
    recorder_t *recorder = NULL;
    char str_buf[128] = {0};
    sensor_config_t * cfg;
    anemometer_config_t * ane_cfg;

    // waiting for configuration load
    do{
        cfg = get_sensor_config_wait("Anemometer");
    }while(!cfg);
    ane_cfg = cfg->user_data;

    // parameters.
    // D=distance to reflector; alpha=angle of reflection.
    // wind speed: v = d/sin(a)*cos(a)((1/T_forward) - (1/T_backward))
    // sound speed: c = d/sin(a)*((1/T_forward) + (1/T_backward)
    // #define HEIGHT   0.05   // the height to the reflection panel.
    // #define PITCH    0.04   // the distance between 2 transceivers.
    float height = ane_cfg->height;
    float pitch = ane_cfg->pitch;
    float alpha = atanf(2*height/pitch);
    float cos_a = cosf(alpha);
    float sin_a = sinf(alpha);

    // pulse generation/modulation, pulse are pwm with 0~99 = 0%~100%
    #define H 99
    #define L 0
    // single rate
    //uint16_t pulse[] = {50, 50, 50, 50};

    // Double rate, to control the +1 or −1 phase
    //uint16_t pulse[] = {H, L, H, L, H, L, H, L}; // normal -> +1 +1 +1 +1
    uint16_t pulse[] = {H, L, H, L, H, L, L, H, L, H}; // normal suppressed -> +1 +1 +1 −1 −1
    //uint16_t pulse[] = {H, L, H, L, H, L, H, L, L, H, L, H}; // normal suppressed 2 -> +1 +1 +1 +1 −1 −1
    //uint16_t pulse[] = {H, L, H, L, L, H, H, L}; // barker-code 4.1 -> +1 +1 −1 +1
    //uint16_t pulse[] = {H, L, H, L, H, L, L, H}; // barker-code 4.2 -> +1 +1 +1 −1
    //uint16_t pulse[] = {H, L, H, L, H, L, H, L, L, H, L, H, H, L, L, H}; // barker-code 7 -> +1 +1 +1 −1 −1 +1 −1
    uint32_t pulse_len = sizeof(pulse) / sizeof(uint16_t);

    // where the process started -> to avoid the direct sound propagation at the beginning. Depended on the mechanical structure.
    // Same unit of ADC sampling -> us.
    #define DEADZONE_OFFSET (200)
    #define VALID_LEN  (ADC_SAMPLE_LEN - DEADZONE_OFFSET)
    #define ZEROCROSS_LEN   (10)
    #define NUM_ZC_AVG      (5) // number of zerocrossing to calculate the beam location.

    // to define the shape of to identify the echo beam
    #define PEAK_LEFT   (3)
    #define PEAK_MAIN   PEAK_LEFT
    #define PEAK_RIGHT  (5)
    #define PEAK_LEN    (PEAK_LEFT + PEAK_RIGHT + 1)
    #define PEAK_ZC     (2)  // start from which peak to identify the zero crossing. (2=3rd)

    // the theoretical arrive time of the first pulse in calm wind and room temperature
    // it decides where to start capture the pulses. No need to be accurate
    //int32_t START_OFFSET = sqrtf((pitch/2.f)*(pitch/2.f) + height*height) * 2.f / 340.f * 1000000;
    // allocate for floating-point signal
    float* sig;
    sig = malloc(sizeof(float) * ADC_SAMPLE_LEN);
    if(!sig){
        LOG_E("No memory for Anemometer data");
        return;
    }

    // zero_level for all direction. see if needed.
    float zero_level[4]= {2048, 2048, 2048, 2048};

    // hardware power_on
    ane_pwr_control(80*1000, true);

    // cap charge
    for(uint32_t i=0; i<50; i++)
    {
        ane_measure_ch(EAST,  pulse, pulse_len, adc_buffer[EAST], ADC_SAMPLE_LEN);
        ane_measure_ch(WEST,  pulse, pulse_len, adc_buffer[WEST], ADC_SAMPLE_LEN);
        ane_measure_ch(NORTH,  pulse, pulse_len, adc_buffer[NORTH], ADC_SAMPLE_LEN);
        ane_measure_ch(SOUTH,  pulse, pulse_len, adc_buffer[SOUTH], ADC_SAMPLE_LEN);
    }

    // measure zero_level - calibration.
    for(uint32_t i=0; i<4; i++){
        rt_thread_mdelay(10);
        adc_sample((ULTRASONIC_CHANNEL)i, adc_buffer[i], ADC_SAMPLE_LEN);
        rt_thread_mdelay(10);
        adc_sample((ULTRASONIC_CHANNEL)i, adc_buffer[i], ADC_SAMPLE_LEN); // sample twice for more stable measurement.
        zero_level[i] = get_zero_level(&adc_buffer[i][DEADZONE_OFFSET], VALID_LEN);
    }
    printf("zeros: %.2f, %.2f, %.2f, %.2f\n", zero_level[0], zero_level[1], zero_level[2], zero_level[3]);

    // collect zerocross base line for each channel.
    // http://www.sengpielaudio.com/calculator-airpressure.htm
    float static_zero_cross[4][ZEROCROSS_LEN] = {0};
    float echo_shape[4][PEAK_LEN][2] = {0};       // store the shape of the echo
    memset(echo_shape, 0, sizeof(echo_shape));

    for (int i = 0; i<10; i++)
    {
        for(int idx = 0; idx < 4; idx ++)
        {
            // make a sample
            ane_measure_ch(idx,  pulse, pulse_len, adc_buffer[idx], ADC_SAMPLE_LEN);
            // convert to float
            preprocess(adc_buffer[idx], sig, zero_level[idx], ADC_SAMPLE_LEN);
            // normalize signal but not include the excitation.
            normalize(&sig[DEADZONE_OFFSET], VALID_LEN);
            // search original peaks, use to rough estimate the data.
            float peaks_zero[PEAK_LEN][2];
            memset(peaks_zero, 0, sizeof(peaks_zero));
            capture_peaks(&sig[DEADZONE_OFFSET], VALID_LEN, echo_shape[idx], PEAK_LEFT, PEAK_RIGHT, 0.2);

            // we start the crossing point from the PEAK_ZC
            int off = echo_shape[idx][PEAK_ZC][0];
            float zero_cross[ZEROCROSS_LEN] = {0};
            linear_interpolation_zerocrossing(&sig[DEADZONE_OFFSET + off], VALID_LEN-off, zero_cross, ZEROCROSS_LEN);

            // recover the actual timestamp from start
            for(int j=0; j<ZEROCROSS_LEN; j++)
                zero_cross[j] += off + DEADZONE_OFFSET;

            // sum them up
            for(int j=0; j<ZEROCROSS_LEN; j++)
                static_zero_cross[idx][j] += zero_cross[j];
        }
    }
    for(int idx = 0; idx < 4; idx ++)
        for(int j=0; j<ZEROCROSS_LEN; j++)
            static_zero_cross[idx][j] /= 10;
    // test
    float *p = &static_zero_cross[0][0];
    for(int i=0; i<sizeof(static_zero_cross)/sizeof(float); i++)
        printf("%f\n", *p++);

//    // for test
//    for (int i = 0; i<1; i++)
//    {
//        for(int idx = 0; idx < 4; idx ++)
//        {
//            rt_thread_delay(10);
//            // make a sample
//            ane_measure_ch(idx,  pulse, pulse_len, adc_buffer[idx], ADC_SAMPLE_LEN);
//        }
//        test_print_raw();
//        rt_thread_delay(3000);
//    }

    // calculate the offset between the first zerocrossing to the start of the wave
    float est_c = speed_of_sound_from_T(air_info.temperature);
    LOG_I("temp: %2.1f degC, est_wind_speed: %3.1fm/s", air_info.temperature, est_c);
    // the offset between the first valid crossing to the wave that actually start.
    float T = 2* height / (sin_a * est_c) * 1000000;
    float pulse_start_offset[4];
    pulse_start_offset[NORTH] = T - average(static_zero_cross[NORTH], NUM_ZC_AVG);
    pulse_start_offset[SOUTH] = T - average(static_zero_cross[SOUTH], NUM_ZC_AVG);
    pulse_start_offset[EAST] = T - average(static_zero_cross[EAST], NUM_ZC_AVG);
    pulse_start_offset[WEST] = T - average(static_zero_cross[WEST], NUM_ZC_AVG);

    LOG_I("Propagation time:%4.1f, est offset: %4.1f,  %4.1f, %4.1f, %4.1f",
            T, pulse_start_offset[0], pulse_start_offset[1],pulse_start_offset[2],pulse_start_offset[3]);

    rt_tick_t period = 1000/(cfg->update_rate * cfg->oversampling);
    int oversampling_count = 0;
    float ns_v=0, ew_v=0; // wind speed
    float ns_c=0, ew_c=0; // sound speed -> these measurements should be very close, other wise the measurment is wrong.
    float v=0, c=0;
    float v_acc=0, c_acc=0, ns_v_acc=0, ew_v_acc=0; // accumulation for oversampling
    float course=0;
    while(1)
    {
        // add small delay in case of over lapping.
        rt_thread_mdelay(period/16);
        rt_thread_mdelay(period - rt_tick_get() % period);
        if(!cfg->is_enable)
            continue;

        // make a sample
        ane_measure_ch(NORTH,  pulse, pulse_len, adc_buffer[NORTH], ADC_SAMPLE_LEN);
        ane_measure_ch(EAST,  pulse, pulse_len, adc_buffer[EAST], ADC_SAMPLE_LEN);
        ane_measure_ch(SOUTH,  pulse, pulse_len, adc_buffer[SOUTH], ADC_SAMPLE_LEN);
        ane_measure_ch(WEST,  pulse, pulse_len, adc_buffer[WEST], ADC_SAMPLE_LEN);

        rt_tick_t tick = rt_tick_get();

        float dt[4] = {0};
        for(int idx = 0; idx < 4; idx ++)
        {
            // offset to zero, and process
            preprocess(adc_buffer[idx], sig, zero_level[idx], ADC_SAMPLE_LEN);
            // normalize to -1 to 1
            normalize(&sig[DEADZONE_OFFSET], VALID_LEN);

            // Beside to use the signal peak to calculate the rough propagation time,
            // We use a few more peak and valley around the main peaks.
            // And use MSE to match the signals.  This is a shape detector.
            // detect peaks as shape.
            float shape[PEAK_LEN][2];
            memset(shape, 0, sizeof(shape));
            capture_peaks(&sig[DEADZONE_OFFSET], VALID_LEN, shape, PEAK_LEFT, PEAK_RIGHT, 0.4);

            // use peak to find the offset if there is any on the main peak
            float mse[7];
            int peak_off = match_shape(echo_shape[idx], shape, PEAK_LEN, mse, 7);
            // finally we can locate the main peak, despite the peak is distorted
            if(peak_off){
                LOG_D("peak offset %d, direction: %s", peak_off, ane_ch_names[idx]);
                LOG_D("mse: %f, %f, %f, %f, %f, %f, %f", mse[0],mse[1],mse[2],mse[3],mse[4],mse[5],mse[6]);
                //match_shape(echo_shape[idx], shape, PEAK_LEN, mse, 7);
            }
//            for(int i=0; i<10; i++){
//                printf("%d,%f\n", (int)shape[i][0], shape[i][1]);
//            }

            // we start the crossing point from the PEAK_ZC + offset detected by shape
            //int off = echo_shape[idx][PEAK_ZC - peak_off][0];
            int off = shape[PEAK_ZC + peak_off][0];
            float zero_cross[ZEROCROSS_LEN] = {0};
            linear_interpolation_zerocrossing(&sig[DEADZONE_OFFSET + off], VALID_LEN-off, zero_cross, ZEROCROSS_LEN);
            // recover the offsets for these zero cross
            for(int j=0; j<ZEROCROSS_LEN; j++)
                zero_cross[j] += off + DEADZONE_OFFSET;

            // finally, uses a few zero crossing points to calculate the propagation time.
            dt[idx] = average(zero_cross, NUM_ZC_AVG);
            dt[idx] += pulse_start_offset[idx];
        }

        // convert us to s
        dt[NORTH] /= 1000000.f;
        dt[SOUTH] /= 1000000.f;
        dt[EAST] /= 1000000.f;
        dt[WEST] /= 1000000.f;

        // wind speed.
        ns_v = height / (sin_a * cos_a) * (1.0f/dt[NORTH] - 1.0f/dt[SOUTH]);
        ew_v = height / (sin_a * cos_a) * (1.0f/dt[EAST] - 1.0f/dt[WEST]);
        v = sqrtf(ns_v*ns_v + ew_v*ew_v);
        v_acc += v;
        ns_v_acc += ns_v;
        ew_v_acc += ew_v;
        // sound speed
        ns_c = height / sin_a * (1.0f/dt[NORTH] + 1.0f/dt[SOUTH]);
        ew_c = height / sin_a * (1.0f/dt[EAST] + 1.0f/dt[WEST]);
        c = (ns_c + ew_c)/2;
        c_acc += c;
        // course
        course = atan2f(-ew_v, -ns_v)/3.1415926*180 + 180;

        // data output
        oversampling_count ++;
        if(oversampling_count >= cfg->oversampling){
            v_acc /= oversampling_count;
            c_acc /= oversampling_count;
            ns_v_acc /= oversampling_count;
            ew_v_acc /= oversampling_count;

            anemometer.course = atan2f(-ew_v_acc, -ns_v_acc)/3.1415926*180 + 180;
            anemometer.speed = v_acc;
            anemometer.soundspeed = c_acc;
            data_updated(&anemometer.info);

            v_acc = 0;
            c_acc = 0;
            ns_v_acc = 0;
            ew_v_acc = 0;
            oversampling_count = 0;

            printf("%3.1f, %3.1f, %2.3f,\n", anemometer.course, anemometer.soundspeed, anemometer.speed);
        }

        //printf("Course=%3.1fdegree, V=%2.1fm/s, C=%3.1fm/s, ns=%2.2fm/s, ew=%2.2fm/s\n", course, v, c, ns_v, ew_v);

//        LOG_I("run time %d ms", rt_tick_get() - tick);
//
        //printf("Course=%3.1fdeg, %3.1f, %2.3f,\n",course, c, v);
        //printf("%3.1f, %2.3f,\n", c, v);
    }

    free(sig);
}


int thread_anemometer_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("anemo", thread_anemometer, RT_NULL, 1024*4, 10, 1000);
    if(!tid)
        return RT_ERROR;
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(thread_anemometer_init);
