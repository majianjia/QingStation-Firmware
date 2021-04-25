#include <data_pool.h>
/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-07     majia       the first version
 */


#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "data_pool.h"

// instances
sys_t sys;
gyro_t gyro;
acc_t acc;
mag_t mag;
orientation_t orientation;
air_info_t air_info;
anemometer_t anemometer;
light_info_t light_info;
rain_t rain;
lightning_t lightning;
gnss_t gnss;

/* simple getters */ // deprecated
float get_unknown(){return -1;}
float get_gyro_x() {return gyro.unit.x;}
float get_gyro_y() {return gyro.unit.y;}
float get_gyro_z() {return gyro.unit.z;}
float get_acc_x() {return acc.unit.x;}
float get_acc_y() {return acc.unit.y;}
float get_acc_z() {return acc.unit.z;}
float get_mag_x() {return mag.unit.x;}
float get_mag_y() {return mag.unit.y;}
float get_mag_z() {return mag.unit.z;}
float get_eular_x() {return orientation.euler.x;}
float get_eular_y() {return orientation.euler.y;}
float get_eular_z() {return orientation.euler.z;}
float get_quat_q0() {return orientation.q[0];}
float get_quat_q1() {return orientation.q[1];}
float get_quat_q2() {return orientation.q[2];}
float get_quat_q3() {return orientation.q[3];}
float get_air_pressure(){return air_info.pressure;}
float get_air_humidity(){return air_info.humidity;}
float get_air_temperature(){return air_info.temperature;}
float get_light_info_r(){return light_info.R;}
float get_light_info_g(){return light_info.G;}
float get_light_info_b(){return light_info.B;}
float get_light_info_ir(){return light_info.IR;}
float get_light_info_all(){return light_info.ALS;}
float get_rain_level(){return rain.rain_level;}
float get_lightning(){return lightning.distance;}
float get_gnss_latitude(){return gnss.latitude;}
float get_gnss_longitude(){return gnss.longitude;}
float get_gnss_speed(){return gnss.speed;}
float get_gnss_altitude(){return gnss.altitude;}
float get_gnss_course(){return gnss.course;}
float get_gnss_num_sat(){return gnss.num_sat;}
float get_gnss_fixed(){return (int)gnss.is_fixed;}
float get_ane_course(){return anemometer.course;}
float get_ane_speed(){return anemometer.speed;}
float get_ane_speed30savg(){return anemometer.speed30savg;}
float get_ane_speed30smax(){return anemometer.speed30smax;}
float get_ane_soundspeed(){return anemometer.soundspeed;}
float get_ane_err_code(){return anemometer.err_code;}
float get_sys_bat_volt(){return sys.bat_voltage;}
float get_sys_volt(){return sys.sys_voltage;}


/* here we map data to their names,
 * so the log and recorder can be easily define which data we want by simply passing a header */
float (*get_data[])() = {
        get_unknown,
        get_gyro_x,
        get_gyro_y,
        get_gyro_z,
        get_acc_x,
        get_acc_y,
        get_acc_z,
        get_mag_x,
        get_mag_y,
        get_mag_z,
        get_eular_x,
        get_eular_y,
        get_eular_z,
        get_quat_q0,
        get_quat_q1,
        get_quat_q2,
        get_quat_q3,
        get_air_pressure,
        get_air_humidity,
        get_air_temperature,
        get_light_info_r,
        get_light_info_g,
        get_light_info_b,
        get_light_info_ir,
        get_light_info_all,
        get_rain_level,
        get_lightning,
        get_gnss_latitude,
        get_gnss_longitude,
        get_gnss_speed,
        get_gnss_altitude,
        get_gnss_course,
        get_gnss_num_sat,
        get_gnss_fixed,
        get_ane_course,
        get_ane_speed,
        get_ane_speed30savg,
        get_ane_speed30smax,
        get_ane_soundspeed,
        get_ane_err_code,
        get_sys_bat_volt,
        get_sys_volt
};

/* simple getters */
int print_unknown(char*buf){return sprintf(buf, "unknown");}
int print_gyro_x(char*buf) {return sprintf(buf, "%.4f", gyro.unit.x);}
int print_gyro_y(char*buf) {return sprintf(buf, "%.4f", gyro.unit.y);}
int print_gyro_z(char*buf) {return sprintf(buf, "%.4f", gyro.unit.z);}
int print_acc_x(char*buf) {return sprintf(buf, "%.4f", acc.unit.x);}
int print_acc_y(char*buf) {return sprintf(buf, "%.4f", acc.unit.y);}
int print_acc_z(char*buf) {return sprintf(buf, "%.4f", acc.unit.z);}
int print_mag_x(char*buf) {return sprintf(buf, "%d", (int)mag.unit.x);}
int print_mag_y(char*buf) {return sprintf(buf, "%d", (int)mag.unit.y);}
int print_mag_z(char*buf) {return sprintf(buf, "%d", (int)mag.unit.z);}
int print_eular_x(char*buf) {return sprintf(buf, "%.2f", orientation.euler.x);}
int print_eular_y(char*buf) {return sprintf(buf, "%.2f", orientation.euler.y);}
int print_eular_z(char*buf) {return sprintf(buf, "%.2f", orientation.euler.z);}
int print_quat_q0(char*buf) {return sprintf(buf, "%.7f", orientation.q[0]);}
int print_quat_q1(char*buf) {return sprintf(buf, "%.7f", orientation.q[1]);}
int print_quat_q2(char*buf) {return sprintf(buf, "%.7f", orientation.q[2]);}
int print_quat_q3(char*buf) {return sprintf(buf, "%.7f", orientation.q[3]);}
int print_air_pressure(char*buf){return sprintf(buf, "%.2f", air_info.pressure);}
int print_air_humidity(char*buf){return sprintf(buf, "%.2f", air_info.humidity);}
int print_air_temperature(char*buf){return sprintf(buf, "%.3f", air_info.temperature);}
int print_light_info_r(char*buf){return sprintf(buf, "%d", light_info.R);}
int print_light_info_g(char*buf){return sprintf(buf, "%d", light_info.G);}
int print_light_info_b(char*buf){return sprintf(buf, "%d", light_info.B);}
int print_light_info_ir(char*buf){return sprintf(buf, "%d", light_info.IR);}
int print_light_info_all(char*buf){return sprintf(buf, "%d", light_info.ALS);}
int print_rain_level(char*buf){return sprintf(buf, "%d", rain.rain_level);}
int print_lightning(char*buf){return sprintf(buf, "%d", (int)lightning.distance);}
int print_gnss_latitude(char*buf){return sprintf(buf, "%f", gnss.latitude);}
int print_gnss_longitude(char*buf){return sprintf(buf, "%f", gnss.longitude);}
int print_gnss_speed(char*buf){return sprintf(buf, "%.2f", gnss.speed);}
int print_gnss_altitude(char*buf){return sprintf(buf, "%.2f", gnss.altitude);}
int print_gnss_course(char*buf){return sprintf(buf, "%.1f", gnss.course);}
int print_gnss_num_sat(char*buf){return sprintf(buf, "%d", gnss.num_sat);}
int print_gnss_fixed(char*buf){if(gnss.is_fixed) return sprintf(buf, "true"); else return sprintf(buf, "false");}
int print_ane_course(char*buf){return sprintf(buf, "%.2f", anemometer.course);}
int print_ane_speed(char*buf){return sprintf(buf, "%.2f", anemometer.speed);}
int print_ane_speed30savg(char*buf){return sprintf(buf, "%.2f", anemometer.speed30savg);}
int print_ane_speed30smax(char*buf){return sprintf(buf, "%.2f", anemometer.speed30smax);}
int print_ane_soundspeed(char*buf){return sprintf(buf, "%.2f", anemometer.soundspeed);}
int print_ane_err_code(char*buf){return sprintf(buf, "%d", anemometer.err_code);};
int print_sys_bat_volt(char*buf){return sprintf(buf, "%.3f", sys.bat_voltage);}
int print_sys_volt(char*buf){return sprintf(buf, "%.3f", sys.sys_voltage);}

/* here we map data to their names,
 * so the log and recorder can be easily define which data we want by simply passing a header */
int (*print_data[])(char* ) = {
        print_unknown,
        print_gyro_x,
        print_gyro_y,
        print_gyro_z,
        print_acc_x,
        print_acc_y,
        print_acc_z,
        print_mag_x,
        print_mag_y,
        print_mag_z,
        print_eular_x,
        print_eular_y,
        print_eular_z,
        print_quat_q0,
        print_quat_q1,
        print_quat_q2,
        print_quat_q3,
        print_air_pressure,
        print_air_humidity,
        print_air_temperature,
        print_light_info_r,
        print_light_info_g,
        print_light_info_b,
        print_light_info_ir,
        print_light_info_all,
        print_rain_level,
        print_lightning,
        print_gnss_latitude,
        print_gnss_longitude,
        print_gnss_speed,
        print_gnss_altitude,
        print_gnss_course,
        print_gnss_num_sat,
        print_gnss_fixed,
        print_ane_course,
        print_ane_speed,
        print_ane_speed30savg,
        print_ane_speed30smax,
        print_ane_soundspeed,
        print_ane_err_code,
        print_sys_bat_volt,
        print_sys_volt
};

/*  names of data is corresponded to the above data getters. */
const char data_name[][DATA_NAME_MAX_LEN] = {
        "unknown",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "acc_x",
        "acc_y",
        "acc_z",
        "mag_x",
        "mag_y",
        "mag_z",
        "eular_x",
        "eular_y",
        "eular_z",
        "quat_q0",
        "quat_q1",
        "quat_q2",
        "quat_q3",
        "pressure",
        "humidity",
        "temp",
        "red",
        "green",
        "blue",
        "infrared",
        "light",
        "rain",
        "lightning",
        "latitude",
        "longitude",
        "speed",
        "altitude",
        "course",
        "num_sat",
        "fixed",
        "windcourse",
        "windspeed",
        "wind30savg",
        "wind30smax",
        "sndspeed",
        "ane_err",
        "bat_volt",
        "sys_volt"
};

const int EXPORT_DATA_SIZE = (sizeof(data_name)/DATA_NAME_MAX_LEN);

/* find the index of the getter (and name) */
uint32_t get_data_index(char* name)
{
    for(uint32_t i=0; i<sizeof(data_name)/sizeof(data_name[0]); i++)
    {
        if(strncmp(name, data_name[i], sizeof(data_name[0])) == 0)
            return i;
    }
    return 0;
}

/* get a list of names return the order of their getters
 * This will use strtok which will destroy the input strings. */
uint32_t get_data_orders(char* names, char* delim, uint16_t *orders, uint16_t max_order_len)
{
    char* saveptr = NULL;
    char* slice = NULL;

    slice = strtok_r(names, delim, &saveptr);
    for(uint32_t i = 0; i<max_order_len && slice != NULL; i++)
    {
        orders[i] = get_data_index(slice);
        slice = strtok_r(NULL, delim, &saveptr);
        if(slice == NULL)
            return i+1;
    }
    return max_order_len;
}

int data_test()
{
    char header[] = "gyro_x,gyro_y,acc_z";
    uint16_t orders[6] = {0};

    get_data_orders(header, ",", orders, 6);

    printf("orders: ");
    for(int i=0; i<sizeof(orders)/sizeof(uint16_t); i++)
    {
        printf("%d,", orders[i]);

    }
    printf("\n");

    return 0;
}
//INIT_APP_EXPORT(data_test);

void data_updated(sensor_info_t *info)
{
    rt_tick_t tick = rt_tick_get();
    info->update_rate = ((float)(tick - info->update_timestamp))/RT_TICK_PER_SECOND;
    info->update_timestamp = tick;
    info->count++;
}

