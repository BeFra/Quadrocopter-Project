/*
 * sensor.h
 *
 * Created: 03.11.2014 21:48:11
 *  Author: befrank
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

float gyro[3];
float acc[3];

/* for calibration */
int gyro_base[3];
int acc_base[3];

/* Adress of the MPU6050 */
#define ADR_MPU 0x68

/* X-Axis */
#define X 0
/* Y-Axis */
#define Y 1
/* Z-Axis */
#define Z 2

/*Acc-Configuration: Full Scale Range +-2G*/
#define ACCELEROMETER_RANGE_2G 0x00
/*Acc-Configuration: Full Scale Range +-4G*/
#define ACCELEROMETER_RANGE_4G 0x01
/*Acc-Configuration: Full Scale Range +-8G*/
#define ACCELEROMETER_RANGE_8G 0x02
/*Acc-Configuration: Full Scale Range +-16G*/
#define ACCELEROMETER_RANGE_16G 0x03

/*Gyro-Configuration: Full Scale Range = +-250°/s*/
#define GYROSCOPE_RANGE_250 0x00
/*Gyro-Configuration: Full Scale Range = +-500°/s*/
#define GYROSCOPE_RANGE_500 0x01
/*Gyro-Configuration: Full Scale Range = +-1000°/s*/
#define GYROSCOPE_RANGE_1000 0x02
/*Gyro-Configuration: Full Scale Range = +-2000°/s*/
#define GYROSCOPE_RANGE_2000 0x03

uint8_t get_sensor_data(uint8_t adr, uint8_t reg);
void send_sensor_data(uint8_t adr, uint8_t reg, uint8_t data);
void init_sensor();
void get_acc_raw(int16_t* data);
void get_acc(float* data);
void get_gyro_raw(int16_t* data);
void get_gyro(float* data);
int16_t get_temperature_raw();
float get_temperature();
void calibrate_gyroscope();
void calibrate_accelerometer();
void get_angles();


#endif /* SENSOR_H_ */