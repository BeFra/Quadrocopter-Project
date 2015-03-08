/*
 * sensor.c
 *
 * Created: 03.11.2014 16:41:50
 *  Author: befrank
 */ 
#include "../i2c/i2c.h"
#include "../utils/print.h"
#include "../time/time.h"
#include "sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 16000000UL //CPU Clock 16Mhz
#endif
#include <util/delay.h>

#define ACCELEROMETER_RANGE ACCELEROMETER_RANGE_2G
#define GYROSCOPE_RANGE GYROSCOPE_RANGE_250
#define RAD_TO_DEG 57.295779513082320876798154814105

void set_angle_data(unsigned long time, float x_angle, float y_angle, float z_angle);
inline unsigned long get_last_time();
inline float get_last_filtert_x_angle();
inline float get_last_filtert_y_angle();
inline float get_last_filtert_z_angle();


unsigned long  timeStamp;
float last_filtert_analge_x;
float last_filtert_analge_y;
float last_filtert_analge_z;

float last_gyro_angle_x;
float last_gyro_angle_y;
float last_gyro_angle_z;




uint8_t get_sensor_data(uint8_t adr, uint8_t reg) {
    uint8_t tmp;

    connect_i2c(adr,I2C_WRITE);
    send_i2c(reg);
    connect_i2c(adr,I2C_READ);
    tmp = read_i2c();
    stop_i2c();
    return tmp;
}

void send_sensor_data(uint8_t adr, uint8_t reg, uint8_t data) {
    
    connect_i2c(adr,I2C_WRITE);
    send_i2c(reg);
    send_i2c(data);
    stop_i2c();
}

void init_sensor() {
    /* init i2c bus */
    init_i2c(); 
    
    send_sensor_data(ADR_MPU,0x6B,0x80); //Power Management: Reset
    _delay_ms(30); //Wait for Reset

    send_sensor_data(ADR_MPU,0x6B,0x00); /*no sleep*/
    _delay_ms(10); /*Wait for Reset*/
    /*disable output to FIFO buffer*/
    send_sensor_data(ADR_MPU,0x6A,0x88);
    /*sample rate*/
    //send_sensor_data(ADR_MPU,0x19,0x01);
	
	/*Power Management: GyroZ PLL Reference*/
    send_sensor_data(ADR_MPU,0x6B,0x03); 
	/*Configuration: Low-Pass: 256Hz*/
    send_sensor_data(ADR_MPU,0x1A,0x00); 
    /* ACCEL_CONFIG */
    send_sensor_data(ADR_MPU,0x1C,ACCELEROMETER_RANGE); 
    /* GYRO_CONFIG */
    send_sensor_data(ADR_MPU,0x1B,GYROSCOPE_RANGE);
    
    /* calibrate gyro and acc */
    calibrate_accelerometer();
    calibrate_gyroscope();
    
    set_angle_data(get_millis(),0,0,0);
}

/*
 * Accelerometer
 */

/* get the raw accelerometer data */
void get_acc_raw(int16_t* data) {
    data[X] = get_sensor_data(ADR_MPU,0x3B) << 8;
    data[X] += get_sensor_data(ADR_MPU,0x3C);
    
    data[Y] = get_sensor_data(ADR_MPU,0x3D) << 8;
    data[Y] += get_sensor_data(ADR_MPU,0x3E);
    
    data[Z] = get_sensor_data(ADR_MPU,0x3F) << 8;
    data[Z] += get_sensor_data(ADR_MPU,0x40);
}

void get_acc(float* data) {
    int16_t tmp[3];
	char tmpONE[10];
    get_acc_raw(tmp);
    if(ACCELEROMETER_RANGE == ACCELEROMETER_RANGE_2G) {
        data[X]=(float) (tmp[X]) / 16384.0; //- acc_base[X]
        data[Y]=(float) (tmp[Y]) / 16384.0; //- acc_base[Y]
        data[Z]=(float) (tmp[Z]) / 16384.0; //- acc_base[Z]
   }
    if(ACCELEROMETER_RANGE == ACCELEROMETER_RANGE_4G) {
        data[X]=(float) (tmp[X]- acc_base[X]) / 8192.0;
        data[Y]=(float) (tmp[Y]- acc_base[Y]) / 8192.0;
        data[Z]=(float) (tmp[Z]- acc_base[Z]) / 8192.0;
    }
    if(ACCELEROMETER_RANGE == ACCELEROMETER_RANGE_8G) {
        data[X]=(float) (tmp[X]- acc_base[X]) / 4096.0;
        data[Y]=(float) (tmp[Y]- acc_base[Y]) / 4096.0;
        data[Z]=(float) (tmp[Z]- acc_base[Z]) / 4096.0;
    }
    if(ACCELEROMETER_RANGE == ACCELEROMETER_RANGE_16G) {
        data[X]=(float) (tmp[X]- acc_base[X]) / 2048.0;
        data[Y]=(float) (tmp[Y]- acc_base[Y]) / 2048.0;
        data[Z]=(float) (tmp[Z]- acc_base[Z]) / 2048.0;
    }   
}

/*
 * Gyroscope
 */
/* get the raw gyroscope data */
void get_gyro_raw(int16_t* data) {
    data[X] = get_sensor_data(ADR_MPU,0x43) << 8;
    data[X] += get_sensor_data(ADR_MPU,0x44);
    
    data[Y] = get_sensor_data(ADR_MPU,0x45) << 8;
    data[Y] += get_sensor_data(ADR_MPU,0x46);
    
    data[Z] = get_sensor_data(ADR_MPU,0x47) << 8;
    data[Z] += get_sensor_data(ADR_MPU,0x48);
}

/*
 * Convert the raw gyro Data to degrees/sec
 */
void get_gyro(float* data) {
    int16_t tmp[3];
    get_gyro_raw(tmp);
    if(GYROSCOPE_RANGE == GYROSCOPE_RANGE_250) {
        data[X]=(float) (tmp[X] - gyro_base[X]) / 131.0;
        data[Y]=(float) (tmp[Y] - gyro_base[Y]) / 131.0;
        data[Z]=(float) (tmp[Z] - gyro_base[Z]) / 131.0;
    }
    if(GYROSCOPE_RANGE == GYROSCOPE_RANGE_500) {
        data[X]=(float) (tmp[X] - gyro_base[X]) / 65.5;
        data[Y]=(float) (tmp[Y] - gyro_base[Y]) / 65.5;
        data[Z]=(float) (tmp[Z] - gyro_base[Z]) / 65.5;
    }
    if(GYROSCOPE_RANGE == GYROSCOPE_RANGE_1000) {
        data[X]=(float) (tmp[X] - gyro_base[X]) / 32.8;
        data[Y]=(float) (tmp[Y] - gyro_base[Y]) / 32.8;
        data[Z]=(float) (tmp[Z] - gyro_base[Z]) / 32.8;
    }
    if(GYROSCOPE_RANGE == GYROSCOPE_RANGE_2000) {
        data[X]=(float) (tmp[X] - gyro_base[X]) / 16.4;
        data[Y]=(float) (tmp[Y] - gyro_base[Y]) / 16.4;
        data[Z]=(float) (tmp[Z] - gyro_base[Z]) / 16.4;
    }
}

/*
 * return the actual raw value of the temperature
 */ 
int16_t get_temperature_raw() {
    int16_t rawData;
    
    rawData = get_sensor_data(ADR_MPU, 0x41) << 8;
    rawData |= get_sensor_data(ADR_MPU,0x42);
    return rawData;
}


/*
 * return the actual temperature value
 */
float get_temperature() {
    float temp_MPU = 0;
    int16_t rawData= get_temperature_raw();
    
    temp_MPU = ((float) rawData) / 340 + 36.53;
    return temp_MPU;
}

/*
 * calibration gyroscope sensor
 */
void calibrate_gyroscope() {
    int16_t gyro_tmp[3];
    int turns = 20;
    for (int i = 0; i < turns ; i++ ) {
        get_gyro_raw(gyro_tmp);
        gyro_base[X] += gyro_tmp[X];
        gyro_base[Y] += gyro_tmp[Y];
        gyro_base[Z] += gyro_tmp[Z];
		_delay_ms(100);
    }
    gyro_base[X] /= turns;
    gyro_base[Y] /= turns;
    gyro_base[Z] /= turns;
}

/*
 * calibrate acceleromenter sensor
 */
void calibrate_accelerometer() {
    int16_t acc_tmp[3];
    int turns = 20;
	char tmpONE[10];    
    for ( int i = 0; i < turns; i++ ) {
        get_acc_raw(acc_tmp);
        acc_base[X] += acc_tmp[X];
        acc_base[Y] += acc_tmp[Y];
        acc_base[Z] += acc_tmp[Z];
		_delay_ms(100);
    }
    acc_base[X] /= turns;
    acc_base[Y] /= turns;
    acc_base[Z] /= turns;
	if(acc_base[X] < 0) acc_base[X] *= -1;
	if(acc_base[Y] < 0) acc_base[Y] *= -1;
	if(acc_base[Z] < 0) acc_base[Z] *= -1;
}
/*
 * calculate angles for x,y,z angles for pitch, raw, roll
 */
void get_angles() {
    unsigned long t_now = get_millis();
    float gyro[3];
    float acc[3];
    char tmp[10];
    get_gyro(gyro);
    get_acc(acc);
    
    float acc_angle_x = atan(acc[X] / sqrt(pow(acc[Y],2) + pow(acc[Z],2))) * RAD_TO_DEG;
    float acc_angle_y = atan(acc[Y] / sqrt(pow(acc[X],2) + pow(acc[Z],2))) * RAD_TO_DEG;
    float acc_angle_z = 0; 
    
    
    /* Compute the filtered gyro angles */
    float dt = (t_now - get_last_time()) / 1000.0;
    float gyro_angle_x = (gyro[X] * dt) + get_last_filtert_x_angle();
    float gyro_angle_y = (gyro[Y] * dt) + get_last_filtert_y_angle();
    float gyro_angle_z = (gyro[Z] * dt) + get_last_filtert_z_angle();
 
    /* complementary Filter */
    const float alpha = 0.96;
    float angle_x =alpha * gyro_angle_x + (1.0 - alpha) * acc_angle_x; 
    float angle_y =alpha * gyro_angle_y + (1.0 - alpha) * acc_angle_y; 
    float angle_z = gyro_angle_z; 
    

    print("x, y, z: ");
    sprintf(tmp,"%f",angle_x);
    for(int x = 0; x < 5; x++) {
        pchar(tmp[x]);
    }
    print(", ");
    sprintf(tmp,"%f",angle_y);
    for(int x = 0; x < 5; x++) {
        pchar(tmp[x]);
    }
    print(", ");
    sprintf(tmp,"%f",angle_z);
    for(int x = 0; x < 5; x++) {
        pchar(tmp[x]);
    }
    print("\n");

    set_angle_data(t_now, angle_x, angle_y, angle_z);
}

void set_angle_data(unsigned long time, float x_angle, float y_angle, float z_angle ) {
    timeStamp = time;
    last_filtert_analge_x = x_angle;
    last_filtert_analge_y = y_angle;
    last_filtert_analge_z = z_angle;
}

inline unsigned long get_last_time() {return timeStamp;}
inline float get_last_filtert_x_angle() {return last_filtert_analge_x;}
inline float get_last_filtert_y_angle() {return last_filtert_analge_y;}
inline float get_last_filtert_z_angle() {return last_filtert_analge_z;}
