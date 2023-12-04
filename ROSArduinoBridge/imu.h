#pragma once

#define M_PI 3.14
#define RAD_TO_DEG 57.2958
#define GYRO_TIME_MS 500
#define GYRO_TIME_S GYRO_TIME_MS/1000

#define mpu_addr 0x68

#define accx_offset -3390 
#define accy_offset -150 
#define accz_offset 1552
#define gyrox_offset 0
#define gyroy_offset 2
#define gyroz_offset -25

int mpu_read_ready_flag = 0;

float pitch = 0;
float pitch_old = 0;
float pitch_new = 0;
float pitch_filter = 0.9;
float roll = 0;
float roll_old = 0;
float roll_new = 0;
float roll_filter = 0.9;
// gyro angles
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
//complimentary filter
float system_pitch = 0;
float system_roll = 0;
float comp_pitch = 0.85;
float comp_roll = 0.85;

float prevT = 0;

float AccX = 0.0f;
float AccY = 0.0f;
float AccZ = 0.0f;

float GyroX = 0.0f;
float GyroY = 0.0f;
float GyroZ = 0.0f;

void setup_imu();
void calc_imu();
void read_imu();