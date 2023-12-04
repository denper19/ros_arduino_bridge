#include "imu.h"

void calc_imu()
{
  Wire.begin();
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 14); // request a total of 14 registers
  AccX = ((Wire.read() << 8 | Wire.read()) - accx_offset) / 16384.0; // X-axis value
  AccY = ((Wire.read() << 8 | Wire.read()) - accy_offset) / 16384.0; // Y-axis value
  AccZ = ((Wire.read() << 8 | Wire.read()) - accz_offset) / 16384.0; // Z-axis value
  GyroX = ((Wire.read() << 8 | Wire.read()) - gyrox_offset) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = ((Wire.read() << 8 | Wire.read()) - gyroy_offset) / 131.0;
  GyroZ = ((Wire.read() << 8 | Wire.read()) - gyroz_offset) / 131.0;
  // Serial.println(AccX);

  pitch_new = atan2((-AccX), sqrt((AccY*AccY) + (AccZ*AccZ))) * RAD_TO_DEG;
  pitch = pitch_filter * pitch_old + (1 - pitch_filter) * pitch_new;
  pitch_old = pitch;

  roll_new  = atan2(AccY, AccZ) * RAD_TO_DEG;
  roll = roll_filter * roll_old + (1 - roll_filter) * roll_new;
  roll_old = roll;

  float currT = millis();
  float dt = (currT - prevT) / 1000.0f;
  prevT = currT;

  gyro_x += GyroX * dt;
  gyro_y += GyroY * dt;
  gyro_z += GyroZ * dt;

  system_pitch = comp_pitch * gyro_y + (1 - comp_pitch) * pitch;
  system_roll  = comp_roll  * gyro_x + (1 - comp_roll)  * roll;

  mpu_read_ready_flag = 0;
}

void setup_imu()
{
// Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  attachInterrupt(digitalPinToInterrupt(2), read_imu, RISING);
}

void read_imu()
{
  mpu_read_ready_flag = 1;
}