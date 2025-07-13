#include "pid.h"

#define IMU_ADDRESS 0x68    //Change to the address of the IMU

void pid::initialise()
{
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  calData calib = { 0 };  //Calibration data
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  } 
  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  IMU.init(calib, IMU_ADDRESS); // why twice?

  // normalised gyro range +/- 1
  err = IMU.setGyroRange(1);
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void pid::calculate(int pitch_, int roll_, int yaw_, int throttle_)
{
  GyroData gyroData;
  IMU.getGyro(&gyroData);

  float error_rate_roll = (roll_ * max_rate_roll) - gyroData.gyroX;
  float error_rate_pitch = (pitch_ * max_rate_pitch) - gyroData.gyroY;
  float error_rate_yaw = (yaw_ * max_rate_yaw) - gyroData.gyroZ;

  float i_limit = 150.0; // PID I-term limiter. The applied I-term cannot exceed or go below (negative) this value. (safety mechanism to prevent excessive spooling of the motors)
    
  // roll PID calc
  float roll_p = error_rate_roll * pid_roll_kp;
  float roll_i = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds);
  roll_i = max(min(roll_i, i_limit), -i_limit); // constrain within I-term limits;
  float roll_d = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds;
  float pid_roll = roll_p + roll_i + roll_d;

  // pitch PID calc
  float pitch_p = error_rate_pitch * pid_pitch_kp;
  float pitch_i = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds);
  pitch_i = max(min(pitch_i, i_limit), -i_limit); // constrain within I-term limits;
  float pitch_d = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds;
  pid_pitch = pitch_p + pitch_i + pitch_d;

  // yaw PID calc
  float yaw_p = error_rate_yaw * pid_yaw_kp;
  float yaw_i = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds);
  yaw_i = max(min(yaw_i, i_limit), -i_limit); // constrain within I-term limits;
  float yaw_d = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds;
  pid_yaw = yaw_p + yaw_i + yaw_d;

  // Save state values for next loop
  roll_last_error = error_rate_roll;
  pitch_last_error = error_rate_pitch;
  yaw_last_error = error_rate_yaw;
  roll_last_integral = roll_i;
  pitch_last_integral = pitch_i;
  yaw_last_integral = yaw_i;
}
