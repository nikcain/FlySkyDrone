#include <IBusBM.h>
#include "pid.h"
//#include "gyro.h"
//#include <ESP32Servo.h>

IBusBM IBus; 

pid pidcalc;

// motor pins
const int mot2_pin = 13; // right front
const int mot3_pin = 15; // left front
const int mot1_pin = 32; // right back
const int mot4_pin = 23; // left back

int m1_drive, m2_drive, m3_drive, m4_drive;

// Desired Flight Controller Cycle time
// This is the number of times per second the flight controller will perform an adjustment loop (PID loop)
float target_cycle_hz = 250.0;
int cycle_time_us;

void setup() {
  Serial.begin(115200);
  IBus.begin(Serial2, IBUSBM_NOTIMER);  
  
  pidcalc.initialise();
  
  ledcAttachPin(mot1_pin, 1);  // 12 kHz PWM, 8-bit resolution
  ledcSetup(1,12000, 8);
  ledcAttachPin(mot2_pin, 2);
  ledcSetup(2,12000, 8);
  ledcAttachPin(mot3_pin, 3);
  ledcSetup(3,12000, 8);
  ledcAttachPin(mot4_pin, 4);
  ledcSetup(4,12000, 8);

  /* if we could use ESP32 3.x (which we can't because of old IBusBM)
  then the above would be simply 
  
  ledcAttach(mot1_pin, 12000, 8);  // 12 kHz PWM, 8-bit resolution
  ledcAttach(mot2_pin, 12000, 8);
  ledcAttach(mot3_pin, 12000, 8);
  ledcAttach(mot4_pin, 12000, 8);
  */
  float cycle_time_seconds = 1.0 / target_cycle_hz;
  pidcalc.cycle_time_seconds = cycle_time_seconds;
  cycle_time_us = int(round(cycle_time_seconds * 1000000)); // multiply by 1,000,000 to go from seconds to microseconds (us)
}

void loop() {
  
  long loop_begin_us = micros();
  
  char pitch, roll, yaw, throttle;

  IBus.loop();
  throttle = IBus.readChannel(2);
  pitch = IBus.readChannel(1);
  yaw = IBus.readChannel(3);
  roll = IBus.readChannel(0);

  // Values are standard 1000-2000 (1500 in centre)
  // normalise the values for simplicity
  throttle = (throttle - 1500) / 500.0;
  pitch = (pitch - 1500) / 500.0;
  yaw = (yaw - 1500) / 500.0;
  roll = (roll - 1500) / 500.0;
  
  pidcalc.calculate(pitch, roll, yaw, throttle);

  // calculate throttle values
  float t1 = throttle + pidcalc.pid_pitch + pidcalc.pid_roll - pidcalc.pid_yaw;
  float t2 = throttle + pidcalc.pid_pitch - pidcalc.pid_roll + pidcalc.pid_yaw;
  float t3 = throttle - pidcalc.pid_pitch + pidcalc.pid_roll + pidcalc.pid_yaw;
  float t4 = throttle - pidcalc.pid_pitch - pidcalc.pid_roll - pidcalc.pid_yaw;
  
  // apply PWM
  ledcWrite(mot1_pin, t1); 
  ledcWrite(mot2_pin, t2);
  ledcWrite(mot3_pin, t3);
  ledcWrite(mot4_pin, t4);

  // match expected Hz
  int loop_end_us = micros();
  int elapsed_us = loop_end_us - loop_begin_us;
  if (elapsed_us < cycle_time_us)
  {
    delayMicroseconds(cycle_time_us - elapsed_us); 
  }
}