#include <IBusBM.h>
#include "pid.h"
#include "MPU6050.h"
#include <ESP32Servo.h>

IBusBM IBus; 

pid pidcalc;

// motor pins
const int mot2_pin = 13; // right front
const int mot3_pin = 15; // left front
const int mot1_pin = 32; // right back
const int mot4_pin = 23; // left back

int m1_drive, m2_drive, m3_drive, m4_drive;

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
}

void loop() {
  char pitch, roll, yaw, throttle;

  IBus.loop();
  throttle = IBus.readChannel(2);
  pitch = IBus.readChannel(1);
  yaw = IBus.readChannel(3);
  roll = IBus.readChannel(0);
  
  pidcalc.calculate(pitch, roll, yaw, throttle);
  float pitch_PID, roll_PID, yaw_PID;
  pidcalc.getValues(pitch_PID, roll_PID, yaw_PID);

  m1_drive = (throttle + pitch_PID - roll_PID + yaw_PID > 255) ? 255 : throttle + pitch_PID - roll_PID + yaw_PID;
  m2_drive = (throttle + pitch_PID + roll_PID - yaw_PID > 255) ? 255 : throttle + pitch_PID + roll_PID - yaw_PID;
  m3_drive = (throttle - pitch_PID + roll_PID + yaw_PID > 255) ? 255 : throttle - pitch_PID + roll_PID + yaw_PID;
  m4_drive = (throttle - pitch_PID - roll_PID - yaw_PID > 255) ? 255 : throttle - pitch_PID - roll_PID - yaw_PID;

  // apply PWM
  ledcWrite(mot1_pin, m1_drive); 
  ledcWrite(mot2_pin, m2_drive);
  ledcWrite(mot3_pin, m3_drive);
  ledcWrite(mot4_pin, m4_drive);

  delay(20);
}