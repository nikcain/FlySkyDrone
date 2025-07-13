#ifndef pid_h
#define pid_h

#include "FastIMU.h"
#include <Wire.h>

class pid
{
  public:
    void initialise();
    void calculate(int pitch_, int roll_, int yaw_, int throttle_);

    float pid_pitch, pid_roll, pid_yaw;
    float cycle_time_seconds;

  private:
    MPU6050 IMU;
    
    // state variables (held across calculations)
    float roll_last_integral = 0.0;
    float roll_last_error = 0.0;
    float pitch_last_integral = 0.0;
    float pitch_last_error = 0.0;
    float yaw_last_integral = 0.0;
    float yaw_last_error = 0.0;

    float max_rate_roll = 30.0;
    float max_rate_pitch = 30.0;
    float max_rate_yaw = 50.0;

    // PID Controller values
    float pid_roll_kp = 0.00043714285;
    float pid_roll_ki = 0.00255;
    float pid_roll_kd = 0.00002571429;
    float pid_pitch_kp = pid_roll_kp;
    float pid_pitch_ki = pid_roll_ki;
    float pid_pitch_kd = pid_roll_kd;
    float pid_yaw_kp = 0.001714287;
    float pid_yaw_ki = 0.003428571;
    float pid_yaw_kd = 0.0;
};


#endif // pid_h