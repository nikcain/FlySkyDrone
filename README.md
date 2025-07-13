# FlySkyDrone

Attempt to build my own drone. Initially 'inspired' by https://diyprojectslab.com/make-esp8266-drone-wallclimb-drone/ but then decided to follow Tim Hanewich's scout controller (https://github.com/TimHanewich/scout) since he does a great job explaining it in great detail.

The drone will be an ESP-WROOM-32 dev board with an MPU6050 gyro chip. I've got four 8520 15000kV brushed motors, with a vague guess at suitable propellers. For ESCs I'll be using mosfets (IRLZ34N) with PWM managed by the ESP32. I won't be using the wifi for control since I've got a FlySky A8S receiver, and this will be more responsive, plus less code.

Current state is that the hardware is complete, and now concentrating on the code.
