#include <IBusBM.h>

IBusBM IBus; 

void setup() {
  Serial.begin(115200);
  IBus.begin(Serial2, IBUSBM_NOTIMER);  
}

void loop() {
  IBus.loop();
  Serial.print("Throttle: " + String(IBus.readChannel(2)));
  Serial.print(" Pitch: " + String(IBus.readChannel(1)));
  Serial.print(" Yaw: " + String(IBus.readChannel(3)));
  Serial.println(" Roll: " + String(IBus.readChannel(0)));
  delay(20);
}