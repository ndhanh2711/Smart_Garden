#include <Arduino.h>

int8_t moisture, sensor_analog;
const int8_t sensor_pin = 34;
 void setup(){
  Serial.begin(9600);

 }
 void loop(){
  sensor_analog = analogRead(sensor_pin);
  moisture = (100 - ((sensor_analog / 4095.00) * 100));
  Serial.print("Moisture = ");
  Serial.print(moisture);
  Serial.println("%");
  delay(1000);
 }