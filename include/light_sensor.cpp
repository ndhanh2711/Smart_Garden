#include <Arduino.h>
#include <Adafruit_Sensor.h>
 #include <Wire.h>
// Chân kết nối quang trở
#define LDR_PIN 34   // Chân analog đọc dữ liệu từ quang trở

void setup() {
    // Khởi tạo kết nối Serial để debug
    Serial.begin(9600);
}

void loop() {
    // Đọc giá trị từ quang trở
    int ldrValue = analogRead(LDR_PIN);

    // In giá trị ra Serial Monitor
    Serial.print("LDR Value: ");
    Serial.println(ldrValue);

    // Đợi 500ms trước khi đọc giá trị tiếp theo
    delay(1000);
}
