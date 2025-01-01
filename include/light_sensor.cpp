#include <Arduino.h>           // Thư viện chính để lập trình cho các board Arduino
#include <Adafruit_Sensor.h>   // Thư viện hỗ trợ giao tiếp với các cảm biến Adafruit
#include <Wire.h>              // Thư viện hỗ trợ giao tiếp I2C

/**
 * @brief Định nghĩa chân kết nối cho quang trở (LDR).
 * 
 * Quang trở (LDR - Light Dependent Resistor) là một cảm biến ánh sáng
 * có khả năng thay đổi điện trở tùy thuộc vào cường độ ánh sáng.
 * Giá trị được đọc là dạng tín hiệu analog (0-4095 trên ESP32).
 */
#define LDR_PIN 34   // Chân analog kết nối với quang trở

/**
 * @brief Hàm khởi tạo chương trình.
 * 
 * - Thiết lập tốc độ giao tiếp Serial để in dữ liệu ra màn hình.
 * - Sử dụng để debug và kiểm tra hoạt động của cảm biến.
 */
void setup() {
    Serial.begin(9600);        // Khởi tạo giao tiếp Serial với tốc độ 9600 baud
}

/**
 * @brief Hàm vòng lặp chính của chương trình.
 * 
 * - Đọc giá trị ánh sáng từ quang trở thông qua chân analog.
 * - In giá trị thu được lên Serial Monitor.
 * - Thêm thời gian trễ giữa các lần đo để ổn định hệ thống.
 */
void loop() {
    // Đọc tín hiệu từ quang trở thông qua chân analog
    int ldrValue = analogRead(LDR_PIN);

    // Hiển thị giá trị đọc được ra Serial Monitor
    Serial.print("LDR Value: ");   // In chuỗi "LDR Value: "
    Serial.println(ldrValue);     // In giá trị tín hiệu đọc được

    // Chờ 1000ms (1 giây) trước khi đọc giá trị tiếp theo
    delay(1000);
}
