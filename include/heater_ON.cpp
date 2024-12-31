#include <Arduino.h>
#include <Adafruit_Sensor.h>
 #include <Wire.h>
#define GPIO_PIN 18  // Chân GPIO điều khiển

// Hàm giả lập trả về nhiệt độ
float getTemperature() {
    // Giá trị giả định (có thể thay đổi khi cần)
    return 8.5;  // Giả định nhiệt độ là 8.5°C
}

// Hàm kiểm tra nhiệt độ và bật/tắt GPIO 18
void controlPumpByTemperature() {
    float temperature = getTemperature();  // Gọi hàm lấy nhiệt độ giả định

    // Hiển thị nhiệt độ lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);

    // Điều kiện bật GPIO 18 nếu nhiệt độ nhỏ hơn 10°C
    if (temperature < 10) {
        digitalWrite(GPIO_PIN, HIGH);  // Bật GPIO 18
        Serial.println("GPIO 18 được bật.");
    } else {
        digitalWrite(GPIO_PIN, LOW);   // Tắt GPIO 18
        Serial.println("GPIO 18 được tắt.");
    }
}

void setup() {
    Serial.begin(9600);

    // Cấu hình chân GPIO 18 làm đầu ra
    pinMode(GPIO_PIN, OUTPUT);
    Serial.println("Bắt đầu kiểm tra nhiệt độ...");
}

void loop() {
    controlPumpByTemperature();  // Gọi hàm kiểm tra và điều khiển
    delay(2000);                 // Chờ 2 giây trước lần kiểm tra tiếp theo
}
