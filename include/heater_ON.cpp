#include <Arduino.h>
#include <Adafruit_Sensor.h> // Thư viện Adafruit Sensor (không được sử dụng trong mã này, nhưng có thể cần nếu kết hợp với cảm biến thực)
#include <Wire.h>            // Thư viện Wire để giao tiếp I2C (không sử dụng trong đoạn mã này)

#define GPIO_PIN 18  // Định nghĩa chân GPIO 18 để điều khiển máy bơm hoặc thiết bị

/**
 * @brief Hàm giả lập trả về nhiệt độ hiện tại.
 * 
 * Hàm này trả về giá trị nhiệt độ cố định, được sử dụng để kiểm tra logic trong hệ thống.
 * Có thể thay thế bằng giá trị đo thực tế từ cảm biến nhiệt độ.
 * 
 * @return float Giá trị nhiệt độ giả lập (°C).
 */
float getTemperature() {
    return 8.5; // Nhiệt độ giả định là 8.5°C
}

/**
 * @brief Hàm kiểm tra nhiệt độ và bật/tắt thiết bị dựa trên ngưỡng nhiệt độ.
 * 
 * - Nếu nhiệt độ nhỏ hơn 10°C, bật thiết bị (GPIO_PIN).
 * - Nếu nhiệt độ lớn hơn hoặc bằng 10°C, tắt thiết bị (GPIO_PIN).
 */
void controlPumpByTemperature() {
    float temperature = getTemperature();  // Lấy giá trị nhiệt độ

    // Hiển thị nhiệt độ lên Serial Monitor
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);

    // Kiểm tra điều kiện và bật/tắt GPIO_PIN
    if (temperature < 10) {
        digitalWrite(GPIO_PIN, HIGH);  // Bật GPIO_PIN
        Serial.println("GPIO 18 được bật.");
    } else {
        digitalWrite(GPIO_PIN, LOW);   // Tắt GPIO_PIN
        Serial.println("GPIO 18 được tắt.");
    }
}

/**
 * @brief Cấu hình ban đầu cho hệ thống.
 * 
 * - Khởi tạo Serial Monitor với tốc độ 9600 baud.
 * - Cấu hình GPIO_PIN làm đầu ra.
 */
void setup() {
    Serial.begin(9600);           // Khởi tạo giao tiếp Serial
    pinMode(GPIO_PIN, OUTPUT);    // Cấu hình GPIO_PIN làm đầu ra
    Serial.println("Bắt đầu kiểm tra nhiệt độ...");
}

/**
 * @brief Vòng lặp chính của chương trình.
 * 
 * - Gọi hàm kiểm tra và điều khiển thiết bị dựa trên nhiệt độ.
 * - Chờ 2 giây giữa các lần kiểm tra để giảm tải CPU.
 */
void loop() {
    controlPumpByTemperature();  // Kiểm tra nhiệt độ và điều khiển thiết bị
    delay(2000);                 // Chờ 2 giây trước lần kiểm tra tiếp theo
}
