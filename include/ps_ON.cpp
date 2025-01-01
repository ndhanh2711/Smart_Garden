#include <Arduino.h>
#include <Adafruit_Sensor.h> // Thư viện Adafruit Sensor (không được sử dụng trong mã này, nhưng có thể cần nếu kết hợp với cảm biến thực)
#include <Wire.h>            // Thư viện Wire để giao tiếp I2C (không sử dụng trong đoạn mã này)

#define MIST_SYSTEM_PIN 19  // Định nghĩa chân GPIO 19 để điều khiển hệ thống phun sương

/**
 * @brief Hàm giả lập trả về độ ẩm hiện tại.
 * 
 * Hàm này trả về giá trị độ ẩm cố định, được sử dụng để kiểm tra logic trong hệ thống.
 * Có thể thay thế bằng giá trị đo thực tế từ cảm biến độ ẩm.
 * 
 * @return float Giá trị độ ẩm giả lập (%RH).
 */
float getHumidity() {
    return 45.0; // Độ ẩm giả định là 45%
}

/**
 * @brief Hàm kiểm tra độ ẩm và bật/tắt hệ thống phun sương dựa trên ngưỡng độ ẩm.
 * 
 * - Nếu độ ẩm nhỏ hơn 50%, bật hệ thống phun sương (MIST_SYSTEM_PIN).
 * - Nếu độ ẩm lớn hơn hoặc bằng 50%, tắt hệ thống phun sương (MIST_SYSTEM_PIN).
 */
void controlMistSystemByHumidity() {
    float humidity = getHumidity();  // Lấy giá trị độ ẩm

    // Hiển thị độ ẩm lên Serial Monitor
    Serial.print("Độ ẩm hiện tại: ");
    Serial.println(humidity);

    // Kiểm tra điều kiện và bật/tắt MIST_SYSTEM_PIN
    if (humidity < 50) {
        digitalWrite(MIST_SYSTEM_PIN, HIGH);  // Bật MIST_SYSTEM_PIN
        Serial.println("Hệ thống phun sương được bật.");
    } else {
        digitalWrite(MIST_SYSTEM_PIN, LOW);   // Tắt MIST_SYSTEM_PIN
        Serial.println("Hệ thống phun sương được tắt.");
    }
}

/**
 * @brief Cấu hình ban đầu cho hệ thống.
 * 
 * - Khởi tạo Serial Monitor với tốc độ 9600 baud.
 * - Cấu hình MIST_SYSTEM_PIN làm đầu ra.
 */
void setup() {
    Serial.begin(9600);                // Khởi tạo giao tiếp Serial
    pinMode(MIST_SYSTEM_PIN, OUTPUT);  // Cấu hình MIST_SYSTEM_PIN làm đầu ra
    Serial.println("Bắt đầu kiểm tra độ ẩm...");
}

/**
 * @brief Vòng lặp chính của chương trình.
 * 
 * - Gọi hàm kiểm tra và điều khiển hệ thống phun sương dựa trên độ ẩm.
 * - Chờ 2 giây giữa các lần kiểm tra để giảm tải CPU.
 */
void loop() {
    controlMistSystemByHumidity();  // Kiểm tra độ ẩm và điều khiển hệ thống phun sương
    delay(2000);                    // Chờ 2 giây trước lần kiểm tra tiếp theo
}
