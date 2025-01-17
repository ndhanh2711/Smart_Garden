#include <Arduino.h>           // Thư viện chính để lập trình cho các board Arduino
#include <Adafruit_Sensor.h>   // Thư viện hỗ trợ giao tiếp với các cảm biến Adafruit (không sử dụng trong đoạn code này nhưng có thể dùng trong các phần khác)
#include <Wire.h>              // Thư viện hỗ trợ giao tiếp I2C (không sử dụng trong đoạn code này nhưng có thể dùng trong các phần khác)

/**
 * @brief Định nghĩa các thông số cho máy bơm và kênh PWM.
 * 
 * - `PUMP_PIN`: Chân điều khiển máy bơm.
 * - `PWM_CHANNEL`: Kênh PWM sử dụng để điều khiển.
 * - `PWM_FREQ`: Tần số PWM, đặt ở 5 kHz để tối ưu cho động cơ.
 * - `PWM_RESOLUTION`: Độ phân giải của PWM, chọn 8-bit (giá trị từ 0 đến 255).
 */
#define PUMP_PIN 4           // Chân kết nối máy bơm
#define PWM_CHANNEL 0        // Kênh PWM cho máy bơm
#define PWM_FREQ 5000        // Tần số PWM (5 kHz)
#define PWM_RESOLUTION 8     // Độ phân giải 8-bit (0-255)

/**
 * @brief Hàm khởi tạo máy bơm.
 * 
 * Cấu hình kênh PWM và liên kết chân điều khiển với kênh PWM.
 */
void Pump_Init() {
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // Cấu hình kênh PWM với tần số và độ phân giải
    ledcAttachPin(PUMP_PIN, PWM_CHANNEL);             // Gắn kênh PWM với chân PUMP_PIN
}

/**
 * @brief Hàm điều khiển máy bơm theo mức bơm.
 * 
 * - Giới hạn mức bơm từ 0 đến 5.
 * - Chuyển đổi mức bơm thành giá trị duty cycle (0-255).
 * - Truyền giá trị duty cycle đến kênh PWM để điều khiển máy bơm.
 * - In thông tin mức bơm và duty cycle ra Serial Monitor để debug.
 * 
 * @param level Mức bơm, giá trị từ 0 (tắt bơm) đến 5 (bơm mạnh nhất).
 */
void Pump_Control(int8_t level) {
    // Giới hạn giá trị đầu vào từ 0 đến 5
    level = constrain(level, 0, 3);

    // Chuyển đổi mức bơm (0-5) sang duty cycle (0-255)
    int8_t dutyCycle = map(level, 0, 3, 0, 255);

    // Gửi giá trị duty cycle đến kênh PWM
    ledcWrite(PWM_CHANNEL, dutyCycle);

    // In thông tin mức bơm và duty cycle ra Serial Monitor
    Serial.print("Pump Level: ");       // Chuỗi thông báo mức bơm
    Serial.print(level);                // In giá trị mức bơm
    Serial.print(" -> Duty Cycle: ");   // Chuỗi thông báo duty cycle
    Serial.println(dutyCycle);          // In giá trị duty cycle
}

/**
 * @brief Hàm khởi tạo chương trình.
 * 
 * - Khởi tạo Serial Monitor để in thông tin.
 * - Gọi hàm khởi tạo máy bơm `Pump_Init`.
 */
void setup() {
    Serial.begin(9600);  // Khởi tạo Serial Monitor với tốc độ 9600 baud
    Pump_Init();         // Khởi tạo máy bơm
}

/**
 * @brief Hàm vòng lặp chính.
 * 
 * - Gọi hàm điều khiển máy bơm với mức bơm cố định (3 trong ví dụ này).
 * - Mức 3 tương ứng với duty cycle trung bình.
 */
void loop() {
    Pump_Control(3); // Điều khiển máy bơm ở mức 3
    delay(1000);     // Chờ 1 giây trước khi lặp lại
}
