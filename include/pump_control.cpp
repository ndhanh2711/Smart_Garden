#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define PUMP_PIN 4      // Chân kết nối máy bơm
#define PWM_CHANNEL 0   // Kênh PWM cho máy bơm
#define PWM_FREQ 5000   // Tần số PWM (5 kHz)
#define PWM_RESOLUTION 8 // Độ phân giải 8-bit (0-255)

/**
 * @brief Hàm khởi tạo máy bơm.
 * 
 * Cấu hình PWM cho chân điều khiển máy bơm.
 */
void Pump_Init() {
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // Cấu hình kênh PWM
    ledcAttachPin(PUMP_PIN, PWM_CHANNEL);             // Gắn kênh PWM với chân PUMP_PIN
}

/**
 * @brief Hàm điều khiển máy bơm theo mức.
 * 
 * @param level Mức bơm, giá trị từ 0 đến 5 (5 mức bơm).
 */
void Pump_Control(int level) {
    // Giới hạn giá trị đầu vào trong khoảng 0 - 5
    level = constrain(level, 0, 5);

    // Chuyển đổi mức từ 0-5 sang giá trị duty cycle 0-255
    int dutyCycle = map(level, 0, 5, 0, 255);

    // Gửi giá trị duty cycle đến PWM
    ledcWrite(PWM_CHANNEL, dutyCycle);

    // In thông tin mức bơm ra Serial Monitor (tùy chọn)
    Serial.print("Pump Level: ");
    Serial.print(level);
    Serial.print(" -> Duty Cycle: ");
    Serial.println(dutyCycle);
}

void setup() {
    Serial.begin(9600); // Khởi tạo Serial Monitor
    Pump_Init();          // Khởi tạo máy bơm
}

void loop() {
        Pump_Control(3); // Gọi hàm điều khiển máy bơm
}