#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
// Khởi tạo đối tượng LCD I2C (địa chỉ LCD thường là 0x27 hoặc 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 16x2 với địa chỉ I2C 0x27

/**
 * @brief Hiển thị nội dung lên màn hình LCD.
 * 
 * @param line Dòng cần hiển thị (0 hoặc 1 cho màn hình LCD 16x2).
 * @param content Nội dung cần hiển thị.
 */
void displayOnLCD(int line, const char* content) {
    if (line < 0 || line > 1) {
        Serial.println("Dòng không hợp lệ! Chỉ hỗ trợ dòng 0 hoặc 1.");
        return;
    }

    lcd.setCursor(0, line);  // Đặt con trỏ đến dòng cụ thể
    lcd.print("                "); // Xóa nội dung dòng hiện tại
    lcd.setCursor(0, line);  // Đặt lại con trỏ
    lcd.print(content);      // In nội dung mới
}

void setup() {
    // Khởi động Serial Monitor
    Serial.begin(9600);

    // Khởi động LCD I2C
    lcd.init();       // Khởi tạo LCD
    lcd.backlight();  // Bật đèn nền LCD

    lcd.print("Initializing..."); // Hiển thị thông báo khởi động
    delay(2000); // Chờ 2 giây
    lcd.clear(); // Xóa màn hình
}

void loop() {
    // Ví dụ sử dụng hàm hiển thị
    displayOnLCD(0, "Hello, World!"); // Hiển thị dòng 0
    displayOnLCD(1, "Arduino I2C LCD"); // Hiển thị dòng 1
    delay(3000); // Chờ 3 giây

    displayOnLCD(0, "Temp: 25 C");    // Hiển thị dòng 0
    displayOnLCD(1, "Humidity: 45%"); // Hiển thị dòng 1
    delay(3000); // Chờ 3 giây
}
