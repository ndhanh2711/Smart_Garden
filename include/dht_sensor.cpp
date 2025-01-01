#include "DHT.h"               // Thư viện hỗ trợ đọc cảm biến DHT (đo nhiệt độ và độ ẩm)
#include <Wire.h>              // Thư viện hỗ trợ giao tiếp I2C
#include <Adafruit_Sensor.h>   // Thư viện hỗ trợ giao tiếp với các cảm biến Adafruit

/**
 * @brief Khởi tạo các thành phần cần thiết cho cảm biến DHT22.
 * 
 * - DHT22 là loại cảm biến đo nhiệt độ và độ ẩm không khí.
 * - Cảm biến sẽ được kết nối với chân số 32 trên vi điều khiển.
 */

#define DHTPIN 32             // Chân kết nối dữ liệu cảm biến DHT22 với vi điều khiển
#define DHTTYPE DHT22         // Loại cảm biến đang sử dụng là DHT22
DHT dht(DHTPIN, DHTTYPE);     // Đối tượng cảm biến DHT, được khởi tạo với chân DHTPIN và loại DHTTYPE

/**
 * @brief Hàm khởi tạo chương trình.
 * 
 * - Khởi tạo giao tiếp Serial để in dữ liệu lên màn hình console.
 * - Khởi tạo cảm biến DHT để chuẩn bị sẵn sàng thu thập dữ liệu.
 */
void setup() {
  Serial.begin(9600);           // Thiết lập tốc độ truyền dữ liệu qua Serial (9600 baud)
  Serial.println(F("DHTxx test!"));  // Thông báo khởi động chương trình
  dht.begin();                  // Bắt đầu hoạt động cảm biến DHT
}

/**
 * @brief Hàm vòng lặp chính của chương trình.
 * 
 * - Đọc dữ liệu từ cảm biến DHT22: nhiệt độ (Celsius, Fahrenheit) và độ ẩm.
 * - Kiểm tra tính hợp lệ của dữ liệu.
 * - Tính chỉ số nhiệt Heat Index dựa trên dữ liệu thu thập được.
 * - Hiển thị kết quả qua giao tiếp Serial.
 */
void loop() {
  delay(2000); // Chờ 2 giây giữa các lần đo (thời gian trễ của cảm biến)

  // Đọc độ ẩm từ cảm biến
  float h = dht.readHumidity();

  // Đọc nhiệt độ ở đơn vị Celsius (mặc định)
  float t = dht.readTemperature();

  // Đọc nhiệt độ ở đơn vị Fahrenheit
  float f = dht.readTemperature(true);

  // Kiểm tra nếu dữ liệu đọc bị lỗi (giá trị không hợp lệ)
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));  // Báo lỗi nếu cảm biến không trả về dữ liệu hợp lệ
    return; // Kết thúc vòng lặp hiện tại và thử lại ở vòng tiếp theo
  }

  // Tính chỉ số nhiệt Heat Index ở đơn vị Fahrenheit
  float hif = dht.computeHeatIndex(f, h);

  // Tính chỉ số nhiệt Heat Index ở đơn vị Celsius
  float hic = dht.computeHeatIndex(t, h, false);

  // Hiển thị độ ẩm lên màn hình Serial
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  "));

  // Hiển thị nhiệt độ ở Celsius
  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
}
