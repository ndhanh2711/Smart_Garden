//---------------------------------------Khai báo thư viện các ngoại vi--------------------------------------
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>


//-----------------------------------Khởi tạo các chân kết nối với ngoại vi----------------------------------

/**
 * @brief Khởi tạo các thành phần cần thiết cho cảm biến DHT22.
 * 
 * - DHT22 là loại cảm biến đo nhiệt độ và độ ẩm không khí.
 * - Cảm biến sẽ được kết nối với chân số 32 trên vi điều khiển.
 */
#define DHTPIN 32     
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);

/**
 * @brief Định nghĩa các thông số cho máy bơm và kênh PWM.
 * 
 * - `PUMP_PIN`: Chân điều khiển máy bơm.
 * - `PWM_CHANNEL`: Kênh PWM sử dụng để điều khiển.
 * - `PWM_FREQ`: Tần số PWM, đặt ở 5 kHz để tối ưu cho động cơ.
 * - `PWM_RESOLUTION`: Độ phân giải của PWM, chọn 8-bit (giá trị từ 0 đến 255).
 */
#define PUMP_PIN 4                // Chân kết nối máy bơm
#define PWM_CHANNEL 0             // Kênh PWM cho máy bơm
#define PWM_FREQ 5000             // Tần số PWM (5 kHz)
#define PWM_RESOLUTION 8          // Độ phân giải 8-bit (0-255)

// Chân kết nối quang trở------------------------------------------------------
#define LDR_PIN 34   // Chân analog đọc dữ liệu từ quang trở

//Chân bật đèn sưởi------------------------------------------------------------
#define SUOI_GPIO_PIN 18

//Chân bật phun sương----------------------------------------------------------
#define PS_GPIO_PIN 19

//Khởi tạo LCD-----------------------------------------------------------------
// Khởi tạo đối tượng LCD I2C (địa chỉ LCD thường là 0x27 hoặc 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 16x2 với địa chỉ I2C 0x27

//Chân Analog kết nối cảm biến độ ẩm đất---------------------------------------
// Khai báo chân cảm biến
const int sensor_pin = 33;
//----------------------------------------Cấu trúc lưu trữ các giá trị đầu vào----------------------------------
struct Smart_Garden{
    float Temperature_Value;
    int Light_Value;
    int Humidity_Value;
    int SoilMoisture_Value;
};
Smart_Garden smart_Garden;

//---------------------------------------------------LCD_DISPLAY------------------------------------------------
/**
 * @brief Hiển thị nội dung lên màn hình LCD.
 * 
 * @param line Dòng cần hiển thị (0 hoặc 1 cho màn hình LCD 16x2).
 * @param content Nội dung cần hiển thị.
 */
void LCD_Display(int line, const char* content) {
    if (line < 0 || line > 1) {
        Serial.println("Dòng không hợp lệ! Chỉ hỗ trợ dòng 0 hoặc 1.");
        return;
    }
    lcd.setCursor(0, line);  // Đặt con trỏ đến dòng cụ thể
    lcd.print("                "); // Xóa nội dung dòng hiện tại
    lcd.setCursor(0, line);  // Đặt lại con trỏ
    lcd.print(content);      // In nội dung mới
}
//---------------------------------------------Hàm chức năng của DHT22------------------------------------------

/**
 * @brief Hàm DHTdata_Read() đọc dữ liệu từ cảm biến DHT và xử lý thông tin.
 * 
 * 1. Đọc độ ẩm từ cảm biến DHT và lưu vào biến `h`.
 * 2. Đọc nhiệt độ theo đơn vị Celsius và Fahrenheit, lưu vào `t` và `f`.
 * 3. Kiểm tra xem dữ liệu đọc được có hợp lệ hay không:
 *    - Nếu bất kỳ giá trị nào là `NaN`, in ra thông báo lỗi trên Serial Monitor.
 * 4. Tính toán chỉ số nhiệt (Heat Index):
 *    - `hif`: Chỉ số nhiệt theo Fahrenheit.
 *    - `hic`: Chỉ số nhiệt theo Celsius.
 * 5. Cập nhật giá trị nhiệt độ và độ ẩm vào cấu trúc dữ liệu `smart_Garden`.
 * 
 * @note Các giá trị tính toán chỉ số nhiệt hiện không được in ra Serial Monitor,
 * nhưng có thể sử dụng nếu cần trong các ứng dụng khác.
 */
void DHTdata_Read() {
    // Đọc độ ẩm
    float h = dht.readHumidity();
    // Đọc nhiệt độ (Celsius)
    float t = dht.readTemperature();
    // Đọc nhiệt độ (Fahrenheit)
    float f = dht.readTemperature(true);

    // Kiểm tra lỗi khi đọc dữ liệu
    if (isnan(h) || isnan(t) || isnan(f)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    // Tính chỉ số nhiệt (Heat Index) trong Fahrenheit và Celsius
    float hif = dht.computeHeatIndex(f, h);
    float hic = dht.computeHeatIndex(t, h, false);

    // // In dữ liệu ra Serial Monitor
    // Serial.print(F("Humidity: "));
    // Serial.print(h);
    // Serial.print(F("%  Temperature: "));
    // Serial.print(t);
    // Serial.print(F("°C"));
    smart_Garden.Temperature_Value = 0.2075 + 1.0201 * t - 0.0004 * t * t; //Công thữc hiệu chuẩn của nhiệt độ
    smart_Garden.Humidity_Value = 0.4189 + 1.0047 * h - 0.000129 * h * h;  //Công thức hiệu chuẩn của độ ẩm
}

//--------------------------------------------Hàm chức năng của máy bơm-----------------------------------------
/**
 * @brief Hàm Pump_Init() khởi tạo cấu hình PWM để điều khiển máy bơm.
 * 
 * 1. Cấu hình kênh PWM với các thông số:
 *    - `PWM_CHANNEL`: Kênh PWM được sử dụng.
 *    - `PWM_FREQ`: Tần số PWM (đơn vị: Hz).
 *    - `PWM_RESOLUTION`: Độ phân giải của tín hiệu PWM (số bit).
 * 2. Gắn chân điều khiển máy bơm (`PUMP_PIN`) với kênh PWM đã cấu hình.
 * 
 * @note Hàm này cần được gọi trong phần `setup()` để khởi tạo trước khi sử dụng máy bơm.
 */
void Pump_Init() {
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // Cấu hình kênh PWM
    ledcAttachPin(PUMP_PIN, PWM_CHANNEL);             // Gắn kênh PWM với chân PUMP_PIN
}
/**
 * @brief Hàm Pump_Control() điều khiển máy bơm theo mức bơm từ 0 đến 3.
 * 
 * @param level Giá trị mức bơm (0-3):
 *    - 0: Tắt máy bơm (duty cycle = 0%).
 *    - 1-3: Điều chỉnh mức bơm với duty cycle tương ứng (tăng từ 0% đến 100%).
 * 
 * - Giá trị `level` được giới hạn trong khoảng 0-3 để tránh lỗi.
 * - Dùng hàm `map()` để chuyển đổi mức bơm sang duty cycle tương ứng (0-255).
 * - Gửi giá trị duty cycle qua PWM để điều chỉnh cường độ bơm.
 * 
 * @note Thông tin mức bơm và duty cycle được in ra Serial Monitor để kiểm tra (có thể bỏ qua nếu không cần).
 */
void Pump_Control(int level) {
    // Giới hạn giá trị đầu vào trong khoảng 0 - 5
    level = constrain(level, 0, 3);

    // Chuyển đổi mức từ 0-3 sang giá trị duty cycle 0-255
    int dutyCycle = map(level, 0, 3, 0, 150);

    // Gửi giá trị duty cycle đến PWM
    ledcWrite(PWM_CHANNEL, dutyCycle);

    // In thông tin mức bơm ra Serial Monitor (tùy chọn)
    Serial.print("Pump Level: ");
    Serial.print(level);
    Serial.print(" -> Duty Cycle: ");
    Serial.println(dutyCycle);
}
//-----------------------------------------Hàm đọc giá trị của quang trở----------------------------------------

//---------------------------------------------Hàm lấy độ ẩm đất------------------------------------------------
int readMoisture(int pin) {
    int sensor_analog = analogRead(pin);  // Đọc giá trị analog từ chân cảm biến
    int moisture = 100 - ((sensor_analog / 4095.0) * 100); // Tính toán độ ẩm
    smart_Garden.SoilMoisture_Value = moisture / 40 * 100;
    return smart_Garden.SoilMoisture_Value ;  // Trả về giá trị độ ẩm
}
//------------------------------------------Hàm lấy các giá trị đầu vào-----------------------------------------

/**
 * @brief Hàm getHumidity() giả lập giá trị độ ẩm từ cảm biến DHT.
 * 
 * - Gọi hàm `DHTdata_Read()` để cập nhật dữ liệu từ cảm biến DHT.
 * - Trả về giá trị độ ẩm giả định (trong ví dụ này là 20%).
 * 
 * @return int Giá trị độ ẩm (đơn vị: %).
 */
int getHumidity() {
    DHTdata_Read();
    return smart_Garden.Humidity_Value;  // Giả định độ ẩm là 45%
}
/**
 * @brief Hàm getLDRvalue() giả lập giá trị cảm biến ánh sáng LDR.
 * 
 * - Đọc giá trị từ cảm biến LDR (cảm biến ánh sáng).
 * - Trong ví dụ này, giá trị đọc từ cảm biến được giả lập thành một giá trị cố định.
 * 
 * @return int Giá trị cảm biến ánh sáng (0-1023, tùy thuộc vào giá trị analogRead() trên ESP32).
 */
int getLDRvalue(){
    //return analogRead(LDR_PIN);
    int check_LDR = analogRead(LDR_PIN);
    smart_Garden.Light_Value = check_LDR;
    return check_LDR;
}
/**
 * @brief Hàm getTemperature() giả lập giá trị nhiệt độ từ cảm biến DHT.
 * 
 * - Gọi hàm `DHTdata_Read()` để cập nhật dữ liệu từ cảm biến DHT.
 * - Trả về giá trị nhiệt độ giả định (trong ví dụ này là 5°C).
 * 
 * @return float Giá trị nhiệt độ (đơn vị: °C).
 */
float getTemperature() {
    DHTdata_Read();
    return smart_Garden.Temperature_Value;  // Giả định nhiệt độ là 8.5°C
}
/**
 * @brief Hàm getMoisture() giả lập giá trị độ ẩm đất.
 * 
 * - Trả về giá trị độ ẩm đất giả định (giả sử là 0, có thể thay đổi theo yêu cầu).
 * 
 * @return int Giá trị độ ẩm đất.
 */
int getMoisture(){
    int moisture = readMoisture(sensor_pin); // Gọi hàm để đọc độ ẩm
    // Serial.print("Moisture = ");
    // Serial.print(moisture);
    // Serial.println("%");
    return moisture;
}
//-----------------------------------------------Hành động đèn sưởi----------------------------------------------
/**
 * @brief Hàm điều khiển bật hệ thống sưởi (GPIO_PIN) khi nhiệt độ thấp.
 * 
 * - Gọi hàm `getTemperature()` để lấy giá trị nhiệt độ hiện tại.
 * - Nếu nhiệt độ nhỏ hơn mức ngưỡng, bật hệ thống sưởi (GPIO 18).
 * - Hiển thị nhiệt độ lên Serial Monitor để kiểm tra.
 */
void control_SUOI_ON() {
    float temperature = getTemperature();  // Gọi hàm lấy nhiệt độ giả định

    // Hiển thị nhiệt độ lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);
    digitalWrite(SUOI_GPIO_PIN, HIGH);  // Bật GPIO 18
}
/**
 * @brief Hàm điều khiển tắt hệ thống sưởi (GPIO_PIN) khi nhiệt độ đạt mức yêu cầu.
 * 
 * - Gọi hàm `getTemperature()` để lấy giá trị nhiệt độ hiện tại.
 * - Nếu nhiệt độ đạt hoặc vượt mức ngưỡng, tắt hệ thống sưởi (GPIO 18).
 * - Hiển thị nhiệt độ lên Serial Monitor để kiểm tra.
 */
void control_SUOI_OFF() {
    float temperature = getTemperature();  // Gọi hàm lấy nhiệt độ giả định

    // Hiển thị nhiệt độ lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);
    digitalWrite(SUOI_GPIO_PIN, LOW);  // Bật GPIO 18
}

//----------------------------------------------Hàm khởi động phun sương---------------------------------------

/**
 * @brief Hàm điều khiển bật hệ thống sưởi (GPIO_PIN) khi nhiệt độ thấp.
 * 
 * - Gọi hàm `getTemperature()` để lấy giá trị nhiệt độ hiện tại.
 * - Nếu nhiệt độ nhỏ hơn mức ngưỡng, bật hệ thống sưởi (GPIO 18).
 * - Hiển thị nhiệt độ lên Serial Monitor để kiểm tra.
 */
void control_PS_ON() {
    float temperature = getTemperature();  // Lấy nhiệt độ giả định
    float humidity = getHumidity();        // Lấy độ ẩm giả định

    // Hiển thị nhiệt độ và độ ẩm lên Serial
    // Serial.print("Nhiệt độ hiện tại: ");
    // Serial.println(temperature);
    // Serial.print("Độ ẩm hiện tại: ");
    // Serial.println(humidity);
    digitalWrite(PS_GPIO_PIN, HIGH);  // Bật GPIO 19
    Serial.println("GPIO 19 được bật.");
}
/**
 * @brief Hàm điều khiển tắt hệ thống sưởi (GPIO_PIN) khi nhiệt độ đạt mức yêu cầu.
 * 
 * - Gọi hàm `getTemperature()` để lấy giá trị nhiệt độ hiện tại.
 * - Nếu nhiệt độ đạt hoặc vượt mức ngưỡng, tắt hệ thống sưởi (GPIO 18).
 * - Hiển thị nhiệt độ lên Serial Monitor để kiểm tra.
 */
void control_PS_OFF() {
    float temperature = getTemperature();  // Lấy nhiệt độ giả định
    float humidity = getHumidity();        // Lấy độ ẩm giả định

    // Hiển thị nhiệt độ và độ ẩm lên Serial
    // Serial.print("Nhiệt độ hiện tại: ");
    // Serial.println(temperature);
    // Serial.print("Độ ẩm hiện tại: ");
    // Serial.println(humidity);
    digitalWrite(PS_GPIO_PIN, LOW);  // Bật GPIO 19
    Serial.println("GPIO 19 được tắt.");
}
//----------------------------------------------Hàm khởi tạo các ngoại vi---------------------------------------
/**
 * @brief Hàm setup() sẽ được gọi một lần khi hệ thống bắt đầu.
 * 
 * - Khởi tạo Serial Communication để có thể gửi dữ liệu tới Serial Monitor với tốc độ 9600 baud.
 * - Khởi tạo cảm biến DHT (nhiệt độ và độ ẩm) để có thể lấy giá trị đo từ cảm biến.
 * - Khởi tạo máy bơm, cấu hình PWM để điều khiển mức bơm.
 * - Cấu hình các chân GPIO dùng để điều khiển hệ thống sưởi ấm, phun sương và đọc giá trị cảm biến ánh sáng.
 * - In thông báo khởi động hệ thống lên Serial Monitor để người dùng biết trạng thái.
 */
void setup() {
Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  
  // Khởi tạo cảm biến nhiệt độ và độ ẩm
  dht.begin();
  
  // Khởi tạo máy bơm
  Pump_Init();
//   //-------------------------------Kích hoạt chế độ tiết kiệm năng lượng Light Sleep---------------------------


// // Cấu hình GPIO
// gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);  // Cảm biến 1
// gpio_set_direction(GPIO_NUM_25, GPIO_MODE_INPUT);  // Cảm biến 2
// gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT);  // Cảm biến 3

// gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);   // Đầu ra 1
// gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);  // Đầu ra 2
// gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);  // Đầu ra 3

// // Kích hoạt Light Sleep
// esp_light_sleep_start();
  // Khởi tạo các chân điều khiển thiết bị
  pinMode(SUOI_GPIO_PIN, OUTPUT);
  pinMode(PS_GPIO_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  // Khởi động LCD I2C
  lcd.init();       // Khởi tạo LCD
  lcd.backlight();  // Bật đèn nền LCD

  lcd.print("Smart Garden......"); // Hiển thị thông báo khởi động
  lcd.clear(); // Xóa màn hình
  // Bắt đầu in thông báo khởi động
  Serial.println("Khởi động hệ thống...");
}
//----------------------------------------------Các ngưỡng điều kiện--------------------------------------------

const float TEMP_MIN = 25.0;     // Ngưỡng bật sưởi ấm (độ C)
const float TEMP_MAX = 28.0;     // Ngưỡng tắt sưởi ấm
const float HUMIDITY_MIN = 50.0; // Ngưỡng bật phun sương
const float HUMIDITY_MAX = 60.0; // Ngưỡng tắt phun sương
const float SOIL_MIN1 = 60.0;    // Ngưỡng tưới mức 1
const float SOIL_MIN2 = 30.0;    // Ngưỡng tưới mức 2
const float SOIL_MIN3 = 15.0;    // Ngưỡng tưới mức 3
//const int LIGHT_MIN = 20;        // Ngưỡng ánh sáng bật sưởi ấm

//---------------------------------------------------Thời gian--------------------------------------------------

// Biến lưu giá trị thời gian hiện tại cho từng tác vụ
unsigned long lastTempHumCheck = 0; // Thời gian lần cuối đo nhiệt độ và độ ẩm
unsigned long lastSoilMoistureCheck = 0; // Thời gian lần cuối đo độ ẩm đất
unsigned long lastLightIntensityCheck = 0; // Thời gian lần cuối đo cường độ ánh sáng

// Khoảng thời gian đo cho từng cảm biến (ms)
const unsigned long tempHumInterval = 2000;         // Đo nhiệt độ, đô ẩm  
const unsigned long soilMoistureInterval = 2000;    // Đo độ ẩm đất   
const unsigned long lightIntensityInterval = 1000;  // Đo cường độ ánh sáng

// Biến thời gian cho việc cập nhật Serial Monitor
unsigned long lastMonitorUpdate = 0;
const unsigned long monitorUpdateInterval = 3000; // 3 giây

int displayState = 0; // Trạng thái hiển thị: 0 hoặc 1
//-----------------------------------------------Hàm chạy chương trình-------------------------------------------

void loop() {

    unsigned long currentMillis = millis();

    // 1. Đo nhiệt độ và độ ẩm không khí (5 giây/lần)
    if (currentMillis - lastTempHumCheck >= tempHumInterval) {
        lastTempHumCheck = currentMillis;

        // Đọc dữ liệu từ cảm biến
        smart_Garden.Temperature_Value = getTemperature();
        smart_Garden.Humidity_Value = getHumidity();

        // Điều khiển hệ thống sưởi
        if (smart_Garden.Temperature_Value < TEMP_MIN) {
            control_SUOI_ON();
        } else if(smart_Garden.Temperature_Value >= TEMP_MAX) {
            control_SUOI_OFF();
        }
        else{
            control_SUOI_ON();
        }

        // Điều khiển phun sương
        if (smart_Garden.Humidity_Value <= HUMIDITY_MIN) {
            control_PS_ON();
        } else if(smart_Garden.Humidity_Value >= HUMIDITY_MAX){
            control_PS_OFF();
        }
        else{
            control_PS_ON();
        }
    }

    // 2. Đo độ ẩm đất (7 giây/lần)
    if (currentMillis - lastSoilMoistureCheck >= soilMoistureInterval) {
        lastSoilMoistureCheck = currentMillis;

        // Đọc dữ liệu từ cảm biến
        smart_Garden.SoilMoisture_Value = getMoisture();

        // Điều khiển máy bơm
        if (smart_Garden.SoilMoisture_Value < 30) {
            Pump_Control(3);  // Mức cao
        } else if (smart_Garden.SoilMoisture_Value < 50) {
            Pump_Control(2);  // Mức trung bình
        } else if (smart_Garden.SoilMoisture_Value < 80) {
            Pump_Control(1);  // Mức thấp
        } else {
            Pump_Control(0);  // Tắt bơm
        }
    }

    // 3. Đo cường độ ánh sáng (3 giây/lần)
    if (currentMillis - lastLightIntensityCheck >= lightIntensityInterval) {
        lastLightIntensityCheck = currentMillis;

        // Đọc dữ liệu từ cảm biến
        smart_Garden.Light_Value = getLDRvalue();

    }
    
    // Cập nhật Serial Monitor mỗi 3 giây
    if (currentMillis - lastMonitorUpdate >= monitorUpdateInterval) {
        lastMonitorUpdate = currentMillis;

        if (displayState == 0) {
            // Hiển thị nhiệt độ (dòng 1) và độ ẩm không khí (dòng 2)
            lcd.setCursor(0, 0);
            lcd.print("Temp: ");
            lcd.print(smart_Garden.Temperature_Value);
            lcd.print(" C  ");

            lcd.setCursor(0, 1);
            lcd.print("Humidity: ");
            lcd.print(smart_Garden.Humidity_Value);
            lcd.print(" %");

            Serial.println("Hiển thị: Nhiệt độ & Độ ẩm không khí");
        } else if (displayState == 1) {
            // Hiển thị độ ẩm đất (dòng 1) và cường độ ánh sáng (dòng 2)
            lcd.setCursor(0, 0);
            lcd.print("Mois: ");
            lcd.print(smart_Garden.SoilMoisture_Value);
            lcd.print(" %                ");

            lcd.setCursor(0, 1);
            lcd.print("Light: ");
            lcd.print(smart_Garden.Light_Value);
            lcd.print("                          ");

            Serial.println("Hiển thị: Độ ẩm đất & Cường độ ánh sáng");
        }

        // Chuyển đổi trạng thái hiển thị
        displayState = 1 - displayState; // Đổi giữa 0 và 1
    }
}
//-------------------------------------------------------END-----------------------------------------------------