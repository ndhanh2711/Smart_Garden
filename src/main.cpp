//--------------------------------Khai báo thư viện các ngoại vi-------------------------------
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>

//----------------------------Khởi tạo các chân kết nối với ngoại vi---------------------------

//Các thành phần của DHT22_Cảm biến nhiệt độ, độ ẩm không khí--------
#define DHTPIN 32     
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);

//Các thành phần khởi tạo điều khiển  máy bơm -----------------------
#define PUMP_PIN 4                // Chân kết nối máy bơm
#define PWM_CHANNEL 0             // Kênh PWM cho máy bơm
#define PWM_FREQ 5000             // Tần số PWM (5 kHz)
#define PWM_RESOLUTION 8          // Độ phân giải 8-bit (0-255)

// Chân kết nối quang trở--------------------------------------------
#define LDR_PIN 34   // Chân analog đọc dữ liệu từ quang trở

//Chân bật đèn sưởi--------------------------------------------------
#define SUOI_GPIO_PIN 18

//Chân bật phun sương------------------------------------------------
#define PS_GPIO_PIN 19

//Khởi tạo LCD-------------------------------------------------------


//---------------------------------Cấu trúc lưu trữ các giá trị đầu vào---------------------------
struct Smart_Garden{
    float Temperature_Value;
    int Light_Value;
    int Humidity_Value;
    int SoilMoisture_Value;
};
Smart_Garden smart_Garden;

// //------------------------Cấu trúc thể hiện các giá trị đầu ra của hệ thống--------------------
// struct DeviceStatus {
//     int pumpLevel;        // Mức hoạt động của bơm (0-5)
//     bool misting;         // Trạng thái phun sương
//     bool heating;         // Trạng thái sưởi
// };
// DeviceStatus deviceStatus;

//-------------------------------------Hàm chức năng của DHT22---------------------------------

// Hàm đọc dữ liệu từ cảm biến DHT và in ra Serial
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

    // In dữ liệu ra Serial Monitor
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C"));
    smart_Garden.Temperature_Value = t;
    smart_Garden.Humidity_Value = h;
}

//-----------------------------------Hàm chức năng của máy bơm-------------------------------
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
    level = constrain(level, 0, 3);

    // Chuyển đổi mức từ 0-5 sang giá trị duty cycle 0-255
    int dutyCycle = map(level, 0, 3, 0, 255);

    // Gửi giá trị duty cycle đến PWM
    ledcWrite(PWM_CHANNEL, dutyCycle);

    // In thông tin mức bơm ra Serial Monitor (tùy chọn)
    Serial.print("Pump Level: ");
    Serial.print(level);
    Serial.print(" -> Duty Cycle: ");
    Serial.println(dutyCycle);
}
//---------------------------------Hàm đọc giá trị của quang trở-----------------------------



//----------------------------------Hàm lấy các giá trị đầu vào------------------------------

// Hàm giả lập trả về độ ẩm
int getHumidity() {
    DHTdata_Read();
    //return smart_Garden.Humidity_Value;  // Giả định độ ẩm là 45%
    return 20;
}
int getLDRvalue(){
    //return analogRead(LDR_PIN);
    int check_LDR = analogRead(LDR_PIN);
    //return check_LDR;
    return 5;
}
float getTemperature() {
    // Giá trị giả định (có thể thay đổi khi cần)
    //return dht.readTemperature();
    DHTdata_Read();
    //return smart_Garden.Temperature_Value;  // Giả định nhiệt độ là 8.5°C
    return 5;
}
int getMoisture(){
    return 0;
}
//------------------------------------Hàm động đèn sưởi-------------------------------------


// Hàm kiểm tra nhiệt độ và bật/tắt GPIO 18
void control_SUOI_ON() {
    float temperature = getTemperature();  // Gọi hàm lấy nhiệt độ giả định

    // Hiển thị nhiệt độ lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);
    digitalWrite(SUOI_GPIO_PIN, HIGH);  // Bật GPIO 18
}
void control_SUOI_OFF() {
    float temperature = getTemperature();  // Gọi hàm lấy nhiệt độ giả định

    // Hiển thị nhiệt độ lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);
    digitalWrite(SUOI_GPIO_PIN, LOW);  // Bật GPIO 18
}

//-----------------------------------Hàm khởi động phun sương--------------------------------

// Hàm kiểm tra điều kiện và bật/tắt GPIO 19
void control_PS_ON() {
    float temperature = getTemperature();  // Lấy nhiệt độ giả định
    float humidity = getHumidity();        // Lấy độ ẩm giả định

    // Hiển thị nhiệt độ và độ ẩm lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);
    Serial.print("Độ ẩm hiện tại: ");
    Serial.println(humidity);

    digitalWrite(PS_GPIO_PIN, HIGH);  // Bật GPIO 19
    Serial.println("GPIO 19 được bật.");
}
void control_PS_OFF() {
    float temperature = getTemperature();  // Lấy nhiệt độ giả định
    float humidity = getHumidity();        // Lấy độ ẩm giả định

    // Hiển thị nhiệt độ và độ ẩm lên Serial
    Serial.print("Nhiệt độ hiện tại: ");
    Serial.println(temperature);
    Serial.print("Độ ẩm hiện tại: ");
    Serial.println(humidity);

    digitalWrite(PS_GPIO_PIN, LOW);  // Bật GPIO 19
    Serial.println("GPIO 19 được tắt.");
}
//-----------------------------------Hàm khởi tạo các ngoại vi---------------------------------

void setup() {
Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  
  // Khởi tạo cảm biến nhiệt độ và độ ẩm
  dht.begin();
  
  // Khởi tạo máy bơm
  Pump_Init();
  
  // Khởi tạo các chân điều khiển thiết bị
  pinMode(SUOI_GPIO_PIN, OUTPUT);
  pinMode(PS_GPIO_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  
  // Bắt đầu in thông báo khởi động
  Serial.println("Khởi động hệ thống...");
}
//------------------------------------Các ngưỡng điều kiện----------------------------------

const float TEMP_MIN = 20.0;     // Ngưỡng bật sưởi ấm (độ C)
const float TEMP_MAX = 30.0;     // Ngưỡng tắt sưởi ấm
const float HUMIDITY_MIN = 40.0; // Ngưỡng bật phun sương
const float HUMIDITY_MAX = 70.0; // Ngưỡng tắt phun sương
const float SOIL_MIN1 = 60.0;    // Ngưỡng tưới mức 1
const float SOIL_MIN2 = 40.0;    // Ngưỡng tưới mức 2
const float SOIL_MIN3 = 20.0;    // Ngưỡng tưới mức 3
const int LIGHT_MIN = 20;        // Ngưỡng ánh sáng bật sưởi ấm
//---------------------------------------Thời gian-------------------------------------------
// Biến lưu giá trị thời gian hiện tại cho từng tác vụ
unsigned long lastTempHumCheck = 0; // Thời gian lần cuối đo nhiệt độ và độ ẩm
unsigned long lastSoilMoistureCheck = 0; // Thời gian lần cuối đo độ ẩm đất
unsigned long lastLightIntensityCheck = 0; // Thời gian lần cuối đo cường độ ánh sáng

// Khoảng thời gian đo cho từng cảm biến (ms)
const unsigned long tempHumInterval = 5000;  // 5 giây
const unsigned long soilMoistureInterval = 7000;  // 7 giây
const unsigned long lightIntensityInterval = 3000;  // 3 giây

// Biến thời gian cho việc cập nhật Serial Monitor
unsigned long lastMonitorUpdate = 0;
const unsigned long monitorUpdateInterval = 3000; // 3 giây
//-------------------------------------Hàm chạy chương trình---------------------------------
void loop() {

  // // Đọc dữ liệu từ cảm biến
  // smart_Garden.Temperature_Value = getTemperature();
  // smart_Garden.Humidity_Value = getHumidity();
  // smart_Garden.Light_Value = getLDRvalue();
  // smart_Garden.SoilMoisture_Value = getMoisture();

  // // Debug giá trị
  // Serial.print("Temperature: "); Serial.println(smart_Garden.Temperature_Value);
  // Serial.print("Humidity: "); Serial.println(smart_Garden.Humidity_Value);
  // Serial.print("Soil Moisture: "); Serial.println(smart_Garden.SoilMoisture_Value);
  // Serial.print("Light Intensity: "); Serial.println(smart_Garden.Light_Value);

  // // Điều kiện sưởi ấm 
  // if (smart_Garden.Temperature_Value < TEMP_MIN && smart_Garden.Light_Value < LIGHT_MIN) {
  //   control_SUOI_ON(); // Bật sưởi
  //   Serial.println("Heater ON");
  // } else if (smart_Garden.Temperature_Value >= TEMP_MAX || smart_Garden.Light_Value >= LIGHT_MIN) {
  //   control_SUOI_OFF();  // Tắt sưởi
  //   Serial.println("Heater OFF");
  // }
  
  // // Hàm điều khiển phun sương 
  // if (smart_Garden.Humidity_Value < HUMIDITY_MIN) {
  //   control_PS_ON(); // Bật phun sương
  //   Serial.println("Sprayer ON");
  // } else if (smart_Garden.Humidity_Value >= HUMIDITY_MAX) {
  //   control_PS_OFF();  // Tắt phun sương
  //   Serial.println("Sprayer OFF");
  // }

  //  // Hàm điều khiển bơm nước
  // if (smart_Garden.SoilMoisture_Value < SOIL_MIN3) {
  //   Pump_Control(3);
  //   Serial.println("Pump Level 3 ON");
  // } else if (smart_Garden.SoilMoisture_Value < SOIL_MIN2) {
  //   Pump_Control(2);
  //   Serial.println("Pump Level 2 ON");
  // } else if (smart_Garden.SoilMoisture_Value < SOIL_MIN1) {
  //   Pump_Control(1);
  //   Serial.println("Pump Level 1 ON");
  // } else {
  //   Pump_Control(0);
  //   Serial.println("Pump OFF");
  // }
  // delay(2000);
    unsigned long currentMillis = millis();

    // 1. Đo nhiệt độ và độ ẩm không khí (5 giây/lần)
    if (currentMillis - lastTempHumCheck >= tempHumInterval) {
        lastTempHumCheck = currentMillis;

        // Đọc dữ liệu từ cảm biến
        smart_Garden.Temperature_Value = getTemperature();
        smart_Garden.Humidity_Value = getHumidity();

        // Hiển thị giá trị
        //Serial.print("Temperature: ");
        //Serial.println(smart_Garden.Temperature_Value);
        //Serial.print("Humidity (Air): ");
        //Serial.println(smart_Garden.Humidity_Value);

        // Điều khiển hệ thống sưởi
        if (smart_Garden.Temperature_Value < 10) {
            control_SUOI_ON();
        } else {
            control_SUOI_OFF();
        }

        // Điều khiển phun sương
        if (smart_Garden.Humidity_Value < 40) {
            control_PS_ON();
        } else {
            control_PS_OFF();
        }
    }

    // 2. Đo độ ẩm đất (7 giây/lần)
    if (currentMillis - lastSoilMoistureCheck >= soilMoistureInterval) {
        lastSoilMoistureCheck = currentMillis;

        // Đọc dữ liệu từ cảm biến
        smart_Garden.SoilMoisture_Value = getMoisture();

        // Hiển thị giá trị
        //Serial.print("Soil Moisture: ");
        //Serial.println(smart_Garden.SoilMoisture_Value);

        // Điều khiển máy bơm
        if (smart_Garden.SoilMoisture_Value < 30) {
            Pump_Control(3);  // Mức cao
        } else if (smart_Garden.SoilMoisture_Value < 50) {
            Pump_Control(2);  // Mức trung bình
        } else if (smart_Garden.SoilMoisture_Value < 70) {
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

        // Hiển thị giá trị
        //Serial.print("Light Intensity: ");
        //Serial.println(smart_Garden.Light_Value);

        // Thực hiện xử lý cường độ ánh sáng nếu cần (ví dụ, kích hoạt màn che)
        // Hiện tại chỉ ghi nhận giá trị cường độ ánh sáng
    }
    // Cập nhật Serial Monitor mỗi 3 giây
    if (currentMillis - lastMonitorUpdate >= monitorUpdateInterval) {
        lastMonitorUpdate = currentMillis;

        Serial.println("----- Smart Garden Monitor -----");
        Serial.print("Nhiệt độ: ");
        Serial.println(smart_Garden.Temperature_Value);

        Serial.print("Độ ẩm không khí: ");
        Serial.println(smart_Garden.Humidity_Value);

        Serial.print("Độ ẩm đất: ");
        Serial.println(smart_Garden.SoilMoisture_Value);

        Serial.print("Cường độ ánh sáng: ");
        Serial.println(smart_Garden.Light_Value);

        Serial.println("--------------------------------");
    }
}
