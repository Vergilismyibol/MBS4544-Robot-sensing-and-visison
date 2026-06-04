#include <Arduino.h>
#include "Wire.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "math.h"

const int IMU_ADDR = 0x68; //ADO=5V -> 0x69

// Declare variables.
// NOTE: theta = pitch, phi = roll, psi = yaw
int16_t accel_X, accel_Y, accel_Z, temp_raw, gyro_X, gyro_Y, gyro_Z;
double aX, aY, aZ, theta_acc, phi_acc; //accel values and theta and phi calculated from accelerometer
double theta_acc_previous = 0, theta_acc_LP, phi_acc_previous = 0, phi_acc_LP;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x68, 0x25,0xdd,0xcc,0x2f,0xf8};

// Structure example to send data
// Must match the receiver structure

typedef struct struct_message {
  double a; // theta (pitch)
  double b; // phi (roll)
} struct_message;

// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

double pitch_offset = 0;
double roll_offset = 0;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

// 检测MPU9250连接
bool checkMPUConnection() {
  Wire.beginTransmission(IMU_ADDR);
  byte error = Wire.endTransmission();
  return (error == 0);
}

// 初始化MPU9250
bool initializeMPU9250() {
  Serial.println("Initializing MPU9250...");
  
  // 唤醒MPU9250
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // 唤醒
  if (Wire.endTransmission() != 0) {
    Serial.println("Failed to wake up MPU9250");
    return false;
  }
  delay(100);
  
  // 配置加速度计 ±2g
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00); 
  if (Wire.endTransmission() != 0) {
    Serial.println("Failed to configure accelerometer");
    return false;
  }
  
  // 配置陀螺仪 ±250°/s
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00); 
  if (Wire.endTransmission() != 0) {
    Serial.println("Failed to configure gyroscope");
    return false;
  }
  
  Serial.println("MPU9250 initialized successfully");
  return true;
}

void setup() {
  Serial.begin(115200);
  
  // 初始化I2C - 增加频率和超时设置
  Wire.begin(21, 22); // 明确指定SDA=21, SCL=22
  Wire.setClock(100000); // 100kHz
  Wire.setTimeout(1000); // 1秒超时
  
  Serial.println("=== MPU9250 IMU Controller ===");
  
  // 检查MPU9250连接
  if (!checkMPUConnection()) {
    Serial.println("ERROR: MPU9250 not found! Please check:");
    Serial.println("1. VCC to 3.3V");
    Serial.println("2. GND to GND");
    Serial.println("3. SDA to GPIO 21"); 
    Serial.println("4. SCL to GPIO 22");
    Serial.println("5. AD0 to GND (for address 0x68)");
    while(1) {
      delay(1000);
      Serial.println("MPU9250 not connected - please check wiring");
    }
  }
  
  Serial.println("MPU9250 detected at address 0x68");
  
  // 初始化MPU9250
  if (!initializeMPU9250()) {
    Serial.println("ERROR: Failed to initialize MPU9250");
    while(1) {
      delay(1000);
      Serial.println("MPU9250 initialization failed");
    }
  }
  
  // WiFi和ESP-NOW设置
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  Serial.print("Controller MAC: ");
  readMacAddress();
  
  // 注册对等设备
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("IMU Controller Ready - Tilt to control the car!");
  Serial.println("==============================================");
}

void loop() {
  // 读取IMU数据
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B); // 从ACCEL_XOUT_H开始
  byte error = Wire.endTransmission(false);
  
  if (error != 0) {
    Serial.println("I2C communication error");
    delay(200);
    return;
  }
  
  // 请求14字节数据
  int bytesReceived = Wire.requestFrom(IMU_ADDR, 14);
  
  if (bytesReceived != 14) {
    Serial.println("Incomplete data received: " + String(bytesReceived) + " bytes");
    delay(200);
    return;
  }
  
  // 读取传感器数据
  accel_X = Wire.read() << 8 | Wire.read();
  accel_Y = Wire.read() << 8 | Wire.read();
  accel_Z = Wire.read() << 8 | Wire.read();
  temp_raw = Wire.read() << 8 | Wire.read();
  gyro_X = Wire.read() << 8 | Wire.read();
  gyro_Y = Wire.read() << 8 | Wire.read();
  gyro_Z = Wire.read() << 8 | Wire.read();
  
  // 数据有效性检查
  if (accel_X == 0 && accel_Y == 0 && accel_Z == 0) {
    Serial.println("Warning: All zero data");
    delay(100);
    return;
  }
  
  // 转换为g值 (±2g范围)
  aX = accel_X / 16384.0;
  aY = accel_Y / 16384.0; 
  aZ = accel_Z / 16384.0;
  
  // 计算俯仰和横滚角
  theta_acc = atan2(aX, sqrt(aY*aY + aZ*aZ)) * (180.0 / PI);
  phi_acc = atan2(aY, sqrt(aX*aX + aZ*aZ)) * (180.0 / PI);
  
  // 低通滤波
  theta_acc_LP = 0.7 * theta_acc_previous + 0.3 * theta_acc;
  phi_acc_LP = 0.7 * phi_acc_previous + 0.3 * phi_acc;
  
  // 应用偏移校准
  theta_acc_LP -= pitch_offset;
  phi_acc_LP -= roll_offset;
  
  theta_acc_previous = theta_acc_LP;
  phi_acc_previous = phi_acc_LP;
  
  // 实时显示角度信息
  Serial.print("Pitch: ");
  Serial.print(theta_acc_LP, 1);
  Serial.print("°, Roll: ");
  Serial.print(phi_acc_LP, 1);
  Serial.print("° -> ");
  
  // 方向指示
  if (abs(theta_acc_LP) < 10 && abs(phi_acc_LP) < 10) {
    Serial.println("STOP");
  } else if (theta_acc_LP > 10 && abs(phi_acc_LP) < 10) {
    Serial.println("FORWARD");
  } else if (theta_acc_LP < -10 && abs(phi_acc_LP) < 10) {
    Serial.println("BACKWARD");
  } else if (abs(theta_acc_LP) < 10 && phi_acc_LP > 10) {
    Serial.println("RIGHT");
  } else if (abs(theta_acc_LP) < 10 && phi_acc_LP < -10) {
    Serial.println("LEFT");
  } else if (theta_acc_LP > 10 && phi_acc_LP > 10) {
    Serial.println("FORWARD-RIGHT");
  } else if (theta_acc_LP > 10 && phi_acc_LP < -10) {
    Serial.println("FORWARD-LEFT");
  } else if (theta_acc_LP < -10 && phi_acc_LP > 10) {
    Serial.println("BACKWARD-RIGHT");
  } else if (theta_acc_LP < -10 && phi_acc_LP < -10) {
    Serial.println("BACKWARD-LEFT");
  }
  
  // 设置发送数据
  myData.a = theta_acc_LP;
  myData.b = phi_acc_LP;
  
  // 通过ESP-NOW发送
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result != ESP_OK) {
    Serial.println("Error sending data");
  }
  
  delay(100); // 控制循环频率
}