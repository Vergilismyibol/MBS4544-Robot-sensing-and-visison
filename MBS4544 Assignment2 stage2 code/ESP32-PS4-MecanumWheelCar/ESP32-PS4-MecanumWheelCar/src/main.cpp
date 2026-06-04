/* Controlling ESP32 Mecanum Wheel car 
//  * created by: Winston Yeung
//  * revision date: 22 Oct 2022
// */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <EEPROM.h>

// Motor direction definitions
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define ROTATE_LEFT 9
#define ROTATE_RIGHT 10
#define STOP 0

#define FRONT_RIGHT_MOTOR 0
#define FRONT_LEFT_MOTOR 1  
#define BACK_LEFT_MOTOR 2
#define BACK_RIGHT_MOTOR 3

#define MAX_MOTOR_SPEED 50 // about 50% duty cycle

unsigned long lastTimeStamp = 0;

// 全局变量定义 - 放在所有函数之前
bool newDataReceived = false;
bool motorsInitialized = false;
static bool testCompleted = false; 

// Motor pins - 确认这些引脚与实际接线一致
//FRONT RIGHT MOTOR
int enableFrontRightMotor = 14; 
int FrontRightMotorPin1 = 26;
int FrontRightMotorPin2 = 27;
//BACK RIGHT MOTOR
int enableBackRightMotor=22; 
int BackRightMotorPin1=16;
int BackRightMotorPin2=17;
//FRONT LEFT MOTOR
int enableFrontLeftMotor = 32;
int FrontLeftMotorPin1 = 33;
int FrontLeftMotorPin2 = 25;
//BACK LEFT MOTOR
int enableBackLeftMotor = 23;
int BackLeftMotorPin1 = 18;
int BackLeftMotorPin2 = 19;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8; // 8-bit
const int PWMSpeedChannel_1 = 1; // 0-15
const int PWMSpeedChannel_2 = 2; // 0-15
const int PWMSpeedChannel_3 = 3; // 0-15
const int PWMSpeedChannel_4 = 4; // 0-15

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    double a; // pitch
    double b; // roll
} struct_message;

// Create a struct_message called myData
struct_message myData;


// 麦轮速度数组 [FR, FL, BL, BR]
int wheelSpeeds[4] = {0, 0, 0, 0};

void rotateMotor(int motorNumber, int motorSpeed) {
    if (!motorsInitialized) return;
    
    motorSpeed = constrain(motorSpeed, -255, 255);
    
    switch(motorNumber) {
        case 0: // FRONT_RIGHT_MOTOR
            digitalWrite(FrontRightMotorPin1, motorSpeed > 0 ? HIGH : LOW); 
            digitalWrite(FrontRightMotorPin2, motorSpeed > 0 ? LOW : HIGH); 
            ledcWrite(PWMSpeedChannel_4, abs(motorSpeed));      
            break;
        case 1: // FRONT_LEFT_MOTOR - 修正左电机方向
            
            digitalWrite(FrontLeftMotorPin1, motorSpeed > 0 ? HIGH : LOW);  
            digitalWrite(FrontLeftMotorPin2, motorSpeed > 0 ? LOW : HIGH);  
            ledcWrite(PWMSpeedChannel_3, abs(motorSpeed));
            break;
        case 2: // BACK_LEFT_MOTOR - 修正左电机方向
            
            digitalWrite(BackLeftMotorPin1, motorSpeed > 0 ? HIGH : LOW);   
            digitalWrite(BackLeftMotorPin2, motorSpeed > 0 ? LOW : HIGH);   
            ledcWrite(PWMSpeedChannel_2, abs(motorSpeed));
            break;
        case 3: // BACK_RIGHT_MOTOR
            digitalWrite(BackRightMotorPin1, motorSpeed > 0 ? HIGH : LOW);
            digitalWrite(BackRightMotorPin2, motorSpeed > 0 ? LOW : HIGH);
            ledcWrite(PWMSpeedChannel_1, abs(motorSpeed));
            break;
    }
}


// 麦轮运动学算法
void mecanumMove(double x, double y, double rotation) {
    // 麦轮运动学公式
    wheelSpeeds[0] = x + y + rotation;  // FR
    wheelSpeeds[1] = -x + y - rotation; // FL  
    wheelSpeeds[2] = -x + y + rotation; // BL
    wheelSpeeds[3] = x + y - rotation;  // BR
    
    // 归一化速度到MAX_MOTOR_SPEED范围内
    int maxSpeed = 0;
    for (int i = 0; i < 4; i++) {
        if (abs(wheelSpeeds[i]) > maxSpeed) {
            maxSpeed = abs(wheelSpeeds[i]);
        }
    }
    
    if (maxSpeed > MAX_MOTOR_SPEED) {
        for (int i = 0; i < 4; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] * MAX_MOTOR_SPEED) / maxSpeed;
        }
    } else {
        for (int i = 0; i < 4; i++) {
            wheelSpeeds[i] = map(wheelSpeeds[i], -100, 100, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        }
    }
    
    // 应用速度到电机
    for (int i = 0; i < 4; i++) {
        rotateMotor(i, wheelSpeeds[i]);
    }
}

// 简化的移动函数
void moveCar(int direction) {
    switch(direction) {
        case FORWARD:
            mecanumMove(0, 100, 0);
            Serial.println("FORWARD - All wheels forward");
            break;
        case BACKWARD:
            mecanumMove(0, -100, 0);
            Serial.println("BACKWARD - All wheels backward");
            break;
        case LEFT:
            mecanumMove(-100, 0, 0);
            Serial.println("LEFT - Strafe left");
            break;
        case RIGHT:
            mecanumMove(100, 0, 0);
            Serial.println("RIGHT - Strafe right");
            break;
        case FORWARD_LEFT:
            mecanumMove(-70, 70, 0);
            Serial.println("FORWARD_LEFT - Diagonal");
            break;
        case FORWARD_RIGHT:
            mecanumMove(70, 70, 0);
            Serial.println("FORWARD_RIGHT - Diagonal");
            break;
        case BACKWARD_LEFT:
            mecanumMove(-70, -70, 0);
            Serial.println("BACKWARD_LEFT - Diagonal");
            break;
        case BACKWARD_RIGHT:
            mecanumMove(70, -70, 0);
            Serial.println("BACKWARD_RIGHT - Diagonal");
            break;
        case ROTATE_LEFT:
            mecanumMove(0, 0, -100);
            Serial.println("ROTATE_LEFT - Counterclockwise");
            break;
        case ROTATE_RIGHT:
            mecanumMove(0, 0, 100);
            Serial.println("ROTATE_RIGHT - Clockwise");
            break;
        case STOP:
            mecanumMove(0, 0, 0);
            Serial.println("STOP - All wheels stopped");
            break;
    }
    
    // 调试输出每个轮子的速度
    Serial.print("Wheel Speeds - FR:");
    Serial.print(wheelSpeeds[0]);
    Serial.print(" FL:");
    Serial.print(wheelSpeeds[1]);
    Serial.print(" BL:");
    Serial.print(wheelSpeeds[2]);
    Serial.print(" BR:");
    Serial.println(wheelSpeeds[3]);
}

// callback function that will be executed when data is received (ESP-NOW接收回调)
// ESP-NOW接收回调
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    newDataReceived = true;
    
    Serial.print("IMU Data Received - Pitch: ");
    Serial.print(myData.a, 1);
    Serial.print("°, Roll: ");
    Serial.print(myData.b, 1);
    Serial.println("°");
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

// 改进的控制逻辑
void controlCar() {
    const int deadZone = 15; // 死区范围
    const int maxTilt = 45;  // 最大倾斜角度
    
    double pitch = myData.a;
    double roll = myData.b;

    pitch = constrain(pitch, -maxTilt, maxTilt);
    roll = constrain(roll, -maxTilt, maxTilt);

    Serial.print("Control Decision - Pitch: "); 
    Serial.print(pitch, 1);
    Serial.print("°, Roll: "); 
    Serial.print(roll, 1);
    Serial.print("° -> ");
    
    // 控制逻辑
    bool pitchActive = abs(pitch) > deadZone;
    bool rollActive = abs(roll) > deadZone;

    if (!pitchActive && !rollActive) {
        moveCar(STOP);
        Serial.println("STOP - Level");
    }
    else if (pitchActive && !rollActive) {
        // 只有俯仰
        if (pitch > 0) {
            moveCar(FORWARD);
            Serial.println("FORWARD");
        } else {
            moveCar(BACKWARD);
            Serial.println("BACKWARD");
        }
    }
    else if (!pitchActive && rollActive) {
        // 只有横滚
        if (roll > 0) {
            moveCar(RIGHT);
            Serial.println("RIGHT");
        } else {
            moveCar(LEFT);
            Serial.println("LEFT");
        }
    }
    else {
        // 组合方向
        if (pitch > 0 && roll > 0) {
            moveCar(FORWARD_RIGHT);
            Serial.println("FORWARD_RIGHT");
        }
        else if (pitch > 0 && roll < 0) {
            moveCar(FORWARD_LEFT);
            Serial.println("FORWARD_LEFT");
        }
        else if (pitch < 0 && roll > 0) {
            moveCar(BACKWARD_RIGHT);
            Serial.println("BACKWARD_RIGHT");
        }
        else if (pitch < 0 && roll < 0) {
            moveCar(BACKWARD_LEFT);
            Serial.println("BACKWARD_LEFT");
        }
        else {
            moveCar(STOP);
            Serial.println("STOP (fallback)");
        }
    }
}

// void onConnect() {
//   Serial.println("Connected!");
// }

void onDisConnect()
{
  rotateMotor(BACK_RIGHT_MOTOR, 0);
  rotateMotor(BACK_LEFT_MOTOR, 0);
  rotateMotor(FRONT_RIGHT_MOTOR, 0);
  rotateMotor(FRONT_LEFT_MOTOR, 0);
}

void setUpPinModes() {
    Serial.println("Initializing motor pins...");
    
    // 设置引脚模式
    pinMode(enableFrontRightMotor, OUTPUT);
    pinMode(FrontRightMotorPin1, OUTPUT);
    pinMode(FrontRightMotorPin2, OUTPUT);
    
    pinMode(enableFrontLeftMotor, OUTPUT);
    pinMode(FrontLeftMotorPin1, OUTPUT);
    pinMode(FrontLeftMotorPin2, OUTPUT);

    pinMode(enableBackRightMotor, OUTPUT);
    pinMode(BackRightMotorPin1, OUTPUT);
    pinMode(BackRightMotorPin2, OUTPUT);
    
    pinMode(enableBackLeftMotor, OUTPUT);
    pinMode(BackLeftMotorPin1, OUTPUT);
    pinMode(BackLeftMotorPin2, OUTPUT);

    // PWM设置
    ledcSetup(PWMSpeedChannel_1, PWMFreq, PWMResolution);
    ledcSetup(PWMSpeedChannel_2, PWMFreq, PWMResolution);
    ledcSetup(PWMSpeedChannel_3, PWMFreq, PWMResolution);
    ledcSetup(PWMSpeedChannel_4, PWMFreq, PWMResolution);
    
    ledcAttachPin(enableBackRightMotor, PWMSpeedChannel_1);
    ledcAttachPin(enableBackLeftMotor, PWMSpeedChannel_2);
    ledcAttachPin(enableFrontRightMotor, PWMSpeedChannel_3);
    ledcAttachPin(enableFrontLeftMotor, PWMSpeedChannel_4); 
    
    // 确保所有电机停止
    digitalWrite(FrontRightMotorPin1, LOW);
    digitalWrite(FrontRightMotorPin2, LOW);
    digitalWrite(FrontLeftMotorPin1, LOW);
    digitalWrite(FrontLeftMotorPin2, LOW);
    digitalWrite(BackRightMotorPin1, LOW);
    digitalWrite(BackRightMotorPin2, LOW);
    digitalWrite(BackLeftMotorPin1, LOW);
    digitalWrite(BackLeftMotorPin2, LOW);
    
    ledcWrite(PWMSpeedChannel_1, 0);
    ledcWrite(PWMSpeedChannel_2, 0);
    ledcWrite(PWMSpeedChannel_3, 0);
    ledcWrite(PWMSpeedChannel_4, 0);
    
    motorsInitialized = true;
    Serial.println("Motor pins initialized and stopped");
}

// 测试函数 - 用于验证每个电机
void testAllMotors() {
    bool testCompleted = EEPROM.read(0);
    // 检查是否已经测试过
    if (testCompleted) {
        Serial.println("Motor test already completed, skipping...");
        return;
    }
    
    Serial.println("=== Motor Test Starting ===");
    
    // 测试各个电机...
    // 测试前右电机
    Serial.println("Testing Front Right Motor Forward...");
    rotateMotor(FRONT_RIGHT_MOTOR, 80);
    delay(2000);
    rotateMotor(FRONT_RIGHT_MOTOR, 0);
    delay(500);
    
    // 测试前左电机
    Serial.println("Testing Front Left Motor Forward...");
    rotateMotor(FRONT_LEFT_MOTOR, 80);
    delay(2000);
    rotateMotor(FRONT_LEFT_MOTOR, 0);
    delay(500);
    
    // 测试后左电机
    Serial.println("Testing Back Left Motor Forward...");
    rotateMotor(BACK_LEFT_MOTOR, 80);
    delay(2000);
    rotateMotor(BACK_LEFT_MOTOR, 0);
    delay(500);
    
    // 测试后右电机
    Serial.println("Testing Back Right Motor Forward...");
    rotateMotor(BACK_RIGHT_MOTOR, 80);
    delay(2000);
    rotateMotor(BACK_RIGHT_MOTOR, 0);
    delay(500);
    
    testCompleted = true;
    Serial.println("=== Motor Test Complete ===");
    
    // 保存测试状态到EEPROM
    EEPROM.write(0, true);
    EEPROM.commit();
    Serial.println("=== Motor Test Complete ===");
}


void setup() {
    Serial.begin(115200);
    Serial.println("=== Mecanum Wheel Car Starting ===");
    Serial.println("Initializing system...");
    
    // 初始化EEPROM
    EEPROM.begin(1);

    // 初始化电机引脚
    setUpPinModes();  

    // 设置为WiFi站点模式
    WiFi.mode(WIFI_STA);

    // 初始化ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        delay(1000);
        ESP.restart(); // 重启尝试恢复
        return;
    }

    // 注册接收回调
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("=== System Ready ===");
    Serial.println("Waiting for IMU controller...");
    delay(100);
    Serial.println("Mecanum Wheel Car Ready - Waiting for IMU data...");
    Serial.println("Expected IMU data range: Pitch ±90°, Roll ±90°");
    Serial.println("Dead zone: ±10°");

    // 在setup中只运行一次电机测试
    // testAllMotors();
}

// loop函数
void loop()
{
    if (newDataReceived) {
        controlCar();
        newDataReceived = false;
    } else {
        // 如果没有新数据，定期输出状态
        if (millis() - lastTimeStamp > 3000) {
            Serial.println("Waiting for IMU data... No data received yet.");
            lastTimeStamp = millis();
        }
    }
    delay(50);
    
}
  
