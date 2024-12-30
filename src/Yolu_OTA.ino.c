#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// WiFi Credentials
const char* ssid = "wifi_ssid";        // Replace with your WiFi SSID
const char* password = "12345678";    // Replace with your WiFi Password

// Pin Definitions
#define SDA_PIN 21
#define SCL_PIN 22
#define PWMA 13
#define AIN2 14
#define AIN1 26
#define STBY 32
#define BIN1 25
#define BIN2 27
#define PWMB 19
#define BUZZER_PIN 15

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Motor & Sensor Settings
#define HIGH_SPEED 150
#define TURN_SPEED 100
#define PWM_FREQUENCY 18000
#define PWM_RESOLUTION 8
#define DISTANCE_THRESHOLD 1000       // Distance to start obstacle avoidance
#define TURN_DISTANCE_THRESHOLD 600  // Distance to check for obstacles during turns

// Motor Offsets
const int offsetA = -1;
const int offsetB = 1;

// Global Variables
int speed = HIGH_SPEED;
unsigned int measurement = 0;
bool obstacleDetected = false;
bool otaActive = false;
bool isAvoidingObstacle = false;
int lastTurnDirection = 1; // 1 for right, -1 for left

// Initialize Motor, Display, and Sensor
Motor motorLeft(AIN1, AIN2, PWMA, offsetA, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 3);
Motor motorRight(BIN1, BIN2, PWMB, offsetB, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
VL53L0X sensor;

// Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

// Semaphore for motor synchronization
SemaphoreHandle_t motorMutex;

// Function to stop all motors
void stopMotors() {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    motorLeft.brake();
    motorRight.brake();
    xSemaphoreGive(motorMutex);
}

// Function to play success tone
void playSuccessTone() {
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);
}

// Function to play obstacle tone
void playObstacleTone() {
    tone(BUZZER_PIN, 1500, 50);
    delay(100);
}

// Function to display text on OLED
void OLED_DisplayText(const String &text, int size = 2, int x = 0, int y = 15) {
    display.clearDisplay();
    display.setTextSize(size);
    display.setTextColor(WHITE);
    display.setCursor(x, y);
    display.println(text);
    display.display();
}

// Function to update OTA progress on the OLED
void OTA_UpdateProgress(int progress) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Updating...");
    display.drawRect(0, 20, 128, 10, WHITE); // Progress bar outline
    display.fillRect(1, 21, progress, 8, WHITE); // Fill progress
    display.display();
}

// Function to move the robot straight
void moveStraight(int speed) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    motorLeft.drive(speed);
    motorRight.drive(speed);
    xSemaphoreGive(motorMutex);
}

// Function to turn the robot at a specified angle
void turnAtAngle(int speed, int angle, bool right) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    int turnTime = map(angle, 0, 180, 0, 1000);
    if (right) {
        motorLeft.drive(speed);
        motorRight.drive(-speed);
    } else {
        motorLeft.drive(-speed);
        motorRight.drive(speed);
    }
    delay(turnTime);
    stopMotors();
    xSemaphoreGive(motorMutex);
}

// Task to read sensors
void sensorTask(void *parameter) {
    for (;;) {
        if (!otaActive) {
            measurement = sensor.readRangeSingleMillimeters();
            if (sensor.timeoutOccurred()) {
                Serial.println("Sensor timeout!");
                OLED_DisplayText("Sensor timeout!", 1);
                continue;
            }

            if (measurement < DISTANCE_THRESHOLD) {
                playObstacleTone();
                obstacleDetected = true;
            } else {
                obstacleDetected = false;
            }

            OLED_DisplayText("Dist: " + String(measurement), 1);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Task to handle motor control
void motorTask(void *parameter) {
    for (;;) {
        if (!otaActive) {
            if (obstacleDetected) {
                stopMotors();
                delay(300);

                // Alternate turn direction
                bool turnRight = (lastTurnDirection == 1) ? false : true;
                lastTurnDirection = turnRight ? 1 : -1;

                turnAtAngle(TURN_SPEED, 90, turnRight);
                isAvoidingObstacle = true;
            }

            if (isAvoidingObstacle) {
                moveStraight(speed);
                isAvoidingObstacle = false;
            } else {
                moveStraight(speed);
            }
        } else {
            stopMotors();
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// OTA setup
void setupOTA() {
    ArduinoOTA.onStart([]() {
        otaActive = true;
        stopMotors();
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Sketch" : "Filesystem";
        Serial.println("Start updating " + type);
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int percent = (progress / (total / 100));
        OTA_UpdateProgress(percent);
        Serial.printf("Progress: %u%%\r", percent);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
        OLED_DisplayText("Update Done", 1);
        delay(2000);
        ESP.restart();
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
}

// Setup WiFi connection
void setupWiFi() {
    OLED_DisplayText("Connecting...", 1);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected");
    OLED_DisplayText("WiFi Connected!", 1);
    delay(2000);
}

// Setup function
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!sensor.init()) {
        OLED_DisplayText("Sensor Error", 1);
        delay(5000);
        ESP.restart();
    }
    sensor.setTimeout(500);
    sensor.setMeasurementTimingBudget(200001);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        while (1);
    }

    motorMutex = xSemaphoreCreateMutex();
    setupWiFi();
    setupOTA();
    playSuccessTone();

    xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, &sensorTaskHandle);
    xTaskCreate(motorTask, "Motor Task", 2048, NULL, 1, &motorTaskHandle);
}

// Main loop
void loop() {
    ArduinoOTA.handle();
}
