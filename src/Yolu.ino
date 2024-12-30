// Include Libraries
#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

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
#define SCREEN_HEIGHT 64 

// Motor & Sensor Settings
#define HIGH_SPEED 100
#define TURN_SPEED 100
#define PWM_FREQUENCY 18000
#define PWM_RESOLUTION 8
#define DISTANCE_THRESHOLD 500
#define TURN_DISTANCE_THRESHOLD 600

// TCA9548A Multiplexer
#define TCA9548A_ADDR 0x70

// Motor Offsets
const int offsetA = -1;
const int offsetB = 1;

// Global Variables
int speed = HIGH_SPEED;
unsigned int measurement;
bool obstacleDetected = false;
bool nearWall = false;
bool stuckDetected = false;

// MPU6050 Variables
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
unsigned long lastMovementTime = 0;

// Mood States
enum Mood { HAPPY, CURIOUS, BORED, EXCITED };
Mood currentMood = HAPPY;

// Motor, Display, and Sensor Initialization
Motor motorLeft(AIN1, AIN2, PWMA, offsetA, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 3);
Motor motorRight(BIN1, BIN2, PWMB, offsetB, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
VL53L0X sensor;

// Semaphore for motor synchronization
SemaphoreHandle_t motorMutex;

// Function to select the TCA9548A multiplexer channel
void tca9548aSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// Function to play tones
void playSuccessTone() { tone(BUZZER_PIN, 2000, 100); delay(150); tone(BUZZER_PIN, 2500, 100); }
void playObstacleTone() { tone(BUZZER_PIN, 1500, 50); delay(100); }
void playStuckTone() { tone(BUZZER_PIN, 1000, 200); delay(200); tone(BUZZER_PIN, 1200, 200); }
void playHappyMelody() { tone(BUZZER_PIN, 500, 200); delay(250); tone(BUZZER_PIN, 700, 200); delay(250); tone(BUZZER_PIN, 900, 200); delay(250); }
void playCuriousBeeps() { for (int i = 0; i < 3; i++) { tone(BUZZER_PIN, random(1000, 1500), 100); delay(150); } }

// Function to display text on OLED
void OLED_DisplayStatus() {
    display.clearDisplay();
    display.setRotation(2); // Flip screen horizontally

    // Display TOF sensor data in yellow area
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.setTextSize(2); // Make text bigger
    display.println("TOF Distance:");
    display.setCursor(0, 16);
    display.println(String(measurement) + " mm");

    // Display other statuses below
    display.setCursor(0, 32);
    display.print("Obstacle: ");
    display.println(obstacleDetected ? "Yes" : "No");
    display.setCursor(0, 48);
    display.print("Stuck: ");
    display.println(stuckDetected ? "Yes" : "No");
    display.setCursor(0, 64);
    display.print("Mood: ");
    display.println(currentMood == HAPPY ? "Happy" : currentMood == CURIOUS ? "Curious" : currentMood == BORED ? "Bored" : "Excited");

    display.display();
}

void OLED_AnimateText(const String &text, int size = 2) {
    display.clearDisplay();
    display.setRotation(2); // Flip screen horizontally because the OLED is fixed on the tail of the robot upside down
    display.setTextSize(size);
    display.setTextColor(WHITE);
    for (int i = 0; i <= text.length(); i++) {
        display.setCursor(0, 32); // Adjusted to fit larger display
        display.println(text.substring(0, i));
        display.display();
        delay(50); // animation speed
    }
}

// Movement Utility Functions
void moveStraight(int speed) { motorLeft.drive(speed); motorRight.drive(speed); }
void stopMotors() { motorLeft.drive(0); motorRight.drive(0); }
void moveBackward() { motorLeft.drive(-90); motorRight.drive(-90); delay(500); }
void turnRandomDirection() { motorLeft.drive(random(0, 2) ? TURN_SPEED : -TURN_SPEED); motorRight.drive(random(0, 2) ? -TURN_SPEED : TURN_SPEED); delay(1000); stopMotors(); }
void dynamicZigzag() {
    for (int i = 0; i < 5; i++) {
        motorLeft.drive(speed);
        motorRight.drive(speed / random(2, 4));
        delay(random(200, 500));
        motorLeft.drive(speed / random(2, 4));
        motorRight.drive(speed);
        delay(random(200, 500));
    }
    stopMotors();
}
void dramaticSpin() {
    motorLeft.drive(TURN_SPEED * 2);
    motorRight.drive(-TURN_SPEED * 2);
    delay(300);
    stopMotors();
    delay(200);
    motorLeft.drive(-TURN_SPEED * 2);
    motorRight.drive(TURN_SPEED * 2);
    delay(300);
    stopMotors();
}

void spinningBehavior() {
    for (int i = 0; i < 3; i++) {
        motorLeft.drive(TURN_SPEED * 2);
        motorRight.drive(-TURN_SPEED * 2);
        delay(300);
        motorLeft.drive(-TURN_SPEED * 2);
        motorRight.drive(TURN_SPEED * 2);
        delay(300);
    }
    stopMotors();
}

// Behavior Functions
void curiousRoutine() {
    playCuriousBeeps();
    OLED_AnimateText("What's this?");
    moveBackward();
    turnRandomDirection();
}
void excitedRoutine() {
    playHappyMelody();
    OLED_AnimateText("I'm so excited!");
    for (int i = 0; i < 3; i++) {
        dramaticSpin();
        dynamicZigzag();
    }
}
void handleMood() {
    switch (currentMood) {
        case HAPPY:
            excitedRoutine();
            break;
        case CURIOUS:
            curiousRoutine();
            break;
        case BORED:
            OLED_AnimateText("Hmmm..."); // just do nothing
            break;
        case EXCITED:
            dynamicZigzag();
            break;
    }
    currentMood = static_cast<Mood>(random(0, 4)); // Change mood randomly
}

// Tasks
void motorControlTask(void *parameter) {
    unsigned long lastPersonalityTime = millis();
    for (;;) {
        if (obstacleDetected) {
            stopMotors(); // Stop the motors immediately
            playObstacleTone();
            moveBackward(); // Move back slightly
            turnRandomDirection(); // Perform a turn but randomized
            OLED_AnimateText("Obstacle Avoided!");
        } else if (stuckDetected) {
            OLED_AnimateText("I'm stuck!");
            playStuckTone();
            moveBackward();
            turnRandomDirection();
        } else {
            moveStraight(speed);
            if (millis() - lastPersonalityTime > random(5000, 10000)) {
                stopMotors();
                int moodChoice = random(0, 5); // Randomize between all behaviors
                if (moodChoice == 4) {
                    spinningBehavior();
                } else {
                    handleMood();
                }
                lastPersonalityTime = millis();
            }
        }
        OLED_DisplayStatus(); // Update OLED display
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void sensorTask(void *parameter) {
    for (;;) {
        measurement = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) {
            OLED_AnimateText("Sensor Timeout");
            continue;
        }
        obstacleDetected = (measurement < DISTANCE_THRESHOLD);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void stuckDetectionTask(void *parameter) {
    for (;;) {
        tca9548aSelect(7); // Select the MPU6050 connected on channel 7 of the multiplexer
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // get readings from the MPU
        if (abs(ax) > 1000 || abs(ay) > 1000 || abs(az) > 1000) {
            lastMovementTime = millis();
            stuckDetected = false;
        } else if (millis() - lastMovementTime > 2000) {
            stuckDetected = true;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Setup Function
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        while (1);
    }
    display.clearDisplay();
    display.display();

    if (!sensor.init()) {
        OLED_AnimateText("VL53L0X Error"); // display error when sensor failed then resstart the ESP
        delay(5000);
        ESP.restart();
    }
    sensor.setTimeout(500);
    sensor.setMeasurementTimingBudget(200000);

    tca9548aSelect(7);
    mpu.initialize();
    if (!mpu.testConnection()) {
        OLED_AnimateText("MPU6050 Error");
        delay(5000);
        ESP.restart();
    }

    playSuccessTone();

    xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
    xTaskCreate(motorControlTask, "Motor Task", 2048, NULL, 1, NULL);
    xTaskCreate(stuckDetectionTask, "Stuck Detection", 2048, NULL, 1, NULL);
}

// Loop Function
void loop() {
    // FreeRTOS handles tasks; no need for code in loop()
    vTaskDelay(portMAX_DELAY); // This sets the RTOS task execution to block for a while then other things can execute
}
