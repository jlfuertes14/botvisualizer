/*
 * MPU6050 Turning Test & Calibration (Using MPU6050_light Library)
 * 
 * PURPOSE: Standalone test for MPU6050 gyroscope turning accuracy
 * 
 * LIBRARY REQUIRED: MPU6050_light by rfetick
 * Install via: Arduino IDE > Tools > Manage Libraries > Search "MPU6050_light"
 * 
 * CONTROLS:
 * - Press BOOT button → Turn RIGHT 90°
 * - Serial command "L" → Turn LEFT 90°
 * - Serial command "R" → Turn RIGHT 90°
 * - Serial command "C" → Recalibrate gyro
 * - Serial command "S" → Show current yaw
 * 
 * USE THIS TO:
 * - Test if MPU6050 is working correctly
 * - Calibrate turn accuracy (adjust TURN_SPEED if needed)
 * - Check for gyro drift issues
 * - Verify motor balance
 */

#include <Wire.h>
#include <MPU6050_light.h>

// ======================= PIN DEFINITIONS =======================

// L298N Motor Driver
#define MOTOR_IN1 23
#define MOTOR_IN2 22
#define MOTOR_ENA 25 
#define MOTOR_IN3 21
#define MOTOR_IN4 19
#define MOTOR_ENB 26 

// MPU6050 I2C Pins
#define MPU6050_SDA 32 
#define MPU6050_SCL 33

#define STATUS_LED 2

// ======================= TUNING CONSTANTS =======================

#define TURN_SPEED 140        // Adjust if turns too fast/slow
#define TURN_TOLERANCE 5.0    // Accept turn within ±5°
#define TURN_TIMEOUT 3000     // Max time for 90° turn (ms)

// ======================= GLOBAL VARIABLES =======================

MPU6050 mpu(Wire);

float startYaw = 0;
int turnCount = 0;
float totalError = 0;A

// ======================= FUNCTION DECLARATIONS =======================

void initMotors();
void setMotors(int L, int R);
void stopMotors();
void turnRight90();
void turnLeft90();
float normalizeAngleDiff(float target, float current);
void printMenu();
void recalibrate();

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP);  // Boot button
    
    Serial.println("\n\n=================================");
    Serial.println("MPU6050 TURNING TEST & CALIBRATION");
    Serial.println("    Using MPU6050_light Library");
    Serial.println("=================================\n");
    
    Serial.println("Initializing motors...");
    initMotors();
    
    Serial.println("Initializing MPU6050...");
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    
    byte status = mpu.begin();
    Serial.print("MPU6050 status: ");
    Serial.println(status);
    
    if (status != 0) {
        Serial.println("❌ ERROR: Could not connect to MPU6050!");
        Serial.println("Check wiring:");
        Serial.println("  SDA → Pin 32");
        Serial.println("  SCL → Pin 33");
        Serial.println("  VCC → 3.3V or 5V");
        Serial.println("  GND → GND");
        while(1) {
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(200);
        }
    }
    
    Serial.println("✅ MPU6050 connected!\n");
    Serial.println("Calibrating gyro & accelerometer...");
    Serial.println("⚠️  IMPORTANT: Keep robot PERFECTLY STILL!");
    Serial.println("Calibration will take ~3 seconds...\n");
    
    delay(1000);
    mpu.calcOffsets();  // Calibrate gyro and accelerometer
    
    Serial.println("✅ Calibration complete!");
    Serial.println("\n✅ Ready for testing!\n");
    
    printMenu();
}

void printMenu() {
    Serial.println("\n--- CONTROLS ---");
    Serial.println("Press BOOT button → Turn RIGHT 90°");
    Serial.println("Send 'L' → Turn LEFT 90°");
    Serial.println("Send 'R' → Turn RIGHT 90°");
    Serial.println("Send 'C' → Recalibrate gyro");
    Serial.println("Send 'S' → Show current yaw");
    Serial.println("Send 'M' → Show this menu");
    Serial.println("Send 'Z' → Reset yaw to 0°");
    Serial.println("----------------\n");
}

// ======================= MAIN LOOP =======================

void loop() {
    // Update MPU6050 readings
    mpu.update();
    
    // Boot button → Right turn
    if (digitalRead(0) == LOW) {
        delay(300);  // Debounce
        Serial.println("\n🔵 BOOT BUTTON PRESSED");
        turnRight90();
        delay(1000);  // Prevent rapid triggers
    }
    
    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case 'L':
            case 'l':
                Serial.println("\n📍 LEFT TURN REQUESTED");
                turnLeft90();
                break;
                
            case 'R':
            case 'r':
                Serial.println("\n📍 RIGHT TURN REQUESTED");
                turnRight90();
                break;
                
            case 'C':
            case 'c':
                Serial.println("\n🔄 RECALIBRATING...");
                recalibrate();
                break;
                
            case 'Z':
            case 'z':
                Serial.println("\n🔄 RESETTING YAW TO 0°");
                mpu.update();
                startYaw = mpu.getAngleZ();
                Serial.println("✅ Yaw reset!");
                break;
                
            case 'S':
            case 's':
                mpu.update();
                Serial.printf("\n📊 Current Yaw: %.1f°\n", mpu.getAngleZ() - startYaw);
                Serial.printf("Gyro Z: %.2f °/s\n", mpu.getGyroZ());
                Serial.printf("Angle Z (raw): %.1f°\n", mpu.getAngleZ());
                if (turnCount > 0) {
                    Serial.printf("Turns completed: %d\n", turnCount);
                    Serial.printf("Average error: %.2f°\n", totalError / turnCount);
                }
                break;
                
            case 'M':
            case 'm':
                printMenu();
                break;
                
            case '\n':
            case '\r':
                // Ignore newlines
                break;
                
            default:
                Serial.printf("Unknown command: '%c'\n", cmd);
                Serial.println("Send 'M' for menu");
        }
    }
    
    // Blink LED to show alive
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        lastBlink = millis();
    }
}

// ======================= MPU6050 FUNCTIONS =======================

void recalibrate() {
    Serial.println("Keep robot still...");
    delay(2000);
    
    turnCount = 0;
    totalError = 0;
    
    mpu.calcOffsets();
    startYaw = mpu.getAngleZ();
    
    Serial.println("✅ Recalibration complete!\n");
}

// ======================= TURNING FUNCTIONS =======================

// Helper function to normalize angle difference to -180 to +180
float normalizeAngleDiff(float target, float current) {
    float diff = target - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

void turnRight90() {
    Serial.println("\n┌─────────────────────────────────┐");
    Serial.println("│   🔄 RIGHT TURN (90°)          │");
    Serial.println("└─────────────────────────────────┘");
    
    // Get fresh yaw reading
    mpu.update();
    float initialYaw = mpu.getAngleZ();
    float target = initialYaw + 90.0;
    
    Serial.printf("Start angle: %.1f°\n", initialYaw);
    Serial.printf("Target angle: %.1f°\n", target);
    Serial.println("Turning...");
    
    unsigned long startTime = millis();
    bool reachedTarget = false;
    float maxGyro = 0;
   
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        
        float currentYaw = mpu.getAngleZ();
        float gyroZ = abs(mpu.getGyroZ());
        if (gyroZ > maxGyro) maxGyro = gyroZ;
        
        float diff = normalizeAngleDiff(target, currentYaw);
        
        // Check if reached target
        if (abs(diff) < TURN_TOLERANCE) {
            reachedTarget = true;
            break;
        }
        
        // Proportional speed control
        int turnSpeed = TURN_SPEED;
        if (abs(diff) < 30) {
            turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
        }
        
        setMotors(turnSpeed, -turnSpeed);
        delay(10);
    }
    
    stopMotors();
    
    // Let gyro settle
    delay(200);
    mpu.update();
    
    float finalYaw = mpu.getAngleZ();
    unsigned long turnTime = millis() - startTime;
    float finalError = normalizeAngleDiff(target, finalYaw);
    
    // Statistics
    turnCount++;
    totalError += abs(finalError);
    
    // Print results
    Serial.println("\n--- RESULTS ---");
    Serial.printf("Final angle: %.1f°\n", finalYaw);
    Serial.printf("Error: %.1f° ", finalError);
    
    if (abs(finalError) < 3) {
        Serial.println("✅ EXCELLENT");
    } else if (abs(finalError) < 5) {
        Serial.println("✅ GOOD");
    } else if (abs(finalError) < 10) {
        Serial.println("⚠️  ACCEPTABLE");
    } else {
        Serial.println("❌ POOR");
    }
    
    Serial.printf("Turn time: %lu ms\n", turnTime);
    Serial.printf("Max gyro: %.1f °/s\n", maxGyro);
    
    if (!reachedTarget) {
        Serial.println("❌ TIMEOUT! Turn did not complete.");
        Serial.println("Possible issues:");
        Serial.println("  - Motors too weak");
        Serial.println("  - TURN_SPEED too low");
        Serial.println("  - Battery voltage low");
        Serial.println("  - Gyro malfunction");
    }
    
    Serial.println("---------------\n");
}

void turnLeft90() {
    Serial.println("\n┌─────────────────────────────────┐");
    Serial.println("│   🔄 LEFT TURN (90°)           │");
    Serial.println("└─────────────────────────────────┘");
    
    // Get fresh yaw reading
    mpu.update();
    float initialYaw = mpu.getAngleZ();
    float target = initialYaw - 90.0;
    
    Serial.printf("Start angle: %.1f°\n", initialYaw);
    Serial.printf("Target angle: %.1f°\n", target);
    Serial.println("Turning...");
    
    unsigned long startTime = millis();
    bool reachedTarget = false;
    float maxGyro = 0;
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        
        float currentYaw = mpu.getAngleZ();
        float gyroZ = abs(mpu.getGyroZ());
        if (gyroZ > maxGyro) maxGyro = gyroZ;
        
        float diff = normalizeAngleDiff(target, currentYaw);
        
        // Check if reached target
        if (abs(diff) < TURN_TOLERANCE) {
            reachedTarget = true;
            break;
        }
        
        // Proportional speed control
        int turnSpeed = TURN_SPEED;
        if (abs(diff) < 30) {
            turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
        }
        
        setMotors(-turnSpeed, turnSpeed);
        delay(10);
    }
    
    stopMotors();
    
    // Let gyro settle
    delay(200);
    mpu.update();
    
    float finalYaw = mpu.getAngleZ();
    unsigned long turnTime = millis() - startTime;
    float finalError = normalizeAngleDiff(target, finalYaw);
    
    // Statistics
    turnCount++;
    totalError += abs(finalError);
    
    // Print results
    Serial.println("\n--- RESULTS ---");
    Serial.printf("Final angle: %.1f°\n", finalYaw);
    Serial.printf("Error: %.1f° ", finalError);
    
    if (abs(finalError) < 3) {
        Serial.println("✅ EXCELLENT");
    } else if (abs(finalError) < 5) {
        Serial.println("✅ GOOD");
    } else if (abs(finalError) < 10) {
        Serial.println("⚠️  ACCEPTABLE");
    } else {
        Serial.println("❌ POOR");
    }
    
    Serial.printf("Turn time: %lu ms\n", turnTime);
    Serial.printf("Max gyro: %.1f °/s\n", maxGyro);
    
    if (!reachedTarget) {
        Serial.println("❌ TIMEOUT! Turn did not complete.");
        Serial.println("Possible issues:");
        Serial.println("  - Motors too weak");
        Serial.println("  - TURN_SPEED too low");
        Serial.println("  - Battery voltage low");
        Serial.println("  - Gyro malfunction");
    }
    
    Serial.println("---------------\n");
}

// ======================= MOTOR FUNCTIONS =======================

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); 
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); 
    pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT);
    pinMode(MOTOR_ENB, OUTPUT);
}

void setMotors(int L, int R) {
    // Left motor
    if (L > 0) { 
        digitalWrite(MOTOR_IN1, HIGH); 
        digitalWrite(MOTOR_IN2, LOW); 
    } else { 
        digitalWrite(MOTOR_IN1, LOW); 
        digitalWrite(MOTOR_IN2, HIGH); 
        L = -L; 
    }
    analogWrite(MOTOR_ENA, constrain(L, 0, 255));

    // Right motor
    if (R > 0) { 
        digitalWrite(MOTOR_IN3, HIGH); 
        digitalWrite(MOTOR_IN4, LOW); 
    } else { 
        digitalWrite(MOTOR_IN3, LOW); 
        digitalWrite(MOTOR_IN4, HIGH); 
        R = -R; 
    }
    analogWrite(MOTOR_ENB, constrain(R, 0, 255));
}

void stopMotors() { 
    setMotors(0, 0); 
}
