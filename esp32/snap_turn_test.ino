/*
 * SNAP TURN TEST - Competition-Style Fast Turns
 * 
 * Tests active braking, kick start, and smooth 90°/180° turns
 * Based on Micromouse competition techniques
 * 
 * LIBRARY REQUIRED: MPU6050_light by rfetick
 * Install: Arduino IDE > Tools > Manage Libraries > "MPU6050_light"
 * 
 * CONTROLS:
 *   BOOT Button → Quick test sequence (R90 → L90 → 180)
 *   
 *   Serial Commands:
 *   'R' → Snap turn RIGHT 90°
 *   'L' → Snap turn LEFT 90°
 *   'U' → Snap U-turn 180°
 *   'F' → Forward with kick start (1 second)
 *   'B' → Test brake only
 *   'C' → Recalibrate gyro
 *   'S' → Show status
 *   'M' → Show menu
 *   
 *   Speed Adjustment:
 *   '+' → Increase turn speed by 10
 *   '-' → Decrease turn speed by 10
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

// ======================= TUNABLE PARAMETERS =======================
// Adjust these to match your robot!

int SPEED_FAST = 180;         // High speed during turn (adjust for your motors)
int SPEED_SLOW = 100;         // Approach speed near target
int SPEED_FORWARD = 150;      // Forward drive speed

#define BRAKE_TIME 50         // How long to reverse brake (ms) - adjust if overshooting
#define KICK_TIME 30          // Full power kick duration (ms)

#define TOLERANCE_90 4.0      // Acceptable error for 90° turns
#define TOLERANCE_180 5.0     // Acceptable error for 180° turns (slightly looser)

#define SLOW_ZONE_90 25       // Start slowing when this close to target (90°)
#define SLOW_ZONE_180 40      // Start slowing when this close to target (180°)

#define TURN_TIMEOUT 2000     // Max time for any turn (ms)

// ======================= GLOBAL VARIABLES =======================

MPU6050 mpu(Wire);

int turnCount = 0;
float totalError = 0;

// Track last motor directions for proper braking
int lastLeftDir = 0;   // 1 = forward, -1 = reverse, 0 = stopped
int lastRightDir = 0;

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP);
    
    Serial.println("\n");
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║     SNAP TURN TEST - Competition      ║");
    Serial.println("║        Style Fast Turning             ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    initMotors();
    
    Serial.println("Initializing MPU6050...");
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    
    if (mpu.begin() != 0) {
        Serial.println("❌ MPU6050 connection failed!");
        while(1) { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); delay(100); }
    }
    
    Serial.println("✅ MPU6050 connected!");
    Serial.println("\n⚠️  CALIBRATING - Keep robot STILL!\n");
    delay(1500);
    mpu.calcOffsets();
    
    Serial.println("✅ Ready!\n");
    printMenu();
    printSettings();
}

void printMenu() {
    Serial.println("╔══════════════════════════════╗");
    Serial.println("║         COMMANDS             ║");
    Serial.println("╠══════════════════════════════╣");
    Serial.println("║  BOOT → Test sequence        ║");
    Serial.println("║  R → Right 90°               ║");
    Serial.println("║  L → Left 90°                ║");
    Serial.println("║  U → U-Turn 180°             ║");
    Serial.println("║  F → Forward + Brake         ║");
    Serial.println("║  B → Brake test only         ║");
    Serial.println("║  C → Recalibrate             ║");
    Serial.println("║  S → Status                  ║");
    Serial.println("║  +/- → Adjust speed          ║");
    Serial.println("╚══════════════════════════════╝\n");
}

void printSettings() {
    Serial.println("Current Settings:");
    Serial.printf("  Fast Speed: %d\n", SPEED_FAST);
    Serial.printf("  Slow Speed: %d\n", SPEED_SLOW);
    Serial.printf("  Brake Time: %d ms\n", BRAKE_TIME);
    Serial.println("");
}

// ======================= MAIN LOOP =======================

void loop() {
    mpu.update();
    
    // Boot button → Test sequence
    if (digitalRead(0) == LOW) {
        delay(300);
        Serial.println("\n🎬 RUNNING TEST SEQUENCE...\n");
        delay(500);
        
        snapRight90();
        delay(1000);
        
        snapLeft90();
        delay(1000);
        
        snapTurn180();
        delay(1000);
        
        Serial.println("\n✅ Sequence complete!\n");
        showStats();
    }
    
    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
    
    // Blink LED
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        lastBlink = millis();
    }
}

void handleCommand(char cmd) {
    switch(cmd) {
        case 'R': case 'r':
            Serial.println("\n▶ SNAP RIGHT 90°");
            snapRight90();
            break;
            
        case 'L': case 'l':
            Serial.println("\n▶ SNAP LEFT 90°");
            snapLeft90();
            break;
            
        case 'U': case 'u':
            Serial.println("\n▶ SNAP U-TURN 180°");
            snapTurn180();
            break;
            
        case 'F': case 'f':
            Serial.println("\n▶ FORWARD + BRAKE TEST");
            testForward();
            break;
            
        case 'B': case 'b':
            Serial.println("\n▶ BRAKE TEST");
            setMotors(150, 150);
            delay(300);
            brakeMotors();
            Serial.println("Brake complete");
            break;
            
        case 'C': case 'c':
            Serial.println("\n🔄 Recalibrating...");
            delay(1000);
            mpu.calcOffsets();
            turnCount = 0;
            totalError = 0;
            Serial.println("✅ Done!\n");
            break;
            
        case 'S': case 's':
            mpu.update();
            showStats();
            break;
            
        case 'M': case 'm':
            printMenu();
            printSettings();
            break;
            
        case '+': case '=':
            SPEED_FAST += 10;
            SPEED_SLOW += 5;
            Serial.printf("Speed UP: Fast=%d, Slow=%d\n", SPEED_FAST, SPEED_SLOW);
            break;
            
        case '-': case '_':
            SPEED_FAST = max(100, SPEED_FAST - 10);
            SPEED_SLOW = max(60, SPEED_SLOW - 5);
            Serial.printf("Speed DOWN: Fast=%d, Slow=%d\n", SPEED_FAST, SPEED_SLOW);
            break;
            
        case '\n': case '\r':
            break;
            
        default:
            Serial.printf("Unknown: '%c' (send M for menu)\n", cmd);
    }
}

void showStats() {
    Serial.println("\n╔═══════════════════════════╗");
    Serial.println("║        STATUS             ║");
    Serial.println("╠═══════════════════════════╣");
    Serial.printf("║  Yaw: %7.1f°            ║\n", mpu.getAngleZ());
    Serial.printf("║  Gyro Z: %6.1f °/s       ║\n", mpu.getGyroZ());
    Serial.printf("║  Turns: %d                ║\n", turnCount);
    if (turnCount > 0) {
        Serial.printf("║  Avg Error: %.1f°         ║\n", totalError / turnCount);
    }
    Serial.println("╚═══════════════════════════╝\n");
}

// ======================= MOTOR FUNCTIONS =======================

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); 
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); 
    pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT);
    pinMode(MOTOR_ENB, OUTPUT);
    stopMotors();
}

void setMotors(int L, int R) {
    // Track directions for braking
    lastLeftDir = (L > 0) ? 1 : (L < 0) ? -1 : 0;
    lastRightDir = (R > 0) ? 1 : (R < 0) ? -1 : 0;
    
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
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);
    analogWrite(MOTOR_ENA, 0);
    analogWrite(MOTOR_ENB, 0);
    lastLeftDir = 0;
    lastRightDir = 0;
}

// ACTIVE BRAKING - Reverses motor direction briefly to stop instantly
void brakeMotors() {
    // Reverse the direction we were going
    int brakeL = -lastLeftDir;
    int brakeR = -lastRightDir;
    
    // Full power brake in opposite direction
    if (brakeL > 0) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); }
    else if (brakeL < 0) { digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH); }
    
    if (brakeR > 0) { digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW); }
    else if (brakeR < 0) { digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH); }
    
    analogWrite(MOTOR_ENA, 255);
    analogWrite(MOTOR_ENB, 255);
    
    delay(BRAKE_TIME);
    
    stopMotors();
}

// KICK START - Full power pulse to overcome static friction
void kickStart() {
    setMotors(255, 255);
    delay(KICK_TIME);
    setMotors(SPEED_FORWARD, SPEED_FORWARD);
}

// ======================= SNAP TURN FUNCTIONS =======================

float normalizeAngle(float diff) {
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

void snapRight90() {
    Serial.println("┌────────────────────────────┐");
    Serial.println("│   ➡️  SNAP RIGHT 90°       │");
    Serial.println("└────────────────────────────┘");
    
    mpu.update();
    float start = mpu.getAngleZ();
    float target = start + 90.0;
    
    Serial.printf("Start: %.1f° → Target: %.1f°\n", start, target);
    
    unsigned long startTime = millis();
    bool success = false;
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float diff = normalizeAngle(target - mpu.getAngleZ());
        
        // Hit target?
        if (abs(diff) < TOLERANCE_90) {
            brakeMotors();
            success = true;
            break;
        }
        
        // Two-stage speed
        int speed = (abs(diff) > SLOW_ZONE_90) ? SPEED_FAST : SPEED_SLOW;
        setMotors(speed, -speed);
    }
    
    if (!success) stopMotors();
    
    delay(50);
    mpu.update();
    
    printTurnResult(target, mpu.getAngleZ(), millis() - startTime, success);
}

void snapLeft90() {
    Serial.println("┌────────────────────────────┐");
    Serial.println("│   ⬅️  SNAP LEFT 90°        │");
    Serial.println("└────────────────────────────┘");
    
    mpu.update();
    float start = mpu.getAngleZ();
    float target = start - 90.0;
    
    Serial.printf("Start: %.1f° → Target: %.1f°\n", start, target);
    
    unsigned long startTime = millis();
    bool success = false;
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float diff = normalizeAngle(target - mpu.getAngleZ());
        
        if (abs(diff) < TOLERANCE_90) {
            brakeMotors();
            success = true;
            break;
        }
        
        int speed = (abs(diff) > SLOW_ZONE_90) ? SPEED_FAST : SPEED_SLOW;
        setMotors(-speed, speed);
    }
    
    if (!success) stopMotors();
    
    delay(50);
    mpu.update();
    
    printTurnResult(target, mpu.getAngleZ(), millis() - startTime, success);
}

void snapTurn180() {
    Serial.println("┌────────────────────────────┐");
    Serial.println("│   🔄 SNAP U-TURN 180°      │");
    Serial.println("└────────────────────────────┘");
    
    mpu.update();
    float start = mpu.getAngleZ();
    float target = start + 180.0;  // Always turn right for 180
    
    Serial.printf("Start: %.1f° → Target: %.1f°\n", start, target);
    
    unsigned long startTime = millis();
    bool success = false;
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float diff = normalizeAngle(target - mpu.getAngleZ());
        
        if (abs(diff) < TOLERANCE_180) {
            brakeMotors();
            success = true;
            break;
        }
        
        // Slower approach zone for 180° (more momentum)
        int speed = (abs(diff) > SLOW_ZONE_180) ? SPEED_FAST : SPEED_SLOW;
        setMotors(speed, -speed);
    }
    
    if (!success) stopMotors();
    
    delay(50);
    mpu.update();
    
    printTurnResult(target, mpu.getAngleZ(), millis() - startTime, success);
}

void printTurnResult(float target, float actual, unsigned long time, bool success) {
    float error = normalizeAngle(target - actual);
    
    turnCount++;
    totalError += abs(error);
    
    Serial.println("\n┌─── RESULT ───┐");
    Serial.printf("│ Final: %6.1f°│\n", actual);
    Serial.printf("│ Error: %+5.1f° ", error);
    
    if (abs(error) < 2) Serial.println("✅ PERFECT");
    else if (abs(error) < 4) Serial.println("✅ GREAT");
    else if (abs(error) < 6) Serial.println("⚠️  OK");
    else Serial.println("❌ POOR");
    
    Serial.printf("│ Time: %4lu ms │\n", time);
    
    if (!success) {
        Serial.println("│ ❌ TIMEOUT!  │");
    }
    
    Serial.println("└──────────────┘\n");
}

void testForward() {
    Serial.println("Testing: Kick Start → Forward → Brake\n");
    
    kickStart();
    Serial.println("  Kick start done");
    
    delay(800);
    Serial.println("  Driving...");
    
    brakeMotors();
    Serial.println("  Brake complete!\n");
}
