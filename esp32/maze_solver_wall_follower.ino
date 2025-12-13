/*
 * Maze Solver Bot - Wall Follower Algorithm with PID
 * 
 * Logic: Left-Hand Rule (always keep left wall)
 * 1. If no wall on LEFT → Turn LEFT and go forward
 * 2. If wall on LEFT and no wall FRONT → Go forward
 * 3. If wall on LEFT and wall FRONT → Turn RIGHT
 * 4. If walls everywhere → Turn around (180°)
 * 
 * PID Control: Uses left & right ultrasonic sensors to stay centered
 * 
 * This algorithm GUARANTEES maze exit for simply-connected mazes!
 */

#include <Wire.h>
#include <NewPing.h>

// ======================= PIN DEFINITIONS =======================

// HC-SR04 Ultrasonic Sensors
#define FRONT_TRIG_PIN 14
#define FRONT_ECHO_PIN 35
#define LEFT_TRIG_PIN 18
#define LEFT_ECHO_PIN 13
#define RIGHT_TRIG_PIN 16
#define RIGHT_ECHO_PIN 34

#define MAX_DISTANCE 80  // Maximum distance in cm
#define SENSOR_SAMPLES 3  // Number of samples for median filter

// NewPing sensor objects
NewPing sonarFront(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

// TCRT5000 Line Sensor (for cell counting/goal detection)
#define LINE_SENSOR_PIN 4

// L298N Motor Driver
#define MOTOR_IN1 23
#define MOTOR_IN2 22
#define MOTOR_ENA 25 
#define MOTOR_IN3 21
#define MOTOR_IN4 19
#define MOTOR_ENB 26 

// MPU6050
#define MPU6050_SDA 32 
#define MPU6050_SCL 33
#define MPU6050_ADDR 0x68

#define STATUS_LED 2

// ======================= TUNING CONSTANTS =======================

// Motor speeds
#define BASE_SPEED 100        // Normal driving speed
#define TURN_SPEED 130        // Turning speed
#define MIN_SPEED 60          // Minimum speed for PID corrections
#define MAX_SPEED 160         // Maximum speed for PID corrections

// Wall detection thresholds (cm)
#define WALL_THRESHOLD 12.0   // Distance to consider a wall present
#define FRONT_STOP_DIST 8.0   // Stop if front wall closer than this
#define IDEAL_WALL_DIST 7.0   // Ideal distance from side wall (center of corridor)

// PID Constants for wall centering
#define KP 8.0                // Proportional gain
#define KI 0.0                // Integral gain (usually 0 for maze)
#define KD 3.0                // Derivative gain

// Maze parameters
#define MAZE_SIZE 5
#define CELL_DRIVE_TIME 400   // ms to drive through one cell (tune this!)

// ======================= GLOBAL VARIABLES =======================

enum RobotState { 
  STATE_IDLE, 
  STATE_WALL_FOLLOW,
  STATE_TURNING,
  STATE_FINISHED 
};
RobotState robotState = STATE_IDLE;

// PID variables
float lastError = 0;
float integral = 0;

// MPU6050 variables
float gyroZOffset = 0;
float currentYaw = 0;
unsigned long lastMPUUpdate = 0;

// Cell counting
int cellCount = 0;
volatile bool lineCrossed = false;
volatile unsigned long lastLinePulse = 0;

// ======================= FUNCTION DECLARATIONS =======================

void initMotors();
void setMotors(int L, int R);
void stopMotors();
float readDistanceFront();
float readDistanceLeft();
float readDistanceRight();
void initMPU6050();
void updateYaw();
void turnRight90();
void turnLeft90();
void turn180();
void driveWithPID();
void wallFollowerStep();
float normalizeAngleDiff(float target, float current);
void IRAM_ATTR onLineCrossed();

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP);  // Boot button
    
    initSensors();
    initMotors();
    initMPU6050();
    
    Serial.println("=================================");
    Serial.println("  WALL FOLLOWER Maze Solver");
    Serial.println("  Algorithm: Left-Hand Rule + PID");
    Serial.println("=================================");
    Serial.println("Press BOOT button to start...");
}

void initSensors() {
    pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(LEFT_TRIG_PIN, OUTPUT); pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT); pinMode(RIGHT_ECHO_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LINE_SENSOR_PIN), onLineCrossed, FALLING);
}

// ======================= MAIN LOOP =======================

void loop() {
    // Start when Boot button pressed
    if (robotState == STATE_IDLE && digitalRead(0) == LOW) {
        delay(500);  // Debounce
        Serial.println("Starting Wall Follower!");
        cellCount = 0;
        robotState = STATE_WALL_FOLLOW;
    }
    
    switch (robotState) {
        case STATE_IDLE:
            // Blink LED slowly
            digitalWrite(STATUS_LED, (millis() % 1000 < 500) ? HIGH : LOW);
            break;
            
        case STATE_WALL_FOLLOW:
            digitalWrite(STATUS_LED, HIGH);
            wallFollowerStep();
            break;
            
        case STATE_TURNING:
            // Handled within turn functions
            break;
            
        case STATE_FINISHED:
            stopMotors();
            // Fast blink to indicate finish
            digitalWrite(STATUS_LED, (millis() % 300 < 150) ? HIGH : LOW);
            
            // Press boot to restart
            if (digitalRead(0) == LOW) {
                delay(500);
                cellCount = 0;
                robotState = STATE_WALL_FOLLOW;
            }
            break;
    }
}

// ======================= WALL FOLLOWER ALGORITHM =======================

void wallFollowerStep() {
    // Read all sensors
    float distFront = readDistanceFront();
    float distLeft = readDistanceLeft();
    float distRight = readDistanceRight();
    
    bool wallFront = (distFront < WALL_THRESHOLD);
    bool wallLeft = (distLeft < WALL_THRESHOLD);
    bool wallRight = (distRight < WALL_THRESHOLD);
    
    Serial.printf("Distances - F:%.1f L:%.1f R:%.1f | Walls - F:%d L:%d R:%d\n",
                  distFront, distLeft, distRight, wallFront, wallLeft, wallRight);
    
    // Check for goal (open on all sides or specific pattern)
    // For 5x5 maze with goal at (0,0), you might detect goal differently
    // Here we check if we've crossed enough cells
    if (cellCount >= MAZE_SIZE * MAZE_SIZE) {
        Serial.println("Max cells reached - assuming goal!");
        robotState = STATE_FINISHED;
        return;
    }
    
    // LEFT-HAND RULE DECISION
    if (!wallLeft) {
        // Case 1: No wall on left → Turn LEFT and go forward
        Serial.println("ACTION: Turn LEFT (no left wall)");
        stopMotors();
        delay(100);
        turnLeft90();
        delay(200);
        driveForwardOneCell();
    }
    else if (wallLeft && !wallFront) {
        // Case 2: Wall on left, no wall in front → Go FORWARD with PID
        Serial.println("ACTION: Drive FORWARD (wall on left, clear ahead)");
        driveWithPID();
    }
    else if (wallLeft && wallFront && !wallRight) {
        // Case 3: Wall on left and front → Turn RIGHT
        Serial.println("ACTION: Turn RIGHT (left & front blocked)");
        stopMotors();
        delay(100);
        turnRight90();
        delay(200);
    }
    else {
        // Case 4: Dead end (walls on all sides) → Turn around
        Serial.println("ACTION: DEAD END - Turn 180°");
        stopMotors();
        delay(100);
        turn180();
        delay(200);
    }
}

// ======================= PID WALL CENTERING =======================

void driveWithPID() {
    float distLeft = readDistanceLeft();
    float distRight = readDistanceRight();
    float distFront = readDistanceFront();
    
    // Stop if front wall too close
    if (distFront < FRONT_STOP_DIST) {
        stopMotors();
        return;
    }
    
    // Calculate error (positive = too close to right, negative = too close to left)
    float error = 0;
    
    if (distLeft < WALL_THRESHOLD && distRight < WALL_THRESHOLD) {
        // Both walls visible - center between them
        error = distLeft - distRight;
    }
    else if (distLeft < WALL_THRESHOLD) {
        // Only left wall - maintain ideal distance from it
        error = IDEAL_WALL_DIST - distLeft;
    }
    else if (distRight < WALL_THRESHOLD) {
        // Only right wall - maintain ideal distance from it
        error = distRight - IDEAL_WALL_DIST;
    }
    else {
        // No walls - just go straight
        error = 0;
    }
    
    // PID calculation
    integral += error;
    integral = constrain(integral, -50, 50);  // Anti-windup
    
    float derivative = error - lastError;
    lastError = error;
    
    float correction = (KP * error) + (KI * integral) + (KD * derivative);
    
    // Apply correction to motors
    int leftSpeed = constrain(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);
    int rightSpeed = constrain(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
    
    setMotors(leftSpeed, rightSpeed);
    
    // Small delay for control loop
    delay(20);
}

void driveForwardOneCell() {
    lineCrossed = false;
    unsigned long startTime = millis();
    
    // Drive with PID until we cross a cell or timeout
    while (!lineCrossed && (millis() - startTime < 2000)) {
        float distFront = readDistanceFront();
        
        // Emergency stop if wall ahead
        if (distFront < FRONT_STOP_DIST) {
            break;
        }
        
        driveWithPID();
    }
    
    stopMotors();
    cellCount++;
    Serial.printf("Cell crossed! Total: %d\n", cellCount);
    delay(100);
}

// ======================= TURNING FUNCTIONS =======================

float normalizeAngleDiff(float target, float current) {
    float diff = target - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

void turnRight90() {
    Serial.println("Turning RIGHT 90°");
    
    updateYaw();
    float startYaw = currentYaw;
    float target = startYaw + 90.0;
    while (target >= 360) target -= 360;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < 3000) {
        updateYaw();
        float diff = normalizeAngleDiff(target, currentYaw);
        
        if (abs(diff) < 5.0) break;
        
        int turnSpeed = TURN_SPEED;
        if (abs(diff) < 30) {
            turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
        }
        
        setMotors(turnSpeed, -turnSpeed);
        delay(10);
    }
    
    stopMotors();
    lastError = 0;  // Reset PID
    integral = 0;
    Serial.println("Right turn complete");
}

void turnLeft90() {
    Serial.println("Turning LEFT 90°");
    
    updateYaw();
    float startYaw = currentYaw;
    float target = startYaw - 90.0;
    while (target < 0) target += 360;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < 3000) {
        updateYaw();
        float diff = normalizeAngleDiff(target, currentYaw);
        
        if (abs(diff) < 5.0) break;
        
        int turnSpeed = TURN_SPEED;
        if (abs(diff) < 30) {
            turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
        }
        
        setMotors(-turnSpeed, turnSpeed);
        delay(10);
    }
    
    stopMotors();
    lastError = 0;  // Reset PID
    integral = 0;
    Serial.println("Left turn complete");
}

void turn180() {
    Serial.println("Turning 180°");
    
    updateYaw();
    float startYaw = currentYaw;
    float target = startYaw + 180.0;
    while (target >= 360) target -= 360;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < 5000) {
        updateYaw();
        float diff = normalizeAngleDiff(target, currentYaw);
        
        if (abs(diff) < 5.0) break;
        
        int turnSpeed = TURN_SPEED;
        if (abs(diff) < 30) {
            turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
        }
        
        setMotors(turnSpeed, -turnSpeed);
        delay(10);
    }
    
    stopMotors();
    lastError = 0;  // Reset PID
    integral = 0;
    Serial.println("180° turn complete");
}

// ======================= MOTOR CONTROL =======================

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT);
    pinMode(MOTOR_ENB, OUTPUT);
}

void setMotors(int L, int R) {
    if (L > 0) { 
        digitalWrite(MOTOR_IN1, HIGH); 
        digitalWrite(MOTOR_IN2, LOW); 
    } else { 
        digitalWrite(MOTOR_IN1, LOW); 
        digitalWrite(MOTOR_IN2, HIGH); 
        L = -L; 
    }
    analogWrite(MOTOR_ENA, constrain(L, 0, 255));

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

// ======================= SENSOR FUNCTIONS =======================

float readDistanceFront() {
    unsigned int distance = sonarFront.ping_median(SENSOR_SAMPLES);
    if (distance == 0) return 999.0;
    return sonarFront.convert_cm(distance);
}

float readDistanceLeft() {
    unsigned int distance = sonarLeft.ping_median(SENSOR_SAMPLES);
    if (distance == 0) return 999.0;
    return sonarLeft.convert_cm(distance);
}

float readDistanceRight() {
    unsigned int distance = sonarRight.ping_median(SENSOR_SAMPLES);
    if (distance == 0) return 999.0;
    return sonarRight.convert_cm(distance);
}

// ======================= MPU6050 =======================

void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    
    delay(100);
    
    // Calibrate gyro
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 2, true);
        int16_t raw = (Wire.read() << 8) | Wire.read();
        sum += raw / 131.0;
        delay(5);
    }
    gyroZOffset = sum / 100.0;
    
    lastMPUUpdate = millis();
    currentYaw = 0;
    
    Serial.printf("MPU6050 calibrated. Offset: %.2f\n", gyroZOffset);
}

void updateYaw() {
    unsigned long now = millis();
    float dt = (now - lastMPUUpdate) / 1000.0;
    lastMPUUpdate = now;
    
    if (dt > 0.1) dt = 0.02;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    float gyroZ = (raw / 131.0) - gyroZOffset;
    
    if (abs(gyroZ) > 0.5) currentYaw += gyroZ * dt;
    
    while (currentYaw < 0) currentYaw += 360;
    while (currentYaw >= 360) currentYaw -= 360;
}

// ======================= INTERRUPT HANDLER =======================

void IRAM_ATTR onLineCrossed() {
    unsigned long now = millis();
    if (now - lastLinePulse > 200) {
        lineCrossed = true;
        lastLinePulse = now;
    }
}
