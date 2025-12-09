/*
 * Maze Solver Bot - ESP32 Q-Learning Firmware (Single File Version)
 * 
 * Hardware:
 * - ESP32 DevKit
 * - 3x HC-SR04 Ultrasonic Sensors (Front, Left, Right)
 * - 1x TCRT5000 Line Sensor (Blue, facing down)
 * - MPU6050 IMU (Precise turning)
 * - TB6612FNG Motor Driver
 * 
 * Marker System:
 * - Single line: Cell boundary
 * - Double line: START position
 * - Triple line: GOAL
 * 
 * Note: Uses ESP32 Arduino Core 3.x LEDC API
 */

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ======================= CONFIGURATION =======================

// WiFi Credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ======================= PIN DEFINITIONS =======================

// HC-SR04 Ultrasonic Sensors
#define FRONT_TRIG_PIN 12
#define FRONT_ECHO_PIN 14
#define LEFT_TRIG_PIN 27
#define LEFT_ECHO_PIN 26
#define RIGHT_TRIG_PIN 25
#define RIGHT_ECHO_PIN 33

// TCRT5000 Line Sensor (facing down, on back of robot)
#define LINE_SENSOR_PIN 34
#define LINE_THRESHOLD 2048  // Analog threshold for line detection (adjust as needed)

// TB6612FNG Motor Driver
#define MOTOR_STBY 4
#define MOTOR_AIN1 16
#define MOTOR_AIN2 17
#define MOTOR_PWMA 5
#define MOTOR_BIN1 18
#define MOTOR_BIN2 19
#define MOTOR_PWMB 21

// MPU6050 (I2C)
#define MPU6050_SDA 21
#define MPU6050_SCL 22

// ======================= THRESHOLDS =======================

// Wall detection threshold (cm)
#define WALL_THRESHOLD 15

// Motor speed (0-255)
#define BASE_SPEED 150
#define TURN_SPEED 120

// PID constants for straight driving
#define KP 2.0
#define KI 0.0
#define KD 0.5

// Turn tolerance (degrees)
#define TURN_TOLERANCE 3

// ======================= MOTOR CONSTANTS =======================

// PWM settings (ESP32 3.x uses pin-based API, channels are auto-assigned)
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ======================= MPU6050 CONSTANTS =======================

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// ======================= Q-LEARNING =======================

#define MAZE_SIZE 5
#define NUM_STATES (MAZE_SIZE * MAZE_SIZE)  // 25 states
#define NUM_ACTIONS 4  // Forward, Right, Backward, Left

// Q-Table: Q[state][action]
float Q[NUM_STATES][NUM_ACTIONS];

// Learning parameters
float alpha = 0.3;      // Learning rate
float gamma_rl = 0.9;   // Discount factor (renamed to avoid conflict)
float epsilon = 0.5;    // Exploration rate (starts high, decreases)
float epsilonDecay = 0.95;
float epsilonMin = 0.1;

// ======================= POSITION TRACKING =======================

int currentX = 0;
int currentY = 0;
int heading = 0;  // 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST

// Heading enums
enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// ======================= STATE MACHINE =======================

enum RobotState { 
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_EXPLORING,
    STATE_AT_GOAL,
    STATE_RETURNING
};
RobotState robotState = STATE_IDLE;

// ======================= EPISODE TRACKING =======================

int episode = 0;
int stepCount = 0;
float totalReward = 0;
bool isAtGoal = false;
bool isAtStart = false;

// ======================= LINE DETECTION =======================

volatile bool cellCrossed = false;
volatile int lineCount = 0;
volatile unsigned long firstPulse = 0;
unsigned long lastCross = 0;

// ======================= MPU6050 VARIABLES =======================

float gyroZOffset = 0;
float currentYaw = 0;
unsigned long lastMPUUpdate = 0;

// ======================= MOTOR PID VARIABLES =======================

float targetHeading = 0;
float lastError = 0;
float integral = 0;

// ======================= WEBSOCKET =======================

WebSocketsServer webSocket(81);
bool clientConnected = false;

// ======================= FUNCTION DECLARATIONS =======================

// Sensors
void initSensors();
float readDistance(int trigPin, int echoPin);
float readFrontDistance();
float readLeftDistance();
float readRightDistance();
bool isFrontWall();
bool isLeftWall();
bool isRightWall();
bool isOnLine();

// Motors
void initMotors();
void setMotorA(int speed);
void setMotorB(int speed);
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void driveForward(int speed = BASE_SPEED);
void driveBackward(int speed = BASE_SPEED);
void turnLeftInPlace(int speed = TURN_SPEED);
void turnRightInPlace(int speed = TURN_SPEED);
void lockHeading(float heading);
void driveForwardWithCorrection();

// MPU6050
void initMPU6050();
float readRawGyroZ();
void updateYaw();
float readMPU6050Yaw();
void calibrateMPU6050Heading();
void turnRight90();
void turnLeft90();

// Q-Learning
void qLearningStep();
int selectAction(int state);
void executeAction(int action);
float getReward();
void updateQValue(int state, int action, int nextState, float reward);
int getState();
void updatePositionBasedOnHeading();
void checkMarkers();
void sendTelemetry();
void sendEpisodeComplete();
void resetEpisode();
void initQLearning();

// ======================= SENSOR FUNCTIONS =======================

void initSensors() {
    // Ultrasonic sensors
    pinMode(FRONT_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(LEFT_TRIG_PIN, OUTPUT);
    pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);
    
    // Line sensor
    pinMode(LINE_SENSOR_PIN, INPUT);
    
    Serial.println("Sensors initialized");
}

float readDistance(int trigPin, int echoPin) {
    // Send trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read echo
    long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
    
    if (duration == 0) {
        return 999.0;  // No echo received
    }
    
    // Calculate distance in cm
    float distance = duration * 0.034 / 2.0;
    return distance;
}

float readFrontDistance() {
    return readDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
}

float readLeftDistance() {
    return readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
}

float readRightDistance() {
    return readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
}

bool isFrontWall() {
    return readFrontDistance() < WALL_THRESHOLD;
}

bool isLeftWall() {
    return readLeftDistance() < WALL_THRESHOLD;
}

bool isRightWall() {
    return readRightDistance() < WALL_THRESHOLD;
}

bool isOnLine() {
    // TCRT5000 analog reading - lower values = line detected (black on white)
    // Adjust LINE_THRESHOLD based on your surface contrast
    int sensorValue = analogRead(LINE_SENSOR_PIN);
    return sensorValue < LINE_THRESHOLD;
}

// ======================= MOTOR FUNCTIONS =======================

void initMotors() {
    // Motor control pins
    pinMode(MOTOR_STBY, OUTPUT);
    pinMode(MOTOR_AIN1, OUTPUT);
    pinMode(MOTOR_AIN2, OUTPUT);
    pinMode(MOTOR_BIN1, OUTPUT);
    pinMode(MOTOR_BIN2, OUTPUT);
    
    // Setup PWM using ESP32 Arduino Core 3.x API
    // ledcAttach(pin, freq, resolution) - channels are auto-assigned
    ledcAttach(MOTOR_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_PWMB, PWM_FREQ, PWM_RESOLUTION);
    
    // Enable motor driver
    digitalWrite(MOTOR_STBY, HIGH);
    
    Serial.println("Motors initialized");
}

void setMotorA(int speed) {
    if (speed > 0) {
        digitalWrite(MOTOR_AIN1, HIGH);
        digitalWrite(MOTOR_AIN2, LOW);
        ledcWrite(MOTOR_PWMA, min(speed, 255));  // ESP32 3.x uses pin, not channel
    } else if (speed < 0) {
        digitalWrite(MOTOR_AIN1, LOW);
        digitalWrite(MOTOR_AIN2, HIGH);
        ledcWrite(MOTOR_PWMA, min(-speed, 255));  // ESP32 3.x uses pin, not channel
    } else {
        digitalWrite(MOTOR_AIN1, LOW);
        digitalWrite(MOTOR_AIN2, LOW);
        ledcWrite(MOTOR_PWMA, 0);
    }
}

void setMotorB(int speed) {
    if (speed > 0) {
        digitalWrite(MOTOR_BIN1, HIGH);
        digitalWrite(MOTOR_BIN2, LOW);
        ledcWrite(MOTOR_PWMB, min(speed, 255));  // ESP32 3.x uses pin, not channel
    } else if (speed < 0) {
        digitalWrite(MOTOR_BIN1, LOW);
        digitalWrite(MOTOR_BIN2, HIGH);
        ledcWrite(MOTOR_PWMB, min(-speed, 255));  // ESP32 3.x uses pin, not channel
    } else {
        digitalWrite(MOTOR_BIN1, LOW);
        digitalWrite(MOTOR_BIN2, LOW);
        ledcWrite(MOTOR_PWMB, 0);
    }
}

void setMotors(int leftSpeed, int rightSpeed) {
    setMotorA(leftSpeed);   // Left motor
    setMotorB(rightSpeed);  // Right motor
}

void stopMotors() {
    setMotors(0, 0);
}

void driveForward(int speed) {
    setMotors(speed, speed);
}

void driveBackward(int speed) {
    setMotors(-speed, -speed);
}

void turnLeftInPlace(int speed) {
    setMotors(-speed, speed);
}

void turnRightInPlace(int speed) {
    setMotors(speed, -speed);
}

void lockHeading(float h) {
    targetHeading = h;
    lastError = 0;
    integral = 0;
}

void driveForwardWithCorrection() {
    float currentHeadingVal = readMPU6050Yaw();
    float error = targetHeading - currentHeadingVal;
    
    // Normalize error to -180 to 180
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    // PID calculation
    integral += error;
    float derivative = error - lastError;
    int correction = (int)(KP * error + KI * integral + KD * derivative);
    lastError = error;
    
    // Apply correction
    int leftSpeed = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;
    
    // Clamp speeds
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    setMotors(leftSpeed, rightSpeed);
}

// ======================= MPU6050 FUNCTIONS =======================

void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // Wake up
    Wire.endTransmission(true);
    
    // Configure gyroscope (±250°/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0);     // ±250°/s
    Wire.endTransmission(true);
    
    // Calibrate gyro offset
    Serial.println("Calibrating MPU6050... Keep robot still!");
    delay(1000);
    
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += readRawGyroZ();
        delay(10);
    }
    gyroZOffset = sum / 100.0;
    
    lastMPUUpdate = millis();
    Serial.println("MPU6050 initialized");
}

float readRawGyroZ() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    
    int16_t raw = (Wire.read() << 8) | Wire.read();
    return raw / 131.0;  // Convert to °/s (±250°/s range)
}

void updateYaw() {
    unsigned long now = millis();
    float dt = (now - lastMPUUpdate) / 1000.0;
    lastMPUUpdate = now;
    
    float gyroZ = readRawGyroZ() - gyroZOffset;
    
    // Integrate gyro to get yaw
    // Only update if rotation is significant (reduce drift)
    if (abs(gyroZ) > 0.5) {
        currentYaw += gyroZ * dt;
    }
    
    // Normalize to 0-360
    while (currentYaw < 0) currentYaw += 360;
    while (currentYaw >= 360) currentYaw -= 360;
}

float readMPU6050Yaw() {
    updateYaw();
    return currentYaw;
}

void calibrateMPU6050Heading() {
    // Reset yaw to 0 (current direction becomes "north")
    currentYaw = 0;
    lastMPUUpdate = millis();
}

void turnRight90() {
    float startYaw = readMPU6050Yaw();
    float targetYaw = startYaw + 90;
    if (targetYaw >= 360) targetYaw -= 360;
    
    Serial.printf("Turning right: %.1f -> %.1f\n", startYaw, targetYaw);
    
    // Turn right
    turnRightInPlace();
    
    // Wait until target reached
    while (true) {
        updateYaw();
        float error = targetYaw - currentYaw;
        
        // Normalize error
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        if (abs(error) < TURN_TOLERANCE) {
            break;
        }
        delay(10);
    }
    
    stopMotors();
    lockHeading(currentYaw);
    delay(100);
}

void turnLeft90() {
    float startYaw = readMPU6050Yaw();
    float targetYaw = startYaw - 90;
    if (targetYaw < 0) targetYaw += 360;
    
    Serial.printf("Turning left: %.1f -> %.1f\n", startYaw, targetYaw);
    
    // Turn left
    turnLeftInPlace();
    
    // Wait until target reached
    while (true) {
        updateYaw();
        float error = targetYaw - currentYaw;
        
        // Normalize error
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        if (abs(error) < TURN_TOLERANCE) {
            break;
        }
        delay(10);
    }
    
    stopMotors();
    lockHeading(currentYaw);
    delay(100);
}

// ======================= LINE SENSOR INTERRUPT =======================

void IRAM_ATTR onLineCrossed() {
    unsigned long now = millis();
    if (now - firstPulse > 500) {
        lineCount = 1;
        firstPulse = now;
    } else {
        lineCount++;
    }
    cellCrossed = true;
}

// ======================= WEBSOCKET HANDLER =======================

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            clientConnected = false;
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected!\n", num);
            clientConnected = true;
            break;
        case WStype_TEXT:
            // Handle commands from visualizer if needed
            break;
    }
}

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Maze Solver Bot v1.0 ===");
    
    // Initialize components
    initSensors();
    initMotors();
    initMPU6050();
    
    // Line sensor interrupt
    pinMode(LINE_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LINE_SENSOR_PIN), onLineCrossed, FALLING);
    
    // Initialize Q-table
    initQLearning();
    
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
    
    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started on port 81");
    
    // Ready to start
    robotState = STATE_CALIBRATING;
    Serial.println("Robot ready! Place on START marker and press boot button to begin.");
}

// ======================= MAIN LOOP =======================

void loop() {
    webSocket.loop();
    
    switch (robotState) {
        case STATE_IDLE:
            // Waiting for command
            break;
            
        case STATE_CALIBRATING:
            // Wait for button press to start
            if (digitalRead(0) == LOW) {  // Boot button pressed
                delay(200);  // Debounce
                Serial.println("Starting maze exploration...");
                episode = 1;
                resetEpisode();
                robotState = STATE_EXPLORING;
            }
            break;
            
        case STATE_EXPLORING:
            qLearningStep();
            break;
            
        case STATE_AT_GOAL:
            // Episode complete - goal reached!
            Serial.printf("Episode %d complete! Steps: %d, Reward: %.1f\n", 
                          episode, stepCount, totalReward);
            sendEpisodeComplete();
            
            // Prepare for next episode
            delay(2000);
            episode++;
            epsilon = max(epsilonMin, epsilon * epsilonDecay);
            
            // Return to start (manual or automated)
            robotState = STATE_RETURNING;
            break;
            
        case STATE_RETURNING:
            // For now, wait for manual reset
            Serial.println("Place robot back at START and press boot button");
            while (digitalRead(0) != LOW) {
                webSocket.loop();
                delay(100);
            }
            delay(200);
            resetEpisode();
            robotState = STATE_EXPLORING;
            break;
    }
}

// ======================= Q-LEARNING IMPLEMENTATION =======================

void initQLearning() {
    // Initialize Q-table with zeros
    for (int s = 0; s < NUM_STATES; s++) {
        for (int a = 0; a < NUM_ACTIONS; a++) {
            Q[s][a] = 0.0;
        }
    }
    Serial.println("Q-table initialized");
}

void qLearningStep() {
    // Get current state
    int state = getState();
    
    // Read walls at current position
    bool walls[4];
    walls[NORTH] = (heading == NORTH) ? isFrontWall() : 
                   (heading == EAST) ? isLeftWall() :
                   (heading == SOUTH) ? !isFrontWall() : isRightWall();
    walls[EAST] = (heading == EAST) ? isFrontWall() :
                  (heading == SOUTH) ? isLeftWall() :
                  (heading == WEST) ? !isFrontWall() : isRightWall();
    walls[SOUTH] = (heading == SOUTH) ? isFrontWall() :
                   (heading == WEST) ? isLeftWall() :
                   (heading == NORTH) ? !isFrontWall() : isRightWall();
    walls[WEST] = (heading == WEST) ? isFrontWall() :
                  (heading == NORTH) ? isLeftWall() :
                  (heading == EAST) ? !isFrontWall() : isRightWall();
    
    // Select action using ε-greedy policy
    int action = selectAction(state);
    
    // Check if action is valid (no wall in that direction)
    int attempts = 0;
    while (walls[action] && attempts < 4) {
        action = (action + 1) % 4;
        attempts++;
    }
    
    if (attempts >= 4) {
        // Dead end - all directions blocked
        Serial.println("Dead end!");
        totalReward -= 50;
        robotState = STATE_RETURNING;
        return;
    }
    
    // Execute the action
    executeAction(action);
    
    // Wait for cell transition
    cellCrossed = false;
    unsigned long startTime = millis();
    while (!cellCrossed && (millis() - startTime < 5000)) {
        driveForwardWithCorrection();
        delay(10);
    }
    stopMotors();
    
    // Check line markers
    checkMarkers();
    
    // Get reward and new state
    float reward = getReward();
    totalReward += reward;
    int nextState = getState();
    
    // Update Q-value
    updateQValue(state, action, nextState, reward);
    
    // Send telemetry
    stepCount++;
    sendTelemetry();
    
    // Small delay for stability
    delay(200);
    
    // Check if goal reached
    if (isAtGoal) {
        robotState = STATE_AT_GOAL;
    }
}

int selectAction(int state) {
    // ε-greedy action selection
    if (random(100) < epsilon * 100) {
        // Random action (exploration)
        return random(4);
    } else {
        // Best action from Q-table (exploitation)
        int bestAction = 0;
        float bestValue = Q[state][0];
        for (int a = 1; a < NUM_ACTIONS; a++) {
            if (Q[state][a] > bestValue) {
                bestValue = Q[state][a];
                bestAction = a;
            }
        }
        return bestAction;
    }
}

void executeAction(int action) {
    // Calculate turn needed
    int turnAmount = (action - heading + 4) % 4;
    
    if (turnAmount == 1) {
        // Turn right 90°
        turnRight90();
    } else if (turnAmount == 2) {
        // Turn around 180°
        turnRight90();
        turnRight90();
    } else if (turnAmount == 3) {
        // Turn left 90°
        turnLeft90();
    }
    // turnAmount == 0 means already facing correct direction
    
    heading = action;
}

float getReward() {
    if (isAtGoal) {
        return 100.0;  // Big reward for goal
    }
    if (isFrontWall() && readFrontDistance() < 5) {
        return -10.0;  // Punishment for hitting wall
    }
    return -1.0;  // Small step penalty
}

void updateQValue(int state, int action, int nextState, float reward) {
    // Find max Q-value for next state
    float maxNextQ = Q[nextState][0];
    for (int a = 1; a < NUM_ACTIONS; a++) {
        if (Q[nextState][a] > maxNextQ) {
            maxNextQ = Q[nextState][a];
        }
    }
    
    // Q-learning update rule
    Q[state][action] += alpha * (reward + gamma_rl * maxNextQ - Q[state][action]);
}

int getState() {
    // Convert (x, y) to state index
    // Handle negative positions by offsetting
    int x = currentX + MAZE_SIZE;  // Offset to handle negatives
    int y = currentY + MAZE_SIZE;
    return (y % MAZE_SIZE) * MAZE_SIZE + (x % MAZE_SIZE);
}

void updatePositionBasedOnHeading() {
    switch (heading) {
        case NORTH: currentY--; break;
        case SOUTH: currentY++; break;
        case EAST:  currentX++; break;
        case WEST:  currentX--; break;
    }
}

void checkMarkers() {
    delay(100);  // Wait for line count to settle
    
    if (lineCount == 1) {
        // Single line = cell boundary
        updatePositionBasedOnHeading();
    } else if (lineCount == 2) {
        // Double line = START
        currentX = 0;
        currentY = 0;
        isAtStart = true;
        Serial.println("START marker detected!");
    } else if (lineCount >= 3) {
        // Triple line = GOAL
        isAtGoal = true;
        Serial.println("GOAL reached!");
    }
    
    lineCount = 0;
}

void resetEpisode() {
    currentX = 0;
    currentY = 0;
    heading = NORTH;  // Assume starting facing north (into maze)
    stepCount = 0;
    totalReward = 0;
    isAtGoal = false;
    isAtStart = false;
    lineCount = 0;
    
    // Lock initial heading from MPU6050
    calibrateMPU6050Heading();
    
    Serial.printf("Episode %d starting...\n", episode);
}

// ======================= TELEMETRY =======================

void sendTelemetry() {
    if (!clientConnected) return;
    
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry";
    
    JsonObject pos = doc.createNestedObject("position");
    pos["x"] = currentX;
    pos["y"] = currentY;
    
    doc["heading"] = heading * 90;  // Convert to degrees
    
    JsonObject walls = doc.createNestedObject("walls");
    walls["front"] = isFrontWall();
    walls["left"] = isLeftWall();
    walls["right"] = isRightWall();
    
    doc["episode"] = episode;
    doc["step"] = stepCount;
    doc["reward"] = totalReward;
    doc["isGoal"] = isAtGoal;
    
    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
}

void sendEpisodeComplete() {
    if (!clientConnected) return;
    
    StaticJsonDocument<128> doc;
    doc["type"] = "episode_complete";
    doc["episode"] = episode;
    doc["totalSteps"] = stepCount;
    doc["totalReward"] = totalReward;
    doc["solved"] = isAtGoal;
    
    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
}
