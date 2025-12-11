/*
 * Maze Solver Bot - ESP32 Q-Learning Firmware (L298N Version)
 * * Hardware:
 * - ESP32 DevKit
 * - 3x HC-SR04 Ultrasonic Sensors
 * - 1x TCRT5000 Line Sensor
 * - MPU6050 IMU
 * - L298N Motor Driver (Replaces TB6612FNG)
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ======================= CONFIGURATION =======================

// WiFi Credentials
const char* WIFI_SSID = "POCO X7 Pro";
const char* WIFI_PASSWORD = "12345678";

// ======================= PIN DEFINITIONS =======================

// HC-SR04 Ultrasonic Sensors
#define FRONT_TRIG_PIN 14
#define FRONT_ECHO_PIN 35
#define LEFT_TRIG_PIN 18
#define LEFT_ECHO_PIN 13
#define RIGHT_TRIG_PIN 16
#define RIGHT_ECHO_PIN 34

// TCRT5000 Line Sensor (facing down)
#define LINE_SENSOR_PIN 4
#define LINE_THRESHOLD 2048

// L298N Motor Driver
// Left Motor
#define MOTOR_IN1 19
#define MOTOR_IN2 21
#define MOTOR_ENA 25 

// Right Motor
#define MOTOR_IN3 22
#define MOTOR_IN4 23
#define MOTOR_ENB 26  // CHANGED: Moved to Pin 4 (Old STBY) to avoid conflict with SDA (21)

// MPU6050 (I2C)
#define MPU6050_SDA 32 // Fixed: SDA is now exclusive to this pin
#define MPU6050_SCL 33

// ======================= THRESHOLDS =======================

// Wall detection threshold (cm)
#define WALL_THRESHOLD 5

// Motor speed (0-255)
// Increased speeds slightly because L298N has a 2V voltage drop
#define BASE_SPEED 100 
#define TURN_SPEED 120

// PID constants for straight driving
#define KP 2.0
#define KI 0.0
#define KD 0.5

// Turn tolerance (degrees)
#define TURN_TOLERANCE 3

// ======================= MOTOR CONSTANTS =======================

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ======================= MPU6050 CONSTANTS =======================

#define MPU6050_ADDR 0x68

// ======================= Q-LEARNING =======================

#define MAZE_SIZE 5
#define NUM_STATES (MAZE_SIZE * MAZE_SIZE)
#define NUM_ACTIONS 4 

float Q[NUM_STATES][NUM_ACTIONS];

float alpha = 0.3;      // Learning rate
float gamma_rl = 0.9;   // Discount factor
float epsilon = 0.5;    // Exploration rate
float epsilonDecay = 0.95;
float epsilonMin = 0.1;

// ======================= POSITION TRACKING =======================

int currentX = 0;
int currentY = 0;
int heading = 0; // 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST

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

// ======================= WEBSOCKET CLIENT =======================

// Railway bridge server (UPDATE THIS after deploying!)
const char* WS_SERVER_HOST = "botvisualizer-production.up.railway.app";  // Replace with your Railway URL

WebSocketsClient webSocket;
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
void setMotorLeft(int speed);
void setMotorRight(int speed);
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
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    
    if (duration == 0) return 999.0;
    
    return duration * 0.034 / 2.0;
}

float readFrontDistance() { return readDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN); }
float readLeftDistance() { return readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN); }
float readRightDistance() { return readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN); }

bool isFrontWall() { return readFrontDistance() < WALL_THRESHOLD; }
bool isLeftWall() { return readLeftDistance() < WALL_THRESHOLD; }
bool isRightWall() { return readRightDistance() < WALL_THRESHOLD; }

bool isOnLine() {
    int sensorValue = analogRead(LINE_SENSOR_PIN);
    return sensorValue < LINE_THRESHOLD;
}

// ======================= MOTOR FUNCTIONS (L298N) =======================

void initMotors() {
    // Motor control pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    
    // Setup PWM
    ledcAttach(MOTOR_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_ENB, PWM_FREQ, PWM_RESOLUTION);
    
    Serial.println("L298N Motors initialized");
}

void setMotorLeft(int speed) {
    if (speed > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_ENA, min(speed, 255));
    } else if (speed < 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        ledcWrite(MOTOR_ENA, min(-speed, 255));
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_ENA, 0);
    }
}

void setMotorRight(int speed) {
    if (speed > 0) {
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        ledcWrite(MOTOR_ENB, min(speed, 255));
    } else if (speed < 0) {
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, HIGH);
        ledcWrite(MOTOR_ENB, min(-speed, 255));
    } else {
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, LOW);
        ledcWrite(MOTOR_ENB, 0);
    }
}

void setMotors(int leftSpeed, int rightSpeed) {
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
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
    
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    integral += error;
    float derivative = error - lastError;
    int correction = (int)(KP * error + KI * integral + KD * derivative);
    lastError = error;
    
    int leftSpeed = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    setMotors(leftSpeed, rightSpeed);
}

// ======================= MPU6050 FUNCTIONS =======================

void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0);
    Wire.endTransmission(true);
    
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
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    
    int16_t raw = (Wire.read() << 8) | Wire.read();
    return raw / 131.0;
}

void updateYaw() {
    unsigned long now = millis();
    float dt = (now - lastMPUUpdate) / 1000.0;
    lastMPUUpdate = now;
    
    float gyroZ = readRawGyroZ() - gyroZOffset;
    
    if (abs(gyroZ) > 0.5) {
        currentYaw += gyroZ * dt;
    }
    
    while (currentYaw < 0) currentYaw += 360;
    while (currentYaw >= 360) currentYaw -= 360;
}

float readMPU6050Yaw() {
    updateYaw();
    return currentYaw;
}

void calibrateMPU6050Heading() {
    currentYaw = 0;
    lastMPUUpdate = millis();
}

void turnRight90() {
    float startYaw = readMPU6050Yaw();
    float targetYaw = startYaw + 90;
    if (targetYaw >= 360) targetYaw -= 360;
    
    Serial.printf("Turning right: %.1f -> %.1f\n", startYaw, targetYaw);
    
    turnRightInPlace();
    
    while (true) {
        updateYaw();
        float error = targetYaw - currentYaw;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        if (abs(error) < TURN_TOLERANCE) break;
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
    
    turnLeftInPlace();
    
    while (true) {
        updateYaw();
        float error = targetYaw - currentYaw;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        if (abs(error) < TURN_TOLERANCE) break;
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

// ======================= WEBSOCKET CLIENT HANDLER =======================

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected!");
            clientConnected = false;
            break;
        case WStype_CONNECTED:
            Serial.println("WebSocket connected to bridge server!");
            clientConnected = true;
            // Send identification message
            {
                StaticJsonDocument<64> doc;
                doc["type"] = "identify";
                doc["client"] = "esp32";
                String json;
                serializeJson(doc, json);
                webSocket.sendTXT(json);
                Serial.println("Sent identification to bridge");
            }
            break;
        case WStype_TEXT:
            // Handle commands from browser (if any)
            break;
        case WStype_ERROR:
            Serial.println("WebSocket error!");
            break;
    }
}

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Maze Solver Bot v1.0 (L298N) ===");
    
    initSensors();
    initMotors();
    initMPU6050();
    
    pinMode(LINE_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LINE_SENSOR_PIN), onLineCrossed, FALLING);
    
    initQLearning();
    
    // Static IP configuration for your network
    IPAddress local_IP(172, 28, 182, 100);   // ESP32's static IP
    IPAddress gateway(172, 28, 182, 3);      // Your network gateway
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(8, 8, 8, 8);               // Google DNS
    
    if (!WiFi.config(local_IP, gateway, subnet, dns)) {
        Serial.println("Static IP configuration failed!");
    }
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected with static IP: ");
    Serial.println(WiFi.localIP());
    
    // Connect to Railway WebSocket bridge server (Secure WebSocket)
    webSocket.beginSSL(WS_SERVER_HOST, 443, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    Serial.printf("Connecting to WebSocket bridge at wss://%s\n", WS_SERVER_HOST);
    
    robotState = STATE_CALIBRATING;
    Serial.println("Robot ready! Place on START and press boot button.");
}

// ======================= MAIN LOOP =======================

void loop() {
    webSocket.loop();
    
    switch (robotState) {
        case STATE_IDLE:
            break;
            
        case STATE_CALIBRATING:
            if (digitalRead(0) == LOW) { // Boot button
                delay(200);
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
            Serial.printf("Episode %d complete! Steps: %d, Reward: %.1f\n", 
                          episode, stepCount, totalReward);
            sendEpisodeComplete();
            delay(2000);
            episode++;
            epsilon = max(epsilonMin, epsilon * epsilonDecay);
            robotState = STATE_RETURNING;
            break;
            
        case STATE_RETURNING:
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
    for (int s = 0; s < NUM_STATES; s++) {
        for (int a = 0; a < NUM_ACTIONS; a++) {
            Q[s][a] = 0.0;
        }
    }
    Serial.println("Q-table initialized");
}

void qLearningStep() {
    int state = getState();
    
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
    
    int action = selectAction(state);
    int attempts = 0;
    while (walls[action] && attempts < 4) {
        action = (action + 1) % 4;
        attempts++;
    }
    
    if (attempts >= 4) {
        Serial.println("Dead end!");
        totalReward -= 50;
        robotState = STATE_RETURNING;
        return;
    }
    
    executeAction(action);
    
    cellCrossed = false;
    unsigned long startTime = millis();
    while (!cellCrossed && (millis() - startTime < 5000)) {
        driveForwardWithCorrection();
        delay(10);
    }
    stopMotors();
    
    checkMarkers();
    
    float reward = getReward();
    totalReward += reward;
    int nextState = getState();
    
    updateQValue(state, action, nextState, reward);
    
    stepCount++;
    sendTelemetry();
    
    delay(200);
    
    if (isAtGoal) robotState = STATE_AT_GOAL;
}

int selectAction(int state) {
    if (random(100) < epsilon * 100) {
        return random(4);
    } else {
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
    int turnAmount = (action - heading + 4) % 4;
    
    if (turnAmount == 1) turnRight90();
    else if (turnAmount == 2) { turnRight90(); turnRight90(); }
    else if (turnAmount == 3) turnLeft90();
    
    heading = action;
}

float getReward() {
    if (isAtGoal) return 100.0;
    if (isFrontWall() && readFrontDistance() < 5) return -10.0;
    return -1.0;
}

void updateQValue(int state, int action, int nextState, float reward) {
    float maxNextQ = Q[nextState][0];
    for (int a = 1; a < NUM_ACTIONS; a++) {
        if (Q[nextState][a] > maxNextQ) {
            maxNextQ = Q[nextState][a];
        }
    }
    Q[state][action] += alpha * (reward + gamma_rl * maxNextQ - Q[state][action]);
}

int getState() {
    int x = currentX + MAZE_SIZE;
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
    delay(100);
    if (lineCount == 1) updatePositionBasedOnHeading();
    else if (lineCount == 2) {
        currentX = 0; currentY = 0; isAtStart = true;
        Serial.println("START marker detected!");
    } else if (lineCount >= 3) {
        isAtGoal = true;
        Serial.println("GOAL reached!");
    }
    lineCount = 0;
}

void resetEpisode() {
    currentX = 0; currentY = 0;
    heading = NORTH;
    stepCount = 0; totalReward = 0;
    isAtGoal = false; isAtStart = false; lineCount = 0;
    calibrateMPU6050Heading();
    Serial.printf("Episode %d starting...\n", episode);
}

void sendTelemetry() {
    if (!clientConnected) return;
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry";
    JsonObject pos = doc.createNestedObject("position");
    pos["x"] = currentX; pos["y"] = currentY;
    doc["heading"] = heading * 90;
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
    webSocket.sendTXT(json);
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
    webSocket.sendTXT(json);
}