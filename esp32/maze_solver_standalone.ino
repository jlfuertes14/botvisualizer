/*
 * Maze Solver Bot - Final Version (Optimistic Q-Learning + Re-Run Proof)
 * Logic:
 * 1. "Discovery Run": Robot assumes empty map. Moves, detects walls, updates map, re-plans in RAM.
 * 2. "Proof Run": Press Boot Button at goal. Robot resets position but KEEPS memory.
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// EEPROM Layout: [Magic(4)] [MapData(25)] = 29 bytes
#define EEPROM_SIZE 64
#define EEPROM_MAGIC 0x4D415A45  // "MAZE" in hex

// ======================= CONFIGURATION =======================

const char* WIFI_SSID = "POCO X7 Pro";
const char* WIFI_PASSWORD = "12345678";
const char* WS_SERVER_HOST = "botvisualizer-production.up.railway.app"; 

// ======================= PIN DEFINITIONS =======================

// HC-SR04 Ultrasonic Sensors
#define FRONT_TRIG_PIN 14
#define FRONT_ECHO_PIN 35
#define LEFT_TRIG_PIN 18
#define LEFT_ECHO_PIN 13
#define RIGHT_TRIG_PIN 16
#define RIGHT_ECHO_PIN 34

// TCRT5000 Line Sensor
#define LINE_SENSOR_PIN 4
#define LINE_THRESHOLD 2048 // Adjust based on your black tape readings

// L298N Motor Driver
#define MOTOR_IN1 19
#define MOTOR_IN2 21
#define MOTOR_ENA 25 
#define MOTOR_IN3 22
#define MOTOR_IN4 23
#define MOTOR_ENB 26 

// MPU6050
#define MPU6050_SDA 32 
#define MPU6050_SCL 33
#define MPU6050_ADDR 0x68

#define STATUS_LED 2 // Onboard LED

// ======================= TUNING CONSTANTS =======================

// Motor Physics
#define BASE_SPEED 180       // High cruising speed to prevent stalling
#define TURN_SPEED 200       // High torque for turns
#define KICK_TIME 40         // Milliseconds of FULL POWER (255) at start
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Navigation
#define MAZE_SIZE 5
#define MAX_CELL_TIME 2000   // Timeout if line not found (ms)
#define WALL_DIST_CM 15.0    // Distance to mark a grid cell as "Wall"

// Q-Learning Hyperparameters
#define NUM_STATES (MAZE_SIZE * MAZE_SIZE)
#define NUM_ACTIONS 4 
float Q[NUM_STATES][NUM_ACTIONS];
float alpha = 0.5;    // High learning rate (learn immediately)
float gamma_rl = 0.9; // Future reward importance

// Adaptive Training - reduce episodes when map is stable
#define MAX_VIRTUAL_EPISODES 2000
#define MIN_VIRTUAL_EPISODES 500
int virtualEpisodeCount = MAX_VIRTUAL_EPISODES;
bool mapChangedThisStep = false;

// Wall-Following PID Constants
#define PID_KP 3.0
#define PID_KI 0.0
#define PID_KD 1.0
#define WALL_FOLLOW_DIST 8.0  // Target distance from wall (cm)
#define WALL_PRESENT_DIST 20.0  // Distance to consider wall present
float pidLastError = 0;
float pidIntegral = 0;

// ======================= GLOBAL VARIABLES =======================

int currentX = 0;
int currentY = 0;
int GOAL_X = 4;
int GOAL_Y = 4;
int heading = 0; // 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST

// THE MEMORY MAP (0=Empty, 1=Wall)
int internalMap[MAZE_SIZE][MAZE_SIZE] = {0}; 

enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

enum RobotState { 
  STATE_IDLE, 
  STATE_THINKING, 
  STATE_MOVING, 
  STATE_AT_GOAL 
};
RobotState robotState = STATE_IDLE;

// Line Detection Interrupt Vars
volatile bool cellCrossed = false;
volatile unsigned long lastPulse = 0;

WebSocketsClient webSocket;
float gyroZOffset = 0;
float currentYaw = 0;
unsigned long lastMPUUpdate = 0;

// Sensor telemetry interval
unsigned long lastSensorUpdate = 0;
#define SENSOR_UPDATE_INTERVAL 50 // Send sensor data every 50ms (20Hz)

// ======================= FUNCTION DECLARATIONS =======================
void initMotors();
void setMotors(int L, int R);
void stopMotors();
void kickStartAndDriveOneCell();
void driveWithPID();
void turnRight90();
void turnLeft90();
float readDistance(int trig, int echo);
void initMPU6050();
void updateYaw();
void trainQTable();
void updateMapAndPlan();
void executeNextStep();
void sendTelemetry();
void sendSensorData();
void saveMapToEEPROM();
void loadMapFromEEPROM();
void IRAM_ATTR onLineCrossed();

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP); // Boot Button for Re-Run
    
    // Initialize EEPROM and load saved map
    EEPROM.begin(EEPROM_SIZE);
    loadMapFromEEPROM();
    
    initSensors();
    initMotors();
    initMPU6050();

    // WiFi
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(" Connected!");

    // WebSocket
    webSocket.beginSSL(WS_SERVER_HOST, 443, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    
    Serial.println("READY. Waiting for Start...");
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
    webSocket.loop();
    
    // Periodically send sensor data to website (regardless of state)
    if (millis() - lastSensorUpdate > SENSOR_UPDATE_INTERVAL) {
        sendSensorData();
        lastSensorUpdate = millis();
    }
    
    switch (robotState) {
        case STATE_IDLE:
            // Blink slowly waiting for start
            if (millis() % 1000 < 500) digitalWrite(STATUS_LED, HIGH);
            else digitalWrite(STATUS_LED, LOW);
            break;
            
        case STATE_THINKING:
            stopMotors();
            digitalWrite(STATUS_LED, HIGH); // LED ON = Processing
            
            // 1. Read Sensors & Update Map
            updateMapAndPlan();
            
            // 2. Transition to Moving
            robotState = STATE_MOVING;
            digitalWrite(STATUS_LED, LOW); 
            break;
            
        case STATE_MOVING:
            executeNextStep();
            
            if (currentX == GOAL_X && currentY == GOAL_Y) {
                robotState = STATE_AT_GOAL;
                sendTelemetry(); 
            } else {
                robotState = STATE_THINKING; // Check sensors at next cell
            }
            break;
            
        case STATE_AT_GOAL:
            stopMotors();
            // Fast celebrate blink
            if (millis() % 300 < 150) digitalWrite(STATUS_LED, HIGH);
            else digitalWrite(STATUS_LED, LOW);
            
            // === RE-RUN LOGIC ===
            if (digitalRead(0) == LOW) { // Boot Button Pressed
                Serial.println("RE-RUN: Resetting Position (Keeping Memory)");
                
                currentX = 0; 
                currentY = 0;
                heading = NORTH; // PLACE ROBOT FACING NORTH!
                
                // Retrain once to generate full path from start
                trainQTable();
                
                delay(1000); // Wait for hand removal
                robotState = STATE_THINKING;
            }
            break;
    }
}

// ======================= OPTIMISTIC Q-LEARNING CORE =======================

void updateMapAndPlan() {
    // 1. READ SENSORS
    float distFront = readDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    float distLeft = readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float distRight = readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    
    bool wallFront = (distFront < WALL_DIST_CM);
    bool wallLeft = (distLeft < WALL_DIST_CM);
    bool wallRight = (distRight < WALL_DIST_CM);

    // 2. CALCULATE GRID COORDINATES OF NEIGHBORS
    int fx = currentX, fy = currentY; 
    int lx = currentX, ly = currentY; 
    int rx = currentX, ry = currentY; 
    
    if (heading == NORTH) { fy--; lx--; rx++; }
    else if (heading == EAST) { fx++; ly--; ry++; }
    else if (heading == SOUTH) { fy++; lx++; rx--; }
    else if (heading == WEST) { fx--; ly++; ry--; }

    bool mapChanged = false;

    // 3. MARK WALLS IN MEMORY
    auto markWall = [&](int x, int y) {
        if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
            if (internalMap[y][x] == 0) {
                internalMap[y][x] = 1; // Mark as Wall
                mapChanged = true;
            }
        }
    };

    if (wallFront) markWall(fx, fy);
    if (wallLeft) markWall(lx, ly);
    if (wallRight) markWall(rx, ry);

    // 4. UPDATE ADAPTIVE TRAINING PARAMETERS
    mapChangedThisStep = mapChanged;
    if (mapChanged) {
        virtualEpisodeCount = MAX_VIRTUAL_EPISODES;  // Full training on new info
        saveMapToEEPROM();  // Persist to EEPROM
        Serial.println("Map updated and saved to EEPROM");
    } else {
        // Reduce training if map is stable
        virtualEpisodeCount = max(MIN_VIRTUAL_EPISODES, virtualEpisodeCount - 100);
    }

    // 5. RE-TRAIN Q-TABLE (Instant Virtual Learning)
    trainQTable(); 
}

// Virtual Q-Learning (Runs in RAM with Adaptive Episode Count)
void trainQTable() {
    // Reset Q-table temporarily to avoid getting stuck in old loops
    memset(Q, 0, sizeof(Q));
    
    unsigned long trainStart = millis();
    
    // Run adaptive number of virtual episodes
    for (int ep = 0; ep < virtualEpisodeCount; ep++) {
        int vx = currentX, vy = currentY; // Start simulation from current spot
        
        for (int step = 0; step < 30; step++) { 
            if (vx == GOAL_X && vy == GOAL_Y) break;
            
            int state = vy * MAZE_SIZE + vx;
            
            // Epsilon Greedy (Virtual)
            int action;
            if (random(100) < 10) action = random(4); 
            else {
                // Exploit Max Q
                float maxQ = -9999;
                action = 0;
                for(int a=0; a<4; a++) if(Q[state][a] > maxQ) { maxQ = Q[state][a]; action = a; }
            }
            
            // Virtual Move
            int nx = vx, ny = vy;
            if (action == NORTH) ny--;
            else if (action == SOUTH) ny++;
            else if (action == EAST) nx++;
            else if (action == WEST) nx--;
            
            // Reward System
            float reward = -1; 
            float maxNextQ = 0;
            
            // Wall Check (Using Internal Map)
            bool hitWall = false;
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) hitWall = true;
            else if (internalMap[ny][nx] == 1) hitWall = true;
            
            if (hitWall) {
                reward = -50; // Punishment
                nx = vx; ny = vy; // Stay put
            } else if (nx == GOAL_X && ny == GOAL_Y) {
                reward = 100; // Goal
            }
            
            // Bellman Update
            int nextState = ny * MAZE_SIZE + nx;
            for(int a=0; a<4; a++) if(Q[nextState][a] > maxNextQ) maxNextQ = Q[nextState][a];
            Q[state][action] += alpha * (reward + gamma_rl * maxNextQ - Q[state][action]);
            
            vx = nx; vy = ny;
        }
    }
    
    Serial.printf("Q-Table trained: %d episodes in %ldms\n", virtualEpisodeCount, millis() - trainStart);
}

// ======================= MOVEMENT & EXECUTION =======================

void executeNextStep() {
    int state = currentY * MAZE_SIZE + currentX;
    
    // 1. Get Best Action
    int bestAction = 0;
    float maxQ = -9999;
    for (int a = 0; a < 4; a++) {
        if (Q[state][a] > maxQ) { maxQ = Q[state][a]; bestAction = a; }
    }
    
    // 2. Turn to Face Action
    int turnDiff = (bestAction - heading + 4) % 4;
    
    if (turnDiff == 1) turnRight90();
    else if (turnDiff == 2) { turnRight90(); turnRight90(); }
    else if (turnDiff == 3) turnLeft90();
    
    heading = bestAction;
    delay(200); // Stabilize before drive

    // 3. Drive 1 Cell
    kickStartAndDriveOneCell();
    
    // 4. Update Position
    if (heading == NORTH) currentY--;
    else if (heading == SOUTH) currentY++;
    else if (heading == EAST) currentX++;
    else if (heading == WEST) currentX--;
    
    sendTelemetry();
}

void kickStartAndDriveOneCell() {
    cellCrossed = false; // Reset interrupt flag
    pidLastError = 0;
    pidIntegral = 0;
    
    // PHASE 1: KICKSTART (Anti-Stall)
    setMotors(255, 255);
    delay(KICK_TIME);
    
    // PHASE 2: CRUISE WITH WALL-FOLLOWING PID UNTIL LINE OR TIMEOUT
    unsigned long startTime = millis();
    
    while (!cellCrossed && (millis() - startTime < MAX_CELL_TIME)) {
        driveWithPID();
        delay(20);  // 50Hz control loop
    }
    
    // PHASE 3: BRAKE
    setMotors(-255, -255);
    delay(50);
    stopMotors();
}

// Wall-Following PID for smoother driving
void driveWithPID() {
    float leftDist = readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float rightDist = readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    
    float error = 0;
    
    bool leftWall = (leftDist < WALL_PRESENT_DIST);
    bool rightWall = (rightDist < WALL_PRESENT_DIST);
    
    if (leftWall && rightWall) {
        // Both walls: center between them
        error = leftDist - rightDist;
    } else if (leftWall) {
        // Only left wall: maintain distance from it
        error = (leftDist - WALL_FOLLOW_DIST) * 2;
    } else if (rightWall) {
        // Only right wall: maintain distance from it
        error = (WALL_FOLLOW_DIST - rightDist) * 2;
    } else {
        // No walls: drive straight
        error = 0;
    }
    
    // PID calculation
    pidIntegral += error;
    pidIntegral = constrain(pidIntegral, -50, 50);  // Anti-windup
    float derivative = error - pidLastError;
    pidLastError = error;
    
    float correction = (PID_KP * error) + (PID_KI * pidIntegral) + (PID_KD * derivative);
    correction = constrain(correction, -60, 60);  // Limit correction
    
    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;
    
    setMotors(constrain(leftSpeed, 100, 255), constrain(rightSpeed, 100, 255));
}

// ======================= MOTOR & SENSOR HELPERS =======================

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
    
    ledcAttach(MOTOR_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_ENB, PWM_FREQ, PWM_RESOLUTION);
}

void setMotors(int L, int R) {
    // Left
    if (L > 0) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); }
    else { digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH); L = -L; }
    ledcWrite(MOTOR_ENA, constrain(L, 0, 255));

    // Right
    if (R > 0) { digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW); }
    else { digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH); R = -R; }
    ledcWrite(MOTOR_ENB, constrain(R, 0, 255));
}

void stopMotors() { setMotors(0, 0); }

void turnRight90() {
    setMotors(255, -255); delay(KICK_TIME); // Kick
    setMotors(TURN_SPEED, -TURN_SPEED);
    
    // Turn using IMU
    float startYaw = currentYaw;
    float target = startYaw + 90;
    if (target >= 360) target -= 360;
    
    unsigned long s = millis();
    while(millis() - s < 2000) {
        updateYaw();
        float diff = target - currentYaw;
        if(diff > 180) diff -= 360;
        if(diff < -180) diff += 360;
        if(abs(diff) < 3) break; 
    }
    stopMotors();
}

void turnLeft90() {
    setMotors(-255, 255); delay(KICK_TIME); // Kick
    setMotors(-TURN_SPEED, TURN_SPEED);
    
    float startYaw = currentYaw;
    float target = startYaw - 90;
    if (target < 0) target += 360;
    
    unsigned long s = millis();
    while(millis() - s < 2000) {
        updateYaw();
        float diff = target - currentYaw;
        if(diff > 180) diff -= 360;
        if(diff < -180) diff += 360;
        if(abs(diff) < 3) break; 
    }
    stopMotors();
}

float readDistance(int trig, int echo) {
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long dur = pulseIn(echo, HIGH, 5000); // 5ms timeout (~80cm)
    if (dur == 0) return 999;
    return dur * 0.034 / 2;
}

// IMU Helpers
void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
    // Calibration
    float sum = 0;
    for (int i=0; i<100; i++) {
        Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 2, true);
        int16_t raw = (Wire.read() << 8) | Wire.read();
        sum += raw/131.0; delay(5);
    }
    gyroZOffset = sum / 100.0;
}

void updateYaw() {
    unsigned long now = millis();
    float dt = (now - lastMPUUpdate) / 1000.0;
    lastMPUUpdate = now;
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    float gyroZ = (raw/131.0) - gyroZOffset;
    if(abs(gyroZ) > 1.0) currentYaw += gyroZ * dt;
    while(currentYaw < 0) currentYaw += 360;
    while(currentYaw >= 360) currentYaw -= 360;
}

void IRAM_ATTR onLineCrossed() {
    unsigned long now = millis();
    if (now - lastPulse > 200) { // Debounce
        cellCrossed = true;
        lastPulse = now;
    }
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_CONNECTED) {
        Serial.println("WebSocket connected");
        // Identify as ESP32
        StaticJsonDocument<128> identifyDoc;
        identifyDoc["type"] = "identify";
        identifyDoc["client"] = "esp32";
        String identifyJson;
        serializeJson(identifyDoc, identifyJson);
        webSocket.sendTXT(identifyJson);
    }
    else if (type == WStype_TEXT) {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, payload);
        
        if (doc["type"] == "start") {
            // Reset map and state on "Start" command
            memset(internalMap, 0, sizeof(internalMap));
            
            // Get start and goal positions from website
            currentX = doc["start"]["x"];
            currentY = doc["start"]["y"];
            GOAL_X = doc["goal"]["x"];
            GOAL_Y = doc["goal"]["y"];
            
            heading = NORTH; // Reset heading
            
            Serial.printf("START received: (%d,%d) -> (%d,%d)\n", currentX, currentY, GOAL_X, GOAL_Y);
            
            robotState = STATE_THINKING;
        }
        else if (doc["type"] == "stop") {
            // Stop command from website
            stopMotors();
            robotState = STATE_IDLE;
            Serial.println("STOP received");
        }
    }
    else if (type == WStype_DISCONNECTED) {
        Serial.println("WebSocket disconnected");
        stopMotors();
        robotState = STATE_IDLE;
    }
}

void sendTelemetry() {
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry";
    doc["x"] = currentX;
    doc["y"] = currentY;
    doc["heading"] = heading;
    
    // Also include current state
    const char* stateNames[] = {"idle", "thinking", "moving", "goal"};
    doc["state"] = stateNames[robotState];
    
    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
}

// Send sensor readings to website for display
void sendSensorData() {
    // Read all sensors
    float distFront = readDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    float distLeft = readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float distRight = readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    
    updateYaw();
    
    StaticJsonDocument<384> doc;
    doc["type"] = "sensors";
    
    // Ultrasonic distances
    doc["front"] = distFront;
    doc["left"] = distLeft;
    doc["right"] = distRight;
    
    // IMU data
    doc["yaw"] = currentYaw;
    
    // Position and heading
    doc["x"] = currentX;
    doc["y"] = currentY;
    doc["heading"] = heading;
    
    // Robot state
    const char* stateNames[] = {"idle", "thinking", "moving", "goal"};
    doc["state"] = stateNames[robotState];
    
    // Wall detection
    doc["wallFront"] = (distFront < WALL_DIST_CM);
    doc["wallLeft"] = (distLeft < WALL_DIST_CM);
    doc["wallRight"] = (distRight < WALL_DIST_CM);
    
    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
}

// ======================= EEPROM PERSISTENCE =======================

void saveMapToEEPROM() {
    int addr = 0;
    
    // Write magic number
    uint32_t magic = EEPROM_MAGIC;
    EEPROM.put(addr, magic);
    addr += sizeof(magic);
    
    // Write internal map (25 bytes for 5x5)
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            EEPROM.write(addr++, (uint8_t)internalMap[y][x]);
        }
    }
    
    EEPROM.commit();
    Serial.printf("Map saved to EEPROM (%d bytes)\n", addr);
}

void loadMapFromEEPROM() {
    int addr = 0;
    
    // Check magic number
    uint32_t magic;
    EEPROM.get(addr, magic);
    addr += sizeof(magic);
    
    if (magic != EEPROM_MAGIC) {
        Serial.println("No valid map in EEPROM - starting fresh");
        memset(internalMap, 0, sizeof(internalMap));
        return;
    }
    
    // Read internal map
    int wallCount = 0;
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            internalMap[y][x] = EEPROM.read(addr++);
            if (internalMap[y][x] == 1) wallCount++;
        }
    }
    
    Serial.printf("Map loaded from EEPROM (%d walls)\n", wallCount);
    
    // If map has data, reduce initial training
    if (wallCount > 0) {
        virtualEpisodeCount = MIN_VIRTUAL_EPISODES;
        Serial.println("Using previous map knowledge - reduced training");
    }
}
