/*
 * Maze Solver Bot - Flood Fill Version
 * Logic:
 * 1. "Discovery Run": Robot explores, detects walls, updates map, uses Flood Fill for optimal path.
 * 2. "Speed Run": Press Boot Button at goal. Robot resets position but KEEPS wall memory.
 * 
 * Flood Fill is deterministic and always finds the shortest path!
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// EEPROM Layout: [Magic(4)] [WallData(MAZE_SIZE*MAZE_SIZE*4 bytes for walls)]
#define EEPROM_SIZE 128
#define EEPROM_MAGIC 0x464C4F44  // "FLOD" in hex

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
#define LINE_THRESHOLD 1500

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

#define STATUS_LED 2

// ======================= TUNING CONSTANTS =======================

#define BASE_SPEED 120
#define TURN_SPEED 140
#define KICK_TIME 40

#define MAZE_SIZE 5
#define MAX_CELL_TIME 2000
#define WALL_DIST_CM 7.0



// ======================= GLOBAL VARIABLES =======================

int currentX = 0;
int currentY = 0;
int GOAL_X = 4;
int GOAL_Y = 4;
int heading = 0; // 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST

enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Flood Fill distance values
int floodMap[MAZE_SIZE][MAZE_SIZE];

// Wall data: for each cell, store walls in 4 directions
// Bit 0 = North wall, Bit 1 = East wall, Bit 2 = South wall, Bit 3 = West wall
uint8_t wallMap[MAZE_SIZE][MAZE_SIZE];

// Track which cells have been visited
bool visited[MAZE_SIZE][MAZE_SIZE];

enum RobotState { 
  STATE_IDLE, 
  STATE_THINKING, 
  STATE_MOVING, 
  STATE_AT_GOAL 
};
RobotState robotState = STATE_IDLE;

volatile bool cellCrossed = false;
volatile unsigned long lastPulse = 0;

WebSocketsClient webSocket;
float gyroZOffset = 0;
float currentYaw = 0;
unsigned long lastMPUUpdate = 0;

unsigned long lastSensorUpdate = 0;
#define SENSOR_UPDATE_INTERVAL 50

// ======================= FUNCTION DECLARATIONS =======================
void initMotors();
void setMotors(int L, int R);
void stopMotors();
void driveOneCell();
void driveForward();
void turnRight90();
void turnLeft90();
float readDistance(int trig, int echo);
void initMPU6050();
void updateYaw();
void floodFill();
void updateWalls();
int getBestDirection();
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
    pinMode(0, INPUT_PULLUP);
    
    EEPROM.begin(EEPROM_SIZE);
    loadMapFromEEPROM();
    
    initSensors();
    initMotors();
    initMPU6050();

    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(" Connected!");

    webSocket.beginSSL(WS_SERVER_HOST, 443, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    
    Serial.println("FLOOD FILL Maze Solver Ready. Waiting for Start...");
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
    
    if (millis() - lastSensorUpdate > SENSOR_UPDATE_INTERVAL) {
        sendSensorData();
        lastSensorUpdate = millis();
    }
    
    switch (robotState) {
        case STATE_IDLE:
            if (millis() % 1000 < 500) digitalWrite(STATUS_LED, HIGH);
            else digitalWrite(STATUS_LED, LOW);
            break;
            
        case STATE_THINKING:
            stopMotors();
            digitalWrite(STATUS_LED, HIGH);
            
            // 1. Update walls from sensors
            updateWalls();
            
            // 2. Recalculate flood fill
            floodFill();
            
            // 3. Transition to Moving
            robotState = STATE_MOVING;
            digitalWrite(STATUS_LED, LOW); 
            break;
            
        case STATE_MOVING:
            executeNextStep();
            
            if (currentX == GOAL_X && currentY == GOAL_Y) {
                robotState = STATE_AT_GOAL;
                saveMapToEEPROM();  // Save learned walls
                Serial.println("GOAL REACHED! Map saved to EEPROM.");
                sendTelemetry(); 
            } else {
                robotState = STATE_THINKING;
            }
            break;
            
        case STATE_AT_GOAL:
            stopMotors();
            if (millis() % 300 < 150) digitalWrite(STATUS_LED, HIGH);
            else digitalWrite(STATUS_LED, LOW);
            
            // RE-RUN: Press Boot Button
            if (digitalRead(0) == LOW) {
                Serial.println("SPEED RUN: Starting with known map!");
                
                currentX = 0; 
                currentY = 0;
                heading = NORTH;
                
                // Reset visited but KEEP wallMap
                memset(visited, false, sizeof(visited));
                floodFill();  // Recalculate from start
                
                delay(1000);
                robotState = STATE_THINKING;
            }
            break;
    }
}

// ======================= FLOOD FILL ALGORITHM =======================

void floodFill() {
    // Initialize all cells to max distance
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            floodMap[y][x] = 255;
        }
    }
    
    // Goal cell has distance 0
    floodMap[GOAL_Y][GOAL_X] = 0;
    
    // BFS to fill distances
    bool changed = true;
    while (changed) {
        changed = false;
        
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int x = 0; x < MAZE_SIZE; x++) {
                if (floodMap[y][x] == 255) continue;
                
                int currentDist = floodMap[y][x];
                
                // Check all 4 neighbors
                // North (y-1)
                if (y > 0 && !(wallMap[y][x] & (1 << NORTH))) {
                    if (floodMap[y-1][x] > currentDist + 1) {
                        floodMap[y-1][x] = currentDist + 1;
                        changed = true;
                    }
                }
                // South (y+1)
                if (y < MAZE_SIZE-1 && !(wallMap[y][x] & (1 << SOUTH))) {
                    if (floodMap[y+1][x] > currentDist + 1) {
                        floodMap[y+1][x] = currentDist + 1;
                        changed = true;
                    }
                }
                // East (x+1)
                if (x < MAZE_SIZE-1 && !(wallMap[y][x] & (1 << EAST))) {
                    if (floodMap[y][x+1] > currentDist + 1) {
                        floodMap[y][x+1] = currentDist + 1;
                        changed = true;
                    }
                }
                // West (x-1)
                if (x > 0 && !(wallMap[y][x] & (1 << WEST))) {
                    if (floodMap[y][x-1] > currentDist + 1) {
                        floodMap[y][x-1] = currentDist + 1;
                        changed = true;
                    }
                }
            }
        }
    }
    
    Serial.printf("Flood Fill complete. Current cell distance: %d\n", floodMap[currentY][currentX]);
}

void updateWalls() {
    // Mark current cell as visited
    visited[currentY][currentX] = true;
    
    // Read sensors
    float distFront = readDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    float distLeft = readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float distRight = readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    
    bool wallFront = (distFront < WALL_DIST_CM);
    bool wallLeft = (distLeft < WALL_DIST_CM);
    bool wallRight = (distRight < WALL_DIST_CM);
    
    // Convert sensor readings to absolute directions based on heading
    int frontDir = heading;
    int leftDir = (heading + 3) % 4;  // WEST when facing NORTH
    int rightDir = (heading + 1) % 4; // EAST when facing NORTH
    
    // Update wall map for current cell
    if (wallFront) {
        wallMap[currentY][currentX] |= (1 << frontDir);
        // Also update neighbor's wall
        int nx = currentX, ny = currentY;
        if (frontDir == NORTH) ny--;
        else if (frontDir == SOUTH) ny++;
        else if (frontDir == EAST) nx++;
        else if (frontDir == WEST) nx--;
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            wallMap[ny][nx] |= (1 << ((frontDir + 2) % 4));
        }
    }
    
    if (wallLeft) {
        wallMap[currentY][currentX] |= (1 << leftDir);
        int nx = currentX, ny = currentY;
        if (leftDir == NORTH) ny--;
        else if (leftDir == SOUTH) ny++;
        else if (leftDir == EAST) nx++;
        else if (leftDir == WEST) nx--;
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            wallMap[ny][nx] |= (1 << ((leftDir + 2) % 4));
        }
    }
    
    if (wallRight) {
        wallMap[currentY][currentX] |= (1 << rightDir);
        int nx = currentX, ny = currentY;
        if (rightDir == NORTH) ny--;
        else if (rightDir == SOUTH) ny++;
        else if (rightDir == EAST) nx++;
        else if (rightDir == WEST) nx--;
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            wallMap[ny][nx] |= (1 << ((rightDir + 2) % 4));
        }
    }
    
    Serial.printf("Walls at (%d,%d): F=%d L=%d R=%d\n", currentX, currentY, wallFront, wallLeft, wallRight);
}

int getBestDirection() {
    int bestDir = -1;
    int bestDist = floodMap[currentY][currentX];  // Must improve
    
    // Check all 4 directions, prefer one that reduces distance
    int dirs[] = {NORTH, EAST, SOUTH, WEST};
    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};
    
    for (int i = 0; i < 4; i++) {
        int dir = dirs[i];
        int nx = currentX + dx[i];
        int ny = currentY + dy[i];
        
        // Check bounds
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
        
        // Check wall
        if (wallMap[currentY][currentX] & (1 << dir)) continue;
        
        // Check if this direction brings us closer to goal
        if (floodMap[ny][nx] < bestDist) {
            bestDist = floodMap[ny][nx];
            bestDir = dir;
        }
    }
    
    // If no improving direction found, we're stuck or need to explore
    if (bestDir == -1) {
        Serial.println("WARNING: No improving path found!");
        // Try any open direction
        for (int i = 0; i < 4; i++) {
            int dir = dirs[i];
            if (!(wallMap[currentY][currentX] & (1 << dir))) {
                int nx = currentX + dx[i];
                int ny = currentY + dy[i];
                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    bestDir = dir;
                    break;
                }
            }
        }
    }
    
    return bestDir;
}

// ======================= MOVEMENT & EXECUTION =======================

void executeNextStep() {
    int bestAction = getBestDirection();
    
    if (bestAction == -1) {
        Serial.println("ERROR: No valid move! Stopping.");
        robotState = STATE_AT_GOAL;
        return;
    }
    
    Serial.printf("Moving %s (flood dist: %d -> %d)\n", 
        bestAction == NORTH ? "NORTH" : bestAction == EAST ? "EAST" : bestAction == SOUTH ? "SOUTH" : "WEST",
        floodMap[currentY][currentX],
        floodMap[currentY + (bestAction == SOUTH ? 1 : bestAction == NORTH ? -1 : 0)]
                [currentX + (bestAction == EAST ? 1 : bestAction == WEST ? -1 : 0)]);
    
    // Turn to face action
    int turnDiff = (bestAction - heading + 4) % 4;
    
    if (turnDiff == 1) turnRight90();
    else if (turnDiff == 2) { turnRight90(); turnRight90(); }
    else if (turnDiff == 3) turnLeft90();
    
    heading = bestAction;
    delay(200);

    // Drive 1 Cell
    driveOneCell();
    
    // Update Position
    if (heading == NORTH) currentY--;
    else if (heading == SOUTH) currentY++;
    else if (heading == EAST) currentX++;
    else if (heading == WEST) currentX--;
    
    sendTelemetry();
}

void driveOneCell() {
    cellCrossed = false;
    unsigned long startTime = millis();
    
    while (!cellCrossed && (millis() - startTime < MAX_CELL_TIME)) {
        driveForward();
        delay(20);
    }
    
    stopMotors();
}

void driveForward() {
    setMotors(BASE_SPEED, BASE_SPEED);
}

// ======================= MOTOR & SENSOR HELPERS =======================

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT);
    pinMode(MOTOR_ENB, OUTPUT);
}

void setMotors(int L, int R) {
    if (L > 0) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); }
    else { digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH); L = -L; }
    analogWrite(MOTOR_ENA, constrain(L, 0, 255));

    if (R > 0) { digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW); }
    else { digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH); R = -R; }
    analogWrite(MOTOR_ENB, constrain(R, 0, 255));
}

void stopMotors() { setMotors(0, 0); }

void turnRight90() {
    setMotors(255, -255); delay(KICK_TIME);
    setMotors(TURN_SPEED, -TURN_SPEED);
    
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
    setMotors(-255, 255); delay(KICK_TIME);
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
    long dur = pulseIn(echo, HIGH, 5000);
    if (dur == 0) return 999;
    return dur * 0.034 / 2;
}

void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
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
    if (now - lastPulse > 200) {
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
        
        // Auto-start: Send start/goal config to visualizer
        StaticJsonDocument<128> startDoc;
        startDoc["type"] = "maze_config";
        startDoc["start"]["x"] = currentX;
        startDoc["start"]["y"] = currentY;
        startDoc["goal"]["x"] = GOAL_X;
        startDoc["goal"]["y"] = GOAL_Y;
        startDoc["size"] = MAZE_SIZE;
        String startJson;
        serializeJson(startDoc, startJson);
        webSocket.sendTXT(startJson);
        
        // Reset state and start solving
        memset(wallMap, 0, sizeof(wallMap));
        memset(visited, false, sizeof(visited));
        heading = NORTH;
        
        Serial.printf("AUTO-START: (%d,%d) -> (%d,%d)\n", currentX, currentY, GOAL_X, GOAL_Y);
        
        floodFill();
        robotState = STATE_THINKING;
    }
    else if (type == WStype_TEXT) {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, payload);
        
        if (doc["type"] == "stop") {
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
    
    const char* stateNames[] = {"idle", "thinking", "moving", "goal"};
    doc["state"] = stateNames[robotState];
    
    // Include flood distance for debugging
    doc["floodDist"] = floodMap[currentY][currentX];
    
    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
}

void sendSensorData() {
    float distFront = readDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    float distLeft = readDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float distRight = readDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    
    updateYaw();
    
    StaticJsonDocument<384> doc;
    doc["type"] = "sensors";
    
    doc["front"] = distFront;
    doc["left"] = distLeft;
    doc["right"] = distRight;
    
    doc["yaw"] = currentYaw;
    
    doc["x"] = currentX;
    doc["y"] = currentY;
    doc["heading"] = heading;
    
    const char* stateNames[] = {"idle", "thinking", "moving", "goal"};
    doc["state"] = stateNames[robotState];
    
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
    
    // Write wall map (25 bytes for 5x5)
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            EEPROM.write(addr++, wallMap[y][x]);
        }
    }
    
    EEPROM.commit();
    Serial.printf("Wall map saved to EEPROM (%d bytes)\n", addr);
}

void loadMapFromEEPROM() {
    int addr = 0;
    
    // Check magic number
    uint32_t magic;
    EEPROM.get(addr, magic);
    addr += sizeof(magic);
    
    if (magic != EEPROM_MAGIC) {
        Serial.println("No valid map in EEPROM - starting fresh");
        memset(wallMap, 0, sizeof(wallMap));
        memset(visited, false, sizeof(visited));
        return;
    }
    
    // Read wall map
    int wallCount = 0;
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            wallMap[y][x] = EEPROM.read(addr++);
            if (wallMap[y][x] != 0) wallCount++;
        }
    }
    
    Serial.printf("Wall map loaded from EEPROM (%d cells with walls)\n", wallCount);
}
