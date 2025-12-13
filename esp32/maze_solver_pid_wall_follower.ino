/*
 * Maze Solver - PID Wall Follower (Left Hand Rule)
 * WITH WebSocket + MPU6050 Telemetry
 * 
 * Hardware:
 * - 3x HC-SR04 (Front, Left, Right)
 * - L298N Motors
 * - MPU6050 Gyroscope
 * 
 * Strategy:
 * - Maintain a set distance from the LEFT wall using PID.
 * - If the left wall disappears -> Turn Left.
 * - If the front is blocked -> Turn Right.
 * - Sends movement telemetry to WebSocket server for visualization.
 * 
 * PID continuously corrects any turn imprecision!
 */

#include <NewPing.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <EEPROM.h>

// ======================= EEPROM PATH STORAGE =======================

#define EEPROM_SIZE 512
#define PATH_MAGIC 0xAB          // Marker for valid saved path
#define MAX_PATH_LENGTH 100      // Max moves to store

// Action codes for path storage
#define ACTION_FORWARD    0
#define ACTION_TURN_LEFT  1
#define ACTION_TURN_RIGHT 2
#define ACTION_U_TURN     3

// EEPROM Layout: [MAGIC(1)][LENGTH(1)][ACTIONS(100)]
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_LENGTH_ADDR 1
#define EEPROM_PATH_START 2

// ======================= PIN DEFINITIONS =======================

// HC-SR04 Ultrasonic Sensors
#define FRONT_TRIG_PIN 14
#define FRONT_ECHO_PIN 35
#define LEFT_TRIG_PIN 18
#define LEFT_ECHO_PIN 13
#define RIGHT_TRIG_PIN 16
#define RIGHT_ECHO_PIN 34

#define MAX_DISTANCE 100 // cm
#define SENSOR_SAMPLES 3

NewPing sonarFront(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

// L298N Motor Driver
#define MOTOR_IN1 23
#define MOTOR_IN2 22
#define MOTOR_ENA 25 
#define MOTOR_IN3 21
#define MOTOR_IN4 19
#define MOTOR_ENB 26 

#define STATUS_LED 2

// ======================= TUNING CONSTANTS =======================

// Motor Speed
#define BASE_SPEED 120      // Standard forward speed (PWM 0-255)
#define TURN_SPEED 140      // Speed during turning
#define MAX_SPEED 150       // Cap for PID adjustment

// Wall Following Thresholds
#define DESIRED_WALL_DIST 4.0  // Target distance when only one wall visible
#define FRONT_THRESHOLD 12.0   // If closer than 12cm, stop/turn
#define NO_WALL_THRESHOLD 25.0 // If left dist > 25cm, wall is gone
#define LEFT_BIAS 1.5          // Stay 1.5cm closer to LEFT wall when centering

// Time-Based Turn Durations (TUNE THESE FOR YOUR ROBOT!)
#define TURN_90_TIME 150       // ms to turn 90 degrees
#define TURN_180_TIME 7000      // ms to turn 180 degrees
#define FORWARD_AFTERGAP 350  // ms to drive forward after detecting gap
#define FORWARD_INTO_CELL 400  // ms to drive into new corridor

// PID Gains - SOFT for gentle "magnetic" pull
float Kp = 1.0;    // Very soft proportional (gentle drift)
float Kd = 0.5;    // Light dampening

// Deadband - ignore small errors for smoother driving
#define DEADBAND 1.5  // Ignore errors smaller than this (cm)

// Minimum speed to prevent motor stalling
#define MIN_SPEED 100

// ======================= GLOBAL VARIABLES =======================

float prevError = 0;

// Stuck detection variables
float lastFront = 0, lastLeft = 0, lastRight = 0;
int stuckCounter = 0;
#define STUCK_THRESHOLD 3       // Cycles before declaring stuck (FAST!)
#define STUCK_TOLERANCE 2.0     // cm - higher tolerance = faster detection
#define BACKUP_TIME 400         // ms to reverse when stuck

// ======================= WIFI & WEBSOCKET =======================

// WiFi credentials - CHANGE THESE
const char* WIFI_SSID = "POCO X7 Pro";
const char* WIFI_PASSWORD = "12345678";

// WebSocket server (Railway deployment)
const char* WS_HOST = "botvisualizer-production.up.railway.app";
const int WS_PORT = 443;
const bool WS_USE_SSL = true;

WebSocketsClient webSocket;
bool wsConnected = false;

// ======================= MPU6050 =======================

MPU6050 mpu(Wire);
float currentYaw = 0;
unsigned long lastMpuUpdate = 0;
#define MPU_UPDATE_INTERVAL 10  // ms between MPU reads

// ======================= CELL POSITION TRACKING =======================

int cellX = 0;          // Current grid position X
int cellY = 0;          // Current grid position Y  
int heading = 0;        // 0=North, 90=East, 180=South, 270=West
int runNumber = 1;      // Current run count
int moveCount = 0;      // Moves in current run
String lastAction = ""; // Last movement action

// ======================= SPEED RUN MODE =======================

bool speedRunMode = false;          // True if executing saved path
uint8_t savedPath[MAX_PATH_LENGTH]; // Buffer for loaded path
int savedPathLength = 0;            // Length of saved path
int speedRunIndex = 0;              // Current position in speed run
#define SPEED_RUN_SPEED 180         // Faster speed for speed run


// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP); // Boot button
    
    initMotors();
    initSensors();
    initEEPROM();
    
    Serial.println("===================================");
    Serial.println("  PID Wall Follower + Self-Learning");
    Serial.println("  WebSocket + MPU6050 + EEPROM");
    Serial.println("===================================");
    
    // Initialize I2C and MPU6050
    Wire.begin();
    byte mpuStatus = mpu.begin();
    Serial.print("MPU6050 status: ");
    Serial.println(mpuStatus);
    
    if (mpuStatus == 0) {
        Serial.println("Calibrating MPU6050... Keep robot still!");
        digitalWrite(STATUS_LED, HIGH);
        mpu.calcOffsets();
        Serial.println("MPU6050 calibrated!");
    } else {
        Serial.println("MPU6050 init failed! Continuing without gyro.");
    }
    
    // Connect to WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int wifiAttempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        delay(500);
        Serial.print(".");
        wifiAttempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        
        // Initialize WebSocket
        if (WS_USE_SSL) {
            webSocket.beginSSL(WS_HOST, WS_PORT, "/");
        } else {
            webSocket.begin(WS_HOST, WS_PORT, "/");
        }
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(3000);
        Serial.println("WebSocket connecting...");
    } else {
        Serial.println("\nWiFi failed! Running offline.");
    }
    
    // Check for saved path to determine mode
    savedPathLength = loadPathFromEEPROM();
    speedRunMode = (savedPathLength > 0);
    
    if (speedRunMode) {
        Serial.println("=== SPEED RUN MODE ===");
        Serial.printf("Loaded %d moves. Press BOOT to start speed run!\n", savedPathLength);
        
        // Fast blink pattern for speed run ready
        while(digitalRead(0) == HIGH) {
            webSocket.loop();
            digitalWrite(STATUS_LED, HIGH); delay(50);
            digitalWrite(STATUS_LED, LOW); delay(50);
        }
        delay(1000);
        digitalWrite(STATUS_LED, HIGH);
        
        // Execute the saved path!
        executeSpeedRun();
        
        // After speed run, stop and wait
        Serial.println("Speed run finished. Reset to run again.");
        while(1) {
            webSocket.loop();
            digitalWrite(STATUS_LED, HIGH); delay(500);
            digitalWrite(STATUS_LED, LOW); delay(500);
        }
    } else {
        Serial.println("=== EXPLORATION MODE ===");
        Serial.println("No saved path. Press BOOT to explore maze...");
        
        // Slow blink pattern for exploration mode
        while(digitalRead(0) == HIGH) {
            webSocket.loop();
            digitalWrite(STATUS_LED, HIGH); delay(200);
            digitalWrite(STATUS_LED, LOW); delay(200);
        }
        delay(1000);
        digitalWrite(STATUS_LED, HIGH);
        Serial.println("GO! Exploring maze...");
        
        // Send run start message
        sendMovement("exploration_start");
    }
}

// ======================= MAIN LOOP =======================

void loop() {
    // Process WebSocket messages
    webSocket.loop();
    updateMPU();
    
    float dFront = readDistanceFront();
    float dLeft = readDistanceLeft();
    float dRight = readDistanceRight();
    
    // Debug output
    Serial.printf("F:%.1f L:%.1f R:%.1f | ", dFront, dLeft, dRight);

    // ========================================================
    // PRIORITY 0: STUCK DETECTION
    // ========================================================
    // Check if readings are staying consistent (robot not moving)
    if (abs(dFront - lastFront) < STUCK_TOLERANCE &&
        abs(dLeft - lastLeft) < STUCK_TOLERANCE &&
        abs(dRight - lastRight) < STUCK_TOLERANCE) {
        stuckCounter++;
    } else {
        stuckCounter = 0;  // Reset if readings change
    }
    
    // Save current readings for next comparison
    lastFront = dFront;
    lastLeft = dLeft;
    lastRight = dRight;
    
    // If stuck for too long, perform recovery!
    if (stuckCounter >= STUCK_THRESHOLD) {
        Serial.println("STUCK DETECTED! Recovering...");
        stuckCounter = 0;
        
        // Recovery: Back up first
        setMotors(-BASE_SPEED, -BASE_SPEED);
        delay(BACKUP_TIME);
        stopMotors();
        delay(100);
        
        // Then decide which way to turn based on available space
        if (dRight > dLeft) {
            Serial.println("Recovery: Turn RIGHT");
            turnRight();
        } else {
            Serial.println("Recovery: Turn LEFT");
            turnLeft();
        }
        return;
    }

    // ========================================================
    // PRIORITY 1: Check for "Gap" on the Left (Turn Left)
    // ========================================================
    if (dLeft > NO_WALL_THRESHOLD) {
        Serial.println("LEFT OPENING! Turning Left.");
        stopMotors();
        delay(100);
        
        // Drive forward to align with the new corridor
        sendMovement("forward");
        driveForward(BASE_SPEED);
        delay(FORWARD_AFTER_GAP);
        
        // Turn left (time-based)
        turnLeft();
        
        // Drive forward into new corridor
        sendMovement("forward");
        driveForward(BASE_SPEED);
        delay(FORWARD_INTO_CELL);
        
        prevError = 0; // Reset PID
        return;
    }

    // ========================================================
    // PRIORITY 2: Check Front Wall (Turn Right or U-Turn)
    // ========================================================
    if (dFront < FRONT_THRESHOLD) {
        Serial.println("FRONT BLOCKED!");
        stopMotors();
        delay(100);
        
        // Check if we can go Right
        if (dRight > 15) {
            Serial.println("Turning Right.");
            turnRight();
        } else {
            // Dead End - U-Turn
            Serial.println("DEAD END! U-Turn.");
            turnAround();
        }
        
        prevError = 0; // Reset PID
        return;
    }

    // ========================================================
    // PRIORITY 3: Follow Wall (PID Control)
    // ========================================================
    pidWallFollow(dLeft, dRight);
}

// ======================= PID CONTROL =======================

void pidWallFollow(float leftDist, float rightDist) {
    float error = 0;
    
    bool leftWall = (leftDist < NO_WALL_THRESHOLD);
    bool rightWall = (rightDist < NO_WALL_THRESHOLD);
    
    if (leftWall && rightWall) {
        // BOTH walls visible - CENTER with LEFT BIAS
        // LEFT_BIAS makes robot stay slightly closer to left wall
        // Equilibrium when leftDist is 1.5cm less than rightDist
        error = (leftDist - rightDist) + LEFT_BIAS;
        Serial.printf("CENTER: L=%.1f R=%.1f ", leftDist, rightDist);
    }
    else if (leftWall) {
        // Only left wall - maintain distance from it
        error = leftDist - DESIRED_WALL_DIST;
        Serial.printf("LEFT WALL: ");
    }
    else if (rightWall) {
        // Only right wall - maintain distance from it
        error = DESIRED_WALL_DIST - rightDist;
        Serial.printf("RIGHT WALL: ");
    }
    else {
        // No walls - go straight
        error = 0;
        Serial.print("NO WALLS ");
    }

    // Clamp error to prevent extreme corrections
    error = constrain(error, -5.0, 5.0);
    
    // DEADBAND: Ignore small errors for smoother, less "sticky" driving
    // This creates the "invisible magnet" feel - gentle pull, not hard correction
    if (abs(error) < DEADBAND) {
        error = 0;  // Don't correct if error is small
    }

    // PD calculation
    float P = Kp * error;
    float D = Kd * (error - prevError);
    float correction = P + D;
    prevError = error;

    // Apply correction
    int leftSpeed = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;

    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    setMotors(leftSpeed, rightSpeed);
    Serial.printf("Err:%.1f L:%d R:%d\n", error, leftSpeed, rightSpeed);
}
// ======================= SIMPLE TIME-BASED TURNS =======================

void turnLeft() {
    Serial.println(">>> Turning LEFT");
    sendMovement("turn_left");
    setMotors(-TURN_SPEED, TURN_SPEED);  // Left backward, Right forward
    delay(TURN_90_TIME);
    stopMotors();
    delay(100);
}

void turnRight() {
    Serial.println(">>> Turning RIGHT");
    sendMovement("turn_right");
    setMotors(TURN_SPEED, -TURN_SPEED);  // Left forward, Right backward
    delay(TURN_90_TIME);
    stopMotors();
    delay(100);
}

void turnAround() {
    Serial.println(">>> Turning 180");
    sendMovement("u_turn");
    setMotors(TURN_SPEED, -TURN_SPEED);
    delay(TURN_180_TIME);
    stopMotors();
    delay(100);
}

// ======================= MOTOR CONTROL =======================

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT); pinMode(MOTOR_ENB, OUTPUT);
}

void setMotors(int L, int R) {
    // Left motor
    if (L >= 0) { 
        digitalWrite(MOTOR_IN1, HIGH); 
        digitalWrite(MOTOR_IN2, LOW); 
    } else { 
        digitalWrite(MOTOR_IN1, LOW); 
        digitalWrite(MOTOR_IN2, HIGH); 
        L = -L; 
    }
    analogWrite(MOTOR_ENA, constrain(L, 0, 255));

    // Right motor
    if (R >= 0) { 
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

void driveForward(int speed) {
    setMotors(speed, speed);
}

// ======================= SENSORS =======================

void initSensors() {
    pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(LEFT_TRIG_PIN, OUTPUT); pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT); pinMode(RIGHT_ECHO_PIN, INPUT);
}

float readDistanceFront() {
    unsigned int distance = sonarFront.ping_median(SENSOR_SAMPLES);
    return (distance == 0) ? 999.0 : sonarFront.convert_cm(distance);
}

float readDistanceLeft() {
    unsigned int distance = sonarLeft.ping_median(SENSOR_SAMPLES);
    return (distance == 0) ? 999.0 : sonarLeft.convert_cm(distance);
}

float readDistanceRight() {
    unsigned int distance = sonarRight.ping_median(SENSOR_SAMPLES);
    return (distance == 0) ? 999.0 : sonarRight.convert_cm(distance);
}

// ======================= WEBSOCKET FUNCTIONS =======================

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected!");
            wsConnected = false;
            break;
            
        case WStype_CONNECTED:
            Serial.println("WebSocket connected!");
            wsConnected = true;
            // Send identification
            webSocket.sendTXT("{\"type\":\"identify\",\"client\":\"esp32\"}");
            // Send path status to website
            delay(500);
            sendPathStatus();
            break;
            
        case WStype_TEXT: {
            Serial.printf("Received: %s\n", payload);
            
            // Parse JSON command
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, payload);
            
            if (!error) {
                // Check if it's a path command
                if (doc.containsKey("command")) {
                    handlePathCommand(doc);
                }
            }
            break;
        }
            
        case WStype_ERROR:
            Serial.println("WebSocket error!");
            break;
            
        default:
            break;
    }
}

void sendMovement(String action) {
    if (!wsConnected) return;
    
    // Update MPU reading
    mpu.update();
    currentYaw = mpu.getAngleZ();
    
    // Read current sensor values
    float front = readDistanceFront();
    float left = readDistanceLeft();
    float right = readDistanceRight();
    
    // Update cell position based on action
    updateCellPosition(action);
    moveCount++;
    lastAction = action;
    
    // Build JSON message
    StaticJsonDocument<256> doc;
    doc["type"] = "movement";
    doc["action"] = action;
    doc["heading"] = currentYaw;
    doc["run"] = runNumber;
    doc["move"] = moveCount;
    
    JsonObject pos = doc.createNestedObject("position");
    pos["x"] = cellX;
    pos["y"] = cellY;
    
    JsonObject sensors = doc.createNestedObject("sensors");
    sensors["front"] = front;
    sensors["left"] = left;
    sensors["right"] = right;
    
    JsonObject walls = doc.createNestedObject("walls");
    walls["front"] = (front < FRONT_THRESHOLD);
    walls["left"] = (left < NO_WALL_THRESHOLD);
    walls["right"] = (right < NO_WALL_THRESHOLD);
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    webSocket.sendTXT(jsonStr);
    
    Serial.print("Sent: ");
    Serial.println(jsonStr);
}

void updateCellPosition(String action) {
    if (action == "forward") {
        // Move forward based on current heading
        if (heading == 0) cellY--;       // North
        else if (heading == 90) cellX++; // East
        else if (heading == 180) cellY++;// South
        else if (heading == 270) cellX--;// West
    }
    else if (action == "turn_left") {
        heading = (heading - 90 + 360) % 360;
    }
    else if (action == "turn_right") {
        heading = (heading + 90) % 360;
    }
    else if (action == "u_turn") {
        heading = (heading + 180) % 360;
    }
}

void updateMPU() {
    if (millis() - lastMpuUpdate >= MPU_UPDATE_INTERVAL) {
        mpu.update();
        currentYaw = mpu.getAngleZ();
        lastMpuUpdate = millis();
    }
}

// ======================= EEPROM FUNCTIONS =======================

void initEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("EEPROM initialized");
}

bool hasSavedPath() {
    return EEPROM.read(EEPROM_MAGIC_ADDR) == PATH_MAGIC;
}

int loadPathFromEEPROM() {
    if (!hasSavedPath()) {
        Serial.println("No saved path in EEPROM");
        return 0;
    }
    
    int length = EEPROM.read(EEPROM_LENGTH_ADDR);
    if (length > MAX_PATH_LENGTH) length = MAX_PATH_LENGTH;
    
    for (int i = 0; i < length; i++) {
        savedPath[i] = EEPROM.read(EEPROM_PATH_START + i);
    }
    
    Serial.printf("Loaded %d moves from EEPROM\n", length);
    return length;
}

void savePathToEEPROM(uint8_t* path, int length) {
    if (length > MAX_PATH_LENGTH) length = MAX_PATH_LENGTH;
    
    EEPROM.write(EEPROM_MAGIC_ADDR, PATH_MAGIC);
    EEPROM.write(EEPROM_LENGTH_ADDR, length);
    
    for (int i = 0; i < length; i++) {
        EEPROM.write(EEPROM_PATH_START + i, path[i]);
    }
    
    EEPROM.commit();
    Serial.printf("Saved %d moves to EEPROM\n", length);
}

void clearPathFromEEPROM() {
    EEPROM.write(EEPROM_MAGIC_ADDR, 0x00);  // Invalidate magic
    EEPROM.commit();
    Serial.println("Path cleared from EEPROM");
}

void sendPathStatus() {
    if (!wsConnected) return;
    
    StaticJsonDocument<128> doc;
    doc["type"] = "path_status";
    doc["hasSavedPath"] = hasSavedPath();
    doc["pathLength"] = hasSavedPath() ? EEPROM.read(EEPROM_LENGTH_ADDR) : 0;
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    webSocket.sendTXT(jsonStr);
}

// ======================= SPEED RUN EXECUTION =======================

void executeSpeedRun() {
    Serial.println("=== STARTING SPEED RUN ===");
    sendMovement("speed_run_start");
    
    for (int i = 0; i < savedPathLength; i++) {
        // Check for abort (button press)
        if (digitalRead(0) == LOW) {
            Serial.println("Speed run aborted!");
            stopMotors();
            sendMovement("speed_run_abort");
            return;
        }
        
        uint8_t action = savedPath[i];
        Serial.printf("Speed run move %d: action %d\n", i, action);
        
        switch (action) {
            case ACTION_FORWARD:
                sendMovement("forward");
                driveForward(SPEED_RUN_SPEED);
                delay(FORWARD_INTO_CELL);  // Adjust for your maze cell size
                stopMotors();
                delay(50);
                break;
                
            case ACTION_TURN_LEFT:
                sendMovement("turn_left");
                setMotors(-TURN_SPEED, TURN_SPEED);
                delay(TURN_90_TIME);
                stopMotors();
                delay(50);
                break;
                
            case ACTION_TURN_RIGHT:
                sendMovement("turn_right");
                setMotors(TURN_SPEED, -TURN_SPEED);
                delay(TURN_90_TIME);
                stopMotors();
                delay(50);
                break;
                
            case ACTION_U_TURN:
                sendMovement("u_turn");
                setMotors(TURN_SPEED, -TURN_SPEED);
                delay(TURN_180_TIME);
                stopMotors();
                delay(50);
                break;
        }
    }
    
    stopMotors();
    Serial.println("=== SPEED RUN COMPLETE ===");
    sendMovement("speed_run_complete");
}

// ======================= HANDLE PATH COMMANDS =======================

void handlePathCommand(JsonDocument& doc) {
    const char* cmd = doc["command"];
    
    if (strcmp(cmd, "save_path") == 0) {
        JsonArray pathArray = doc["path"].as<JsonArray>();
        int length = pathArray.size();
        if (length > MAX_PATH_LENGTH) length = MAX_PATH_LENGTH;
        
        uint8_t path[MAX_PATH_LENGTH];
        for (int i = 0; i < length; i++) {
            path[i] = pathArray[i].as<uint8_t>();
        }
        
        savePathToEEPROM(path, length);
        
        // Send confirmation
        StaticJsonDocument<64> response;
        response["type"] = "path_saved";
        response["length"] = length;
        String jsonStr;
        serializeJson(response, jsonStr);
        webSocket.sendTXT(jsonStr);
    }
    else if (strcmp(cmd, "clear_path") == 0) {
        clearPathFromEEPROM();
        
        StaticJsonDocument<64> response;
        response["type"] = "path_cleared";
        String jsonStr;
        serializeJson(response, jsonStr);
        webSocket.sendTXT(jsonStr);
    }
    else if (strcmp(cmd, "get_path_status") == 0) {
        sendPathStatus();
    }
}

