/*
 * Maze Solver Bot - Flood Fill Standalone Version
 * Logic:
 * 1. "Discovery Run": Robot explores, detects walls, updates map, uses Flood Fill for optimal path.
 * 2. "Speed Run": Press Boot Button at goal. Robot resets position but KEEPS wall memory.
 * 
 * Flood Fill is deterministic and always finds the shortest path!
 * 
 * This is a standalone version WITHOUT WebSocket/WiFi connectivity.
 */

#include <Wire.h>
#include <EEPROM.h>
#include <NewPing.h>
#include <MPU6050_light.h>

// EEPROM Layout: [Magic(4)] [WallData(MAZE_SIZE*MAZE_SIZE*4 bytes for walls)]
#define EEPROM_SIZE 128
#define EEPROM_MAGIC 0x464C4F44  // "FLOD" in hex

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

// TCRT5000 Line Sensor
#define LINE_SENSOR_PIN 4
#define LINE_THRESHOLD 1000

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

#define BASE_SPEED 120
#define SPEED_FAST 180       // High speed during snap turns
#define SPEED_SLOW 100       // Approach speed near target
#define BRAKE_TIME 50        // Active brake duration (ms)
#define KICK_TIME 30         // Full power kick duration (ms)

#define TOLERANCE_90 4.0     // Acceptable error for 90° turns
#define TOLERANCE_180 5.0    // Acceptable error for 180° turns
#define SLOW_ZONE_90 25      // Start slowing at this distance from target
#define SLOW_ZONE_180 40     // Larger slow zone for 180° turns
#define TURN_TIMEOUT 2000    // Max time for any turn (ms)

#define MAZE_SIZE 5
#define MAX_CELL_TIME 2000
#define WALL_DIST_CM 7.0

// ======================= GLOBAL VARIABLES =======================

int currentX = 4;
int currentY = 4;
int GOAL_X = 0;
int GOAL_Y = 0;
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

// MPU6050 using library
MPU6050 mpu(Wire);

// Track motor directions for active braking
int lastLeftDir = 0;   // 1=forward, -1=reverse, 0=stopped
int lastRightDir = 0;

// ======================= FUNCTION DECLARATIONS =======================
void initMotors();
void setMotors(int L, int R);
void stopMotors();
void brakeMotors();
void kickStart();
void driveOneCell();
void driveForward();
void turnRight90();
void turnLeft90();
void turn180();
float normalizeAngle(float diff);
float readDistanceFront();
float readDistanceLeft();
float readDistanceRight();
void initMPU6050();
void floodFill();
void updateWalls();
int getBestDirection();
void executeNextStep();
void saveMapToEEPROM();
void loadMapFromEEPROM();
void IRAM_ATTR onLineCrossed();
void initSensors();
void startSolving();

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP);  // Boot button
    
    EEPROM.begin(EEPROM_SIZE);
    
    // --- NEW: EEPROM RESET FEATURE ---
    // If Button is HELD during startup, wipe the memory
    if (digitalRead(0) == LOW) {
        Serial.println("WIPING EEPROM...");
        
        // Blink LED fast to indicate wiping
        for(int i=0; i<5; i++) {
            digitalWrite(STATUS_LED, HIGH); delay(100);
            digitalWrite(STATUS_LED, LOW); delay(100);
        }
        
        // Write 0 to all addresses
        for (int i = 0; i < EEPROM_SIZE; i++) {
            EEPROM.write(i, 0);
        }
        EEPROM.commit();
        
        Serial.println("EEPROM CLEARED! Please Restart.");
        
        // Lock here (Stops the robot until you restart)
        while(1) {
            digitalWrite(STATUS_LED, HIGH); delay(500);
            digitalWrite(STATUS_LED, LOW); delay(500);
        }
    }
    loadMapFromEEPROM();
    
    initSensors();
    initMotors();
    initMPU6050();
    
    Serial.println("FLOOD FILL Maze Solver Ready (Standalone Mode).");
    Serial.println("Press BOOT button to start solving...");
}

void initSensors() {
    pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(LEFT_TRIG_PIN, OUTPUT); pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT); pinMode(RIGHT_ECHO_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LINE_SENSOR_PIN), onLineCrossed, FALLING);
}

void startSolving() {
    // Reset state for new run
    memset(wallMap, 0, sizeof(wallMap));
    memset(visited, false, sizeof(visited));
    currentX = 4;
    currentY = 4;
    heading = NORTH;
    
    Serial.printf("Starting maze solve: (%d,%d) -> (%d,%d)\n", currentX, currentY, GOAL_X, GOAL_Y);
    
    floodFill();
    robotState = STATE_THINKING;
}

// ======================= MAIN LOOP =======================

void loop() {
    // Start solving when Boot button pressed in IDLE state
    if (robotState == STATE_IDLE && digitalRead(0) == LOW) {
        delay(500);  // Debounce
        startSolving();
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
                
                currentX = 4; 
                currentY = 4;
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
    float distFront = readDistanceFront();
    float distLeft = readDistanceLeft();
    float distRight = readDistanceRight();
    
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
    else if (turnDiff == 2) turn180();  // Single smooth 180 instead of two 90s
    else if (turnDiff == 3) turnLeft90();
    
    heading = bestAction;
    // No delay needed - active braking provides settling time

    // Drive 1 Cell
    driveOneCell();
    
    // Update Position
    if (heading == NORTH) currentY--;
    else if (heading == SOUTH) currentY++;
    else if (heading == EAST) currentX++;
    else if (heading == WEST) currentX--;
    
    Serial.printf("Now at position: (%d, %d)\n", currentX, currentY);
}

void driveOneCell() {
    cellCrossed = false; 
    
    // 1. KICK START + BLIND DRIVE to leave intersection
    Serial.println("Driving cell...");
    kickStart();  // Full power pulse to overcome friction
    
    unsigned long blindStart = millis();
    while (millis() - blindStart < 200) {
        driveForward();
    }
    
    // 2. ACTIVE SENSOR DRIVE - look for line
    unsigned long startTime = millis();
    
    while (!cellCrossed && (millis() - startTime < MAX_CELL_TIME)) {
        driveForward();
        
        if (digitalRead(LINE_SENSOR_PIN) == LOW) {
            brakeMotors();  // Snap stop on line
            Serial.println("Line found - BRAKE");
            return;
        }
        delay(5); 
    }
    
    brakeMotors();  // Stop at end
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
    // Track directions for active braking
    lastLeftDir = (L > 0) ? 1 : (L < 0) ? -1 : 0;
    lastRightDir = (R > 0) ? 1 : (R < 0) ? -1 : 0;
    
    if (L > 0) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); }
    else { digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH); L = -L; }
    analogWrite(MOTOR_ENA, constrain(L, 0, 255));

    if (R > 0) { digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW); }
    else { digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH); R = -R; }
    analogWrite(MOTOR_ENB, constrain(R, 0, 255));
}

void stopMotors() { 
    digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
    analogWrite(MOTOR_ENA, 0);
    analogWrite(MOTOR_ENB, 0);
    lastLeftDir = 0;
    lastRightDir = 0;
}

// ACTIVE BRAKING - Reverses motor direction briefly to stop instantly
void brakeMotors() {
    int brakeL = -lastLeftDir;
    int brakeR = -lastRightDir;
    
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
    setMotors(BASE_SPEED, BASE_SPEED);
}

// Helper function to normalize angle difference to -180 to +180
float normalizeAngle(float diff) {
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

// ======================= SNAP TURN FUNCTIONS =======================

void turnRight90() {
    Serial.println(">> SNAP RIGHT 90");
    
    mpu.update();
    float start = mpu.getAngleZ();
    float target = start + 90.0;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float diff = normalizeAngle(target - mpu.getAngleZ());
        
        if (abs(diff) < TOLERANCE_90) {
            brakeMotors();
            break;
        }
        
        int speed = (abs(diff) > SLOW_ZONE_90) ? SPEED_FAST : SPEED_SLOW;
        setMotors(speed, -speed);
    }
    
    if (millis() - startTime >= TURN_TIMEOUT) stopMotors();
    
    delay(50);
    mpu.update();
    Serial.printf("Done. Angle: %.1f\n", mpu.getAngleZ());
}

void turnLeft90() {
    Serial.println("<< SNAP LEFT 90");
    
    mpu.update();
    float start = mpu.getAngleZ();
    float target = start - 90.0;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float diff = normalizeAngle(target - mpu.getAngleZ());
        
        if (abs(diff) < TOLERANCE_90) {
            brakeMotors();
            break;
        }
        
        int speed = (abs(diff) > SLOW_ZONE_90) ? SPEED_FAST : SPEED_SLOW;
        setMotors(-speed, speed);
    }
    
    if (millis() - startTime >= TURN_TIMEOUT) stopMotors();
    
    delay(50);
    mpu.update();
    Serial.printf("Done. Angle: %.1f\n", mpu.getAngleZ());
}

void turn180() {
    Serial.println(">> SNAP U-TURN 180");
    
    mpu.update();
    float start = mpu.getAngleZ();
    float target = start + 180.0;
    
    unsigned long startTime = millis();
    
    while (millis() - startTime < TURN_TIMEOUT) {
        mpu.update();
        float diff = normalizeAngle(target - mpu.getAngleZ());
        
        if (abs(diff) < TOLERANCE_180) {
            brakeMotors();
            break;
        }
        
        int speed = (abs(diff) > SLOW_ZONE_180) ? SPEED_FAST : SPEED_SLOW;
        setMotors(speed, -speed);
    }
    
    if (millis() - startTime >= TURN_TIMEOUT) stopMotors();
    
    delay(50);
    mpu.update();
    Serial.printf("Done. Angle: %.1f\n", mpu.getAngleZ());
}



// NewPing-based distance readings with median filtering
float readDistanceFront() {
    unsigned int distance = sonarFront.ping_median(SENSOR_SAMPLES);
    if (distance == 0) return 999.0;  // No echo received
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

void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    
    Serial.println("Initializing MPU6050...");
    
    if (mpu.begin() != 0) {
        Serial.println("MPU6050 FAILED!");
        while(1) { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); delay(100); }
    }
    
    Serial.println("Calibrating gyro - keep still!");
    delay(1000);
    mpu.calcOffsets();
    Serial.println("MPU6050 ready!");
}



void IRAM_ATTR onLineCrossed() {
    unsigned long now = millis();
    if (now - lastPulse > 200) {
        cellCrossed = true;
        lastPulse = now;
    }
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
    EEPROM.get(addr, magic);a
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
