/*
 * MPU6050 IMU handling for Maze Solver Bot
 * Used for precise turning and heading tracking
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include "config.h"
#include "motors.h"

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// Calibration
float gyroZOffset = 0;
float currentYaw = 0;
unsigned long lastMPUUpdate = 0;

// ======================= INITIALIZATION =======================

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

// ======================= RAW READING =======================

float readRawGyroZ() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    
    int16_t raw = (Wire.read() << 8) | Wire.read();
    return raw / 131.0;  // Convert to °/s (±250°/s range)
}

// ======================= YAW TRACKING =======================

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

// ======================= TURNING FUNCTIONS =======================

void turnRight90() {
    extern void lockHeading(float);
    
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
    extern void lockHeading(float);
    lockHeading(currentYaw);
    delay(100);
}

#endif
