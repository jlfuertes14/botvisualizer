/*
 * Motor control for Maze Solver Bot
 * TB6612FNG dual motor driver
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

// PWM channels
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ======================= INITIALIZATION =======================

void initMotors() {
    // Motor control pins
    pinMode(MOTOR_STBY, OUTPUT);
    pinMode(MOTOR_AIN1, OUTPUT);
    pinMode(MOTOR_AIN2, OUTPUT);
    pinMode(MOTOR_BIN1, OUTPUT);
    pinMode(MOTOR_BIN2, OUTPUT);
    
    // Setup PWM
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWMA, PWM_CHANNEL_A);
    ledcAttachPin(MOTOR_PWMB, PWM_CHANNEL_B);
    
    // Enable motor driver
    digitalWrite(MOTOR_STBY, HIGH);
    
    Serial.println("Motors initialized");
}

// ======================= BASIC MOTOR CONTROL =======================

void setMotorA(int speed) {
    if (speed > 0) {
        digitalWrite(MOTOR_AIN1, HIGH);
        digitalWrite(MOTOR_AIN2, LOW);
        ledcWrite(PWM_CHANNEL_A, min(speed, 255));
    } else if (speed < 0) {
        digitalWrite(MOTOR_AIN1, LOW);
        digitalWrite(MOTOR_AIN2, HIGH);
        ledcWrite(PWM_CHANNEL_A, min(-speed, 255));
    } else {
        digitalWrite(MOTOR_AIN1, LOW);
        digitalWrite(MOTOR_AIN2, LOW);
        ledcWrite(PWM_CHANNEL_A, 0);
    }
}

void setMotorB(int speed) {
    if (speed > 0) {
        digitalWrite(MOTOR_BIN1, HIGH);
        digitalWrite(MOTOR_BIN2, LOW);
        ledcWrite(PWM_CHANNEL_B, min(speed, 255));
    } else if (speed < 0) {
        digitalWrite(MOTOR_BIN1, LOW);
        digitalWrite(MOTOR_BIN2, HIGH);
        ledcWrite(PWM_CHANNEL_B, min(-speed, 255));
    } else {
        digitalWrite(MOTOR_BIN1, LOW);
        digitalWrite(MOTOR_BIN2, LOW);
        ledcWrite(PWM_CHANNEL_B, 0);
    }
}

void setMotors(int leftSpeed, int rightSpeed) {
    setMotorA(leftSpeed);   // Left motor
    setMotorB(rightSpeed);  // Right motor
}

void stopMotors() {
    setMotors(0, 0);
}

// ======================= MOVEMENT FUNCTIONS =======================

void driveForward(int speed = BASE_SPEED) {
    setMotors(speed, speed);
}

void driveBackward(int speed = BASE_SPEED) {
    setMotors(-speed, -speed);
}

void turnLeftInPlace(int speed = TURN_SPEED) {
    setMotors(-speed, speed);
}

void turnRightInPlace(int speed = TURN_SPEED) {
    setMotors(speed, -speed);
}

// ======================= PID STRAIGHT DRIVING =======================

float targetHeading = 0;
float lastError = 0;
float integral = 0;

void lockHeading(float heading) {
    targetHeading = heading;
    lastError = 0;
    integral = 0;
}

void driveForwardWithCorrection() {
    extern float readMPU6050Yaw();  // From mpu6050.h
    
    float currentHeading = readMPU6050Yaw();
    float error = targetHeading - currentHeading;
    
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

#endif
