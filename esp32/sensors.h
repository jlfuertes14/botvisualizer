/*
 * Sensor handling for Maze Solver Bot
 * HC-SR04 Ultrasonic sensors for wall detection
 * IR sensor for line detection
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

// ======================= INITIALIZATION =======================

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

// ======================= ULTRASONIC READING =======================

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

// ======================= WALL DETECTION =======================

bool isFrontWall() {
    return readFrontDistance() < WALL_THRESHOLD;
}

bool isLeftWall() {
    return readLeftDistance() < WALL_THRESHOLD;
}

bool isRightWall() {
    return readRightDistance() < WALL_THRESHOLD;
}

// ======================= LINE SENSOR =======================

bool isOnLine() {
    return digitalRead(LINE_SENSOR_PIN) == LOW;  // Active low
}

#endif
