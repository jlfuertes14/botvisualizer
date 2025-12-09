/*
 * Configuration file for Maze Solver Bot
 * Modify these values to match your hardware setup
 */

#ifndef CONFIG_H
#define CONFIG_H

// ======================= WIFI =======================
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

// IR Line Sensor (facing down, on back of robot)
#define LINE_SENSOR_PIN 34

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

#endif
