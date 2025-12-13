    /*
    * Simple Motor Test - L298N Motor Driver
    * Uses analogWrite() for simpler PWM control
    * 
    * Test sequence: Forward → Backward → Left → Right → Stop
    */

    // ======================= PIN DEFINITIONS =======================

    // L298N Motor Driver - Left Motor
    #define MOTOR_IN1 19
    #define MOTOR_IN2 21
    #define MOTOR_ENA 25 

    // L298N Motor Driver - Right Motor
    #define MOTOR_IN3 22
    #define MOTOR_IN4 23
    #define MOTOR_ENB 26

    // ======================= MOTOR CONSTANTS =======================

    #define BASE_SPEED 200   // Increased! (0-255) - try 220-255 if still slow
    #define TURN_SPEED 180

    // ======================= MOTOR FUNCTIONS =======================

    void initMotors() {
        // Motor direction control pins
        pinMode(MOTOR_IN1, OUTPUT);
        pinMode(MOTOR_IN2, OUTPUT);
        pinMode(MOTOR_IN3, OUTPUT);
        pinMode(MOTOR_IN4, OUTPUT);
        
        // PWM pins - analogWrite auto-configures these
        pinMode(MOTOR_ENA, OUTPUT);
        pinMode(MOTOR_ENB, OUTPUT);
        
        Serial.println("Motors initialized!");
    }

    void setMotorLeft(int speed) {
        if (speed > 0) {
            // Forward
            digitalWrite(MOTOR_IN1, HIGH);
            digitalWrite(MOTOR_IN2, LOW);
            analogWrite(MOTOR_ENA, constrain(speed, 0, 255));
        } else if (speed < 0) {
            // Backward
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, HIGH);
            analogWrite(MOTOR_ENA, constrain(-speed, 0, 255));
        } else {
            // Stop
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, LOW);
            analogWrite(MOTOR_ENA, 0);
        }
    }

    void setMotorRight(int speed) {
        if (speed > 0) {
            // Forward
            digitalWrite(MOTOR_IN3, HIGH);
            digitalWrite(MOTOR_IN4, LOW);
            analogWrite(MOTOR_ENB, constrain(speed, 0, 255));
        } else if (speed < 0) {
            // Backward
            digitalWrite(MOTOR_IN3, LOW);
            digitalWrite(MOTOR_IN4, HIGH);
            analogWrite(MOTOR_ENB, constrain(-speed, 0, 255));
        } else {
            // Stop
            digitalWrite(MOTOR_IN3, LOW);
            digitalWrite(MOTOR_IN4, LOW);
            analogWrite(MOTOR_ENB, 0);
        }
    }

    void setMotors(int leftSpeed, int rightSpeed) {
        setMotorLeft(leftSpeed);
        setMotorRight(rightSpeed);
    }

    void stopMotors() {
        setMotors(0, 0);
        Serial.println("STOP");
    }

    void driveForward(int speed) {
        setMotors(speed, speed);
        Serial.printf("FORWARD at speed %d\n", speed);
    }

    void driveBackward(int speed) {
        setMotors(-speed, -speed);
        Serial.printf("BACKWARD at speed %d\n", speed);
    }

    void turnLeft(int speed) {
        setMotors(-speed, speed);
        Serial.printf("TURN LEFT at speed %d\n", speed);
    }

    void turnRight(int speed) {
        setMotors(speed, -speed);
        Serial.printf("TURN RIGHT at speed %d\n", speed);
    }

    // ======================= SETUP =======================

    void setup() {
        Serial.begin(115200);
        Serial.println("\n=== Simple Motor Test (analogWrite) ===");
        
        initMotors();
        
        Serial.println("Starting motor test in 2 seconds...");
        Serial.printf("BASE_SPEED: %d (increase to 220-255 if too slow)\n", BASE_SPEED);
        delay(2000);
    }

    // ======================= MAIN LOOP =======================

    void loop() {
        Serial.println("\n--- Starting Test Sequence ---");
        
        // 1. Forward for 2 seconds
        driveForward(BASE_SPEED);
        delay(2000);
        
        // 2. Stop briefly
        stopMotors();
        delay(500);
        
        // 3. Backward for 2 seconds
        driveBackward(BASE_SPEED);
        delay(2000);
        
        // 4. Stop briefly
        stopMotors();
        delay(500);
        
        // 5. Turn left for 1 second
        turnLeft(TURN_SPEED);
        delay(1000);
        
        // 6. Stop briefly
        stopMotors();
        delay(500);
        
        // 7. Turn right for 1 second
        turnRight(TURN_SPEED);
        delay(1000);
        
        // 8. Full speed test!
        Serial.println("FULL SPEED TEST!");
        driveForward(255);
        delay(1500);
        
        // 9. Stop
        stopMotors();
        
        Serial.println("--- Test Complete! Waiting 3 seconds ---\n");
        delay(3000);
    }
