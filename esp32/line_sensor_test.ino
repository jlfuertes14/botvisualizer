/*
 * Line Sensor Test - TCRT5000
 * Upload this to test your line sensor readings
 */

#define LINE_SENSOR_PIN 4
#define LINE_THRESHOLD 2048  // Adjust this based on readings

void setup() {
    Serial.begin(115200);
    pinMode(LINE_SENSOR_PIN, INPUT);
    
    Serial.println("\n=== Line Sensor Test ===");
    Serial.println("Move the sensor over different surfaces:");
    Serial.println("- White surface = HIGH value");
    Serial.println("- Black line = LOW value");
    Serial.println("- Adjust LINE_THRESHOLD based on readings\n");
}

void loop() {
    int sensorValue = analogRead(LINE_SENSOR_PIN);
    bool isOnLine = sensorValue < LINE_THRESHOLD;
    
    // Visual bar graph
    int barLength = map(sensorValue, 0, 4095, 0, 30);
    String bar = "";
    for (int i = 0; i < barLength; i++) bar += "█";
    for (int i = barLength; i < 30; i++) bar += "░";
    
    Serial.printf("Value: %4d | %s | %s\n", 
                  sensorValue, 
                  bar.c_str(),
                  isOnLine ? "⬛ ON LINE" : "⬜ OFF LINE");
    
    delay(100);
}
