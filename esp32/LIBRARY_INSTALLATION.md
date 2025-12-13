# Installing MPU6050_light Library

## Quick Installation Guide

### **Arduino IDE Method (Easiest):**

1. **Open Arduino IDE**

2. **Go to Library Manager:**
   - Click: `Tools` → `Manage Libraries...`
   - Or press: `Ctrl + Shift + I`

3. **Search for the library:**
   - Type: `MPU6050_light` in search box
   - Look for library by **rfetick**

4. **Install:**
   - Click the `Install` button
   - Wait for download to complete

5. **Verify:**
   - Go to: `Sketch` → `Include Library`
   - Look for `MPU6050_light` in the list

6. **Done!** ✅

---

## Why Use MPU6050_light Library?

### **Advantages Over Raw I2C:**

| Feature | Raw I2C (Old Code) | MPU6050_light Library |
|---------|-------------------|----------------------|
| **Setup Code** | ~50 lines | ~3 lines |
| **Calibration** | Manual offset calculation | `mpu.calcOffsets()` |
| **Filtering** | None | Built-in complementary filter |
| **Drift Compensation** | Manual deadzone | Automatic |
| **Ease of Use** | Complex | Simple |
| **Reliability** | Requires tuning | Works out-of-box |

### **Code Comparison:**

**Old Way (Raw I2C):**
```cpp
// Setup
Wire.begin(MPU6050_SDA, MPU6050_SCL);
Wire.beginTransmission(MPU6050_ADDR); 
Wire.write(0x6B); 
Wire.write(0); 
Wire.endTransmission(true);

// Manual calibration
float sum = 0;
for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    sum += raw / 131.0; 
    delay(5);
}
gyroZOffset = sum / 100.0;

// Read gyro
Wire.beginTransmission(MPU6050_ADDR); 
Wire.write(0x47); 
Wire.endTransmission(false);
Wire.requestFrom(MPU6050_ADDR, 2, true);
int16_t raw = (Wire.read() << 8) | Wire.read();
float gyroZ = (raw / 131.0) - gyroZOffset;
```

**New Way (Library):**
```cpp
// Setup
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

Wire.begin(MPU6050_SDA, MPU6050_SCL);
mpu.begin();
mpu.calcOffsets();  // Auto-calibrate

// Read gyro
mpu.update();
float gyroZ = mpu.getGyroZ();
float angleZ = mpu.getAngleZ();  // Integrated angle!
```

**Result:** 90% less code, better accuracy! 🎉

---

## Library Features

### **Available Methods:**

```cpp
// Initialization
mpu.begin();              // Initialize sensor
mpu.calcOffsets();        // Calibrate (robot must be still!)

// Reading Data
mpu.update();             // Update all readings (call in loop!)

// Get Gyro (degrees per second)
mpu.getGyroX();           // Pitch rate
mpu.getGyroY();           // Roll rate  
mpu.getGyroZ();           // Yaw rate (what we use for turning!)

// Get Angles (integrated from gyro + accel)
mpu.getAngleX();          // Pitch angle
mpu.getAngleY();          // Roll angle
mpu.getAngleZ();          // Yaw angle (perfect for turns!)

// Get Accelerometer (g-force)
mpu.getAccX();
mpu.getAccY();
mpu.getAccZ();

// Calibration
mpu.setGyroOffsets(x, y, z);     // Set custom offsets
mpu.setAccOffsets(x, y, z);      // Set custom offsets
```

### **What We Use for Turning:**

```cpp
// Get current yaw angle (integrated over time)
mpu.update();
float currentAngle = mpu.getAngleZ();

// For 90° turn:
float startAngle = mpu.getAngleZ();
float targetAngle = startAngle + 90.0;

while (abs(targetAngle - mpu.getAngleZ()) > 5.0) {
    mpu.update();  // Keep reading!
    // ... turn motors ...
}
```

---

## Troubleshooting Installation

### **Error: "MPU6050_light.h: No such file or directory"**

**Solution:**
1. Library not installed properly
2. Try manual installation method
3. Restart Arduino IDE
4. Check library is in: `Documents/Arduino/libraries/MPU6050_light/`

---

### **Error: "MPU6050 status: 1" or "Could not connect"**

**Solution:**
1. Check wiring:
   - SDA → Pin 32
   - SCL → Pin 33  
   - VCC → 3.3V or 5V
   - GND → GND

2. Check I2C address:
   - Default is `0x68`
   - If using AD0 pin high, use `0x69`

3. Test I2C scanner:
```cpp
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    Wire.begin(32, 33);  // SDA, SCL
    
    Serial.println("\nI2C Scanner");
    for (byte i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Device found at 0x%02X\n", i);
        }
    }
    Serial.println("Scan complete");
}

void loop() {}
```

Expected output: `Device found at 0x68`

---

### **Error: Compilation failed or warnings**

**Solution:**
1. Make sure ESP32 board is selected:
   - `Tools` → `Board` → `ESP32 Dev Module`
2. Update ESP32 board package:
   - `Tools` → `Board` → `Boards Manager`
   - Search "esp32", update to latest
3. Clear cache and recompile

---

## Next Steps

1. ✅ **Install MPU6050_light library**
2. ✅ **Upload `mpu6050_turn_test.ino`**
3. ✅ **Run calibration tests**
4. ✅ **Find optimal TURN_SPEED**
5. ✅ **Use calibrated values in main maze solver**

---

## Additional Resources

- **Library GitHub:** https://github.com/rfetick/MPU6050_light
- **Library Documentation:** See examples in `File` → `Examples` → `MPU6050_light`
- **Datasheet:** MPU6050 Product Specification (Google it)
- **I2C Tutorial:** https://learn.sparkfun.com/tutorials/i2c

---

**Once installed, the test program will work much better than raw I2C!** 🚀
