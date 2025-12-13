# MPU6050 Turning Accuracy Fixes

## Problems Identified

Your MPU6050-based turning had several issues causing inaccurate 90-degree turns:

### 1. **Uninitialized Timer (`lastMPUUpdate`)**
- **Issue**: `lastMPUUpdate` started at 0, causing a huge `dt` value on the first `updateYaw()` call
- **Impact**: First turn would get massive angle drift from incorrect time delta
- **Fix**: Initialize `lastMPUUpdate = millis()` after MPU6050 calibration

### 2. **Gyro Deadzone Too High**
- **Issue**: `abs(gyroZ) > 1.0` threshold ignored small rotations
- **Impact**: Slow turning movements weren't registered, causing cumulative error
- **Fix**: Lowered threshold to `0.5` degrees/second for better sensitivity

### 3. **No dt Capping**
- **Issue**: If `updateYaw()` wasn't called frequently, large `dt` values caused drift
- **Impact**: Irregular update timing led to angle calculation errors
- **Fix**: Added `if (dt > 0.1) dt = 0.02` to cap maximum time delta

### 4. **Poor Angle Wrapping Logic**
- **Issue**: Simple angle difference calculation failed near 0°/360° boundary
- **Impact**: Turning from 10° to 350° would try to turn 340° instead of -20°
- **Fix**: Created `normalizeAngleDiff()` helper function for proper angle difference calculation

### 5. **No Proportional Control**
- **Issue**: Constant speed turning caused overshoot
- **Impact**: Robot would overshoot target and oscillate back and forth
- **Fix**: Implemented proportional speed control that slows down when within 30° of target

### 6. **Tolerance Too Tight**
- **Issue**: 3° tolerance was hard to achieve with gyro noise and motor momentum
- **Impact**: Turn function would timeout instead of completing
- **Fix**: Increased tolerance to 5° for more reliable turn completion

## Changes Made

### File: `maze_solver_floodfill_standalone.ino`

#### 1. Improved `initMPU6050()` (Lines ~590-613)
```cpp
void initMPU6050() {
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
    
    delay(100);  // Give MPU time to stabilize ✅ NEW
    
    // Calibration loop...
    gyroZOffset = sum / 100.0;
    
    // ✅ NEW: Initialize timer AFTER calibration
    lastMPUUpdate = millis();
    currentYaw = 0;
    
    Serial.printf("MPU6050 calibrated. Gyro Z offset: %.2f\n", gyroZOffset);
}
```

#### 2. Improved `updateYaw()` (Lines ~616-635)
```cpp
void updateYaw() {
    unsigned long now = millis();
    float dt = (now - lastMPUUpdate) / 1000.0;
    lastMPUUpdate = now;
    
    // ✅ NEW: Prevent huge dt values
    if (dt > 0.1) dt = 0.02;
    
    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    float gyroZ = (raw/131.0) - gyroZOffset;
    
    // ✅ CHANGED: Lower deadzone (was 1.0, now 0.5)
    if(abs(gyroZ) > 0.5) currentYaw += gyroZ * dt;
    
    // Normalize angle
    while(currentYaw < 0) currentYaw += 360;
    while(currentYaw >= 360) currentYaw -= 360;
}
```

#### 3. New Helper Function `normalizeAngleDiff()`
```cpp
// ✅ NEW: Properly handle angle wrapping
float normalizeAngleDiff(float target, float current) {
    float diff = target - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}
```

#### 4. Completely Rewritten `turnRight90()` and `turnLeft90()`
```cpp
void turnRight90() {
    Serial.println("Starting RIGHT turn (90°)");
    
    updateYaw();  // Fresh read
    float startYaw = currentYaw;
    float target = startYaw + 90.0;
    
    // Normalize target to 0-360
    while (target >= 360) target -= 360;
    while (target < 0) target += 360;
    
    Serial.printf("Start: %.1f° → Target: %.1f°\n", startYaw, target);
    
    unsigned long startTime = millis();
   
    while(millis() - startTime < 3000) {  // 3s timeout
        updateYaw();
        
        float diff = normalizeAngleDiff(target, currentYaw);
        
        // ✅ CHANGED: 5° tolerance (was 3°)
        if (abs(diff) < 5.0) {
            break;
        }
        
        // ✅ NEW: Proportional speed control
        int turnSpeed = TURN_SPEED;
        if (abs(diff) < 30) {
            // Slow down when close to prevent overshoot
            turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
        }
        
        setMotors(turnSpeed, -turnSpeed);
        delay(10);
    }
    
    stopMotors();
    updateYaw();
    Serial.printf("RIGHT turn complete. Final: %.1f° (error: %.1f°)\n", 
                  currentYaw, normalizeAngleDiff(target, currentYaw));
    delay(100);  // Let robot settle
}
```

## Testing Recommendations

1. **Upload the fixed code** to your ESP32
2. **Monitor Serial output** during turns - you'll now see:
   - "Starting RIGHT/LEFT turn (90°)"
   - "Start: X° → Target: Y°"
   - "Turn complete. Final: Z° (error: E°)"
3. **Check turn accuracy**:
   - Error should be within ±5°
   - If consistently over/undershooting, you may need to adjust:
     - `TURN_SPEED` (line 60) - increase if turning too slowly
     - Proportional speed range (currently 80-140)
     - Gyro offset calibration

4. **If still having issues**, check:
   - MPU6050 is stable and mounted firmly
   - Wire connections are solid (especially ground)
   - Battery voltage is sufficient (low voltage = weak motors = inconsistent turns)
   - Motor torque is balanced (both motors should spin at same speed)

## Additional Tuning Parameters

If you need to fine-tune further, adjust these values:

```cpp
#define TURN_SPEED 140        // Base turning speed (increase if too slow)
```

In turn functions (lines ~548-552):
```cpp
if (abs(diff) < 30) {
    // Adjust these values:
    // - First number (80): minimum speed when very close to target
    // - Second number (30): distance threshold to start slowing down
    turnSpeed = map(abs(diff), 0, 30, 80, TURN_SPEED);
}
```

## Summary

The main issues were:
1. ❌ Timer initialization bug causing huge drift on first turn
2. ❌ Deadzone too high, ignoring small movements
3. ❌ No proportional control causing overshoot
4. ❌ Poor angle wrapping near 0°/360° boundary

All fixed! ✅ Your robot should now turn much more accurately.
