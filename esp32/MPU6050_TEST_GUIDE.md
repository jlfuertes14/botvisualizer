# MPU6050 Turn Test Guide

## What is this?

**`mpu6050_turn_test.ino`** is a standalone test program using the **MPU6050_light library** to:
- ✅ Test MPU6050 gyroscope accuracy
- ✅ Calibrate turning parameters
- ✅ Diagnose gyro drift issues
- ✅ Verify motor balance
- ✅ Find optimal `TURN_SPEED` value

**Benefits of using the library:**
- 🎯 Better calibration algorithm
- 🎯 Built-in filtering for smoother readings
- 🎯 Automatic offset calculation
- 🎯 Easier to use than raw I2C

**Use this BEFORE running the full maze solver!**

---

## Installation

### **Step 1: Install MPU6050_light Library**

**Method 1: Arduino Library Manager (Recommended)**
1. Open Arduino IDE
2. Go to: **Tools → Manage Libraries...**
3. Search for: **"MPU6050_light"**
4. Find library by **rfetick**
5. Click **Install**

**Method 2: Manual Installation**
1. Download from: https://github.com/rfetick/MPU6050_light
2. Extract to: `Documents/Arduino/libraries/`
3. Restart Arduino IDE

### **Step 2: Verify Installation**
Check that you can see it in: **Sketch → Include Library → MPU6050_light**

---

## How to Use

### **Step 1: Upload the Test Sketch**
1. Open `mpu6050_turn_test.ino` in Arduino IDE
2. Select your ESP32 board
3. Upload to robot
4. Open Serial Monitor (115200 baud)

### **Step 2: Calibration**
When you first power on:
```
=================================
MPU6050 TURNING TEST & CALIBRATION
=================================

Initializing motors...
Calibrating MPU6050...
⚠️  IMPORTANT: Keep robot PERFECTLY STILL!

✅ MPU6050 calibrated!
   Gyro Z offset: 2.34 °/s

✅ Ready for testing!
```

**Check the offset:**
- ✅ Good: -5 to +5
- ⚠️ Warning: -10 to +10
- ❌ Bad: > 10 or < -10 → Recalibrate!

### **Step 3: Test Turns**

#### **Method 1: Boot Button**
- Press BOOT button on ESP32
- Robot turns RIGHT 90°
- Results shown in Serial Monitor

#### **Method 2: Serial Commands**
Send these commands in Serial Monitor:

| Command | Action |
|---------|--------|
| `R` | Turn RIGHT 90° |
| `L` | Turn LEFT 90° |
| `C` | Recalibrate gyro |
| `S` | Show current yaw & statistics |
| `M` | Show menu |

---

## Understanding the Results

### **Good Turn Example:**
```
┌─────────────────────────────────┐
│   🔄 RIGHT TURN (90°)          │
└─────────────────────────────────┘
Start angle: 0.0°
Target angle: 90.0°
Turning...

--- RESULTS ---
Final angle: 88.7°
Error: -1.3° ✅ EXCELLENT
Turn time: 456 ms
Max gyro: 89.3 °/s
---------------
```

**What this means:**
- Error < 3° = ✅ **EXCELLENT** - Perfect calibration!
- Error < 5° = ✅ **GOOD** - Working well
- Error < 10° = ⚠️ **ACCEPTABLE** - Usable but could improve
- Error > 10° = ❌ **POOR** - Needs adjustment

### **Bad Turn Example (Timeout):**
```
┌─────────────────────────────────┐
│   🔄 RIGHT TURN (90°)          │
└─────────────────────────────────┘
Start angle: 0.0°
Target angle: 90.0°
Turning...

--- RESULTS ---
Final angle: 52.3°
Error: 37.7° ❌ POOR
Turn time: 3000 ms
Max gyro: 12.1 °/s
❌ TIMEOUT! Turn did not complete.
Possible issues:
  - Motors too weak
  - TURN_SPEED too low
  - Battery voltage low
  - Gyro malfunction
---------------
```

---

## Calibrating TURN_SPEED

If turns are consistently over/undershooting, adjust `TURN_SPEED`:

### **Turns Are Too SLOW (timeouts):**
```cpp
#define TURN_SPEED 140  // Increase to 160, 180, or 200
```

### **Turns OVERSHOOT (error > 10°):**
```cpp
#define TURN_SPEED 140  // Decrease to 120 or 100
```

### **Finding the Optimal Value:**
1. Start with `TURN_SPEED 140`
2. Do 5 test turns (press BOOT button 5 times)
3. Send `S` to see average error
4. Adjust and repeat until average error < 5°

---

## Checking for Gyro Drift

### **Test Procedure:**
1. Send `S` to see current yaw (should be ~0°)
2. **Turn robot by hand** exactly 90° clockwise
3. Send `S` again
4. Should show ~90°

**If it shows a very different number:**
- Gyro is drifting
- Deadzone might be too low/high
- MPU6050 might be faulty

---

## Adjusting Parameters

All tuning parameters are at the top of the file:

```cpp
#define TURN_SPEED 140        // Motor speed during turn
#define GYRO_DEADZONE 1.5     // Ignore rotation below this
#define TURN_TOLERANCE 5.0    // Accept turn within ±5°
#define TURN_TIMEOUT 3000     // Max time for turn (ms)
```

### **If Getting Drift:**
Increase `GYRO_DEADZONE`:
```cpp
#define GYRO_DEADZONE 2.0  // Try 2.0, 2.5, or 3.0
```

### **If Turns Are Too Precise/Slow:**
Increase `TURN_TOLERANCE`:
```cpp
#define TURN_TOLERANCE 7.0  // Accept ±7° instead of ±5°
```

---

## Statistics Tracking

After multiple turns, send `S` to see stats:

```
📊 Current Yaw: 359.2°
Turns completed: 8
Average error: 3.45°
```

**Good Performance:**
- Average error < 5° ✅
- Consistent errors (not random)
- Turn time 400-800ms

**Poor Performance:**
- Average error > 10° ❌
- Wildly varying errors
- Frequent timeouts

---

## Common Issues & Solutions

### **Issue 1: High Gyro Offset on Startup**
```
⚠️  WARNING: High offset! Robot might have moved during calibration.
```

**Solution:**
- Send `C` to recalibrate
- Make sure robot is on stable surface
- Don't touch robot during startup

---

### **Issue 2: Consistent Overshoot**
```
All turns show error: +15° to +20°
```

**Solution:**
- Robot is turning too fast
- Decrease `TURN_SPEED` by 20:
  ```cpp
  #define TURN_SPEED 120  // Was 140
  ```

---

### **Issue 3: Consistent Undershoot**
```
All turns show error: -15° to -20°
```

**Solution:**
- Robot is turning too slow
- Increase `TURN_SPEED` by 20:
  ```cpp
  #define TURN_SPEED 160  // Was 140
  ```

---

### **Issue 4: Random Errors (±30°)**
```
Turn 1: +25°
Turn 2: -18°
Turn 3: +32°
```

**Solution:**
- Gyro drift or vibration
- Check MPU6050 mounting (should be firm)
- Increase `GYRO_DEADZONE`:
  ```cpp
  #define GYRO_DEADZONE 2.5  // Was 1.5
  ```

---

### **Issue 5: Frequent Timeouts**
```
❌ TIMEOUT! Turn did not complete.
```

**Solutions:**
1. **Check battery voltage** (should be > 6V)
2. **Increase TURN_SPEED**:
   ```cpp
   #define TURN_SPEED 180
   ```
3. **Check motor connections**
4. **Test motors manually** (might be weak/damaged)

---

## Recommended Testing Procedure

### **Initial Calibration (Do Once):**
```
1. Upload code
2. Place robot on flat surface
3. Don't touch for 10 seconds
4. Check gyro offset in Serial Monitor
5. If offset > 10, send 'C' to recalibrate
```

### **Turn Accuracy Test (Do 5-10 times):**
```
1. Press BOOT button → RIGHT turn
2. Watch results
3. Send 'L' → LEFT turn
4. Watch results
5. Repeat 5-10 times
6. Send 'S' to see average error
```

### **Optimal Value Search:**
```
If average error > 5°:
  → Adjust TURN_SPEED
  → Repeat test
  → Compare average error
  → Continue until < 5°
```

---

## What to Look For

### **✅ GOOD Performance:**
- Calibration offset: -5 to +5
- Turn errors: < 5°
- Turn time: 400-800ms
- Max gyro during turn: 60-120 °/s
- No timeouts
- Consistent results

### **❌ POOR Performance:**
- Calibration offset: > 10
- Turn errors: > 10°
- Frequent timeouts
- Max gyro < 30 °/s (motors too weak)
- Wildly varying errors (drift)

---

## Using Results in Main Program

Once you find good values:

1. **Note your optimal TURN_SPEED**
2. **Note your GYRO_DEADZONE**
3. **Update `maze_solver_floodfill_standalone.ino`:**

```cpp
// At top of maze solver file:
#define TURN_SPEED 160  // Your calibrated value
```

```cpp
// In updateYaw() function:
if(abs(gyroZ) > 2.0) {  // Your calibrated deadzone
```

---

## Example Session

```
=================================
MPU6050 TURNING TEST & CALIBRATION
=================================

✅ MPU6050 calibrated!
   Gyro Z offset: 1.87 °/s

--- CONTROLS ---
Press BOOT button → Turn RIGHT 90°
Send 'L' → Turn LEFT 90°
Send 'C' → Recalibrate gyro
Send 'S' → Show current yaw
----------------

🔵 BOOT BUTTON PRESSED

┌─────────────────────────────────┐
│   🔄 RIGHT TURN (90°)          │
└─────────────────────────────────┘
Start angle: 0.0°
Target angle: 90.0°
Turning...

--- RESULTS ---
Final angle: 88.3°
Error: -1.7° ✅ EXCELLENT
Turn time: 512 ms
Max gyro: 82.4 °/s
---------------

> L

📍 LEFT TURN REQUESTED

┌─────────────────────────────────┐
│   🔄 LEFT TURN (90°)           │
└─────────────────────────────────┘
Start angle: 88.3°
Target angle: 358.3°
Turning...

--- RESULTS ---
Final angle: 0.7°
Error: 2.4° ✅ EXCELLENT
Turn time: 498 ms
Max gyro: 79.1 °/s
---------------

> S

📊 Current Yaw: 0.7°
Turns completed: 2
Average error: 2.05°
```

---

## Quick Reference

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Offset > 10 | Moved during calibration | Send `C` to recalibrate |
| Consistent overshoot +15° | TURN_SPEED too high | Decrease by 20 |
| Consistent undershoot -15° | TURN_SPEED too low | Increase by 20 |
| Random errors ±30° | Gyro drift/vibration | Increase GYRO_DEADZONE |
| Frequent timeouts | Weak motors or low battery | Increase TURN_SPEED, check power |
| Max gyro < 30 °/s | Motors not spinning | Check wiring |

---

**After calibrating, use these values in your main maze solver!** 🚀
