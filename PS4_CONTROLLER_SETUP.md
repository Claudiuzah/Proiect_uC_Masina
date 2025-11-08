# PS4 Controller Setup Guide

## 📋 Requirements

- PS4 DualShock 4 controller
- ESP32 development board
- USB cable (for initial pairing)
- Arduino IDE with ESP32 support

## 🔧 Step 1: Install PS4Controller Library

### Arduino IDE:
1. Open Arduino IDE
2. Go to **Tools → Manage Libraries**
3. Search for **"PS4Controller"**
4. Install **"PS4Controller" by aed3**

### Alternative (Manual):
```bash
cd ~/Documents/Arduino/libraries
git clone https://github.com/aed3/PS4-esp32.git
```

## 🎮 Step 2: Get PS4 Controller MAC Address

### Method 1: Using SixaxisPairTool (Windows)
1. Download SixaxisPairTool from: https://github.com/user-attachments/files/15892942/SixaxisPairToolSetup-1.3.2.zip
2. Connect PS4 controller via USB
3. Run SixaxisPairTool.exe
4. Your controller's MAC address will be displayed
5. **Write down the MAC address** (format: XX:XX:XX:XX:XX:XX)

### Method 2: Using Bluetooth Settings (Windows 10/11)
1. Connect controller via USB
2. Open **Settings → Bluetooth & devices**
3. Look for "Wireless Controller"
4. Right-click → Properties → Find MAC address

### Method 3: Using Terminal (Linux/Mac)
```bash
# Linux
hcitool scan

# Mac
system_profiler SPBluetoothDataType
```

## 🔌 Step 3: Get ESP32 MAC Address (Optional)

If you want to set a specific MAC for the ESP32:

1. Upload this test sketch to ESP32:
```cpp
#include "esp_bt_main.h"
#include "esp_bt_device.h"

void setup() {
  Serial.begin(115200);
  btStart();
  Serial.print("ESP32 MAC: ");
  Serial.println(ESP.getEfuseMac(), HEX);
}

void loop() {}
```

2. Open Serial Monitor at 115200 baud
3. Note the MAC address

## ⚙️ Step 4: Configure the Code

### Option A: Let ESP32 Accept Any Controller (Easiest)
In `ESP32_PS4_Controller.ino`, use:
```cpp
PS4.begin();  // No MAC address - accepts first controller
```

### Option B: Pair to Specific MAC (More Secure)
In `ESP32_PS4_Controller.ino`, line ~155, change:
```cpp
PS4.begin("01:02:03:04:05:06");  // Replace with your ESP32's MAC
```

## 📤 Step 5: Upload Code to ESP32

1. Open `ESP32_PS4_Controller.ino` in Arduino IDE
2. Select board: **Tools → Board → ESP32 Dev Module**
3. Select correct COM port
4. Click **Upload**
5. Wait for "Done uploading"

## 🔗 Step 6: Pair PS4 Controller

### First-Time Pairing:

1. **Disconnect** PS4 controller from USB
2. Make sure ESP32 is powered and running the code
3. On PS4 controller, press and hold:
   - **PS button** (center PlayStation logo)
   - **Share button** (small button to the left of touchpad)
4. Hold both buttons for **3-5 seconds**
5. Light bar will start **flashing white** rapidly
6. After a few seconds, light bar turns **solid blue** → Connected! ✅
7. ESP32 Serial Monitor should show: "PS4 Controller Connected!"

### If Pairing Fails:

**Reset the controller:**
1. Find the small reset button on the back (near L2)
2. Use a paperclip to press it for 5 seconds
3. Try pairing again

**Force forget previous pairings:**
1. Connect controller to PC via USB
2. Use SixaxisPairTool to change master address to ESP32's MAC
3. Disconnect USB and try wireless pairing

## 🎮 Step 7: Test Controls

### Controller Mapping:

| Control | Action |
|---------|--------|
| **Left Stick UP** | Forward |
| **Left Stick DOWN** | Backward |
| **Left Stick LEFT** | Turn Left |
| **Left Stick RIGHT** | Turn Right |
| **Right Stick LEFT/RIGHT** | Rotate in place |
| **L1 Button** | Rotate Left |
| **R1 Button** | Rotate Right |
| **Circle Button (○)** | Emergency Stop |
| **Triangle Button (△)** | Speed Up |
| **Cross Button (✕)** | Speed Down |

### Visual Feedback:
- **ESP32 LED** blinks when sending commands
- **Controller light bar** turns blue when connected
- **Serial Monitor** shows commands being sent

## 🔍 Troubleshooting

### Controller Won't Connect
- ✅ Make sure ESP32 code is uploaded and running
- ✅ Controller is in pairing mode (flashing white)
- ✅ No other Bluetooth devices interfering
- ✅ ESP32 is within 10 meters of controller
- ✅ Try resetting controller with reset button

### Controller Connects but Car Doesn't Move
- ✅ Check UART wiring: ESP32 GPIO17 → STM32 PA10
- ✅ Check common ground: ESP32 GND ↔ STM32 GND
- ✅ Verify STM32 is running and ready
- ✅ Check battery voltage (needs >6V)
- ✅ Open Serial Monitor to see if commands are being sent

### Controller Disconnects Randomly
- ✅ Check controller battery level (charge it!)
- ✅ Reduce distance between controller and ESP32
- ✅ Avoid WiFi interference (turn off WiFi on ESP32)
- ✅ Power ESP32 from stable power source

### Serial Monitor Shows Errors
```
Bluetooth is not enabled!
```
→ Go to **Tools → Partition Scheme → Minimal SPIFFS** (allows Bluetooth)

```
PS4 Controller not found
```
→ Library not installed. Go to Tools → Manage Libraries → Install PS4Controller

### Latency/Lag Issues
- ✅ Reduce UART debug prints in code
- ✅ Increase UART baud rate (already at 115200)
- ✅ Check for electromagnetic interference
- ✅ Ensure controller is fully charged

## 🔋 Battery Life

- **PS4 Controller:** ~4-8 hours
- **ESP32:** Powered via USB or battery
- **Motors:** Use 2x 18650 (7.4V) for best performance

## 📊 Serial Monitor Debug Output

When working correctly, you should see:
```
========================================
ESP32 PS4 Controller for STM32 Car
========================================

[1] LED initialized
[2] UART initialized:
    TX: GPIO17 -> STM32 PA10 (RX)
    RX: GPIO16 <- STM32 PA9 (TX)
    Baud: 115200

[3] Initializing PS4 Controller...

========================================
READY!
========================================
Waiting for PS4 controller...

PS4 Controller Connected!
Sending: W
Sending: X
Sending: A
Sending: D
Controller Battery: 87%
```

## 🎯 Pro Tips

1. **Calibrate joystick deadzone** - Adjust `DEADZONE` in code (default 20)
2. **Smooth control** - Use joystick for gradual speed, buttons for quick actions
3. **Battery monitoring** - Controller battery % shows in Serial Monitor every 10s
4. **Auto-stop safety** - Car stops if no input for 500ms
5. **LED feedback** - ESP32 LED blinks on every command sent

## 🔄 Re-pairing After Changes

If you modify the code or reset ESP32:
1. PS4 controller should auto-reconnect
2. If not, press **PS + Share** again for 3 seconds

## 📝 Advanced: Custom Button Mapping

Edit the `notify()` function in `ESP32_PS4_Controller.ino`:

```cpp
if (PS4.Square()) {
  sendToSTM32('T');  // Send test command
}

if (PS4.L2()) {
  // L2 trigger - could use analog value
  int trigger = PS4.L2Value();  // 0-255
  // Map to speed or special function
}
```

Available buttons:
- `PS4.Square()`, `PS4.Triangle()`, `PS4.Circle()`, `PS4.Cross()`
- `PS4.Up()`, `PS4.Down()`, `PS4.Left()`, `PS4.Right()` (D-pad)
- `PS4.L1()`, `PS4.L2()`, `PS4.R1()`, `PS4.R2()`
- `PS4.Share()`, `PS4.Options()`, `PS4.PSButton()`
- `PS4.Touchpad()`

---

**Enjoy controlling your micromouse car with a PS4 controller!** 🎮🚗💨
