# STM32F429I Micromouse Car Project

**Course:** Microcontrollers - USV FIESC 3331A 2025-2026  
**Platform:** STM32F429I-DISCOVERY Board  
**Motor Driver:** L9100S Dual H-Bridge  
**Wireless Control:** ESP32 with Bluepad32 (PS4 Controller Support)

## 📋 Project Overview

This project implements a wireless-controlled micromouse car using the STM32F429I Discovery board with TouchGFX GUI. The car is controlled via Bluetooth using a PS4 DualShock 4 controller connected to an ESP32 running Bluepad32 firmware.

### Key Features

- ✅ PWM-based motor speed control (0-999 speed levels)
- ✅ **PS4 DualShock 4 wireless control via Bluepad32**
- ✅ Racing game-style controls (R2=forward, L2=backward)
- ✅ Analog stick steering with deadzone compensation
- ✅ Real-time speed adjustment while moving
- ✅ Multiple movement modes (forward, backward, turn, rotate in place)
- ✅ Visual LED feedback indicators
- ✅ FreeRTOS-based architecture
- ✅ TouchGFX GUI integration

## 🔧 Hardware Requirements

### Main Components
- **STM32F429I-DISCOVERY** board
- **L9100S** dual DC motor driver module
- **ESP32** development board (WiFi control)
- **2x DC motors** (for left and right wheels)
- **Battery pack** (recommended: 2x 18650 Li-ion 7.4V or 6x AA 9V)
- **Jumper wires** and breadboard

### Pin Connections

#### Motor Driver (L9100S) to STM32
| L9100S Pin | STM32 Pin | Function | Description |
|------------|-----------|----------|-------------|
| IA1 | PB4 | TIM3_CH1 | Motor A PWM (speed) |
| IA2 | PC10 | GPIO | Motor A direction |
| IB1 | PA7 | TIM3_CH2 | Motor B PWM (speed) |
| IB2 | PC11 | GPIO | Motor B direction |
| VCC | 7.4V | Power | Motor power supply |
| GND | GND | Ground | Common ground |

#### ESP32 to STM32 UART (PS4 Controller Commands)
| ESP32 Pin | STM32 Pin | Function |
|-----------|-----------|----------|
| **GPIO21** (TX) | PA10 (USART1_RX) | UART transmit |
| **GPIO22** (RX) | PA9 (USART1_TX) | UART receive |
| 5V (VIN) | 5V | Power (ESP32 powered by STM32) |
| GND | GND | **Common ground (CRITICAL!)** |

**Note:** GPIO16/17 (Serial2 default) conflicts with Bluepad32. Use GPIO21/22 instead.

#### Power Distribution
- **Battery (+)** → L9100S VCC (motor power)
- **Battery (-)** → Common ground
- **STM32 5V** → ESP32 VIN (ESP32 power)
- **All GND pins** → Common ground (STM32, ESP32, L9100S, Battery)

#### User Interface
| STM32 Pin | Function | Description |
|-----------|----------|-------------|
| PG13 | LD3 (Green LED) | Task running indicator |
| PG14 | LD4 (Red LED) | Command received indicator |
| PA0 | USER Button | User input (optional) |

### Power Connections
- **STM32**: Powered via USB (5V)
- **ESP32**: Powered from STM32 5V pin (or separate USB during debugging)
- **Motors**: External battery (recommended 2x 18650 Li-ion 7.4V, or 6x AA 9V)
- **⚠️ CRITICAL**: All grounds MUST be connected together (STM32 GND - ESP32 GND - Battery GND - Motor Driver GND)

**Note on Ground Loops:** When both STM32 and ESP32 are connected via USB to the same PC, a ground loop can occur causing UART communication issues. For final operation, power ESP32 from STM32's 5V pin.

## 📦 Software Requirements

### Development Tools
- **STM32CubeIDE** or **IAR EWARM** / **Keil MDK-ARM**
- **STM32CubeMX** (for peripheral configuration)
- **TouchGFX Designer** (optional, for GUI editing)
- **Arduino IDE 2.x** (for ESP32 programming)

### Dependencies
- **STM32 HAL Library**
- **FreeRTOS** (included in project)
- **TouchGFX Framework** (included in project)
- **ESP32 Arduino Core 2.0.14** (NOT 3.x - compatibility issues)
- **Bluepad32 Arduino Library** (via board manager)

## 🚀 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/STM32-Micromouse-Car.git
cd STM32-Micromouse-Car
```

### 2. STM32 Setup

#### Open Project
1. Open STM32CubeIDE
2. Import existing project: `Proiect_uC/`
3. Build the project (Project → Build All)

#### Flash to Board
1. Connect STM32F429I-DISCOVERY via USB
2. Click Run → Debug (or press F11)
3. The program will flash and start automatically

### 3. ESP32 Setup with Bluepad32

#### Install ESP32 Board Support with Bluepad32
1. Open Arduino IDE 2.x
2. Go to File → Preferences
3. Add to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json
   ```
4. Go to Tools → Board → Boards Manager
5. Search for "ESP32" and install version **2.0.14** (NOT 3.x)
6. **Install Bluepad32 Board Package**:
   - Add this URL to Board Manager URLs (on new line):
   ```
   https://raw.githubusercontent.com/ricardoquesada/esp-idf-arduino-bluepad32-template/main/package_esp32_bluepad32_index.json
   ```
   - In Boards Manager, search "bluepad32"
   - Install **"esp32 bluepad32 by Ricardo Quesada"**

#### Upload Code
1. Open `ESP32_Bluepad32_Controller/ESP32_Bluepad32_Controller.ino`
2. Select board: **Tools → Board → ESP32 Arduino → Bluepad32 for Arduino → ESP32 Dev Module**
3. Select correct COM port
4. Click Upload

#### Pair PS4 Controller
1. **First-time pairing**:
   - Hold **PS button + Share button** for 3 seconds
   - Controller will blink white rapidly
   - ESP32 will detect and pair automatically
   - Controller LED turns **blue** when connected
2. **Reconnecting**:
   - Press **PS button** only
   - Controller reconnects automatically if already paired
3. **Check Serial Monitor** (115200 baud) to verify connection:
   ```
   Device connected, index = 0
   Battery: 226/255
   MAC Address: F0:24:F9:59:AB:70
   ```

### 4. Hardware Assembly
1. **Connect motors** to L9100S outputs (MA+, MA-, MB+, MB-)
2. **Wire L9100S to STM32** according to pin table above
3. **Wire ESP32 to STM32** UART (TX→RX, RX→TX, GND→GND)
4. **Connect battery** to L9100S VCC and GND
5. **⚠️ Connect all grounds together** (critical!)
6. Add **10kΩ pull-down resistors** on motor control pins (optional, prevents startup twitch)

## 🎮 Usage

### PS4 Controller (Racing Game Controls)

**Control Mapping:**

| Button/Stick | Command | Action | Notes |
|--------------|---------|--------|-------|
| **R2 (Right Trigger)** | `1` | Forward | Throttle - hold for continuous forward |
| **L2 (Left Trigger)** | `2` | Backward | Brake/Reverse - hold for continuous backward |
| **Left Stick Left** | `3` | Turn Left | Analog steering while moving |
| **Left Stick Right** | `4` | Turn Right | Analog steering while moving |
| **L1 (Left Bumper)** | `5` | Rotate Left | Spin counter-clockwise in place |
| **R1 (Right Bumper)** | `6` | Rotate Right | Spin clockwise in place |
| **Triangle ▲** | `7` | Speed Up | +100 PWM, with 200ms delay |
| **Cross ✕** | `8` | Speed Down | -100 PWM, with 200ms delay |
| **Circle ○** | `0` | Stop | Emergency stop |

**Operating Tips:**
- **Deadzone**: 250 units to prevent stick drift
- **Trigger Threshold**: > 300 to activate (prevents false triggers)
- **Speed Control**: Default 900, range 500-1000
- **Battery Status**: Check Serial Monitor (ESP32) for controller battery level
- **Connection**: Blue LED on controller = connected, White blinking = pairing mode

### UART Protocol (for debugging)

The ESP32 sends single-byte commands (`0`-`8`) to STM32 via UART:
- **Baud Rate**: 9600
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **TX Frequency**: Commands sent every 50ms when active
- **Idle State**: `0` (stop) sent when no input detected

## 🔍 Project Structure

```
Proiect_uC_Masina/
├── README.md                          # This file
├── HARDWARE_GUIDE.md                  # Detailed hardware assembly
├── SOFTWARE_GUIDE.md                  # Software architecture details
├── changelog.txt                      # Version history
│
├── Proiect_uC/                        # STM32 main project
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h                 # Pin definitions & prototypes
│   │   │   ├── stm32f4xx_it.h        # Interrupt handlers
│   │   │   └── FreeRTOSConfig.h      # RTOS configuration
│   │   └── Src/
│   │       ├── main.c                 # Main application & motor control
│   │       ├── freertos.c             # FreeRTOS tasks
│   │       ├── stm32f4xx_it.c        # Interrupt implementations
│   │       └── stm32f4xx_hal_msp.c   # HAL MSP (peripheral init)
│   │
│   ├── Drivers/                       # STM32 HAL drivers
│   ├── Middlewares/                   # FreeRTOS & TouchGFX
│   ├── TouchGFX/                      # GUI application
│   ├── gcc/                           # GCC build files
│   ├── MDK-ARM/                       # Keil build files
│   └── EWARM/                         # IAR build files
│
└── ESP32_Bluepad32_Controller/
    └── ESP32_Bluepad32_Controller.ino # ESP32 PS4 controller interface
```

## 🧩 Motor Control Algorithm

### PWM Speed Control
- **Timer:** TIM3 running at 20kHz (Prescaler=89, ARR=999)
- **Speed range:** 0-999 (0% to 100% duty cycle)
- **Default speed:** 500 (50%)
- **Speed steps:** 100 (adjustable via +/- commands)

### L9100S Control Logic

#### Forward Motion
- IA1 = PWM (variable duty cycle)
- IA2 = LOW (0V)
- Result: Motor spins forward proportional to PWM

#### Backward Motion
- IA1 = PWM (inverted: 999 - speed)
- IA2 = HIGH (3.3V)
- Result: Motor spins backward proportional to PWM

#### Stop
- IA1 = 0 (no PWM)
- IA2 = LOW (0V)
- Result: Motor brake

### Movement Functions

```c
Motor_Forward(speed)      // Both motors forward
Motor_Backward(speed)     // Both motors backward
Motor_TurnLeft(speed)     // Left motor 50%, right motor 100%
Motor_TurnRight(speed)    // Left motor 100%, right motor 50%
Motor_RotateLeft(speed)   // Left backward, right forward
Motor_RotateRight(speed)  // Left forward, right backward
Motor_Stop()              // Both motors stop
```

## 🛠️ Configuration

### Adjusting Controller Deadzone (ESP32)
Edit `ESP32_Bluepad32_Controller.ino`:
```cpp
const int DEADZONE = 250;  // Increase if stick drift, decrease for sensitivity
```

### Adjusting Trigger Threshold
Edit `ESP32_Bluepad32_Controller.ino`:
```cpp
if (r2Value > 300) sendToSTM32('1');  // Change 300 to adjust sensitivity
```

### Adjusting Motor Speed Range
Edit `Core/Inc/main.h`:
```c
#define MOTOR_MAX_SPEED 999  // Maximum PWM value
#define MOTOR_MIN_SPEED 0    // Minimum PWM value
```

### Changing Default Speed
Edit `Core/Src/main.c` in `StartDefaultTask()`:
```c
uint16_t current_speed = 900;  // Change default (0-999), current is 900
```

### Adjusting Speed Steps
Modify `case '7'` and `case '8'` in `main.c`:
```c
case '7':  // Triangle - Speed up
  if(current_speed < 900) current_speed += 100;  // Change 100 to desired step
  break;
```

## 📊 System Architecture

### FreeRTOS Tasks
1. **defaultTask** (128 words stack, Normal priority)
   - UART command processing
   - Motor control execution
   - LED status indicators
   - Runs at 10Hz (100ms delay)

2. **GUI_Task** (8192 words stack, Normal priority)
   - TouchGFX interface updates
   - Display rendering
   - Touch event handling

### Communication Protocols
- **USART1:** 9600 baud, 8N1 (ESP32 ↔ STM32 command channel)
- **ESP32 Serial:** 115200 baud (USB debug output)
- **Bluetooth:** Classic Bluetooth 2.0 (PS4 controller)

### Interrupt Priorities (FreeRTOS)
- **DMA2D:** Priority 5
- **LTDC:** Priority 5
- **USART1:** Priority 5
- **TIM6:** Priority 15 (FreeRTOS tick)

### Important Notes
- **Printf Blocking:** Avoid using `printf()` in high-frequency loops - it can block FreeRTOS task switching and freeze TouchGFX
- **UART Non-blocking:** `HAL_UART_Receive()` uses 5ms timeout to prevent task starvation
- **Task Delay:** `osDelay(100)` in main loop is critical for FreeRTOS scheduler

## 🐛 Troubleshooting

### 🚨 ESP32 UART Not Working (GPIO16/17)
**Symptom:** ESP32 Serial Monitor shows "Sending: 1" but STM32 receives nothing or 0xFF garbage  
**Cause:** **Bluepad32 reserves GPIO16/17 (Serial2)** for internal use - UART transmission is blocked  
**Solution:**
- ✅ Use **GPIO21/22 with HardwareSerial(1)** instead of Serial2
- Change in code:
  ```cpp
  #define STM32_TX_PIN 21  // NOT 17
  #define STM32_RX_PIN 22  // NOT 16
  HardwareSerial STM32_SERIAL(1);  // NOT Serial2
  ```
- **Physically rewire:** GPIO21→PA10, GPIO22→PA9

### 🚨 TouchGFX Freezes / White Screen
**Symptom:** Screen freezes when receiving UART commands, LEDs stay on continuously  
**Cause:** **Printf() in high-frequency loops blocks FreeRTOS** task switching, starving TouchGFX task  
**Solution:**
- ✅ Remove `printf()` statements from main UART receive loop
- Keep only startup/initialization messages
- Use LED indicators instead of printf for real-time debugging
- Ensure `osDelay(100)` exists in main loop for task scheduling

### 🚨 Ground Loop / UART Interference
**Symptom:** TeraTerm input blocked, ESP32 commands garbled when both connected via USB  
**Cause:** Ground loop when STM32 USB and ESP32 USB both connected to same PC  
**Solution:**
- ✅ Power ESP32 from **STM32 5V pin** instead of separate USB
- Use only **one USB connection** for final operation (STM32 for power + debug)
- ESP32 USB only for initial programming, then disconnect
- **CRITICAL:** Connect all grounds together (STM32-ESP32-Battery-Motor Driver)

### 🚨 PS4 Controller Won't Connect
**Symptom:** Controller blinks white continuously, never turns blue  
**Cause:** Not in pairing mode or wrong board package  
**Solution:**
- Hold **PS + Share** for 3 seconds (not just PS button)
- Verify Bluepad32 board package installed (not just ESP32 core)
- Check ESP32 Serial Monitor (115200 baud) for connection messages
- Reset ESP32 and try pairing again
- Check controller battery level (charge if < 50%)

### Motors spin backward on reset
**Cause:** GPIO pins in undefined state during boot  
**Solution:** 
- Add 10kΩ pull-down resistors to IA1, IA2, IB1, IB2
- Code includes emergency motor stop at startup

### Motors spin slowly or inconsistently
**Cause:** Insufficient battery voltage/current  
**Solution:**
- Upgrade to 2x 18650 Li-ion (7.4V) - **recommended**
- Or use 6x AA batteries (9V) - better than AAA
- Check battery voltage under load (should be >6V)
- 4x AAA batteries (6V) are too weak for sustained operation

### PS4 Controller Stick Drift
**Symptom:** Car moves without touching controller  
**Cause:** Analog stick centers not exactly 0  
**Solution:**
- Increase `DEADZONE` value in ESP32 code (current: 250)
- Clean analog sticks (dust can cause drift)
- Check Serial Monitor for stick values at rest (should be near 0)

### STM32 Receives 0xFF Constantly
**Symptom:** UART RX shows 0xFF (255) even with no data  
**Cause:** RX pin floating (no pull-up), interpreting noise as high  
**Solution:**
- Verify ESP32 TX physically connected to STM32 RX (PA10)
- Check voltage on TX line with multimeter (should be 3.3V idle)
- Ensure ESP32 code actually sends data (check Serial Monitor)
- Add 10kΩ pull-up resistor to RX pin if problem persists

### Motors don't respond to commands
**Cause:** Battery too weak, wiring error, or missing common ground  
**Solution:**
- Verify L9100S connections match pin table exactly
- Test with higher PWM values (speed 700-900)
- Check battery voltage under load (>6V required)
- Verify common ground connections
- Test motor driver outputs with multimeter
- Verify USART1 baud rate is 9600 on both sides

## 🔄 Future Improvements

- [x] ~~Add Bluetooth control~~ - **Implemented with PS4 controller via Bluepad32**
- [x] ~~Simplify UART architecture~~ - **Completed: USART1 only**
- [ ] Upgrade to 2x 18650 Li-ion batteries (7.4V)
- [ ] Add ultrasonic sensors for obstacle detection
- [ ] Implement maze-solving algorithm
- [ ] Add battery voltage monitoring via ADC
- [ ] Implement PID speed control for consistent velocity
- [ ] Add encoder feedback for precise movements
- [ ] Add gyroscope for rotation angle control
- [ ] Implement autonomous navigation mode
- [ ] Add line-following capability with IR sensors
- [ ] Create data logging to SD card

## 📝 License

This project is developed for educational purposes as part of the Microcontrollers course at USV FIESC.

## 👥 Contributors

- **Student Name** - Main developer
- **Course:** Microcontrollers (FIESC 3331A)
- **Year:** 2025-2026
- **Institution:** USV (Universitatea Stefan cel Mare din Suceava)

## 📧 Contact

For questions or issues, please open an issue on GitHub or contact via university email.

## 🙏 Acknowledgments

- STMicroelectronics for STM32CubeIDE and HAL libraries
- TouchGFX team for GUI framework
- Espressif Systems for ESP32 Arduino core
- Ricardo Quesada for Bluepad32 library
- Course instructors and lab assistants

---

**⚠️ Safety Notes:**
- Never connect motor battery directly to STM32 (use L9100S as intermediary)
- Ensure proper heatsinking for L9100S if running high currents
- Use appropriate battery protection (especially for Li-ion/LiPo)
- Keep motor driver away from microcontroller to avoid EMI
- Add decoupling capacitors near motor driver (100nF + 10µF)

**📌 Version:** 2.0 - PS4 Controller Edition  
**📅 Last Updated:** January 2025

---

## 📋 Changelog

### Version 2.0 (January 2025)
- ✅ **Added PS4 DualShock 4 controller support** via Bluepad32
- ✅ **Racing game controls:** R2/L2 triggers for throttle/brake, analog stick steering
- ✅ **Simplified UART:** Removed USART2, using only USART1 (PA9/PA10)
- ✅ **Fixed GPIO conflict:** Moved ESP32 from GPIO16/17 (Serial2) to GPIO21/22 (HardwareSerial1)
- ✅ **Fixed TouchGFX freeze:** Removed printf() blocking from main loop
- ✅ **Resolved ground loop:** Power ESP32 from STM32 5V pin
- ✅ **Command protocol:** Simplified to number commands 0-8
- ⚠️ **Removed keyboard control:** Simplified to PS4 controller only

### Version 1.0 (November 2024)
- Initial release with WiFi web interface control
- Dual UART support (USART1 + USART2)
- Keyboard control via TeraTerm
- Basic motor control functions
