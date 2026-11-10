# SmartServo: High-Performance Closed-Loop Bus Servo System

An open-source, high-precision smart bus servo platform based on the **ElectronBot** micro-servo architecture, upgraded with robust industrial-grade I2C communication, zero-lockup hardware bus recovery, friction deadband feedforward, and a real-time calibration GUI.

---

## 🚀 Key Features & Architectural Improvements

| Area | Original ElectronBot Implementation | SmartServo Upgraded Architecture |
|---|---|---|
| **I2C Slave Stability** | Suffered from bus lockups due to interrupt collisions and unhandled slave clock stretching (Issues #131, #238). | **Zero-lockup architecture**: Priority 0 I2C & DMA interrupts, STM32F0 hardware register compliance (`I2C_OAR1`), and automatic DMA Channel 3 reset on partial-write aborts. |
| **I2C Bus Recovery** | None; master firmware blocked indefinitely in `do { ... } while(state != HAL_OK)` loops. | **Automated 9-clock bus unwedge** and zero-byte frame resync inside `SmartServoComm::recoverBus()` on the ESP32-S3 bridge, plus background GUI watchdog recovery. |
| **Position Accuracy** | ~10° angular error due to heavy mechanical gear stiction (~1:250 SG90 gearbox ratio). | **Friction feedforward boost**: $\pm 210$ PWM counts outside a $1.0^\circ$ deadband, dropping settled steady-state error to **$\le 1.5^\circ$** with zero buzzing/hunting inside the deadband. |
| **ADC Sampling** | 1.5-cycle sampling time with a DMA memory pointer increment bug (`MINC`) causing RAM drift and 0V readings during motion. | **Circular DMA mode** with 239.5-cycle sampling time, eliminating RAM drift and contact micro-bounce, coupled with a 3-sample gap safety filter. |
| **PID Parameter Sync** | One-way writes only; GUI displayed hardcoded defaults and could not read Flash gains. | **Full two-way read/write**: Extended protocol with opcodes `0x16`–`0x19` (`GET_KP`, `GET_KI`, `GET_KV`, `GET_KD`) automatically loaded on connection. |
| **Control Interface** | Basic test script. | **Full-featured Python GUI**: Live oscilloscope (Target vs Actual), visual dial gauge, single-click 0°/180° ADC calibration, PID tuning, and automated 60ms sweep mode. |

---

## 🛠️ Hardware Architecture & Wiring

The system consists of three main hardware components:
1. **STM32F030F4P6 Servo Controller**: Mounts directly inside the modified micro-servo case. Runs the 200 Hz PID closed-loop position control and acts as an I2C slave.
2. **Seeed Studio XIAO ESP32-S3 USB Bridge**: Connects to the host PC via USB CDC serial and bridges commands to the I2C bus at 100 kHz.
3. **Host Computer**: Runs `smartservo_py` (Python SDK and Tkinter GUI).

```
                      +-----------------------------+
                      |       Host Computer         |
                      |   smartservo_py (GUI/SDK)   |
                      +--------------+--------------+
                                     | USB CDC Serial (115200 baud)
                                     v
                      +-----------------------------+
                      |   Seeed Studio XIAO ESP32-S3|
                      |     (Master I2C Bridge)     |
                      +------+---------------+------+
                             | SDA           | SCL
                       (GPIO 5)       (GPIO 6)
                             |               |
       +---------------------+---------------+---------------------+
       |                                                           |
       v (I2C Bus: 100 kHz, 4.7k Pull-ups to 3.3V)                 v
+---------------+                                           +---------------+
| SmartServo #1 |                                           | SmartServo #N |
| STM32F030F4P6 | (Address 6)                               | STM32F030F4P6 | (Address N)
| L9110S Driver |                                           | L9110S Driver |
+---------------+                                           +---------------+
```

### Pinout Mapping

#### STM32F030F4P6 (TSSOP-20)
| Pin | Function | Connection |
|---|---|---|
| **PA4** (Pin 10) | ADC_IN4 | Potentiometer Wiper (with 0.1 µF filter capacitor to GND) |
| **PA6** (Pin 12) | TIM3_CH1 | L9110S H-Bridge Motor Input A (PWM) |
| **PA7** (Pin 13) | TIM3_CH2 | L9110S H-Bridge Motor Input B (PWM) |
| **PA9** (Pin 17) | I2C1_SCL | I2C Clock (with 4.7 k$\Omega$ pullup to 3.3V) |
| **PA10** (Pin 18) | I2C1_SDA | I2C Data (with 4.7 k$\Omega$ pullup to 3.3V) |
| **PA13 / PA14** | SWDIO / SWCLK | ST-Link V2 SWD Programming Header |
| **VDDA / VDD** | 3.3V Power | 3.3V from LDO with 0.1 µF bypass |
| **VSS** | Ground | Common Power Ground |

#### Seeed Studio XIAO ESP32-S3
| Pin | Function | Connection |
|---|---|---|
| **D4 (GPIO 5)** | I2C SDA | Connects to Servo I2C SDA |
| **D5 (GPIO 6)** | I2C SCL | Connects to Servo I2C SCL |
| **5V** | Power Out | Connects to Servo 5V Rail (with 330 µF bulk capacitor) |
| **GND** | Ground | Connects to Servo GND Rail |

#### 5-Pin Motor Connector (JST)
1. **Pin 1**: `VRef` (2.5V precision reference from TL431)
2. **Pin 2**: `ADC` (Potentiometer wiper to PA4)
3. **Pin 3**: `GND` (Potentiometer low-side ground)
4. **Pin 4**: `Mot+` (L9110S output pin 6)
5. **Pin 5**: `Mot-` (L9110S output pin 7)

---

## 📡 I2C Protocol Specification

Transactions use standard **5-byte packets**:
* **Master to Slave (Write)**: `[Opcode] [Float Byte 0] [Float Byte 1] [Float Byte 2] [Float Byte 3]` (Little-Endian float32).
* **Slave to Master (Read)**: `[Opcode] [Float Byte 0] [Float Byte 1] [Float Byte 2] [Float Byte 3]`.

### Opcode Table

| Opcode (Hex) | Command | Parameter / Payload | Return Value | Flash Committed? |
|---|---|---|---|---|
| `0x01` | `SET_ANGLE` | Target Angle ($0.0^\circ$ – $180.0^\circ$) | Current Measured Angle | No |
| `0x02` | `SET_VELOCITY` | Target Velocity Limit | Current Measured Velocity | No |
| `0x03` | `SET_TORQUE` | Torque Limit ($0.0$ – $1.0$) | Current Output Torque | No |
| `0x11` | `GET_ANGLE` | `0.0` (dummy) | Current Measured Angle | No |
| `0x12` | `GET_VELOCITY` | `0.0` (dummy) | Estimated Velocity (deg/s) | No |
| `0x13` | `GET_RAW_ADC` | `0.0` (dummy) | Raw ADC Value ($0$ – $4095$) | No |
| `0x14` | `GET_INVERT` | `0.0` (dummy) | Polarity ($1.0$ = Inverted, $0.0$ = Normal) | No |
| `0x15` | `GET_MAX_ANGLE`| `0.0` (dummy) | Max Mechanical Angle ($180.0$ or $360.0$) | No |
| `0x16` | `GET_KP` | `0.0` (dummy) | Proportional Gain ($K_p$) | No |
| `0x17` | `GET_KI` | `0.0` (dummy) | Integral Gain ($K_i$) | No |
| `0x18` | `GET_KV` | `0.0` (dummy) | Velocity Feedforward ($K_v$) | No |
| `0x19` | `GET_KD` | `0.0` (dummy) | Derivative / Damping ($K_d$) | No |
| `0x21` | `SET_NODE_ID` | New I2C 7-bit Address (e.g. $6.0$) | Current Angle | **Yes** |
| `0x22` | `SET_KP` | Position Gain $K_p$ | Current Angle | **Yes** |
| `0x23` | `SET_KI` | Integral Gain $K_i$ | Current Angle | **Yes** |
| `0x24` | `SET_KV` | Velocity Gain $K_v$ | Current Angle | **Yes** |
| `0x25` | `SET_KD` | Derivative Gain $K_d$ | Current Angle | **Yes** |
| `0x26` | `SET_TORQUE_LIMIT` | Max Torque ($0.0$ – $1.0$) | Current Angle | **Yes** |
| `0x27` | `SET_INIT_POS` | Default Boot Angle | Current Angle | **Yes** |
| `0x28` | `SET_ADC_MIN` | Raw ADC at $0^\circ$ | Current Angle | **Yes** |
| `0x29` | `SET_ADC_MAX` | Raw ADC at Max Angle | Current Angle | **Yes** |
| `0x2A` | `SET_MAX_ANGLE` | Max Angle ($180.0$ or $360.0$) | Current Angle | **Yes** |
| `0x30` | `SET_INVERT` | $1.0$ (Inverted) or $0.0$ (Normal) | Current Angle | **Yes** |
| `0xFF` | `SET_ENABLE` | $1.0$ (Enable) or $0.0$ (Disable) | Current Angle | No |

---

## 💻 Software Installation & Usage

### 1. Requirements

Ensure Python 3.9+ and PlatformIO are installed:
```bash
pip install pyserial
```

### 2. Flashing the Firmware

#### Flash STM32F030F4 Controller:
Connect ST-Link V2 to the SWD header and run:
```bash
pio run -d ServoDrive-fw-ll -e stm32f030f4 -t upload
```

#### Flash Seeed Studio XIAO ESP32-S3 Bridge:
Connect ESP32-S3 via USB and run:
```bash
pio run -d ESP32S3_ServoTester -t upload
```

### 3. Launching the GUI

Run the graphical tuner and monitor:
```bash
python3 smartservo_py/servo_gui.py
```

#### GUI Features:
* **Port Auto-Detection**: Automatically prioritizes `/dev/ttyACM*` or `/dev/ttyUSB*` ports.
* **Scan Bus**: Detects all connected servos on the I2C bus and selects the active node.
* **Dial Gauge & Scope**: Real-time 28 Hz tracking of commanded setpoint vs actual horn position.
* **One-Click Calibration**: Buttons to sample and save the minimum ($0^\circ$) and maximum ($180^\circ$) ADC readings directly to internal Flash.
* **Flash Gain Synchronization**: Automatically queries the active servo's saved PID gains on connect.
* **Auto Sweep**: Smoothly traverses $5^\circ \leftrightarrow 175^\circ$ at a 60ms cadence without bus flooding.

---

## ⚙️ Theory of Control & Anti-Stiction Feedforward

Micro-servos like the SG90 utilize high gear reduction ratios (~1:250) made of small plastic gears. This produces non-linear static friction (stiction) that stalls the motor when the proportional error is small:

$$\tau_{\text{stiction}} \approx 180\text{--}220 \text{ PWM counts (out of 1000)}$$

With a pure linear PID controller ($K_p = 35.0$), when the angular error is under $5^\circ$, the PID output is:
$$u_{\text{PID}} = 35 \times 5.0 = 175 \text{ PWM}$$
This is below the static breakaway threshold, causing the motor to stall 5°–10° away from the setpoint.

To overcome this without introducing high-frequency buzzing or motor heating, a **deadband feedforward controller** was implemented in [`ServoDrive-fw-ll/src/motor.cpp`](file:///home/gautham/Workspace/Projects/smartservo/ServoDrive-fw-ll/src/motor.cpp):

```cpp
// Deadband feedforward stiction compensation
if (errorPos > 1.0f)
{
    dce.output += 210.0f;  // Overcome positive breakaway friction
}
else if (errorPos < -1.0f)
{
    dce.output -= 210.0f;  // Overcome negative breakaway friction
}
else
{
    dce.output = 0.0f;     // Inside deadband: zero power to prevent heat & buzzing
}
```

This ensures fast, crisp arrivals at the target angle within $\pm 1.5^\circ$ of tolerance.

---

## 🔧 Hardware Troubleshooting Tips

1. **ADC Glitching / Reading 0 During Motion**:
   * Solder a **0.1 µF (100 nF) ceramic capacitor** between `PA4` and `GND`.
   * Add a **330 µF to 470 µF electrolytic bulk capacitor** across the +5V and GND power rails to absorb motor inrush current spikes.
2. **I2C Bus Not Responding**:
   * Ensure 4.7 k$\Omega$ pull-up resistors are installed between SCL $\rightarrow$ 3.3V and SDA $\rightarrow$ 3.3V.
   * If hot-plugging lines, the ESP32 bridge will automatically pulse 9 SCL recovery cycles and resync within 3 retry attempts.

---

## 📄 License

This project is licensed under the Apache 2.0 / BSD 3-Clause License, matching the underlying upstream components.
