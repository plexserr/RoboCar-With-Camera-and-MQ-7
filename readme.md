# ESP32 Robot & WiFi Camera Project

This project consists of two main ESP32-based components that work independently:

1.  **🤖 Robot Controller:** An ESP32 dev kit that controls a **4WD robot chassis**. It receives commands via **Bluetooth** and simultaneously reads an **MQ-7 CO sensor**, hosting a local web server to display the live sensor data.
2.  **👁️ WiFi Camera Server:** An ESP32-CAM (AI-Thinker model) that hosts a high-performance web server, streaming live video to a web browser with a full control panel.
3.  **🧪 Sensor Calibration:** A utility sketch used to find the correct `R0` (clean air) value for the MQ-7 sensor.

---

## 1. 🤖 Robot Controller (`MotorDriver&Sensor.ino`)

This code runs on a standard ESP32 development board. It's designed to control a **4WD robot chassis** by driving two motor channels (e.g., left side and right side).

It receives commands via **Bluetooth** and simultaneously reads an **MQ-7 CO sensor**, hosting a local web server to display the live sensor data.

### Features

* **Dual Control:**
    * **Bluetooth RC:** Responds to single-character commands ('F', 'B', 'L', 'R', 'S') for movement and '1'-'q' for speed control.
    * **WiFi Web Server:** Hosts a simple webpage that auto-refreshes, showing the live MQ-7 sensor reading in PPM.
* [cite_start]**Sensor Integration:** Reads an MQ-7 CO sensor [cite: 4][cite_start], calculates its resistance (`Rs`), and converts the `Rs/R0` ratio into a PPM (Parts Per Million) value[cite: 35, 45].
* [cite_start]**Non-Blocking:** Uses `millis()` for sensor reads [cite: 72] [cite_start]and `server.handleClient()` [cite: 71] to ensure the Bluetooth controls and web server run smoothly without blocking each other.

> **⚠️ IMPORTANT CODE CORRECTION**
>
> Your `MotorDriver&Sensor.ino` file has a mismatch with your wiring. You must update the code to use **GPIO 13** for the Motor B speed pin.
>
> In `MotorDriver&Sensor.ino`, change this line:
> `const int MOTOR_B_PWM_PIN = 12; // ENB`
>
> ...to this:
> `const int MOTOR_B_PWM_PIN = 13; // ENB`

### 🔌 Robot Controller Pinout

This pinout is based on your connections for a 4WD chassis (with the **left motors wired in parallel** and the **right motors wired in parallel**) and an L298N driver.

| Component | ESP32 Pin (GPIO) | L298N / Device Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Motor A (Left Side)** | [cite_start]`GPIO 14` [cite: 23] | `ENA` | PWM Channel 0 |
| | [cite_start]`GPIO 27` [cite: 23] | `IN1` | |
| | [cite_start]`GPIO 26` [cite: 24] | `IN2` | |
| **Motor B (Right Side)** | `GPIO 13` | `ENB` | PWM Channel 1 **(Code must be 13)** |
| | [cite_start]`GPIO 25` [cite: 25] | `IN3` | |
| | [cite_start]`GPIO 33` [cite: 25] | `IN4` | |
| **MQ-7 Sensor** | [cite_start]`GPIO 34` [cite: 13] | `AOUT` | [cite_start]Must be an ADC1 pin (32-39) as WiFi uses ADC2. [cite: 12] |
| **Voltage Div** | [cite_start]`GPIO 34` [cite: 85] | (Center) | [cite_start]`[AOUT] -- [220Ω] -- [GPIO 34] -- [220Ω] -- [GND]` [cite: 16, 17, 85] |

---

## 2. 👁️ WiFi Camera Server (`Camraserver.ino`)

This code is designed for an ESP32-CAM module. It connects to WiFi and runs a robust web server (from `app_httpd.cpp`) that provides a live video stream and a full UI for adjusting camera settings.

### Features

* [cite_start]**High-Quality Stream:** Uses `PIXFORMAT_JPEG` and PSRAM to provide a high-resolution, high-quality video stream[cite: 106, 107].
* **Web Interface:** Serves a complete HTML page (from `camera_index.h`) with controls for resolution, quality, brightness, contrast, and special effects.
* **LED Flash Control:** Allows you to toggle the onboard flash LED via the web interface.

### 🔌 Camera Pinout (AI-Thinker Model)

This configuration is set in `board_config.h` by defining `CAMERA_MODEL_AI_THINKER`. The pins are defined in `camera_pins.h`.
---

## 3. 🧪 Sensor Calibration (`MQ-7Callibration.ino`)

This is a **one-time utility sketch**. [cite_start]Its only purpose is to find your specific sensor's `R0` value (its resistance in clean air)[cite: 77]. [cite_start]This value is crucial for accurate PPM calculation[cite: 82].

### How to Calibrate Your Sensor

1.  **Wire the Robot:** Connect your MQ-7 sensor to your **Robot ESP32** (not the camera) exactly as shown in the "Robot Controller Pinout" table (i.e., AOUT to `GPIO 34` with the 220Ω voltage divider [cite: 85]).
2.  **Burn-in (New Sensor):** If the sensor is new, power it on and let it run for at least 24 hours before calibrating[cite: 78].
3.  **Upload Sketch:** Upload the `MQ-7Callibration.ino` sketch to your Robot ESP32[cite: 79].
4.  **Find Clean Air:** Place the powered-on sensor in fresh, clean air (ideally outside)[cite: 80].
5.  **Monitor:** Open the Arduino Serial Monitor at **115200 baud**[cite: 80].
6.  **Wait:** Let the sketch run for 15-20 minutes. [cite_start]You will see the `Rs (Clean Air Resistance)` value printed repeatedly[cite: 81].
7.  **Get R0:** Once this `Rs` value stops changing and becomes stable, that is your `R0` value[cite: 81].
8.  **Update Robot Code:** Copy this stable `R0` value and paste it into the `R0_CLEAN_AIR_RESISTANCE` constant in your main `MotorDriver&Sensor.ino` file[cite: 82].

---

## 🚀 How to Run the Project

1.  **Calibrate:** Follow all the steps in the **Sensor Calibration** section above.
2.  **Configure Robot (`MotorDriver&Sensor.ino`):**
    * Open `MotorDriver&Sensor.ino`.
    * **Update `const int MOTOR_B_PWM_PIN = 12;` to `13`.**
    *Set `R0_CLEAN_AIR_RESISTANCE` [cite: 21] to the value you found in the calibration step.
    * Change `ssid` [cite: 9] [cite_start]and `password` [cite: 10] to your WiFi credentials.
    * Upload the code to your Robot ESP32.
3.  **Configure Camera (`Camraserver.ino`):**
    * Open `Camraserver.ino`.
    * Change `ssid` and `password` to your WiFi credentials[cite: 102].
    * Ensure `board_config.h` has `CAMERA_MODEL_AI_THINKER` selected.
    * Upload the code to your ESP32-CAM module.
4.  **Run:**
    * Power on both devices.
    * Open the Arduino Serial Monitor for the **Robot ESP32** to see its IP address[cite: 5, 59].
    *Open the Arduino Serial Monitor for the **ESP32-CAM** to see its IP address[cite: 121].
5.  **Connect:**
    * **For Sensor Data:** Open a browser and go to the Robot's IP address (e.g., `http://192.168.1.100`)[cite: 6].
    * **For Video Stream:** Open a browser and go to the Camera's IP address (e.g., `http://192.168.1.101`)[cite: 121].
    * **For Robot Control:** Use your phone to find and pair with the Bluetooth device named "MyRobot"[cite: 56]. [cite_start]Use a Bluetooth serial terminal app to send commands[cite: 62].
