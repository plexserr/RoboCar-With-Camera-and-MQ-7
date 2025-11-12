# How to Calibrate Your MQ-7 Sensor (Finding R0)

This guide explains how to find your specific sensor's **R0** value (its baseline resistance in clean air). This is the **most important step** for getting accurate PPM (Parts Per Million) readings.

You will need the `MQ7_Calibration_Sketch.ino` file for this process.

### Step 1: "Burn-in" Your Sensor (Do This First!)

* **Why?** MQ-series sensors use a chemical heater that needs to stabilize. New sensors will give wildly incorrect readings until this is done.
* **How?** Leave your robot powered on, with the sensor connected, for at least **24 hours**. 48 hours is even better.
* You only need to do this **once** for the entire life of the sensor.

### Step 2: Configure the Calibration Code

Before uploading, open `MQ7_Calibration_Sketch.ino` and double-check the constants in the "SENSOR CONFIGURATION" section:

* `R1_DIVIDER_OHMS`
* `R2_DIVIDER_OHMS`
* `LOAD_RESISTOR_OHMS`

Make sure these values match your physical hardware. The pre-filled values (`220.0`, `220.0`, and `10000.0`) are based on our previous conversation.

### Step 3: Upload the Calibration Code

Upload the `MQ7_Calibration_Sketch.ino` sketch to your ESP32. This simple sketch *only* reads the sensor and prints its resistance, with no Wi-Fi or Bluetooth to interfere.

### Step 4: Run the Test

1.  **Find Clean Air:** Take your robot to the cleanest air possible. **Outside** is ideal. A well-ventilated room, far from kitchens or exhaust, is the next best choice.
2.  **Connect and Run:** Plug the ESP32 into your computer to power it on.
3.  **Open Serial Monitor:** Open the Arduino IDE Serial Monitor and set the baud rate to **115200**.
4.  **Wait:** Let the sketch run for **15 to 20 minutes**. The sensor needs this time to acclimate to the clean air and give a stable reading.

### Step 5: Find Your R0 Value

* Watch the `Rs (Clean Air Resistance)` value in the Serial Monitor.
* At first, this value may jump around. After 15-20 minutes, it will stabilize and only fluctuate slightly.

    ```
    // Serial Monitor Output (example)
    ...
    Rs (Clean Air Resistance): 27105.34
    Rs (Clean Air Resistance): 27012.19
    Rs (Clean Air Resistance): 27050.00
    Rs (Clean Air Resistance): 27048.12
    ```

* The stable value it settles on (e.g., `27050.00`) is your unique **R0 value**.
* **Write this number down!**

### Step 6: Update Your Main Code

1.  Go back and open your main robot file (`ESP32_Car_and_Sensor_Server.ino`).
2.  Find the `SENSOR CALIBRATION CONSTANTS` section near the top.
3.  Replace the default value with the `R0` value you just found.

    ```
    // --- 3. CLEAN AIR RESISTANCE (R0) ---
    // You MUST find this by calibrating. See guide after the code.
    // I am using a common default, but yours WILL be different.
    const float R0_CLEAN_AIR_RESISTANCE = 27050.0; // <-- YOUR NEW VALUE HERE
    ```

4.  Upload your main code.

Your robot's web server will now show much more accurate PPM readings, as it's calibrated specifically to your sensor.