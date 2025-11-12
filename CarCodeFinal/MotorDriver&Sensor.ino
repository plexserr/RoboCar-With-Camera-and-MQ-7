/*
 * ========================================
 * ESP32 BLUETOOTH RC CAR CONTROLLER
 * V2.x.x CORE + WIFI SENSOR WEB SERVER
 * ========================================
 * This code creates a Bluetooth device
 * named "MyRobot" for RC control.
 *
 * It ALSO connects to your WiFi and
 * hosts a web server that displays
 * live MQ-7 sensor readings in PPM.
 * NOTE: Your ESP and Host Device must be on the same network.
 * --- Your Pinout ---
 * Motor A: ENA=14, IN1=27, IN2=26
 * Motor B: ENB=12, IN3=25, IN4=33
 *
 * --- NEW SENSOR PIN ---
 * MQ-7 AOUT: GPIO 34
 *
 * --- App Commands (Bluetooth) ---
 * 'F', 'B', 'L', 'R', 'S', '1'-'9', 'q'
 *
 * --- WiFi Access ---
 * Connect to ESP32 to your Host Machine and Open Serial Monitor after uploading your code.
 * Press the RST/EN button on the ESP32 to see the IP address assigned by your router.
 * Open a web browser and enter that IP address to see the sensor data.
 * ========================================
 */

#include "BluetoothSerial.h"
#include <WiFi.h>           // --- NEW --- For WiFi functionality
#include <WebServer.h>      // --- NEW --- To host the web server
#include <math.h>           // --- NEW --- For log10() and pow()

// Check if Bluetooth is properly included
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run 'make menuconfig' to enable it
#endif

// Create a Bluetooth Serial object
BluetoothSerial SerialBT;

// --- NEW --- WiFi Credentials
const char* ssid = "YOUR_WIFI_SSID";     // <--- PUT YOUR WIFI NAME HERE
const char* password = "YOUR_WIFI_PASSWORD"; // <--- PUT YOUR WIFI PASSWORD HERE

// --- NEW --- Web Server object on port 80
WebServer server(80);

// --- NEW --- MQ-7 Sensor Pin
// Must be an ADC1 pin (e.g., 32-39) because ADC2 is used by WiFi.
const int MQ7_PIN = 34;

// --- NEW --- Global variables for sensor reading
volatile int mq7Reading_raw = 0; // The raw 0-4095 value
volatile float mq7PPM = 0.0;     // The calculated PPM value

// --- NEW --- Non-blocking timer for sensor
unsigned long lastSensorReadTime = 0;
const long sensorReadInterval = 1000; // Read sensor every 1 second (1000ms)


// ================================================
// --- NEW --- SENSOR CALIBRATION CONSTANTS ---
// YOU MUST UPDATE THESE VALUES FOR YOUR SENSOR
// ================================================

// --- 1. VOLTAGE DIVIDER RESISTORS ---
// I am assuming your "two 220ohms resistors" are a voltage
// divider from a 5V AOUT pin to your 3.3V ESP32 pin.
// Wiring: [AOUT] -- [R1] -- [ESP32 Pin] -- [R2] -- [GND]
const float R1_DIVIDER_OHMS = 220.0;
const float R2_DIVIDER_OHMS = 220.0;
// *** IF YOUR MODULE IS 3.3V or HAS A 3.3V AOUT,
// *** AND YOU ARE *NOT* USING A DIVIDER,
// *** SET R1_DIVIDER_OHMS = 0.0;
// *** (This will disable the divider correction math)

// --- 2. MODULE LOAD RESISTOR (R_L) ---
// This is the resistor ON THE MODULE, not your 220s.
// Most modules use 10k (10000) or 5k (5000). Check your module.
const float LOAD_RESISTOR_OHMS = 10000.0;

// --- 3. CLEAN AIR RESISTANCE (R0) ---
// You MUST find this by calibrating. See guide after the code.
// I am using a common default, but yours WILL be different.
const float R0_CLEAN_AIR_RESISTANCE = 27000.0;

// --- 4. SENSOR CIRCUIT VOLTAGES ---
const float VCC_VOLTAGE = 5.0; // Module Vcc (usually 5V)
const float ESP32_ADC_MAX_VOLTS = 3.3; // ESP32 ADC Vref


// --- Motor A Pins ---
const int MOTOR_A_PWM_PIN = 14; // ENA
const int MOTOR_A_IN1_PIN = 27; // IN1
const int MOTOR_A_IN2_PIN = 26; // IN2

// --- Motor B Pins ---
const int MOTOR_B_PWM_PIN = 12; // ENB
const int MOTOR_B_IN3_PIN = 25; // IN3
const int MOTOR_B_IN4_PIN = 33; // IN4

// --- ESP32 PWM Configuration (v2 Style) ---
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_A = 0;    // Channel 0 for Motor A
const int PWM_CHANNEL_B = 1;    // Channel 1 for Motor B

// Global variable to hold the current speed (0-255)
int currentSpeed = 200;

// ==================================
//  MOTOR CONTROL FUNCTIONS (Unchanged)
// ==================================
void moveForward() {
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void moveBackward() {
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, HIGH);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void moveLeft() {
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void moveRight() {
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, HIGH);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void moveStop() {
  // Use active braking for a sharp stop
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, HIGH);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}


// ==================================
//  --- NEW --- PPM CALCULATION
// ==================================

/**
 * @brief Calculates the PPM of CO from the raw ADC reading.
 * This function uses the formulas from the MQ-7 datasheet
 * and corrects for a voltage divider.
 * @param raw_adc The raw 0-4095 value from analogRead()
 * @return A float value for PPM.
 */
float calculatePPM(int raw_adc) {
    // 1. Convert ADC (0-4095) to measured voltage (0-3.3V)
    float V_measured = (raw_adc / 4095.0) * ESP32_ADC_MAX_VOLTS;

    // 2. Correct for the voltage divider (if used)
    // V_actual = V_measured * (R1 + R2) / R2
    // If R1 = 0, V_actual = V_measured.
    float V_RL; // This is the TRUE voltage at AOUT
    if (R1_DIVIDER_OHMS > 0.0 && R2_DIVIDER_OHMS > 0.0) {
      V_RL = V_measured * (R1_DIVIDER_OHMS + R2_DIVIDER_OHMS) / R2_DIVIDER_OHMS;
    } else {
      V_RL = V_measured; // No divider
    }

    // Prevent division by zero if V_RL is 0
    if (V_RL == 0) return 0; // No gas or error

    // 3. Calculate Sensor Resistance (Rs)
    // Formula: Rs = ( (Vcc * R_L) / V_RL ) - R_L
    float Rs = ( (VCC_VOLTAGE * LOAD_RESISTOR_OHMS) / V_RL ) - LOAD_RESISTOR_OHMS;
    if (Rs < 0) Rs = 0; // Can't be negative

    // 4. Calculate Rs/R0 ratio
    float ratio = Rs / R0_CLEAN_AIR_RESISTANCE;
    if (ratio <= 0) return 0; // Avoid log(0)

    // 5. Calculate PPM using the log-log curve from the datasheet
    // Formula: log10(ppm) = m * log10(ratio) + b
    // For MQ-7 CO:
    // m = -1.5 (The slope of the graph)
    // b = 2.0 (The intercept, approx. log10(100))
    // This is an approximation.
    float log10_ppm = -1.5 * log10(ratio) + 2.0;
    float ppm = pow(10, log10_ppm);

    return ppm;
}


// ==================================
//  --- NEW --- WEB SERVER HANDLERS
// ==================================

/**
 * @brief Handles the root URL ('/')
 * Serves the main HTML page with auto-refreshing JavaScript.
 */
void handleRoot() {
  // This string (R"rawliteral(...)rawliteral") is the HTML for the webpage
  String html = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Sensor Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #f2f2f2; margin-top: 40px;}
    h1 { color: #333; }
    .card { background: #fff; padding: 30px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); display: inline-block; }
    .sensor-value { font-size: 4.5em; color: #007BFF; margin: 20px 0; font-weight: bold;}
    .unit { font-size: 1.5em; color: #888; }
  </style>
</head>
<body>
  <h1>ESP32 Robot Sensor</h1>
  <div class="card">
    <h2>MQ-7 CO (PPM)</h2>
    <div class="sensor-value" id="co_value">...</div>
    <div class="unit">Parts Per Million</div>
  </div>
  <script>
    // This JavaScript runs in the browser
    // It fetches new data from the ESP32 every 2 seconds
    setInterval(function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          // Update the value on the page
          document.getElementById("co_value").innerHTML = this.responseText;
        }
      };
      xhttp.open("GET", "/data", true); // Ask the ESP32 for the /data URL
      xhttp.send();
    }, 2000); // 2000 milliseconds = 2 seconds
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html); // Send the HTML page to the browser
}

/**
 * @brief Handles the '/data' URL
 * This is what the JavaScript calls to get the new sensor value.
 * It sends the formatted PPM value as plain text.
 */
void handleData() {
  // Format the PPM float to 2 decimal places
  char buffer[10];
  dtostrf(mq7PPM, 6, 2, buffer); // 6 total width, 2 after decimal
  server.send(200, "text/plain", String(buffer));
}

/**
 * @brief Handles 404 Not Found
 */
void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ==================================
//  SETUP (Now with WiFi & WebServer)
// ==================================
void setup() {
  Serial.begin(115200);

  // Set all motor pins as OUTPUTs
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN3_PIN, OUTPUT);
  pinMode(MOTOR_B_IN4_PIN, OUTPUT);

  // --- NEW --- Set sensor pin as INPUT
  pinMode(MQ7_PIN, INPUT);

  // --- v2 PWM Setup (The key part) ---
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM_PIN, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_PWM_PIN, PWM_CHANNEL_B);
  
  // Stop motors at startup
  moveStop();

  // Start the Bluetooth server with a name
  SerialBT.begin("MyRobot"); 
  Serial.println("Bluetooth device is ready. You can pair with 'MyRobot'.");

  // --- NEW --- WiFi Setup
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // <-- This is the address for your browser

  // --- NEW --- Web Server Route Setup
  server.on("/", handleRoot);     // Call handleRoot() when / is requested
  server.on("/data", handleData); // Call handleData() when /data is requested
  server.onNotFound(handleNotFound); // Handle 404
  
  // --- NEW --- Start the server
  server.begin();
  Serial.println("HTTP server started. Open the IP in your browser.");
}

// ==================================
//  LOOP (Now handles BT, Web, & Sensor)
// ==================================
void loop() {
  
  // --- Part 1: Handle Bluetooth Commands ---
  // (This is your original code, unchanged)
  if (SerialBT.available()) {
    
    // Read the incoming command as a character
    char cmd = SerialBT.read();
    
    // Print the command to the Serial Monitor for debugging
    Serial.print("Received BT command: ");
    Serial.println(cmd);

    // Act on the command
    switch (cmd) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': moveLeft(); break;
      case 'R': moveRight(); break;
      case 'S': moveStop(); break;
      
      // Speed Commands
      case '1': currentSpeed = 25; break;  // ~10% of 255
      case '2': currentSpeed = 51; break;
      case '3': currentSpeed = 76; break;
      case '4': currentSpeed = 102; break;
      case '5': currentSpeed = 127; break; // 50%
      case '6': currentSpeed = 153; break;
      case '7': currentSpeed = 178; break;
      case '8': currentSpeed = 204; break;
      case '9': currentSpeed = 230; break;
      case 'q': currentSpeed = 255; break; // 100%
    }
  }

  // --- NEW --- Part 2: Handle Web Server Clients ---
  // This must be called every loop to listen for browser requests
  server.handleClient();

  // --- NEW --- Part 3: Read Sensor (Non-Blocking) ---
  // Use millis() to read the sensor without blocking the loop
  if (millis() - lastSensorReadTime > sensorReadInterval) {
    lastSensorReadTime = millis(); // Update the last read time
    
    // Read the analog value from the sensor
    mq7Reading_raw = analogRead(MQ7_PIN);
    
    // --- NEW --- Calculate PPM
    mq7PPM = calculatePPM(mq7Reading_raw);
    
    // Optional: Print to serial monitor for debugging
    Serial.print("MQ-7 Raw: ");
    Serial.print(mq7Reading_raw);
    Serial.print("  Calculated PPM: ");
    Serial.println(mq7PPM);
  }
}