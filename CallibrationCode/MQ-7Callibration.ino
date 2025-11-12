/*
 * ========================================
 * MQ-7 SENSOR CALIBRATION SKETCH
 * ========================================
 *
 * This code's ONLY purpose is to find your
 * sensor's resistance in clean air (R0).
 *
 * HOW TO USE:
 * 1. Make sure your sensor has been "burned-in"
 * (powered on) for at least 24 hours.
 * 2. CHECK THE CONSTANTS in the
 * "SENSOR CONFIGURATION" section below.
 * 3. Upload this code.
 * 4. Put the sensor in clean, fresh air (outside).
 * 5. Open the Serial Monitor (115200 baud).
 * 6. Let it run for 15-20 minutes until the
 * "Rs (Clean Air Resistance)" value is stable.
 * 7. This stable value is your R0.
 * 8. Copy this R0 value and paste it into your
 * main robot code's "R0_CLEAN_AIR_RESISTANCE"
 * constant.
 */

// --- Sensor Pin ---
const int MQ7_PIN = 34; // AOUT pin from sensor


// ================================================
// --- SENSOR CONFIGURATION ---
// !! CHECK THESE VALUES !!
// ================================================

// --- 1. YOUR VOLTAGE DIVIDER ---
// I am assuming your "two 220ohms resistors" are a voltage
// divider from a 5V AOUT pin to your 3.3V ESP32 pin.
// Wiring: [AOUT] -- [R1] -- [ESP32 Pin] -- [R2] -- [GND]
const float R1_DIVIDER_OHMS = 220.0;
const float R2_DIVIDER_OHMS = 220.0;
// *** IF YOUR MODULE IS 3.3V AND YOU ARE *NOT* USING A DIVIDER,
// *** SET R1_DIVIDER_OHMS = 0.0;
// *** This will disable the divider correction math.

// --- 2. MODULE LOAD RESISTOR (R_L) ---
// The resistor ON THE MODULE (often a 10k or 5k pot).
const float LOAD_RESISTOR_OHMS = 10000.0;

// --- 3. SENSOR CIRCUIT VOLTAGES ---
const float VCC_VOLTAGE = 5.0; // Module Vcc (usually 5V)
const float ESP32_ADC_MAX_VOLTS = 3.3; // ESP32 ADC Vref

// ================================================

void setup() {
  Serial.begin(115200);
  pinMode(MQ7_PIN, INPUT);

  Serial.println("=========================");
  Serial.println("MQ-7 Calibration Sketch");
  Serial.println("=========================");
  Serial.println("Place sensor in clean, fresh air.");
  Serial.println("Waiting 15-20 minutes for R0 to stabilize...");
  Serial.println();
}

void loop() {
  // Read the raw ADC value (0-4095)
  int mq7Reading_raw = analogRead(MQ7_PIN);

  // --- Calculate Sensor Resistance (Rs) ---

  // 1. Convert ADC (0-4095) to measured voltage (0-3.3V)
  float V_measured = (mq7Reading_raw / 4095.0) * ESP32_ADC_MAX_VOLTS;

  // 2. Correct for the voltage divider (if used)
  float V_RL; // This is the TRUE voltage at the sensor's AOUT
  if (R1_DIVIDER_OHMS > 0.0 && R2_DIVIDER_OHMS > 0.0) {
    V_RL = V_measured * (R1_DIVIDER_OHMS + R2_DIVIDER_OHMS) / R2_DIVIDER_OHMS;
  } else {
    V_RL = V_measured; // No divider
  }

  // Prevent division by zero if V_RL is 0
  if (V_RL == 0) V_RL = 0.0001; 

  // 3. Calculate Sensor Resistance (Rs)
  // Formula: Rs = ( (Vcc * R_L) / V_RL ) - R_L
  float Rs = ( (VCC_VOLTAGE * LOAD_RESISTOR_OHMS) / V_RL ) - LOAD_RESISTOR_OHMS;
  if (Rs < 0) Rs = 0; // Can't be negative

  // Print the values to the Serial Monitor
  Serial.print("Raw ADC: ");
  Serial.print(mq7Reading_raw);
  Serial.print("\t V_measured (at pin): ");
  Serial.print(V_measured, 3);
  Serial.print("V \t V_RL (at sensor): ");
  Serial.print(V_RL, 3);
  Serial.print("V \t Rs (Clean Air Resistance): ");
  Serial.println(Rs, 2); // This is the value you want

  // Wait 1 seconds for the next reading
  delay(1000);
}