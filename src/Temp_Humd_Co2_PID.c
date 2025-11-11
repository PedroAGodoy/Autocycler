#include <Arduino.h>
#include <SensirionI2cScd30.h>
#include <Wire.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

SensirionI2cScd30 sensor; //Connect SCL to GPIO 22 and SDA to GPIO 21
static char errorMessage[128];
static int16_t error;

// Timing variables
const unsigned long SENSOR_READ_INTERVAL = 3000;  // Increased to 3s for slow system
unsigned long previousMillis = 0;

// Motor control pins
const int PWM_PIN = 5;
const int DIRECTION_PIN = 18;

// PID Control - Humidity
double humidityInput;          // Current humidity reading
double humiditySetpoint = 40;  // Target humidity
double pidOutput;              // PID calculated output
float pwmValue = 0;            // PWM value sent to motor

// PID Tuning parameters
double Kp = 10, Ki = 0.01, Kd = 1;

PID humidityPID(&humidityInput, &pidOutput, &humiditySetpoint, Kp, Ki, Kd, DIRECT);

// AUTOTUNE CONFIGURATION (Optimized for humidity)
PID_ATune aTune(&humidityInput, &pidOutput);
bool tuning = false;
const double ATUNE_STEP = 25;         // Reduced step for slow humidity system
const double ATUNE_NOISE = 1.0;       // Tighter noise band (was 1.5)
const unsigned int ATUNE_LOOKBACK = 120; // 120s lookback for slow systems
unsigned long lastTuneTime = 0;       // Tracks autotune timing
// --- END AUTOTUNE ---

void plot(String label, float value, bool isLast = false) {
  Serial.print(label);
  if (label != "") Serial.print(":");
  Serial.print(value);
  isLast ? Serial.println() : Serial.print(",");
}

void initializeSensor() {
  Wire.begin();
  sensor.begin(Wire, SCD30_I2C_ADDR_61);
  sensor.stopPeriodicMeasurement();
  sensor.softReset();
  delay(2000);
  
  uint8_t major, minor;
  error = sensor.readFirmwareVersion(major, minor);
  if (error) {
    Serial.print("Firmware version error: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  
  error = sensor.startPeriodicMeasurement(0);
  if (error) {
    Serial.print("Measurement start error: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }
}

void startAutoTune() {
  if (!tuning) {
    // NEW: Only start when near setpoint (±5%)
    if (abs(humidityInput - humiditySetpoint) < 5.0) {
      tuning = true;
      humidityPID.SetMode(MANUAL);  // Disable PID
      
      aTune.SetNoiseBand(ATUNE_NOISE);
      aTune.SetOutputStep(ATUNE_STEP);
      aTune.SetLookbackSec(ATUNE_LOOKBACK);
      aTune.SetControlType(1);  // PID control (not PI) :cite[7]
      
      Serial.println("Autotune: STARTED (Humidity Mode)");
    } else {
      Serial.println("Error: Start autotune near setpoint (±5%)");
    }
  }
}

void stopAutoTune() {
  tuning = false;
  humidityPID.SetMode(AUTOMATIC);  // Always return to automatic mode
  
  if (aTune.GetKp() > 0) {  // Only apply valid parameters
    Kp = aTune.GetKp();
    Ki = aTune.GetKi();
    Kd = aTune.GetKd();
    humidityPID.SetTunings(Kp, Ki, Kd);
    Serial.print("Autotune SUCCESS! New Parameters - Kp:");
    Serial.print(Kp);
    Serial.print(" Ki:");
    Serial.print(Ki);
    Serial.print(" Kd:");
    Serial.println(Kd);
  } else {
    Serial.println("Autotune: FAILED (keeping old tunings)");
  }
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(DIRECTION_PIN, LOW);

  Serial.begin(115200);
  //while (!Serial) delay(100);

  initializeSensor();
  humidityPID.SetOutputLimits(-255, 255);
  humidityPID.SetMode(AUTOMATIC);
  humidityPID.SetSampleTime(3000);  // 3s sample time (valid >14ms) :cite[1]:cite[9]
  Serial.println("Humidity Controller v2.0");
}

void updateSensorData() {
  float co2 = 0.0, temperature = 0.0, humidity = 0.0;
  error = sensor.blockingReadMeasurementData(co2, temperature, humidity);
  if (error) {
    Serial.print("Sensor read error: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  Serial.print("Temperature_°C ");
  Serial.print(temperature);
  Serial.print("; Humidity_% "); 
  Serial.print(humidity);
  Serial.print("; CO2_ppm "); 
  Serial.println(co2);
  //Serial.print(";\n");

  /* //FOR DEBUGGING
  plot("Temperature_°C", temperature, false);
  plot("Humidity_%", humidity, false);
  plot("CO2_ppm", co2, false);
  plot("pwmValue", pwmValue, false);
  plot("PID_Output", pidOutput, true);
  */
  
  humidityInput = humidity;  // Update PID input
}

void handleMotorOutput() {
  // Measured Max/Min Values from testing
  int Max_i = 123; // Max current
  int Min_i = 93;  // Min current

  if (pidOutput < 0) {
    // Cooling mode - see Motor driver Manual
    float pidOutputInv = -pidOutput; // convert to positive
    pwmValue = ((pidOutputInv/255)*(Max_i - Min_i)) + Min_i;
    digitalWrite(DIRECTION_PIN, LOW); // Cooling direction
  } else {
    // Heating mode with reduced gain
    pwmValue = ((pidOutput * 0.42 / 255) * (Max_i - Min_i)) + Min_i;
    digitalWrite(DIRECTION_PIN, HIGH); // Heating direction
  }

  analogWrite(PWM_PIN, pwmValue);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Sensor reading at 2,5s intervals
  if (currentMillis - previousMillis >= 2500) {
    updateSensorData();
    previousMillis = currentMillis;
  }

  // AUTOTUNE HANDLING (NEW TIMING APPROACH)
  if (tuning) {
    // Run autotune at 3s intervals (replaces SetSampleTime)
    if (currentMillis - lastTuneTime >= 3000) {
      lastTuneTime = currentMillis;
      if (aTune.Runtime()) {  // Non-zero when complete
        tuning = false;
        
        // Apply new parameters if valid
        if (aTune.GetKp() > 0) {
          Kp = aTune.GetKp();
          Ki = aTune.GetKi();
          Kd = aTune.GetKd();
          humidityPID.SetTunings(Kp, Ki, Kd);
          Serial.print("NEW TUNINGS | Kp:");
          Serial.print(Kp);
          Serial.print(" Ki:");
          Serial.print(Ki);
          Serial.print(" Kd:");
          Serial.println(Kd);
        } else {
          Serial.println("Autotune FAILED - Invalid values");
        }
        humidityPID.SetMode(AUTOMATIC);
      }
    }
  } 
  else {
    humidityPID.Compute(); // Normal PID operation
  }

  // Serial command handling
  if (Serial.available() && Serial.read() == 'autotune' && !tuning) {
    startAutoTune();
  }

  handleMotorOutput();
}