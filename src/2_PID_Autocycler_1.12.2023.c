// Autocycler temperature regulation implemented with two different loops using ESP32-DevKitC-v4

#include "CytronMotorDriver.h"
#include <PID_v1.h>

#define TC_PIN_AIR 33       // defined ADC pin 33
#define TC_PIN_PROBE 32     // defined ADC pin 32
#define AREF 3.286          // set to AREF, Connect the AD8495 to 3.3 V supply
#define ADC_RESOLUTION 12   // 12-bit resolution of ESP32

//Define Variables we'll be connecting to
double Setpoint_AIR, Input_AIR, Output_AIR,Setpoint_PROBE, Input_PROBE, Output_PROBE;
double reading = 0, voltage, temperature_AIR,reading_PROBE = 0, voltage_PROBE, temperature_PROBE;

//Specify the links and initial tuning parameters_AIR
double Kp_AIR=1, Ki_AIR=2, Kd_AIR=0.4;
double aggKp_AIR=4, aggKi_AIR=0.2, aggKd_AIR=1;
PID myPID_AIR(&temperature_AIR, &Output_AIR, &Setpoint_AIR, Kp_AIR, Ki_AIR, Kd_AIR, DIRECT);

//Specify the links and initial tuning parameters
double Kp_PROBE=3, Ki_PROBE=0.5, Kd_PROBE=1;
double aggKp_PROBE=4, aggKi_PROBE=0.2, aggKd_PROBE=1;
PID myPID_PROBE(&temperature_PROBE, &Output_PROBE, &Setpoint_PROBE, Kp_PROBE, Ki_PROBE, Kd_PROBE, DIRECT);


double get_temperature(int PIN) {
  reading = analogRead(PIN);
 // Serial.print("read = "); 
 // Serial.print(reading);
  voltage = reading * (AREF / 4095);
 // Serial.print("volt = ");
 // Serial.print(voltage);
  return ((voltage - 1.25) / 0.005)+34;
}

// Configure the motor driver.
CytronMD Driver_AIR(PWM_DIR,17,16);  // PWM = Pin 17, DIR = Pin 16.
CytronMD Driver_PROBE(PWM_DIR,4,2);  // PWM = Pin 4, DIR = Pin 2. || Kühlng OBEN

// The setup routine runs once when you press reset.
void setup() {
   Serial.begin(9600);

   Setpoint_AIR = 15 ; //hier Lufttemperatur einstellen für automatische Regelung
   myPID_AIR.SetOutputLimits(-255, -1);
   //turn the PID on
   myPID_AIR.SetMode(AUTOMATIC);
   Setpoint_PROBE = 15; //hier Probentemperatur einstellen für automatische Regelung
   myPID_PROBE.SetOutputLimits(-100, -80);
   //turn the PID on
   myPID_PROBE.SetMode(AUTOMATIC);


  Serial.print("Upload version 13.12.2023");


}


// The loop routine runs over and over again forever.
void loop() {
  temperature_AIR = get_temperature(TC_PIN_AIR);
  temperature_PROBE = get_temperature(TC_PIN_PROBE);
//  double gap = abs(Setpoint-temperature); //distance away from setpoint
//  if (gap < 1)
//  {  //we're close to setpoint, use conservative tuning parameters
//    myPID.SetTunings(Kp, Ki, Kd);
//  }
//  else
//  {
//     //we're far from setpoint, use aggressive tuning parameters
//     myPID.SetTunings(aggKp, aggKi, aggKd);
//  }

  myPID_AIR.Compute(); 
  Driver_AIR.setSpeed(Output_AIR);  // Output_AIR -> automatische Regelung, ansonsten Wert zwischen -255 (kühlen) und 255 Heizen
  myPID_PROBE.Compute(); 
  Driver_PROBE.setSpeed(-100); // Output_PROBE -> automatische Regelung, ansonsten Wert zwischen -255 (kühlen) und 255 Heizen


    
  Serial.print("Air-T = ");
  Serial.print(temperature_AIR);
  Serial.print(" C");
  Serial.print("  ;  ");
  Serial.print("Duty_AIR = ");
  Serial.print(Output_AIR);
  Serial.print(" %");
  Serial.print("  ;  ");
  
  Serial.print("Probe-T = ");
  Serial.print(temperature_PROBE);
  Serial.print(" C");
  Serial.print("  ;  ");
  Serial.print("Duty_PROBE = ");
  Serial.print(Output_PROBE);
  Serial.println(" %");
  
  delay(1000);
  
}