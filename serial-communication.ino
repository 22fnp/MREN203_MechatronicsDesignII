/*
 * 
 * file serial-communication.ino
 * author Sebastien Sauter
 * description Complete Arduino program for serial communication on Lynxmotion Rover
 * version 1.0
 * date 2025-03-26
 *
 */

#include <Arduino.h>

/*
 * ----------------------
 * VARIABLE DECLARATIONS
 * ----------------------
 */

 // Motor driver speed PWM pin (must be a PWM pin)
int EA = 6;
int EB = 5;

// Motor driver direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 11;
int I4 = 10;

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_AL = 8;
const byte SIGNAL_BL = 9;
const byte SIGNAL_AR = 2;
const byte SIGNAL_BR = 3;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle's track length [m]
const double ELL = 0.2775;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksR = 0;
volatile long encoder_ticksL = 0;

// Estimated wheel angular rate [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Estimated wheel translational speed [m/s] v = omega * radius
double v_L = 0.0;
double v_R = 0.0;

// Vehicle's estimated translational speed and angular turning rate [m/s] & [rad/s]
double v = 0.0;
double omega = 0.0;

// Sampling interval for measurements [ms]
const int T = 500;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

/*
 * ----------------------
 * FUNCTION DECLARATIONS
 * ----------------------
 */
 
void decodeEncoderTicksR() { // Function is called when SIGNAL_A goes HIGH
    if (digitalRead(SIGNAL_BR) == LOW) {
        encoder_ticksR--; // SIGNAL_A leads SIGNAL_B, so count one way
    }
    else {
        encoder_ticksR++; // SIGNAL_B leads SIGNAL_A, so count the other way
    }
}

void decodeEncoderTicksL() {
    if (digitalRead(SIGNAL_BL) == LOW) {
        encoder_ticksL--;
    }
    else {
        encoder_ticksL++;
    }
}

// Compute vehicle translational speed [m/s]
double compute_vehicle_speed(double v_L, double v_R) {
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R) {
  double omega;
  omega = 1.0 / ELL * (v_R - v_L);
  return omega;
}

/*
 * ----------------------
 * SETUP CODE
 * ----------------------
 */

void setup() {
      
  // Open the serial port at 115200 bps
  Serial.begin(115200);

  // Set the pin modes for the motor driver
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  
  // Set the pin modes for the encoders
  pinMode(SIGNAL_AR, INPUT);
  pinMode(SIGNAL_BR, INPUT);
  pinMode(SIGNAL_AL, INPUT);
  pinMode(SIGNAL_BL, INPUT);
  
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksR, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicksL, RISING);
  
  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");

}

/*
 * ----------------------
 * MAIN LOOP
 * ----------------------
 */

void loop() {
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    
    // Estimate wheels translational speed
    v_L = omega_L * RHO;
    v_R = omega_R * RHO * (-1);

    // Compute vehicle speed and rate
    v = compute_vehicle_speed(v_L, v_R);
    omega = compute_vehicle_rate(v_L, v_R);
      
    // Serial print commands, TX for transmission
    Serial.print("TX_V:" + String(v) + ";"); // Cast v to string for Pi
    Serial.print("TX_W:" + String(omega) + ";");
    
    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticksR = 0;
    encoder_ticksL = 0;
  }

}
