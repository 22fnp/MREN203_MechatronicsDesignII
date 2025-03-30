/*
 * 
 * file serial-communication.ino
 * author Sebastien Sauter
 * description Complete Arduino program for serial communication on Lynxmotion Rover
 * version 1.0
 * date 2025-03-26
 *
 */

#include <ArduinoJson.h>

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

// Sampling period [ms]
const int T = 50;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Desired wheel speed
float v_desired = 0; // Scalar quantity, 0.7 m/s is an appropriate desired speed
int v_direction = 0; // Direction case variable

// Desired turning rate
float omega_desired = 0;

// Individual desired wheel speeds
double v_R_desired = 0.0;
double v_L_desired = 0.0;

// Proportional error
double e_nowR;
double e_nowL;

// Integral error
double e_intLT = 0;
double e_intRT = 0;
double e_intL[] = {0, 0, 0, 0, 0};
double e_intR[] = {0, 0, 0, 0, 0};

// Counter variable for PID
uint8_t i = 0;

// Controller constants
double k_p = 2000; 
double k_i = 180; 

// Set the wheel motor PWM command [0-255] org. 128
short u_L;
short u_R;

// Variable to disable integral contributions
bool antiWindUp = 1;

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

// Compute left wheel speed from vehicle translational velocity and angular turning rate
double compute_left_wheel_speed(double V, double Omega) {
  double V_L;
  V_L = V - (ELL / 2) * Omega;
  return V_L;
}

double compute_right_wheel_speed(double V, double Omega) {
  double V_R;
  V_R = V + (ELL / 2) * Omega;
  return V_R;
}

// PI controller function with antiWindUp
short PI_controller(double e_now, double e_intT, double k_P, double k_I) {
  short u;
  u = (short)((k_P * e_now) + (k_I * e_intT * antiWindUp));
  if (u > 255)
  {
    u = 255;
    antiWindUp = 0;
  }
  else {
    antiWindUp = 1;
  }
  return u;
}

// Determine the rovers direction: forward, reverse, left, right
int determine_v_direction(double V, double Omega) {
  int direction;
  if (Omega = 0) {
    if (V >= 0) {
      direction = 1;
    } else {
      direction = 2;
    }
  } else {
    if (Omega > 0) {
      direction = 4;
    } else {
      direction = 3;
    }
  }
  return direction;
}

/*
 * ----------------------
 * SETUP CODE
 * ----------------------
 */

void setup() {
      
  // Open the serial port at 115200 bps
  Serial.begin(115200);
  while (!Serial)
    continue;

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
    /*
    Serial.print("TX_V");
    Serial.print(v);
    Serial.print("\t");
    Serial.print("TX_W:");
    Serial.print(omega);
    Serial.print("\n");
    */

    // Read in json msg from pi
    String json = Serial.readStringUntil('\n');
    json.trim();

    // Create json object, 256 bits gives more space than needed
    StaticJsonDocument<256> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, json);

    // Test if parsing succeeds
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    
    // Transfer data from doc object to variables
    v_desired = doc["translational_speed"].as<float>();
    omega_desired = doc["angular_rate"].as<float>();

    // Troubleshooting measure
    Serial.print("v_desired: ");
    Serial.print(v_desired);
    Serial.print("\t");
    Serial.print("omega_desired: ");
    Serial.println(omega_desired);

    // Determine the individual wheel speeds and direction
    v_R_desired = compute_left_wheel_speed(v_desired, omega_desired);
    v_L_desired = compute_right_wheel_speed(v_desired, omega_desired);
    v_direction = determine_v_direction(v_desired, omega_desired);
    
    // Find the proportional error in wheel speed
    e_nowR = v_R_desired - v_R;
    e_nowL = v_L_desired - v_L;

    // Integral error NEED TO UPDATE TO SUM OF ONLY RELEVANT PREV ERRORS USE QUEUE.
    e_intRT -= e_intR[i];   // Subtract oldest value from integral error sum
    e_intR[i] = e_nowR;     // Update the oldest calue of integral error array
    e_intRT += e_intR[i];   // Add updated value to array

    e_intLT -= e_intL[i];   // Same as prev
    e_intL[i] = e_nowL;
    e_intLT += e_intL[i];

    i = (i + 1) % 5;        // Increment i and reset i when equal to 5
    
    // Call PI controller
    u_L = PI_controller(e_nowL, e_intLT, k_p, k_i);
    u_R = PI_controller(e_nowR, e_intRT, k_p, k_i);

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticksR = 0;
    encoder_ticksL = 0;
  }

  // Switch case to set the direction of the motors
  switch (v_direction) {
    case 1:
      // Forward
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);
      digitalWrite(I3, LOW);
      digitalWrite(I4, HIGH);
      break;
    case 2:
      // Reverse
      digitalWrite(I1, LOW);
      digitalWrite(I2, HIGH);
      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
      break;
    case 3:
      // Right
      digitalWrite(I1, LOW);
      digitalWrite(I2, HIGH);
      digitalWrite(I3, LOW);
      digitalWrite(I4, HIGH);
      break;
    case 4:
      // Left
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);
      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
      break;
  }

  // PWM command to the motor driver
  analogWrite(EA, u_R);
  analogWrite(EB, u_L);
}
