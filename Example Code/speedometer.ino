/**
   @file motor-angular-rate.ino
   @author Joshua Marshall (joshua.marshall@queensu.ca)
   @brief Arduino program to estimate motor speed from encoder.
   @version 2.1
   @date 2022-12-09

   @copyright Copyright (c) 2021-2022

*/

// Wheel PWM pin (must be a PWM pin)
int EA = 6;//6
int EB = 5;//5

// Wheel direction digital pins
int I1 = 7;//7
int I2 = 4;//4
int I3 = 11;//11
int I4 = 10;//10

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_AL = 8; //8
const byte SIGNAL_BL = 9; //9
const byte SIGNAL_AR = 2; //2
const byte SIGNAL_BR = 3; //3


// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle's track length
const double ELL = 0.2775;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksR = 0;
volatile long encoder_ticksL = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;
double v_L = 0.0;
double v_R = 0.0;
double v = 0.0;
double omega = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 50;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Desired wheel speed
float Vd = 0; // Vd is a scalar quantity, 0.7 m/s is an appropriate desired speed
bool dirn = 0; // Direction of Vd, 0 (positive), 1 (negative).

// Desired turning rate
float omega_D = 0;

// Proportional error
double e_nowR;
double e_nowL;

// Integral error
double e_intLT = 0;
double e_intRT = 0;
double e_intL[] = {0, 0, 0, 0, 0};
double e_intR[] = {0, 0, 0, 0, 0};

// Derrivative error
double e_dRT;
double e_dLT;
double e_dL[] = {0, 0, 0, 0, 0};
double e_dR[] = {0, 0, 0, 0, 0};

// Counter variable for PID
uint8_t i = 0;

// Counter variable for step response and pulsating response of PID.
uint8_t count = 1;

// Left and Right wheel speeds based on desired linear velocity
double Vd_L = Vd - (ELL / 2) * omega_D;
double Vd_R = Vd + (ELL / 2) * omega_D;

// Controller Const
double k_p = 2000; //Max 620 - Min 600 (best 600, 2250 is too high 2100 is fine 200 is best for only K.
double k_int = 180; //With only PI 400 is too high, with PID 450 is too high, with only PI 350 max - 3?? min. 350 is best! 375 is even too high
double k_d = 40;

// Set the wheel motor PWM command [0-255] org. 128
short u_L;
short u_R;

// This function is called when SIGNAL_B goes HIGH
void decodeEncoderTicksR()
{
  if (digitalRead(SIGNAL_AR) == LOW)
  {
    // SIGNAL_BR leads SIGNAL_AR, so wheels spin backwards
    encoder_ticksR--;
  }
  else
  {
    // SIGNAL_AR leads SIGNAL_BR, so wheels spin forwards
    encoder_ticksR++;
  }
}

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksL()
{
  if (digitalRead(SIGNAL_BL) == LOW)
  {
    // SIGNAL_AL leads SIGNAL_BL, so wheels spin backwards
    encoder_ticksL--;
  }
  else
  {
    // SIGNAL_BL leads SIGNAL_AL, so wheels spin forwards
    encoder_ticksL++;
  }
}

// Variable to disable Integral controller
bool antiWindUp = 1;

// Function for Proportional Integral Controller
short PI_controller(double e_now, double e_intT, double e_d, double k_P, double k_int, double k_d)
{
  short u;
  u = (short)(k_P * e_now + k_int * e_intT * antiWindUp);
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

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R)
{
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R)
{
  double omega;
  omega = 1.0 / ELL * (v_R - v_L);
  return omega;
}


void setup()
{
  // Open the serial port at 9600 bps
  Serial.begin(9600);

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
  attachInterrupt(digitalPinToInterrupt(SIGNAL_BR), decodeEncoderTicksR, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicksL, RISING);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

void loop()
{
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T)
  {
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    v_L = omega_L * RHO;
    v_R = omega_R * RHO;
    v = compute_vehicle_speed(v_L, v_R);
    omega = compute_vehicle_rate(v_L, v_R);

    // Find the proportional error in wheel speed.
    e_nowR = Vd_R - v_R;
    e_nowL = Vd_L - v_L;

    // Integral error NEED TO UPDATE TO SUM OF ONLY RELEVANT PREV ERRORS USE QUEUE.
    e_intRT -= e_intR[i];   // Subtract oldest value from integral error sum
    e_intR[i] = e_nowR;     // Update the oldest calue of integral error array
    e_intRT += e_intR[i];    // Add updated value to array

    e_intLT -= e_intL[i];   // Same as prev
    e_intL[i] = e_nowL;
    e_intLT += e_intL[i];

    // Derivative error
    e_dRT += e_dR[i];   // Add oldest value from integral error sum
    e_dR[i] = e_nowR;   // Update the oldest calue of integral error array
    e_dRT -= e_dR[i];   // Add updated value to array

    e_dLT += e_dL[i];   // Same as prev
    e_dL[i] = e_nowL;
    e_dLT -= e_dL[i];

    // Reset counter i
    i = (i + 1) % 5;        // Increment i and reset i when equal to 5
    count++;

    // Used to get pulsating response of changing Vd
    if (count % 101 == 0) {
      Vd_L = (Vd_L == 0) ? 0.7 : 0;
      Vd_R = (Vd_R == 0) ? 0.7 : 0;
      count = 1;
    }

//    // Used to get step response (ensure to initially set Vd = 0)
//    if (count % 50 == 0) {
//      Vd_L = 0.7;
//      Vd_R = 0.7;
//      count = 1;
//    }

    // Print some stuff to the serial monitor
    /*
      Serial.print("Encoder ticks L: ");
      Serial.print(encoder_ticksL);
      Serial.print("\t");
    */
    Serial.print(Vd_R); //Print the desired wheel speed
    Serial.print("\t");

    /*
      Serial.print("Encoder ticks R: ");
      Serial.print(encoder_ticksR);
      Serial.print("\t");
    */
    Serial.print(v);
    Serial.print("\n");
    //    Serial.print("Estimated overall speed: ");
    //    Serial.print(v);
    //    Serial.print(" m/s");
    //    Serial.print("\n");
    //    Serial.print("Estimated overall angular rate: ");
    //    Serial.print(omega);
    //    Serial.print(" rad/s");
    //    Serial.print("\n");


    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticksR = 0;
    encoder_ticksL = 0;

    // Compute the new PWM
    u_L = PI_controller(e_nowL, e_intLT, e_dLT, k_p, k_int, k_d);
    u_R = PI_controller(e_nowR, e_intRT, e_dRT, k_p, k_int, k_d);
  }

  // Select a direction
  if (dirn == 0) {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  } else {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }


  // PWM command to the motor driver
  analogWrite(EA, u_R);
  analogWrite(EB, u_L);
}
