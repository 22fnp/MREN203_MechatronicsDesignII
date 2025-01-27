/**
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

// Wheel PWM pin (must be a PWM pin)
int EA = 6;
int EB = 5;

// Wheel direction digital pins
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
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Proportional Control Variables
Vd = 0.5; // Desired wheel speed is 0.5 m/s


// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksR()
{
    if (digitalRead(SIGNAL_BR) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksR--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksR++;
    }
}
void decodeEncoderTicksL()
{
    if (digitalRead(SIGNAL_BL) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksL--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksL++;
    }
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
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksR, RISING);
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
        v_R = omega_R * RHO * (-1);
        v = compute_vehicle_speed(v_L, v_R);
        omega = compute_vehicle_rate(v_L, v_R);
        
        

        // Print some stuff to the serial monitor
        /*
        Serial.print("Encoder ticks L: ");
        Serial.print(encoder_ticksL);
        Serial.print("\t");
        */
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\t");
        /*
        Serial.print("Encoder ticks R: ");
        Serial.print(encoder_ticksR);
        Serial.print("\t");
        */
        Serial.print("Estimated right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n");
        Serial.print("\n");
        Serial.print("Estimated overall speed: ");
        Serial.print(v);
        Serial.print(" m/s");
        Serial.print("\n");
        Serial.print("Estimated overall angular rate: ");
        Serial.print(omega);
        Serial.print(" rad/s");
        Serial.print("\n");
        

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticksR = 0;
        encoder_ticksL = 0;
    }

    // Find the error in wheel speed
    e_nowR = Vd - v_R;
    e_nowL = Vd - v_L;
    
    // Set the wheel motor PWM command [0-255] org. 128
    uint8_t u_L = 180 + kp * e_nowL;
    uint8_t u_R = 180 + kp * e_nowR;

    // Select a direction
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);

    // PWM command to the motor driver
    analogWrite(EA, uR);
    analogWrite(EB, uL);
}
