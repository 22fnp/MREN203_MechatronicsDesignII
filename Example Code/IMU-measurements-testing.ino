/**
 * @file IMU-measurements.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read the LSM6DS3 IMU on the Arduino UNO WiFi Rev2.
 * @version 1.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

// Need this library installed to access the UNO Wifi Rev2 board's IMU
// For more details, look here: https://www.arduino.cc/reference/en/libraries/arduino_lsm6ds3/
#include <Arduino_LSM6DS3.h>

// Wheel PWM pin (must be a PWM pin)
int EA = 6;
int EB = 5;

// Wheel direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 11;
int I4 = 10;

// Left wheel encoder digital pins
const byte SIGNAL_AL = 8;
const byte SIGNAL_BL = 9;
const byte SIGNAL_AR = 2;
const byte SIGNAL_BR = 3;

// Motor PWM command variable [0-255]
byte u = 0;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle's track length
const double ELL = 0.2775;

// Sampling interval for measurements in milliseconds
const int T = 10;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;
long t = 0;

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

// Variables to store angular rates from the gyro [degrees/s]
float omega_x, omega_y, omega_z;

// Variables to store accelerations [g's]
float a_x, a_y, a_z;

// Variables to store sample rates from sensor [Hz]
float a_f, g_f;


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

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);

    // Wait for serial connection before starting
    while (!Serial)
    {
        delay(10);
    }

    Serial.println();

    // Check that the board is initialized
    if (!IMU.begin())
    {
        // Print an error message if the IMU is not ready
        Serial.print("Failed to initialize IMU :(");
        Serial.print("\n");
        while (1)
        {
            delay(10);
        }
    }

    // Read the sample rate of the accelerometer and gyroscope
    a_f = IMU.accelerationSampleRate();
    g_f = IMU.gyroscopeSampleRate();

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

    // Print these values to the serial window
    Serial.print("Accelerometer sample rate: ");
    Serial.println(a_f);
    Serial.print("Gyroscope sample rate: ");
    Serial.println(g_f);
}

void loop()
{
    for (u=0; u <= 255; u += 5) {
    // Get the elapsed time [ms]
    t_now = millis();
    t = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(T);
        omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(T);
        v_L = omega_L * RHO;
        v_R = omega_R * RHO * (-1);
        v = compute_vehicle_speed(v_L, v_R);
        omega = compute_vehicle_rate(v_L, v_R);
        
//        Serial.print("Estimated left wheel speed: ");
//        Serial.print(omega_L);
//        Serial.print(" rad/s");
//        Serial.print("\t");
//        Serial.print("Estimated right wheel speed: ");
//        Serial.print(omega_R);
//        Serial.print(" rad/s");
//        Serial.print("\n");
//        Serial.print("\n");
//        Serial.print("Estimated overall speed: ");
//        Serial.print(v);
//        Serial.print(" m/s");
//        Serial.print("\n");
//        Serial.print("Estimated overall angular rate: ");
        

        // Record the current time [ms]
       
       
        
    }
    
    
    Serial.print(omega);
        // Serial.print(" rad/s");
        Serial.print("\t");
    
    // Timing in the loop is controlled by the IMU reporting when
    // it is ready for another measurement.
    // The accelerometer and gyroscope output at the same rate and
    // will give us their measurements at a steady frequency.

    // Read from the accelerometer
    /*
    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(a_x, a_y, a_z);

        /// Print the accelerometer measurements to the Serial Monitor
        Serial.print(a_x);
        Serial.print("\t");
        Serial.print(a_y);
        Serial.print("\t");
        Serial.print(a_z);
        Serial.print(" g\t\t");
    }
    */
    // Read from the gyroscope
    if (IMU.gyroscopeAvailable())
    {
        IMU.readGyroscope(omega_x, omega_y, omega_z);
        omega_z = omega_z + 0.24;
        omega_z = omega_z * (3.14159265 / 180);
        
        // Print the gyroscope measurements to the Serial Monitor
//        Serial.print(omega_x);
//        Serial.print("\t");
//        Serial.print(omega_y);
//        Serial.print("\t");
        Serial.print(omega_z);
        Serial.print("\t");
    }

    
    Serial.print(u);
    Serial.print("\t");
    Serial.print(t);
    Serial.print("\n");

    
    // Set the wheel motor PWM command [0-255] org. 128

    // Select a direction
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    
    
      // PWM command to the motor driver
      analogWrite(EA, u);
      analogWrite(EB, u+100);
      delay(50);
    }
    
    
}
