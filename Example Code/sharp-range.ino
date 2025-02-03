/**
 * @file sharp-range.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <math.h>

// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN_FRONT = A5;
const byte SHARP_PIN_RIGHT = A4;
const byte SHARP_PIN_LEFT = A3;

float x_front = 0.0;
float x_right = 0.0;
float x_left = 0.0;

int sharp_val_front_avrg = 0;
int sharp_val_right_avrg = 0;
int sharp_val_left_avrg = 0;

// Variables to store the proximity measurement
int sharp_val_front = 0; // integer read from analog pin
int sharp_val_right = 0;
int sharp_val_left = 0;
float sharp_range; // range measurement [cm]

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);
}

void loop()
{
    sharp_val_front = analogRead(SHARP_PIN_FRONT);
    sharp_val_right = analogRead(SHARP_PIN_RIGHT);
    sharp_val_left = analogRead(SHARP_PIN_LEFT);

    
    for(int i = 0; i < 10; i++) {
      // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
      sharp_val_front += analogRead(SHARP_PIN_FRONT);
      sharp_val_right += analogRead(SHARP_PIN_RIGHT);
      sharp_val_left += analogRead(SHARP_PIN_LEFT);
    }

    sharp_val_front_avrg = sharp_val_front / 10;
    sharp_val_right_avrg = sharp_val_right / 10;
    sharp_val_left_avrg = sharp_val_left / 10;
    
    
    x_front = 160.72 * exp(-0.005*sharp_val_front_avrg);
    x_right = 160.72 * exp(-0.005*sharp_val_right_avrg);
    x_left = 160.72 * exp(-0.005*sharp_val_left_avrg);
    
    // Print all values
    Serial.print(sharp_val_front_avrg);
    Serial.print("\t");
    Serial.print(x_front);
    Serial.print("\n");

    // Delay for a bit before reading the sensor again
    delay(50);
}
