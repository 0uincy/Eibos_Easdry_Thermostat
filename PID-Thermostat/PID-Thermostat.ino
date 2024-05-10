#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <PID_v1.h>
#include <LapX9C10X.h>

// Create the display object using the SH1106 constructor for I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Create the sensor object
Adafruit_AHTX0 aht;

// Define digital potentiometer control pins
#define UDPIN 12
#define CSPIN 13
#define INCPIN 14
LapX9C10X heat(INCPIN, UDPIN, CSPIN, LAPX9C10X_X9C104);

// Define PID constants
double Kp = 0.5;  // Proportional constant
double Ki = 0.1;  // Integral constant
double Kd = 0.1;  // Derivative constant

// Define PID variables
double input, output, setpoint;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long targetTemp = 30;
int potPin = 0;
int potValue = 0;
int wiper = 0;
int heatOutput = 99;

void setup() {
  display.begin();  // Initialize the display
  if (!aht.begin()) {
    display.clearBuffer();          
    display.setFont(u8g2_font_luRS10_tf);
    display.drawStr(0, 10, "AHT10 not found!");
    display.sendBuffer();
    while(1);  // Stay here forever if the sensor is not found
  }

  // Read values from the sensor
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // Initialize the PID
  input = temp.temperature; // Read initial temperature
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 99); // Output range for digital potentiometer (0-100 for percentage)
  setpoint = targetTemp; // Set the target temperature

  // Initialize X9C104 
  heat.begin(0);

  Serial.begin(115200);
}


void loop() {
  // Set target temperature with potentiometer
  potValue = analogRead(potPin);
  targetTemp = 30+(potValue-13)/25.25;

  // Read values from the sensor
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // Compute PID output
  setpoint = targetTemp;
  input = temp.temperature;
  myPID.Compute();
  heatOutput = 99 - output;
  heat.set(heatOutput);
  wiper = heat.get();

  // Print temperature and PID output for debugging
  Serial.print("Temperature: ");
  Serial.print(input);
  Serial.print(", Target: ");
  Serial.print(setpoint);
  Serial.print(", PID Output: ");
  Serial.print(output);
  Serial.print(" %");
  Serial.print(", Wiper Position: ");
  Serial.println(wiper);

  // Prepare the display buffer
  display.clearBuffer();
  display.setFont(u8g2_font_luRS10_tf);

  // Create strings for temperature and humidity
  char currentStr[16];
  char targetStr[16];
  char humStr[16];
  
  sprintf(currentStr, "Temp: %.1f C", temp.temperature);
  sprintf(targetStr, "Target: %.d C", targetTemp);
  sprintf(humStr, "Heat: %.f %%", output+1);

  // Print the strings to the display buffer
  display.drawStr(0, 14, currentStr);
  display.drawStr(0, 32, targetStr);
  display.drawStr(0, 60, humStr);

  // Send the buffer to the display
  display.sendBuffer();
}