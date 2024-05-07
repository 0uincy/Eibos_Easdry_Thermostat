#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Create the display object using the SH1106 constructor for I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Create the sensor object
Adafruit_AHTX0 aht;

// Define EC11 Rotary Encoder Pins
#define EncA 12;
#define EncB 14;

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;

volatile int counter = 0;

int targetTemp = 45;

void setup() {
  display.begin();  // Initialize the display
  if (!aht.begin()) {
    display.clearBuffer();          
    display.setFont(u8g2_font_luRS10_tf);
    display.drawStr(0, 10, "AHT10 not found!");
    display.sendBuffer();
    while(1);  // Stay here forever if the sensor is not found
  }

  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);

  Serial.begin(115200);
}

void loop() {
  static int lastCounter = 0;

  // If count has changed print the new value to serial
  if(counter != lastCounter){
    Serial.println(counter);
    lastCounter = counter;
  }

  // Read values from the sensor
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // Prepare the display buffer
  display.clearBuffer();
  display.setFont(u8g2_font_luRS10_tf);

  // Create strings for temperature and humidity
  char currentStr[16];
  char targetStr[16];
  char humStr[16];
  
  sprintf(currentStr, "Current: %.1f C", temp.temperature);
  sprintf(targetStr, "Target: %.f C", targetTemp);
  sprintf(humStr, "Humidity: %.f %%", humidity.relative_humidity);

  // Print the strings to the display buffer
  display.drawStr(0, 14, currentStr);
  display.drawStr(0, 32, targetStr);
  display.drawStr(0, 60, humStr);

  // Send the buffer to the display
  display.sendBuffer();

  // Update every 0.5 seconds
  delay(100);
}

void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
} 