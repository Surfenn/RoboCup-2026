#include "colorsensor.h"        // Includes the ColorSensor class declarations
#include <Wire.h>               // Includes I2C communication support
#include "Adafruit_TCS34725.h"  // Includes the Adafruit library for the TCS34725/TCS3472-family color sensor

// I2C addresses
static const uint8_t TCA_ADDR = 0x70;  // I2C address of the TCA9548A multiplexer
static const uint8_t TCS_ADDR = 0x29;  // I2C address of each TCS color sensor

// Threshold above baseline to detect boundary
static const uint16_t BUFFER = 150;    // Minimum increase above baseline required to treat something as a boundary

// Arrays (8 sensors)
static uint16_t analogValues[ColorSensor::NUM_SENSORS];  // Stores the latest reading from each sensor
static uint16_t greenValues[ColorSensor::NUM_SENSORS];   // Stores the baseline reading for each sensor from startup

// Re-used sensor object (we switch mux channels before reading)
static Adafruit_TCS34725 tcs(                          // Creates one reusable color sensor object
  TCS34725_INTEGRATIONTIME_50MS,                       // Sets integration time to 50 ms
  TCS34725_GAIN_4X                                     // Sets sensor gain to 4x
);

// ---------- helpers ----------
static void tcaSelect(uint8_t channel) {              // Selects one channel on the TCA9548A multiplexer
  if (channel > 7) return;                            // Only channels 0 through 7 are valid
  Wire1.beginTransmission(TCA_ADDR);                  // Starts I2C transmission to the multiplexer on Wire1
  Wire1.write(1 << channel);                          // Enables only the chosen channel
  Wire1.endTransmission();                            // Ends the I2C transmission
}

static bool i2cDetect(uint8_t addr) {                 // Checks whether a device responds at a given I2C address
  Wire1.beginTransmission(addr);                      // Starts I2C transmission to the target address
  return (Wire1.endTransmission() == 0);              // Returns true if the device acknowledged
}

// Choose what value you want to treat as “analog” for line detect.
// Green works if line is green-ish; Clear often works best for white lines.
static uint16_t readSensorValue(uint8_t ch) {         // Reads the chosen measurement value from one sensor channel
  tcaSelect(ch);                                      // Switches the multiplexer to the requested sensor channel
  delayMicroseconds(500);                             // Small delay to allow the mux channel to settle

  uint16_t r, g, b, c;                                // Variables to store raw red, green, blue, and clear values
  tcs.getRawData(&r, &g, &b, &c);                     // Reads raw RGBC values from the selected sensor

  // return g;   // use GREEN (switch to `return c;` if your line is white)
  return c;                                          // Returns the clear-channel reading, which is better for white line detection
}

// ---------- class methods ----------
void ColorSensor::init() {                            // Initializes the color sensor system
  Wire1.begin();                                      // Starts the secondary I2C bus

  // Detect mux
  if (!i2cDetect(TCA_ADDR)) {                         // Checks whether the multiplexer is present
    Serial.println("ERROR: TCA9548A (0x70) not found on Wire1.");  // Prints an error if the mux is missing
    return;                                           // Exits initialization instead of freezing forever
  }

  // Detect & init each sensor on channels 0..7
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {      // Loops through all sensor channels
    tcaSelect(ch);                                    // Selects the current mux channel
    delay(5);                                         // Gives the channel time to settle

    if (!i2cDetect(TCS_ADDR)) {                       // Checks whether a color sensor responds on this channel
      Serial.print("ERROR: TCS34725 (0x29) not found on mux channel ");  // Prints an error message prefix
      Serial.println(ch);                             // Prints the failing channel number
      while (1) {}                                    // Stops the program forever if a sensor is missing on this channel
    }

    if (!tcs.begin(TCS_ADDR, &Wire1)) {               // Initializes the sensor object on the selected channel
      Serial.print("ERROR: TCS34725 begin() failed on channel ");  // Prints error message prefix
      Serial.println(ch);                             // Prints the failing channel number
      while (1) {}                                    // Stops the program forever if sensor initialization fails
    }
  }

  // Take baseline (assumes robot starts on field, not on line)
  delay(200);                                         // Waits briefly before taking baseline readings
  updateReadings();                                   // Reads all current sensor values
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {         // Loops through all sensors
    greenValues[i] = analogValues[i];                 // Saves the startup reading as the baseline for that sensor
  }
}

uint16_t* ColorSensor::getAnalogValues() {            // Returns a pointer to the latest sensor readings array
  return analogValues;                                // Gives access to the current readings
}

void ColorSensor::updateReadings() {                  // Updates all sensor readings
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {         // Loops through every sensor
    analogValues[i] = readSensorValue(i);             // Reads and stores the value for that sensor
  }
}

void ColorSensor::printReadings() {                   // Prints the current sensor readings to Serial
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {         // Loops through each sensor
    Serial.print(i);                                  // Prints the sensor index
    Serial.print(": ");                               // Prints a separator
    Serial.print(analogValues[i]);                    // Prints that sensor’s current reading
    Serial.print("\t");                               // Prints a tab for readability
  }
  Serial.println();                                   // Ends the line after printing all readings
}

void ColorSensor::printGreenValues() {                // Prints the stored baseline values to Serial
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {         // Loops through each sensor
    Serial.print(i);                                  // Prints the sensor index
    Serial.print(": ");                               // Prints a separator
    Serial.print(greenValues[i]);                     // Prints that sensor’s baseline value
    Serial.print("\t");                               // Prints a tab for readability
  }
  Serial.println();                                   // Ends the line after printing all baseline values
}

float ColorSensor::getAvoidAngle() {                  // Calculates the direction the robot should move to avoid the detected line
  float weightedX = 0.0f;                             // X component of weighted line vector
  float weightedY = 0.0f;                             // Y component of weighted line vector
  float totalWeight = 0.0f;                           // Sum of all contributing sensor weights

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {         // Loops through all sensors
    int diff = (int)analogValues[i] - (int)greenValues[i];  // Computes how much brighter than baseline this sensor is
    if (diff <= (int)BUFFER) continue;                // Ignores this sensor if it does not exceed the threshold

    float w = (float)(diff - (int)BUFFER);            // Converts excess brightness into a weight
    float angleRad = (i * 2.0f * (float)M_PI) / (float)NUM_SENSORS;  // Assigns this sensor an angle around the robot

    weightedX += w * cosf(angleRad);                  // Adds this sensor’s weighted X direction
    weightedY += w * sinf(angleRad);                  // Adds this sensor’s weighted Y direction
    totalWeight += w;                                 // Adds this sensor’s weight to the total
  }

  if (totalWeight <= 0.0f) return -1.0f;              // Returns -1 if no line was detected strongly enough

  float lineAngle = atan2f(weightedY, weightedX) * (180.0f / (float)M_PI);  // Converts the weighted vector into an angle in degrees
  if (lineAngle < 0) lineAngle += 360.0f;             // Normalizes angle into the range 0 to 360

  // Move away from line direction
  return fmodf(lineAngle + 180.0f, 360.0f);           // Returns the opposite direction so the robot moves away from the line
}