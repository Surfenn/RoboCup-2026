#include "compass.h"        // Includes the Compass class definition and required libraries

// Initializes the BNO055 compass sensor
void Compass::initialize() {
  Wire.begin();             // Starts I2C communication (default I2C bus)
  delay(10);                // Small delay to allow I2C bus to stabilize

  if (!bno.begin()) {       // Attempts to initialize the BNO055 sensor
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"); // Prints error if sensor not found
    ok = false;             // Marks sensor as not working
    return;                 // Exit function (prevents robot from freezing)
  }

  ok = true;                // Marks sensor as successfully initialized

  bno.setExtCrystalUse(true);      // Enables use of external crystal for better accuracy
  bno.setMode(OPERATION_MODE_NDOF); // Sets sensor to full fusion mode (absolute orientation)
  delay(50);                      // Waits briefly for mode change to take effect
}

// Reads the current compass heading
float Compass::readCompass() {
  if (!ok) return NAN;      // If sensor failed initialization, return invalid value

  sensors_event_t event;    // Creates a structure to hold sensor data
  bno.getEvent(&event);     // Retrieves the latest orientation data from the sensor

  float heading = event.orientation.x; // Extracts heading (yaw) in degrees (typically 0–360)

  // Normalize heading to stay within 0 to 360 degrees
  while (heading < 0) heading += 360.0f;     // If negative, wrap around to positive range
  while (heading >= 360.0f) heading -= 360.0f; // If >=360, wrap back into range

  return heading;           // Returns normalized heading value
}