#include "IR.h"          // Includes the header file for the IR class, where the class methods and declarations are defined.
#include <string>        // Includes the C++ string library. Not currently used in active code, but needed for the commented-out string function below.
#include <limits.h>      // Includes integer limit constants like INT_MAX and INT_MIN.

int pins[] = {23, 24, 25, 26, 27, 38, 39, 40, 41, 14, 15, 18, 19, 20, 21, 22}; // Stores the 16 Teensy pins connected to the IR sensors.
// {38, 39, 20, 21, 22, 23, 31, 32, 24, 25, 26, 27, 14, 15, 16, 17}             // Old pin layout kept as a reference.

const int NUM_IR_PINS = 16;   // Constant storing the total number of IR sensors.

double readings[NUM_IR_PINS]; // Global array that stores the normalized reading for each IR sensor.

void IR::initIR() { // Defines the IR class function initIR(), which initializes the IR sensor pins.
  for (unsigned int i = 0; i < arrayLength(pins); i++) { // Loops through every element in the pins array.
    pinMode(pins[i], INPUT); // Sets each IR sensor pin as an input so the Teensy can read signals from it.
  }
} 

void IR::updateReadings() { // Defines the IR class function updateReadings(), which updates the normalized IR readings.
  double pwReadings[NUM_IR_PINS];
  double minVal = INT_MAX;  // Starts minVal at the largest possible int so any real pulse reading will be smaller.
  double maxVal = INT_MIN;  // Starts maxVal at the smallest possible int so any real pulse reading will be larger.

  for (int i = 0; i < NUM_IR_PINS; i++) { // First pass through all sensors to find the minimum and maximum pulse widths.
    pwReadings[i] = pulseIn(pins[i], HIGH, 800); // Measures how long the signal on this pin stays HIGH, with an 800 microsecond timeout.
    if (pwReadings[i] != 0 && pwReadings[i] < minVal) minVal = pwReadings[i]; // If the reading is valid and smaller than current min, update minVal.
    if (pwReadings[i] > maxVal) maxVal = pwReadings[i]; // If the reading is larger than current max, update maxVal.
  }

  if (minVal == INT_MAX || maxVal == INT_MIN) { // Checks if no valid readings were found at all.
    for (int i = 0; i < NUM_IR_PINS; i++) { // Loops through every sensor index.
      readings[i] = 0; // Sets every normalized reading to 0 because no ball/signal was detected.
    }
    return;
  }

  double range = maxVal - minVal; // Computes the difference between the largest and smallest pulse widths.

  if (range == 0){
    for (int i = 0; i < NUM_IR_PINS; i++){
      readings[i] = 0; 
    }
    return;
  }

  for (int i = 0; i < NUM_IR_PINS; i++) { // Second pass through all sensors to calculate normalized readings.
    if (pwReadings[i] == 0) { // Checks if this sensor did not detect a valid pulse.
      readings[i] = 0; // Stores 0 for this sensor.
    }
    else { // Runs if the reading is valid.
      readings[i] = 1.0 - ((pwReadings[i] - minVal) / range); // Normalizes the reading so shorter pulse widths become larger weights, closer to 1.
    }
  }
}

// float* IR::getReadingsArr() { // Defines a function that returns a newly created float array of normalized readings.
//   double* pwReadings = getPWsArr(); // Calls getPWsArr() to get the raw pulse-width readings for all sensors.
//   float* pinReadings = new float[NUM_IR_PINS]; // Dynamically allocates a new float array to hold normalized values.

//   double minVal = INT_MAX; // Starts minVal at a very large value.
//   double maxVal = INT_MIN; // Starts maxVal at a very small value.

//   for (int i = 0; i < NUM_IR_PINS; i++) { // Loops through all pulse-width readings.
//     if (pwReadings[i] != 0 && pwReadings[i] < minVal) minVal = pwReadings[i]; // Updates minVal using the smallest nonzero reading.
//     if (pwReadings[i] > maxVal) maxVal = pwReadings[i]; // Updates maxVal using the largest reading.
//   }

//   if (minVal == INT_MAX || maxVal == INT_MIN) { // Checks if no valid pulse-width readings were found.
//     for (int i = 0; i < NUM_IR_PINS; i++) { // Loops through all sensor indices.
//       pinReadings[i] = 0; // Sets every normalized reading in the new array to 0.
//     }
//     return pinReadings; // Returns the all-zero array immediately.
//   }

//   double range = maxVal - minVal; // Calculates the spread between the largest and smallest pulse widths.

//   for (int i = 0; i < NUM_IR_PINS; i++) { // Loops through all raw pulse-width readings again.
//     if (pwReadings[i] == 0) { // Checks if the reading is invalid or no signal was detected.
//       pinReadings[i] = 0; // Stores 0 for that sensor.
//     }
//     else { // Runs when the reading is valid.
//       pinReadings[i] = (float)(1 - ((pwReadings[i] - minVal) / range)); // Normalizes the pulse width into a value from 0 to 1.
//     }
//   }

//   return pinReadings; // Returns the newly allocated normalized readings array.
// }

// std::string IR::stringBallReadings() { // Commented-out function that would return all readings as a formatted string.
//     string ballReadings = ""; // Creates an empty string to build the result.
//     currReadings = getReadingsArr(); // Gets the current normalized readings array.
//
//     for (int i = 0; i < NUM_IR_PINS; i++) { // Loops through all IR sensors.
//         ballReadings += "Pin #: " + std::to_string(pins[i]) + // Adds the pin number text.
//                         " has a value of: " + std::to_string(currReadings[i]) + // Adds the reading value text.
//                         "\n"; // Adds a newline after each sensor reading.
//     }
//
//     return ballReadings; // Returns the complete formatted string.
// }

float IR::getBallAngle() { // Defines a function that estimates the angle of the ball based on sensor readings.
  int n = NUM_IR_PINS; // Stores the total number of sensors locally for easier use.

  float dampen = 2.5;    // Sets the exponent used to strengthen stronger sensor readings and weaken smaller ones.
  float weightedX = 0; // Will store the x-component of the weighted vector sum.
  float weightedY = 0; // Will store the y-component of the weighted vector sum.
  float totalWeight = 0; // Will store the sum of all used weights.

  for (int i = 0; i < n; i++) { // Loops through every sensor.
    float weight = readings[i] > 0.95 ? 1 : readings[i]; // If a reading is very high, clamp it to 1; otherwise use its actual value.
    float angleRad = (i * 2.0 * M_PI) / n; // Converts this sensor's position around the circle into an angle in radians.

    if (weight < 0.5) continue; // Ignores weak readings below 0.5 so they do not affect the ball angle.

    float adjustedWeight = pow(weight, dampen); // Raises the weight to a power so stronger readings dominate more.

    weightedX += adjustedWeight * cos(angleRad); // Adds this sensor's weighted x-direction contribution.
    weightedY += adjustedWeight * sin(angleRad); // Adds this sensor's weighted y-direction contribution.
    totalWeight += adjustedWeight; // Adds this sensor's adjusted weight to the total.
  }

  if (totalWeight == 0) { // Checks if no sensor had a strong enough reading.
      return -1;          // Returns -1 to show that no valid ball direction was found.
  }

  float normX = weightedX / totalWeight; // Divides the total x-value by total weight to normalize it.
  float normY = weightedY / totalWeight; // Divides the total y-value by total weight to normalize it.

  float angle = atan2(normY, normX) * (180.0 / M_PI); // Converts the final x/y vector into an angle in degrees.

  if (angle < 0) angle += 360.0; // If the angle is negative, shifts it into the range 0 to 360 degrees.

  return angle; // Returns the detected ball angle in degrees.
}


// double* IR::getPWsArr() { // Commented-out function that would return raw pulse widths for every sensor.
//   double* pinReadings = new double[NUM_IR_PINS]; // Allocates a new array for raw pulse-width readings.
//   for (unsigned int i = 0; i < arrayLength(pins); i++) { // Loops through all sensor pins.
//     pinReadings[i] = pulseIn(pins[i], HIGH, 800); // Measures the HIGH pulse width for each sensor.
//   }
//   return pinReadings; // Returns the raw pulse-width array.
// }

void IR::printReadingsArr() { // Defines a function to print the normalized readings array to the Serial Monitor.
  Serial.print("[ "); // Prints the opening bracket for the array display.
  for(int i = 0; i < NUM_IR_PINS; i++) { // Loops through all normalized readings.
    Serial.print(readings[i]); // Prints the current reading value.
    Serial.print(" "); // Prints a space after each value for readability.
  }
  Serial.println("]"); // Prints the closing bracket and ends the line.
}

// void IR::printPWsArr() { // Commented-out function that would print raw pulse-width values.
//   double* arr = getPWsArr(); // Gets the raw pulse-width array.
//   Serial.print("[ "); // Prints an opening bracket.
//   for(int i = 0; i < NUM_IR_PINS; i++) { // Loops through all raw readings.
//     Serial.print(arr[i]); // Prints each raw pulse-width value.
//     Serial.print(" "); // Prints a space after each value.
//   }
//   Serial.println("]"); // Prints the closing bracket and ends the line.
// }