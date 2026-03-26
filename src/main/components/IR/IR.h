#ifndef IR_H               // Include guard start → prevents multiple inclusions of this header file
#define IR_H
#include <string>          // Includes string library (not currently used, but kept in case needed later)
// IR class handles infrared sensor readings and ball angle calculation
class IR {
  public:
     // Prints the normalized IR readings (after updateReadings has processed them)
     void printReadingsArr();
     // Reads raw IR sensor values, normalizes them, and stores results internally
     void updateReadings();
     // Initializes IR sensor pins (sets them as INPUT)
     void initIR();
     // Calculates and returns the angle of the ball based on IR sensor readings
     float getBallAngle();
};

// Template function to get the size of a static array at compile time
// Example: int arr[5]; → arrayLength(arr) returns 5
template <typename T, size_t N>
constexpr size_t arrayLength(const T (&)[N]) {
    return N;             // Returns the number of elements in the array
}

#endif                    // End of include guard