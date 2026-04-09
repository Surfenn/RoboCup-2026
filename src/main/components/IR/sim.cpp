#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

const int NUM_IR_PINS = 16;
float readings[NUM_IR_PINS];

float getBallAngle() {
  int n = NUM_IR_PINS;
  float sharpen = 2.5;
  float weightedX = 0.0;
  float weightedY = 0.0;
  float totalWeight = 0.0;

  for (int i = 0; i < n; i++) {
    float weight = readings[i];

    if (weight <= 0.0) continue;

    float angleRad = (-i * 2.0 * M_PI) / n;
    float adjustedWeight = pow(weight, sharpen);

    weightedX += adjustedWeight * cos(angleRad);
    weightedY += adjustedWeight * sin(angleRad);
    totalWeight += adjustedWeight;
  }

  if (totalWeight == 0.0) return -1;

  float angle = atan2(weightedY, weightedX) * 180.0 / M_PI;
  if (angle < 0) angle += 360.0;

  if (angle >= 360.0) angle -= 360.0;

  return angle;
}

int main() {
    for(int i=0; i<16; i++) readings[i] = 0;
    
    // Simulate ball on right (i=12)
    readings[12] = 1.0;
    readings[11] = 0.5;
    readings[13] = 0.5;
    
    float angleRight = getBallAngle();
    cout << "Angle when ball on right (i=12): " << angleRight << endl;
    
    for(int i=0; i<16; i++) readings[i] = 0;
    
    // Simulate ball on left (i=4)
    readings[4] = 1.0;
    readings[3] = 0.5;
    readings[5] = 0.5;
    
    float angleLeft = getBallAngle();
    cout << "Angle when ball on left (i=4): " << angleLeft << endl;
    
    for(int i=0; i<16; i++) readings[i] = 0;
    // Simulate ball on front (i=0)
    readings[0] = 1.0;
    readings[1] = 0.5;
    readings[15] = 0.5;
    
    float angleFront = getBallAngle();
    cout << "Angle when ball on front (i=0): " << angleFront << endl;
    
    return 0;
}
