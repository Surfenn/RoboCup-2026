#ifndef IR_H
#define IR_H

class IR {
  public:

    float* getReadingsArr();
    double* getPWsArr();
    // void printReadingsArr();
    // void printPWsArr();
    void updateReadings();
    void initIR();
    float getBallAngle();

  private:
    float avgX = 0.0;
    float avgY = 0.0;

};

template <typename T, size_t N>
constexpr size_t arrayLength(const T (&)[N]) {
    return N;
}

#endif