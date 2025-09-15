#pragma once

#include <array>
#include <cmath>
namespace Hilton {

class KalmanFilter {
public:
    KalmanFilter() : state{0.0}, bias{0.0}, estimateError(1.0), processNoise(1e-3), measurementNoise(1e-2) {}

    void initialize(double processNoise, double measurementNoise, double initialError = 1.0) {
        this->processNoise = processNoise;
        this->measurementNoise = measurementNoise;
        this->estimateError = initialError;
    }

    double estimate(double measurement, double rate, double deltaTime) {
        // Prediction step
        double rateCorrected = rate - bias;
        state += deltaTime * rateCorrected;
        estimateError += deltaTime * processNoise;

        // Update step
        double gain = estimateError / (estimateError + measurementNoise);
        state += gain * (measurement - state);
        bias += gain * rateCorrected; // Optional bias correction
        estimateError *= (1.0 - gain);

        return state;
    }

private:
    double state;          // Estimated state (e.g., orientation)
    double bias;           // Bias in the gyroscope
    double estimateError;  // Current error estimate
    double processNoise;   // Noise in the process (e.g., gyroscope)
    double measurementNoise; // Noise in the measurement (e.g., accelerometer)
};

}