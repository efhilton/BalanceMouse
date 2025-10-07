#pragma once

#include <array>
#include <cmath>
namespace Hilton {

class KalmanFilter {
public:
    KalmanFilter() : state{0.0}, bias{0.0}, estimateError(1.0), processNoise(1e-3), measurementNoise(1e-2) {}

    void initialize(const double newProcessNoise, const double newMeasurementNoise, const double newInitialError = 1.0) {
        this->processNoise = newProcessNoise;
        this->measurementNoise = newMeasurementNoise;
        this->estimateError = newInitialError;
    }

    double estimate(const double measurement, const double rate, const double deltaTime) {
        // Prediction step
        const double rateCorrected = rate - bias;
        state += deltaTime * rateCorrected;
        estimateError += deltaTime * processNoise;

        // Update step
        const double gain = estimateError / (estimateError + measurementNoise);
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