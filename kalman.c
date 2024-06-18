#include "kalman.h"

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R) {
    kf->x = 0.0f;      // Initial state estimate
    kf->P = 1.0f;      // Initial error covariance
    kf->Q = Q;         // Process noise covariance
    kf->R = R;         // Measurement noise covariance
}

float KalmanFilter_Update(KalmanFilter *kf, float z) {
    // Prediction update
    float x_pred = kf->x;
    float P_pred = kf->P + kf->Q;

    // Measurement update (Kalman gain calculation)
    float K = P_pred / (P_pred + kf->R);

    // State update
    kf->x = x_pred + K * (z - x_pred);

    // Error covariance update
    kf->P = (1 - K) * P_pred;

    return kf->x;   // Return the filtered value
}
