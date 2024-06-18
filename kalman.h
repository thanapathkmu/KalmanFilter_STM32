#ifndef KALMAN_H_
#define KALMAN_H_

typedef struct {
    float x;           // State estimate
    float P;           // Error covariance
    float Q;           // Process noise covariance
    float R;           // Measurement noise covariance
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R);
float KalmanFilter_Update(KalmanFilter *kf, float z);

#endif /* KALMAN_H_ */
