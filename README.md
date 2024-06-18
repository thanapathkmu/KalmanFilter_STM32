## Kalman Filter Implementation in C

This repository provides a basic implementation of the Kalman Filter algorithm in C. The Kalman Filter is used for estimating the state of a dynamic system by combining noisy sensor measurements with a system model.

### Files
* kalman.h: Header file defining the KalmanFilter struct and function prototypes.
* kalman.c: Implementation file containing functions for initializing and updating the Kalman Filter.

### Include kalman.h in your C code: 

```c
#include "kalman.h"
```

### Define a KalmanFilter struct:**

```c
KalmanFilter myFilter;
```

### Initialize the filter using KalmanFilter_Init:**
```c
KalmanFilter_Init(&myFilter, processNoiseCov, measurementNoiseCov);
```

* processNoiseCov: Process noise covariance (Q).
* measurementNoiseCov: Measurement noise covariance (R).

**Update the filter with a new measurement using KalmanFilter_Update:**

```c
float filteredValue = KalmanFilter_Update(&myFilter, measurement);
```

* measurement: New measurement value.
* filteredValue: Filtered state estimate returned by the function.

### Example
```c
#include <stdio.h>
#include "kalman.h"

int main() {
  // Define filter variables
  float Q = 0.01f;
  float R = 0.1f;
  float trueValue = 1.0f;
  float measurement;

  // Initialize Kalman filter
  KalmanFilter myFilter;
  KalmanFilter_Init(&myFilter, Q, R);

  // Simulate noisy measurements
  for (int i = 0; i < 10; i++) {
    measurement = trueValue + (float)rand() / (RAND_MAX / 0.2f) - 0.1f; // Add noise

    // Update filter and get filtered value
    float filteredValue = KalmanFilter_Update(&myFilter, measurement);

    printf("True value: %.2f, Measurement: %.2f, Filtered value: %.2f\n", trueValue, measurement, filteredValue);
  }

  return 0;
}
```
This example demonstrates how to initialize and use the Kalman Filter to estimate a true value from noisy measurements in a C program. The filter initialization (KalmanFilter_Init) sets up the process and measurement noise covariances, while KalmanFilter_Update iteratively updates the filter with new measurements to produce filtered estimates.

Feel free to integrate this Kalman Filter implementation into your projects by cloning or forking this repository!

## Summary

This repository hosts a C implementation of the Kalman Filter, a versatile algorithm for state estimation in dynamic systems. It includes header (kalman.h) and implementation (kalman.c) files defining and implementing the KalmanFilter struct and necessary functions. Users can initialize the filter with process and measurement noise covariances and update it with new measurements to obtain filtered state estimates. An example illustrates its usage in filtering noisy sensor measurements, aiding integration into diverse applications.
