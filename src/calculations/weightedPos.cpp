#include <Arduino.h>

#include "main.h"

int weightedPos(int normalized[], int activeCount) {
    long numerator = 0;
    long denominator = 0;
    int weights[NUM_SENSORS] = {-3000, -2000, -1000, -200, 200, 1000, 2000, 3000};

    if (activeCount == 0) {
        if (lastError < 0) return -3000; 
        if (lastError > 0) return 3000;  
        return 0;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        numerator += (long)normalized[i] * weights[i];
        denominator += normalized[i];
    }
    return (int)(numerator / denominator);
}