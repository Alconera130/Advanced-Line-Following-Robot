#pragma once

#include <Arduino.h>

void PID(int position);
int weightedPos(int normalized[], int activeCount);