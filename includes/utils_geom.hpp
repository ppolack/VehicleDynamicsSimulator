#pragma once
#include <math.h>

/*
 * Normalize angle between ]-Pi;Pi]
 */
double normalizeAngle(double x) {
    while (x <= -M_PI) {
        x += 2 * M_PI;
    }
    while (x > M_PI) {
        x -= 2 * M_PI;
    }
    return x;
}
