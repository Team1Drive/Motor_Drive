#include "modulation.h"
#include <cmath>
#include <cstdint>

#define M_PI 3.14159265358979323846264338327950288f
#define SQRT3 1.73205080756887729352744634150587236f

void SVPWM(float v_alpha, float v_beta, float v_dc, float* dutyA, float* dutyB, float* dutyC) {
    // θ calculation
    float theta = atan2f(v_beta, v_alpha);
    if (theta < 0) {
        theta += 2U * M_PI;
    }

    // Sector
    const float sector_angle = M_PI / 3.0f;         // 60 degrees in radians
    int sector = (int)(theta / sector_angle);       // Determine sector of θ
    float theta_s = theta - sector * sector_angle;  // Angle within sector
    float theta_sn = sector_angle - theta_s;        // Complementary angle within sector

    // Magnitude
    float v_ref = sqrtf(v_alpha * v_alpha + v_beta * v_beta);   // Reference voltage magnitude
    float v_ratio = v_ref / v_dc;                   // Voltage ratio (normalized magnitude)

    const float csc60 = 2 / SQRT3;                  // Cosecant of 60 degrees
    
    // Duty cycle
    float d2 = sinf(theta_s) * v_ratio * csc60;     // Far side duty
    float d1 = sinf(theta_sn) * v_ratio * csc60;    // Near side duty
    float d0 = 1.0f - d1 - d2;                      // Zero vector duty

    float da, db, dc;

    switch (sector) {
    case 0:
        // Sector 1
        da = d0 / 2.0f + d1 + d2;
        db = d0 / 2.0f + d2;
        dc = d0 / 2.0f;
        break;
    case 1:
        // Sector 2
        da = d0 / 2.0f + d1;
        db = d0 / 2.0f + d1 + d2;
        dc = d0 / 2.0f;
        break;
    case 2:
        // Sector 3
        da = d0 / 2.0f;
        db = d0 / 2.0f + d1 + d2;
        dc = d0 / 2.0f + d2;
        break;
    case 3:
        // Sector 4
        da = d0 / 2.0f;
        db = d0 / 2.0f + d1;
        dc = d0 / 2.0f + d1 + d2;
        break;
    case 4:
        // Sector 5
        da = d0 / 2.0f + d2;
        db = d0 / 2.0f;
        dc = d0 / 2.0f + d1 + d2;
        break;
    case 5:
        // Sector 6
        da = d0 / 2.0f + d1 + d2;
        db = d0 / 2.0f;
        dc = d0 / 2.0f + d1;
        break;
    default:
        da = db = dc = 0.0f;
        break;
    }

    if (da < 0.0f) da = 0.0f;
    if (db < 0.0f) db = 0.0f;
    if (dc < 0.0f) dc = 0.0f;
    if (da > 1.0f) da = 1.0f;
    if (db > 1.0f) db = 1.0f;
    if (dc > 1.0f) dc = 1.0f;

    *dutyA = da;
    *dutyB = db;
    *dutyC = dc;
}

void clarke(float a, float b, float c, float* alpha, float* beta) {
    *alpha = (2U * a - (b + c)) / 3.0f;
    *beta = (b - c) / SQRT3;
}

void inv_clarke(float alpha, float beta, float* a, float* b, float* c) {
    *a = alpha;
    *b = (-alpha + SQRT3 * beta) / 2.0f;
    *c = (-alpha - SQRT3 * beta) / 2.0f;
}

void park(float alpha, float beta, float theta, float* d, float* q) {
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

void inv_park(float d, float q, float theta, float* alpha, float* beta) {
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}