#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "gtest/gtest.h"

extern "C" {
    #include "common/maths.h"
    #include "common/filter.h"
}

#define M_PI_FLOAT 3.14159265358979323846f

fastKalman_t fkfFilter;
fastKalman_t fkfFilter2;
biquadFilter_t bqrcFilter;

TEST(BQRCKalmanTest, BQRCKalmanTest)
{
    float kalmanOut;
    float kalmanOut2;
    float bqrcOut;
    uint16_t randVal;
    uint16_t q = 300;
    uint16_t r = 88;
    uint8_t p = 0;
    float dT = 1.0f / 32000.0f;
    float bqrcQ = (float) q * 1e-6f;
    float bqrcR = (float) r * 1e-3f;
    float bqrcK = 2.0f * sqrt(bqrcQ) / (sqrt(bqrcQ + 4.0f * bqrcR) + sqrt(bqrcQ));
    float f_cut = 1.0f / (2.0f * M_PI_FLOAT * (0.5f * dT * ((1.0f / bqrcK) - 2.0f)));
    fastKalmanInit(&fkfFilter, q, r, p);
    fastKalmanInit(&fkfFilter2, q, r, p);
    biquadRCFIR2FilterInit(&bqrcFilter, f_cut, dT);

    srand(time(NULL));
    for (int i=1; i <= 500; i++) {
        randVal = rand() % 2000;
        kalmanOut = fastKalmanUpdate(&fkfFilter, (float)randVal);
        kalmanOut2 = fastKalmanUpdate2(&fkfFilter2, (float)randVal);
        bqrcOut = biquadFilterApply(&bqrcFilter, (float)randVal);
        printf("Input Value: %4d --- FKF K: %.10f --- FKF: %15.10f --- FKF2 K: %.10f --- FKF2: %15.10f --- BQRC: %15.10f\n", randVal, fkfFilter.k, kalmanOut, fkfFilter2.k, kalmanOut2, bqrcOut);
    }
    printf ("-------------------\nK :: FKF: %.10f -- FKF2: %.10f -- BQRC: %.10f\n", fkfFilter.k, fkfFilter2.k, bqrcFilter.b0 * 2.0f);
}

TEST(BQRCKalmanTest, BQRCKalmanTestKalmanK)
{
    float kalmanOut;
    float kalmanOut2;
    float bqrcOut;
    uint16_t randVal;
    uint16_t q = 300;
    uint16_t r = 88;
    uint8_t p = 0;
    float dT = 1.0f / 32000.0f;
    float bqrcQ = (float) q * 1e-6f;
    float bqrcR = (float) r * 1e-3f;
    float bqrcK = 2.0f * sqrt(bqrcQ) / (sqrt(bqrcQ + 4.0f * bqrcR) + sqrt(bqrcQ));
    float f_cut = 1.0f / (2.0f * M_PI_FLOAT * (0.5f * dT * ((1.0f / bqrcK) - 2.0f)));
    fastKalmanInit(&fkfFilter, q, r, p);
    biquadRCFIR2FilterInit(&bqrcFilter, f_cut, dT);

    srand(time(NULL));
    for (int i=1; i <= 500; i++) {
        randVal = rand() % 2000;
        kalmanOut = fastKalmanUpdate(&fkfFilter, (float)randVal);
        bqrcFilter.b0 = fkfFilter.k;
        bqrcFilter.b1 = fkfFilter.k;
        bqrcFilter.a1 = -(1.0f - fkfFilter.k * 2.0f);
        kalmanOut2 = fastKalmanUpdate2(&fkfFilter2, (float)randVal);
        bqrcOut = biquadFilterApply(&bqrcFilter, (float)randVal);
        printf("Input Value: %4d --- FKF K: %.10f --- FKF: %15.10f --- FKF2 K: %.10f --- FKF2: %15.10f --- BQRC: %15.10f\n", randVal, fkfFilter.k, kalmanOut, fkfFilter2.k, kalmanOut2, bqrcOut);
    }
    printf ("-------------------\nK :: FKF: %.10f -- FKF2: %.10f -- BQRC: %.10f\n", fkfFilter.k, fkfFilter2.k, bqrcFilter.b0 * 2.0f);
}