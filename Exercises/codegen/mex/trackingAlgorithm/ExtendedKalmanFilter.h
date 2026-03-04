/*
 * ExtendedKalmanFilter.h
 *
 * Code generation for function 'ExtendedKalmanFilter'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "trackingAlgorithm_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void ExtendedKalmanFilter_set_State(const emlrtStack *sp, trackingEKF *obj,
                                    const real_T b_value[6]);

void c_ExtendedKalmanFilter_set_Meas(const emlrtStack *sp, trackingEKF *obj,
                                     real_T b_value[16]);

void c_ExtendedKalmanFilter_set_Proc(const emlrtStack *sp, trackingEKF *obj,
                                     const real_T b_value[9]);

void c_ExtendedKalmanFilter_set_Stat(const emlrtStack *sp, trackingEKF *obj,
                                     const real_T b_value[36]);

void c_ExtendedKalmanFilter_validate(const emlrtStack *sp, trackingEKF *obj,
                                     const real_T varargin_2_OriginPosition[9],
                                     const real_T varargin_2_OriginVelocity[9],
                                     const real_T varargin_2_Orientation[27]);

/* End of code generation (ExtendedKalmanFilter.h) */
