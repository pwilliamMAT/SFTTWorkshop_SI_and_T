/*
 * trackingEKF.h
 *
 * Code generation for function 'trackingEKF'
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
void trackingEKF_correct(const emlrtStack *sp, trackingEKF *filter,
                         const real_T varargin_1[4],
                         const real_T varargin_3_OriginPosition[9],
                         const real_T varargin_3_OriginVelocity[9],
                         const real_T varargin_3_Orientation[27]);

void trackingEKF_residual(const emlrtStack *sp, trackingEKF *EKF,
                          const real_T z[4],
                          const real_T c_measurementParams_f2_OriginPo[9],
                          const real_T c_measurementParams_f2_OriginVe[9],
                          const real_T c_measurementParams_f2_Orientat[27],
                          real_T res[4], real_T S[16]);

/* End of code generation (trackingEKF.h) */
