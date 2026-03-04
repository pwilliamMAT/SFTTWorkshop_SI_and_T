/*
 * EKFStateEstimator.h
 *
 * Code generation for function 'EKFStateEstimator'
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
real_T EKFStateEstimator_likelihood(
    const emlrtStack *sp,
    const c_fusion_tracker_sensorspecs_Ae *estimator_SensorSpecifications,
    trackingEKF *estimator_TrackingFilter, const real_T pdf_State[6],
    const real_T pdf_StateCovariance[36], const real_T measurement[4]);

void EKFStateEstimator_merge(const emlrtStack *sp,
                             const c_emxArray_struct_T *pdfs,
                             const emxArray_real_T *weights, struct_T *pdf);

/* End of code generation (EKFStateEstimator.h) */
