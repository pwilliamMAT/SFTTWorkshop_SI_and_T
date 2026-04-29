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
void EKFStateEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *estimator_TargetSpecifications,
    trackingEKF *estimator_TrackingFilter, struct_T *pdf, real_T dT);

real_T c_EKFStateEstimator_detectionPr(
    const emlrtStack *sp,
    const c_fusion_tracker_sensorspecs_Ae *estimator_SensorSpecifications,
    const real_T pdf_State[6]);

real_T c_EKFStateEstimator_gateProbabi(const emlrtStack *sp, real_T gateSize);

/* End of code generation (EKFStateEstimator.h) */
