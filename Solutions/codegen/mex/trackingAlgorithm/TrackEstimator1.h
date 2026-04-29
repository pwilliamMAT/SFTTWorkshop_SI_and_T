/*
 * TrackEstimator1.h
 *
 * Code generation for function 'TrackEstimator1'
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
real_T TrackEstimator_distance(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, const struct_T *pdf,
    const real_T measurement[4], real_T b_time);

real_T TrackEstimator_likelihood(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, struct_T *pdf,
    const real_T measurement[4], real_T b_time);

void TrackEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    trackingEKF *d_estimator_StateEstimator_Stat, struct_T *pdf, real_T b_time);

real_T c_TrackEstimator_likelihoodUnas(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, struct_T *pdf, real_T b_time,
    real_T gateSize);

real_T c_TrackEstimator_sampleDistribu(
    trackingEKF *c_estimator_StateEstimator_Stat, uint32_T *pdf_TrackID,
    uint32_T *pdf_Age, boolean_T *pdf_IsConfirmed, boolean_T *pdf_IsCoasted,
    real_T pdf_State[6], real_T pdf_StateCovariance[36],
    real_T *pdf_ExistenceProbability);

/* End of code generation (TrackEstimator1.h) */
