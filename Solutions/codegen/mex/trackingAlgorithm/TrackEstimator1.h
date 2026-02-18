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
    trackingEKF *e_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *i_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *j_estimator_StateEstimator_Stat,
    trackingEKF *k_estimator_StateEstimator_Stat, b_struct_T *pdf,
    const real_T measurement[4], real_T b_time);

real_T TrackEstimator_likelihood(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *i_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *j_estimator_StateEstimator_Stat,
    trackingEKF *k_estimator_StateEstimator_Stat, b_struct_T *pdf,
    const real_T measurement[4], real_T b_time);

void TrackEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    trackingEKF *d_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *e_estimator_StateEstimator_Stat,
    trackingEKF *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat, b_struct_T *pdf,
    real_T b_time);

void TrackEstimator_setup(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                          i_fusion_tracker_internal_estim *estimator,
                          trackingEKF *iobj_0);

real_T c_TrackEstimator_likelihoodUnas(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *i_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *j_estimator_StateEstimator_Stat,
    trackingEKF *k_estimator_StateEstimator_Stat, b_struct_T *pdf,
    real_T b_time, real_T gateSize);

real_T c_TrackEstimator_sampleDistribu(
    trackingEKF *c_estimator_StateEstimator_Stat,
    trackingEKF *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, uint32_T *pdf_TrackID,
    uint32_T *pdf_Age, boolean_T *pdf_IsConfirmed, boolean_T *pdf_IsCoasted,
    struct_T pdf_Hypothesis[3], real_T pdf_LogWeights[3],
    boolean_T pdf_IsValid[3], real_T *pdf_ExistenceProbability);

void c_TrackEstimator_set_TargetSpec(
    trackingAlgorithmStackData *SD, i_fusion_tracker_internal_estim *obj,
    const c_fusion_tracker_targetspecs_Pa *val_f1,
    const c_fusion_tracker_targetspecs_Ge *val_f2,
    const c_fusion_tracker_targetspecs_He *val_f3);

void c_TrackEstimator_updateEstimato(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    i_fusion_tracker_internal_estim *estimator,
    const real_T modelData_LookTime_data[],
    const int32_T modelData_LookTime_size[2],
    const real_T modelData_LookAzimuth_data[],
    const int32_T modelData_LookAzimuth_size[2],
    const real_T modelData_LookElevation_data[],
    const int32_T modelData_LookElevation_size[2],
    const real_T modelData_DetectionTime_data[],
    const int32_T modelData_DetectionTime_size[2],
    const real_T modelData_AzimuthNoise_data[],
    const int32_T modelData_AzimuthNoise_size[2],
    const real_T modelData_ElevationNoise_data[],
    const int32_T modelData_ElevationNoise_size[2],
    const real_T modelData_RangeNoise_data[],
    const int32_T modelData_RangeNoise_size[2],
    const real_T modelData_RangeRateNoise_data[],
    const int32_T modelData_RangeRateNoise_size[2]);

/* End of code generation (TrackEstimator1.h) */
