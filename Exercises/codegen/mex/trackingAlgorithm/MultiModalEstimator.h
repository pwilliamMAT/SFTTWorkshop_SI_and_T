/*
 * MultiModalEstimator.h
 *
 * Code generation for function 'MultiModalEstimator'
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
void MultiModalEstimator_merge(const emlrtStack *sp,
                               real_T estimator_DeletionThreshold,
                               const emxArray_struct_T *pdfs,
                               const emxArray_real_T *weights, b_struct_T *pdf);

void MultiModalEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_Estimators_f1_Targe,
    trackingEKF *c_estimator_Estimators_f1_Track,
    const c_fusion_tracker_targetspecs_Ge *c_estimator_Estimators_f2_Targe,
    trackingEKF *c_estimator_Estimators_f2_Track,
    const c_fusion_tracker_targetspecs_He *c_estimator_Estimators_f3_Targe,
    trackingEKF *c_estimator_Estimators_f3_Track, b_struct_T *pdf, real_T dT);

real_T c_MultiModalEstimator_detection(
    const emlrtStack *sp,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_Estimators_f1_Senso,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_Estimators_f2_Senso,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_Estimators_f3_Senso,
    const struct_T pdf_Hypothesis[3], const real_T pdf_LogWeights[3],
    const boolean_T pdf_IsValid[3]);

real_T c_MultiModalEstimator_gateProba(const emlrtStack *sp, real_T gateSize);

real_T c_MultiModalEstimator_survivalP(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_Estimators_f1_Targe,
    const c_fusion_tracker_targetspecs_Ge *c_estimator_Estimators_f2_Targe,
    const c_fusion_tracker_targetspecs_He *c_estimator_Estimators_f3_Targe,
    const real_T pdf_LogWeights[3], const boolean_T pdf_IsValid[3], real_T dT);

/* End of code generation (MultiModalEstimator.h) */
