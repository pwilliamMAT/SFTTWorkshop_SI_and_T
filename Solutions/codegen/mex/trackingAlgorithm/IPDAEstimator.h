/*
 * IPDAEstimator.h
 *
 * Code generation for function 'IPDAEstimator'
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
void IPDAEstimator_correctJPDA(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Targ,
    c_fusion_tracker_sensorspecs_Ae *c_estimator_StateEstimator_Sens,
    trackingEKF *c_estimator_StateEstimator_Trac,
    c_fusion_tracker_sensorspecs_Ae *c_estimator_ExistenceEstimator_,
    struct_T *pdf, const real_T measurements_data[],
    const int32_T measurements_size[2], const real_T dTs_data[],
    const int32_T dTs_size[2], const b_emxArray_struct_T *modelData,
    const real_T assignmentProbs_data[], int32_T assignmentProbs_size,
    real_T gateSize);

/* End of code generation (IPDAEstimator.h) */
