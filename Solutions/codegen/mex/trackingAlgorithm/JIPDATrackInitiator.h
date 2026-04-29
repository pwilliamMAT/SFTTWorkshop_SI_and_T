/*
 * JIPDATrackInitiator.h
 *
 * Code generation for function 'JIPDATrackInitiator'
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
void initializeTrack(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_StateEstimator_Exis,
    struct_T *track, const real_T z[4], real_T b_time);

/* End of code generation (JIPDATrackInitiator.h) */
