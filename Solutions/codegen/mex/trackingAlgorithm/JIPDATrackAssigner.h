/*
 * JIPDATrackAssigner.h
 *
 * Code generation for function 'JIPDATrackAssigner'
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
void JIPDATrackAssigner_assign(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T assigner_AssignmentThreshold, real_T c_assigner_InitializationThresh,
    real_T assigner_MaxNumEvents,
    const c_fusion_tracker_sensorspecs_Ae *c_assigner_Estimator_StateEstim,
    c_fusion_tracker_internal_estim *d_assigner_Estimator_StateEstim,
    d_fusion_tracker_internal_estim *e_assigner_Estimator_StateEstim,
    e_fusion_tracker_internal_estim *f_assigner_Estimator_StateEstim,
    const c_fusion_tracker_sensorspecs_Ae *g_assigner_Estimator_StateEstim,
    const emxArray_struct_T *trackList, const struct0_T *sensorData,
    emxArray_real_T *assignment, emxArray_boolean_T *unassignedTracks,
    struct0_T *unassignedSensorData);

/* End of code generation (JIPDATrackAssigner.h) */
