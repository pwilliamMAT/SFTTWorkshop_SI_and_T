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
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    h_fusion_tracker_internal_estim *estimator, b_struct_T *pdf,
    const real_T measurements_data[], const int32_T measurements_size[2],
    const real_T dTs_data[], const int32_T dTs_size[2],
    const b_emxArray_struct_T *modelData, const real_T assignmentProbs_data[],
    int32_T assignmentProbs_size, real_T gateSize);

int32_T IPDAEstimator_toObjectTrack(const struct_T pdf_Hypothesis[3],
                                    const real_T pdf_LogWeights[3],
                                    real_T pdf_ExistenceProbability,
                                    real_T objectTrackPdf_State[6],
                                    real_T objectTrackPdf_StateCovariance[36],
                                    real_T c_objectTrackPdf_ObjectClassPro[3],
                                    char_T objectTrackPdf_TrackLogic[10],
                                    real_T *objectTrackPdf_TrackLogicState);

/* End of code generation (IPDAEstimator.h) */
