/*
 * objectTrack.h
 *
 * Code generation for function 'objectTrack'
 *
 */

#pragma once

/* Include files */
#include "fusionAlgorithm_internal_types.h"
#include "fusionAlgorithm_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void objectTrack_objectTrack(
    const emlrtStack *sp, uint32_T varargin_1_SourceIndex,
    real_T varargin_1_UpdateTime, const real_T varargin_1_State[6],
    const real_T varargin_1_StateCovariance[36],
    real_T varargin_1_ObjectClassID,
    const real_T c_varargin_1_ObjectClassProbabi[],
    const int32_T d_varargin_1_ObjectClassProbabi[2],
    const char_T varargin_1_TrackLogic[10],
    const real_T varargin_1_TrackLogicState_data[],
    const int32_T varargin_1_TrackLogicState_size[2],
    boolean_T varargin_1_IsConfirmed, boolean_T varargin_1_IsCoasted,
    boolean_T varargin_1_IsSelfReported,
    const struct1_T varargin_1_ObjectAttributes, b_objectTrack *track);

/* End of code generation (objectTrack.h) */
