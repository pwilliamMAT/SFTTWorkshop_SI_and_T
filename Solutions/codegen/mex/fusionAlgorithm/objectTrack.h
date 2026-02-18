/*
 * objectTrack.h
 *
 * Code generation for function 'objectTrack'
 *
 */

#pragma once

/* Include files */
#include "fusionAlgorithm_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
uint32_T b_objectTrack_objectTrack(
    uint32_T *track_BranchID, uint32_T *track_SourceIndex, uint32_T *track_Age,
    real_T *track_ObjectClassID, real_T c_track_ObjectClassProbabilitie[],
    int32_T d_track_ObjectClassProbabilitie[2], boolean_T *track_IsConfirmed,
    boolean_T *track_IsCoasted, boolean_T *track_IsSelfReported,
    real_T track_pState[6], real_T track_pStateCovariance[36],
    real_T *track_pUpdateTime);

uint32_T b_objectTrack_set_TrackID(
    uint32_T b_value, uint32_T *obj_BranchID, uint32_T *obj_SourceIndex,
    uint32_T *obj_Age, real_T *obj_ObjectClassID,
    real_T c_obj_ObjectClassProbabilities_[],
    int32_T d_obj_ObjectClassProbabilities_[2], boolean_T *obj_IsConfirmed,
    boolean_T *obj_IsCoasted, boolean_T *obj_IsSelfReported,
    real_T obj_pState[6], real_T obj_pStateCovariance[36],
    real_T *obj_pUpdateTime);

void c_objectTrack_set_ObjectClassPr(const emlrtStack *sp, b_objectTrack *obj,
                                     const real_T value_data[],
                                     const int32_T value_size[2]);

void d_objectTrack_set_ObjectClassPr(const emlrtStack *sp, objectTrack *obj,
                                     real_T b_value);

uint32_T objectTrack_objectTrack(
    uint32_T *track_BranchID, uint32_T *track_SourceIndex, uint32_T *track_Age,
    real_T *track_ObjectClassID, real_T *track_ObjectClassProbabilities,
    boolean_T *track_IsConfirmed, boolean_T *track_IsCoasted,
    boolean_T *track_IsSelfReported, real_T track_pState[6],
    real_T track_pStateCovariance[36], real_T *track_pUpdateTime);

uint32_T objectTrack_set_TrackID(
    const emlrtStack *sp, real_T b_value, uint32_T *obj_BranchID,
    uint32_T *obj_SourceIndex, uint32_T *obj_Age, real_T *obj_ObjectClassID,
    real_T c_obj_ObjectClassProbabilities_[],
    int32_T d_obj_ObjectClassProbabilities_[2], boolean_T *obj_IsConfirmed,
    boolean_T *obj_IsCoasted, boolean_T *obj_IsSelfReported,
    real_T obj_pState[6], real_T obj_pStateCovariance[36],
    real_T *obj_pUpdateTime);

/* End of code generation (objectTrack.h) */
