/*
 * fusionAlgorithm.h
 *
 * Code generation for function 'fusionAlgorithm'
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
void Ecef2nedTrack(const emlrtStack *sp, uint32_T centralTrack_TrackID,
                   uint32_T centralTrack_BranchID,
                   uint32_T centralTrack_SourceIndex, uint32_T centralTrack_Age,
                   real_T centralTrack_ObjectClassID,
                   const real_T c_centralTrack_ObjectClassProba[],
                   const int32_T d_centralTrack_ObjectClassProba[2],
                   boolean_T centralTrack_IsConfirmed,
                   boolean_T centralTrack_IsCoasted,
                   boolean_T centralTrack_IsSelfReported,
                   const real_T centralTrack_pState[6],
                   const real_T centralTrack_pStateCovariance[36],
                   real_T centralTrack_pUpdateTime, b_objectTrack *radarTrack);

void Ned2ecefTrack(const emlrtStack *sp, uint32_T radarTrack_TrackID,
                   uint32_T radarTrack_BranchID,
                   uint32_T radarTrack_SourceIndex, uint32_T radarTrack_Age,
                   real_T radarTrack_ObjectClassID,
                   const real_T c_radarTrack_ObjectClassProbabi[],
                   const int32_T d_radarTrack_ObjectClassProbabi[2],
                   boolean_T radarTrack_IsConfirmed,
                   boolean_T radarTrack_IsCoasted,
                   boolean_T radarTrack_IsSelfReported,
                   const real_T radarTrack_pState[6],
                   const real_T radarTrack_pStateCovariance[36],
                   real_T radarTrack_pUpdateTime, b_objectTrack *centralTrack);

void b_local2central(const emlrtStack *sp, uint32_T localTrack_TrackID,
                     uint32_T localTrack_BranchID,
                     uint32_T localTrack_SourceIndex, uint32_T localTrack_Age,
                     real_T localTrack_ObjectClassID,
                     real_T c_localTrack_ObjectClassProbabi,
                     boolean_T localTrack_IsConfirmed,
                     boolean_T localTrack_IsCoasted,
                     boolean_T localTrack_IsSelfReported,
                     const real_T localTrack_pState[6],
                     const real_T localTrack_pStateCovariance[36],
                     real_T localTrack_pUpdateTime, objectTrack *centralTrack);

void central2local(const emlrtStack *sp, uint32_T centralTrack_TrackID,
                   uint32_T centralTrack_BranchID,
                   uint32_T centralTrack_SourceIndex, uint32_T centralTrack_Age,
                   real_T centralTrack_ObjectClassID,
                   real_T c_centralTrack_ObjectClassProba,
                   boolean_T centralTrack_IsConfirmed,
                   boolean_T centralTrack_IsCoasted,
                   boolean_T centralTrack_IsSelfReported,
                   const real_T centralTrack_pState[6],
                   const real_T centralTrack_pStateCovariance[36],
                   real_T centralTrack_pUpdateTime, objectTrack *localTrack);

emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

void fusionAlgorithm(const emlrtStack *sp, const emxArray_struct0_T *tracks,
                     real_T b_time, struct2_T fusedTracks_data[],
                     int32_T fusedTracks_size[1]);

void fusionAlgorithm_delete(void);

void fusionAlgorithm_emx_free(const emlrtStack *sp);

void fusionAlgorithm_emx_init(const emlrtStack *sp);

void fusionAlgorithm_init(void);

void fusionAlgorithm_new(void);

void local2central(const emlrtStack *sp, uint32_T localTrack_TrackID,
                   uint32_T localTrack_BranchID,
                   uint32_T localTrack_SourceIndex,
                   real_T localTrack_UpdateTime, uint32_T localTrack_Age,
                   const real_T localTrack_State[6],
                   const real_T localTrack_StateCovariance[36],
                   real_T localTrack_ObjectClassID,
                   const real_T c_localTrack_ObjectClassProbabi[],
                   const int32_T d_localTrack_ObjectClassProbabi[2],
                   boolean_T localTrack_IsConfirmed,
                   boolean_T localTrack_IsCoasted,
                   boolean_T localTrack_IsSelfReported,
                   b_objectTrack *centralTrack);

/* End of code generation (fusionAlgorithm.h) */
