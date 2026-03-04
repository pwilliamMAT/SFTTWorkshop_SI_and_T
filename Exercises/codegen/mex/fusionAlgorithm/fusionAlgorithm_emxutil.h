/*
 * fusionAlgorithm_emxutil.h
 *
 * Code generation for function 'fusionAlgorithm_emxutil'
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
void c_emxFreeStruct_matlab_internal(const emlrtStack *sp,
                                     c_matlab_internal_coder_minPrio *pStruct);

void c_emxInitStruct_matlab_internal(const emlrtStack *sp,
                                     c_matlab_internal_coder_minPrio *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_boolean_T(const emlrtStack *sp,
                                 emxArray_boolean_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_objectTrack(const emlrtStack *sp,
                                   emxArray_objectTrack *emxArray,
                                   int32_T oldNumel,
                                   const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_ptrdiff_t(const emlrtStack *sp,
                                 emxArray_ptrdiff_t *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct0_T(const emlrtStack *sp,
                                 emxArray_struct0_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct1_T(const emlrtStack *sp,
                                 emxArray_struct1_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct2_T(const emlrtStack *sp,
                                 emxArray_struct2_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_uint32_T(const emlrtStack *sp,
                                emxArray_uint32_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_uint8_T(const emlrtStack *sp, emxArray_uint8_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation);

void emxExpand_struct0_T(emxArray_struct0_T *emxArray, int32_T fromIndex,
                         int32_T toIndex);

void emxFreeStruct_trackFuser(const emlrtStack *sp, trackFuser *pStruct);

void emxFree_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray);

void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray);

void emxFree_objectTrack(const emlrtStack *sp,
                         emxArray_objectTrack **pEmxArray);

void emxFree_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray);

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

void emxFree_struct0_T(const emlrtStack *sp, emxArray_struct0_T **pEmxArray);

void emxFree_struct1_T(const emlrtStack *sp, emxArray_struct1_T **pEmxArray);

void emxFree_struct2_T(const emlrtStack *sp, emxArray_struct2_T **pEmxArray);

void emxFree_uint32_T(const emlrtStack *sp, emxArray_uint32_T **pEmxArray);

void emxFree_uint8_T(const emlrtStack *sp, emxArray_uint8_T **pEmxArray);

void emxInitStruct_struct0_T(struct0_T *pStruct);

void emxInitStruct_trackFuser(const emlrtStack *sp, trackFuser *pStruct,
                              const emlrtRTEInfo *srcLocation);

void emxInit_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray,
                       int32_T b_numDimensions, const emlrtRTEInfo *srcLocation,
                       boolean_T doPush);

void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
                     int32_T b_numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_objectTrack(const emlrtStack *sp, emxArray_objectTrack **pEmxArray,
                         const emlrtRTEInfo *srcLocation);

void emxInit_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T b_numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_struct0_T(const emlrtStack *sp, emxArray_struct0_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_struct1_T(const emlrtStack *sp, emxArray_struct1_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_struct2_T(const emlrtStack *sp, emxArray_struct2_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_uint32_T(const emlrtStack *sp, emxArray_uint32_T **pEmxArray,
                      int32_T b_numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_uint8_T(const emlrtStack *sp, emxArray_uint8_T **pEmxArray,
                     int32_T b_numDimensions, const emlrtRTEInfo *srcLocation);

/* End of code generation (fusionAlgorithm_emxutil.h) */
