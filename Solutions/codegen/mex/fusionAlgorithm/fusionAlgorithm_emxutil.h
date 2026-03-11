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

void emxCopyMatrix_char_T(char_T dst[7], const char_T src[7]);

void emxCopyMatrix_real_T(real_T dst[6], const real_T src[6]);

void emxCopyMatrix_real_T1(real_T dst[36], const real_T src[36]);

void emxCopyStruct_objectTrack(const emlrtStack *sp, objectTrack *dst,
                               const objectTrack *src,
                               const emlrtRTEInfo *srcLocation);

void emxCopyStruct_struct2_T(const emlrtStack *sp, struct2_T *dst,
                             const struct2_T *src,
                             const emlrtRTEInfo *srcLocation);

void emxCopy_boolean_T(const emlrtStack *sp, emxArray_boolean_T **dst,
                       emxArray_boolean_T *const *src,
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

void emxEnsureCapacity_struct2_T(const emlrtStack *sp, struct2_T b_data[100],
                                 int32_T b_size, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct2_T1(const emlrtStack *sp,
                                  emxArray_struct2_T *emxArray,
                                  int32_T oldNumel,
                                  const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T(const emlrtStack *sp,
                                emxArray_struct_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_uint32_T(const emlrtStack *sp,
                                emxArray_uint32_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_uint8_T(const emlrtStack *sp, emxArray_uint8_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation);

void emxExpand_objectTrack(const emlrtStack *sp, emxArray_objectTrack *emxArray,
                           int32_T fromIndex, int32_T toIndex,
                           const emlrtRTEInfo *srcLocation);

void emxExpand_struct0_T(emxArray_struct0_T *emxArray, int32_T fromIndex,
                         int32_T toIndex);

void emxExpand_struct2_T(const emlrtStack *sp, emxArray_struct2_T *emxArray,
                         int32_T fromIndex, int32_T toIndex,
                         const emlrtRTEInfo *srcLocation);

void emxExpand_struct2_T_100(const emlrtStack *sp, struct2_T b_data[100],
                             int32_T fromIndex, int32_T toIndex,
                             const emlrtRTEInfo *srcLocation);

void emxFreeMatrix_objectTrack(const emlrtStack *sp, objectTrack pMatrix[100]);

void emxFreeMatrix_objectTrack1(const emlrtStack *sp, objectTrack *pMatrix);

void emxFreeStruct_objectTrack(const emlrtStack *sp, objectTrack *pStruct);

void emxFreeStruct_struct2_T(const emlrtStack *sp, struct2_T *pStruct);

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

void emxFree_struct2_T_100(const emlrtStack *sp,
                           emxArray_struct2_T_100 *pEmxArray);

void emxFree_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray);

void emxFree_uint32_T(const emlrtStack *sp, emxArray_uint32_T **pEmxArray);

void emxFree_uint8_T(const emlrtStack *sp, emxArray_uint8_T **pEmxArray);

void emxInitMatrix_objectTrack(const emlrtStack *sp, objectTrack pMatrix[100],
                               const emlrtRTEInfo *srcLocation,
                               boolean_T doPush);

void emxInitMatrix_objectTrack1(const emlrtStack *sp, objectTrack *pMatrix,
                                const emlrtRTEInfo *srcLocation);

void emxInitStruct_objectTrack(const emlrtStack *sp, objectTrack *pStruct,
                               const emlrtRTEInfo *srcLocation,
                               boolean_T doPush);

void emxInitStruct_struct0_T(struct0_T *pStruct);

void emxInitStruct_struct2_T(const emlrtStack *sp, struct2_T *pStruct,
                             const emlrtRTEInfo *srcLocation);

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

void emxInit_struct2_T_100(emxArray_struct2_T_100 *pEmxArray);

void emxInit_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray,
                      const emlrtRTEInfo *srcLocation);

void emxInit_uint32_T(const emlrtStack *sp, emxArray_uint32_T **pEmxArray,
                      int32_T b_numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_uint8_T(const emlrtStack *sp, emxArray_uint8_T **pEmxArray,
                     int32_T b_numDimensions, const emlrtRTEInfo *srcLocation);

void emxTrim_objectTrack(const emlrtStack *sp, emxArray_objectTrack *emxArray,
                         int32_T fromIndex, int32_T toIndex);

void emxTrim_struct2_T(const emlrtStack *sp, emxArray_struct2_T *emxArray,
                       int32_T fromIndex, int32_T toIndex);

void emxTrim_struct2_T_100(const emlrtStack *sp, struct2_T b_data[100],
                           int32_T fromIndex, int32_T toIndex);

/* End of code generation (fusionAlgorithm_emxutil.h) */
