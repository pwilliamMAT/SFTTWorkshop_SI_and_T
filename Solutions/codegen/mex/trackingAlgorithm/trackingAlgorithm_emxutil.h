/*
 * trackingAlgorithm_emxutil.h
 *
 * Code generation for function 'trackingAlgorithm_emxutil'
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
void c_emxCopyMatrix_fusion_tracker_(const c_fusion_tracker_sensorspecs_Ae *src,
                                     c_fusion_tracker_sensorspecs_Ae *dst);

void c_emxCopyStruct_fusion_internal(const emlrtStack *sp,
                                     c_fusion_internal_assignment_JV *dst,
                                     const c_fusion_internal_assignment_JV *src,
                                     const emlrtRTEInfo *srcLocation);

void c_emxCopyStruct_fusion_tracker_(const emlrtStack *sp,
                                     g_fusion_tracker_internal_compo *dst,
                                     const g_fusion_tracker_internal_compo *src,
                                     const emlrtRTEInfo *srcLocation);

void c_emxEnsureCapacity_fusion_inte(const emlrtStack *sp,
                                     c_emxArray_fusion_internal_assi *emxArray,
                                     int32_T oldNumel,
                                     const emlrtRTEInfo *srcLocation);

void c_emxExpand_fusion_internal_ass(const emlrtStack *sp,
                                     c_emxArray_fusion_internal_assi *emxArray,
                                     int32_T fromIndex, int32_T toIndex,
                                     const emlrtRTEInfo *srcLocation);

void c_emxFreeStruct_fusion_internal(const emlrtStack *sp,
                                     c_fusion_internal_assignment_JV *pStruct);

void c_emxFreeStruct_fusion_tracker_(const emlrtStack *sp,
                                     g_fusion_tracker_internal_compo *pStruct);

void c_emxFree_fusion_internal_assig(
    const emlrtStack *sp, c_emxArray_fusion_internal_assi **pEmxArray);

void c_emxInitStruct_fusion_internal(const emlrtStack *sp,
                                     c_fusion_internal_assignment_JV *pStruct,
                                     const emlrtRTEInfo *srcLocation,
                                     boolean_T doPush);

void c_emxInitStruct_fusion_tracker_(const emlrtStack *sp,
                                     g_fusion_tracker_internal_compo *pStruct,
                                     const emlrtRTEInfo *srcLocation,
                                     boolean_T doPush);

void c_emxInit_fusion_internal_assig(
    const emlrtStack *sp, c_emxArray_fusion_internal_assi **pEmxArray,
    const emlrtRTEInfo *srcLocation);

void c_emxTrim_fusion_internal_assig(const emlrtStack *sp,
                                     c_emxArray_fusion_internal_assi *emxArray,
                                     int32_T fromIndex, int32_T toIndex);

void d_emxFreeStruct_fusion_internal(const emlrtStack *sp,
                                     c_fusion_internal_assignment_Pr *pStruct);

void d_emxFreeStruct_fusion_tracker_(const emlrtStack *sp,
                                     fusion_tracker_JIPDATracker *pStruct);

void d_emxInitStruct_fusion_internal(const emlrtStack *sp,
                                     c_fusion_internal_assignment_Pr *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void d_emxInitStruct_fusion_tracker_(const emlrtStack *sp,
                                     fusion_tracker_JIPDATracker *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void e_emxFreeStruct_fusion_tracker_(const emlrtStack *sp,
                                     c_fusion_tracker_internal_compo *pStruct);

void e_emxInitStruct_fusion_tracker_(const emlrtStack *sp,
                                     c_fusion_tracker_internal_compo *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void emxCopyMatrix_int32_T(int32_T dst[2], const int32_T src[2]);

void emxCopy_boolean_T(const emlrtStack *sp, emxArray_boolean_T **dst,
                       emxArray_boolean_T *const *src,
                       const emlrtRTEInfo *srcLocation);

void emxCopy_real_T(const emlrtStack *sp, emxArray_real_T **dst,
                    emxArray_real_T *const *src,
                    const emlrtRTEInfo *srcLocation);

void emxCopy_struct_T(const emlrtStack *sp, emxArray_struct_T **dst,
                      emxArray_struct_T *const *src,
                      const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_boolean_T(const emlrtStack *sp,
                                 emxArray_boolean_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_cell_wrap_81(const emlrtStack *sp,
                                    emxArray_cell_wrap_81 *emxArray,
                                    int32_T oldNumel,
                                    const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_cell_wrap_82(const emlrtStack *sp,
                                    emxArray_cell_wrap_82 *emxArray,
                                    int32_T oldNumel,
                                    const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_int8_T(const emlrtStack *sp, emxArray_int8_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct1_T(const emlrtStack *sp,
                                 emxArray_struct1_T *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T(const emlrtStack *sp,
                                emxArray_struct_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T1(const emlrtStack *sp,
                                 b_emxArray_struct_T *emxArray,
                                 int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_struct_T2(const emlrtStack *sp,
                                 c_emxArray_struct_T *emxArray,
                                 int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_uint32_T(const emlrtStack *sp,
                                emxArray_uint32_T *emxArray, int32_T oldNumel,
                                const emlrtRTEInfo *srcLocation);

void emxExpand_cell_wrap_81(emxArray_cell_wrap_81 *emxArray, int32_T fromIndex,
                            int32_T toIndex);

void emxExpand_cell_wrap_82(const emlrtStack *sp,
                            emxArray_cell_wrap_82 *emxArray, int32_T fromIndex,
                            int32_T toIndex, const emlrtRTEInfo *srcLocation);

void emxExpand_struct_T(b_emxArray_struct_T *emxArray, int32_T fromIndex,
                        int32_T toIndex);

void emxFreeStruct_cell_wrap_82(const emlrtStack *sp, cell_wrap_82 *pStruct);

void emxFree_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray);

void emxFree_cell_wrap_81(const emlrtStack *sp,
                          emxArray_cell_wrap_81 **pEmxArray);

void emxFree_cell_wrap_82(const emlrtStack *sp,
                          emxArray_cell_wrap_82 **pEmxArray);

void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray);

void emxFree_int8_T(const emlrtStack *sp, emxArray_int8_T **pEmxArray);

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

void emxFree_struct1_T(const emlrtStack *sp, emxArray_struct1_T **pEmxArray);

void emxFree_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray);

void emxFree_struct_T1(const emlrtStack *sp, b_emxArray_struct_T **pEmxArray);

void emxFree_struct_T2(const emlrtStack *sp, c_emxArray_struct_T **pEmxArray);

void emxFree_uint32_T(const emlrtStack *sp, emxArray_uint32_T **pEmxArray);

void emxInitStruct_cell_wrap_81(cell_wrap_81 *pStruct);

void emxInitStruct_cell_wrap_82(const emlrtStack *sp, cell_wrap_82 *pStruct,
                                const emlrtRTEInfo *srcLocation);

void emxInitStruct_struct_T(c_struct_T *pStruct);

void emxInit_boolean_T(const emlrtStack *sp, emxArray_boolean_T **pEmxArray,
                       int32_T numDimensions, const emlrtRTEInfo *srcLocation,
                       boolean_T doPush);

void emxInit_cell_wrap_81(const emlrtStack *sp,
                          emxArray_cell_wrap_81 **pEmxArray,
                          const emlrtRTEInfo *srcLocation);

void emxInit_cell_wrap_82(const emlrtStack *sp,
                          emxArray_cell_wrap_82 **pEmxArray,
                          const emlrtRTEInfo *srcLocation);

void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
                     int32_T numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_int8_T(const emlrtStack *sp, emxArray_int8_T **pEmxArray,
                    const emlrtRTEInfo *srcLocation);

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T numDimensions, const emlrtRTEInfo *srcLocation,
                    boolean_T doPush);

void emxInit_struct1_T(const emlrtStack *sp, emxArray_struct1_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_struct_T(const emlrtStack *sp, emxArray_struct_T **pEmxArray,
                      const emlrtRTEInfo *srcLocation, boolean_T doPush);

void emxInit_struct_T1(const emlrtStack *sp, b_emxArray_struct_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_struct_T2(const emlrtStack *sp, c_emxArray_struct_T **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_uint32_T(const emlrtStack *sp, emxArray_uint32_T **pEmxArray,
                      int32_T numDimensions, const emlrtRTEInfo *srcLocation,
                      boolean_T doPush);

void emxTrim_cell_wrap_82(const emlrtStack *sp, emxArray_cell_wrap_82 *emxArray,
                          int32_T fromIndex, int32_T toIndex);

/* End of code generation (trackingAlgorithm_emxutil.h) */
