/*
 * JVProblemSolutionPair.h
 *
 * Code generation for function 'JVProblemSolutionPair'
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
void JVProblemSolutionPair_partition(
    const emlrtStack *sp, const emxArray_real_T *obj_PaddedCostMatrix,
    const emxArray_real_T *obj_RowSoln, const emxArray_real_T *obj_ColSoln,
    const emxArray_boolean_T *obj_IsEnforced, const int32_T obj_CostSize[2],
    real_T obj_BestSolutionCost, const emxArray_boolean_T *obj_IsDummySolution,
    const emxArray_real_T *obj_ColReduction,
    const emxArray_real_T *obj_RowReduction,
    c_emxArray_fusion_internal_assi *objArray);

real_T c_JVProblemSolutionPair_JVProbl(
    const emlrtStack *sp, const emxArray_real_T *costMatrix,
    const real_T costOfNonAssignment_f1_data[],
    int32_T costOfNonAssignment_f1_size,
    const emxArray_real_T *costOfNonAssignment_f2,
    emxArray_real_T *obj_PaddedCostMatrix, emxArray_real_T *obj_RowSoln,
    emxArray_real_T *obj_ColSoln, emxArray_boolean_T *obj_IsEnforced,
    int32_T obj_CostSize[2], emxArray_boolean_T *obj_IsDummySolution,
    emxArray_real_T *obj_ColReduction, emxArray_real_T *obj_RowReduction,
    boolean_T *obj_IsSolved, real_T *obj_LowerBound);

/* End of code generation (JVProblemSolutionPair.h) */
