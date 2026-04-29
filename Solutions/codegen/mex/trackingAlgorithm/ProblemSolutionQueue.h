/*
 * ProblemSolutionQueue.h
 *
 * Code generation for function 'ProblemSolutionQueue'
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
void ProblemSolutionQueue_sortQueue(const emlrtStack *sp,
                                    c_fusion_internal_assignment_Pr *obj);

int32_T c_ProblemSolutionQueue_ProblemS(
    const emlrtStack *sp, const c_fusion_internal_assignment_JV *problem,
    int32_T maxNumSubProblems,
    c_emxArray_fusion_internal_assi *obj_AllProblemList,
    int32_T *obj_MaxNumSubProblems);

boolean_T
c_ProblemSolutionQueue_extractB(const emlrtStack *sp,
                                c_fusion_internal_assignment_Pr *obj,
                                c_fusion_internal_assignment_JV *topProblem);

void c_ProblemSolutionQueue_formatSo(
    const emlrtStack *sp, int32_T obj_NumProblems,
    int32_T obj_MaxNumSubProblems,
    const c_emxArray_fusion_internal_assi *obj_AllProblemList,
    emxArray_cell_wrap_74 *assignments, emxArray_cell_wrap_75 *unassignedRows,
    emxArray_cell_wrap_75 *unassignedCols, emxArray_real_T *cost);

/* End of code generation (ProblemSolutionQueue.h) */
