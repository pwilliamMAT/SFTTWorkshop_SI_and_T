/*
 * AbstractProblemSolutionPair.h
 *
 * Code generation for function 'AbstractProblemSolutionPair'
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
void c_AbstractProblemSolutionPair_g(const emlrtStack *sp,
                                     const emxArray_real_T *obj_RowSoln,
                                     const int32_T obj_CostSize[2],
                                     emxArray_real_T *val);

/* End of code generation (AbstractProblemSolutionPair.h) */
