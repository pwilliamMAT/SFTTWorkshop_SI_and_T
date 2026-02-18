/*
 * combineVectorElements.h
 *
 * Code generation for function 'combineVectorElements'
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
void b_combineVectorElements(const emlrtStack *sp, const emxArray_boolean_T *x,
                             emxArray_int32_T *y);

int32_T c_combineVectorElements(const emlrtStack *sp,
                                const emxArray_boolean_T *x);

int32_T combineVectorElements(const emlrtStack *sp,
                              const emxArray_boolean_T *x);

/* End of code generation (combineVectorElements.h) */
