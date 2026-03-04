/*
 * combineVectorElements.h
 *
 * Code generation for function 'combineVectorElements'
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
int32_T b_combineVectorElements(const emlrtStack *sp,
                                const emxArray_boolean_T *x);

int32_T combineVectorElements(const boolean_T x_data[],
                              const int32_T x_size[2]);

/* End of code generation (combineVectorElements.h) */
