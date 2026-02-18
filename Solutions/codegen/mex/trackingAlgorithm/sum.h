/*
 * sum.h
 *
 * Code generation for function 'sum'
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
int32_T b_sum(const emxArray_real_T *x, real_T y_data[]);

real_T c_sum(const emlrtStack *sp, const emxArray_real_T *x);

void sum(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y);

/* End of code generation (sum.h) */
