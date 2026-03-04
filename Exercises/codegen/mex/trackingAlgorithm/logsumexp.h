/*
 * logsumexp.h
 *
 * Code generation for function 'logsumexp'
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
real_T b_logsumexp(const emlrtStack *sp, const emxArray_real_T *x);

real_T c_logsumexp(const emlrtStack *sp, const real_T x[3]);

real_T logsumexp(const emlrtStack *sp, const real_T x_data[],
                 const int32_T x_size[2]);

/* End of code generation (logsumexp.h) */
