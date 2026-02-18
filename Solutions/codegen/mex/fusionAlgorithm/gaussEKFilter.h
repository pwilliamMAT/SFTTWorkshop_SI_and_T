/*
 * gaussEKFilter.h
 *
 * Code generation for function 'gaussEKFilter'
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
void b_gaussEKFilter_predict(const emlrtStack *sp, const real_T x[6],
                             real_T P[36], const real_T Q[9], real_T varargin_1,
                             real_T xk[6]);

void gaussEKFilter_predict(const emlrtStack *sp, const emxArray_real_T *x,
                           emxArray_real_T *P, const real_T Q[9],
                           real_T varargin_1, emxArray_real_T *xk);

/* End of code generation (gaussEKFilter.h) */
