/*
 * mldivide.h
 *
 * Code generation for function 'mldivide'
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
void b_mldivide(const emlrtStack *sp, const real_T A[36], real_T Y[36]);

void mldivide(const emlrtStack *sp, const emxArray_real_T *A,
              const emxArray_real_T *B, emxArray_real_T *Y);

/* End of code generation (mldivide.h) */
