/*
 * sortIdx.h
 *
 * Code generation for function 'sortIdx'
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
void b_sortIdx(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx);

void sortIdx(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx);

/* End of code generation (sortIdx.h) */
