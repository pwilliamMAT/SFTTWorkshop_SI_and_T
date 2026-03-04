/*
 * sort.h
 *
 * Code generation for function 'sort'
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
void b_sort(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx);

void sort(const emlrtStack *sp, emxArray_uint32_T *x);

/* End of code generation (sort.h) */
