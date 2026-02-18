/*
 * sortIdx.h
 *
 * Code generation for function 'sortIdx'
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
void b_merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
                   emxArray_real_T *x, int32_T offset, int32_T n,
                   int32_T preSortLevel, emxArray_int32_T *iwork,
                   emxArray_real_T *xwork);

void merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
                 emxArray_uint32_T *x, int32_T offset, int32_T n,
                 int32_T preSortLevel, emxArray_int32_T *iwork,
                 emxArray_uint32_T *xwork);

/* End of code generation (sortIdx.h) */
