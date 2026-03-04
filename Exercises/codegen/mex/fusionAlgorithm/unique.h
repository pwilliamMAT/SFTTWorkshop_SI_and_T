/*
 * unique.h
 *
 * Code generation for function 'unique'
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
void b_unique_vector(const emlrtStack *sp, const emxArray_uint32_T *a,
                     emxArray_uint32_T *b);

void c_unique_vector(const emlrtStack *sp, const emxArray_uint32_T *a,
                     emxArray_uint32_T *b);

void unique_vector(const emlrtStack *sp, const emxArray_real_T *a,
                   emxArray_real_T *b);

/* End of code generation (unique.h) */
