/*
 * eml_setop.h
 *
 * Code generation for function 'eml_setop'
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
void do_vectors(const emlrtStack *sp, const emxArray_uint32_T *a,
                const emxArray_uint32_T *b, emxArray_uint32_T *c,
                emxArray_int32_T *ia, emxArray_int32_T *ib);

/* End of code generation (eml_setop.h) */
