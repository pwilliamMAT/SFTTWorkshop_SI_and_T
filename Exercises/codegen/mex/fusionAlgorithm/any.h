/*
 * any.h
 *
 * Code generation for function 'any'
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
boolean_T any(const boolean_T x_data[], const int32_T x_size[2]);

boolean_T b_any(const emlrtStack *sp, const emxArray_boolean_T *x);

/* End of code generation (any.h) */
