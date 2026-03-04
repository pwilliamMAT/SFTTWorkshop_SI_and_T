/*
 * all.h
 *
 * Code generation for function 'all'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void all(const emlrtStack *sp, const boolean_T x[36], boolean_T y[6]);

boolean_T b_all(const boolean_T x_data[], const int32_T x_size[2]);

/* End of code generation (all.h) */
