/*
 * xzgebal.h
 *
 * Code generation for function 'xzgebal'
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
int32_T xzgebal(const emlrtStack *sp, real_T A[9], int32_T *ihi,
                real_T scale[3]);

/* End of code generation (xzgebal.h) */
