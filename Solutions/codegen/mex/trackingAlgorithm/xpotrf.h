/*
 * xpotrf.h
 *
 * Code generation for function 'xpotrf'
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
int32_T b_xpotrf(const emlrtStack *sp, real_T A[36]);

int32_T c_xpotrf(const emlrtStack *sp, real_T A[16]);

int32_T xpotrf(const emlrtStack *sp, real_T A[9]);

/* End of code generation (xpotrf.h) */
