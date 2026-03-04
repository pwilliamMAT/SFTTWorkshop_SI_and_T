/*
 * xhseqr.h
 *
 * Code generation for function 'xhseqr'
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
int32_T b_xhseqr(const emlrtStack *sp, real_T h[36]);

int32_T c_xhseqr(const emlrtStack *sp, real_T h[16]);

int32_T xhseqr(const emlrtStack *sp, real_T h[9]);

/* End of code generation (xhseqr.h) */
