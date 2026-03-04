/*
 * mrdivide_helper.h
 *
 * Code generation for function 'mrdivide_helper'
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
void b_mrdiv(const emlrtStack *sp, real_T A[36], const real_T B[36]);

void mrdiv(const emlrtStack *sp, real_T A[4], const real_T B[16]);

/* End of code generation (mrdivide_helper.h) */
