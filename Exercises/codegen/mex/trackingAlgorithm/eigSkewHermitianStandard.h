/*
 * eigSkewHermitianStandard.h
 *
 * Code generation for function 'eigSkewHermitianStandard'
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
void b_eigSkewHermitianStandard(const emlrtStack *sp, const real_T A[16],
                                creal_T V[4]);

void eigSkewHermitianStandard(const emlrtStack *sp, const real_T A[36],
                              creal_T V[6]);

/* End of code generation (eigSkewHermitianStandard.h) */
