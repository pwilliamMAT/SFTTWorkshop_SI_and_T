/*
 * xswap.h
 *
 * Code generation for function 'xswap'
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
void b_xswap(real_T x[9], int32_T ix0, int32_T iy0);

void c_xswap(real_T x[36], int32_T ix0, int32_T iy0);

void d_xswap(real_T x[16], int32_T ix0, int32_T iy0);

void xswap(const emlrtStack *sp, int32_T n, real_T x[9], int32_T ix0,
           int32_T iy0);

/* End of code generation (xswap.h) */
