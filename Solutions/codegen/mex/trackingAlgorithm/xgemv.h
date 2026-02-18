/*
 * xgemv.h
 *
 * Code generation for function 'xgemv'
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
void b_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[54],
             int32_T ia0, const real_T x[54], int32_T ix0, real_T y[6]);

void c_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[40],
             int32_T ia0, const real_T x[40], int32_T ix0, real_T y[4]);

void d_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[60],
             int32_T ia0, const real_T x[60], int32_T ix0, real_T y[6]);

void xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[9],
           int32_T ia0, const real_T x[9], int32_T ix0, real_T y[3]);

/* End of code generation (xgemv.h) */
