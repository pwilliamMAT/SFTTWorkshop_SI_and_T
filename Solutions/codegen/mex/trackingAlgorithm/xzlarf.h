/*
 * xzlarf.h
 *
 * Code generation for function 'xzlarf'
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
int32_T b_ilazlc(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[40],
                 int32_T ia0);

void b_xzlarf(const emlrtStack *sp, int32_T m, int32_T n, int32_T iv0,
              real_T tau, real_T C[36], int32_T ic0, real_T work[6]);

int32_T c_ilazlc(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[60],
                 int32_T ia0);

void c_xzlarf(const emlrtStack *sp, int32_T m, int32_T n, int32_T iv0,
              real_T tau, real_T C[16], int32_T ic0, real_T work[4]);

int32_T ilazlc(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[54],
               int32_T ia0);

void xzlarf(const emlrtStack *sp, int32_T m, int32_T n, int32_T iv0, real_T tau,
            real_T C[9], int32_T ic0, real_T work[3]);

/* End of code generation (xzlarf.h) */
