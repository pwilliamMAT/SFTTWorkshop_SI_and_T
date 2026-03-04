/*
 * svd.h
 *
 * Code generation for function 'svd'
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
void b_svd(const emlrtStack *sp, const real_T A[36], real_T U[36], real_T s[6],
           real_T V[36]);

void c_svd(const emlrtStack *sp, const real_T A[16], real_T U[16], real_T s[4],
           real_T V[16]);

void svd(const emlrtStack *sp, const real_T A[9], real_T U[9], real_T s[3],
         real_T V[9]);

/* End of code generation (svd.h) */
