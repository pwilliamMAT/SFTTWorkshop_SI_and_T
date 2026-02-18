/*
 * xscal.h
 *
 * Code generation for function 'xscal'
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
void b_xscal(const emlrtStack *sp, int32_T n, real_T a, real_T x[9],
             int32_T ix0);

void c_xscal(const emlrtStack *sp, real_T a, real_T x[9], int32_T ix0);

void d_xscal(const emlrtStack *sp, real_T x[9], int32_T ix0);

void e_xscal(const emlrtStack *sp, int32_T n, real_T a, real_T x[6],
             int32_T ix0);

void f_xscal(const emlrtStack *sp, real_T a, real_T x[36], int32_T ix0);

void g_xscal(const emlrtStack *sp, real_T x[36], int32_T ix0);

void h_xscal(const emlrtStack *sp, int32_T n, real_T a, real_T x[16],
             int32_T ix0);

void i_xscal(const emlrtStack *sp, int32_T n, real_T a, real_T x[4],
             int32_T ix0);

void j_xscal(const emlrtStack *sp, real_T a, real_T x[16], int32_T ix0);

void k_xscal(const emlrtStack *sp, real_T x[16], int32_T ix0);

void l_xscal(real_T a, real_T x[3]);

void xscal(const emlrtStack *sp, int32_T n, real_T a, real_T x[36],
           int32_T ix0);

/* End of code generation (xscal.h) */
