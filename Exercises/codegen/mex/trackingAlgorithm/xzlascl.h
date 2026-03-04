/*
 * xzlascl.h
 *
 * Code generation for function 'xzlascl'
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
void b_xzlascl(const emlrtStack *sp, real_T cfrom, real_T cto, int32_T m,
               real_T A[3]);

void c_xzlascl(real_T cfrom, real_T cto, real_T A[3]);

void d_xzlascl(real_T cfrom, real_T cto, real_T A[6]);

void e_xzlascl(real_T cfrom, real_T cto, real_T A[4]);

void f_xzlascl(real_T cfrom, real_T cto, real_T A[9]);

void g_xzlascl(real_T cfrom, real_T cto, real_T A[36]);

void h_xzlascl(real_T cfrom, real_T cto, real_T A[16]);

void xzlascl(const emlrtStack *sp, real_T cfrom, real_T cto, int32_T m,
             real_T A[3], int32_T iA0);

/* End of code generation (xzlascl.h) */
