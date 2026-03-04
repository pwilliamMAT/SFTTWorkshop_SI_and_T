/*
 * trisolve.h
 *
 * Code generation for function 'trisolve'
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
void b_trisolve(const real_T A[16], real_T B[24]);

void trisolve(const real_T A[16], real_T B[24]);

/* End of code generation (trisolve.h) */
