/*
 * ConstantVelocityModel.h
 *
 * Code generation for function 'ConstantVelocityModel'
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
void c_ConstantVelocityModel_predict(const emlrtStack *sp,
                                     const real_T state[6], real_T varargin_2,
                                     real_T F[36], real_T B[18]);

/* End of code generation (ConstantVelocityModel.h) */
