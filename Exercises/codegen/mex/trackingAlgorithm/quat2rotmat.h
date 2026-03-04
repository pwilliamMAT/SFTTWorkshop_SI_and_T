/*
 * quat2rotmat.h
 *
 * Code generation for function 'quat2rotmat'
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
void quat2rotmat(const emlrtStack *sp, const real_T q_a_data[],
                 int32_T q_a_size, const real_T q_b_data[], int32_T q_b_size,
                 const real_T q_c_data[], int32_T q_c_size,
                 const real_T q_d_data[], int32_T q_d_size, real_T rot_data[],
                 int32_T rot_size[3]);

/* End of code generation (quat2rotmat.h) */
