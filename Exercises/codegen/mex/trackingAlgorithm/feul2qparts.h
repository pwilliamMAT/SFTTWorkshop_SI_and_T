/*
 * feul2qparts.h
 *
 * Code generation for function 'feul2qparts'
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
int32_T b_minus(real_T in1_data[], const real_T in2_data[],
                const int32_T *in2_size, const real_T in3_data[],
                const int32_T *in3_size);

int32_T b_plus(real_T in1_data[], const real_T in2_data[],
               const int32_T *in2_size, const real_T in3_data[],
               const int32_T *in3_size);

int32_T b_times(real_T in1_data[], const real_T in2_data[],
                const int32_T *in2_size, const real_T in3_data[],
                const int32_T *in3_size);

void times(real_T in1_data[], int32_T *in1_size, const real_T in2_data[],
           const int32_T *in2_size);

/* End of code generation (feul2qparts.h) */
