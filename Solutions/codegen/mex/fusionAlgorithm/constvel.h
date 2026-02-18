/*
 * constvel.h
 *
 * Code generation for function 'constvel'
 *
 */

#pragma once

/* Include files */
#include "fusionAlgorithm_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void binary_expand_op(const emlrtStack *sp, emxArray_real_T *in1,
                      const emxArray_real_T *in2, int32_T in3);

void binary_expand_op_1(const emlrtStack *sp, emxArray_real_T *in1,
                        const emxArray_real_T *in2, int32_T in3);

void constvel(const emlrtStack *sp, real_T state[6], const real_T varargin_1[3],
              real_T varargin_2);

void plus(const emlrtStack *sp, emxArray_real_T *in1,
          const emxArray_real_T *in2);

/* End of code generation (constvel.h) */
