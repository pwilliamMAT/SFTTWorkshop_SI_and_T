/*
 * computeLikelihoodByIndex.h
 *
 * Code generation for function 'computeLikelihoodByIndex'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "trackingAlgorithm_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T parseIndex(const emlrtStack *sp, const emxArray_struct_T *trackList,
                  const int32_T z_size[2], real_T b_index, real_T *measIndex);

/* End of code generation (computeLikelihoodByIndex.h) */
