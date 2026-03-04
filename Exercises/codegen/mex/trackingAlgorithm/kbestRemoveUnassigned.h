/*
 * kbestRemoveUnassigned.h
 *
 * Code generation for function 'kbestRemoveUnassigned'
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
int32_T kbestRemoveUnassigned(const emlrtStack *sp,
                              emxArray_uint32_T *assignment,
                              const int32_T costSize[2],
                              uint32_T unassignedRows_data[],
                              emxArray_uint32_T *unassignedCols);

/* End of code generation (kbestRemoveUnassigned.h) */
