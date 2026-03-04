/*
 * connectedTracks.h
 *
 * Code generation for function 'connectedTracks'
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
int32_T connectedTracks(const emlrtStack *sp, const emxArray_boolean_T *A,
                        int32_T clustRows_data[], int32_T clustRows_size[2],
                        emxArray_int32_T *clustCols);

/* End of code generation (connectedTracks.h) */
