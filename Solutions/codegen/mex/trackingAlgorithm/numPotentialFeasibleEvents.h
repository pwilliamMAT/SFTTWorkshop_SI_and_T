/*
 * numPotentialFeasibleEvents.h
 *
 * Code generation for function 'numPotentialFeasibleEvents'
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
int32_T numPotentialFeasibleEvents(const emlrtStack *sp,
                                   const emxArray_boolean_T *validationMatrix,
                                   int32_T numMeas, int32_T numTracks);

/* End of code generation (numPotentialFeasibleEvents.h) */
