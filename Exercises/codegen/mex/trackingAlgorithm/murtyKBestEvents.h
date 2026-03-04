/*
 * murtyKBestEvents.h
 *
 * Code generation for function 'murtyKBestEvents'
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
void murtyKBestEvents(const emlrtStack *sp,
                      const emxArray_real_T *likelihoodMatrix, real_T k,
                      emxArray_boolean_T *FJE, emxArray_real_T *FJEProbs);

/* End of code generation (murtyKBestEvents.h) */
