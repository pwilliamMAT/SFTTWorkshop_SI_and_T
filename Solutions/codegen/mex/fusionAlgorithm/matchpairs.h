/*
 * matchpairs.h
 *
 * Code generation for function 'matchpairs'
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
boolean_T
matlabPerfectMatching(const emlrtStack *sp, const emxArray_real_T *matrixRep,
                      emxArray_int32_T *matchCtoR, emxArray_int32_T *matchRtoC,
                      emxArray_real_T *rowWeight, emxArray_real_T *colWeight);

/* End of code generation (matchpairs.h) */
