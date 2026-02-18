/*
 * jpda.h
 *
 * Code generation for function 'jpda'
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
void jpda(const emlrtStack *sp, const emxArray_real_T *likelihoodMatrix,
          real_T maxNumEvents, emxArray_real_T *posterior);

/* End of code generation (jpda.h) */
