/*
 * fusecovint.h
 *
 * Code generation for function 'fusecovint'
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
void fusecovint(const emlrtStack *sp, const emxArray_real_T *trackState,
                const emxArray_real_T *trackCov, const char_T minProp[5],
                real_T fusedState[6], real_T fusedCov[36]);

/* End of code generation (fusecovint.h) */
