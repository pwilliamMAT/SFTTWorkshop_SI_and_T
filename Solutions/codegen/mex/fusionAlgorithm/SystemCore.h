/*
 * SystemCore.h
 *
 * Code generation for function 'SystemCore'
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
void SystemCore_releaseWrapper(trackFuser *obj);

void SystemCore_reset(const emlrtStack *sp,
                      c_matlabshared_tracking_interna *obj);

void SystemCore_step(const emlrtStack *sp, trackFuser *obj,
                     const emxArray_struct0_T *varargin_1, real_T varargin_2,
                     struct2_T varargout_1_data[], int32_T *varargout_1_size);

/* End of code generation (SystemCore.h) */
