/*
 * trackFuser.h
 *
 * Code generation for function 'trackFuser'
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
void trackFuser_resetImpl(const emlrtStack *sp, trackFuser *obj);

void trackFuser_stepImpl(const emlrtStack *sp, trackFuser *obj,
                         const emxArray_struct0_T *localTracks, real_T tFusion,
                         struct2_T confTracks_data[], int32_T *confTracks_size);

/* End of code generation (trackFuser.h) */
