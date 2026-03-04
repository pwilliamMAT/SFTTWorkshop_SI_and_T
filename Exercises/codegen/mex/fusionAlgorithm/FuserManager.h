/*
 * FuserManager.h
 *
 * Code generation for function 'FuserManager'
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
void FuserManager_distance(const emlrtStack *sp, trackFuser *obj,
                           const emxArray_struct0_T *localTracks,
                           emxArray_real_T *costMatrix);

fuserSourceConfiguration *FuserManager_getConfigByID(const emlrtStack *sp,
                                                     trackFuser *obj,
                                                     uint32_T configID);

void FuserManager_setupImpl(const emlrtStack *sp, trackFuser *obj,
                            const emxArray_struct0_T *tracks);

/* End of code generation (FuserManager.h) */
