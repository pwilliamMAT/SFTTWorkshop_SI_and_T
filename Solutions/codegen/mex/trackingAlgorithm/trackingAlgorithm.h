/*
 * trackingAlgorithm.h
 *
 * Code generation for function 'trackingAlgorithm'
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
emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

void trackingAlgorithm(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                       const struct0_T dets[1], const cell_4 *targetSpec,
                       const c_fusion_tracker_sensorspecs_Ae *sensorSpec,
                       emxArray_struct1_T *tracks);

void trackingAlgorithm_emx_free(const emlrtStack *sp);

void trackingAlgorithm_emx_init(const emlrtStack *sp);

void trackingAlgorithm_init(void);

/* End of code generation (trackingAlgorithm.h) */
