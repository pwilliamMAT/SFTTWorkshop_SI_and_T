/*
 * trackEstimator.h
 *
 * Code generation for function 'trackEstimator'
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
void trackEstimator(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                    const c_fusion_tracker_targetspecs_Pa *tgtSpecs_f1,
                    const c_fusion_tracker_targetspecs_Ge *tgtSpecs_f2,
                    const c_fusion_tracker_targetspecs_He *tgtSpecs_f3,
                    const c_fusion_tracker_sensorspecs_Ae *sensorSpecs,
                    trackingEKF *iobj_0,
                    i_fusion_tracker_internal_estim *trkEstimator);

/* End of code generation (trackEstimator.h) */
