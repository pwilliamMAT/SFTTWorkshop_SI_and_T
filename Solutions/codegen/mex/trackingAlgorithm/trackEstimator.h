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
trackingEKF *trackEstimator(
    const emlrtStack *sp, const c_fusion_tracker_targetspecs_Pa *tgtSpecs,
    const c_fusion_tracker_sensorspecs_Ae *sensorSpecs, trackingEKF *iobj_0,
    c_fusion_tracker_targetspecs_Pa *c_trkEstimator_StateEstimator_S,
    c_fusion_tracker_sensorspecs_Ae *d_trkEstimator_StateEstimator_S,
    c_fusion_tracker_sensorspecs_Ae *c_trkEstimator_StateEstimator_E);

/* End of code generation (trackEstimator.h) */
