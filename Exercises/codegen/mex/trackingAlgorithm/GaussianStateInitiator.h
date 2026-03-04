/*
 * GaussianStateInitiator.h
 *
 * Code generation for function 'GaussianStateInitiator'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void c_GaussianStateInitiator_initia(
    const emlrtStack *sp, const real_T motionModel_PropVelocityMean[3],
    const real_T c_motionModel_PropVelocityVaria[9],
    const real_T measModel_OriginPosition[9],
    const real_T measModel_OriginVelocity[9],
    const real_T measModel_Orientation[27], real_T measModel_AzimuthVariance,
    real_T measModel_ElevationVariance, real_T measModel_RangeVariance,
    real_T measModel_RangeRateVariance, const real_T measurement[4],
    real_T pdf_State[6], real_T pdf_StateCovariance[36]);

/* End of code generation (GaussianStateInitiator.h) */
