/*
 * EKFCorrectorAdditive.h
 *
 * Code generation for function 'EKFCorrectorAdditive'
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
void c_EKFCorrectorAdditive_getMeasu(const emlrtStack *sp, const real_T Rs[16],
                                     const real_T x[6], const real_T S[36],
                                     const real_T varargin_2_OriginPosition[9],
                                     const real_T varargin_2_OriginVelocity[9],
                                     const real_T varargin_2_Orientation[27],
                                     real_T zEstimated[4], real_T Pxy[24],
                                     real_T Sy[16], real_T dHdx[24]);

/* End of code generation (EKFCorrectorAdditive.h) */
