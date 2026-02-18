/*
 * stateToMeasurementWrapped.h
 *
 * Code generation for function 'stateToMeasurementWrapped'
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
void stateToMeasurementWrapped(const emlrtStack *sp, const real_T x[6],
                               const real_T measurementModel_OriginPosition[9],
                               const real_T measurementModel_OriginVelocity[9],
                               const real_T measurementModel_Orientation[27],
                               real_T z[4]);

/* End of code generation (stateToMeasurementWrapped.h) */
