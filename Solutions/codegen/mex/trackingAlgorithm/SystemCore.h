/*
 * SystemCore.h
 *
 * Code generation for function 'SystemCore'
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
void SystemCore_step(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                     fusion_tracker_JIPDATracker *obj,
                     const real_T varargin_1_LookTime_data[],
                     const int32_T varargin_1_LookTime_size[2],
                     const real_T varargin_1_LookAzimuth_data[],
                     const int32_T varargin_1_LookAzimuth_size[2],
                     const real_T varargin_1_LookElevation_data[],
                     const int32_T varargin_1_LookElevation_size[2],
                     const real_T varargin_1_DetectionTime_data[],
                     const int32_T varargin_1_DetectionTime_size[2],
                     const real_T varargin_1_Azimuth_data[],
                     const int32_T varargin_1_Azimuth_size[2],
                     const real_T varargin_1_Elevation_data[],
                     const int32_T varargin_1_Elevation_size[2],
                     const real_T varargin_1_Range_data[],
                     const int32_T varargin_1_Range_size[2],
                     const real_T varargin_1_RangeRate_data[],
                     const int32_T varargin_1_RangeRate_size[2],
                     const real_T varargin_1_AzimuthAccuracy_data[],
                     const int32_T varargin_1_AzimuthAccuracy_size[2],
                     const real_T c_varargin_1_ElevationAccuracy_[],
                     const int32_T d_varargin_1_ElevationAccuracy_[2],
                     const real_T varargin_1_RangeAccuracy_data[],
                     const int32_T varargin_1_RangeAccuracy_size[2],
                     const real_T c_varargin_1_RangeRateAccuracy_[],
                     const int32_T d_varargin_1_RangeRateAccuracy_[2],
                     emxArray_struct1_T *varargout_1);

/* End of code generation (SystemCore.h) */
