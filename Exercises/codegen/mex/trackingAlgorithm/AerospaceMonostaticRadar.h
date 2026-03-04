/*
 * AerospaceMonostaticRadar.h
 *
 * Code generation for function 'AerospaceMonostaticRadar'
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
void c_AerospaceMonostaticRadar_pars(
    const emlrtStack *sp, const real_T sensorData_LookTime_data[],
    const int32_T sensorData_LookTime_size[2],
    const real_T sensorData_LookAzimuth_data[],
    const int32_T sensorData_LookAzimuth_size[2],
    const real_T sensorData_LookElevation_data[],
    const int32_T sensorData_LookElevation_size[2],
    const real_T sensorData_DetectionTime_data[],
    const int32_T sensorData_DetectionTime_size[2],
    const real_T sensorData_Azimuth_data[],
    const int32_T sensorData_Azimuth_size[2],
    const real_T sensorData_Elevation_data[],
    const int32_T sensorData_Elevation_size[2],
    const real_T sensorData_Range_data[],
    const int32_T sensorData_Range_size[2],
    const real_T sensorData_RangeRate_data[],
    const int32_T sensorData_RangeRate_size[2],
    const real_T sensorData_AzimuthAccuracy_data[],
    const int32_T sensorData_AzimuthAccuracy_size[2],
    const real_T c_sensorData_ElevationAccuracy_[],
    const int32_T d_sensorData_ElevationAccuracy_[2],
    const real_T sensorData_RangeAccuracy_data[],
    const int32_T sensorData_RangeAccuracy_size[2],
    const real_T c_sensorData_RangeRateAccuracy_[],
    const int32_T d_sensorData_RangeRateAccuracy_[2], real_T z_data[],
    int32_T z_size[2], b_emxArray_struct_T *modelData);

void c_AerospaceMonostaticRadar_time(const real_T in_LookTime_data[],
                                     const int32_T in_LookTime_size[2],
                                     const real_T in_DetectionTime_data[],
                                     const int32_T in_DetectionTime_size[2],
                                     real_T t_data[], int32_T t_size[2]);

void c_AerospaceMonostaticRadar_upda(
    const emlrtStack *sp, c_fusion_tracker_sensorspecs_Ae *obj,
    const real_T modelData_LookTime_data[],
    const int32_T modelData_LookTime_size[2],
    const real_T modelData_LookAzimuth_data[],
    const int32_T modelData_LookAzimuth_size[2],
    const real_T modelData_LookElevation_data[],
    const int32_T modelData_LookElevation_size[2],
    const real_T modelData_DetectionTime_data[],
    const int32_T modelData_DetectionTime_size[2],
    const real_T modelData_AzimuthNoise_data[],
    const int32_T modelData_AzimuthNoise_size[2],
    const real_T modelData_ElevationNoise_data[],
    const int32_T modelData_ElevationNoise_size[2],
    const real_T modelData_RangeNoise_data[],
    const int32_T modelData_RangeNoise_size[2],
    const real_T modelData_RangeRateNoise_data[],
    const int32_T modelData_RangeRateNoise_size[2]);

/* End of code generation (AerospaceMonostaticRadar.h) */
