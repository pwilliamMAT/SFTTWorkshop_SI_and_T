/*
 * JIPDATrackUpdater.h
 *
 * Code generation for function 'JIPDATrackUpdater'
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
void JIPDATrackUpdater_update(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T updater_AssignmentThreshold,
    h_fusion_tracker_internal_estim *c_updater_Estimator_StateEstima,
    emxArray_struct_T *trackList, const real_T sensorData_LookTime_data[],
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
    const int32_T d_sensorData_RangeRateAccuracy_[2],
    const emxArray_real_T *assignment);

/* End of code generation (JIPDATrackUpdater.h) */
