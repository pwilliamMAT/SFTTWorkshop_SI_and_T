/*
 * fusionAlgorithm_internal_types.h
 *
 * Code generation for function 'fusionAlgorithm'
 *
 */

#pragma once

/* Include files */
#include "fusionAlgorithm_types.h"
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_b_objectTrack
#define typedef_b_objectTrack
typedef struct {
  uint32_T SourceIndex;
  real_T ObjectClassID;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  struct1_T ObjectAttributes;
  real_T pState[6];
  real_T pStateCovariance[36];
  real_T pUpdateTime;
} b_objectTrack;
#endif /* typedef_b_objectTrack */

#ifndef typedef_rtDesignRangeCheckInfo
#define typedef_rtDesignRangeCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtDesignRangeCheckInfo;
#endif /* typedef_rtDesignRangeCheckInfo */

#ifndef typedef_rtRunTimeErrorInfo
#define typedef_rtRunTimeErrorInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtRunTimeErrorInfo;
#endif /* typedef_rtRunTimeErrorInfo */

/* End of code generation (fusionAlgorithm_internal_types.h) */
