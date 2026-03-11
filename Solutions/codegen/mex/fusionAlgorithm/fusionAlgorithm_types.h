/*
 * fusionAlgorithm_types.h
 *
 * Code generation for function 'fusionAlgorithm'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include <stddef.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3
typedef struct {
  uint32_T f1[8];
} cell_wrap_3;
#endif /* typedef_cell_wrap_3 */

#ifndef typedef_trackHistoryLogic
#define typedef_trackHistoryLogic
typedef struct {
  boolean_T pRecentHistory[50];
  boolean_T pIsFirstUpdate;
} trackHistoryLogic;
#endif /* typedef_trackHistoryLogic */

#ifndef c_typedef_fusion_internal_Fuser
#define c_typedef_fusion_internal_Fuser
typedef struct {
  char_T StateFusionParameters[5];
  real_T ProcessNoise[9];
} fusion_internal_Fuserxcov;
#endif /* c_typedef_fusion_internal_Fuser */

#ifndef c_typedef_c_matlabshared_tracki
#define c_typedef_c_matlabshared_tracki
typedef struct {
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T AssignmentThreshold[2];
  real_T pCostOfNonAssignment;
} c_matlabshared_tracking_interna;
#endif /* c_typedef_c_matlabshared_tracki */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef struct_emxArray_real_T_1x10
#define struct_emxArray_real_T_1x10
struct emxArray_real_T_1x10 {
  real_T data[10];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1x10 */
#ifndef typedef_emxArray_real_T_1x10
#define typedef_emxArray_real_T_1x10
typedef struct emxArray_real_T_1x10 emxArray_real_T_1x10;
#endif /* typedef_emxArray_real_T_1x10 */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_uint32_T
#define struct_emxArray_uint32_T
struct emxArray_uint32_T {
  uint32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_uint32_T */
#ifndef typedef_emxArray_uint32_T
#define typedef_emxArray_uint32_T
typedef struct emxArray_uint32_T emxArray_uint32_T;
#endif /* typedef_emxArray_uint32_T */

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T
struct emxArray_uint8_T {
  uint8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_uint8_T */
#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T
typedef struct emxArray_uint8_T emxArray_uint8_T;
#endif /* typedef_emxArray_uint8_T */

#ifndef c_typedef_c_matlab_internal_cod
#define c_typedef_c_matlab_internal_cod
typedef struct {
  emxArray_int32_T *heap;
  emxArray_int32_T *indexToHeap;
  int32_T len;
} c_matlab_internal_coder_minPrio;
#endif /* c_typedef_c_matlab_internal_cod */

#ifndef struct_emxArray_ptrdiff_t
#define struct_emxArray_ptrdiff_t
struct emxArray_ptrdiff_t {
  ptrdiff_t *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_ptrdiff_t */
#ifndef typedef_emxArray_ptrdiff_t
#define typedef_emxArray_ptrdiff_t
typedef struct emxArray_ptrdiff_t emxArray_ptrdiff_t;
#endif /* typedef_emxArray_ptrdiff_t */

#ifndef c_typedef_fuserSourceConfigurat
#define c_typedef_fuserSourceConfigurat
typedef struct {
  real_T SourceIndex;
  boolean_T IsInternalSource;
  boolean_T IsInitializingCentralTracks;
  boolean_T pIsTransformToCentralValid;
  boolean_T pIsTransformToLocalValid;
} fuserSourceConfiguration;
#endif /* c_typedef_fuserSourceConfigurat */

#ifndef typedef_adsbCategory
#define typedef_adsbCategory
typedef uint8_T adsbCategory;
#endif /* typedef_adsbCategory */

#ifndef adsbCategory_constants
#define adsbCategory_constants

/* enum adsbCategory */
#define No_Category_Information ((adsbCategory)0U)
#define Light ((adsbCategory)1U)
#define Small ((adsbCategory)2U)
#define Large ((adsbCategory)3U)
#define High_Vortex_Large ((adsbCategory)4U)
#define Heavy ((adsbCategory)5U)
#define High_Performance ((adsbCategory)6U)
#define Rotorcraft ((adsbCategory)7U)
#define Glider_Sailplane ((adsbCategory)8U)
#define Lighter_than_air ((adsbCategory)9U)
#define Parachutist_Skydiver ((adsbCategory)10U)
#define Ultralight ((adsbCategory)11U)
#define Unmanned_Aerial_Vehicle ((adsbCategory)12U)
#define Space_Vehicle ((adsbCategory)13U)
#define Surface_Vehicle ((adsbCategory)14U)
#define Obstacle ((adsbCategory)15U)

#endif /* adsbCategory_constants */

#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  char_T Callsign[8];
  adsbCategory Category;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_objectTrack
#define typedef_objectTrack
typedef struct {
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  uint32_T Age;
  real_T ObjectClassID;
  real_T ObjectClassProbabilities;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  struct1_T ObjectAttributes;
  real_T pState[6];
  real_T pStateCovariance[36];
  real_T pUpdateTime;
  emxArray_boolean_T *pTrackLogicState;
} objectTrack;
#endif /* typedef_objectTrack */

#ifndef typedef_trackFuser
#define typedef_trackFuser
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  cell_wrap_3 inputVarSize[2];
  real_T ProcessNoise[9];
  objectTrack pTracksList[100];
  trackHistoryLogic *pTrackLogics[100];
  real_T pNumLiveTracks;
  uint32_T pTrackIDs[100];
  boolean_T pConfirmedTracks[100];
  real_T pSourceConfigIDs[2];
  emxArray_boolean_T *pUsedConfigIDs;
  fuserSourceConfiguration *pSourceConfigurations[2];
  real_T pNumUsedConfigs;
  boolean_T pIsValidSource[2];
  real_T pLastTimeStamp;
  fusion_internal_Fuserxcov cFuser;
  c_matlabshared_tracking_interna cAssigner;
  uint32_T pLastTrackID;
  trackHistoryLogic coder_buffer_pobj0[100];
  fuserSourceConfiguration coder_buffer_pobj1[2];
} trackFuser;
#endif /* typedef_trackFuser */

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  real_T UpdateTime;
  uint32_T Age;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ObjectClassID;
  emxArray_real_T_1x10 ObjectClassProbabilities;
  char_T TrackLogic[10];
  emxArray_real_T_1x10 TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  struct1_T ObjectAttributes;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef typedef_emxArray_struct0_T
#define typedef_emxArray_struct0_T
typedef struct {
  struct0_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct0_T;
#endif /* typedef_emxArray_struct0_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  real_T UpdateTime;
  uint32_T Age;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ObjectClassID;
  real_T ObjectClassProbabilities;
  char_T TrackLogic[7];
  emxArray_boolean_T *TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  struct1_T ObjectAttributes;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef typedef_emxArray_struct2_T
#define typedef_emxArray_struct2_T
typedef struct {
  struct2_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct2_T;
#endif /* typedef_emxArray_struct2_T */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  real_T UpdateTime;
  uint32_T Age;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ObjectClassID;
  real_T ObjectClassProbabilities;
  char_T TrackLogic[7];
  boolean_T TrackLogicState[3];
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  struct1_T ObjectAttributes;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_emxArray_struct_T
#define typedef_emxArray_struct_T
typedef struct {
  struct_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct_T;
#endif /* typedef_emxArray_struct_T */

#ifndef typedef_emxArray_objectTrack
#define typedef_emxArray_objectTrack
typedef struct {
  objectTrack *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_objectTrack;
#endif /* typedef_emxArray_objectTrack */

#ifndef typedef_emxArray_struct1_T
#define typedef_emxArray_struct1_T
typedef struct {
  struct1_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_struct1_T;
#endif /* typedef_emxArray_struct1_T */

#ifndef typedef_emxArray_struct2_T_100
#define typedef_emxArray_struct2_T_100
typedef struct {
  struct2_T data[100];
  int32_T size[1];
} emxArray_struct2_T_100;
#endif /* typedef_emxArray_struct2_T_100 */

/* End of code generation (fusionAlgorithm_types.h) */
