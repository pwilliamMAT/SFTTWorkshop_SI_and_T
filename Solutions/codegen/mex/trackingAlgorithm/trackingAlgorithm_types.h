/*
 * trackingAlgorithm_types.h
 *
 * Code generation for function 'trackingAlgorithm'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef c_typedef_c_fusion_tracker_surv
#define c_typedef_c_fusion_tracker_surv
typedef struct {
  real_T SurvivalRate;
} c_fusion_tracker_survival_Unifo;
#endif /* c_typedef_c_fusion_tracker_surv */

#ifndef c_typedef_c_fusion_tracker_birt
#define c_typedef_c_fusion_tracker_birt
typedef struct {
  real_T BirthDensity;
} c_fusion_tracker_birth_UniformP;
#endif /* c_typedef_c_fusion_tracker_birt */

#ifndef c_typedef_c_fusion_tracker_clut
#define c_typedef_c_fusion_tracker_clut
typedef struct {
  real_T ClutterDensity;
} c_fusion_tracker_clutter_Unifor;
#endif /* c_typedef_c_fusion_tracker_clut */

#ifndef c_typedef_c_fusion_tracker_tran
#define c_typedef_c_fusion_tracker_tran
typedef struct {
  real_T PropVelocityMean[3];
  real_T PropVelocityVariance[9];
  real_T PropAccelerationVariance[9];
} c_fusion_tracker_transition_Con;
#endif /* c_typedef_c_fusion_tracker_tran */

#ifndef c_typedef_c_fusion_tracker_meas
#define c_typedef_c_fusion_tracker_meas
typedef struct {
  real_T OriginPosition[9];
  real_T OriginVelocity[9];
  real_T Orientation[27];
  real_T AzimuthVariance;
  real_T ElevationVariance;
  real_T RangeVariance;
  real_T RangeRateVariance;
} c_fusion_tracker_measurement_Az;
#endif /* c_typedef_c_fusion_tracker_meas */

#ifndef c_typedef_c_fusion_tracker_dete
#define c_typedef_c_fusion_tracker_dete
typedef struct {
  real_T OriginPosition[9];
  real_T OriginVelocity[9];
  real_T Orientation[27];
  real_T AzimuthLimits[2];
  real_T ElevationLimits[2];
  real_T RangeLimits[2];
  real_T RangeRateLimits[2];
  real_T DetectionProbability;
  real_T MinDetectionProbability;
} c_fusion_tracker_detectability_;
#endif /* c_typedef_c_fusion_tracker_dete */

#ifndef c_typedef_d_fusion_tracker_dete
#define c_typedef_d_fusion_tracker_dete
typedef struct {
  c_fusion_tracker_detectability_ FieldsOfView[108];
  int32_T NumModels;
} d_fusion_tracker_detectability_;
#endif /* c_typedef_d_fusion_tracker_dete */

#ifndef typedef_trackingEKF
#define typedef_trackingEKF
typedef struct {
  real_T pState[6];
  real_T pSqrtStateCovariance[36];
  real_T pSqrtStateCovarianceScalar;
  boolean_T pIsSetStateCovariance;
  real_T pSqrtProcessNoise[9];
  real_T pSqrtProcessNoiseScalar;
  boolean_T pIsSetProcessNoise;
  real_T pSqrtMeasurementNoise[16];
  real_T pSqrtMeasurementNoiseScalar;
  boolean_T pIsValidStateTransitionFcn;
  boolean_T pIsValidMeasurementFcn;
  boolean_T pIsFirstCallPredict;
  boolean_T pIsFirstCallCorrect;
  real_T pJacobian[36];
  boolean_T pIsDistributionsSetup;
  boolean_T pIsInitialized;
  boolean_T IsLastJacobianInitialized;
} trackingEKF;
#endif /* typedef_trackingEKF */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T Time;
  uint32_T TrackID;
  uint32_T Age;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ExistenceProbability;
} struct_T;
#endif /* typedef_struct_T */

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

#ifndef c_typedef_c_fusion_tracker_inte
#define c_typedef_c_fusion_tracker_inte
typedef struct {
  real_T TimeTolerance;
  real_T CurrentIndex;
  emxArray_real_T *ProcessingTimestamps;
} c_fusion_tracker_internal_compo;
#endif /* c_typedef_c_fusion_tracker_inte */

#ifndef struct_emxArray_real_T_1x100
#define struct_emxArray_real_T_1x100
struct emxArray_real_T_1x100 {
  real_T data[100];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1x100 */
#ifndef typedef_emxArray_real_T_1x100
#define typedef_emxArray_real_T_1x100
typedef struct emxArray_real_T_1x100 emxArray_real_T_1x100;
#endif /* typedef_emxArray_real_T_1x100 */

#ifndef struct_emxArray_real_T_1x50
#define struct_emxArray_real_T_1x50
struct emxArray_real_T_1x50 {
  real_T data[50];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1x50 */
#ifndef typedef_emxArray_real_T_1x50
#define typedef_emxArray_real_T_1x50
typedef struct emxArray_real_T_1x50 emxArray_real_T_1x50;
#endif /* typedef_emxArray_real_T_1x50 */

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  emxArray_real_T_1x100 LookTime;
  emxArray_real_T_1x100 LookAzimuth;
  emxArray_real_T_1x100 LookElevation;
  emxArray_real_T_1x50 DetectionTime;
  emxArray_real_T_1x50 Azimuth;
  emxArray_real_T_1x50 Elevation;
  emxArray_real_T_1x50 Range;
  emxArray_real_T_1x50 RangeRate;
  emxArray_real_T_1x50 AzimuthAccuracy;
  emxArray_real_T_1x50 ElevationAccuracy;
  emxArray_real_T_1x50 RangeAccuracy;
  emxArray_real_T_1x50 RangeRateAccuracy;
} struct0_T;
#endif /* typedef_struct0_T */

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

#ifndef struct_emxArray_real_T_1x1
#define struct_emxArray_real_T_1x1
struct emxArray_real_T_1x1 {
  real_T data[1];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1x1 */
#ifndef typedef_emxArray_real_T_1x1
#define typedef_emxArray_real_T_1x1
typedef struct emxArray_real_T_1x1 emxArray_real_T_1x1;
#endif /* typedef_emxArray_real_T_1x1 */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  emxArray_real_T_1x100 LookTime;
  emxArray_real_T_1x100 LookAzimuth;
  emxArray_real_T_1x100 LookElevation;
  emxArray_real_T_1x1 DetectionTime;
  emxArray_real_T_1x1 AzimuthNoise;
  emxArray_real_T_1x1 ElevationNoise;
  emxArray_real_T_1x1 RangeNoise;
  emxArray_real_T_1x1 RangeRateNoise;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_b_emxArray_struct_T
#define typedef_b_emxArray_struct_T
typedef struct {
  b_struct_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} b_emxArray_struct_T;
#endif /* typedef_b_emxArray_struct_T */

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T
struct emxArray_int8_T {
  int8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int8_T */
#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T
typedef struct emxArray_int8_T emxArray_int8_T;
#endif /* typedef_emxArray_int8_T */

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

#ifndef c_typedef_c_fusion_internal_ass
#define c_typedef_c_fusion_internal_ass
typedef struct {
  emxArray_real_T *PaddedCostMatrix;
  emxArray_real_T *RowSoln;
  emxArray_real_T *ColSoln;
  emxArray_boolean_T *IsEnforced;
  int32_T CostSize[2];
  real_T BestSolutionCost;
  emxArray_boolean_T *IsDummySolution;
  boolean_T IsSolved;
  emxArray_real_T *ColReduction;
  emxArray_real_T *RowReduction;
  real_T LowerBound;
} c_fusion_internal_assignment_JV;
#endif /* c_typedef_c_fusion_internal_ass */

#ifndef c_typedef_c_emxArray_fusion_int
#define c_typedef_c_emxArray_fusion_int
typedef struct {
  c_fusion_internal_assignment_JV *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} c_emxArray_fusion_internal_assi;
#endif /* c_typedef_c_emxArray_fusion_int */

#ifndef d_typedef_c_fusion_internal_ass
#define d_typedef_c_fusion_internal_ass
typedef struct {
  int32_T NumProblems;
  int32_T MaxNumSubProblems;
  c_emxArray_fusion_internal_assi *AllProblemList;
} c_fusion_internal_assignment_Pr;
#endif /* d_typedef_c_fusion_internal_ass */

#ifndef struct_emxArray_uint32_T_51x2
#define struct_emxArray_uint32_T_51x2
struct emxArray_uint32_T_51x2 {
  uint32_T data[102];
  int32_T size[2];
};
#endif /* struct_emxArray_uint32_T_51x2 */
#ifndef typedef_emxArray_uint32_T_51x2
#define typedef_emxArray_uint32_T_51x2
typedef struct emxArray_uint32_T_51x2 emxArray_uint32_T_51x2;
#endif /* typedef_emxArray_uint32_T_51x2 */

#ifndef typedef_cell_wrap_74
#define typedef_cell_wrap_74
typedef struct {
  emxArray_uint32_T_51x2 f1;
} cell_wrap_74;
#endif /* typedef_cell_wrap_74 */

#ifndef typedef_emxArray_cell_wrap_74
#define typedef_emxArray_cell_wrap_74
typedef struct {
  cell_wrap_74 *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_cell_wrap_74;
#endif /* typedef_emxArray_cell_wrap_74 */

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

#ifndef typedef_cell_wrap_75
#define typedef_cell_wrap_75
typedef struct {
  emxArray_uint32_T *f1;
} cell_wrap_75;
#endif /* typedef_cell_wrap_75 */

#ifndef typedef_emxArray_cell_wrap_75
#define typedef_emxArray_cell_wrap_75
typedef struct {
  cell_wrap_75 *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_cell_wrap_75;
#endif /* typedef_emxArray_cell_wrap_75 */

#ifndef typedef_struct1_T
#define typedef_struct1_T
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
  char_T TrackLogic[10];
  real_T TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
} struct1_T;
#endif /* typedef_struct1_T */

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

#ifndef c_typedef_c_fusion_tracker_targ
#define c_typedef_c_fusion_tracker_targ
typedef struct {
  c_fusion_tracker_transition_Con StateTransitionModel;
  c_fusion_tracker_survival_Unifo SurvivalModel;
  boolean_T IsLockedDataType[2];
} c_fusion_tracker_targetspecs_Pa;
#endif /* c_typedef_c_fusion_tracker_targ */

#ifndef c_typedef_c_fusion_tracker_sens
#define c_typedef_c_fusion_tracker_sens
typedef struct {
  c_fusion_tracker_measurement_Az MeasurementModel;
  d_fusion_tracker_detectability_ DetectabilityModel;
  c_fusion_tracker_birth_UniformP BirthModel;
  c_fusion_tracker_clutter_Unifor ClutterModel;
  boolean_T IsLockedDataType[5];
  real_T AzimuthResolution;
  real_T ElevationResolution;
  real_T RangeResolution;
  real_T RangeRateResolution;
  real_T FalseAlarmRate;
  real_T BirthRate;
  char_T MountingLocationUnits;
  char_T MountingAnglesUnits[3];
  char_T PlatformPositionUnits;
  char_T FieldOfViewUnits[3];
  char_T RangeLimitsUnits;
  char_T RangeRateLimitsUnits[3];
  char_T AzimuthResolutionUnits[3];
  char_T ElevationResolutionUnits[3];
  char_T RangeResolutionUnits;
  char_T RangeRateResolutionUnits[3];
} c_fusion_tracker_sensorspecs_Ae;
#endif /* c_typedef_c_fusion_tracker_sens */

#ifndef d_typedef_c_fusion_tracker_inte
#define d_typedef_c_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  trackingEKF *TrackingFilter;
} c_fusion_tracker_internal_estim;
#endif /* d_typedef_c_fusion_tracker_inte */

#ifndef c_typedef_d_fusion_tracker_inte
#define c_typedef_d_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  real_T DetectionProbability;
  real_T SurvivalProbability;
} d_fusion_tracker_internal_estim;
#endif /* c_typedef_d_fusion_tracker_inte */

#ifndef c_typedef_e_fusion_tracker_inte
#define c_typedef_e_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_internal_estim StateEstimator;
  d_fusion_tracker_internal_estim ExistenceEstimator;
} e_fusion_tracker_internal_estim;
#endif /* c_typedef_e_fusion_tracker_inte */

#ifndef c_typedef_f_fusion_tracker_inte
#define c_typedef_f_fusion_tracker_inte
typedef struct {
  e_fusion_tracker_internal_estim StateEstimator;
} f_fusion_tracker_internal_estim;
#endif /* c_typedef_f_fusion_tracker_inte */

#ifndef d_typedef_d_fusion_tracker_inte
#define d_typedef_d_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  f_fusion_tracker_internal_estim Estimator;
} d_fusion_tracker_internal_compo;
#endif /* d_typedef_d_fusion_tracker_inte */

#ifndef d_typedef_e_fusion_tracker_inte
#define d_typedef_e_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  real_T AssignmentThreshold;
  real_T InitializationThreshold;
  real_T MaxNumEvents;
  f_fusion_tracker_internal_estim Estimator;
} e_fusion_tracker_internal_compo;
#endif /* d_typedef_e_fusion_tracker_inte */

#ifndef d_typedef_f_fusion_tracker_inte
#define d_typedef_f_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  real_T AssignmentThreshold;
  f_fusion_tracker_internal_estim Estimator;
} f_fusion_tracker_internal_compo;
#endif /* d_typedef_f_fusion_tracker_inte */

#ifndef c_typedef_g_fusion_tracker_inte
#define c_typedef_g_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  emxArray_struct_T *InternalTrackList;
} g_fusion_tracker_internal_compo;
#endif /* c_typedef_g_fusion_tracker_inte */

#ifndef c_typedef_h_fusion_tracker_inte
#define c_typedef_h_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  real_T ConfirmationThreshold;
  real_T DeletionThreshold;
  f_fusion_tracker_internal_estim Estimator;
} h_fusion_tracker_internal_compo;
#endif /* c_typedef_h_fusion_tracker_inte */

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
  real_T pState[6];
  real_T pStateCovariance[36];
  real_T pUpdateTime;
  real_T pTrackLogicState;
} objectTrack;
#endif /* typedef_objectTrack */

#ifndef c_typedef_i_fusion_tracker_inte
#define c_typedef_i_fusion_tracker_inte
typedef struct {
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  f_fusion_tracker_internal_estim Estimator;
  objectTrack SampleTrack;
} i_fusion_tracker_internal_compo;
#endif /* c_typedef_i_fusion_tracker_inte */

#ifndef c_typedef_b_JIPDATrackUpdater_u
#define c_typedef_b_JIPDATrackUpdater_u
typedef struct {
  c_fusion_tracker_sensorspecs_Ae updater_Estimator_StateEstimato;
} b_JIPDATrackUpdater_update;
#endif /* c_typedef_b_JIPDATrackUpdater_u */

#ifndef c_typedef_b_JIPDATrackAssigner_
#define c_typedef_b_JIPDATrackAssigner_
typedef struct {
  c_fusion_tracker_sensorspecs_Ae val;
  c_fusion_tracker_sensorspecs_Ae estimator_SensorSpecifications;
} b_JIPDATrackAssigner_assign;
#endif /* c_typedef_b_JIPDATrackAssigner_ */

#ifndef c_typedef_b_SystemCore_checkTun
#define c_typedef_b_SystemCore_checkTun
typedef struct {
  i_fusion_tracker_internal_compo obj;
  e_fusion_tracker_internal_compo b_obj;
  h_fusion_tracker_internal_compo c_obj;
  f_fusion_tracker_internal_compo d_obj;
  d_fusion_tracker_internal_compo e_obj;
} b_SystemCore_checkTunableProps;
#endif /* c_typedef_b_SystemCore_checkTun */

#ifndef typedef_b_JIPDATracker_stepImpl
#define typedef_b_JIPDATracker_stepImpl
typedef struct {
  i_fusion_tracker_internal_compo r1;
  h_fusion_tracker_internal_compo r;
  d_fusion_tracker_internal_compo r2;
  e_fusion_tracker_internal_estim t39_Estimator_StateEstimator;
  g_fusion_tracker_internal_compo obj;
  g_fusion_tracker_internal_compo b_obj;
  c_fusion_tracker_sensorspecs_Ae val;
  c_fusion_tracker_sensorspecs_Ae estimator_SensorSpecifications;
  c_fusion_tracker_sensorspecs_Ae b_val;
} b_JIPDATracker_stepImpl;
#endif /* typedef_b_JIPDATracker_stepImpl */

#ifndef typedef_b_trackingAlgorithm_api
#define typedef_b_trackingAlgorithm_api
typedef struct {
  c_fusion_tracker_sensorspecs_Ae sensorSpec;
} b_trackingAlgorithm_api;
#endif /* typedef_b_trackingAlgorithm_api */

#ifndef c_typedef_fusion_tracker_JIPDAT
#define c_typedef_fusion_tracker_JIPDAT
typedef struct {
  boolean_T tunablePropertyChanged[7];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  c_fusion_tracker_targetspecs_Pa TargetSpecifications[1];
  c_fusion_tracker_sensorspecs_Ae SensorSpecifications[1];
  real_T MaxMahalanobisDistance;
  real_T c_ConfirmationExistenceProbabil;
  real_T DeletionExistenceProbability;
  real_T MaxNumEvents;
  c_fusion_tracker_internal_compo *Scheduler;
  d_fusion_tracker_internal_compo Initiator[1];
  e_fusion_tracker_internal_compo Assigner[1];
  f_fusion_tracker_internal_compo Updater[1];
  g_fusion_tracker_internal_compo TrackListManager;
  h_fusion_tracker_internal_compo TrackMaintenance;
  i_fusion_tracker_internal_compo Outputter;
  uint32_T LastTrackID;
  trackingEKF coder_buffer_pobj0[5];
  c_fusion_tracker_internal_compo coder_buffer_pobj1;
} fusion_tracker_JIPDATracker;
#endif /* c_typedef_fusion_tracker_JIPDAT */

#ifndef typedef_b_SystemCore_step
#define typedef_b_SystemCore_step
typedef struct {
  i_fusion_tracker_internal_compo r;
  e_fusion_tracker_internal_compo assigners;
  f_fusion_tracker_internal_compo updaters;
  d_fusion_tracker_internal_compo initiators;
  g_fusion_tracker_internal_compo obj;
  c_fusion_tracker_sensorspecs_Ae spec;
  c_fusion_tracker_sensorspecs_Ae c_obj_Estimator_StateEstimator_;
  c_fusion_tracker_sensorspecs_Ae d_obj_Estimator_StateEstimator_;
} b_SystemCore_step;
#endif /* typedef_b_SystemCore_step */

#ifndef c_typedef_trackingAlgorithmStac
#define c_typedef_trackingAlgorithmStac
typedef struct {
  union {
    b_JIPDATrackUpdater_update f0;
    b_JIPDATrackAssigner_assign f1;
    b_SystemCore_checkTunableProps f2;
  } u1;
  b_JIPDATracker_stepImpl f3;
  b_SystemCore_step f4;
  b_trackingAlgorithm_api f5;
} trackingAlgorithmStackData;
#endif /* c_typedef_trackingAlgorithmStac */

/* End of code generation (trackingAlgorithm_types.h) */
