/*
 * JIPDATrackInitiator.c
 *
 * Code generation for function 'JIPDATrackInitiator'
 *
 */

/* Include files */
#include "JIPDATrackInitiator.h"
#include "CompositeFieldOfViewModel.h"
#include "EKFStateEstimator.h"
#include "GaussianStateInitiator.h"
#include "logsumexp.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo tnb_emlrtRSI = {
    137,               /* lineNo */
    "initializeTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo unb_emlrtRSI = {
    70,                          /* lineNo */
    "TrackEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo vnb_emlrtRSI = {
    83,                         /* lineNo */
    "IPDAEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo wnb_emlrtRSI = {
    84,                         /* lineNo */
    "IPDAEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo xnb_emlrtRSI = {
    85,                         /* lineNo */
    "IPDAEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ynb_emlrtRSI = {
    149,                              /* lineNo */
    "MultiModalEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo aob_emlrtRSI = {
    150,                              /* lineNo */
    "MultiModalEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo bob_emlrtRSI = {
    156,                              /* lineNo */
    "MultiModalEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo cob_emlrtRSI = {
    101,                            /* lineNo */
    "EKFStateEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

/* Function Definitions */
void initializeTrack(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *i_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *j_estimator_StateEstimator_Stat,
    trackingEKF *k_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_StateEstimator_Exis,
    b_struct_T *track, const real_T z[4], real_T b_time)
{
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T b_track[6];
  real_T L[3];
  real_T Pd;
  int32_T k;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  st.site = &tnb_emlrtRSI;
  b_st.site = &unb_emlrtRSI;
  c_st.site = &vnb_emlrtRSI;
  d_st.site = &ynb_emlrtRSI;
  e_st.site = &cob_emlrtRSI;
  c_GaussianStateInitiator_initia(
      &e_st,
      c_estimator_StateEstimator_Stat->StateTransitionModel.PropVelocityMean,
      c_estimator_StateEstimator_Stat->StateTransitionModel
          .PropVelocityVariance,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
      d_estimator_StateEstimator_Stat->MeasurementModel.Orientation,
      d_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance,
      d_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance,
      d_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance,
      d_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance, z,
      track->Hypothesis[0].State, track->Hypothesis[0].StateCovariance);
  d_st.site = &aob_emlrtRSI;
  e_st.site = &aob_emlrtRSI;
  Pd = EKFStateEstimator_likelihood(
      &e_st, d_estimator_StateEstimator_Stat, e_estimator_StateEstimator_Stat,
      track->Hypothesis[0].State, track->Hypothesis[0].StateCovariance, z);
  if (Pd < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  L[0] = muDoubleScalarLog(Pd);
  d_st.site = &ynb_emlrtRSI;
  e_st.site = &cob_emlrtRSI;
  c_GaussianStateInitiator_initia(
      &e_st,
      f_estimator_StateEstimator_Stat->StateTransitionModel.PropVelocityMean,
      f_estimator_StateEstimator_Stat->StateTransitionModel
          .PropVelocityVariance,
      g_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
      g_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
      g_estimator_StateEstimator_Stat->MeasurementModel.Orientation,
      g_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance,
      g_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance,
      g_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance,
      g_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance, z,
      track->Hypothesis[1].State, track->Hypothesis[1].StateCovariance);
  d_st.site = &aob_emlrtRSI;
  e_st.site = &aob_emlrtRSI;
  Pd = EKFStateEstimator_likelihood(
      &e_st, g_estimator_StateEstimator_Stat, h_estimator_StateEstimator_Stat,
      track->Hypothesis[1].State, track->Hypothesis[1].StateCovariance, z);
  if (Pd < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  L[1] = muDoubleScalarLog(Pd);
  d_st.site = &ynb_emlrtRSI;
  e_st.site = &cob_emlrtRSI;
  c_GaussianStateInitiator_initia(
      &e_st,
      i_estimator_StateEstimator_Stat->StateTransitionModel.PropVelocityMean,
      i_estimator_StateEstimator_Stat->StateTransitionModel
          .PropVelocityVariance,
      j_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
      j_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
      j_estimator_StateEstimator_Stat->MeasurementModel.Orientation,
      j_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance,
      j_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance,
      j_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance,
      j_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance, z,
      track->Hypothesis[2].State, track->Hypothesis[2].StateCovariance);
  d_st.site = &aob_emlrtRSI;
  e_st.site = &aob_emlrtRSI;
  Pd = EKFStateEstimator_likelihood(
      &e_st, j_estimator_StateEstimator_Stat, k_estimator_StateEstimator_Stat,
      track->Hypothesis[2].State, track->Hypothesis[2].StateCovariance, z);
  if (Pd < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  L[2] = muDoubleScalarLog(Pd);
  d_st.site = &bob_emlrtRSI;
  Pd = c_logsumexp(&d_st, L);
  track->LogWeights[0] = L[0] - Pd;
  track->IsValid[0] = true;
  track->LogWeights[1] = L[1] - Pd;
  track->IsValid[1] = true;
  track->LogWeights[2] = L[2] - Pd;
  track->IsValid[2] = true;
  c_st.site = &wnb_emlrtRSI;
  d_st.site = &qx_emlrtRSI;
  b_track[0] = track->Hypothesis[0].State[0];
  b_track[3] = track->Hypothesis[0].State[1];
  b_track[1] = track->Hypothesis[0].State[2];
  b_track[4] = track->Hypothesis[0].State[3];
  b_track[2] = track->Hypothesis[0].State[4];
  b_track[5] = track->Hypothesis[0].State[5];
  e_st.site = &tx_emlrtRSI;
  L[0] = c_CompositeFieldOfViewModel_det(
      &e_st, d_estimator_StateEstimator_Stat->DetectabilityModel.FieldsOfView,
      d_estimator_StateEstimator_Stat->DetectabilityModel.NumModels, b_track);
  d_st.site = &qx_emlrtRSI;
  b_track[0] = track->Hypothesis[1].State[0];
  b_track[3] = track->Hypothesis[1].State[1];
  b_track[1] = track->Hypothesis[1].State[2];
  b_track[4] = track->Hypothesis[1].State[3];
  b_track[2] = track->Hypothesis[1].State[4];
  b_track[5] = track->Hypothesis[1].State[5];
  e_st.site = &tx_emlrtRSI;
  L[1] = c_CompositeFieldOfViewModel_det(
      &e_st, g_estimator_StateEstimator_Stat->DetectabilityModel.FieldsOfView,
      g_estimator_StateEstimator_Stat->DetectabilityModel.NumModels, b_track);
  d_st.site = &qx_emlrtRSI;
  b_track[0] = track->Hypothesis[2].State[0];
  b_track[3] = track->Hypothesis[2].State[1];
  b_track[1] = track->Hypothesis[2].State[2];
  b_track[4] = track->Hypothesis[2].State[3];
  b_track[2] = track->Hypothesis[2].State[4];
  b_track[5] = track->Hypothesis[2].State[5];
  e_st.site = &tx_emlrtRSI;
  L[2] = c_CompositeFieldOfViewModel_det(
      &e_st, j_estimator_StateEstimator_Stat->DetectabilityModel.FieldsOfView,
      j_estimator_StateEstimator_Stat->DetectabilityModel.NumModels, b_track);
  d_st.site = &rx_emlrtRSI;
  p = false;
  for (k = 0; k < 3; k++) {
    if (p || (L[k] < 0.0)) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  L[0] = muDoubleScalarLog(L[0]);
  L[1] = muDoubleScalarLog(L[1]);
  L[2] = muDoubleScalarLog(L[2]);
  r = _mm_loadu_pd(&track->LogWeights[0]);
  r1 = _mm_loadu_pd(&L[0]);
  _mm_storeu_pd(&L[0], _mm_add_pd(r, r1));
  L[2] += track->LogWeights[2];
  d_st.site = &sx_emlrtRSI;
  Pd = c_logsumexp(&d_st, L);
  Pd = muDoubleScalarExp(Pd);
  c_st.site = &xnb_emlrtRSI;
  d_st.site = &ay_emlrtRSI;
  e_st.site = &by_emlrtRSI;
  if (!(Pd >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &u_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  e_st.site = &by_emlrtRSI;
  f_st.site = &hl_emlrtRSI;
  if (!(Pd < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &f_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  Pd *= c_estimator_StateEstimator_Exis->BirthModel.BirthDensity;
  track->ExistenceProbability =
      Pd / (Pd + c_estimator_StateEstimator_Exis->ClutterModel.ClutterDensity);
  track->ExistenceProbability =
      muDoubleScalarMax(track->ExistenceProbability, 1.4901161193847656E-8);
  track->Time = b_time;
  track->Age = 1U;
  track->IsConfirmed = false;
  track->IsCoasted = false;
}

/* End of code generation (JIPDATrackInitiator.c) */
