/*
 * TrackEstimator1.c
 *
 * Code generation for function 'TrackEstimator1'
 *
 */

/* Include files */
#include "TrackEstimator1.h"
#include "EKFStateEstimator.h"
#include "ExtendedKalmanFilter.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "trackingEKF.h"
#include "xzgetrf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo at_emlrtRSI = {
    92,                                    /* lineNo */
    "TrackEstimator/likelihoodUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo bt_emlrtRSI = {
    93,                                    /* lineNo */
    "TrackEstimator/likelihoodUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo ct_emlrtRSI = {
    80,                       /* lineNo */
    "TrackEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo fw_emlrtRSI = {
    107,                                  /* lineNo */
    "IPDAEstimator/likelihoodUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo gw_emlrtRSI = {
    108,                                  /* lineNo */
    "IPDAEstimator/likelihoodUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo pw_emlrtRSI = {
    107,                       /* lineNo */
    "TrackEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo qw_emlrtRSI = {
    108,                       /* lineNo */
    "TrackEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo rw_emlrtRSI = {
    174,                      /* lineNo */
    "IPDAEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo sw_emlrtRSI = {
    190,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo tw_emlrtRSI = {
    191,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo uw_emlrtRSI = {
    196,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo vw_emlrtRSI = {
    199,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ww_emlrtRSI = {
    202,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo mab_emlrtRSI = {
    87,                          /* lineNo */
    "TrackEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo nab_emlrtRSI = {
    88,                          /* lineNo */
    "TrackEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo oab_emlrtRSI = {
    99,                         /* lineNo */
    "IPDAEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo pab_emlrtRSI = {
    100,                        /* lineNo */
    "IPDAEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo qab_emlrtRSI = {
    138,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo rab_emlrtRSI = {
    139,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo sab_emlrtRSI = {
    144,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo tab_emlrtRSI = {
    145,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo uab_emlrtRSI =
    {
        395,                      /* lineNo */
        "trackingEKF/likelihood", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo vab_emlrtRSI =
    {
        396,                      /* lineNo */
        "trackingEKF/likelihood", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo wab_emlrtRSI = {
    19,                 /* lineNo */
    "KalmanLikelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\KalmanLikelihood"
    ".m" /* pathName */
};

static emlrtRSInfo xab_emlrtRSI = {
    21,                 /* lineNo */
    "KalmanLikelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\KalmanLikelihood"
    ".m" /* pathName */
};

static emlrtRSInfo yab_emlrtRSI = {
    21,    /* lineNo */
    "det", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\det.m" /* pathName
                                                                       */
};

/* Function Definitions */
real_T TrackEstimator_distance(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, const struct_T *pdf,
    const real_T measurement[4], real_T b_time)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  struct_T b_pdf;
  real_T R[16];
  real_T Y[4];
  real_T r[4];
  real_T d;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_pdf = *pdf;
  st.site = &pw_emlrtRSI;
  TrackEstimator_predict(&st, c_estimator_StateEstimator_Stat,
                         e_estimator_StateEstimator_Stat, &b_pdf, b_time);
  st.site = &qw_emlrtRSI;
  b_st.site = &rw_emlrtRSI;
  c_st.site = &sw_emlrtRSI;
  ExtendedKalmanFilter_set_State(&c_st, e_estimator_StateEstimator_Stat,
                                 b_pdf.State);
  c_st.site = &tw_emlrtRSI;
  c_ExtendedKalmanFilter_set_Stat(&c_st, e_estimator_StateEstimator_Stat,
                                  b_pdf.StateCovariance);
  memset(&R[0], 0, 16U * sizeof(real_T));
  R[0] = d_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance;
  R[5] = d_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance;
  R[10] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance;
  R[15] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance;
  c_st.site = &uw_emlrtRSI;
  c_ExtendedKalmanFilter_set_Meas(&c_st, e_estimator_StateEstimator_Stat, R);
  c_st.site = &vw_emlrtRSI;
  trackingEKF_residual(
      &c_st, e_estimator_StateEstimator_Stat, measurement,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
      d_estimator_StateEstimator_Stat->MeasurementModel.Orientation, r, R);
  c_st.site = &ww_emlrtRSI;
  Y[0] = r[0];
  Y[1] = r[1];
  Y[2] = r[2];
  Y[3] = r[3];
  d_st.site = &wy_emlrtRSI;
  mrdiv(&d_st, Y, R);
  d = ((Y[0] * r[0] + Y[1] * r[1]) + Y[2] * r[2]) + Y[3] * r[3];
  c_st.site = &ww_emlrtRSI;
  if (d < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  return muDoubleScalarSqrt(d);
}

real_T TrackEstimator_likelihood(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, struct_T *pdf,
    const real_T measurement[4], real_T b_time)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T R[16];
  real_T Y[4];
  real_T zres[4];
  real_T l;
  real_T val;
  int32_T ipiv[4];
  boolean_T isodd;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  st.site = &mab_emlrtRSI;
  TrackEstimator_predict(&st, c_estimator_StateEstimator_Stat,
                         e_estimator_StateEstimator_Stat, pdf, b_time);
  st.site = &nab_emlrtRSI;
  b_st.site = &oab_emlrtRSI;
  c_st.site = &qab_emlrtRSI;
  ExtendedKalmanFilter_set_State(&c_st, e_estimator_StateEstimator_Stat,
                                 pdf->State);
  c_st.site = &rab_emlrtRSI;
  c_ExtendedKalmanFilter_set_Stat(&c_st, e_estimator_StateEstimator_Stat,
                                  pdf->StateCovariance);
  memset(&R[0], 0, 16U * sizeof(real_T));
  R[0] = d_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance;
  R[5] = d_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance;
  R[10] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance;
  R[15] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance;
  c_st.site = &sab_emlrtRSI;
  c_ExtendedKalmanFilter_set_Meas(&c_st, e_estimator_StateEstimator_Stat, R);
  c_st.site = &tab_emlrtRSI;
  d_st.site = &uab_emlrtRSI;
  trackingEKF_residual(
      &d_st, e_estimator_StateEstimator_Stat, measurement,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
      d_estimator_StateEstimator_Stat->MeasurementModel.Orientation, zres, R);
  d_st.site = &vab_emlrtRSI;
  e_st.site = &wab_emlrtRSI;
  Y[0] = zres[0];
  Y[1] = zres[1];
  Y[2] = zres[2];
  Y[3] = zres[3];
  f_st.site = &wy_emlrtRSI;
  mrdiv(&f_st, Y, R);
  e_st.site = &xab_emlrtRSI;
  f_st.site = &yab_emlrtRSI;
  g_st.site = &eab_emlrtRSI;
  xzgetrf(&g_st, R, ipiv);
  isodd = (ipiv[0] > 1);
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }
  l = R[0] * R[5] * R[10] * R[15];
  if (ipiv[2] > 3) {
    isodd = !isodd;
  }
  if (isodd) {
    l = -l;
  }
  e_st.site = &xab_emlrtRSI;
  if (l < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  b_st.site = &pab_emlrtRSI;
  c_st.site = &pab_emlrtRSI;
  val = c_EKFStateEstimator_detectionPr(&c_st, d_estimator_StateEstimator_Stat,
                                        pdf->State);
  c_st.site = &nw_emlrtRSI;
  d_st.site = &ow_emlrtRSI;
  if (!(val >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &u_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  d_st.site = &ow_emlrtRSI;
  e_st.site = &tk_emlrtRSI;
  if (!(val < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  return muDoubleScalarMax(
             muDoubleScalarExp(
                 -(((Y[0] * zres[0] + Y[1] * zres[1]) + Y[2] * zres[2]) +
                   Y[3] * zres[3]) /
                 2.0) /
                 39.478417604357432 / muDoubleScalarSqrt(l),
             2.2250738585072014E-308) *
         (val * pdf->ExistenceProbability);
}

void TrackEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    trackingEKF *d_estimator_StateEstimator_Stat, struct_T *pdf, real_T b_time)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T dT;
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
  dT = b_time - pdf->Time;
  if (dT > 0.0) {
    real_T val;
    st.site = &ct_emlrtRSI;
    b_st.site = &et_emlrtRSI;
    EKFStateEstimator_predict(&b_st, c_estimator_StateEstimator_Stat,
                              d_estimator_StateEstimator_Stat, pdf, dT);
    b_st.site = &dt_emlrtRSI;
    c_st.site = &nv_emlrtRSI;
    d_st.site = &ov_emlrtRSI;
    e_st.site = &id_emlrtRSI;
    f_st.site = &jd_emlrtRSI;
    if ((c_estimator_StateEstimator_Stat->SurvivalModel.SurvivalRate < 0.0) &&
        (muDoubleScalarFloor(dT) != dT)) {
      emlrtErrorWithMessageIdR2018a(&f_st, &dc_emlrtRTEI,
                                    "Coder:toolbox:power_domainError",
                                    "Coder:toolbox:power_domainError", 0);
    }
    val = muDoubleScalarPower(
        c_estimator_StateEstimator_Stat->SurvivalModel.SurvivalRate, dT);
    b_st.site = &dt_emlrtRSI;
    c_st.site = &pv_emlrtRSI;
    d_st.site = &qv_emlrtRSI;
    if (!(val >= 0.0)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &u_emlrtRTEI,
                                    "MATLAB:validators:mustBeNonnegative",
                                    "MATLAB:validators:mustBeNonnegative", 0);
    }
    d_st.site = &qv_emlrtRSI;
    e_st.site = &tk_emlrtRSI;
    if (!(val < 1.0)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
          "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
    }
    pdf->ExistenceProbability *= val;
  }
  pdf->Time += dT;
  pdf->IsCoasted = true;
}

real_T c_TrackEstimator_likelihoodUnas(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, struct_T *pdf, real_T b_time,
    real_T gateSize)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T l;
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
  st.site = &at_emlrtRSI;
  TrackEstimator_predict(&st, c_estimator_StateEstimator_Stat,
                         e_estimator_StateEstimator_Stat, pdf, b_time);
  st.site = &bt_emlrtRSI;
  b_st.site = &fw_emlrtRSI;
  l = c_EKFStateEstimator_gateProbabi(&b_st, gateSize);
  b_st.site = &gw_emlrtRSI;
  c_st.site = &gw_emlrtRSI;
  l *= c_EKFStateEstimator_detectionPr(&c_st, d_estimator_StateEstimator_Stat,
                                       pdf->State);
  c_st.site = &nw_emlrtRSI;
  d_st.site = &ow_emlrtRSI;
  if (!(l >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &u_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  d_st.site = &ow_emlrtRSI;
  e_st.site = &tk_emlrtRSI;
  if (!(l < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  return 1.0 - l * pdf->ExistenceProbability;
}

real_T c_TrackEstimator_sampleDistribu(
    trackingEKF *c_estimator_StateEstimator_Stat, uint32_T *pdf_TrackID,
    uint32_T *pdf_Age, boolean_T *pdf_IsConfirmed, boolean_T *pdf_IsCoasted,
    real_T pdf_State[6], real_T pdf_StateCovariance[36],
    real_T *pdf_ExistenceProbability)
{
  real_T pdf_Time;
  int32_T i;
  int32_T i2;
  for (i = 0; i < 6; i++) {
    pdf_State[i] = c_estimator_StateEstimator_Stat->pState[i];
  }
  if ((!c_estimator_StateEstimator_Stat->pIsSetStateCovariance) ||
      (c_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar != -1.0)) {
    pdf_Time = c_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      c_estimator_StateEstimator_Stat->pSqrtStateCovariance[i] =
          pdf_Time * (real_T)iv[i];
    }
    c_estimator_StateEstimator_Stat->pIsSetStateCovariance = true;
    c_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar = -1.0;
  }
  pdf_Time = 0.0;
  *pdf_TrackID = 0U;
  *pdf_Age = 0U;
  *pdf_IsConfirmed = false;
  *pdf_IsCoasted = false;
  memset(&pdf_StateCovariance[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    int32_T b_i;
    int32_T i1;
    b_i = 6 * i + 2;
    i1 = 6 * i + 4;
    for (i2 = 0; i2 < 6; i2++) {
      __m128d r;
      __m128d r1;
      __m128d r2;
      r = _mm_loadu_pd(
          &c_estimator_StateEstimator_Stat->pSqrtStateCovariance[6 * i2]);
      r1 = _mm_loadu_pd(&pdf_StateCovariance[6 * i]);
      r2 = _mm_set1_pd(
          c_estimator_StateEstimator_Stat->pSqrtStateCovariance[i + 6 * i2]);
      _mm_storeu_pd(&pdf_StateCovariance[6 * i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(
          &c_estimator_StateEstimator_Stat->pSqrtStateCovariance[6 * i2 + 2]);
      r1 = _mm_loadu_pd(&pdf_StateCovariance[b_i]);
      _mm_storeu_pd(&pdf_StateCovariance[b_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(
          &c_estimator_StateEstimator_Stat->pSqrtStateCovariance[6 * i2 + 4]);
      r1 = _mm_loadu_pd(&pdf_StateCovariance[i1]);
      _mm_storeu_pd(&pdf_StateCovariance[i1],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  *pdf_ExistenceProbability = 0.0;
  return pdf_Time;
}

/* End of code generation (TrackEstimator1.c) */
