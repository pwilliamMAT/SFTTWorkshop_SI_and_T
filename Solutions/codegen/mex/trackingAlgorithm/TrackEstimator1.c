/*
 * TrackEstimator1.c
 *
 * Code generation for function 'TrackEstimator1'
 *
 */

/* Include files */
#include "TrackEstimator1.h"
#include "AerospaceMonostaticRadar.h"
#include "EKFStateEstimator.h"
#include "ExtendedKalmanFilter.h"
#include "MultiModalEstimator.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "trackingEKF.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo hc_emlrtRSI = {
    56,                     /* lineNo */
    "TrackEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI = {
    71,                    /* lineNo */
    "IPDAEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI = {
    72,                    /* lineNo */
    "IPDAEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI = {
    67,                    /* lineNo */
    "IPDAEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo lc_emlrtRSI = {
    90,                          /* lineNo */
    "MultiModalEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo mc_emlrtRSI = {
    91,                          /* lineNo */
    "MultiModalEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI = {
    92,                          /* lineNo */
    "MultiModalEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI = {
    58,                        /* lineNo */
    "EKFStateEstimator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo st_emlrtRSI = {
    80,                       /* lineNo */
    "TrackEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo ey_emlrtRSI = {
    107,                       /* lineNo */
    "TrackEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo fy_emlrtRSI = {
    108,                       /* lineNo */
    "TrackEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo gy_emlrtRSI = {
    174,                      /* lineNo */
    "IPDAEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo hy_emlrtRSI = {
    220,                            /* lineNo */
    "MultiModalEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo iy_emlrtRSI = {
    190,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo jy_emlrtRSI = {
    191,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ky_emlrtRSI = {
    196,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ly_emlrtRSI = {
    199,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo my_emlrtRSI = {
    202,                          /* lineNo */
    "EKFStateEstimator/distance", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ccb_emlrtRSI = {
    87,                          /* lineNo */
    "TrackEstimator/likelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo dcb_emlrtRSI = {
    88,                          /* lineNo */
    "TrackEstimator/likelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo ecb_emlrtRSI = {
    99,                         /* lineNo */
    "IPDAEstimator/likelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo fcb_emlrtRSI = {
    100,                        /* lineNo */
    "IPDAEstimator/likelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo gcb_emlrtRSI = {
    175,                              /* lineNo */
    "MultiModalEstimator/likelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo hcb_emlrtRSI = {
    178,                              /* lineNo */
    "MultiModalEstimator/likelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

/* Function Definitions */
real_T TrackEstimator_distance(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *i_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *j_estimator_StateEstimator_Stat,
    trackingEKF *k_estimator_StateEstimator_Stat, b_struct_T *pdf,
    const real_T measurement[4], real_T b_time)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T R[16];
  real_T Y[4];
  real_T r[4];
  real_T di[3];
  real_T d;
  int32_T b_k;
  int32_T idx;
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
  st.site = &ey_emlrtRSI;
  TrackEstimator_predict(
      &st, c_estimator_StateEstimator_Stat, e_estimator_StateEstimator_Stat,
      f_estimator_StateEstimator_Stat, h_estimator_StateEstimator_Stat,
      i_estimator_StateEstimator_Stat, k_estimator_StateEstimator_Stat, pdf,
      b_time);
  st.site = &fy_emlrtRSI;
  b_st.site = &gy_emlrtRSI;
  di[0] = 1.7976931348623157E+308;
  di[1] = 1.7976931348623157E+308;
  di[2] = 1.7976931348623157E+308;
  if (pdf->IsValid[0]) {
    c_st.site = &hy_emlrtRSI;
    d_st.site = &iy_emlrtRSI;
    ExtendedKalmanFilter_set_State(&d_st, e_estimator_StateEstimator_Stat,
                                   pdf->Hypothesis[0].State);
    d_st.site = &jy_emlrtRSI;
    c_ExtendedKalmanFilter_set_Stat(&d_st, e_estimator_StateEstimator_Stat,
                                    pdf->Hypothesis[0].StateCovariance);
    memset(&R[0], 0, 16U * sizeof(real_T));
    R[0] = d_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance;
    R[5] = d_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance;
    R[10] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance;
    R[15] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance;
    d_st.site = &ky_emlrtRSI;
    c_ExtendedKalmanFilter_set_Meas(&d_st, e_estimator_StateEstimator_Stat, R);
    d_st.site = &ly_emlrtRSI;
    trackingEKF_residual(
        &d_st, e_estimator_StateEstimator_Stat, measurement,
        d_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
        d_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
        d_estimator_StateEstimator_Stat->MeasurementModel.Orientation, r, R);
    d_st.site = &my_emlrtRSI;
    Y[0] = r[0];
    Y[1] = r[1];
    Y[2] = r[2];
    Y[3] = r[3];
    e_st.site = &mbb_emlrtRSI;
    mrdiv(&e_st, Y, R);
    d = ((Y[0] * r[0] + Y[1] * r[1]) + Y[2] * r[2]) + Y[3] * r[3];
    d_st.site = &my_emlrtRSI;
    if (d < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    di[0] = muDoubleScalarSqrt(d);
  }
  if (pdf->IsValid[1]) {
    c_st.site = &hy_emlrtRSI;
    d_st.site = &iy_emlrtRSI;
    ExtendedKalmanFilter_set_State(&d_st, h_estimator_StateEstimator_Stat,
                                   pdf->Hypothesis[1].State);
    d_st.site = &jy_emlrtRSI;
    c_ExtendedKalmanFilter_set_Stat(&d_st, h_estimator_StateEstimator_Stat,
                                    pdf->Hypothesis[1].StateCovariance);
    memset(&R[0], 0, 16U * sizeof(real_T));
    R[0] = g_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance;
    R[5] = g_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance;
    R[10] = g_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance;
    R[15] = g_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance;
    d_st.site = &ky_emlrtRSI;
    c_ExtendedKalmanFilter_set_Meas(&d_st, h_estimator_StateEstimator_Stat, R);
    d_st.site = &ly_emlrtRSI;
    trackingEKF_residual(
        &d_st, h_estimator_StateEstimator_Stat, measurement,
        g_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
        g_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
        g_estimator_StateEstimator_Stat->MeasurementModel.Orientation, r, R);
    d_st.site = &my_emlrtRSI;
    Y[0] = r[0];
    Y[1] = r[1];
    Y[2] = r[2];
    Y[3] = r[3];
    e_st.site = &mbb_emlrtRSI;
    mrdiv(&e_st, Y, R);
    d = ((Y[0] * r[0] + Y[1] * r[1]) + Y[2] * r[2]) + Y[3] * r[3];
    d_st.site = &my_emlrtRSI;
    if (d < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    di[1] = muDoubleScalarSqrt(d);
  }
  if (pdf->IsValid[2]) {
    c_st.site = &hy_emlrtRSI;
    d_st.site = &iy_emlrtRSI;
    ExtendedKalmanFilter_set_State(&d_st, k_estimator_StateEstimator_Stat,
                                   pdf->Hypothesis[2].State);
    d_st.site = &jy_emlrtRSI;
    c_ExtendedKalmanFilter_set_Stat(&d_st, k_estimator_StateEstimator_Stat,
                                    pdf->Hypothesis[2].StateCovariance);
    memset(&R[0], 0, 16U * sizeof(real_T));
    R[0] = j_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance;
    R[5] = j_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance;
    R[10] = j_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance;
    R[15] = j_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance;
    d_st.site = &ky_emlrtRSI;
    c_ExtendedKalmanFilter_set_Meas(&d_st, k_estimator_StateEstimator_Stat, R);
    d_st.site = &ly_emlrtRSI;
    trackingEKF_residual(
        &d_st, k_estimator_StateEstimator_Stat, measurement,
        j_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
        j_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
        j_estimator_StateEstimator_Stat->MeasurementModel.Orientation, r, R);
    d_st.site = &my_emlrtRSI;
    Y[0] = r[0];
    Y[1] = r[1];
    Y[2] = r[2];
    Y[3] = r[3];
    e_st.site = &mbb_emlrtRSI;
    mrdiv(&e_st, Y, R);
    d = ((Y[0] * r[0] + Y[1] * r[1]) + Y[2] * r[2]) + Y[3] * r[3];
    d_st.site = &my_emlrtRSI;
    if (d < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    di[2] = muDoubleScalarSqrt(d);
  }
  if (!muDoubleScalarIsNaN(di[0])) {
    idx = 1;
  } else {
    int32_T k;
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(di[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    d = di[0];
  } else {
    d = di[idx - 1];
    idx++;
    for (b_k = idx; b_k < 4; b_k++) {
      real_T b_d;
      b_d = di[b_k - 1];
      if (d > b_d) {
        d = b_d;
      }
    }
  }
  return d;
}

real_T TrackEstimator_likelihood(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *i_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *j_estimator_StateEstimator_Stat,
    trackingEKF *k_estimator_StateEstimator_Stat, b_struct_T *pdf,
    const real_T measurement[4], real_T b_time)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T Li[3];
  real_T l;
  real_T xmax;
  int32_T b_k;
  int32_T idx;
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
  st.site = &ccb_emlrtRSI;
  TrackEstimator_predict(
      &st, c_estimator_StateEstimator_Stat, e_estimator_StateEstimator_Stat,
      f_estimator_StateEstimator_Stat, h_estimator_StateEstimator_Stat,
      i_estimator_StateEstimator_Stat, k_estimator_StateEstimator_Stat, pdf,
      b_time);
  st.site = &dcb_emlrtRSI;
  b_st.site = &ecb_emlrtRSI;
  c_st.site = &gcb_emlrtRSI;
  d_st.site = &gcb_emlrtRSI;
  l = EKFStateEstimator_likelihood(
      &d_st, d_estimator_StateEstimator_Stat, e_estimator_StateEstimator_Stat,
      pdf->Hypothesis[0].State, pdf->Hypothesis[0].StateCovariance,
      measurement);
  if (l < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  Li[0] = muDoubleScalarLog(l) + pdf->LogWeights[0];
  c_st.site = &gcb_emlrtRSI;
  d_st.site = &gcb_emlrtRSI;
  l = EKFStateEstimator_likelihood(
      &d_st, g_estimator_StateEstimator_Stat, h_estimator_StateEstimator_Stat,
      pdf->Hypothesis[1].State, pdf->Hypothesis[1].StateCovariance,
      measurement);
  if (l < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  Li[1] = muDoubleScalarLog(l);
  Li[1] += pdf->LogWeights[1];
  c_st.site = &gcb_emlrtRSI;
  d_st.site = &gcb_emlrtRSI;
  l = EKFStateEstimator_likelihood(
      &d_st, j_estimator_StateEstimator_Stat, k_estimator_StateEstimator_Stat,
      pdf->Hypothesis[2].State, pdf->Hypothesis[2].StateCovariance,
      measurement);
  if (l < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  Li[2] = muDoubleScalarLog(l);
  Li[2] += pdf->LogWeights[2];
  c_st.site = &hcb_emlrtRSI;
  if (!muDoubleScalarIsNaN(Li[0])) {
    idx = 1;
  } else {
    int32_T k;
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(Li[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    xmax = Li[0];
  } else {
    xmax = Li[idx - 1];
    idx++;
    for (b_k = idx; b_k < 4; b_k++) {
      l = Li[b_k - 1];
      if (xmax < l) {
        xmax = l;
      }
    }
  }
  if (!muDoubleScalarIsInf(xmax)) {
    Li[0] = muDoubleScalarExp(Li[0] - xmax);
    Li[1] = muDoubleScalarExp(Li[1] - xmax);
    Li[2] = muDoubleScalarExp(Li[2] - xmax);
    l = d_sumColumnB(Li);
    d_st.site = &lw_emlrtRSI;
    if (l < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
    }
    l = muDoubleScalarLog(l);
    xmax += l;
  }
  xmax = muDoubleScalarExp(xmax);
  b_st.site = &fcb_emlrtRSI;
  c_st.site = &fcb_emlrtRSI;
  l = c_MultiModalEstimator_detection(
      &c_st, d_estimator_StateEstimator_Stat, g_estimator_StateEstimator_Stat,
      j_estimator_StateEstimator_Stat, pdf->Hypothesis, pdf->LogWeights,
      pdf->IsValid);
  c_st.site = &cy_emlrtRSI;
  d_st.site = &dy_emlrtRSI;
  if (!(l >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &l_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  d_st.site = &dy_emlrtRSI;
  e_st.site = &il_emlrtRSI;
  if (!(l < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  return xmax * (l * pdf->ExistenceProbability);
}

void TrackEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    trackingEKF *d_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_Ge *e_estimator_StateEstimator_Stat,
    trackingEKF *f_estimator_StateEstimator_Stat,
    const c_fusion_tracker_targetspecs_He *g_estimator_StateEstimator_Stat,
    trackingEKF *h_estimator_StateEstimator_Stat, b_struct_T *pdf,
    real_T b_time)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
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
  dT = b_time - pdf->Time;
  if (dT > 0.0) {
    real_T val;
    st.site = &st_emlrtRSI;
    b_st.site = &ut_emlrtRSI;
    MultiModalEstimator_predict(
        &b_st, c_estimator_StateEstimator_Stat, d_estimator_StateEstimator_Stat,
        e_estimator_StateEstimator_Stat, f_estimator_StateEstimator_Stat,
        g_estimator_StateEstimator_Stat, h_estimator_StateEstimator_Stat, pdf,
        dT);
    b_st.site = &tt_emlrtRSI;
    c_st.site = &tt_emlrtRSI;
    val = c_MultiModalEstimator_survivalP(
        &c_st, c_estimator_StateEstimator_Stat, e_estimator_StateEstimator_Stat,
        g_estimator_StateEstimator_Stat, pdf->LogWeights, pdf->IsValid, dT);
    c_st.site = &ax_emlrtRSI;
    d_st.site = &bx_emlrtRSI;
    if (!(val >= 0.0)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &l_emlrtRTEI,
                                    "MATLAB:validators:mustBeNonnegative",
                                    "MATLAB:validators:mustBeNonnegative", 0);
    }
    d_st.site = &bx_emlrtRSI;
    e_st.site = &il_emlrtRSI;
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

void TrackEstimator_setup(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                          i_fusion_tracker_internal_estim *estimator,
                          trackingEKF *iobj_0)
{
  c_fusion_tracker_sensorspecs_Ae b_obj_SensorSpecifications_idx_;
  c_fusion_tracker_sensorspecs_Ae e_estimator_ExistenceEstimator_;
  c_fusion_tracker_sensorspecs_Ae obj_SensorSpecifications_idx_0;
  c_fusion_tracker_targetspecs_Ge b_obj_TargetSpecifications_idx_;
  c_fusion_tracker_targetspecs_He c_obj_TargetSpecifications_idx_;
  c_fusion_tracker_targetspecs_Pa obj_TargetSpecifications_idx_0;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  trackingEKF *b_obj_TrackingFilter;
  trackingEKF *obj_TrackingFilter;
  real_T dv[9];
  real_T c_estimator_ExistenceEstimator_;
  real_T d_estimator_ExistenceEstimator_;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &hc_emlrtRSI;
  b_st.site = &kc_emlrtRSI;
  SD->u1.f4.r = estimator->StateEstimator.StateEstimator;
  c_st.site = &lc_emlrtRSI;
  obj_TargetSpecifications_idx_0 =
      estimator->StateEstimator.StateEstimator.TargetSpecifications.f1;
  c_st.site = &mc_emlrtRSI;
  SD->u1.f4.obj_SensorSpecifications_idx_0 =
      estimator->StateEstimator.StateEstimator.SensorSpecifications[0];
  c_st.site = &nc_emlrtRSI;
  d_st.site = &oc_emlrtRSI;
  obj_TrackingFilter = trackingEKF_trackingEKF(
      &d_st, &iobj_0[0],
      estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
          .StateTransitionModel.PropAccelerationVariance);
  c_st.site = &lc_emlrtRSI;
  b_obj_TargetSpecifications_idx_ =
      estimator->StateEstimator.StateEstimator.TargetSpecifications.f2;
  c_st.site = &mc_emlrtRSI;
  obj_SensorSpecifications_idx_0 =
      estimator->StateEstimator.StateEstimator.SensorSpecifications[0];
  c_st.site = &nc_emlrtRSI;
  d_st.site = &oc_emlrtRSI;
  b_obj_TrackingFilter = trackingEKF_trackingEKF(
      &d_st, &iobj_0[1],
      estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
          .StateTransitionModel.PropAccelerationVariance);
  c_st.site = &lc_emlrtRSI;
  c_obj_TargetSpecifications_idx_ =
      estimator->StateEstimator.StateEstimator.TargetSpecifications.f3;
  c_st.site = &mc_emlrtRSI;
  b_obj_SensorSpecifications_idx_ =
      estimator->StateEstimator.StateEstimator.SensorSpecifications[0];
  c_st.site = &nc_emlrtRSI;
  d_st.site = &oc_emlrtRSI;
  iobj_0 = trackingEKF_trackingEKF(
      &d_st, &iobj_0[2],
      estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
          .StateTransitionModel.PropAccelerationVariance);
  c_estimator_ExistenceEstimator_ =
      estimator->StateEstimator.ExistenceEstimator.DetectionProbability;
  d_estimator_ExistenceEstimator_ =
      estimator->StateEstimator.ExistenceEstimator.SurvivalProbability;
  b_st.site = &ic_emlrtRSI;
  b_st.site = &jc_emlrtRSI;
  e_estimator_ExistenceEstimator_ =
      estimator->StateEstimator.StateEstimator.SensorSpecifications[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .StateTransitionModel.PropVelocityMean[0] =
      SD->u1.f4.r.TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .StateTransitionModel.PropVelocityMean[1] =
      SD->u1.f4.r.TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .StateTransitionModel.PropVelocityMean[2] =
      SD->u1.f4.r.TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[2];
  memcpy(&dv[0],
         &SD->u1.f4.r.TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
        .StateTransitionModel.PropVelocityVariance[i] = dv[i];
  }
  memcpy(&dv[0],
         &SD->u1.f4.r.TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
        .StateTransitionModel.PropAccelerationVariance[i] = dv[i];
  }
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .SurvivalModel = SD->u1.f4.r.TargetSpecifications.f1.SurvivalModel;
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .IsLockedDataType[0] =
      SD->u1.f4.r.TargetSpecifications.f1.IsLockedDataType[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .IsLockedDataType[1] =
      SD->u1.f4.r.TargetSpecifications.f1.IsLockedDataType[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .StateTransitionModel.PropVelocityMean[0] =
      SD->u1.f4.r.TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .StateTransitionModel.PropVelocityMean[1] =
      SD->u1.f4.r.TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .StateTransitionModel.PropVelocityMean[2] =
      SD->u1.f4.r.TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[2];
  memcpy(&dv[0],
         &SD->u1.f4.r.TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
        .StateTransitionModel.PropVelocityVariance[i] = dv[i];
  }
  memcpy(&dv[0],
         &SD->u1.f4.r.TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
        .StateTransitionModel.PropAccelerationVariance[i] = dv[i];
  }
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .SurvivalModel = SD->u1.f4.r.TargetSpecifications.f2.SurvivalModel;
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .IsLockedDataType[0] =
      SD->u1.f4.r.TargetSpecifications.f2.IsLockedDataType[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .IsLockedDataType[1] =
      SD->u1.f4.r.TargetSpecifications.f2.IsLockedDataType[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .StateTransitionModel.PropVelocityMean[0] =
      SD->u1.f4.r.TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .StateTransitionModel.PropVelocityMean[1] =
      SD->u1.f4.r.TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .StateTransitionModel.PropVelocityMean[2] =
      SD->u1.f4.r.TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[2];
  memcpy(&dv[0],
         &SD->u1.f4.r.TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
        .StateTransitionModel.PropVelocityVariance[i] = dv[i];
  }
  memcpy(&dv[0],
         &SD->u1.f4.r.TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
        .StateTransitionModel.PropAccelerationVariance[i] = dv[i];
  }
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .SurvivalModel = SD->u1.f4.r.TargetSpecifications.f3.SurvivalModel;
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .IsLockedDataType[0] =
      SD->u1.f4.r.TargetSpecifications.f3.IsLockedDataType[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .IsLockedDataType[1] =
      SD->u1.f4.r.TargetSpecifications.f3.IsLockedDataType[1];
  estimator->StateEstimator.StateEstimator.SensorSpecifications[0] =
      SD->u1.f4.r.SensorSpecifications[0];
  estimator->StateEstimator.StateEstimator.DeletionThreshold =
      SD->u1.f4.r.DeletionThreshold;
  estimator->StateEstimator.StateEstimator.Estimators.f1
      .TargetSpecifications[0] = obj_TargetSpecifications_idx_0;
  estimator->StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = SD->u1.f4.obj_SensorSpecifications_idx_0;
  estimator->StateEstimator.StateEstimator.Estimators.f1.TrackingFilter =
      obj_TrackingFilter;
  estimator->StateEstimator.StateEstimator.Estimators.f2
      .TargetSpecifications[0] = b_obj_TargetSpecifications_idx_;
  estimator->StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = obj_SensorSpecifications_idx_0;
  estimator->StateEstimator.StateEstimator.Estimators.f2.TrackingFilter =
      b_obj_TrackingFilter;
  estimator->StateEstimator.StateEstimator.Estimators.f3
      .TargetSpecifications[0] = c_obj_TargetSpecifications_idx_;
  estimator->StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = b_obj_SensorSpecifications_idx_;
  estimator->StateEstimator.StateEstimator.Estimators.f3.TrackingFilter =
      iobj_0;
  estimator->StateEstimator.ExistenceEstimator.SensorSpecifications[0] =
      e_estimator_ExistenceEstimator_;
  estimator->StateEstimator.ExistenceEstimator.DetectionProbability =
      c_estimator_ExistenceEstimator_;
  estimator->StateEstimator.ExistenceEstimator.SurvivalProbability =
      d_estimator_ExistenceEstimator_;
}

real_T c_TrackEstimator_sampleDistribu(
    trackingEKF *c_estimator_StateEstimator_Stat,
    trackingEKF *d_estimator_StateEstimator_Stat,
    trackingEKF *e_estimator_StateEstimator_Stat, uint32_T *pdf_TrackID,
    uint32_T *pdf_Age, boolean_T *pdf_IsConfirmed, boolean_T *pdf_IsCoasted,
    struct_T pdf_Hypothesis[3], real_T pdf_LogWeights[3],
    boolean_T pdf_IsValid[3], real_T *pdf_ExistenceProbability)
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  real_T a[36];
  real_T dv[36];
  real_T pdf_Time;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  for (i = 0; i < 6; i++) {
    pdf_Hypothesis[0].State[i] = c_estimator_StateEstimator_Stat->pState[i];
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
  memcpy(&dv[0], &c_estimator_StateEstimator_Stat->pSqrtStateCovariance[0],
         36U * sizeof(real_T));
  memcpy(&a[0], &c_estimator_StateEstimator_Stat->pSqrtStateCovariance[0],
         36U * sizeof(real_T));
  for (b_i = 0; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      pdf_Hypothesis[0].StateCovariance[i + 6 * b_i] = 0.0;
    }
    i1 = 6 * b_i + 2;
    i2 = 6 * b_i + 4;
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&a[6 * i]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[0].StateCovariance[6 * b_i]);
      r2 = _mm_set1_pd(dv[b_i + 6 * i]);
      _mm_storeu_pd(&pdf_Hypothesis[0].StateCovariance[6 * b_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i + 2]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[0].StateCovariance[i1]);
      _mm_storeu_pd(&pdf_Hypothesis[0].StateCovariance[i1],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i + 4]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[0].StateCovariance[i2]);
      _mm_storeu_pd(&pdf_Hypothesis[0].StateCovariance[i2],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i < 6; i++) {
    pdf_Hypothesis[1].State[i] = d_estimator_StateEstimator_Stat->pState[i];
  }
  if ((!d_estimator_StateEstimator_Stat->pIsSetStateCovariance) ||
      (d_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar != -1.0)) {
    pdf_Time = d_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      d_estimator_StateEstimator_Stat->pSqrtStateCovariance[i] =
          pdf_Time * (real_T)iv[i];
    }
    d_estimator_StateEstimator_Stat->pIsSetStateCovariance = true;
    d_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar = -1.0;
  }
  memcpy(&dv[0], &d_estimator_StateEstimator_Stat->pSqrtStateCovariance[0],
         36U * sizeof(real_T));
  memcpy(&a[0], &d_estimator_StateEstimator_Stat->pSqrtStateCovariance[0],
         36U * sizeof(real_T));
  for (b_i = 0; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      pdf_Hypothesis[1].StateCovariance[i + 6 * b_i] = 0.0;
    }
    i1 = 6 * b_i + 2;
    i2 = 6 * b_i + 4;
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&a[6 * i]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[1].StateCovariance[6 * b_i]);
      r2 = _mm_set1_pd(dv[b_i + 6 * i]);
      _mm_storeu_pd(&pdf_Hypothesis[1].StateCovariance[6 * b_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i + 2]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[1].StateCovariance[i1]);
      _mm_storeu_pd(&pdf_Hypothesis[1].StateCovariance[i1],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i + 4]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[1].StateCovariance[i2]);
      _mm_storeu_pd(&pdf_Hypothesis[1].StateCovariance[i2],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i < 6; i++) {
    pdf_Hypothesis[2].State[i] = e_estimator_StateEstimator_Stat->pState[i];
  }
  if ((!e_estimator_StateEstimator_Stat->pIsSetStateCovariance) ||
      (e_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar != -1.0)) {
    pdf_Time = e_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      e_estimator_StateEstimator_Stat->pSqrtStateCovariance[i] =
          pdf_Time * (real_T)iv[i];
    }
    e_estimator_StateEstimator_Stat->pIsSetStateCovariance = true;
    e_estimator_StateEstimator_Stat->pSqrtStateCovarianceScalar = -1.0;
  }
  memcpy(&dv[0], &e_estimator_StateEstimator_Stat->pSqrtStateCovariance[0],
         36U * sizeof(real_T));
  memcpy(&a[0], &e_estimator_StateEstimator_Stat->pSqrtStateCovariance[0],
         36U * sizeof(real_T));
  for (b_i = 0; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      pdf_Hypothesis[2].StateCovariance[i + 6 * b_i] = 0.0;
    }
    i1 = 6 * b_i + 2;
    i2 = 6 * b_i + 4;
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&a[6 * i]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[2].StateCovariance[6 * b_i]);
      r2 = _mm_set1_pd(dv[b_i + 6 * i]);
      _mm_storeu_pd(&pdf_Hypothesis[2].StateCovariance[6 * b_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i + 2]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[2].StateCovariance[i1]);
      _mm_storeu_pd(&pdf_Hypothesis[2].StateCovariance[i1],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i + 4]);
      r1 = _mm_loadu_pd(&pdf_Hypothesis[2].StateCovariance[i2]);
      _mm_storeu_pd(&pdf_Hypothesis[2].StateCovariance[i2],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  pdf_Time = 0.0;
  *pdf_TrackID = 0U;
  *pdf_Age = 0U;
  *pdf_IsConfirmed = false;
  *pdf_IsCoasted = false;
  pdf_LogWeights[0] = -1.0986122886681098;
  pdf_IsValid[0] = true;
  pdf_LogWeights[1] = -1.0986122886681098;
  pdf_IsValid[1] = true;
  pdf_LogWeights[2] = -1.0986122886681098;
  pdf_IsValid[2] = true;
  *pdf_ExistenceProbability = 0.0;
  return pdf_Time;
}

void c_TrackEstimator_set_TargetSpec(
    trackingAlgorithmStackData *SD, i_fusion_tracker_internal_estim *obj,
    const c_fusion_tracker_targetspecs_Pa *val_f1,
    const c_fusion_tracker_targetspecs_Ge *val_f2,
    const c_fusion_tracker_targetspecs_He *val_f3)
{
  c_fusion_tracker_internal_estim b_obj;
  d_fusion_tracker_internal_estim c_obj;
  e_fusion_tracker_internal_estim d_obj;
  SD->u1.f5.obj = obj->StateEstimator;
  SD->u1.f5.b_obj = obj->StateEstimator.StateEstimator;
  SD->u1.f5.b_obj.TargetSpecifications.f1 = *val_f1;
  SD->u1.f5.b_obj.TargetSpecifications.f2 = *val_f2;
  SD->u1.f5.b_obj.TargetSpecifications.f3 = *val_f3;
  b_obj = obj->StateEstimator.StateEstimator.Estimators.f1;
  b_obj.TargetSpecifications[0] = *val_f1;
  SD->u1.f5.b_obj.Estimators.f1 = b_obj;
  c_obj = obj->StateEstimator.StateEstimator.Estimators.f2;
  c_obj.TargetSpecifications[0] = *val_f2;
  SD->u1.f5.b_obj.Estimators.f2 = c_obj;
  d_obj = obj->StateEstimator.StateEstimator.Estimators.f3;
  d_obj.TargetSpecifications[0] = *val_f3;
  SD->u1.f5.b_obj.Estimators.f3 = d_obj;
  SD->u1.f5.obj.StateEstimator = SD->u1.f5.b_obj;
  obj->StateEstimator = SD->u1.f5.obj;
}

void c_TrackEstimator_updateEstimato(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    i_fusion_tracker_internal_estim *estimator,
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
    const int32_T modelData_RangeRateNoise_size[2])
{
  c_fusion_tracker_sensorspecs_Ae estimator_SensorSpecifications;
  c_fusion_tracker_sensorspecs_Ae val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T dv[9];
  real_T estimator_DetectionProbability;
  real_T estimator_SurvivalProbability;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &mq_emlrtRSI;
  b_st.site = &nq_emlrtRSI;
  SD->u1.f0.r = estimator->StateEstimator.StateEstimator;
  c_st.site = &pq_emlrtRSI;
  SD->u1.f0.r1 = estimator->StateEstimator.StateEstimator.Estimators.f1;
  SD->u1.f0.val = estimator->StateEstimator.StateEstimator.Estimators.f1
                      .SensorSpecifications[0];
  d_st.site = &qq_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &d_st, &SD->u1.f0.val, modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  c_st.site = &pq_emlrtRSI;
  SD->u1.f0.r2 = estimator->StateEstimator.StateEstimator.Estimators.f2;
  SD->u1.f0.b_val = estimator->StateEstimator.StateEstimator.Estimators.f2
                        .SensorSpecifications[0];
  d_st.site = &qq_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &d_st, &SD->u1.f0.b_val, modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  c_st.site = &pq_emlrtRSI;
  SD->u1.f0.r3 = estimator->StateEstimator.StateEstimator.Estimators.f3;
  val = estimator->StateEstimator.StateEstimator.Estimators.f3
            .SensorSpecifications[0];
  d_st.site = &qq_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &d_st, &val, modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  b_st.site = &oq_emlrtRSI;
  estimator_DetectionProbability =
      estimator->StateEstimator.ExistenceEstimator.DetectionProbability;
  estimator_SurvivalProbability =
      estimator->StateEstimator.ExistenceEstimator.SurvivalProbability;
  estimator_SensorSpecifications =
      estimator->StateEstimator.ExistenceEstimator.SensorSpecifications[0];
  c_st.site = &dt_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &c_st, &estimator_SensorSpecifications, modelData_LookTime_data,
      modelData_LookTime_size, modelData_LookAzimuth_data,
      modelData_LookAzimuth_size, modelData_LookElevation_data,
      modelData_LookElevation_size, modelData_DetectionTime_data,
      modelData_DetectionTime_size, modelData_AzimuthNoise_data,
      modelData_AzimuthNoise_size, modelData_ElevationNoise_data,
      modelData_ElevationNoise_size, modelData_RangeNoise_data,
      modelData_RangeNoise_size, modelData_RangeRateNoise_data,
      modelData_RangeRateNoise_size);
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .StateTransitionModel.PropVelocityMean[0] =
      SD->u1.f0.r.TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .StateTransitionModel.PropVelocityMean[1] =
      SD->u1.f0.r.TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .StateTransitionModel.PropVelocityMean[2] =
      SD->u1.f0.r.TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[2];
  memcpy(&dv[0],
         &SD->u1.f0.r.TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
        .StateTransitionModel.PropVelocityVariance[i] = dv[i];
  }
  memcpy(&dv[0],
         &SD->u1.f0.r.TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
        .StateTransitionModel.PropAccelerationVariance[i] = dv[i];
  }
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .SurvivalModel = SD->u1.f0.r.TargetSpecifications.f1.SurvivalModel;
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .IsLockedDataType[0] =
      SD->u1.f0.r.TargetSpecifications.f1.IsLockedDataType[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f1
      .IsLockedDataType[1] =
      SD->u1.f0.r.TargetSpecifications.f1.IsLockedDataType[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .StateTransitionModel.PropVelocityMean[0] =
      SD->u1.f0.r.TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .StateTransitionModel.PropVelocityMean[1] =
      SD->u1.f0.r.TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .StateTransitionModel.PropVelocityMean[2] =
      SD->u1.f0.r.TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[2];
  memcpy(&dv[0],
         &SD->u1.f0.r.TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
        .StateTransitionModel.PropVelocityVariance[i] = dv[i];
  }
  memcpy(&dv[0],
         &SD->u1.f0.r.TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
        .StateTransitionModel.PropAccelerationVariance[i] = dv[i];
  }
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .SurvivalModel = SD->u1.f0.r.TargetSpecifications.f2.SurvivalModel;
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .IsLockedDataType[0] =
      SD->u1.f0.r.TargetSpecifications.f2.IsLockedDataType[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f2
      .IsLockedDataType[1] =
      SD->u1.f0.r.TargetSpecifications.f2.IsLockedDataType[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .StateTransitionModel.PropVelocityMean[0] =
      SD->u1.f0.r.TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .StateTransitionModel.PropVelocityMean[1] =
      SD->u1.f0.r.TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[1];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .StateTransitionModel.PropVelocityMean[2] =
      SD->u1.f0.r.TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[2];
  memcpy(&dv[0],
         &SD->u1.f0.r.TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
        .StateTransitionModel.PropVelocityVariance[i] = dv[i];
  }
  memcpy(&dv[0],
         &SD->u1.f0.r.TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
        .StateTransitionModel.PropAccelerationVariance[i] = dv[i];
  }
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .SurvivalModel = SD->u1.f0.r.TargetSpecifications.f3.SurvivalModel;
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .IsLockedDataType[0] =
      SD->u1.f0.r.TargetSpecifications.f3.IsLockedDataType[0];
  estimator->StateEstimator.StateEstimator.TargetSpecifications.f3
      .IsLockedDataType[1] =
      SD->u1.f0.r.TargetSpecifications.f3.IsLockedDataType[1];
  estimator->StateEstimator.StateEstimator.SensorSpecifications[0] =
      SD->u1.f0.r.SensorSpecifications[0];
  estimator->StateEstimator.StateEstimator.DeletionThreshold =
      SD->u1.f0.r.DeletionThreshold;
  estimator->StateEstimator.StateEstimator.Estimators.f1
      .TargetSpecifications[0] = SD->u1.f0.r1.TargetSpecifications[0];
  estimator->StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = SD->u1.f0.val;
  estimator->StateEstimator.StateEstimator.Estimators.f1.TrackingFilter =
      SD->u1.f0.r1.TrackingFilter;
  estimator->StateEstimator.StateEstimator.Estimators.f2
      .TargetSpecifications[0] = SD->u1.f0.r2.TargetSpecifications[0];
  estimator->StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = SD->u1.f0.b_val;
  estimator->StateEstimator.StateEstimator.Estimators.f2.TrackingFilter =
      SD->u1.f0.r2.TrackingFilter;
  estimator->StateEstimator.StateEstimator.Estimators.f3
      .TargetSpecifications[0] = SD->u1.f0.r3.TargetSpecifications[0];
  estimator->StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = val;
  estimator->StateEstimator.StateEstimator.Estimators.f3.TrackingFilter =
      SD->u1.f0.r3.TrackingFilter;
  estimator->StateEstimator.ExistenceEstimator.SensorSpecifications[0] =
      estimator_SensorSpecifications;
  estimator->StateEstimator.ExistenceEstimator.DetectionProbability =
      estimator_DetectionProbability;
  estimator->StateEstimator.ExistenceEstimator.SurvivalProbability =
      estimator_SurvivalProbability;
}

/* End of code generation (TrackEstimator1.c) */
