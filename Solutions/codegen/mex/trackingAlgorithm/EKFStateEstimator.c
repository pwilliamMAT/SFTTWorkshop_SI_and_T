/*
 * EKFStateEstimator.c
 *
 * Code generation for function 'EKFStateEstimator'
 *
 */

/* Include files */
#include "EKFStateEstimator.h"
#include "ExtendedKalmanFilter.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "trackingAlgorithm_types.h"
#include "trackingEKF.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo gcb_emlrtRSI = {
    138,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo hcb_emlrtRSI = {
    139,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo icb_emlrtRSI = {
    144,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo jcb_emlrtRSI = {
    145,                            /* lineNo */
    "EKFStateEstimator/likelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo knb_emlrtRSI = {
    220,                       /* lineNo */
    "EKFStateEstimator/merge", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo lnb_emlrtRSI = {
    221,                       /* lineNo */
    "EKFStateEstimator/merge", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtBCInfo qk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    211,                       /* lineNo */
    57,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo rk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    211,                       /* lineNo */
    49,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo sk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    212,                       /* lineNo */
    77,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo tk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    212,                       /* lineNo */
    69,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo uk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    217,                       /* lineNo */
    69,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo vk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    216,                       /* lineNo */
    21,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

/* Function Definitions */
real_T EKFStateEstimator_likelihood(
    const emlrtStack *sp,
    const c_fusion_tracker_sensorspecs_Ae *estimator_SensorSpecifications,
    trackingEKF *estimator_TrackingFilter, const real_T pdf_State[6],
    const real_T pdf_StateCovariance[36], const real_T measurement[4])
{
  emlrtStack st;
  real_T R[16];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &gcb_emlrtRSI;
  ExtendedKalmanFilter_set_State(&st, estimator_TrackingFilter, pdf_State);
  st.site = &hcb_emlrtRSI;
  c_ExtendedKalmanFilter_set_Stat(&st, estimator_TrackingFilter,
                                  pdf_StateCovariance);
  memset(&R[0], 0, 16U * sizeof(real_T));
  R[0] = estimator_SensorSpecifications->MeasurementModel.AzimuthVariance;
  R[5] = estimator_SensorSpecifications->MeasurementModel.ElevationVariance;
  R[10] = estimator_SensorSpecifications->MeasurementModel.RangeVariance;
  R[15] = estimator_SensorSpecifications->MeasurementModel.RangeRateVariance;
  st.site = &icb_emlrtRSI;
  c_ExtendedKalmanFilter_set_Meas(&st, estimator_TrackingFilter, R);
  st.site = &jcb_emlrtRSI;
  return trackingEKF_likelihood(
      &st, estimator_TrackingFilter, measurement,
      estimator_SensorSpecifications->MeasurementModel.OriginPosition,
      estimator_SensorSpecifications->MeasurementModel.OriginVelocity,
      estimator_SensorSpecifications->MeasurementModel.Orientation);
}

void EKFStateEstimator_merge(const emlrtStack *sp,
                             const c_emxArray_struct_T *pdfs,
                             const emxArray_real_T *weights, struct_T *pdf)
{
  __m128d r;
  __m128d r1;
  emlrtStack st;
  const struct_T *pdfs_data;
  real_T b_e[36];
  real_T e[6];
  const real_T *weights_data;
  real_T d;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  weights_data = weights->data;
  pdfs_data = pdfs->data;
  r = _mm_set1_pd(weights_data[0]);
  _mm_storeu_pd(&pdf->State[0],
                _mm_mul_pd(r, _mm_loadu_pd(&pdfs_data[0].State[0])));
  _mm_storeu_pd(&pdf->State[2],
                _mm_mul_pd(r, _mm_loadu_pd(&pdfs_data[0].State[2])));
  _mm_storeu_pd(&pdf->State[4],
                _mm_mul_pd(r, _mm_loadu_pd(&pdfs_data[0].State[4])));
  for (i = 0; i <= 34; i += 2) {
    _mm_storeu_pd(
        &pdf->StateCovariance[i],
        _mm_mul_pd(r, _mm_loadu_pd(&pdfs_data[0].StateCovariance[i])));
  }
  b_i = pdfs->size[0];
  for (i = 0; i <= b_i - 2; i++) {
    if (i + 2 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, weights->size[0], &rk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (i + 2 > b_i) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, b_i, &qk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    r = _mm_loadu_pd(&pdf->State[0]);
    d = weights_data[i + 1];
    r1 = _mm_set1_pd(d);
    _mm_storeu_pd(
        &pdf->State[0],
        _mm_add_pd(r,
                   _mm_mul_pd(r1, _mm_loadu_pd(&pdfs_data[i + 1].State[0]))));
    r = _mm_loadu_pd(&pdf->State[2]);
    _mm_storeu_pd(
        &pdf->State[2],
        _mm_add_pd(r,
                   _mm_mul_pd(r1, _mm_loadu_pd(&pdfs_data[i + 1].State[2]))));
    r = _mm_loadu_pd(&pdf->State[4]);
    _mm_storeu_pd(
        &pdf->State[4],
        _mm_add_pd(r,
                   _mm_mul_pd(r1, _mm_loadu_pd(&pdfs_data[i + 1].State[4]))));
    if (i + 2 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, weights->size[0], &tk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (i + 2 > b_i) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, b_i, &sk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    for (c_i = 0; c_i < 36; c_i++) {
      pdf->StateCovariance[c_i] += d * pdfs_data[i + 1].StateCovariance[c_i];
    }
  }
  for (c_i = 0; c_i < b_i; c_i++) {
    if (c_i + 1 > b_i) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, b_i, &vk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    r = _mm_loadu_pd(&pdf->State[0]);
    _mm_storeu_pd(&e[0], _mm_sub_pd(_mm_loadu_pd(&pdfs_data[c_i].State[0]), r));
    if (c_i + 1 > b_i) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, b_i, &vk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    r = _mm_loadu_pd(&pdf->State[2]);
    _mm_storeu_pd(&e[2], _mm_sub_pd(_mm_loadu_pd(&pdfs_data[c_i].State[2]), r));
    if (c_i + 1 > b_i) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, b_i, &vk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    r = _mm_loadu_pd(&pdf->State[4]);
    _mm_storeu_pd(&e[4], _mm_sub_pd(_mm_loadu_pd(&pdfs_data[c_i].State[4]), r));
    if (c_i + 1 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, weights->size[0], &uk_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&e[0]);
      r1 = _mm_set1_pd(e[i]);
      _mm_storeu_pd(&b_e[6 * i], _mm_mul_pd(r, r1));
      r = _mm_loadu_pd(&e[2]);
      _mm_storeu_pd(&b_e[6 * i + 2], _mm_mul_pd(r, r1));
      r = _mm_loadu_pd(&e[4]);
      _mm_storeu_pd(&b_e[6 * i + 4], _mm_mul_pd(r, r1));
    }
    for (i = 0; i < 36; i++) {
      pdf->StateCovariance[i] += weights_data[c_i] * b_e[i];
    }
  }
  st.site = &knb_emlrtRSI;
  d = c_sum(&st, weights);
  r = _mm_loadu_pd(&pdf->State[0]);
  r1 = _mm_set1_pd(d);
  _mm_storeu_pd(&pdf->State[0], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&pdf->State[2]);
  _mm_storeu_pd(&pdf->State[2], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&pdf->State[4]);
  _mm_storeu_pd(&pdf->State[4], _mm_div_pd(r, r1));
  st.site = &lnb_emlrtRSI;
  d = c_sum(&st, weights);
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&pdf->StateCovariance[i]);
    r = _mm_div_pd(r, _mm_set1_pd(d));
    _mm_storeu_pd(&pdf->StateCovariance[i], r);
  }
}

/* End of code generation (EKFStateEstimator.c) */
