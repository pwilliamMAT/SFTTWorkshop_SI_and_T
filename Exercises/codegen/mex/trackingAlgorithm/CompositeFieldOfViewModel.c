/*
 * CompositeFieldOfViewModel.c
 *
 * Code generation for function 'CompositeFieldOfViewModel'
 *
 */

/* Include files */
#include "CompositeFieldOfViewModel.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo wx_emlrtRSI = {
    61,                                        /* lineNo */
    "CompositeFieldOfViewModel/detectability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+detectability/"
    "CompositeFieldOfViewModel.m" /* pathName */
};

static emlrtRSInfo xx_emlrtRSI = {
    62,                                        /* lineNo */
    "CompositeFieldOfViewModel/detectability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+detectability/"
    "CompositeFieldOfViewModel.m" /* pathName */
};

static emlrtBCInfo ad_emlrtBCI = {
    0,                                         /* iFirst */
    107,                                       /* iLast */
    62,                                        /* lineNo */
    55,                                        /* colNo */
    "",                                        /* aName */
    "CompositeFieldOfViewModel/detectability", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+detectability/"
    "CompositeFieldOfViewModel.m", /* pName */
    0                              /* checkKind */
};

/* Function Definitions */
real_T c_CompositeFieldOfViewModel_det(
    const emlrtStack *sp,
    const c_fusion_tracker_detectability_ model_FieldsOfView[108],
    int32_T model_NumModels, const real_T state[6])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T b_xP[3];
  real_T c_xP[3];
  real_T xP[3];
  real_T Pd;
  real_T Pmiss;
  int32_T b_i;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  Pmiss = 1.0;
  st.site = &wx_emlrtRSI;
  if (model_NumModels > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (i = 0; i < model_NumModels; i++) {
    __m128d r;
    real_T azimuth;
    real_T d;
    real_T d1;
    real_T d2;
    real_T d3;
    real_T d4;
    real_T d5;
    real_T d6;
    real_T d7;
    real_T elevation;
    real_T rr;
    real_T xP_tmp;
    st.site = &xx_emlrtRSI;
    if (i > 107) {
      emlrtDynamicBoundsCheckR2012b(i, 0, 107, &ad_emlrtBCI, &st);
    }
    xP[0] = state[0];
    b_xP[0] = state[3];
    xP[1] = state[1];
    b_xP[1] = state[4];
    xP[2] = state[2];
    b_xP[2] = state[5];
    for (b_i = 0; b_i < 3; b_i++) {
      int32_T c_i;
      int32_T i1;
      r = _mm_loadu_pd(&xP[0]);
      c_i = 3 * (2 - b_i);
      _mm_storeu_pd(
          &c_xP[0],
          _mm_sub_pd(r,
                     _mm_loadu_pd(&model_FieldsOfView[i].OriginPosition[c_i])));
      c_xP[2] = xP[2] - model_FieldsOfView[i].OriginPosition[c_i + 2];
      memset(&xP[0], 0, 3U * sizeof(real_T));
      r = _mm_loadu_pd(&xP[0]);
      i1 = 9 * (2 - b_i);
      _mm_storeu_pd(
          &xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &model_FieldsOfView[i].Orientation[i1]),
                                   _mm_set1_pd(c_xP[0]))));
      Pd = model_FieldsOfView[i].Orientation[i1 + 2];
      xP[2] += Pd * c_xP[0];
      c_xP[0] = b_xP[0] - model_FieldsOfView[i].OriginVelocity[c_i];
      r = _mm_loadu_pd(&xP[0]);
      _mm_storeu_pd(
          &xP[0],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&model_FieldsOfView[i].Orientation[i1 + 3]),
                     _mm_set1_pd(c_xP[1]))));
      rr = model_FieldsOfView[i].Orientation[i1 + 5];
      xP[2] += rr * c_xP[1];
      c_xP[1] = b_xP[1] - model_FieldsOfView[i].OriginVelocity[c_i + 1];
      r = _mm_loadu_pd(&xP[0]);
      _mm_storeu_pd(
          &xP[0],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&model_FieldsOfView[i].Orientation[i1 + 6]),
                     _mm_set1_pd(c_xP[2]))));
      xP_tmp = model_FieldsOfView[i].Orientation[i1 + 8];
      xP[2] += xP_tmp * c_xP[2];
      c_xP[2] = b_xP[2] - model_FieldsOfView[i].OriginVelocity[c_i + 2];
      memset(&b_xP[0], 0, 3U * sizeof(real_T));
      r = _mm_loadu_pd(&b_xP[0]);
      _mm_storeu_pd(
          &b_xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &model_FieldsOfView[i].Orientation[i1]),
                                   _mm_set1_pd(c_xP[0]))));
      b_xP[2] += Pd * c_xP[0];
      r = _mm_loadu_pd(&b_xP[0]);
      _mm_storeu_pd(
          &b_xP[0],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&model_FieldsOfView[i].Orientation[i1 + 3]),
                     _mm_set1_pd(c_xP[1]))));
      b_xP[2] += rr * c_xP[1];
      r = _mm_loadu_pd(&b_xP[0]);
      _mm_storeu_pd(
          &b_xP[0],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&model_FieldsOfView[i].Orientation[i1 + 6]),
                     _mm_set1_pd(c_xP[2]))));
      b_xP[2] += xP_tmp * c_xP[2];
    }
    azimuth = 57.295779513082323 * muDoubleScalarAtan2(xP[1], xP[0]);
    Pd = xP[0] * xP[0] + xP[1] * xP[1];
    elevation =
        57.295779513082323 * muDoubleScalarAtan2(xP[2], muDoubleScalarSqrt(Pd));
    Pd = muDoubleScalarSqrt(Pd + xP[2] * xP[2]);
    r = _mm_loadu_pd(&xP[0]);
    _mm_storeu_pd(&xP[0], _mm_div_pd(r, _mm_set1_pd(Pd)));
    xP[2] /= Pd;
    rr = (b_xP[0] * xP[0] + b_xP[1] * xP[1]) + b_xP[2] * xP[2];
    xP_tmp = 0.0;
    d = model_FieldsOfView[i].AzimuthLimits[0];
    d1 = model_FieldsOfView[i].AzimuthLimits[1];
    d2 = model_FieldsOfView[i].ElevationLimits[0];
    d3 = model_FieldsOfView[i].ElevationLimits[1];
    d4 = model_FieldsOfView[i].RangeLimits[0];
    d5 = model_FieldsOfView[i].RangeLimits[1];
    d6 = model_FieldsOfView[i].RangeRateLimits[0];
    d7 = model_FieldsOfView[i].RangeRateLimits[1];
    if ((azimuth >= d) && (azimuth <= d1) &&
        ((elevation >= d2) && (elevation <= d3)) &&
        ((Pd >= d4) && (Pd <= d5)) && ((rr >= d6) && (rr <= d7))) {
      xP_tmp = model_FieldsOfView[i].DetectionProbability;
    }
    if ((!(azimuth >= d)) || (!(azimuth <= d1)) ||
        ((!(elevation >= d2)) || (!(elevation <= d3))) ||
        ((!(Pd >= d4)) || (!(Pd <= d5))) || ((!(rr >= d6)) || (!(rr <= d7)))) {
      xP_tmp = model_FieldsOfView[i].MinDetectionProbability;
    }
    Pmiss *= 1.0 - xP_tmp;
  }
  return 1.0 - Pmiss;
}

/* End of code generation (CompositeFieldOfViewModel.c) */
