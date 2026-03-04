/*
 * stateToMeasurementWrapped.c
 *
 * Code generation for function 'stateToMeasurementWrapped'
 *
 */

/* Include files */
#include "stateToMeasurementWrapped.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo iab_emlrtRSI = {
    11,                          /* lineNo */
    "stateToMeasurementWrapped", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "stateToMeasurementWrapped.m" /* pathName */
};

static emlrtRSInfo jab_emlrtRSI = {
    15,                   /* lineNo */
    "stateToMeasurement", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "stateToMeasurement.m" /* pathName */
};

static emlrtRSInfo mab_emlrtRSI = {
    121,                                                  /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/measurement", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+measurement/"
    "AzimuthElevationRangeAndRangeRateModel.m" /* pathName */
};

/* Function Definitions */
void stateToMeasurementWrapped(const emlrtStack *sp, const real_T x[6],
                               const real_T measurementModel_OriginPosition[9],
                               const real_T measurementModel_OriginVelocity[9],
                               const real_T measurementModel_Orientation[27],
                               real_T z[4])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_xP[3];
  real_T rVec[3];
  real_T xP[3];
  real_T b_xP_tmp;
  real_T c_xP_tmp;
  real_T xP_tmp;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &iab_emlrtRSI;
  b_st.site = &jab_emlrtRSI;
  xP[0] = x[0];
  b_xP[0] = x[1];
  xP[1] = x[2];
  b_xP[1] = x[3];
  xP[2] = x[4];
  b_xP[2] = x[5];
  for (i = 0; i < 3; i++) {
    int32_T b_i;
    int32_T i1;
    r = _mm_loadu_pd(&xP[0]);
    b_i = 3 * (2 - i);
    _mm_storeu_pd(
        &rVec[0],
        _mm_sub_pd(r, _mm_loadu_pd(&measurementModel_OriginPosition[b_i])));
    rVec[2] = xP[2] - measurementModel_OriginPosition[b_i + 2];
    memset(&xP[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&xP[0]);
    i1 = 9 * (2 - i);
    _mm_storeu_pd(
        &xP[0],
        _mm_add_pd(r,
                   _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[i1]),
                              _mm_set1_pd(rVec[0]))));
    xP_tmp = measurementModel_Orientation[i1 + 2];
    xP[2] += xP_tmp * rVec[0];
    rVec[0] = b_xP[0] - measurementModel_OriginVelocity[b_i];
    r = _mm_loadu_pd(&xP[0]);
    _mm_storeu_pd(
        &xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[i1 + 3]),
                          _mm_set1_pd(rVec[1]))));
    b_xP_tmp = measurementModel_Orientation[i1 + 5];
    xP[2] += b_xP_tmp * rVec[1];
    rVec[1] = b_xP[1] - measurementModel_OriginVelocity[b_i + 1];
    r = _mm_loadu_pd(&xP[0]);
    _mm_storeu_pd(
        &xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[i1 + 6]),
                          _mm_set1_pd(rVec[2]))));
    c_xP_tmp = measurementModel_Orientation[i1 + 8];
    xP[2] += c_xP_tmp * rVec[2];
    rVec[2] = b_xP[2] - measurementModel_OriginVelocity[b_i + 2];
    memset(&b_xP[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&b_xP[0]);
    _mm_storeu_pd(
        &b_xP[0],
        _mm_add_pd(r,
                   _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[i1]),
                              _mm_set1_pd(rVec[0]))));
    b_xP[2] += xP_tmp * rVec[0];
    r = _mm_loadu_pd(&b_xP[0]);
    _mm_storeu_pd(
        &b_xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[i1 + 3]),
                          _mm_set1_pd(rVec[1]))));
    b_xP[2] += b_xP_tmp * rVec[1];
    r = _mm_loadu_pd(&b_xP[0]);
    _mm_storeu_pd(
        &b_xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[i1 + 6]),
                          _mm_set1_pd(rVec[2]))));
    b_xP[2] += c_xP_tmp * rVec[2];
  }
  r = _mm_loadu_pd(&xP[0]);
  _mm_storeu_pd(&rVec[0], _mm_mul_pd(r, r));
  xP_tmp = xP[2] * xP[2];
  rVec[2] = xP_tmp;
  b_xP_tmp = d_sumColumnB(rVec);
  c_st.site = &mab_emlrtRSI;
  if (b_xP_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  b_xP_tmp = muDoubleScalarSqrt(b_xP_tmp);
  r = _mm_loadu_pd(&xP[0]);
  _mm_storeu_pd(&rVec[0], _mm_div_pd(r, _mm_set1_pd(b_xP_tmp)));
  z[0] = 57.295779513082323 * muDoubleScalarAtan2(xP[1], xP[0]);
  c_xP_tmp = xP[0] * xP[0] + xP[1] * xP[1];
  z[1] = 57.295779513082323 *
         muDoubleScalarAtan2(xP[2], muDoubleScalarSqrt(c_xP_tmp));
  z[2] = muDoubleScalarSqrt(c_xP_tmp + xP_tmp);
  z[3] = (b_xP[0] * rVec[0] + b_xP[1] * rVec[1]) + b_xP[2] * (xP[2] / b_xP_tmp);
}

/* End of code generation (stateToMeasurementWrapped.c) */
