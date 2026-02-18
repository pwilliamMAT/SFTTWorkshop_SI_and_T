/*
 * stateToMeasurementJacobian.c
 *
 * Code generation for function 'stateToMeasurementJacobian'
 *
 */

/* Include files */
#include "stateToMeasurementJacobian.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo qab_emlrtRSI = {
    13,                           /* lineNo */
    "stateToMeasurementJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\stateToMeasurementJacobi"
    "an.m" /* pathName */
};

static emlrtRSInfo rab_emlrtRSI = {
    135,                                                     /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/measurementjac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo sab_emlrtRSI = {
    136,                                                     /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/measurementjac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo tab_emlrtRSI = {
    138,                                                     /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/measurementjac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo uab_emlrtRSI = {
    139,                                                     /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/measurementjac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo vab_emlrtRSI = {
    140,                                                     /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/measurementjac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo abb_emlrtRSI = {
    79,             /* lineNo */
    "rangeratejac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\rangerate"
    "jac.m" /* pathName */
};

static emlrtRTEInfo cb_emlrtRTEI = {
    13,               /* lineNo */
    13,               /* colNo */
    "toLogicalCheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\toLogicalCheck.m" /* pName */
};

/* Function Definitions */
void stateToMeasurementJacobian(const emlrtStack *sp, const real_T x[6],
                                const real_T measurementModel_OriginPosition[9],
                                const real_T measurementModel_OriginVelocity[9],
                                const real_T measurementModel_Orientation[27],
                                real_T H[24])
{
  static const int8_T b_posJac3D[18] = {1, 0, 0, 0, 1, 0, 0, 0, 1,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int8_T b_velJac3D[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const int8_T b_iv[9] = {0, 1, 0, 0, 1, 0, 0, 1, 0};
  static const int8_T b_iv1[6] = {0, 1, 3, 4, 6, 7};
  __m128d r;
  __m128d r1;
  __m128d r2;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T d_posJac3D[36];
  real_T dv[24];
  real_T dv1[24];
  real_T b_measurementModel_Orientation[18];
  real_T c_posJac3D[18];
  real_T posJac3D[18];
  real_T velJac3D[18];
  real_T jacobianRangeRateRow[9];
  real_T state3DPoint[6];
  real_T xE[6];
  real_T b_xP[3];
  real_T c_xP[3];
  real_T xP[3];
  real_T absxk;
  real_T b_absxk;
  real_T b_scale;
  real_T b_x;
  real_T c_x;
  real_T relvelnorm;
  real_T rnorm;
  real_T scale;
  real_T t;
  real_T xysq;
  real_T xyzsq;
  int32_T H1_tmp;
  int32_T c_H1_tmp;
  int32_T c_i;
  int32_T i;
  int8_T H1[36];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  for (i = 0; i < 36; i++) {
    H1[i] = 0;
  }
  memset(&jacobianRangeRateRow[0], 0, 9U * sizeof(real_T));
  jacobianRangeRateRow[0] = 1.0;
  jacobianRangeRateRow[4] = 1.0;
  jacobianRangeRateRow[8] = 1.0;
  for (i = 0; i < 3; i++) {
    int32_T b_H1_tmp;
    int32_T b_i;
    b_i = (int32_T)jacobianRangeRateRow[3 * i];
    H1_tmp = i << 1;
    b_H1_tmp = 6 * H1_tmp;
    H1[b_H1_tmp] = (int8_T)b_i;
    c_H1_tmp = 6 * (H1_tmp + 1);
    H1[c_H1_tmp + 3] = (int8_T)b_i;
    b_i = (int32_T)jacobianRangeRateRow[3 * i + 1];
    H1[b_H1_tmp + 1] = (int8_T)b_i;
    H1[c_H1_tmp + 4] = (int8_T)b_i;
    b_i = (int32_T)jacobianRangeRateRow[3 * i + 2];
    H1[b_H1_tmp + 2] = (int8_T)b_i;
    H1[c_H1_tmp + 5] = (int8_T)b_i;
  }
  st.site = &qab_emlrtRSI;
  xP[0] = x[0];
  b_xP[0] = x[1];
  xP[1] = x[2];
  b_xP[1] = x[3];
  xP[2] = x[4];
  b_xP[2] = x[5];
  for (i = 0; i < 3; i++) {
    r = _mm_loadu_pd(&xP[0]);
    H1_tmp = 3 * (2 - i);
    _mm_storeu_pd(
        &c_xP[0],
        _mm_sub_pd(r, _mm_loadu_pd(&measurementModel_OriginPosition[H1_tmp])));
    c_xP[2] = xP[2] - measurementModel_OriginPosition[H1_tmp + 2];
    memset(&xP[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&xP[0]);
    c_H1_tmp = 9 * (2 - i);
    _mm_storeu_pd(
        &xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[c_H1_tmp]),
                          _mm_set1_pd(c_xP[0]))));
    absxk = measurementModel_Orientation[c_H1_tmp + 2];
    xP[2] += absxk * c_xP[0];
    c_xP[0] = b_xP[0] - measurementModel_OriginVelocity[H1_tmp];
    r = _mm_loadu_pd(&xP[0]);
    _mm_storeu_pd(
        &xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(
                   _mm_loadu_pd(&measurementModel_Orientation[c_H1_tmp + 3]),
                   _mm_set1_pd(c_xP[1]))));
    b_absxk = measurementModel_Orientation[c_H1_tmp + 5];
    xP[2] += b_absxk * c_xP[1];
    c_xP[1] = b_xP[1] - measurementModel_OriginVelocity[H1_tmp + 1];
    r = _mm_loadu_pd(&xP[0]);
    _mm_storeu_pd(
        &xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(
                   _mm_loadu_pd(&measurementModel_Orientation[c_H1_tmp + 6]),
                   _mm_set1_pd(c_xP[2]))));
    t = measurementModel_Orientation[c_H1_tmp + 8];
    xP[2] += t * c_xP[2];
    c_xP[2] = b_xP[2] - measurementModel_OriginVelocity[H1_tmp + 2];
    memset(&b_xP[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&b_xP[0]);
    _mm_storeu_pd(
        &b_xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[c_H1_tmp]),
                          _mm_set1_pd(c_xP[0]))));
    b_xP[2] += absxk * c_xP[0];
    r = _mm_loadu_pd(&b_xP[0]);
    _mm_storeu_pd(
        &b_xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(
                   _mm_loadu_pd(&measurementModel_Orientation[c_H1_tmp + 3]),
                   _mm_set1_pd(c_xP[1]))));
    b_xP[2] += b_absxk * c_xP[1];
    r = _mm_loadu_pd(&b_xP[0]);
    _mm_storeu_pd(
        &b_xP[0],
        _mm_add_pd(
            r, _mm_mul_pd(
                   _mm_loadu_pd(&measurementModel_Orientation[c_H1_tmp + 6]),
                   _mm_set1_pd(c_xP[2]))));
    b_xP[2] += t * c_xP[2];
  }
  for (i = 0; i < 18; i++) {
    posJac3D[i] = b_posJac3D[i];
    velJac3D[i] = b_velJac3D[i];
  }
  for (c_i = 0; c_i < 3; c_i++) {
    memset(&b_measurementModel_Orientation[0], 0, 18U * sizeof(real_T));
    H1_tmp = 9 * (2 - c_i);
    for (i = 0; i < 6; i++) {
      absxk = posJac3D[3 * i];
      r = _mm_loadu_pd(&b_measurementModel_Orientation[3 * i]);
      _mm_storeu_pd(
          &b_measurementModel_Orientation[3 * i],
          _mm_add_pd(
              r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[H1_tmp]),
                            _mm_set1_pd(absxk))));
      c_H1_tmp = 3 * i + 2;
      b_measurementModel_Orientation[c_H1_tmp] +=
          measurementModel_Orientation[H1_tmp + 2] * absxk;
      absxk = posJac3D[3 * i + 1];
      r = _mm_loadu_pd(&b_measurementModel_Orientation[3 * i]);
      _mm_storeu_pd(
          &b_measurementModel_Orientation[3 * i],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&measurementModel_Orientation[H1_tmp + 3]),
                     _mm_set1_pd(absxk))));
      b_measurementModel_Orientation[c_H1_tmp] +=
          measurementModel_Orientation[H1_tmp + 5] * absxk;
      absxk = posJac3D[c_H1_tmp];
      r = _mm_loadu_pd(&b_measurementModel_Orientation[3 * i]);
      _mm_storeu_pd(
          &b_measurementModel_Orientation[3 * i],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&measurementModel_Orientation[H1_tmp + 6]),
                     _mm_set1_pd(absxk))));
      b_measurementModel_Orientation[c_H1_tmp] +=
          measurementModel_Orientation[H1_tmp + 8] * absxk;
    }
    memcpy(&posJac3D[0], &b_measurementModel_Orientation[0],
           18U * sizeof(real_T));
    memset(&b_measurementModel_Orientation[0], 0, 18U * sizeof(real_T));
    H1_tmp = 9 * (2 - c_i);
    for (i = 0; i < 6; i++) {
      absxk = velJac3D[3 * i];
      r = _mm_loadu_pd(&b_measurementModel_Orientation[3 * i]);
      _mm_storeu_pd(
          &b_measurementModel_Orientation[3 * i],
          _mm_add_pd(
              r, _mm_mul_pd(_mm_loadu_pd(&measurementModel_Orientation[H1_tmp]),
                            _mm_set1_pd(absxk))));
      c_H1_tmp = 3 * i + 2;
      b_measurementModel_Orientation[c_H1_tmp] +=
          measurementModel_Orientation[H1_tmp + 2] * absxk;
      absxk = velJac3D[3 * i + 1];
      r = _mm_loadu_pd(&b_measurementModel_Orientation[3 * i]);
      _mm_storeu_pd(
          &b_measurementModel_Orientation[3 * i],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&measurementModel_Orientation[H1_tmp + 3]),
                     _mm_set1_pd(absxk))));
      b_measurementModel_Orientation[c_H1_tmp] +=
          measurementModel_Orientation[H1_tmp + 5] * absxk;
      absxk = velJac3D[c_H1_tmp];
      r = _mm_loadu_pd(&b_measurementModel_Orientation[3 * i]);
      _mm_storeu_pd(
          &b_measurementModel_Orientation[3 * i],
          _mm_add_pd(
              r, _mm_mul_pd(
                     _mm_loadu_pd(&measurementModel_Orientation[H1_tmp + 6]),
                     _mm_set1_pd(absxk))));
      b_measurementModel_Orientation[c_H1_tmp] +=
          measurementModel_Orientation[H1_tmp + 8] * absxk;
    }
    memcpy(&velJac3D[0], &b_measurementModel_Orientation[0],
           18U * sizeof(real_T));
  }
  b_st.site = &rab_emlrtRSI;
  b_st.site = &rab_emlrtRSI;
  xysq = xP[0] * xP[0] + xP[1] * xP[1];
  b_st.site = &sab_emlrtRSI;
  b_st.site = &sab_emlrtRSI;
  b_st.site = &sab_emlrtRSI;
  xyzsq = xysq + xP[2] * xP[2];
  b_st.site = &tab_emlrtRSI;
  b_x = muDoubleScalarSqrt(xysq);
  b_st.site = &tab_emlrtRSI;
  b_st.site = &tab_emlrtRSI;
  b_st.site = &uab_emlrtRSI;
  c_x = muDoubleScalarSqrt(xyzsq);
  b_st.site = &uab_emlrtRSI;
  b_st.site = &uab_emlrtRSI;
  b_st.site = &vab_emlrtRSI;
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(xP[0]);
  if (absxk > 3.3121686421112381E-170) {
    rnorm = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    rnorm = t * t;
  }
  b_absxk = muDoubleScalarAbs(b_xP[0]);
  if (b_absxk > 3.3121686421112381E-170) {
    relvelnorm = 1.0;
    b_scale = b_absxk;
  } else {
    absxk = b_absxk / 3.3121686421112381E-170;
    relvelnorm = absxk * absxk;
  }
  absxk = muDoubleScalarAbs(xP[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rnorm = rnorm * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    rnorm += t * t;
  }
  b_absxk = muDoubleScalarAbs(b_xP[1]);
  if (b_absxk > b_scale) {
    absxk = b_scale / b_absxk;
    relvelnorm = relvelnorm * absxk * absxk + 1.0;
    b_scale = b_absxk;
  } else {
    absxk = b_absxk / b_scale;
    relvelnorm += absxk * absxk;
  }
  absxk = muDoubleScalarAbs(xP[2]);
  if (absxk > scale) {
    t = scale / absxk;
    rnorm = rnorm * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    rnorm += t * t;
  }
  b_absxk = muDoubleScalarAbs(b_xP[2]);
  if (b_absxk > b_scale) {
    absxk = b_scale / b_absxk;
    relvelnorm = relvelnorm * absxk * absxk + 1.0;
    b_scale = b_absxk;
  } else {
    absxk = b_absxk / b_scale;
    relvelnorm += absxk * absxk;
  }
  rnorm = scale * muDoubleScalarSqrt(rnorm);
  relvelnorm = b_scale * muDoubleScalarSqrt(relvelnorm);
  if (rnorm > 0.0) {
    absxk = (xP[0] * b_xP[0] + xP[1] * b_xP[1]) + xP[2] * b_xP[2];
    b_absxk = rnorm * rnorm;
    jacobianRangeRateRow[0] =
        (b_xP[0] * rnorm - absxk * xP[0] / rnorm) / b_absxk;
    jacobianRangeRateRow[1] = xP[0] / rnorm;
    jacobianRangeRateRow[2] = 0.0;
    jacobianRangeRateRow[3] =
        (b_xP[1] * rnorm - absxk * xP[1] / rnorm) / b_absxk;
    jacobianRangeRateRow[4] = xP[1] / rnorm;
    jacobianRangeRateRow[5] = 0.0;
    jacobianRangeRateRow[6] =
        (b_xP[2] * rnorm - absxk * xP[2] / rnorm) / b_absxk;
    jacobianRangeRateRow[7] = xP[2] / rnorm;
    jacobianRangeRateRow[8] = 0.0;
  } else {
    c_st.site = &abb_emlrtRSI;
    if (muDoubleScalarIsNaN(relvelnorm)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI, "MATLAB:nologicalnan",
                                    "MATLAB:nologicalnan", 0);
    }
    if (relvelnorm != 0.0) {
      jacobianRangeRateRow[0] = 0.0;
      jacobianRangeRateRow[1] = b_xP[0] / relvelnorm;
      jacobianRangeRateRow[2] = 0.0;
      jacobianRangeRateRow[3] = 0.0;
      jacobianRangeRateRow[4] = b_xP[1] / relvelnorm;
      jacobianRangeRateRow[5] = 0.0;
      jacobianRangeRateRow[6] = 0.0;
      jacobianRangeRateRow[7] = b_xP[2] / relvelnorm;
      jacobianRangeRateRow[8] = 0.0;
    } else {
      for (i = 0; i < 9; i++) {
        jacobianRangeRateRow[i] = b_iv[i];
      }
    }
  }
  for (i = 0; i < 6; i++) {
    xE[i] = jacobianRangeRateRow[b_iv1[i]];
  }
  state3DPoint[0] = xE[0];
  state3DPoint[3] = xE[1];
  state3DPoint[1] = xE[2];
  state3DPoint[4] = xE[3];
  state3DPoint[2] = xE[4];
  state3DPoint[5] = xE[5];
  for (i = 0; i < 6; i++) {
    xE[i] = state3DPoint[i];
  }
  memset(&c_posJac3D[0], 0, 18U * sizeof(real_T));
  memset(&b_measurementModel_Orientation[0], 0, 18U * sizeof(real_T));
  dv[0] = 57.295779513082323 * (-xP[1] / xysq);
  dv[4] = 57.295779513082323 * (xP[0] / xysq);
  dv[8] = 0.0;
  dv[12] = 0.0;
  dv[16] = 0.0;
  dv[20] = 0.0;
  dv[1] = 57.295779513082323 * (-xP[0] * xP[2] / b_x / xyzsq);
  dv[5] = 57.295779513082323 * (-xP[1] * xP[2] / b_x / xyzsq);
  dv[9] = 57.295779513082323 * (b_x / xyzsq);
  dv[13] = 0.0;
  dv[17] = 0.0;
  dv[21] = 0.0;
  dv[2] = xP[0] / c_x;
  dv[6] = xP[1] / c_x;
  dv[10] = xP[2] / c_x;
  dv[14] = 0.0;
  dv[18] = 0.0;
  dv[22] = 0.0;
  for (c_i = 0; c_i < 6; c_i++) {
    H1_tmp = 3 * c_i + 2;
    for (i = 0; i < 6; i++) {
      int8_T i1;
      i1 = iv[i + 6 * c_i];
      r = _mm_loadu_pd(&posJac3D[3 * i]);
      r1 = _mm_loadu_pd(&c_posJac3D[3 * c_i]);
      r2 = _mm_set1_pd(i1);
      _mm_storeu_pd(&c_posJac3D[3 * c_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&velJac3D[3 * i]);
      r1 = _mm_loadu_pd(&b_measurementModel_Orientation[3 * c_i]);
      _mm_storeu_pd(&b_measurementModel_Orientation[3 * c_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      c_H1_tmp = 3 * i + 2;
      c_posJac3D[H1_tmp] += posJac3D[c_H1_tmp] * (real_T)i1;
      b_measurementModel_Orientation[H1_tmp] += velJac3D[c_H1_tmp] * (real_T)i1;
    }
    dv[(c_i << 2) + 3] = xE[c_i];
    d_posJac3D[6 * c_i] = c_posJac3D[3 * c_i];
    d_posJac3D[6 * c_i + 3] = b_measurementModel_Orientation[3 * c_i];
    H1_tmp = 3 * c_i + 1;
    d_posJac3D[6 * c_i + 1] = c_posJac3D[H1_tmp];
    d_posJac3D[6 * c_i + 4] = b_measurementModel_Orientation[H1_tmp];
    H1_tmp = 3 * c_i + 2;
    d_posJac3D[6 * c_i + 2] = c_posJac3D[H1_tmp];
    d_posJac3D[6 * c_i + 5] = b_measurementModel_Orientation[H1_tmp];
  }
  memset(&dv1[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    H1_tmp = i << 2;
    for (c_i = 0; c_i < 6; c_i++) {
      c_H1_tmp = c_i << 2;
      r = _mm_loadu_pd(&dv[c_H1_tmp]);
      r1 = _mm_loadu_pd(&dv1[H1_tmp]);
      r2 = _mm_set1_pd(d_posJac3D[c_i + 6 * i]);
      _mm_storeu_pd(&dv1[H1_tmp], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&dv[c_H1_tmp + 2]);
      r1 = _mm_loadu_pd(&dv1[H1_tmp + 2]);
      _mm_storeu_pd(&dv1[H1_tmp + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  memset(&H[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    H1_tmp = i << 2;
    for (c_i = 0; c_i < 6; c_i++) {
      c_H1_tmp = c_i << 2;
      r = _mm_loadu_pd(&dv1[c_H1_tmp]);
      r1 = _mm_loadu_pd(&H[H1_tmp]);
      r2 = _mm_set1_pd(H1[c_i + 6 * i]);
      _mm_storeu_pd(&H[H1_tmp], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&dv1[c_H1_tmp + 2]);
      r1 = _mm_loadu_pd(&H[H1_tmp + 2]);
      _mm_storeu_pd(&H[H1_tmp + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
}

/* End of code generation (stateToMeasurementJacobian.c) */
