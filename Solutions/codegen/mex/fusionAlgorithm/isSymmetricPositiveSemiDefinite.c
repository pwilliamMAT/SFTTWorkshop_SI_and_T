/*
 * isSymmetricPositiveSemiDefinite.c
 *
 * Code generation for function 'isSymmetricPositiveSemiDefinite'
 *
 */

/* Include files */
#include "isSymmetricPositiveSemiDefinite.h"
#include "all.h"
#include "eig.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>

/* Function Definitions */
void isSymmetricPositiveSemiDefinite(const emlrtStack *sp,
                                     const real_T b_value[36])
{
  __m128d r;
  emlrtStack st;
  creal_T d[6];
  real_T notSymmetric_tmp[36];
  real_T y[36];
  real_T varargin_1[6];
  real_T dv[2];
  real_T dv1[2];
  real_T absx;
  real_T x;
  int32_T exponent;
  int32_T i;
  int32_T idx;
  int32_T k;
  boolean_T b_y[36];
  boolean_T b_x[6];
  boolean_T c_y;
  boolean_T exitg1;
  boolean_T notPositiveSemidefinite;
  st.prev = sp;
  st.tls = sp->tls;
  for (k = 0; k < 6; k++) {
    varargin_1[k] = muDoubleScalarAbs(b_value[k + 6 * k]);
  }
  if (!muDoubleScalarIsNaN(varargin_1[0])) {
    idx = 1;
  } else {
    int32_T b_k;
    idx = 0;
    b_k = 2;
    exitg1 = false;
    while ((!exitg1) && (b_k < 7)) {
      if (!muDoubleScalarIsNaN(varargin_1[b_k - 1])) {
        idx = b_k;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }
  if (idx == 0) {
    absx = varargin_1[0];
  } else {
    absx = varargin_1[idx - 1];
    idx++;
    for (k = idx; k < 7; k++) {
      x = varargin_1[k - 1];
      if (absx < x) {
        absx = x;
      }
    }
  }
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    absx = rtNaN;
  } else if (absx < 4.4501477170144028E-308) {
    absx = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    absx = ldexp(1.0, exponent - 53);
  }
  absx *= 100.0;
  for (k = 0; k < 6; k++) {
    for (i = 0; i < 6; i++) {
      notSymmetric_tmp[i + 6 * k] = b_value[k + 6 * i];
    }
  }
  for (k = 0; k <= 34; k += 2) {
    r = _mm_loadu_pd(&notSymmetric_tmp[k]);
    _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[k]), r));
    dv1[0] = muDoubleScalarAbs(dv[0]);
    dv1[1] = muDoubleScalarAbs(dv[1]);
    r = _mm_loadu_pd(&dv1[0]);
    _mm_storeu_pd(&y[k], r);
  }
  x = muDoubleScalarSqrt(absx);
  for (k = 0; k < 36; k++) {
    b_y[k] = (y[k] < x);
  }
  st.site = &vc_emlrtRSI;
  all(&st, b_y, b_x);
  c_y = true;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx < 6)) {
    if (!b_x[idx]) {
      c_y = false;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  for (k = 0; k <= 34; k += 2) {
    r = _mm_loadu_pd(&notSymmetric_tmp[k]);
    _mm_storeu_pd(
        &notSymmetric_tmp[k],
        _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[k]), r), _mm_set1_pd(2.0)));
  }
  st.site = &wc_emlrtRSI;
  eig(&st, notSymmetric_tmp, d);
  for (k = 0; k < 6; k++) {
    b_x[k] = (d[k].re < -absx);
  }
  notPositiveSemidefinite = false;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx < 6)) {
    if (b_x[idx]) {
      notPositiveSemidefinite = true;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  if (notPositiveSemidefinite || (!c_y)) {
    emlrtErrorWithMessageIdR2018a(
        sp, &r_emlrtRTEI,
        "shared_tracking:KalmanFilter:invalidCovarianceValues",
        "shared_tracking:KalmanFilter:invalidCovarianceValues", 3, 4, 15,
        "StateCovariance");
  }
}

/* End of code generation (isSymmetricPositiveSemiDefinite.c) */
