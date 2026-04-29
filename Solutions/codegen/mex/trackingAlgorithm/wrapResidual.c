/*
 * wrapResidual.c
 *
 * Code generation for function 'wrapResidual'
 *
 */

/* Include files */
#include "wrapResidual.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtECInfo fb_emlrtECI = {
    -1,             /* nDims */
    50,             /* lineNo */
    1,              /* colNo */
    "wrapResidual", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\wrapResidual.m" /* pName */
};

/* Function Definitions */
void wrapResidual(const emlrtStack *sp, real_T residual[4])
{
  static real_T bounds[8] = {-180.0, -90.0, 0.0, 0.0, 180.0, 90.0, 0.0, 0.0};
  __m128d r;
  __m128d r1;
  real_T b_bounds[8];
  real_T b_tmp_data[4];
  real_T resToWrap_data[4];
  real_T y_data[4];
  real_T dv[2];
  real_T dv1[2];
  int32_T i;
  int32_T partialTrueCount;
  int32_T trueCount;
  int8_T tmp_data[4];
  bounds[2] = rtMinusInf;
  bounds[3] = rtMinusInf;
  bounds[6] = rtInf;
  bounds[7] = rtInf;
  memcpy(&b_bounds[0], &bounds[0], 8U * sizeof(real_T));
  b_bounds[0] = -180.0;
  b_bounds[1] = -90.0;
  b_bounds[4] = 180.0;
  b_bounds[5] = 90.0;
  dv[0] = muDoubleScalarAbs(residual[0]);
  dv[1] = muDoubleScalarAbs(residual[1]);
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&b_bounds[4]);
  _mm_storeu_pd(&dv1[0], _mm_div_pd(r, r1));
  dv[0] = muDoubleScalarAbs(dv1[0]);
  dv[1] = muDoubleScalarAbs(dv1[1]);
  r = _mm_loadu_pd(&dv[0]);
  _mm_storeu_pd(&resToWrap_data[0], r);
  dv[0] = muDoubleScalarAbs(residual[2]);
  dv[1] = muDoubleScalarAbs(residual[3]);
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&b_bounds[6]);
  _mm_storeu_pd(&dv1[0], _mm_div_pd(r, r1));
  dv[0] = muDoubleScalarAbs(dv1[0]);
  dv[1] = muDoubleScalarAbs(dv1[1]);
  r = _mm_loadu_pd(&dv[0]);
  _mm_storeu_pd(&resToWrap_data[2], r);
  trueCount = 0;
  if (resToWrap_data[0] > 0.001) {
    trueCount = 1;
  }
  if (resToWrap_data[1] > 0.001) {
    trueCount++;
  }
  partialTrueCount = 0;
  if (resToWrap_data[0] > 0.001) {
    tmp_data[0] = 0;
    partialTrueCount = 1;
  }
  if (resToWrap_data[1] > 0.001) {
    tmp_data[partialTrueCount] = 1;
  }
  for (i = 0; i < trueCount; i++) {
    int8_T i1;
    i1 = tmp_data[i];
    partialTrueCount = (int32_T)b_bounds[i1];
    resToWrap_data[i] = residual[i1] - (real_T)partialTrueCount;
    y_data[i] = b_bounds[i1 + 4] - (real_T)partialTrueCount;
  }
  for (i = 0; i < trueCount; i++) {
    b_tmp_data[i] = b_mod(resToWrap_data[i], (int32_T)y_data[i]);
  }
  for (i = 0; i < trueCount; i++) {
    resToWrap_data[i] = b_tmp_data[i] + b_bounds[tmp_data[i]];
  }
  emlrtSubAssignSizeCheckR2012b(&trueCount, 1, &trueCount, 1, &fb_emlrtECI,
                                (emlrtCTX)sp);
  for (i = 0; i < trueCount; i++) {
    residual[tmp_data[i]] = resToWrap_data[i];
  }
}

/* End of code generation (wrapResidual.c) */
