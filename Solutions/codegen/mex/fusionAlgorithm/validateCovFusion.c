/*
 * validateCovFusion.c
 *
 * Code generation for function 'validateCovFusion'
 *
 */

/* Include files */
#include "validateCovFusion.h"
#include "all.h"
#include "eig.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>

/* Variable Definitions */
static emlrtRSInfo mr_emlrtRSI = {
    15,                  /* lineNo */
    "validateCovFusion", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\validateCovFusion.m" /* pathName */
};

static emlrtRSInfo nr_emlrtRSI = {
    16,                  /* lineNo */
    "validateCovFusion", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\validateCovFusion.m" /* pathName */
};

static emlrtRSInfo or_emlrtRSI = {
    24,                  /* lineNo */
    "validateCovFusion", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\validateCovFusion.m" /* pathName */
};

static emlrtBCInfo pg_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    24,                  /* lineNo */
    86,                  /* colNo */
    "",                  /* aName */
    "validateCovFusion", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\validateCovFusion.m", /* pName */
    0                                /* checkKind */
};

static emlrtRTEInfo bc_emlrtRTEI = {
    21,                  /* lineNo */
    41,                  /* colNo */
    "validateCovFusion", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\validateCovFusion.m" /* pName */
};

/* Function Definitions */
void validateCovFusion(const emlrtStack *sp, const emxArray_real_T *x,
                       const emxArray_real_T *p)
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack st;
  real_T notSymmetric_tmp[36];
  const real_T *p_data;
  const real_T *x_data;
  int32_T b_i;
  int32_T exponent;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T k;
  int32_T notSymmetric_tmp_tmp;
  boolean_T b_p;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  p_data = p->data;
  x_data = x->data;
  st.site = &mr_emlrtRSI;
  b_st.site = &gb_emlrtRSI;
  b_p = true;
  notSymmetric_tmp_tmp = 6 * x->size[1];
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= notSymmetric_tmp_tmp - 1)) {
    if ((!muDoubleScalarIsInf(x_data[idx])) &&
        (!muDoubleScalarIsNaN(x_data[idx]))) {
      idx++;
    } else {
      b_p = false;
      exitg1 = true;
    }
  }
  if (!b_p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:fusecovint:expectedFinite", 3, 4, 27,
        "input number 1, trackState,");
  }
  st.site = &nr_emlrtRSI;
  b_st.site = &gb_emlrtRSI;
  b_p = true;
  notSymmetric_tmp_tmp = 36 * p->size[2];
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= notSymmetric_tmp_tmp - 1)) {
    if ((!muDoubleScalarIsInf(p_data[idx])) &&
        (!muDoubleScalarIsNaN(p_data[idx]))) {
      idx++;
    } else {
      b_p = false;
      exitg1 = true;
    }
  }
  if (!b_p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:fusecovint:expectedFinite", 3, 4, 25,
        "input number 2, trackCov,");
  }
  if (x->size[1] != p->size[2]) {
    emlrtErrorWithMessageIdR2018a(sp, &bc_emlrtRTEI,
                                  "fusion:covFusion:dataInconsistent",
                                  "fusion:covFusion:dataInconsistent", 6, 4, 10,
                                  "trackState", 4, 8, "trackCov");
  }
  i = p->size[2];
  r = _mm_set1_pd(2.0);
  for (b_i = 0; b_i < i; b_i++) {
    creal_T d[6];
    real_T b_x[36];
    real_T y[36];
    real_T varargin_1[6];
    real_T absx;
    real_T tol;
    boolean_T b_y[36];
    boolean_T c_x[6];
    boolean_T notPositiveSemidefinite;
    st.site = &or_emlrtRSI;
    if (b_i + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, i, &pg_emlrtBCI, &st);
    }
    for (k = 0; k < 6; k++) {
      varargin_1[k] = muDoubleScalarAbs(p_data[(k + 6 * k) + 36 * b_i]);
    }
    if (!muDoubleScalarIsNaN(varargin_1[0])) {
      idx = 1;
    } else {
      idx = 0;
      notSymmetric_tmp_tmp = 2;
      exitg1 = false;
      while ((!exitg1) && (notSymmetric_tmp_tmp < 7)) {
        if (!muDoubleScalarIsNaN(varargin_1[notSymmetric_tmp_tmp - 1])) {
          idx = notSymmetric_tmp_tmp;
          exitg1 = true;
        } else {
          notSymmetric_tmp_tmp++;
        }
      }
    }
    if (idx == 0) {
      absx = varargin_1[0];
    } else {
      absx = varargin_1[idx - 1];
      notSymmetric_tmp_tmp = idx + 1;
      for (k = notSymmetric_tmp_tmp; k < 7; k++) {
        tol = varargin_1[k - 1];
        if (absx < tol) {
          absx = tol;
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
    tol = 100.0 * absx;
    for (k = 0; k < 6; k++) {
      for (i1 = 0; i1 < 6; i1++) {
        absx = p_data[(k + 6 * i1) + 36 * b_i];
        notSymmetric_tmp_tmp = i1 + 6 * k;
        notSymmetric_tmp[notSymmetric_tmp_tmp] = absx;
        b_x[notSymmetric_tmp_tmp] =
            p_data[notSymmetric_tmp_tmp + 36 * b_i] - absx;
      }
    }
    for (k = 0; k < 36; k++) {
      y[k] = muDoubleScalarAbs(b_x[k]);
    }
    absx = muDoubleScalarSqrt(tol);
    for (k = 0; k < 36; k++) {
      b_y[k] = (y[k] < absx);
    }
    b_st.site = &vc_emlrtRSI;
    all(&b_st, b_y, c_x);
    b_p = true;
    notSymmetric_tmp_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (notSymmetric_tmp_tmp < 6)) {
      if (!c_x[notSymmetric_tmp_tmp]) {
        b_p = false;
        exitg1 = true;
      } else {
        notSymmetric_tmp_tmp++;
      }
    }
    for (k = 0; k < 6; k++) {
      __m128d r1;
      r1 = _mm_loadu_pd(&notSymmetric_tmp[6 * k]);
      notSymmetric_tmp_tmp = 6 * k + 36 * b_i;
      _mm_storeu_pd(
          &notSymmetric_tmp[6 * k],
          _mm_div_pd(
              _mm_add_pd(_mm_loadu_pd(&p_data[notSymmetric_tmp_tmp]), r1), r));
      idx = 6 * k + 2;
      r1 = _mm_loadu_pd(&notSymmetric_tmp[idx]);
      _mm_storeu_pd(
          &notSymmetric_tmp[idx],
          _mm_div_pd(
              _mm_add_pd(_mm_loadu_pd(&p_data[notSymmetric_tmp_tmp + 2]), r1),
              r));
      idx = 6 * k + 4;
      r1 = _mm_loadu_pd(&notSymmetric_tmp[idx]);
      _mm_storeu_pd(
          &notSymmetric_tmp[idx],
          _mm_div_pd(
              _mm_add_pd(_mm_loadu_pd(&p_data[notSymmetric_tmp_tmp + 4]), r1),
              r));
    }
    b_st.site = &wc_emlrtRSI;
    eig(&b_st, notSymmetric_tmp, d);
    for (k = 0; k < 6; k++) {
      c_x[k] = (d[k].re < -tol);
    }
    notPositiveSemidefinite = false;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx < 6)) {
      if (c_x[idx]) {
        notPositiveSemidefinite = true;
        exitg1 = true;
      } else {
        idx++;
      }
    }
    if (notPositiveSemidefinite || (!b_p)) {
      emlrtErrorWithMessageIdR2018a(
          &st, &r_emlrtRTEI,
          "shared_tracking:KalmanFilter:invalidCovarianceValues",
          "shared_tracking:KalmanFilter:invalidCovarianceValues", 3, 4, 8,
          "trackCov");
    }
  }
}

/* End of code generation (validateCovFusion.c) */
