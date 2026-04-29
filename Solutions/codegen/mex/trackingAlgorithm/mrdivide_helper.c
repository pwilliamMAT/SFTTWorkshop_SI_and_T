/*
 * mrdivide_helper.c
 *
 * Code generation for function 'mrdivide_helper'
 *
 */

/* Include files */
#include "mrdivide_helper.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "warning.h"
#include "xzgetrf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo jab_emlrtRSI = {
    59,      /* lineNo */
    "xtrsm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xtrsm."
    "m" /* pathName */
};

static emlrtRSInfo kab_emlrtRSI = {
    143,     /* lineNo */
    "xtrsm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xtrsm.m" /* pathName */
};

/* Function Definitions */
void b_mrdiv(const emlrtStack *sp, real_T A[36], const real_T B[36])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack st;
  real_T b_A[36];
  real_T smax;
  int32_T b_j;
  int32_T b_kBcol;
  int32_T ijA;
  int32_T info;
  int32_T j;
  int32_T jA;
  int32_T jBcol;
  int32_T k;
  int32_T kBcol;
  int8_T ipiv[6];
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  st.site = &xy_emlrtRSI;
  b_st.site = &yy_emlrtRSI;
  c_st.site = &aab_emlrtRSI;
  d_st.site = &cab_emlrtRSI;
  memcpy(&b_A[0], &B[0], 36U * sizeof(real_T));
  e_st.site = &eab_emlrtRSI;
  for (k = 0; k < 6; k++) {
    ipiv[k] = (int8_T)(k + 1);
  }
  info = 0;
  for (j = 0; j < 5; j++) {
    int32_T b;
    int32_T jj;
    int32_T jp1j;
    int32_T mmj;
    mmj = 4 - j;
    b = j * 7;
    jj = j * 7;
    jp1j = b + 2;
    jA = 7 - j;
    f_st.site = &fab_emlrtRSI;
    g_st.site = &fh_emlrtRSI;
    kBcol = 0;
    smax = muDoubleScalarAbs(b_A[jj]);
    h_st.site = &gh_emlrtRSI;
    for (k = 2; k < jA; k++) {
      real_T s;
      s = muDoubleScalarAbs(b_A[(b + k) - 1]);
      if (s > smax) {
        kBcol = k - 1;
        smax = s;
      }
    }
    if (b_A[jj + kBcol] != 0.0) {
      if (kBcol != 0) {
        kBcol += j;
        ipiv[j] = (int8_T)(kBcol + 1);
        for (k = 0; k < 6; k++) {
          b_kBcol = j + k * 6;
          smax = b_A[b_kBcol];
          jBcol = kBcol + k * 6;
          b_A[b_kBcol] = b_A[jBcol];
          b_A[jBcol] = smax;
        }
      }
      jA = (jj - j) + 6;
      f_st.site = &gab_emlrtRSI;
      for (k = jp1j; k <= jA; k++) {
        b_A[k - 1] /= b_A[jj];
      }
    } else {
      info = j + 1;
    }
    f_st.site = &hab_emlrtRSI;
    g_st.site = &iab_emlrtRSI;
    h_st.site = &cf_emlrtRSI;
    i_st.site = &df_emlrtRSI;
    jA = jj + 8;
    j_st.site = &ef_emlrtRSI;
    for (b_j = 0; b_j <= mmj; b_j++) {
      smax = b_A[(b + b_j * 6) + 6];
      if (smax != 0.0) {
        kBcol = (jA - j) + 4;
        j_st.site = &ff_emlrtRSI;
        if ((jA <= kBcol) && (kBcol > 2147483646)) {
          k_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&k_st);
        }
        for (ijA = jA; ijA <= kBcol; ijA++) {
          b_A[ijA - 1] += b_A[((jj + ijA) - jA) + 1] * -smax;
        }
      }
      jA += 6;
    }
  }
  if ((info == 0) && (!(b_A[35] != 0.0))) {
    info = 6;
  }
  d_st.site = &dab_emlrtRSI;
  e_st.site = &jab_emlrtRSI;
  for (ijA = 0; ijA < 6; ijA++) {
    jBcol = 6 * ijA - 1;
    jA = 6 * ijA;
    f_st.site = &kab_emlrtRSI;
    for (b_j = 0; b_j < ijA; b_j++) {
      kBcol = 6 * b_j;
      smax = b_A[b_j + jA];
      if (smax != 0.0) {
        for (k = 0; k < 6; k++) {
          b_kBcol = (k + jBcol) + 1;
          A[b_kBcol] -= smax * A[k + kBcol];
        }
      }
    }
    smax = 1.0 / b_A[ijA + jA];
    for (k = 0; k <= 4; k += 2) {
      __m128d r;
      jA = (k + jBcol) + 1;
      r = _mm_loadu_pd(&A[jA]);
      r = _mm_mul_pd(_mm_set1_pd(smax), r);
      _mm_storeu_pd(&A[jA], r);
    }
  }
  for (k = 5; k >= 0; k--) {
    jA = 6 * k - 1;
    kBcol = k + 2;
    for (b_j = kBcol; b_j < 7; b_j++) {
      b_kBcol = 6 * (b_j - 1);
      smax = b_A[b_j + jA];
      if (smax != 0.0) {
        for (ijA = 0; ijA < 6; ijA++) {
          jBcol = (ijA + jA) + 1;
          A[jBcol] -= smax * A[ijA + b_kBcol];
        }
      }
    }
  }
  for (k = 4; k >= 0; k--) {
    int8_T i;
    i = ipiv[k];
    if (i != k + 1) {
      for (b_j = 0; b_j < 6; b_j++) {
        jA = b_j + 6 * k;
        smax = A[jA];
        kBcol = b_j + 6 * (i - 1);
        A[jA] = A[kBcol];
        A[kBcol] = smax;
      }
    }
  }
  if (info > 0) {
    c_st.site = &bab_emlrtRSI;
    if (!emlrtSetWarningFlag(&c_st)) {
      d_st.site = &lab_emlrtRSI;
      c_warning(&d_st);
    }
  }
}

void mrdiv(const emlrtStack *sp, real_T A[4], const real_T B[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b_A[16];
  real_T temp;
  int32_T ipiv[4];
  int32_T info;
  int32_T j;
  int32_T jAcol;
  int32_T k;
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
  st.site = &xy_emlrtRSI;
  b_st.site = &yy_emlrtRSI;
  c_st.site = &aab_emlrtRSI;
  d_st.site = &cab_emlrtRSI;
  memcpy(&b_A[0], &B[0], 16U * sizeof(real_T));
  e_st.site = &eab_emlrtRSI;
  info = xzgetrf(&e_st, b_A, ipiv);
  d_st.site = &dab_emlrtRSI;
  for (j = 0; j < 4; j++) {
    jAcol = j << 2;
    for (k = 0; k < j; k++) {
      temp = b_A[k + jAcol];
      if (temp != 0.0) {
        A[j] -= temp * A[k];
      }
    }
    A[j] *= 1.0 / b_A[j + jAcol];
  }
  for (j = 3; j >= 0; j--) {
    int32_T i;
    jAcol = (j << 2) - 1;
    i = j + 2;
    for (k = i; k < 5; k++) {
      temp = b_A[k + jAcol];
      if (temp != 0.0) {
        A[j] -= temp * A[k - 1];
      }
    }
  }
  if (ipiv[2] != 3) {
    temp = A[2];
    A[2] = A[ipiv[2] - 1];
    A[ipiv[2] - 1] = temp;
  }
  if (ipiv[1] != 2) {
    temp = A[1];
    A[1] = A[ipiv[1] - 1];
    A[ipiv[1] - 1] = temp;
  }
  if (ipiv[0] != 1) {
    temp = A[0];
    A[0] = A[ipiv[0] - 1];
    A[ipiv[0] - 1] = temp;
  }
  if (info > 0) {
    c_st.site = &bab_emlrtRSI;
    if (!emlrtSetWarningFlag(&c_st)) {
      d_st.site = &lab_emlrtRSI;
      c_warning(&d_st);
    }
  }
}

/* End of code generation (mrdivide_helper.c) */
