/*
 * xzgetrf.c
 *
 * Code generation for function 'xzgetrf'
 *
 */

/* Include files */
#include "xzgetrf.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo vf_emlrtRSI = {
    23,       /* lineNo */
    "ixamax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\ixamax.m" /* pathName */
};

static emlrtRSInfo wf_emlrtRSI = {
    24,       /* lineNo */
    "ixamax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\ixamax.m" /* pathName */
};

static emlrtRSInfo ll_emlrtRSI = {
    41,        /* lineNo */
    "xzgetrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgetrf.m" /* pathName */
};

static emlrtRSInfo ml_emlrtRSI = {
    55,        /* lineNo */
    "xzgetrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgetrf.m" /* pathName */
};

static emlrtRSInfo nl_emlrtRSI = {
    63,        /* lineNo */
    "xzgetrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgetrf.m" /* pathName */
};

static emlrtRSInfo ol_emlrtRSI = {
    45,      /* lineNo */
    "xgeru", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xgeru."
    "m" /* pathName */
};

/* Function Definitions */
int32_T xzgetrf(const emlrtStack *sp, real_T A[36], int32_T ipiv[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  int32_T ijA;
  int32_T info;
  int32_T j;
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  _mm_storeu_si128(
      (__m128i *)&ipiv[0],
      _mm_add_epi32(_mm_set1_epi32(1),
                    _mm_add_epi32(_mm_set1_epi32(0),
                                  _mm_loadu_si128((const __m128i *)&iv1[0]))));
  ipiv[4] = 5;
  ipiv[5] = 6;
  info = 0;
  for (j = 0; j < 5; j++) {
    real_T smax;
    int32_T a;
    int32_T b;
    int32_T jA;
    int32_T jj;
    int32_T jp1j;
    int32_T mmj;
    mmj = 4 - j;
    b = j * 7;
    jj = j * 7;
    jp1j = b + 2;
    jA = 7 - j;
    st.site = &ll_emlrtRSI;
    b_st.site = &vf_emlrtRSI;
    a = 0;
    smax = muDoubleScalarAbs(A[jj]);
    c_st.site = &wf_emlrtRSI;
    for (k = 2; k < jA; k++) {
      real_T s;
      s = muDoubleScalarAbs(A[(b + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[jj + a] != 0.0) {
      if (a != 0) {
        a += j;
        ipiv[j] = a + 1;
        for (k = 0; k < 6; k++) {
          int32_T i;
          jA = j + k * 6;
          smax = A[jA];
          i = a + k * 6;
          A[jA] = A[i];
          A[i] = smax;
        }
      }
      jA = (jj - j) + 6;
      st.site = &ml_emlrtRSI;
      for (k = jp1j; k <= jA; k++) {
        A[k - 1] /= A[jj];
      }
    } else {
      info = j + 1;
    }
    st.site = &nl_emlrtRSI;
    b_st.site = &ol_emlrtRSI;
    c_st.site = &ne_emlrtRSI;
    d_st.site = &oe_emlrtRSI;
    jA = jj + 8;
    e_st.site = &pe_emlrtRSI;
    for (k = 0; k <= mmj; k++) {
      smax = A[(b + k * 6) + 6];
      if (smax != 0.0) {
        a = (jA - j) + 4;
        e_st.site = &qe_emlrtRSI;
        if ((jA <= a) && (a > 2147483646)) {
          f_st.site = &sb_emlrtRSI;
          check_forloop_overflow_error(&f_st);
        }
        for (ijA = jA; ijA <= a; ijA++) {
          A[ijA - 1] += A[((jj + ijA) - jA) + 1] * -smax;
        }
      }
      jA += 6;
    }
  }
  if ((info == 0) && (!(A[35] != 0.0))) {
    info = 6;
  }
  return info;
}

/* End of code generation (xzgetrf.c) */
