/*
 * xpotrf.c
 *
 * Code generation for function 'xpotrf'
 *
 */

/* Include files */
#include "xpotrf.h"
#include "rt_nonfinite.h"
#include "xdotc.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo wd_emlrtRSI = {
    42,        /* lineNo */
    "zpotrfU", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzpotrf.m" /* pathName */
};

static emlrtRSInfo ei_emlrtRSI = {
    16,       /* lineNo */
    "xpotrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xpotrf.m" /* pathName */
};

static emlrtRSInfo fi_emlrtRSI = {
    19,        /* lineNo */
    "xzpotrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzpotrf.m" /* pathName */
};

/* Function Definitions */
int32_T b_xpotrf(const emlrtStack *sp, real_T A[36])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T alpha1;
  real_T beta1;
  int32_T info;
  int32_T j;
  int32_T k;
  char_T TRANSA;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &ei_emlrtRSI;
  b_st.site = &fi_emlrtRSI;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 6)) {
    real_T ssq;
    int32_T idxA1j;
    int32_T idxAjj;
    idxA1j = j * 6;
    idxAjj = idxA1j + j;
    c_st.site = &wd_emlrtRSI;
    ssq = xdotc(&c_st, j, A, idxA1j + 1, A, idxA1j + 1);
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = muDoubleScalarSqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < 6) {
        int32_T idxAjjp1;
        idxAjjp1 = idxAjj + 7;
        if (j >= 1) {
          alpha1 = -1.0;
          beta1 = 1.0;
          TRANSA = 'T';
          m_t = (ptrdiff_t)j;
          n_t = (ptrdiff_t)(5 - j);
          lda_t = (ptrdiff_t)6;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)6;
          dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A[idxA1j + 6], &lda_t,
                &A[idxA1j], &incx_t, &beta1, &A[idxAjj + 6], &incy_t);
        }
        ssq = 1.0 / ssq;
        idxA1j = (idxAjj + 6 * (4 - j)) + 7;
        for (k = idxAjjp1; k <= idxA1j; k += 6) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

int32_T c_xpotrf(const emlrtStack *sp, real_T A[16])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T alpha1;
  real_T beta1;
  int32_T info;
  int32_T j;
  int32_T k;
  char_T TRANSA;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &ei_emlrtRSI;
  b_st.site = &fi_emlrtRSI;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    real_T ssq;
    int32_T idxA1j;
    int32_T idxAjj;
    idxA1j = j << 2;
    idxAjj = idxA1j + j;
    c_st.site = &wd_emlrtRSI;
    ssq = c_xdotc(&c_st, j, A, idxA1j + 1, A, idxA1j + 1);
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = muDoubleScalarSqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < 4) {
        int32_T idxAjjp1;
        idxAjjp1 = idxAjj + 5;
        if (j >= 1) {
          alpha1 = -1.0;
          beta1 = 1.0;
          TRANSA = 'T';
          m_t = (ptrdiff_t)j;
          n_t = (ptrdiff_t)(3 - j);
          lda_t = (ptrdiff_t)4;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)4;
          dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A[idxA1j + 4], &lda_t,
                &A[idxA1j], &incx_t, &beta1, &A[idxAjj + 4], &incy_t);
        }
        ssq = 1.0 / ssq;
        idxA1j = (idxAjj + ((2 - j) << 2)) + 5;
        for (k = idxAjjp1; k <= idxA1j; k += 4) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

int32_T xpotrf(const emlrtStack *sp, real_T A[9])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T alpha1;
  real_T beta1;
  int32_T info;
  int32_T j;
  int32_T k;
  char_T TRANSA;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &ei_emlrtRSI;
  b_st.site = &fi_emlrtRSI;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 3)) {
    real_T ssq;
    int32_T idxA1j;
    int32_T idxAjj;
    idxA1j = j * 3;
    idxAjj = idxA1j + j;
    c_st.site = &wd_emlrtRSI;
    ssq = b_xdotc(&c_st, j, A, idxA1j + 1, A, idxA1j + 1);
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = muDoubleScalarSqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < 3) {
        int32_T idxAjjp1;
        idxAjjp1 = idxAjj + 4;
        if (j >= 1) {
          alpha1 = -1.0;
          beta1 = 1.0;
          TRANSA = 'T';
          m_t = (ptrdiff_t)j;
          n_t = (ptrdiff_t)(2 - j);
          lda_t = (ptrdiff_t)3;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)3;
          dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A[idxA1j + 3], &lda_t,
                &A[idxA1j], &incx_t, &beta1, &A[idxAjj + 3], &incy_t);
        }
        ssq = 1.0 / ssq;
        idxA1j = (idxAjj + 3 * (1 - j)) + 4;
        for (k = idxAjjp1; k <= idxA1j; k += 3) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

/* End of code generation (xpotrf.c) */
