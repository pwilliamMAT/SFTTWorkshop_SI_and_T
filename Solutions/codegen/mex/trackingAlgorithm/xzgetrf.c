/*
 * xzgetrf.c
 *
 * Code generation for function 'xzgetrf'
 *
 */

/* Include files */
#include "xzgetrf.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
int32_T xzgetrf(const emlrtStack *sp, real_T A[16], int32_T ipiv[4])
{
  static const int32_T offsets[4] = {0, 1, 2, 3};
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
      _mm_add_epi32(
          _mm_set1_epi32(1),
          _mm_add_epi32(_mm_set1_epi32(0),
                        _mm_loadu_si128((const __m128i *)&offsets[0]))));
  info = 0;
  for (j = 0; j < 3; j++) {
    real_T smax;
    int32_T a;
    int32_T b;
    int32_T jA;
    int32_T jj;
    int32_T jp1j;
    int32_T mmj;
    mmj = 2 - j;
    b = j * 5;
    jj = j * 5;
    jp1j = b + 2;
    jA = 5 - j;
    st.site = &fab_emlrtRSI;
    b_st.site = &fh_emlrtRSI;
    a = 0;
    smax = muDoubleScalarAbs(A[jj]);
    c_st.site = &gh_emlrtRSI;
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
        jA = j + a;
        ipiv[j] = jA + 1;
        smax = A[j];
        A[j] = A[jA];
        A[jA] = smax;
        smax = A[j + 4];
        A[j + 4] = A[jA + 4];
        A[jA + 4] = smax;
        smax = A[j + 8];
        A[j + 8] = A[jA + 8];
        A[jA + 8] = smax;
        smax = A[j + 12];
        A[j + 12] = A[jA + 12];
        A[jA + 12] = smax;
      }
      jA = (jj - j) + 4;
      st.site = &gab_emlrtRSI;
      for (k = jp1j; k <= jA; k++) {
        A[k - 1] /= A[jj];
      }
    } else {
      info = j + 1;
    }
    st.site = &hab_emlrtRSI;
    b_st.site = &iab_emlrtRSI;
    c_st.site = &cf_emlrtRSI;
    d_st.site = &df_emlrtRSI;
    jA = jj + 6;
    e_st.site = &ef_emlrtRSI;
    for (k = 0; k <= mmj; k++) {
      smax = A[(b + (k << 2)) + 4];
      if (smax != 0.0) {
        a = (jA - j) + 2;
        e_st.site = &ff_emlrtRSI;
        if ((jA <= a) && (a > 2147483646)) {
          f_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&f_st);
        }
        for (ijA = jA; ijA <= a; ijA++) {
          A[ijA - 1] += A[((jj + ijA) - jA) + 1] * -smax;
        }
      }
      jA += 4;
    }
  }
  if ((info == 0) && (!(A[15] != 0.0))) {
    info = 4;
  }
  return info;
}

/* End of code generation (xzgetrf.c) */
