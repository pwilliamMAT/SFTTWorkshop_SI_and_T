/*
 * xgemv.c
 *
 * Code generation for function 'xgemv'
 *
 */

/* Include files */
#include "xgemv.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_mexutil.h"
#include <string.h>

/* Function Definitions */
void b_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[54],
             int32_T ia0, const real_T x[54], int32_T ix0, real_T y[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T ia;
  int32_T iac;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &hf_emlrtRSI;
  if (n != 0) {
    int32_T b;
    int32_T i;
    b_st.site = &jf_emlrtRSI;
    if (n > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    b = (uint8_T)n;
    if (b - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)b * sizeof(real_T));
    }
    i = ia0 + 9 * (n - 1);
    for (iac = ia0; iac <= i; iac += 9) {
      real_T c;
      c = 0.0;
      b = (iac + m) - 1;
      b_st.site = &tf_emlrtRSI;
      if ((iac <= b) && (b > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (ia = iac; ia <= b; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      b = div_nde_s32_floor(iac - ia0, 9);
      y[b] += c;
    }
  }
}

void c_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[40],
             int32_T ia0, const real_T x[40], int32_T ix0, real_T y[4])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T ia;
  int32_T iac;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &hf_emlrtRSI;
  if (n != 0) {
    int32_T b;
    int32_T i;
    b_st.site = &jf_emlrtRSI;
    if (n > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    b = (uint8_T)n;
    if (b - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)b * sizeof(real_T));
    }
    i = ia0 + 10 * (n - 1);
    for (iac = ia0; iac <= i; iac += 10) {
      real_T c;
      c = 0.0;
      b = (iac + m) - 1;
      b_st.site = &tf_emlrtRSI;
      if ((iac <= b) && (b > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (ia = iac; ia <= b; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      b = div_nde_s32_floor(iac - ia0, 10);
      y[b] += c;
    }
  }
}

void d_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[60],
             int32_T ia0, const real_T x[60], int32_T ix0, real_T y[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T ia;
  int32_T iac;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &hf_emlrtRSI;
  if (n != 0) {
    int32_T b;
    int32_T i;
    b_st.site = &jf_emlrtRSI;
    if (n > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    b = (uint8_T)n;
    if (b - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)b * sizeof(real_T));
    }
    i = ia0 + 10 * (n - 1);
    for (iac = ia0; iac <= i; iac += 10) {
      real_T c;
      c = 0.0;
      b = (iac + m) - 1;
      b_st.site = &tf_emlrtRSI;
      if ((iac <= b) && (b > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (ia = iac; ia <= b; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      b = div_nde_s32_floor(iac - ia0, 10);
      y[b] += c;
    }
  }
}

void xgemv(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[9],
           int32_T ia0, const real_T x[9], int32_T ix0, real_T y[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T ia;
  int32_T iac;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &hf_emlrtRSI;
  if (m != 0) {
    int32_T i;
    int32_T ix;
    b_st.site = &jf_emlrtRSI;
    if (m > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    ix = (uint8_T)m;
    if (ix - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)ix * sizeof(real_T));
    }
    ix = ix0;
    i = ia0 + 3 * (n - 1);
    for (iac = ia0; iac <= i; iac += 3) {
      int32_T b;
      b = (iac + m) - 1;
      b_st.site = &if_emlrtRSI;
      if ((iac <= b) && (b > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (ia = iac; ia <= b; ia++) {
        int32_T i1;
        i1 = ia - iac;
        y[i1] += A[ia - 1] * x[ix - 1];
      }
      ix++;
    }
  }
}

/* End of code generation (xgemv.c) */
