/*
 * sumMatrixIncludeNaN.c
 *
 * Code generation for function 'sumMatrixIncludeNaN'
 *
 */

/* Include files */
#include "sumMatrixIncludeNaN.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo hg_emlrtRSI = {
    178,          /* lineNo */
    "sumColumnB", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo ig_emlrtRSI = {
    210,         /* lineNo */
    "sumColumn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

/* Function Definitions */
real_T sumColumnB(const emlrtStack *sp, const real_T x_data[], int32_T vlen)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T y;
  int32_T i;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &hg_emlrtRSI;
  y = x_data[0];
  b_st.site = &ig_emlrtRSI;
  if (vlen - 1 > 2147483646) {
    c_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  i = (uint8_T)(vlen - 1);
  for (k = 0; k < i; k++) {
    y += x_data[k + 1];
  }
  return y;
}

/* End of code generation (sumMatrixIncludeNaN.c) */
