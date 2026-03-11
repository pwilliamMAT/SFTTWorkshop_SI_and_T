/*
 * all.c
 *
 * Code generation for function 'all'
 *
 */

/* Include files */
#include "all.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo xc_emlrtRSI =
    {
        13,    /* lineNo */
        "all", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\all.m" /* pathName
                                                                        */
};

/* Function Definitions */
void all(const emlrtStack *sp, const boolean_T x[36], boolean_T y[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i;
  int32_T i2;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &xc_emlrtRSI;
  for (i = 0; i < 6; i++) {
    y[i] = true;
  }
  i2 = 6;
  for (i = 0; i < 6; i++) {
    int32_T a;
    int32_T i1;
    boolean_T exitg1;
    a = i2;
    i1 = i2 - 5;
    i2 += 6;
    b_st.site = &yc_emlrtRSI;
    if ((i1 <= a) && (a > 2147483646)) {
      c_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    exitg1 = false;
    while ((!exitg1) && (i1 <= a)) {
      if (!x[i1 - 1]) {
        y[i] = false;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
}

boolean_T b_all(const boolean_T x_data[], const int32_T x_size[2])
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x_size[1])) {
    if (!x_data[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

/* End of code generation (all.c) */
