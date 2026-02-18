/*
 * issorted.c
 *
 * Code generation for function 'issorted'
 *
 */

/* Include files */
#include "issorted.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo
    yp_emlrtRSI =
        {
            74,         /* lineNo */
            "issorted", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\issorte"
            "d.m" /* pathName */
};

static emlrtRSInfo
    aq_emlrtRSI =
        {
            112,      /* lineNo */
            "looper", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\issorte"
            "d.m" /* pathName */
};

static emlrtRSInfo
    bq_emlrtRSI =
        {
            95,       /* lineNo */
            "looper", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\issorte"
            "d.m" /* pathName */
};

/* Function Definitions */
boolean_T issorted(const emlrtStack *sp, const emxArray_uint32_T *x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int32_T dim;
  const uint32_T *x_data;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  x_data = x->data;
  y = true;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }
  if (x->size[0] != 0) {
    int32_T n;
    if (dim <= 1) {
      n = x->size[0];
    } else {
      n = 1;
    }
    if (n != 1) {
      int32_T i;
      boolean_T exitg1;
      st.site = &yp_emlrtRSI;
      if (dim == 2) {
        i = -1;
      } else {
        i = 0;
      }
      b_st.site = &bq_emlrtRSI;
      n = 0;
      exitg1 = false;
      while ((!exitg1) && (n <= i)) {
        int32_T k;
        boolean_T exitg2;
        b_st.site = &aq_emlrtRSI;
        if (dim == 1) {
          n = x->size[0] - 1;
        } else {
          n = x->size[0];
        }
        c_st.site = &bq_emlrtRSI;
        if (n > 2147483646) {
          d_st.site = &sb_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= n - 1)) {
          int32_T subs[2];
          subs[0] = k + 1;
          subs[1] = 1;
          subs[dim - 1]++;
          y = (x_data[k] <= x_data[subs[0] - 1]);
          if (!y) {
            exitg2 = true;
          } else {
            k++;
          }
        }
        if (!y) {
          exitg1 = true;
        } else {
          n = 1;
        }
      }
    }
  }
  return y;
}

/* End of code generation (issorted.c) */
