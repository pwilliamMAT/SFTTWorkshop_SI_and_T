/*
 * colon.c
 *
 * Code generation for function 'colon'
 *
 */

/* Include files */
#include "colon.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo ubb_emlrtRSI =
    {
        162,                            /* lineNo */
        "eml_integer_colon_dispatcher", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pathName
                                                                          */
};

static emlrtRSInfo vbb_emlrtRSI =
    {
        190,                        /* lineNo */
        "eml_signed_integer_colon", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pathName
                                                                          */
};

static emlrtRTEInfo re_emlrtRTEI =
    {
        186,     /* lineNo */
        20,      /* colNo */
        "colon", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pName
                                                                          */
};

/* Function Definitions */
void eml_integer_colon_dispatcher(const emlrtStack *sp, int32_T b,
                                  emxArray_int32_T *y)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T k;
  int32_T n;
  int32_T yk;
  int32_T *y_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &ubb_emlrtRSI;
  if (b < 1) {
    n = 0;
  } else {
    n = b;
  }
  yk = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n;
  emxEnsureCapacity_int32_T(&st, y, yk, &re_emlrtRTEI);
  y_data = y->data;
  if (n > 0) {
    y_data[0] = 1;
    yk = 1;
    b_st.site = &vbb_emlrtRSI;
    if (n > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (k = 2; k <= n; k++) {
      yk++;
      y_data[k - 1] = yk;
    }
  }
}

/* End of code generation (colon.c) */
