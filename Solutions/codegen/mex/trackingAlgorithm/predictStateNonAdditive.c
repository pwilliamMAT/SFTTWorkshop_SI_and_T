/*
 * predictStateNonAdditive.c
 *
 * Code generation for function 'predictStateNonAdditive'
 *
 */

/* Include files */
#include "predictStateNonAdditive.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo gu_emlrtRSI = {
    13,                        /* lineNo */
    "predictStateNonAdditive", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\predictStateNonAdditive."
    "m" /* pathName */
};

static emlrtRSInfo hu_emlrtRSI = {
    65,                              /* lineNo */
    "ConstantVelocityModel/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "transition\\ConstantVelocityModel.m" /* pathName */
};

static emlrtRSInfo iu_emlrtRSI =
    {
        98,         /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

static emlrtRSInfo ju_emlrtRSI =
    {
        159,        /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

static emlrtRSInfo ku_emlrtRSI =
    {
        165,        /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

/* Function Definitions */
void predictStateNonAdditive(const emlrtStack *sp, real_T x[6], real_T dT)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T d;
  real_T d1;
  real_T d2;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &gu_emlrtRSI;
  b_st.site = &hu_emlrtRSI;
  c_st.site = &iu_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(x[k])) && (!muDoubleScalarIsNaN(x[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constvel:expectedFinite", 3, 4, 22, "input number 1, state,");
  }
  c_st.site = &ju_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  if (muDoubleScalarIsInf(dT) || muDoubleScalarIsNaN(dT)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constvel:expectedFinite", 3, 4, 19, "input number 3, dt,");
  }
  c_st.site = &ku_emlrtRSI;
  d = x[1];
  d1 = 0.5 * (dT * dT) * 0.0;
  x[0] = (x[0] + d * dT) + d1;
  d2 = 0.0 * dT;
  d += d2;
  x[1] = d;
  c_st.site = &ku_emlrtRSI;
  d = x[3];
  x[2] = (x[2] + d * dT) + d1;
  d += d2;
  x[3] = d;
  c_st.site = &ku_emlrtRSI;
  d = x[5];
  x[4] = (x[4] + d * dT) + d1;
  d += d2;
  x[5] = d;
}

/* End of code generation (predictStateNonAdditive.c) */
