/*
 * ConstantVelocityModel.c
 *
 * Code generation for function 'ConstantVelocityModel'
 *
 */

/* Include files */
#include "ConstantVelocityModel.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fv_emlrtRSI = {
    72,                                 /* lineNo */
    "ConstantVelocityModel/predictjac", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "transition\\ConstantVelocityModel.m" /* pathName */
};

static emlrtRSInfo gv_emlrtRSI =
    {
        54,            /* lineNo */
        "constveljac", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvelj"
        "ac.m" /* pathName */
};

static emlrtRSInfo hv_emlrtRSI =
    {
        67,            /* lineNo */
        "constveljac", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvelj"
        "ac.m" /* pathName */
};

static emlrtRSInfo iv_emlrtRSI =
    {
        73,            /* lineNo */
        "constveljac", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvelj"
        "ac.m" /* pathName */
};

/* Function Definitions */
void c_ConstantVelocityModel_predict(const emlrtStack *sp,
                                     const real_T state[6], real_T varargin_2,
                                     real_T F[36], real_T B[18])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T B_idx_0;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &fv_emlrtRSI;
  b_st.site = &gv_emlrtRSI;
  c_st.site = &fe_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(state[k])) && (!muDoubleScalarIsNaN(state[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constveljac:expectedFinite", 3, 4, 22,
        "input number 1, state,");
  }
  b_st.site = &hv_emlrtRSI;
  c_st.site = &fe_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_2) || muDoubleScalarIsNaN(varargin_2)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constveljac:expectedFinite", 3, 4, 19, "input number 3, dt,");
  }
  b_st.site = &iv_emlrtRSI;
  B_idx_0 = varargin_2 * varargin_2 / 2.0;
  memset(&F[0], 0, 36U * sizeof(real_T));
  F[0] = 1.0;
  F[14] = 1.0;
  F[28] = 1.0;
  F[1] = 0.0;
  F[15] = 0.0;
  F[29] = 0.0;
  F[6] = varargin_2;
  F[20] = varargin_2;
  F[34] = varargin_2;
  F[7] = 1.0;
  F[21] = 1.0;
  F[35] = 1.0;
  memset(&B[0], 0, 18U * sizeof(real_T));
  B[0] = B_idx_0;
  B[8] = B_idx_0;
  B[16] = B_idx_0;
  B[1] = varargin_2;
  B[9] = varargin_2;
  B[17] = varargin_2;
}

/* End of code generation (ConstantVelocityModel.c) */
