/*
 * computeLikelihoodByIndex.c
 *
 * Code generation for function 'computeLikelihoodByIndex'
 *
 */

/* Include files */
#include "computeLikelihoodByIndex.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo jt_emlrtRSI = {
    36,           /* lineNo */
    "parseIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtRSInfo lt_emlrtRSI = {
    15,                                              /* lineNo */
    "ind2sub",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/ind2sub.m" /* pathName */
};

static emlrtRSInfo nt_emlrtRSI = {
    30,                                              /* lineNo */
    "ind2sub",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/ind2sub.m" /* pathName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    13,                                                         /* lineNo */
    27,                                                         /* colNo */
    "mustBeInteger",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/validators/mustBeInteger.m" /* pName */
};

static emlrtRTEInfo u_emlrtRTEI = {
    21,                                                  /* lineNo */
    15,                                                  /* colNo */
    "ind2sub",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/ind2sub.m" /* pName */
};

/* Function Definitions */
real_T parseIndex(const emlrtStack *sp, const emxArray_struct_T *trackList,
                  const int32_T z_size[2], real_T b_index, real_T *measIndex)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T trkIndex;
  int32_T vk;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &jt_emlrtRSI;
  b_st.site = &lt_emlrtRSI;
  if (muDoubleScalarIsInf(b_index) || muDoubleScalarIsNaN(b_index)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &t_emlrtRTEI,
                                  "MATLAB:validators:mustBeFinite",
                                  "MATLAB:validators:mustBeFinite", 0);
  }
  b_st.site = &lt_emlrtRSI;
  if (!(b_index == muDoubleScalarFloor(b_index))) {
    emlrtErrorWithMessageIdR2018a(&b_st, &s_emlrtRTEI,
                                  "MATLAB:validators:mustBeInteger",
                                  "MATLAB:validators:mustBeInteger", 0);
  }
  b_st.site = &nt_emlrtRSI;
  if ((int32_T)b_index >
      (z_size[1] + 1) * (int32_T)((uint32_T)trackList->size[0] + 1U)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &u_emlrtRTEI,
                                  "Coder:MATLAB:ind2sub_IndexOutOfRange",
                                  "Coder:MATLAB:ind2sub_IndexOutOfRange", 0);
  }
  vk = (int32_T)((uint32_T)((int32_T)b_index - 1) / (uint32_T)(z_size[1] + 1));
  trkIndex = (real_T)(vk + 1) - 1.0;
  *measIndex = (real_T)((int32_T)b_index - vk * (z_size[1] + 1)) - 1.0;
  return trkIndex;
}

/* End of code generation (computeLikelihoodByIndex.c) */
