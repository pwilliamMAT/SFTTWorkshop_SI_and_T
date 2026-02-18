/*
 * fuserSourceConfiguration.c
 *
 * Code generation for function 'fuserSourceConfiguration'
 *
 */

/* Include files */
#include "fuserSourceConfiguration.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo bb_emlrtRSI = {
    184,                              /* lineNo */
    "fuserSourceConfiguration/clone", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    185,                              /* lineNo */
    "fuserSourceConfiguration/clone", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    180,                              /* lineNo */
    "fuserSourceConfiguration/clone", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    98,                                                  /* lineNo */
    "fuserSourceConfiguration/fuserSourceConfiguration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    14,                 /* lineNo */
    37,                 /* colNo */
    "validatepositive", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatepositive.m" /* pName */
};

/* Function Definitions */
fuserSourceConfiguration *
fuserSourceConfiguration_clone(const emlrtStack *sp,
                               const fuserSourceConfiguration *obj,
                               fuserSourceConfiguration *iobj_0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  fuserSourceConfiguration *clonedObj;
  real_T varargin_2;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &bb_emlrtRSI;
  st.site = &cb_emlrtRSI;
  st.site = &db_emlrtRSI;
  varargin_2 = obj->SourceIndex;
  iobj_0->pIsTransformToCentralValid = false;
  iobj_0->pIsTransformToLocalValid = false;
  b_st.site = &eb_emlrtRSI;
  c_st.site = &fb_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_2) || muDoubleScalarIsNaN(varargin_2) ||
      (!(muDoubleScalarFloor(varargin_2) == varargin_2))) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:fuserSourceConfiguration:expectedInteger", 3, 4, 28,
        "input number 1, SourceIndex,");
  }
  c_st.site = &fb_emlrtRSI;
  if (varargin_2 <= 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &d_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedPositive",
        "MATLAB:fuserSourceConfiguration:expectedPositive", 3, 4, 28,
        "input number 1, SourceIndex,");
  }
  b_st.site = &e_emlrtRSI;
  clonedObj = iobj_0;
  b_st.site = &f_emlrtRSI;
  iobj_0->SourceIndex = varargin_2;
  b_st.site = &g_emlrtRSI;
  iobj_0->IsInternalSource = obj->IsInternalSource;
  iobj_0->IsInitializingCentralTracks = obj->IsInitializingCentralTracks;
  return clonedObj;
}

/* End of code generation (fuserSourceConfiguration.c) */
