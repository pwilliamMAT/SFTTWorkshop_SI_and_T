/*
 * objectTrack.c
 *
 * Code generation for function 'objectTrack'
 *
 */

/* Include files */
#include "objectTrack.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo xf_emlrtRSI =
    {
        355,                                        /* lineNo */
        "objectTrack/set.ObjectClassProbabilities", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo yf_emlrtRSI =
    {
        358,                                        /* lineNo */
        "objectTrack/set.ObjectClassProbabilities", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ag_emlrtRSI = {
    20,    /* lineNo */
    "sum", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\sum.m" /* pathName
                                                                        */
};

static emlrtRSInfo cg_emlrtRSI = {
    86,                      /* lineNo */
    "combineVectorElements", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\combin"
    "eVectorElements.m" /* pathName */
};

static emlrtRSInfo dg_emlrtRSI = {
    99,                 /* lineNo */
    "blockedSummation", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\blocke"
    "dSummation.m" /* pathName */
};

static emlrtRSInfo eg_emlrtRSI = {
    22,                    /* lineNo */
    "sumMatrixIncludeNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo fg_emlrtRSI = {
    42,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRTEInfo hc_emlrtRTEI =
    {
        359,                                        /* lineNo */
        17,                                         /* colNo */
        "objectTrack/set.ObjectClassProbabilities", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pName */
};

static emlrtRTEInfo ic_emlrtRTEI = {
    13,                 /* lineNo */
    37,                 /* colNo */
    "validatenonempty", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonempty.m" /* pName */
};

static emlrtRTEInfo jc_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatele.m" /* pName */
};

/* Function Definitions */
uint32_T b_objectTrack_objectTrack(
    uint32_T *track_BranchID, uint32_T *track_SourceIndex, uint32_T *track_Age,
    real_T *track_ObjectClassID, real_T c_track_ObjectClassProbabilitie[],
    int32_T d_track_ObjectClassProbabilitie[2], boolean_T *track_IsConfirmed,
    boolean_T *track_IsCoasted, boolean_T *track_IsSelfReported,
    real_T track_pState[6], real_T track_pStateCovariance[36],
    real_T *track_pUpdateTime)
{
  int32_T i;
  uint32_T track_TrackID;
  track_TrackID = 1U;
  *track_BranchID = 0U;
  *track_SourceIndex = 1U;
  *track_pUpdateTime = 0.0;
  *track_Age = 1U;
  for (i = 0; i < 6; i++) {
    track_pState[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    track_pStateCovariance[i] = iv[i];
  }
  *track_IsConfirmed = true;
  *track_IsCoasted = false;
  *track_IsSelfReported = true;
  d_track_ObjectClassProbabilitie[0] = 1;
  d_track_ObjectClassProbabilitie[1] = 1;
  c_track_ObjectClassProbabilitie[0] = 1.0;
  *track_ObjectClassID = 0.0;
  return track_TrackID;
}

uint32_T b_objectTrack_set_TrackID(
    uint32_T b_value, uint32_T *obj_BranchID, uint32_T *obj_SourceIndex,
    uint32_T *obj_Age, real_T *obj_ObjectClassID,
    real_T c_obj_ObjectClassProbabilities_[],
    int32_T d_obj_ObjectClassProbabilities_[2], boolean_T *obj_IsConfirmed,
    boolean_T *obj_IsCoasted, boolean_T *obj_IsSelfReported,
    real_T obj_pState[6], real_T obj_pStateCovariance[36],
    real_T *obj_pUpdateTime)
{
  int32_T i;
  *obj_BranchID = 0U;
  *obj_SourceIndex = 1U;
  *obj_Age = 1U;
  *obj_ObjectClassID = 0.0;
  d_obj_ObjectClassProbabilities_[0] = 1;
  d_obj_ObjectClassProbabilities_[1] = 1;
  c_obj_ObjectClassProbabilities_[0] = 1.0;
  *obj_IsConfirmed = true;
  *obj_IsCoasted = false;
  *obj_IsSelfReported = true;
  for (i = 0; i < 6; i++) {
    obj_pState[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj_pStateCovariance[i] = iv[i];
  }
  *obj_pUpdateTime = 0.0;
  return b_value;
}

void c_objectTrack_set_ObjectClassPr(const emlrtStack *sp, b_objectTrack *obj,
                                     const real_T value_data[],
                                     const int32_T value_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T y;
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
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  st.site = &xf_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  if (value_size[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &ic_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:objectTrack:expectedNonempty", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  b_st.site = &fb_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= value_size[1] - 1)) {
    if ((!muDoubleScalarIsInf(value_data[k])) &&
        (!muDoubleScalarIsNaN(value_data[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  b_st.site = &fb_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= value_size[1] - 1)) {
    if (!(value_data[k] < 0.0)) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  b_st.site = &fb_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= value_size[1] - 1)) {
    if (value_data[k] <= 1.0) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &jc_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
        "MATLAB:objectTrack:notLessEqual", 9, 4, 24, "ObjectClassProbabilities",
        4, 2, "<=", 4, 1, "1");
  }
  st.site = &yf_emlrtRSI;
  b_st.site = &ag_emlrtRSI;
  c_st.site = &bg_emlrtRSI;
  d_st.site = &cg_emlrtRSI;
  e_st.site = &dg_emlrtRSI;
  f_st.site = &eg_emlrtRSI;
  g_st.site = &fg_emlrtRSI;
  y = sumColumnB(&g_st, value_data, value_size[1]);
  st.site = &yf_emlrtRSI;
  if (muDoubleScalarAbs(y - 1.0) > 1.4901161193847656E-8) {
    emlrtErrorWithMessageIdR2018a(
        sp, &hc_emlrtRTEI,
        "shared_tracking:objectTrack:invalidClassProbabilityVector",
        "shared_tracking:objectTrack:invalidClassProbabilityVector", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  obj->ObjectClassProbabilities.size[0] = 1;
  k = value_size[1];
  obj->ObjectClassProbabilities.size[1] = value_size[1];
  memcpy(&obj->ObjectClassProbabilities.data[0], &value_data[0],
         (uint32_T)k * sizeof(real_T));
}

void d_objectTrack_set_ObjectClassPr(const emlrtStack *sp, objectTrack *obj,
                                     real_T b_value)
{
  emlrtStack b_st;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &xf_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  if (muDoubleScalarIsInf(b_value) || muDoubleScalarIsNaN(b_value)) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  b_st.site = &fb_emlrtRSI;
  if (b_value < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  b_st.site = &fb_emlrtRSI;
  if (!(b_value <= 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &jc_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
        "MATLAB:objectTrack:notLessEqual", 9, 4, 24, "ObjectClassProbabilities",
        4, 2, "<=", 4, 1, "1");
  }
  st.site = &yf_emlrtRSI;
  if (muDoubleScalarAbs(b_sumColumnB(b_value) - 1.0) > 1.4901161193847656E-8) {
    emlrtErrorWithMessageIdR2018a(
        sp, &hc_emlrtRTEI,
        "shared_tracking:objectTrack:invalidClassProbabilityVector",
        "shared_tracking:objectTrack:invalidClassProbabilityVector", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  obj->ObjectClassProbabilities = b_value;
}

uint32_T objectTrack_objectTrack(
    uint32_T *track_BranchID, uint32_T *track_SourceIndex, uint32_T *track_Age,
    real_T *track_ObjectClassID, real_T *track_ObjectClassProbabilities,
    boolean_T *track_IsConfirmed, boolean_T *track_IsCoasted,
    boolean_T *track_IsSelfReported, real_T track_pState[6],
    real_T track_pStateCovariance[36], real_T *track_pUpdateTime)
{
  int32_T i;
  uint32_T track_TrackID;
  track_TrackID = 1U;
  *track_BranchID = 0U;
  *track_SourceIndex = 1U;
  *track_pUpdateTime = 0.0;
  *track_Age = 1U;
  for (i = 0; i < 6; i++) {
    track_pState[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    track_pStateCovariance[i] = iv[i];
  }
  *track_IsConfirmed = true;
  *track_IsCoasted = false;
  *track_IsSelfReported = true;
  *track_ObjectClassProbabilities = 1.0;
  *track_ObjectClassID = 0.0;
  return track_TrackID;
}

uint32_T objectTrack_set_TrackID(
    const emlrtStack *sp, real_T b_value, uint32_T *obj_BranchID,
    uint32_T *obj_SourceIndex, uint32_T *obj_Age, real_T *obj_ObjectClassID,
    real_T c_obj_ObjectClassProbabilities_[],
    int32_T d_obj_ObjectClassProbabilities_[2], boolean_T *obj_IsConfirmed,
    boolean_T *obj_IsCoasted, boolean_T *obj_IsSelfReported,
    real_T obj_pState[6], real_T obj_pStateCovariance[36],
    real_T *obj_pUpdateTime)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  uint32_T obj_TrackID;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  *obj_BranchID = 0U;
  *obj_SourceIndex = 1U;
  *obj_Age = 1U;
  *obj_ObjectClassID = 0.0;
  d_obj_ObjectClassProbabilities_[0] = 1;
  d_obj_ObjectClassProbabilities_[1] = 1;
  c_obj_ObjectClassProbabilities_[0] = 1.0;
  *obj_IsConfirmed = true;
  *obj_IsCoasted = false;
  *obj_IsSelfReported = true;
  for (i = 0; i < 6; i++) {
    obj_pState[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj_pStateCovariance[i] = iv[i];
  }
  *obj_pUpdateTime = 0.0;
  st.site = &ph_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  if (muDoubleScalarIsInf(b_value) || muDoubleScalarIsNaN(b_value) ||
      (!(muDoubleScalarFloor(b_value) == b_value))) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:objectTrack:expectedInteger", 3, 4, 7, "TrackID");
  }
  if (b_value < 4.294967296E+9) {
    if (b_value >= 0.0) {
      obj_TrackID = (uint32_T)b_value;
    } else {
      obj_TrackID = 0U;
    }
  } else {
    obj_TrackID = MAX_uint32_T;
  }
  return obj_TrackID;
}

/* End of code generation (objectTrack.c) */
