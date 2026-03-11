/*
 * objectTrack.c
 *
 * Code generation for function 'objectTrack'
 *
 */

/* Include files */
#include "objectTrack.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_internal_types.h"
#include "fusionAlgorithm_types.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "validateattributes.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo mc_emlrtRSI =
    {
        492,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI =
    {
        519,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI =
    {
        521,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo qc_emlrtRSI =
    {
        524,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo yf_emlrtRSI =
    {
        355,                                        /* lineNo */
        "objectTrack/set.ObjectClassProbabilities", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ag_emlrtRSI =
    {
        358,                                        /* lineNo */
        "objectTrack/set.ObjectClassProbabilities", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo bg_emlrtRSI = {
    20,    /* lineNo */
    "sum", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\sum.m" /* pathName
                                                                        */
};

static emlrtRSInfo dg_emlrtRSI = {
    86,                      /* lineNo */
    "combineVectorElements", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\combin"
    "eVectorElements.m" /* pathName */
};

static emlrtRSInfo eg_emlrtRSI = {
    99,                 /* lineNo */
    "blockedSummation", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\blocke"
    "dSummation.m" /* pathName */
};

static emlrtRSInfo fg_emlrtRSI = {
    22,                    /* lineNo */
    "sumMatrixIncludeNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo gg_emlrtRSI = {
    42,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo
    ng_emlrtRSI =
        {
            241,       /* lineNo */
            "charcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

static emlrtRSInfo
    og_emlrtRSI =
        {
            242,       /* lineNo */
            "charcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

static emlrtRSInfo pg_emlrtRSI = {
    16,      /* lineNo */
    "lower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\strfun\\lower.m" /* pathName
                                                                         */
};

static emlrtRSInfo qg_emlrtRSI = {
    10,                     /* lineNo */
    "eml_string_transform", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\strfun\\eml_string_"
    "transform.m" /* pathName */
};

static emlrtRSInfo rg_emlrtRSI =
    {
        18,            /* lineNo */
        "ifWhileCond", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\ifWhileCond.m" /* pathName */
};

static emlrtRSInfo sg_emlrtRSI =
    {
        31,            /* lineNo */
        "checkNoNaNs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\ifWhileCond.m" /* pathName */
};

static emlrtRSInfo tg_emlrtRSI =
    {
        324,                               /* lineNo */
        "objectTrack/set.TrackLogicState", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ug_emlrtRSI =
    {
        600,                                   /* lineNo */
        "objectTrack/validateTrackLogicState", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    22,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatele.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatescalar", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatescalar.m" /* pName */
};

static emlrtRTEInfo n_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatele.m" /* pName */
};

static emlrtRTEInfo o_emlrtRTEI = {
    13,                 /* lineNo */
    37,                 /* colNo */
    "validatenonempty", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonempty.m" /* pName */
};

static emlrtRTEInfo p_emlrtRTEI =
    {
        359,                                        /* lineNo */
        17,                                         /* colNo */
        "objectTrack/set.ObjectClassProbabilities", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI =
    {
        527,                         /* lineNo */
        17,                          /* colNo */
        "objectTrack/setProperties", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pName */
};

/* Function Definitions */
void objectTrack_objectTrack(
    const emlrtStack *sp, uint32_T varargin_1_SourceIndex,
    real_T varargin_1_UpdateTime, const real_T varargin_1_State[6],
    const real_T varargin_1_StateCovariance[36],
    real_T varargin_1_ObjectClassID,
    const real_T c_varargin_1_ObjectClassProbabi[],
    const int32_T d_varargin_1_ObjectClassProbabi[2],
    const char_T varargin_1_TrackLogic[10],
    const real_T varargin_1_TrackLogicState_data[],
    const int32_T varargin_1_TrackLogicState_size[2],
    boolean_T varargin_1_IsConfirmed, boolean_T varargin_1_IsCoasted,
    boolean_T varargin_1_IsSelfReported,
    const struct1_T varargin_1_ObjectAttributes, b_objectTrack *track)
{
  static const char_T b_cv[10] = {'I', 'n', 't', 'e', 'g',
                                  'r', 'a', 't', 'e', 'd'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  real_T y;
  int32_T i;
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  st.site = &ic_emlrtRSI;
  track->SourceIndex = varargin_1_SourceIndex;
  track->IsConfirmed = varargin_1_IsConfirmed;
  track->IsCoasted = varargin_1_IsCoasted;
  track->IsSelfReported = varargin_1_IsSelfReported;
  track->ObjectAttributes = varargin_1_ObjectAttributes;
  b_st.site = &jc_emlrtRSI;
  c_st.site = &rc_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (varargin_1_UpdateTime < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
  }
  d_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_1_UpdateTime) ||
      muDoubleScalarIsNaN(varargin_1_UpdateTime)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
  }
  track->pUpdateTime = varargin_1_UpdateTime;
  b_st.site = &kc_emlrtRSI;
  c_st.site = &sc_emlrtRSI;
  validateattributes(&c_st, varargin_1_State);
  for (i = 0; i < 6; i++) {
    track->pState[i] = varargin_1_State[i];
  }
  b_st.site = &lc_emlrtRSI;
  c_st.site = &tc_emlrtRSI;
  b_validateattributes(&c_st, varargin_1_StateCovariance);
  c_st.site = &uc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&c_st, varargin_1_StateCovariance);
  memcpy(&track->pStateCovariance[0], &varargin_1_StateCovariance[0],
         36U * sizeof(real_T));
  b_st.site = &mc_emlrtRSI;
  c_st.site = &yf_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (d_varargin_1_ObjectClassProbabi[1] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &o_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:objectTrack:expectedNonempty", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  d_st.site = &gb_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= d_varargin_1_ObjectClassProbabi[1] - 1)) {
    if ((!muDoubleScalarIsInf(c_varargin_1_ObjectClassProbabi[k])) &&
        (!muDoubleScalarIsNaN(c_varargin_1_ObjectClassProbabi[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  d_st.site = &gb_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= d_varargin_1_ObjectClassProbabi[1] - 1)) {
    if (!(c_varargin_1_ObjectClassProbabi[k] < 0.0)) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  d_st.site = &gb_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= d_varargin_1_ObjectClassProbabi[1] - 1)) {
    if (c_varargin_1_ObjectClassProbabi[k] <= 1.0) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &n_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
        "MATLAB:objectTrack:notLessEqual", 9, 4, 24, "ObjectClassProbabilities",
        4, 2, "<=", 4, 1, "1");
  }
  c_st.site = &ag_emlrtRSI;
  d_st.site = &bg_emlrtRSI;
  e_st.site = &cg_emlrtRSI;
  f_st.site = &dg_emlrtRSI;
  g_st.site = &eg_emlrtRSI;
  h_st.site = &fg_emlrtRSI;
  i_st.site = &gg_emlrtRSI;
  y = sumColumnB(&i_st, c_varargin_1_ObjectClassProbabi,
                 d_varargin_1_ObjectClassProbabi[1]);
  c_st.site = &ag_emlrtRSI;
  if (muDoubleScalarAbs(y - 1.0) > 1.4901161193847656E-8) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &p_emlrtRTEI,
        "shared_tracking:objectTrack:invalidClassProbabilityVector",
        "shared_tracking:objectTrack:invalidClassProbabilityVector", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  b_st.site = &nc_emlrtRSI;
  c_st.site = &jg_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (varargin_1_ObjectClassID < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 13, "ObjectClassID");
  }
  d_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_1_ObjectClassID) ||
      muDoubleScalarIsNaN(varargin_1_ObjectClassID) ||
      (!(muDoubleScalarFloor(varargin_1_ObjectClassID) ==
         varargin_1_ObjectClassID))) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:objectTrack:expectedInteger", 3, 4, 13, "ObjectClassID");
  }
  track->ObjectClassID = varargin_1_ObjectClassID;
  b_st.site = &oc_emlrtRSI;
  c_st.site = &kg_emlrtRSI;
  d_st.site = &lg_emlrtRSI;
  p = false;
  k = 0;
  int32_T exitg2;
  do {
    exitg2 = 0;
    if (k < 10) {
      uint8_T u;
      e_st.site = &mg_emlrtRSI;
      u = (uint8_T)varargin_1_TrackLogic[k];
      if (u > 127) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &i_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      e_st.site = &ng_emlrtRSI;
      e_st.site = &og_emlrtRSI;
      f_st.site = &pg_emlrtRSI;
      g_st.site = &qg_emlrtRSI;
      e_st.site = &og_emlrtRSI;
      f_st.site = &pg_emlrtRSI;
      g_st.site = &qg_emlrtRSI;
      if (cv[u] != cv[(int32_T)b_cv[k]]) {
        exitg2 = 1;
      } else {
        k++;
      }
    } else {
      p = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);
  if (p) {
    int32_T x_size_idx_1;
    boolean_T x_data[10];
    b_st.site = &pc_emlrtRSI;
    k = varargin_1_TrackLogicState_size[1];
    x_size_idx_1 = varargin_1_TrackLogicState_size[1];
    for (i = 0; i < k; i++) {
      x_data[i] = (varargin_1_TrackLogicState_data[i] == -1.0);
    }
    p = (varargin_1_TrackLogicState_size[1] != 0);
    if (p) {
      c_st.site = &rg_emlrtRSI;
      d_st.site = &sg_emlrtRSI;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= x_size_idx_1 - 1)) {
        if (!x_data[k]) {
          p = false;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (!p) {
      b_st.site = &qc_emlrtRSI;
      c_st.site = &tg_emlrtRSI;
      d_st.site = &ug_emlrtRSI;
      e_st.site = &gb_emlrtRSI;
      p = true;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= varargin_1_TrackLogicState_size[1] - 1)) {
        if ((!muDoubleScalarIsInf(varargin_1_TrackLogicState_data[k])) &&
            (!muDoubleScalarIsNaN(varargin_1_TrackLogicState_data[k]))) {
          k++;
        } else {
          p = false;
          exitg1 = true;
        }
      }
      if (!p) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &m_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedFinite",
            "MATLAB:objectTrack:expectedFinite", 3, 4, 15, "TrackLogicState");
      }
      e_st.site = &gb_emlrtRSI;
      p = true;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= varargin_1_TrackLogicState_size[1] - 1)) {
        if (!(varargin_1_TrackLogicState_data[k] < 0.0)) {
          k++;
        } else {
          p = false;
          exitg1 = true;
        }
      }
      if (!p) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &l_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedNonnegative",
            "MATLAB:objectTrack:expectedNonnegative", 3, 4, 15,
            "TrackLogicState");
      }
      e_st.site = &gb_emlrtRSI;
      if (varargin_1_TrackLogicState_size[1] != 1) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &k_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedScalar",
            "MATLAB:objectTrack:expectedScalar", 3, 4, 15, "TrackLogicState");
      }
      e_st.site = &gb_emlrtRSI;
      if (!(varargin_1_TrackLogicState_data[0] <= 1.0)) {
        emlrtErrorWithMessageIdR2018a(
            &e_st, &j_emlrtRTEI, "MATLAB:validateattributes:expectedScalar",
            "MATLAB:objectTrack:notLessEqual", 9, 4, 15, "TrackLogicState", 4,
            2, "<=", 4, 1, "1");
      }
    }
  } else {
    emlrtErrorWithMessageIdR2018a(
        &st, &q_emlrtRTEI, "shared_tracking:objectTrack:invalidTrackLogicType",
        "shared_tracking:objectTrack:invalidTrackLogicType", 3, 4, 10,
        "TrackLogic");
  }
}

/* End of code generation (objectTrack.c) */
