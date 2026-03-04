/*
 * Fuserxcov.c
 *
 * Code generation for function 'Fuserxcov'
 *
 */

/* Include files */
#include "Fuserxcov.h"
#include "assertValidSizeArg.h"
#include "eml_int_forloop_overflow_check.h"
#include "ensurePosDefMatrix.h"
#include "fusecovint.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "gaussEKFilter.h"
#include "indexShapeCheck.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "unique.h"
#include "validateattributes.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ur_emlrtRSI = {
    25,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo vr_emlrtRSI = {
    26,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo wr_emlrtRSI = {
    33,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo xr_emlrtRSI = {
    36,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo yr_emlrtRSI = {
    37,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo as_emlrtRSI = {
    39,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo bs_emlrtRSI = {
    40,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo cs_emlrtRSI = {
    48,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo ds_emlrtRSI = {
    55,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo es_emlrtRSI = {
    56,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo fs_emlrtRSI = {
    57,               /* lineNo */
    "Fuserxcov/fuse", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pathName
                                                                        */
};

static emlrtRSInfo gs_emlrtRSI = {
    39,                                         /* lineNo */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m" /* pathName */
};

static emlrtRSInfo hs_emlrtRSI = {
    44,                                         /* lineNo */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m" /* pathName */
};

static emlrtRSInfo is_emlrtRSI = {
    45,                                         /* lineNo */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m" /* pathName */
};

static emlrtRSInfo js_emlrtRSI = {
    37,                                             /* lineNo */
    "sort",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sort.m" /* pathName */
};

static emlrtBCInfo dj_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    41,                                         /* lineNo */
    42,                                         /* colNo */
    "",                                         /* aName */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ej_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    39,                                         /* lineNo */
    42,                                         /* colNo */
    "",                                         /* aName */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo fj_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    39,                                         /* lineNo */
    47,                                         /* colNo */
    "",                                         /* aName */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo gj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    43,               /* lineNo */
    55,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo hj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    42,               /* lineNo */
    53,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo ij_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    31,               /* lineNo */
    42,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo jj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    42,               /* lineNo */
    33,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo kj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    43,               /* lineNo */
    35,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo lj_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    41,                                         /* lineNo */
    47,                                         /* colNo */
    "",                                         /* aName */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo mj_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    41,                                         /* lineNo */
    30,                                         /* colNo */
    "",                                         /* aName */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo nj_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    45,                                         /* lineNo */
    37,                                         /* colNo */
    "",                                         /* aName */
    "AbstractFusionAlgorithm/sortTracksByTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo oj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    37,               /* lineNo */
    51,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo pj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    42,               /* lineNo */
    68,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtBCInfo qj_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    43,               /* lineNo */
    70,               /* colNo */
    "",               /* aName */
    "Fuserxcov/fuse", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m", /* pName
                                                                         */
    0 /* checkKind */
};

static emlrtRTEInfo sg_emlrtRTEI = {
    39,                                           /* lineNo */
    5,                                            /* colNo */
    "find",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

static emlrtRTEInfo ai_emlrtRTEI = {
    39,                        /* lineNo */
    17,                        /* colNo */
    "AbstractFusionAlgorithm", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m" /* pName */
};

static emlrtRTEInfo bi_emlrtRTEI = {
    38,                                             /* lineNo */
    5,                                              /* colNo */
    "sort",                                         /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sort.m" /* pName */
};

static emlrtRTEInfo ci_emlrtRTEI = {
    36,          /* lineNo */
    41,          /* colNo */
    "Fuserxcov", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pName
                                                                        */
};

static emlrtRTEInfo di_emlrtRTEI = {
    358,                                          /* lineNo */
    24,                                           /* colNo */
    "find",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

static emlrtRTEInfo ei_emlrtRTEI = {
    36,          /* lineNo */
    17,          /* colNo */
    "Fuserxcov", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pName
                                                                        */
};

static emlrtRTEInfo fi_emlrtRTEI = {
    26,          /* lineNo */
    13,          /* colNo */
    "Fuserxcov", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pName
                                                                        */
};

static emlrtRTEInfo gi_emlrtRTEI = {
    39,          /* lineNo */
    17,          /* colNo */
    "Fuserxcov", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pName
                                                                        */
};

static emlrtRTEInfo hi_emlrtRTEI = {
    40,          /* lineNo */
    17,          /* colNo */
    "Fuserxcov", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pName
                                                                        */
};

static emlrtRTEInfo ii_emlrtRTEI = {
    36,                        /* lineNo */
    17,                        /* colNo */
    "AbstractFusionAlgorithm", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
    "AbstractFusionAlgorithm.m" /* pName */
};

static emlrtRTEInfo ji_emlrtRTEI = {
    20,          /* lineNo */
    31,          /* colNo */
    "Fuserxcov", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/Fuserxcov.m" /* pName
                                                                        */
};

/* Function Definitions */
void Fuserxcov_fuse(const emlrtStack *sp, const fusion_internal_Fuserxcov *obj,
                    c_objectTrack *centralTrack,
                    const emxArray_objectTrack *sourceTracks,
                    const emxArray_real_T *inAssigned)
{
  const c_objectTrack *sourceTracks_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_boolean_T *x;
  emxArray_int32_T *b_I;
  emxArray_int32_T *ii;
  emxArray_real_T *allCovars;
  emxArray_real_T *allStates;
  emxArray_real_T *allTimes;
  emxArray_real_T *uniqueTimes;
  emxArray_uint32_T *tracksAtThisTime;
  real_T varargin_1[2];
  const real_T *inAssigned_data;
  real_T trackTime;
  real_T *allCovars_data;
  real_T *allStates_data;
  real_T *allTimes_data;
  real_T *uniqueTimes_data;
  int32_T b_i;
  int32_T b_k;
  int32_T b_loop_ub;
  int32_T c_I;
  int32_T i;
  int32_T i1;
  int32_T ibtile;
  int32_T idx;
  int32_T j;
  int32_T k;
  int32_T loop_ub;
  int32_T *I_data;
  int32_T *ii_data;
  uint32_T b_varargin_1[3];
  uint32_T *tracksAtThisTime_data;
  boolean_T *x_data;
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
  inAssigned_data = inAssigned->data;
  sourceTracks_data = sourceTracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ur_emlrtRSI;
  loop_ub = inAssigned->size[0];
  if (inAssigned->size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, inAssigned->size[0], &fj_emlrtBCI, &st);
  }
  i = sourceTracks->size[1] - 1;
  idx = (int32_T)inAssigned_data[0] - 1;
  if (((int32_T)inAssigned_data[0] - 1 < 0) ||
      ((int32_T)inAssigned_data[0] - 1 > sourceTracks->size[1] - 1)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)inAssigned_data[0] - 1, 0,
                                  sourceTracks->size[1] - 1, &ej_emlrtBCI, &st);
  }
  b_st.site = &gs_emlrtRSI;
  c_st.site = &kk_emlrtRSI;
  emxInit_real_T(&b_st, &allTimes, 2, &ii_emlrtRTEI);
  ibtile = allTimes->size[0] * allTimes->size[1];
  allTimes->size[0] = 1;
  allTimes->size[1] = inAssigned->size[0];
  emxEnsureCapacity_real_T(&b_st, allTimes, ibtile, &ai_emlrtRTEI);
  allTimes_data = allTimes->data;
  for (b_i = 0; b_i < loop_ub; b_i++) {
    allTimes_data[b_i] = sourceTracks_data[idx].pUpdateTime;
  }
  for (b_i = 0; b_i <= loop_ub - 2; b_i++) {
    if (b_i + 2 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, loop_ub, &lj_emlrtBCI, &st);
    }
    ibtile = (int32_T)inAssigned_data[b_i + 1] - 1;
    if ((ibtile < 0) || (ibtile > i)) {
      emlrtDynamicBoundsCheckR2012b(ibtile, 0, i, &dj_emlrtBCI, &st);
    }
    if (b_i + 2 > allTimes->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 2, 1, allTimes->size[1], &mj_emlrtBCI,
                                    &st);
    }
    allTimes_data[b_i + 1] =
        sourceTracks_data[(int32_T)inAssigned_data[b_i + 1] - 1].pUpdateTime;
  }
  b_st.site = &hs_emlrtRSI;
  emxInit_int32_T(&b_st, &ii, 2, &sd_emlrtRTEI);
  c_st.site = &js_emlrtRSI;
  b_sort(&c_st, allTimes, ii);
  ii_data = ii->data;
  allTimes_data = allTimes->data;
  emxInit_int32_T(&b_st, &b_I, 2, &ji_emlrtRTEI);
  ibtile = b_I->size[0] * b_I->size[1];
  b_I->size[0] = 1;
  idx = ii->size[1];
  b_I->size[1] = ii->size[1];
  emxEnsureCapacity_int32_T(&b_st, b_I, ibtile, &bi_emlrtRTEI);
  I_data = b_I->data;
  for (b_i = 0; b_i < idx; b_i++) {
    I_data[b_i] = ii_data[b_i];
  }
  b_st.site = &is_emlrtRSI;
  indexShapeCheck(&b_st, inAssigned->size[0], b_I->size);
  for (b_i = 0; b_i < idx; b_i++) {
    ibtile = I_data[b_i];
    if ((ibtile < 1) || (ibtile > loop_ub)) {
      emlrtDynamicBoundsCheckR2012b(ibtile, 1, loop_ub, &nj_emlrtBCI, &st);
    }
  }
  st.site = &vr_emlrtRSI;
  emxInit_real_T(&st, &uniqueTimes, 2, &fi_emlrtRTEI);
  b_st.site = &ib_emlrtRSI;
  unique_vector(&b_st, allTimes, uniqueTimes);
  uniqueTimes_data = uniqueTimes->data;
  trackTime = centralTrack->pUpdateTime;
  i1 = uniqueTimes->size[1];
  if (uniqueTimes->size[1] - 1 >= 0) {
    b_loop_ub = allTimes->size[1];
    c_I = ii->size[1];
    varargin_1[0] = 1.0;
    b_varargin_1[0] = 1U;
    b_varargin_1[1] = 1U;
  }
  emxInit_uint32_T(sp, &tracksAtThisTime, 2, &ei_emlrtRTEI);
  emxInit_real_T(sp, &allStates, 2, &gi_emlrtRTEI);
  emxInit_real_T(sp, &allCovars, 3, &hi_emlrtRTEI);
  emxInit_boolean_T(sp, &x, 2, &ci_emlrtRTEI, true);
  for (j = 0; j < i1; j++) {
    real_T b_x[6];
    int32_T c_loop_ub;
    int32_T i2;
    boolean_T exitg1;
    if (j + 1 > i1) {
      emlrtDynamicBoundsCheckR2012b(j + 1, 1, i1, &ij_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    st.site = &wr_emlrtRSI;
    b_gaussEKFilter_predict(&st, centralTrack->pState,
                            centralTrack->pStateCovariance, obj->ProcessNoise,
                            uniqueTimes_data[j] - trackTime, b_x);
    st.site = &xr_emlrtRSI;
    ibtile = x->size[0] * x->size[1];
    x->size[0] = 1;
    loop_ub = allTimes->size[1];
    x->size[1] = allTimes->size[1];
    emxEnsureCapacity_boolean_T(&st, x, ibtile, &ci_emlrtRTEI);
    x_data = x->data;
    trackTime = uniqueTimes_data[j];
    for (b_i = 0; b_i < b_loop_ub; b_i++) {
      x_data[b_i] = (trackTime == allTimes_data[b_i]);
    }
    b_st.site = &am_emlrtRSI;
    c_st.site = &bm_emlrtRSI;
    idx = 0;
    ibtile = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = allTimes->size[1];
    emxEnsureCapacity_int32_T(&c_st, ii, ibtile, &di_emlrtRTEI);
    ii_data = ii->data;
    d_st.site = &cm_emlrtRSI;
    if (x->size[1] > 2147483646) {
      e_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    ibtile = 0;
    exitg1 = false;
    while ((!exitg1) && (ibtile <= loop_ub - 1)) {
      if (x_data[ibtile]) {
        idx++;
        ii_data[idx - 1] = ibtile + 1;
        if (idx >= loop_ub) {
          exitg1 = true;
        } else {
          ibtile++;
        }
      } else {
        ibtile++;
      }
    }
    if (idx > x->size[1]) {
      emlrtErrorWithMessageIdR2018a(&c_st, &db_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (x->size[1] == 1) {
      if (idx == 0) {
        ii->size[0] = 1;
        ii->size[1] = 0;
      }
    } else {
      ibtile = ii->size[0] * ii->size[1];
      if (idx < 1) {
        ii->size[1] = 0;
      } else {
        ii->size[1] = idx;
      }
      emxEnsureCapacity_int32_T(&c_st, ii, ibtile, &sg_emlrtRTEI);
      ii_data = ii->data;
    }
    ibtile = tracksAtThisTime->size[0] * tracksAtThisTime->size[1];
    tracksAtThisTime->size[0] = 1;
    c_loop_ub = ii->size[1];
    tracksAtThisTime->size[1] = ii->size[1];
    emxEnsureCapacity_uint32_T(&st, tracksAtThisTime, ibtile, &ei_emlrtRTEI);
    tracksAtThisTime_data = tracksAtThisTime->data;
    for (b_i = 0; b_i < c_loop_ub; b_i++) {
      tracksAtThisTime_data[b_i] = (uint32_T)ii_data[b_i];
    }
    st.site = &yr_emlrtRSI;
    indexShapeCheck(&st, b_I->size[1], tracksAtThisTime->size);
    for (b_i = 0; b_i < c_loop_ub; b_i++) {
      idx = (int32_T)tracksAtThisTime_data[b_i];
      if (idx > c_I) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, c_I, &oj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
    }
    st.site = &as_emlrtRSI;
    varargin_1[1] = (real_T)tracksAtThisTime->size[1] + 1.0;
    b_st.site = &kk_emlrtRSI;
    assertValidSizeArg(&b_st, varargin_1);
    idx = allStates->size[0] * allStates->size[1];
    allStates->size[0] = 6;
    i2 = tracksAtThisTime->size[1] + 1;
    allStates->size[1] = tracksAtThisTime->size[1] + 1;
    emxEnsureCapacity_real_T(&st, allStates, idx, &bh_emlrtRTEI);
    allStates_data = allStates->data;
    b_st.site = &nk_emlrtRSI;
    if (tracksAtThisTime->size[1] + 1 > 2147483646) {
      c_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (b_i = 0; b_i < i2; b_i++) {
      ibtile = b_i * 6;
      for (k = 0; k < 6; k++) {
        allStates_data[ibtile + k] = b_x[k];
      }
    }
    st.site = &bs_emlrtRSI;
    b_varargin_1[2] = (uint32_T)tracksAtThisTime->size[1] + 1U;
    b_st.site = &kk_emlrtRSI;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx < 3)) {
      if (b_varargin_1[idx] > 2147483647U) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &y_emlrtRTEI,
            "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector",
            "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector", 4, 12,
            MIN_int32_T, 12, MAX_int32_T);
      } else {
        idx++;
      }
    }
    if ((uint32_T)tracksAtThisTime->size[1] + 1U > 2147483647U) {
      emlrtErrorWithMessageIdR2018a(&b_st, &ab_emlrtRTEI,
                                    "Coder:MATLAB:pmaxsize",
                                    "Coder:MATLAB:pmaxsize", 0);
    }
    ibtile = allCovars->size[0] * allCovars->size[1] * allCovars->size[2];
    allCovars->size[0] = 6;
    allCovars->size[1] = 6;
    loop_ub = (int32_T)((uint32_T)tracksAtThisTime->size[1] + 1U);
    allCovars->size[2] = (int32_T)((uint32_T)tracksAtThisTime->size[1] + 1U);
    emxEnsureCapacity_real_T(&st, allCovars, ibtile, &bh_emlrtRTEI);
    allCovars_data = allCovars->data;
    b_st.site = &nk_emlrtRSI;
    if ((int32_T)((uint32_T)tracksAtThisTime->size[1] + 1U) > 2147483646) {
      c_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (b_i = 0; b_i < loop_ub; b_i++) {
      for (k = 0; k < 36; k++) {
        allCovars_data[k + b_i * 36] = centralTrack->pStateCovariance[k];
      }
    }
    for (b_k = 0; b_k < c_loop_ub; b_k++) {
      if (b_k + 1 > c_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, c_loop_ub, &pj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      idx =
          (int32_T)
              inAssigned_data[I_data[(int32_T)tracksAtThisTime_data[b_k] - 1] -
                              1] -
          1;
      if ((idx < 0) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 0, i, &hj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (((int32_T)((uint32_T)b_k + 2U) < 1) ||
          ((int32_T)((uint32_T)b_k + 2U) > i2)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_k + 2U), 1, i2,
                                      &jj_emlrtBCI, (emlrtConstCTX)sp);
      }
      for (b_i = 0; b_i < 6; b_i++) {
        allStates_data[b_i + 6 * (b_k + 1)] =
            sourceTracks_data
                [(int32_T)inAssigned_data
                     [I_data[(int32_T)tracksAtThisTime_data[b_k] - 1] - 1] -
                 1]
                    .pState[b_i];
      }
      if (b_k + 1 > c_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, c_loop_ub, &qj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 0, i, &gj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (((int32_T)((uint32_T)b_k + 2U) < 1) ||
          ((int32_T)((uint32_T)b_k + 2U) > loop_ub)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_k + 2U), 1, loop_ub,
                                      &kj_emlrtBCI, (emlrtConstCTX)sp);
      }
      for (b_i = 0; b_i < 6; b_i++) {
        for (k = 0; k < 6; k++) {
          idx = k + 6 * b_i;
          allCovars_data[idx + 36 * (b_k + 1)] =
              sourceTracks_data
                  [(int32_T)inAssigned_data
                       [I_data[(int32_T)tracksAtThisTime_data[b_k] - 1] - 1] -
                   1]
                      .pStateCovariance[idx];
        }
      }
    }
    st.site = &cs_emlrtRSI;
    fusecovint(&st, allStates, allCovars, obj->StateFusionParameters,
               centralTrack->pState, centralTrack->pStateCovariance);
    trackTime = uniqueTimes_data[j];
  }
  emxFree_int32_T(sp, &b_I);
  emxFree_real_T(sp, &allTimes);
  emxFree_int32_T(sp, &ii);
  emxFree_boolean_T(sp, &x);
  emxFree_real_T(sp, &allCovars);
  emxFree_real_T(sp, &allStates);
  emxFree_uint32_T(sp, &tracksAtThisTime);
  emxFree_real_T(sp, &uniqueTimes);
  st.site = &ds_emlrtRSI;
  b_st.site = &rc_emlrtRSI;
  validateattributes(&b_st, centralTrack->pState);
  st.site = &es_emlrtRSI;
  b_st.site = &es_emlrtRSI;
  ensurePosDefMatrix(&b_st, centralTrack->pStateCovariance);
  b_st.site = &sc_emlrtRSI;
  b_validateattributes(&b_st, centralTrack->pStateCovariance);
  b_st.site = &tc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&b_st, centralTrack->pStateCovariance);
  st.site = &fs_emlrtRSI;
  b_st.site = &qc_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  if (trackTime < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
  }
  c_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(trackTime) || muDoubleScalarIsNaN(trackTime)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
  }
  centralTrack->pUpdateTime = trackTime;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (Fuserxcov.c) */
