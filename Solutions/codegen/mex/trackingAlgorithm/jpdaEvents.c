/*
 * jpdaEvents.c
 *
 * Code generation for function 'jpdaEvents'
 *
 */

/* Include files */
#include "jpdaEvents.h"
#include "all.h"
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleJPDAEvents.h"
#include "indexShapeCheck.h"
#include "logsumexp.h"
#include "murtyKBestEvents.h"
#include "numPotentialFeasibleEvents.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo sdb_emlrtRSI = {
    94,                                               /* lineNo */
    "jpdaEvents",                                     /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/jpdaEvents.m" /* pathName */
};

static emlrtRSInfo tdb_emlrtRSI = {
    62,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo udb_emlrtRSI = {
    60,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo vdb_emlrtRSI = {
    48,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo wdb_emlrtRSI = {
    34,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo xdb_emlrtRSI = {
    31,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo ydb_emlrtRSI = {
    29,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo aeb_emlrtRSI = {
    19,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo beb_emlrtRSI = {
    17,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo ceb_emlrtRSI = {
    14,                /* lineNo */
    "kbestJPDAEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pathName */
};

static emlrtRSInfo yjb_emlrtRSI = {
    22,                           /* lineNo */
    "feasibleAndSortKBestEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pathName */
};

static emlrtRSInfo akb_emlrtRSI = {
    23,                           /* lineNo */
    "feasibleAndSortKBestEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pathName */
};

static emlrtRSInfo bkb_emlrtRSI = {
    25,                           /* lineNo */
    "feasibleAndSortKBestEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pathName */
};

static emlrtRSInfo ckb_emlrtRSI = {
    37,                           /* lineNo */
    "feasibleAndSortKBestEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pathName */
};

static emlrtRSInfo dkb_emlrtRSI = {
    38,                           /* lineNo */
    "feasibleAndSortKBestEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pathName */
};

static emlrtRSInfo ekb_emlrtRSI = {
    41,                           /* lineNo */
    "feasibleAndSortKBestEvents", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pathName */
};

static emlrtRSInfo pkb_emlrtRSI = {
    14,                             /* lineNo */
    "jointFeasibleEventLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pathName */
};

static emlrtRSInfo qkb_emlrtRSI = {
    18,                             /* lineNo */
    "jointFeasibleEventLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pathName */
};

static emlrtRSInfo rkb_emlrtRSI = {
    19,                             /* lineNo */
    "jointFeasibleEventLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pathName */
};

static emlrtRSInfo skb_emlrtRSI = {
    20,                             /* lineNo */
    "jointFeasibleEventLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pathName */
};

static emlrtRTEInfo ib_emlrtRTEI = {
    34,                /* lineNo */
    23,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo jb_emlrtRTEI = {
    31,                /* lineNo */
    23,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = {
    29,                /* lineNo */
    23,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = {
    24,                /* lineNo */
    23,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = {
    23,                /* lineNo */
    23,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtBCInfo td_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    37,                           /* lineNo */
    30,                           /* colNo */
    "",                           /* aName */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo ud_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    37,                           /* lineNo */
    34,                           /* colNo */
    "",                           /* aName */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo vd_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    41,                           /* lineNo */
    22,                           /* colNo */
    "",                           /* aName */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo wd_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    41,                           /* lineNo */
    26,                           /* colNo */
    "",                           /* aName */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m", /* pName */
    0                               /* checkKind */
};

static emlrtRTEInfo ob_emlrtRTEI = {
    14,                 /* lineNo */
    37,                 /* colNo */
    "validatepositive", /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+valattr/validatepositive.m" /* pName
                                                                            */
};

static emlrtBCInfo xd_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    17,                             /* lineNo */
    20,                             /* colNo */
    "",                             /* aName */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m", /* pName */
    0                                 /* checkKind */
};

static emlrtBCInfo yd_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    41,                           /* lineNo */
    18,                           /* colNo */
    "",                           /* aName */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo ae_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    20,                             /* lineNo */
    40,                             /* colNo */
    "",                             /* aName */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m", /* pName */
    0                                 /* checkKind */
};

static emlrtBCInfo be_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    20,                             /* lineNo */
    22,                             /* colNo */
    "",                             /* aName */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m", /* pName */
    0                                 /* checkKind */
};

static emlrtRTEInfo gf_emlrtRTEI = {
    29,                /* lineNo */
    27,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo hf_emlrtRTEI = {
    34,                /* lineNo */
    31,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo if_emlrtRTEI = {
    34,                /* lineNo */
    27,                /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo jf_emlrtRTEI = {
    42,                /* lineNo */
    5,                 /* colNo */
    "kbestJPDAEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestJPDAEvents.m" /* pName */
};

static emlrtRTEInfo kf_emlrtRTEI = {
    23,                           /* lineNo */
    16,                           /* colNo */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pName */
};

static emlrtRTEInfo lf_emlrtRTEI = {
    14,                             /* lineNo */
    16,                             /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo nf_emlrtRTEI = {
    18,                             /* lineNo */
    36,                             /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo of_emlrtRTEI = {
    18,                             /* lineNo */
    5,                              /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo pf_emlrtRTEI = {
    37,                           /* lineNo */
    1,                            /* colNo */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pName */
};

static emlrtRTEInfo qf_emlrtRTEI = {
    38,                           /* lineNo */
    16,                           /* colNo */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pName */
};

static emlrtRTEInfo rf_emlrtRTEI = {
    17,                             /* lineNo */
    12,                             /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo sf_emlrtRTEI = {
    19,                             /* lineNo */
    5,                              /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo tf_emlrtRTEI = {
    20,                             /* lineNo */
    40,                             /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo uf_emlrtRTEI = {
    20,                             /* lineNo */
    31,                             /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

static emlrtRTEInfo vf_emlrtRTEI = {
    41,                           /* lineNo */
    1,                            /* colNo */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pName */
};

static emlrtRTEInfo wf_emlrtRTEI = {
    22,                           /* lineNo */
    1,                            /* colNo */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pName */
};

static emlrtRTEInfo xf_emlrtRTEI = {
    23,                           /* lineNo */
    1,                            /* colNo */
    "feasibleAndSortKBestEvents", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "feasibleAndSortKBestEvents.m" /* pName */
};

static emlrtRTEInfo yf_emlrtRTEI = {
    14,                             /* lineNo */
    1,                              /* colNo */
    "jointFeasibleEventLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointFeasibleEventLikelihood.m" /* pName */
};

/* Function Definitions */
void jpdaEvents(const emlrtStack *sp, const emxArray_real_T *varargin_1,
                real_T varargin_2, emxArray_boolean_T *varargout_1,
                emxArray_real_T *varargout_2)
{
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_boolean_T c_varargin_1_data;
  emxArray_boolean_T *FJEPaddedi;
  emxArray_boolean_T *allFJE;
  emxArray_boolean_T *b_allFJE;
  emxArray_boolean_T *isTrackUnassigned;
  emxArray_boolean_T *validationMatrix;
  emxArray_int32_T *iidx;
  emxArray_int32_T *nz;
  emxArray_real_T *b_varargin_1;
  emxArray_real_T *logkposterior;
  emxArray_real_T *loglhood;
  emxArray_real_T *logposterior;
  emxArray_real_T *r;
  const real_T *varargin_1_data;
  real_T *logkposterior_data;
  real_T *loglhood_data;
  real_T *logposterior_data;
  int32_T sizes[2];
  int32_T b_i;
  int32_T b_k;
  int32_T b_loglhood;
  int32_T b_loop_ub;
  int32_T c_k;
  int32_T c_loop_ub;
  int32_T d_loop_ub;
  int32_T e_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T jpdaEvents_numThreads;
  int32_T k;
  int32_T kFound;
  int32_T loop_ub;
  int32_T nPotentials;
  int32_T nx;
  int32_T varargin_1_size;
  int32_T z;
  int32_T *nz_data;
  boolean_T b_varargin_1_data[51];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T p;
  boolean_T *FJEPaddedi_data;
  boolean_T *allFJE_data;
  boolean_T *isTrackUnassigned_data;
  boolean_T *validationMatrix_data;
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
  varargin_1_data = varargin_1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &sdb_emlrtRSI;
  b_st.site = &ceb_emlrtRSI;
  c_st.site = &ge_emlrtRSI;
  p = true;
  nx = varargin_1->size[0] * varargin_1->size[1];
  nPotentials = 0;
  exitg1 = false;
  while ((!exitg1) && (nPotentials <= nx - 1)) {
    if ((!muDoubleScalarIsInf(varargin_1_data[nPotentials])) &&
        (!muDoubleScalarIsNaN(varargin_1_data[nPotentials]))) {
      nPotentials++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:jpdaEvents:expectedFinite", 3, 4, 16, "likelihoodMatrix");
  }
  c_st.site = &ge_emlrtRSI;
  p = true;
  nPotentials = 0;
  exitg1 = false;
  while ((!exitg1) && (nPotentials <= nx - 1)) {
    if (!(varargin_1_data[nPotentials] < 0.0)) {
      nPotentials++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &nb_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:jpdaEvents:expectedNonnegative", 3, 4, 16, "likelihoodMatrix");
  }
  b_st.site = &beb_emlrtRSI;
  c_st.site = &ge_emlrtRSI;
  if (varargin_2 <= 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &ob_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedPositive",
        "MATLAB:jpdaEvents:expectedPositive", 3, 4, 1, "k");
  }
  p = ((!muDoubleScalarIsInf(varargin_2)) &&
       (!muDoubleScalarIsNaN(varargin_2)));
  if (p) {
    b_st.site = &aeb_emlrtRSI;
    c_st.site = &ge_emlrtRSI;
    if (!(muDoubleScalarFloor(varargin_2) == varargin_2)) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &pb_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedInteger",
          "MATLAB:jpdaEvents:expectedInteger", 3, 4, 1, "k");
    }
  }
  if (varargin_1->size[0] <= 1) {
    emlrtErrorWithMessageIdR2018a(
        &st, &mb_emlrtRTEI, "fusion:jpdaEvents:expectedLikelihoodMoreRows",
        "fusion:jpdaEvents:expectedLikelihoodMoreRows", 0);
  }
  if (varargin_1->size[1] <= 1) {
    emlrtErrorWithMessageIdR2018a(
        &st, &lb_emlrtRTEI, "fusion:jpdaEvents:expectedLikelihoodMoreColumns",
        "fusion:jpdaEvents:expectedLikelihoodMoreColumns", 0);
  }
  loop_ub = varargin_1->size[1];
  emxInit_boolean_T(&st, &isTrackUnassigned, 2, &of_emlrtRTEI, true);
  nPotentials = isTrackUnassigned->size[0] * isTrackUnassigned->size[1];
  isTrackUnassigned->size[0] = 1;
  z = varargin_1->size[1] - 1;
  isTrackUnassigned->size[1] = varargin_1->size[1] - 1;
  emxEnsureCapacity_boolean_T(&st, isTrackUnassigned, nPotentials,
                              &gf_emlrtRTEI);
  isTrackUnassigned_data = isTrackUnassigned->data;
  nPotentials = varargin_1->size[1];
  if (varargin_1->size[1] - 1 < 1600) {
    for (i = 0; i <= loop_ub - 2; i++) {
      isTrackUnassigned_data[i] =
          (varargin_1_data[varargin_1->size[0] * (i + 1)] > 0.0);
    }
  } else {
    emlrtEnterParallelRegion(&st, omp_in_parallel());
    emlrtPushJmpBuf(&st, &emlrtJBStack);
    jpdaEvents_numThreads = emlrtAllocRegionTLSs(
        st.tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(jpdaEvents_numThreads)

    for (i = 0; i <= nPotentials - 2; i++) {
      isTrackUnassigned_data[i] =
          (varargin_1_data[varargin_1->size[0] * (i + 1)] > 0.0);
    }
    emlrtPopJmpBuf(&st, &emlrtJBStack);
    emlrtExitParallelRegion(&st, omp_in_parallel());
  }
  b_st.site = &ydb_emlrtRSI;
  if (!all(&b_st, isTrackUnassigned)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &kb_emlrtRTEI,
        "fusion:jpdaEvents:expectedPositiveTrackUnassignment",
        "fusion:jpdaEvents:expectedPositiveTrackUnassignment", 0);
  }
  kFound = varargin_1->size[0];
  b_loop_ub = varargin_1->size[0] - 1;
  varargin_1_size = varargin_1->size[0] - 1;
  for (k = 0; k <= kFound - 2; k++) {
    b_varargin_1_data[k] = (varargin_1_data[k + 1] > 0.0);
  }
  c_varargin_1_data.data = &b_varargin_1_data[0];
  c_varargin_1_data.size = &varargin_1_size;
  c_varargin_1_data.allocatedSize = 51;
  c_varargin_1_data.numDimensions = 1;
  c_varargin_1_data.canFreeData = false;
  b_st.site = &xdb_emlrtRSI;
  if (!b_all(&b_st, &c_varargin_1_data)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &jb_emlrtRTEI,
        "fusion:jpdaEvents:expectedPositiveDetectionUnassignment",
        "fusion:jpdaEvents:expectedPositiveDetectionUnassignment", 0);
  }
  emxInit_real_T(&st, &b_varargin_1, 2, &hf_emlrtRTEI, true);
  nPotentials = b_varargin_1->size[0] * b_varargin_1->size[1];
  b_varargin_1->size[0] = varargin_1->size[0] - 1;
  b_varargin_1->size[1] = z;
  emxEnsureCapacity_real_T(&st, b_varargin_1, nPotentials, &hf_emlrtRTEI);
  logkposterior_data = b_varargin_1->data;
  for (k = 0; k <= loop_ub - 2; k++) {
    for (b_i = 0; b_i <= kFound - 2; b_i++) {
      logkposterior_data[b_i + b_varargin_1->size[0] * k] =
          varargin_1_data[(b_i + varargin_1->size[0] * (k + 1)) + 1];
    }
  }
  emxInit_real_T(&st, &r, 2, &if_emlrtRTEI, true);
  b_st.site = &wdb_emlrtRSI;
  sum(&b_st, b_varargin_1, r);
  logkposterior_data = r->data;
  emxFree_real_T(&st, &b_varargin_1);
  nPotentials = isTrackUnassigned->size[0] * isTrackUnassigned->size[1];
  isTrackUnassigned->size[0] = 1;
  c_loop_ub = r->size[1];
  isTrackUnassigned->size[1] = r->size[1];
  emxEnsureCapacity_boolean_T(&st, isTrackUnassigned, nPotentials,
                              &if_emlrtRTEI);
  isTrackUnassigned_data = isTrackUnassigned->data;
  nPotentials = r->size[1];
  if (r->size[1] < 1600) {
    for (i1 = 0; i1 < c_loop_ub; i1++) {
      isTrackUnassigned_data[i1] = (logkposterior_data[i1] >= 0.0);
    }
  } else {
    emlrtEnterParallelRegion(&st, omp_in_parallel());
    emlrtPushJmpBuf(&st, &emlrtJBStack);
    jpdaEvents_numThreads = emlrtAllocRegionTLSs(
        st.tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(jpdaEvents_numThreads)

    for (i1 = 0; i1 < nPotentials; i1++) {
      isTrackUnassigned_data[i1] = (logkposterior_data[i1] >= 0.0);
    }
    emlrtPopJmpBuf(&st, &emlrtJBStack);
    emlrtExitParallelRegion(&st, omp_in_parallel());
  }
  emxFree_real_T(&st, &r);
  b_st.site = &wdb_emlrtRSI;
  if (!all(&b_st, isTrackUnassigned)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &ib_emlrtRTEI, "fusion:jpdaEvents:expectedNonzeroLikelihoodColumn",
        "fusion:jpdaEvents:expectedNonzeroLikelihoodColumn", 0);
  }
  emxInit_boolean_T(&st, &validationMatrix, 2, &jf_emlrtRTEI, true);
  nPotentials = validationMatrix->size[0] * validationMatrix->size[1];
  validationMatrix->size[0] = b_loop_ub;
  validationMatrix->size[1] = loop_ub;
  emxEnsureCapacity_boolean_T(&st, validationMatrix, nPotentials,
                              &jf_emlrtRTEI);
  validationMatrix_data = validationMatrix->data;
  for (k = 0; k < loop_ub; k++) {
    for (b_i = 0; b_i <= kFound - 2; b_i++) {
      validationMatrix_data[b_i + validationMatrix->size[0] * k] =
          (varargin_1_data[(b_i + varargin_1->size[0] * k) + 1] > 0.0);
    }
  }
  b_st.site = &vdb_emlrtRSI;
  nPotentials = numPotentialFeasibleEvents(&b_st, validationMatrix,
                                           varargin_1->size[0] - 1,
                                           varargin_1->size[1] - 1);
  emxInit_boolean_T(&st, &allFJE, 3, &wf_emlrtRTEI, true);
  emxInit_real_T(&st, &logposterior, 1, &xf_emlrtRTEI, true);
  emxInit_real_T(&st, &logkposterior, 1, &pf_emlrtRTEI, true);
  emxInit_int32_T(&st, &iidx, 1, &tf_emlrtRTEI);
  emxInit_real_T(&st, &loglhood, 1, &yf_emlrtRTEI, true);
  emxInit_boolean_T(&st, &FJEPaddedi, 2, &sf_emlrtRTEI, true);
  emxInit_int32_T(&st, &nz, 2, &ag_emlrtRTEI);
  emxInit_boolean_T(&st, &b_allFJE, 2, &nf_emlrtRTEI, true);
  guard1 = false;
  if (p) {
    if (nPotentials >= 0) {
      c_loop_ub = nPotentials;
    } else if (nPotentials == MIN_int32_T) {
      c_loop_ub = MAX_int32_T;
    } else {
      c_loop_ub = -nPotentials;
    }
    z = (int32_T)((uint32_T)c_loop_ub / 10U);
    c_loop_ub -= z * 10;
    if (c_loop_ub >= 5) {
      z++;
    }
    if (nPotentials < 0) {
      z = -z;
    }
    if ((varargin_2 < z) ||
        ((varargin_1->size[1] - 1 > 8) && (varargin_1->size[0] - 1 > 8))) {
      b_st.site = &udb_emlrtRSI;
      murtyKBestEvents(&b_st, varargin_1, varargin_2, varargout_1, varargout_2);
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    real_T d;
    boolean_T b_p;
    b_st.site = &tdb_emlrtRSI;
    c_st.site = &yjb_emlrtRSI;
    feasibleJPDAEvents(&c_st, validationMatrix, allFJE);
    allFJE_data = allFJE->data;
    c_st.site = &akb_emlrtRSI;
    loop_ub = allFJE->size[2];
    nPotentials = logposterior->size[0];
    logposterior->size[0] = allFJE->size[2];
    emxEnsureCapacity_real_T(&c_st, logposterior, nPotentials, &kf_emlrtRTEI);
    logposterior_data = logposterior->data;
    d_st.site = &pkb_emlrtRSI;
    nPotentials = loglhood->size[0];
    loglhood->size[0] = nx;
    emxEnsureCapacity_real_T(&d_st, loglhood, nPotentials, &lf_emlrtRTEI);
    loglhood_data = loglhood->data;
    for (k = 0; k < nx; k++) {
      loglhood_data[k] = varargin_1_data[k];
    }
    b_p = false;
    for (k = 0; k < nx; k++) {
      if (b_p || (varargin_1_data[k] < 0.0)) {
        b_p = true;
      }
    }
    if (b_p) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
    }
    e_st.site = &ffb_emlrtRSI;
    f_st.site = &ns_emlrtRSI;
    if (nx > 2147483646) {
      g_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&g_st);
    }
    if (nx < 1600) {
      for (b_k = 0; b_k < nx; b_k++) {
        loglhood_data[b_k] = muDoubleScalarLog(loglhood_data[b_k]);
      }
    } else {
      emlrtEnterParallelRegion(&e_st, omp_in_parallel());
      emlrtPushJmpBuf(&e_st, &emlrtJBStack);
      jpdaEvents_numThreads =
          emlrtAllocRegionTLSs(e_st.tls, omp_in_parallel(),
                               omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(jpdaEvents_numThreads)

      for (b_k = 0; b_k < nx; b_k++) {
        loglhood_data[b_k] = muDoubleScalarLog(loglhood_data[b_k]);
      }
      emlrtPopJmpBuf(&e_st, &emlrtJBStack);
      emlrtExitParallelRegion(&e_st, omp_in_parallel());
    }
    if (allFJE->size[2] - 1 >= 0) {
      if (allFJE->size[1] < 2) {
        i2 = 0;
        i3 = 0;
      } else {
        i2 = 1;
        i3 = allFJE->size[1];
      }
      d_loop_ub = i3 - i2;
      e_loop_ub = allFJE->size[1];
      if (allFJE->size[1] != 0) {
        sizes[0] = (int8_T)allFJE->size[0];
      } else {
        sizes[0] = 0;
      }
      b_loglhood = nx;
    }
    for (b_i = 0; b_i < loop_ub; b_i++) {
      if (b_i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, loop_ub, &xd_emlrtBCI, &c_st);
      }
      d_st.site = &qkb_emlrtRSI;
      e_st.site = &gdb_emlrtRSI;
      z = allFJE->size[0];
      nPotentials = b_allFJE->size[0] * b_allFJE->size[1];
      b_allFJE->size[0] = allFJE->size[0];
      b_allFJE->size[1] = i3 - i2;
      emxEnsureCapacity_boolean_T(&e_st, b_allFJE, nPotentials, &nf_emlrtRTEI);
      isTrackUnassigned_data = b_allFJE->data;
      for (k = 0; k < d_loop_ub; k++) {
        for (i4 = 0; i4 < z; i4++) {
          isTrackUnassigned_data[i4 + b_allFJE->size[0] * k] =
              allFJE_data[(i4 + allFJE->size[0] * (i2 + k)) +
                          allFJE->size[0] * allFJE->size[1] * b_i];
        }
      }
      f_st.site = &ow_emlrtRSI;
      b_combineVectorElements(&f_st, b_allFJE, nz);
      nz_data = nz->data;
      nPotentials = isTrackUnassigned->size[0] * isTrackUnassigned->size[1];
      isTrackUnassigned->size[0] = 1;
      c_loop_ub = nz->size[1] + 1;
      isTrackUnassigned->size[1] = nz->size[1] + 1;
      emxEnsureCapacity_boolean_T(&c_st, isTrackUnassigned, nPotentials,
                                  &of_emlrtRTEI);
      isTrackUnassigned_data = isTrackUnassigned->data;
      isTrackUnassigned_data[0] = false;
      nPotentials = nz->size[1];
      for (k = 0; k < nPotentials; k++) {
        isTrackUnassigned_data[k + 1] = (nz_data[k] == 0);
      }
      d_st.site = &rkb_emlrtRSI;
      e_st.site = &mkb_emlrtRSI;
      f_st.site = &bq_emlrtRSI;
      if ((allFJE->size[1] != isTrackUnassigned->size[1]) &&
          (allFJE->size[1] != 0)) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &m_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
            "MATLAB:catenate:matrixDimensionMismatch", 0);
      }
      nPotentials = validationMatrix->size[0] * validationMatrix->size[1];
      validationMatrix->size[0] = allFJE->size[0];
      validationMatrix->size[1] = allFJE->size[1];
      emxEnsureCapacity_boolean_T(&e_st, validationMatrix, nPotentials,
                                  &rf_emlrtRTEI);
      validationMatrix_data = validationMatrix->data;
      for (k = 0; k < e_loop_ub; k++) {
        for (i4 = 0; i4 < z; i4++) {
          validationMatrix_data[i4 + validationMatrix->size[0] * k] =
              allFJE_data[(i4 + allFJE->size[0] * k) +
                          allFJE->size[0] * allFJE->size[1] * b_i];
        }
      }
      nPotentials = FJEPaddedi->size[0] * FJEPaddedi->size[1];
      FJEPaddedi->size[0] = sizes[0] + 1;
      FJEPaddedi->size[1] = nz->size[1] + 1;
      emxEnsureCapacity_boolean_T(&e_st, FJEPaddedi, nPotentials,
                                  &sf_emlrtRTEI);
      FJEPaddedi_data = FJEPaddedi->data;
      for (k = 0; k < c_loop_ub; k++) {
        FJEPaddedi_data[FJEPaddedi->size[0] * k] = isTrackUnassigned_data[k];
        nPotentials = sizes[0];
        for (i4 = 0; i4 < nPotentials; i4++) {
          FJEPaddedi_data[(i4 + FJEPaddedi->size[0] * k) + 1] =
              validationMatrix_data[i4 + sizes[0] * k];
        }
      }
      z = FJEPaddedi->size[0] * FJEPaddedi->size[1];
      c_loop_ub = 0;
      for (k = 0; k < z; k++) {
        if (FJEPaddedi_data[k]) {
          c_loop_ub++;
        }
      }
      nPotentials = iidx->size[0];
      iidx->size[0] = c_loop_ub;
      emxEnsureCapacity_int32_T(&c_st, iidx, nPotentials, &tf_emlrtRTEI);
      nz_data = iidx->data;
      nPotentials = 0;
      for (k = 0; k < z; k++) {
        if (FJEPaddedi_data[k]) {
          nz_data[nPotentials] = k;
          nPotentials++;
        }
      }
      c_loop_ub = iidx->size[0];
      nPotentials = logkposterior->size[0];
      logkposterior->size[0] = iidx->size[0];
      emxEnsureCapacity_real_T(&c_st, logkposterior, nPotentials,
                               &uf_emlrtRTEI);
      logkposterior_data = logkposterior->data;
      for (k = 0; k < c_loop_ub; k++) {
        if (nz_data[k] > b_loglhood - 1) {
          emlrtDynamicBoundsCheckR2012b(nz_data[k], 0, b_loglhood - 1,
                                        &ae_emlrtBCI, &c_st);
        }
        logkposterior_data[k] = loglhood_data[nz_data[k]];
      }
      if (b_i + 1 > logposterior->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, logposterior->size[0],
                                      &be_emlrtBCI, &c_st);
      }
      d_st.site = &skb_emlrtRSI;
      logposterior_data[b_i] = c_sum(&d_st, logkposterior);
    }
    c_st.site = &bkb_emlrtRSI;
    d_st.site = &ao_emlrtRSI;
    c_sort(&d_st, logposterior, iidx);
    nz_data = iidx->data;
    logposterior_data = logposterior->data;
    b_loop_ub = iidx->size[0];
    nPotentials = loglhood->size[0];
    loglhood->size[0] = iidx->size[0];
    emxEnsureCapacity_real_T(&c_st, loglhood, nPotentials, &mf_emlrtRTEI);
    loglhood_data = loglhood->data;
    for (k = 0; k < b_loop_ub; k++) {
      loglhood_data[k] = nz_data[k];
    }
    if (p) {
      if ((int32_T)varargin_2 > logposterior->size[0]) {
        kFound = logposterior->size[0];
      } else {
        kFound = (int32_T)varargin_2;
      }
    } else {
      kFound = logposterior->size[0];
    }
    if (kFound < 1) {
      z = 0;
    } else {
      if (logposterior->size[0] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, logposterior->size[0], &td_emlrtBCI,
                                      &b_st);
      }
      if (kFound > logposterior->size[0]) {
        emlrtDynamicBoundsCheckR2012b(kFound, 1, logposterior->size[0],
                                      &ud_emlrtBCI, &b_st);
      }
      z = kFound;
    }
    sizes[0] = 1;
    sizes[1] = z;
    c_st.site = &ckb_emlrtRSI;
    b_indexShapeCheck(&c_st, logposterior->size[0], sizes);
    nPotentials = logkposterior->size[0];
    logkposterior->size[0] = z;
    emxEnsureCapacity_real_T(&b_st, logkposterior, nPotentials, &pf_emlrtRTEI);
    logkposterior_data = logkposterior->data;
    for (k = 0; k < z; k++) {
      logkposterior_data[k] = logposterior_data[k];
    }
    c_st.site = &dkb_emlrtRSI;
    d_st.site = &dkb_emlrtRSI;
    d = b_logsumexp(&d_st, logkposterior);
    nPotentials = varargout_2->size[0];
    varargout_2->size[0] = z;
    emxEnsureCapacity_real_T(&c_st, varargout_2, nPotentials, &qf_emlrtRTEI);
    logkposterior_data = varargout_2->data;
    nPotentials = (z / 2) << 1;
    c_loop_ub = nPotentials - 2;
    for (k = 0; k <= c_loop_ub; k += 2) {
      __m128d r1;
      r1 = _mm_loadu_pd(&logposterior_data[k]);
      _mm_storeu_pd(&logkposterior_data[k], _mm_sub_pd(r1, _mm_set1_pd(d)));
    }
    for (k = nPotentials; k < z; k++) {
      logkposterior_data[k] = logposterior_data[k] - d;
    }
    d_st.site = &mw_emlrtRSI;
    nPotentials = varargout_2->size[0];
    e_st.site = &ns_emlrtRSI;
    if (varargout_2->size[0] > 2147483646) {
      f_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }
    if (varargout_2->size[0] < 1600) {
      for (c_k = 0; c_k < nPotentials; c_k++) {
        logkposterior_data[c_k] = muDoubleScalarExp(logkposterior_data[c_k]);
      }
    } else {
      emlrtEnterParallelRegion(&d_st, omp_in_parallel());
      emlrtPushJmpBuf(&d_st, &emlrtJBStack);
      jpdaEvents_numThreads =
          emlrtAllocRegionTLSs(d_st.tls, omp_in_parallel(),
                               omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(jpdaEvents_numThreads)

      for (c_k = 0; c_k < nPotentials; c_k++) {
        logkposterior_data[c_k] = muDoubleScalarExp(logkposterior_data[c_k]);
      }
      emlrtPopJmpBuf(&d_st, &emlrtJBStack);
      emlrtExitParallelRegion(&d_st, omp_in_parallel());
    }
    if (kFound < 1) {
      kFound = 0;
    } else {
      if (b_loop_ub < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, b_loop_ub, &vd_emlrtBCI, &b_st);
      }
      if (kFound > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(kFound, 1, b_loop_ub, &wd_emlrtBCI,
                                      &b_st);
      }
    }
    sizes[0] = 1;
    sizes[1] = kFound;
    c_st.site = &ekb_emlrtRSI;
    b_indexShapeCheck(&c_st, loglhood->size[0], sizes);
    c_loop_ub = allFJE->size[0];
    nPotentials =
        varargout_1->size[0] * varargout_1->size[1] * varargout_1->size[2];
    varargout_1->size[0] = allFJE->size[0];
    z = allFJE->size[1];
    varargout_1->size[1] = allFJE->size[1];
    varargout_1->size[2] = kFound;
    emxEnsureCapacity_boolean_T(&b_st, varargout_1, nPotentials, &vf_emlrtRTEI);
    isTrackUnassigned_data = varargout_1->data;
    for (k = 0; k < kFound; k++) {
      for (b_i = 0; b_i < z; b_i++) {
        for (i4 = 0; i4 < c_loop_ub; i4++) {
          nPotentials = (int32_T)loglhood_data[k];
          if ((nPotentials < 1) || (nPotentials > loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(nPotentials, 1, loop_ub, &yd_emlrtBCI,
                                          &b_st);
          }
          isTrackUnassigned_data[(i4 + varargout_1->size[0] * b_i) +
                                 varargout_1->size[0] * varargout_1->size[1] *
                                     k] =
              allFJE_data[(i4 + allFJE->size[0] * b_i) + allFJE->size[0] *
                                                             allFJE->size[1] *
                                                             (nPotentials - 1)];
        }
      }
    }
  }
  emxFree_boolean_T(&st, &b_allFJE);
  emxFree_int32_T(&st, &nz);
  emxFree_boolean_T(&st, &FJEPaddedi);
  emxFree_boolean_T(&st, &isTrackUnassigned);
  emxFree_real_T(&st, &loglhood);
  emxFree_int32_T(&st, &iidx);
  emxFree_real_T(&st, &logkposterior);
  emxFree_real_T(&st, &logposterior);
  emxFree_boolean_T(&st, &allFJE);
  emxFree_boolean_T(&st, &validationMatrix);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (jpdaEvents.c) */
