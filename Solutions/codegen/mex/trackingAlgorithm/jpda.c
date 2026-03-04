/*
 * jpda.c
 *
 * Code generation for function 'jpda'
 *
 */

/* Include files */
#include "jpda.h"
#include "eml_int_forloop_overflow_check.h"
#include "jpdaEvents.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo at_emlrtRSI = {
    114,                                                           /* lineNo */
    "computeDimsData",                                             /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m" /* pathName
                                                                    */
};

static emlrtRSInfo qdb_emlrtRSI =
    {
        13,     /* lineNo */
        "jpda", /* fcnName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
        "jpda.m" /* pathName */
};

static emlrtRSInfo rdb_emlrtRSI =
    {
        16,     /* lineNo */
        "jpda", /* fcnName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
        "jpda.m" /* pathName */
};

static emlrtRSInfo oeb_emlrtRSI = {
    112,                /* lineNo */
    "blockedSummation", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo peb_emlrtRSI = {
    173,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo qeb_emlrtRSI = {
    190,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo reb_emlrtRSI = {
    192,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo seb_emlrtRSI = {
    204,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo teb_emlrtRSI = {
    207,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo ueb_emlrtRSI = {
    225,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo veb_emlrtRSI = {
    227,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo web_emlrtRSI = {
    238,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pathName
                                                                         */
};

static emlrtRSInfo wkb_emlrtRSI = {
    34,                       /* lineNo */
    "jointEventsToPosterior", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pathName */
};

static emlrtRSInfo xkb_emlrtRSI = {
    23,                       /* lineNo */
    "jointEventsToPosterior", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pathName */
};

static emlrtECInfo n_emlrtECI = {
    -1,                       /* nDims */
    34,                       /* lineNo */
    1,                        /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtBCInfo rd_emlrtBCI = {
    -1,                       /* iFirst */
    -1,                       /* iLast */
    34,                       /* lineNo */
    46,                       /* colNo */
    "",                       /* aName */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m", /* pName */
    0                           /* checkKind */
};

static emlrtECInfo o_emlrtECI = {
    -1,                       /* nDims */
    23,                       /* lineNo */
    5,                        /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtBCInfo sd_emlrtBCI = {
    -1,                       /* iFirst */
    -1,                       /* iLast */
    23,                       /* lineNo */
    17,                       /* colNo */
    "",                       /* aName */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m", /* pName */
    0                           /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = {
    13,                       /* lineNo */
    34,                       /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m", /* pName */
    4                           /* checkKind */
};

static emlrtRTEInfo ue_emlrtRTEI =
    {
        16,     /* lineNo */
        1,      /* colNo */
        "jpda", /* fName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
        "jpda.m" /* pName */
};

static emlrtRTEInfo ve_emlrtRTEI = {
    23,                       /* lineNo */
    32,                       /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtRTEInfo we_emlrtRTEI = {
    23,                       /* lineNo */
    28,                       /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtRTEInfo xe_emlrtRTEI = {
    146,                /* lineNo */
    24,                 /* colNo */
    "blockedSummation", /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pName
                                                                         */
};

static emlrtRTEInfo ye_emlrtRTEI = {
    153,                /* lineNo */
    23,                 /* colNo */
    "blockedSummation", /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pName
                                                                         */
};

static emlrtRTEInfo af_emlrtRTEI = {
    34,                       /* lineNo */
    34,                       /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtRTEInfo bf_emlrtRTEI = {
    34,                       /* lineNo */
    26,                       /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtRTEInfo cf_emlrtRTEI = {
    34,                       /* lineNo */
    20,                       /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtRTEInfo df_emlrtRTEI =
    {
        1,      /* lineNo */
        22,     /* colNo */
        "jpda", /* fName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
        "jpda.m" /* pName */
};

static emlrtRTEInfo ef_emlrtRTEI = {
    23,                       /* lineNo */
    5,                        /* colNo */
    "jointEventsToPosterior", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "jointEventsToPosterior.m" /* pName */
};

static emlrtRTEInfo ff_emlrtRTEI = {
    153,                /* lineNo */
    1,                  /* colNo */
    "blockedSummation", /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/blockedSummation.m" /* pName
                                                                         */
};

/* Function Definitions */
void jpda(const emlrtStack *sp, const emxArray_real_T *likelihoodMatrix,
          real_T maxNumEvents, emxArray_real_T *posterior)
{
  __m128d r2;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  emxArray_boolean_T *FJE;
  emxArray_real_T *b_posterior;
  emxArray_real_T *bsum;
  emxArray_real_T *r;
  emxArray_real_T *varargin_2;
  emxArray_real_T *x;
  real_T c_varargin_2;
  real_T *bsum_data;
  real_T *posterior_data;
  real_T *r1;
  real_T *x_data;
  int32_T b_FJE[2];
  int32_T b_loop_ub;
  int32_T b_xj;
  int32_T c_loop_ub;
  int32_T c_xj;
  int32_T d_loop_ub;
  int32_T firstBlockLength;
  int32_T i;
  int32_T ib;
  int32_T jpda_numThreads;
  int32_T loop_ub;
  int32_T n;
  int32_T nx;
  int32_T xblockoffset;
  int32_T xj;
  boolean_T *FJE_data;
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &FJE, 3, &df_emlrtRTEI, true);
  emxInit_real_T(sp, &bsum, 1, &ff_emlrtRTEI, true);
  st.site = &qdb_emlrtRSI;
  jpdaEvents(&st, likelihoodMatrix, maxNumEvents, FJE, bsum);
  bsum_data = bsum->data;
  FJE_data = FJE->data;
  st.site = &rdb_emlrtRSI;
  if (FJE->size[1] - 1 < 0) {
    emlrtNonNegativeCheckR2012b(-1.0, &d_emlrtDCI, &st);
  }
  loop_ub = FJE->size[0] + 1;
  nx = posterior->size[0] * posterior->size[1];
  posterior->size[0] = FJE->size[0] + 1;
  b_loop_ub = FJE->size[1] - 1;
  posterior->size[1] = FJE->size[1] - 1;
  emxEnsureCapacity_real_T(&st, posterior, nx, &ue_emlrtRTEI);
  posterior_data = posterior->data;
  nx = (FJE->size[0] + 1) * (FJE->size[1] - 1);
  for (xj = 0; xj < nx; xj++) {
    posterior_data[xj] = 0.0;
  }
  if (FJE->size[1] < 2) {
    xblockoffset = 0;
    firstBlockLength = 0;
  } else {
    xblockoffset = 1;
    firstBlockLength = FJE->size[1];
  }
  b_st.site = &xkb_emlrtRSI;
  nx = bsum->size[0];
  c_st.site = &ys_emlrtRSI;
  d_st.site = &at_emlrtRSI;
  n = bsum->size[0];
  if (bsum->size[0] < 1) {
    n = 1;
  }
  if (FJE->size[2] > muIntScalarMax_sint32(nx, n)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &p_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (bsum->size[0] != FJE->size[2]) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &q_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  c_loop_ub = FJE->size[2];
  d_loop_ub = FJE->size[0];
  if ((FJE->size[0] < 1) || (FJE->size[0] > FJE->size[0] + 1)) {
    emlrtDynamicBoundsCheckR2012b(FJE->size[0], 1, FJE->size[0] + 1,
                                  &sd_emlrtBCI, &st);
  }
  b_st.site = &xkb_emlrtRSI;
  emxInit_real_T(&b_st, &x, 3, &ve_emlrtRTEI, true);
  nx = x->size[0] * x->size[1] * x->size[2];
  x->size[0] = FJE->size[0];
  n = firstBlockLength - xblockoffset;
  x->size[1] = n;
  x->size[2] = FJE->size[2];
  emxEnsureCapacity_real_T(&b_st, x, nx, &ve_emlrtRTEI);
  x_data = x->data;
  for (xj = 0; xj < c_loop_ub; xj++) {
    for (b_xj = 0; b_xj < n; b_xj++) {
      for (c_xj = 0; c_xj < d_loop_ub; c_xj++) {
        x_data[(c_xj + x->size[0] * b_xj) + x->size[0] * x->size[1] * xj] =
            (real_T)FJE_data[(c_xj + FJE->size[0] * (xblockoffset + b_xj)) +
                             FJE->size[0] * FJE->size[1] * xj] *
            bsum_data[xj];
      }
    }
  }
  c_st.site = &nw_emlrtRSI;
  d_st.site = &ow_emlrtRSI;
  e_st.site = &pw_emlrtRSI;
  emxInit_real_T(&e_st, &r, 2, &ef_emlrtRTEI, true);
  if ((x->size[0] == 0) || (x->size[1] == 0) || (x->size[2] == 0)) {
    nx = r->size[0] * r->size[1];
    r->size[0] = x->size[0];
    r->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&e_st, r, nx, &we_emlrtRTEI);
    r1 = r->data;
    nx = x->size[0] * x->size[1];
    for (xj = 0; xj < nx; xj++) {
      r1[xj] = 0.0;
    }
  } else {
    __m128d r3;
    int32_T bvstride;
    int32_T lastBlockLength;
    int32_T nblocks;
    int32_T vstride;
    int32_T xoffset;
    f_st.site = &oeb_emlrtRSI;
    vstride = x->size[0] * x->size[1];
    bvstride = vstride << 10;
    nx = r->size[0] * r->size[1];
    r->size[0] = x->size[0];
    r->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&f_st, r, nx, &xe_emlrtRTEI);
    r1 = r->data;
    nx = bsum->size[0];
    bsum->size[0] = vstride;
    emxEnsureCapacity_real_T(&f_st, bsum, nx, &ye_emlrtRTEI);
    bsum_data = bsum->data;
    if (x->size[2] <= 1024) {
      firstBlockLength = FJE->size[2];
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = (int32_T)((uint32_T)x->size[2] >> 10);
      lastBlockLength = x->size[2] - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    g_st.site = &peb_emlrtRSI;
    if (vstride > 2147483646) {
      h_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&h_st);
    }
    for (xj = 0; xj < vstride; xj++) {
      r1[xj] = x_data[xj];
      bsum_data[xj] = 0.0;
    }
    g_st.site = &qeb_emlrtRSI;
    for (xj = 2; xj <= firstBlockLength; xj++) {
      xoffset = (xj - 1) * vstride;
      g_st.site = &reb_emlrtRSI;
      nx = (vstride / 2) << 1;
      n = nx - 2;
      for (b_xj = 0; b_xj <= n; b_xj += 2) {
        r2 = _mm_loadu_pd(&r1[b_xj]);
        r3 = _mm_loadu_pd(&x_data[xoffset + b_xj]);
        _mm_storeu_pd(&r1[b_xj], _mm_add_pd(r2, r3));
      }
      for (b_xj = nx; b_xj < vstride; b_xj++) {
        r1[b_xj] += x_data[xoffset + b_xj];
      }
    }
    g_st.site = &seb_emlrtRSI;
    for (ib = 2; ib <= nblocks; ib++) {
      xblockoffset = (ib - 1) * bvstride;
      g_st.site = &teb_emlrtRSI;
      for (b_xj = 0; b_xj < vstride; b_xj++) {
        bsum_data[b_xj] = x_data[xblockoffset + b_xj];
      }
      if (ib == nblocks) {
        nx = lastBlockLength;
      } else {
        nx = 1024;
      }
      g_st.site = &ueb_emlrtRSI;
      for (xj = 2; xj <= nx; xj++) {
        xoffset = xblockoffset + (xj - 1) * vstride;
        g_st.site = &veb_emlrtRSI;
        n = (vstride / 2) << 1;
        c_loop_ub = n - 2;
        for (c_xj = 0; c_xj <= c_loop_ub; c_xj += 2) {
          r2 = _mm_loadu_pd(&bsum_data[c_xj]);
          r3 = _mm_loadu_pd(&x_data[xoffset + c_xj]);
          _mm_storeu_pd(&bsum_data[c_xj], _mm_add_pd(r2, r3));
        }
        for (c_xj = n; c_xj < vstride; c_xj++) {
          bsum_data[c_xj] += x_data[xoffset + c_xj];
        }
      }
      g_st.site = &web_emlrtRSI;
      nx = (vstride / 2) << 1;
      n = nx - 2;
      for (c_xj = 0; c_xj <= n; c_xj += 2) {
        r2 = _mm_loadu_pd(&r1[c_xj]);
        r3 = _mm_loadu_pd(&bsum_data[c_xj]);
        _mm_storeu_pd(&r1[c_xj], _mm_add_pd(r2, r3));
      }
      for (c_xj = nx; c_xj < vstride; c_xj++) {
        r1[c_xj] += bsum_data[c_xj];
      }
    }
  }
  emxFree_real_T(&e_st, &bsum);
  emxFree_real_T(&e_st, &x);
  b_FJE[0] = FJE->size[0];
  b_FJE[1] = FJE->size[1] - 1;
  emlrtSubAssignSizeCheckR2012b(&b_FJE[0], 2, &r->size[0], 2, &o_emlrtECI, &st);
  b_FJE[0] = FJE->size[0];
  for (xj = 0; xj < b_loop_ub; xj++) {
    for (b_xj = 0; b_xj < d_loop_ub; b_xj++) {
      posterior_data[b_xj + posterior->size[0] * xj] = r1[b_xj + b_FJE[0] * xj];
    }
  }
  emxFree_real_T(&st, &r);
  c_loop_ub = posterior->size[0] - 1;
  if ((posterior->size[0] - 1 < 1) ||
      (posterior->size[0] - 1 > FJE->size[0] + 1)) {
    emlrtDynamicBoundsCheckR2012b(posterior->size[0] - 1, 1, FJE->size[0] + 1,
                                  &rd_emlrtBCI, &st);
  }
  emxInit_real_T(&st, &b_posterior, 2, &af_emlrtRTEI, true);
  nx = b_posterior->size[0] * b_posterior->size[1];
  b_posterior->size[0] = posterior->size[0] - 1;
  b_posterior->size[1] = FJE->size[1] - 1;
  emxFree_boolean_T(&st, &FJE);
  emxEnsureCapacity_real_T(&st, b_posterior, nx, &af_emlrtRTEI);
  bsum_data = b_posterior->data;
  for (xj = 0; xj < b_loop_ub; xj++) {
    for (b_xj = 0; b_xj <= loop_ub - 2; b_xj++) {
      bsum_data[b_xj + b_posterior->size[0] * xj] =
          posterior_data[b_xj + posterior->size[0] * xj];
    }
  }
  emxInit_real_T(&st, &varargin_2, 2, &bf_emlrtRTEI, true);
  b_st.site = &wkb_emlrtRSI;
  sum(&b_st, b_posterior, varargin_2);
  emxFree_real_T(&st, &b_posterior);
  nx = varargin_2->size[0] * varargin_2->size[1];
  varargin_2->size[0] = 1;
  emxEnsureCapacity_real_T(&st, varargin_2, nx, &bf_emlrtRTEI);
  bsum_data = varargin_2->data;
  nx = varargin_2->size[1] - 1;
  n = (varargin_2->size[1] / 2) << 1;
  firstBlockLength = n - 2;
  for (xj = 0; xj <= firstBlockLength; xj += 2) {
    r2 = _mm_loadu_pd(&bsum_data[xj]);
    _mm_storeu_pd(&bsum_data[xj], _mm_sub_pd(_mm_set1_pd(1.0), r2));
  }
  for (xj = n; xj <= nx; xj++) {
    bsum_data[xj] = 1.0 - bsum_data[xj];
  }
  nx = varargin_2->size[0] * varargin_2->size[1];
  varargin_2->size[0] = 1;
  emxEnsureCapacity_real_T(&st, varargin_2, nx, &cf_emlrtRTEI);
  bsum_data = varargin_2->data;
  nx = varargin_2->size[1] - 1;
  n = varargin_2->size[1] - 1;
  if (varargin_2->size[1] < 1600) {
    for (i = 0; i <= nx; i++) {
      real_T b_varargin_2;
      b_varargin_2 = bsum_data[i];
      bsum_data[i] = muDoubleScalarMax(0.0, b_varargin_2);
    }
  } else {
    emlrtEnterParallelRegion(&st, omp_in_parallel());
    emlrtPushJmpBuf(&st, &emlrtJBStack);
    jpda_numThreads = emlrtAllocRegionTLSs(
        st.tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(jpda_numThreads) private(c_varargin_2)

    for (i = 0; i <= n; i++) {
      c_varargin_2 = bsum_data[i];
      bsum_data[i] = muDoubleScalarMax(0.0, c_varargin_2);
    }
    emlrtPopJmpBuf(&st, &emlrtJBStack);
    emlrtExitParallelRegion(&st, omp_in_parallel());
  }
  b_FJE[0] = 1;
  b_FJE[1] = b_loop_ub;
  emlrtSubAssignSizeCheckR2012b(&b_FJE[0], 2, &varargin_2->size[0], 2,
                                &n_emlrtECI, &st);
  for (xj = 0; xj < b_loop_ub; xj++) {
    posterior_data[c_loop_ub + posterior->size[0] * xj] = bsum_data[xj];
  }
  emxFree_real_T(&st, &varargin_2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (jpda.c) */
