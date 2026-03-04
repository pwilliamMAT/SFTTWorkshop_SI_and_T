/*
 * MultiModalEstimator.c
 *
 * Code generation for function 'MultiModalEstimator'
 *
 */

/* Include files */
#include "MultiModalEstimator.h"
#include "CompositeFieldOfViewModel.h"
#include "EKFStateEstimator.h"
#include "ExtendedKalmanFilter.h"
#include "logsumexp.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "trackingEKF.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo vt_emlrtRSI = {
    165,                           /* lineNo */
    "MultiModalEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo wt_emlrtRSI = {
    114,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo xt_emlrtRSI = {
    115,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo yt_emlrtRSI = {
    119,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo au_emlrtRSI = {
    122,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo fw_emlrtRSI = {
    127,                                       /* lineNo */
    "MultiModalEstimator/survivalProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo gw_emlrtRSI = {
    130,                                       /* lineNo */
    "MultiModalEstimator/survivalProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo hw_emlrtRSI = {
    131,                                       /* lineNo */
    "MultiModalEstimator/survivalProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo iw_emlrtRSI = {
    86,                                      /* lineNo */
    "EKFStateEstimator/survivalProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo jw_emlrtRSI = {
    37,                                             /* lineNo */
    "UniformSurvivalRateModel/survivalProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+survival/"
    "UniformSurvivalRateModel.m" /* pathName */
};

static emlrtRSInfo cx_emlrtRSI = {
    137,                                   /* lineNo */
    "MultiModalEstimator/gateProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo dx_emlrtRSI = {
    91,                                  /* lineNo */
    "EKFStateEstimator/gateProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo
    ex_emlrtRSI =
        {
            12,                /* lineNo */
            "gateProbability", /* fcnName */
            "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/"
            "gateProbability.m" /* pathName */
};

static emlrtRSInfo fx_emlrtRSI = {
    43,                                                 /* lineNo */
    "gammainc",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/gammainc.m" /* pathName */
};

static emlrtRSInfo gx_emlrtRSI = {
    92,                                                 /* lineNo */
    "scalar_gammainc",                                  /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/gammainc.m" /* pathName */
};

static emlrtRSInfo hx_emlrtRSI = {
    374,            /* lineNo */
    "eml_gammainc", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/private/eml_gammainc.m" /* pathName
                                                                     */
};

static emlrtRSInfo jx_emlrtRSI = {
    294,            /* lineNo */
    "eml_gammainc", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/private/eml_gammainc.m" /* pathName
                                                                     */
};

static emlrtRSInfo kx_emlrtRSI = {
    198,            /* lineNo */
    "eml_gammainc", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/private/eml_gammainc.m" /* pathName
                                                                     */
};

static emlrtRSInfo fnb_emlrtRSI = {
    233,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo gnb_emlrtRSI = {
    238,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo hnb_emlrtRSI = {
    244,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo inb_emlrtRSI = {
    245,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo jnb_emlrtRSI = {
    249,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo knb_emlrtRSI = {
    252,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRSInfo lnb_emlrtRSI = {
    259,                         /* lineNo */
    "MultiModalEstimator/merge", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    82,                                          /* lineNo */
    5,                                           /* colNo */
    "fltpower",                                  /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/power.m" /* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = {
    356,                                                            /* lineNo */
    13,                                                             /* colNo */
    "eml_gammainc",                                                 /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/private/eml_gammainc.m" /* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = {
    276,                                                            /* lineNo */
    13,                                                             /* colNo */
    "eml_gammainc",                                                 /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/specfun/private/eml_gammainc.m" /* pName */
};

static emlrtBCInfo jk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    238,                         /* lineNo */
    77,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo kk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    238,                         /* lineNo */
    46,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo lk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    236,                         /* lineNo */
    39,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo mk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    236,                         /* lineNo */
    29,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo nk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    237,                         /* lineNo */
    32,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo ok_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    240,                         /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo pk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    238,                         /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "MultiModalEstimator/merge", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m", /* pName */
    0                        /* checkKind */
};

static emlrtRTEInfo ol_emlrtRTEI = {
    233,                   /* lineNo */
    17,                    /* colNo */
    "MultiModalEstimator", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pName */
};

static emlrtRTEInfo pl_emlrtRTEI = {
    234,                   /* lineNo */
    17,                    /* colNo */
    "MultiModalEstimator", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pName */
};

/* Function Definitions */
void MultiModalEstimator_merge(const emlrtStack *sp,
                               real_T estimator_DeletionThreshold,
                               const emxArray_struct_T *pdfs,
                               const emxArray_real_T *weights, b_struct_T *pdf)
{
  __m128d r;
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  const b_struct_T *pdfs_data;
  c_emxArray_struct_T *hypPdfs;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emlrtStack *b_sp;
  emxArray_real_T *hypWeights;
  struct_T *hypPdfs_data;
  real_T mergedWeights[3];
  real_T mergedWeights_data[3];
  const real_T *weights_data;
  real_T x;
  real_T *hypWeights_data;
  int32_T mergedWeights_size[2];
  int32_T MultiModalEstimator_merge_numThreads;
  int32_T b_k;
  int32_T c_k;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  int32_T partialTrueCount;
  int32_T trueCount;
  int8_T tmp_data[3];
  int8_T i1;
  boolean_T b_p;
  boolean_T c_p;
  boolean_T emlrtHadParallelError = false;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  weights_data = weights->data;
  pdfs_data = pdfs->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &fnb_emlrtRSI;
  emxInit_struct_T2(&st, &hypPdfs, &ol_emlrtRTEI);
  loop_ub = pdfs->size[0];
  partialTrueCount = hypPdfs->size[0];
  hypPdfs->size[0] = pdfs->size[0];
  emxEnsureCapacity_struct_T2(&st, hypPdfs, partialTrueCount, &ol_emlrtRTEI);
  hypPdfs_data = hypPdfs->data;
  for (i = 0; i < loop_ub; i++) {
    hypPdfs_data[i] = pdf->Hypothesis[0];
  }
  emxInit_real_T(sp, &hypWeights, 1, &pl_emlrtRTEI, true);
  partialTrueCount = hypWeights->size[0];
  hypWeights->size[0] = pdfs->size[0];
  emxEnsureCapacity_real_T(sp, hypWeights, partialTrueCount, &pl_emlrtRTEI);
  hypWeights_data = hypWeights->data;
  for (i = 0; i < loop_ub; i++) {
    hypWeights_data[i] = 0.0;
  }
  partialTrueCount = pdfs->size[0];
  trueCount = (pdfs->size[0] < 1600);
  if (trueCount) {
    for (k = 0; k < loop_ub; k++) {
      if (k + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, loop_ub, &lk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (k + 1 > hypPdfs->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, hypPdfs->size[0], &mk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      hypPdfs_data[k] = pdfs_data[k].Hypothesis[0];
      if (k + 1 > weights->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, weights->size[0], &nk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (weights_data[k] > 0.0) {
        st.site = &gnb_emlrtRSI;
        if (k + 1 > weights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, weights->size[0],
                                        &jk_emlrtBCI, &st);
        }
        if (weights_data[k] < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        if (k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, loop_ub, &kk_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        if (k + 1 > hypWeights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, hypWeights->size[0],
                                        &pk_emlrtBCI, (emlrtConstCTX)sp);
        }
        hypWeights_data[k] =
            pdfs_data[k].LogWeights[0] + muDoubleScalarLog(weights_data[k]);
      } else {
        if (k + 1 > hypWeights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, hypWeights->size[0],
                                        &ok_emlrtBCI, (emlrtConstCTX)sp);
        }
        hypWeights_data[k] = -1.7976931348623157E+308;
      }
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    MultiModalEstimator_merge_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(                                              \
        MultiModalEstimator_merge_numThreads) private(p, emlrtJBEnviron, c_st) \
    firstprivate(b_st, emlrtHadParallelError)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        b_st.prev = sp;
        b_st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
        b_st.site = NULL;
        emlrtSetJmpBuf(&b_st, &emlrtJBEnviron);
        c_st.prev = &b_st;
        c_st.tls = b_st.tls;
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (k = 0; k < partialTrueCount; k++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          if (k + 1 > pdfs->size[0]) {
            emlrtDynamicBoundsCheckR2012b(k + 1, 1, pdfs->size[0], &lk_emlrtBCI,
                                          &b_st);
          }
          if (k + 1 > hypPdfs->size[0]) {
            emlrtDynamicBoundsCheckR2012b(k + 1, 1, hypPdfs->size[0],
                                          &mk_emlrtBCI, &b_st);
          }
          hypPdfs_data[k] = pdfs_data[k].Hypothesis[0];
          if (k + 1 > weights->size[0]) {
            emlrtDynamicBoundsCheckR2012b(k + 1, 1, weights->size[0],
                                          &nk_emlrtBCI, &b_st);
          }
          if (weights_data[k] > 0.0) {
            c_st.site = &gnb_emlrtRSI;
            if (k + 1 > weights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(k + 1, 1, weights->size[0],
                                            &jk_emlrtBCI, &c_st);
            }
            p = (weights_data[k] < 0.0);
            if (p) {
              emlrtErrorWithMessageIdR2018a(
                  &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
                  "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
            }
            if (k + 1 > pdfs->size[0]) {
              emlrtDynamicBoundsCheckR2012b(k + 1, 1, pdfs->size[0],
                                            &kk_emlrtBCI, &b_st);
            }
            if (k + 1 > hypWeights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(k + 1, 1, hypWeights->size[0],
                                            &pk_emlrtBCI, &b_st);
            }
            hypWeights_data[k] =
                pdfs_data[k].LogWeights[0] + muDoubleScalarLog(weights_data[k]);
          } else {
            if (k + 1 > hypWeights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(k + 1, 1, hypWeights->size[0],
                                            &ok_emlrtBCI, &b_st);
            }
            hypWeights_data[k] = -1.7976931348623157E+308;
          }
        } else {
          emlrtHadParallelError = true;
        }
      }
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  st.site = &hnb_emlrtRSI;
  EKFStateEstimator_merge(&st, hypPdfs, weights, &pdf->Hypothesis[0]);
  st.site = &inb_emlrtRSI;
  mergedWeights[0] = b_logsumexp(&st, hypWeights);
  st.site = &fnb_emlrtRSI;
  partialTrueCount = hypPdfs->size[0];
  hypPdfs->size[0] = loop_ub;
  emxEnsureCapacity_struct_T2(&st, hypPdfs, partialTrueCount, &ol_emlrtRTEI);
  hypPdfs_data = hypPdfs->data;
  for (i = 0; i < loop_ub; i++) {
    hypPdfs_data[i] = pdf->Hypothesis[1];
  }
  partialTrueCount = hypWeights->size[0];
  hypWeights->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, hypWeights, partialTrueCount, &pl_emlrtRTEI);
  hypWeights_data = hypWeights->data;
  for (i = 0; i < loop_ub; i++) {
    hypWeights_data[i] = 0.0;
  }
  if (trueCount) {
    for (b_k = 0; b_k < loop_ub; b_k++) {
      if (b_k + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, loop_ub, &lk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (b_k + 1 > hypPdfs->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, hypPdfs->size[0],
                                      &mk_emlrtBCI, (emlrtConstCTX)sp);
      }
      hypPdfs_data[b_k] = pdfs_data[b_k].Hypothesis[1];
      if (b_k + 1 > weights->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, weights->size[0],
                                      &nk_emlrtBCI, (emlrtConstCTX)sp);
      }
      if (weights_data[b_k] > 0.0) {
        st.site = &gnb_emlrtRSI;
        if (b_k + 1 > weights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, weights->size[0],
                                        &jk_emlrtBCI, &st);
        }
        if (weights_data[b_k] < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        if (b_k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, loop_ub, &kk_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        if (b_k + 1 > hypWeights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, hypWeights->size[0],
                                        &pk_emlrtBCI, (emlrtConstCTX)sp);
        }
        hypWeights_data[b_k] =
            pdfs_data[b_k].LogWeights[1] + muDoubleScalarLog(weights_data[b_k]);
      } else {
        if (b_k + 1 > hypWeights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, hypWeights->size[0],
                                        &ok_emlrtBCI, (emlrtConstCTX)sp);
        }
        hypWeights_data[b_k] = -1.7976931348623157E+308;
      }
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    MultiModalEstimator_merge_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(                                              \
        MultiModalEstimator_merge_numThreads) private(b_p, emlrtJBEnviron,     \
                                                          e_st)                \
    firstprivate(d_st, emlrtHadParallelError)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        d_st.prev = sp;
        d_st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
        d_st.site = NULL;
        emlrtSetJmpBuf(&d_st, &emlrtJBEnviron);
        e_st.prev = &d_st;
        e_st.tls = d_st.tls;
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (b_k = 0; b_k < loop_ub; b_k++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          if (b_k + 1 > pdfs->size[0]) {
            emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, pdfs->size[0],
                                          &lk_emlrtBCI, &d_st);
          }
          if (b_k + 1 > hypPdfs->size[0]) {
            emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, hypPdfs->size[0],
                                          &mk_emlrtBCI, &d_st);
          }
          hypPdfs_data[b_k] = pdfs_data[b_k].Hypothesis[1];
          if (b_k + 1 > weights->size[0]) {
            emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, weights->size[0],
                                          &nk_emlrtBCI, &d_st);
          }
          if (weights_data[b_k] > 0.0) {
            e_st.site = &gnb_emlrtRSI;
            if (b_k + 1 > weights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, weights->size[0],
                                            &jk_emlrtBCI, &e_st);
            }
            b_p = (weights_data[b_k] < 0.0);
            if (b_p) {
              emlrtErrorWithMessageIdR2018a(
                  &e_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
                  "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
            }
            if (b_k + 1 > pdfs->size[0]) {
              emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, pdfs->size[0],
                                            &kk_emlrtBCI, &d_st);
            }
            if (b_k + 1 > hypWeights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, hypWeights->size[0],
                                            &pk_emlrtBCI, &d_st);
            }
            hypWeights_data[b_k] = pdfs_data[b_k].LogWeights[1] +
                                   muDoubleScalarLog(weights_data[b_k]);
          } else {
            if (b_k + 1 > hypWeights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(b_k + 1, 1, hypWeights->size[0],
                                            &ok_emlrtBCI, &d_st);
            }
            hypWeights_data[b_k] = -1.7976931348623157E+308;
          }
        } else {
          emlrtHadParallelError = true;
        }
      }
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  st.site = &hnb_emlrtRSI;
  EKFStateEstimator_merge(&st, hypPdfs, weights, &pdf->Hypothesis[1]);
  st.site = &inb_emlrtRSI;
  mergedWeights[1] = b_logsumexp(&st, hypWeights);
  st.site = &fnb_emlrtRSI;
  partialTrueCount = hypPdfs->size[0];
  hypPdfs->size[0] = loop_ub;
  emxEnsureCapacity_struct_T2(&st, hypPdfs, partialTrueCount, &ol_emlrtRTEI);
  hypPdfs_data = hypPdfs->data;
  for (i = 0; i < loop_ub; i++) {
    hypPdfs_data[i] = pdf->Hypothesis[2];
  }
  partialTrueCount = hypWeights->size[0];
  hypWeights->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, hypWeights, partialTrueCount, &pl_emlrtRTEI);
  hypWeights_data = hypWeights->data;
  for (i = 0; i < loop_ub; i++) {
    hypWeights_data[i] = 0.0;
  }
  if (trueCount) {
    for (c_k = 0; c_k < loop_ub; c_k++) {
      if (c_k + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, loop_ub, &lk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (c_k + 1 > hypPdfs->size[0]) {
        emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, hypPdfs->size[0],
                                      &mk_emlrtBCI, (emlrtConstCTX)sp);
      }
      hypPdfs_data[c_k] = pdfs_data[c_k].Hypothesis[2];
      if (c_k + 1 > weights->size[0]) {
        emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, weights->size[0],
                                      &nk_emlrtBCI, (emlrtConstCTX)sp);
      }
      if (weights_data[c_k] > 0.0) {
        st.site = &gnb_emlrtRSI;
        if (c_k + 1 > weights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, weights->size[0],
                                        &jk_emlrtBCI, &st);
        }
        if (weights_data[c_k] < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        if (c_k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, loop_ub, &kk_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        if (c_k + 1 > hypWeights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, hypWeights->size[0],
                                        &pk_emlrtBCI, (emlrtConstCTX)sp);
        }
        hypWeights_data[c_k] =
            pdfs_data[c_k].LogWeights[2] + muDoubleScalarLog(weights_data[c_k]);
      } else {
        if (c_k + 1 > hypWeights->size[0]) {
          emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, hypWeights->size[0],
                                        &ok_emlrtBCI, (emlrtConstCTX)sp);
        }
        hypWeights_data[c_k] = -1.7976931348623157E+308;
      }
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    MultiModalEstimator_merge_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(                                              \
        MultiModalEstimator_merge_numThreads) private(c_p, emlrtJBEnviron,     \
                                                          g_st)                \
    firstprivate(f_st, emlrtHadParallelError)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        f_st.prev = sp;
        f_st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
        f_st.site = NULL;
        emlrtSetJmpBuf(&f_st, &emlrtJBEnviron);
        g_st.prev = &f_st;
        g_st.tls = f_st.tls;
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (c_k = 0; c_k < loop_ub; c_k++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          if (c_k + 1 > pdfs->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, pdfs->size[0],
                                          &lk_emlrtBCI, &f_st);
          }
          if (c_k + 1 > hypPdfs->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, hypPdfs->size[0],
                                          &mk_emlrtBCI, &f_st);
          }
          hypPdfs_data[c_k] = pdfs_data[c_k].Hypothesis[2];
          if (c_k + 1 > weights->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, weights->size[0],
                                          &nk_emlrtBCI, &f_st);
          }
          if (weights_data[c_k] > 0.0) {
            g_st.site = &gnb_emlrtRSI;
            if (c_k + 1 > weights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, weights->size[0],
                                            &jk_emlrtBCI, &g_st);
            }
            c_p = (weights_data[c_k] < 0.0);
            if (c_p) {
              emlrtErrorWithMessageIdR2018a(
                  &g_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
                  "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
            }
            if (c_k + 1 > pdfs->size[0]) {
              emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, pdfs->size[0],
                                            &kk_emlrtBCI, &f_st);
            }
            if (c_k + 1 > hypWeights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, hypWeights->size[0],
                                            &pk_emlrtBCI, &f_st);
            }
            hypWeights_data[c_k] = pdfs_data[c_k].LogWeights[2] +
                                   muDoubleScalarLog(weights_data[c_k]);
          } else {
            if (c_k + 1 > hypWeights->size[0]) {
              emlrtDynamicBoundsCheckR2012b(c_k + 1, 1, hypWeights->size[0],
                                            &ok_emlrtBCI, &f_st);
            }
            hypWeights_data[c_k] = -1.7976931348623157E+308;
          }
        } else {
          emlrtHadParallelError = true;
        }
      }
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  st.site = &hnb_emlrtRSI;
  EKFStateEstimator_merge(&st, hypPdfs, weights, &pdf->Hypothesis[2]);
  emxFree_struct_T2(sp, &hypPdfs);
  st.site = &inb_emlrtRSI;
  mergedWeights[2] = b_logsumexp(&st, hypWeights);
  emxFree_real_T(sp, &hypWeights);
  st.site = &jnb_emlrtRSI;
  x = c_logsumexp(&st, mergedWeights);
  r = _mm_loadu_pd(&mergedWeights[0]);
  _mm_storeu_pd(&mergedWeights[0], _mm_sub_pd(r, _mm_set1_pd(x)));
  mergedWeights[2] -= x;
  st.site = &knb_emlrtRSI;
  if (estimator_DeletionThreshold < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  x = muDoubleScalarLog(estimator_DeletionThreshold);
  pdf->IsValid[0] = (mergedWeights[0] >= x);
  pdf->IsValid[1] = (mergedWeights[1] >= x);
  pdf->IsValid[2] = (mergedWeights[2] >= x);
  trueCount = 0;
  if (pdf->IsValid[0]) {
    trueCount = 1;
  }
  if (pdf->IsValid[1]) {
    trueCount++;
  }
  if (pdf->IsValid[2]) {
    trueCount++;
  }
  partialTrueCount = 0;
  if (pdf->IsValid[0]) {
    tmp_data[0] = 0;
    partialTrueCount = 1;
  }
  if (pdf->IsValid[1]) {
    tmp_data[partialTrueCount] = 1;
    partialTrueCount++;
  }
  if (pdf->IsValid[2]) {
    tmp_data[partialTrueCount] = 2;
  }
  mergedWeights_size[0] = 1;
  mergedWeights_size[1] = trueCount;
  for (i = 0; i < trueCount; i++) {
    mergedWeights_data[i] = mergedWeights[tmp_data[i]];
  }
  if (!pdf->IsValid[0]) {
    pdf->LogWeights[0] = -1.7976931348623157E+308;
  }
  if (!pdf->IsValid[1]) {
    pdf->LogWeights[1] = -1.7976931348623157E+308;
  }
  if (!pdf->IsValid[2]) {
    pdf->LogWeights[2] = -1.7976931348623157E+308;
  }
  st.site = &lnb_emlrtRSI;
  x = logsumexp(&st, mergedWeights_data, mergedWeights_size);
  for (i = 0; i < trueCount; i++) {
    i1 = tmp_data[i];
    pdf->LogWeights[i1] = mergedWeights[i1] - x;
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void MultiModalEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_Estimators_f1_Targe,
    trackingEKF *c_estimator_Estimators_f1_Track,
    const c_fusion_tracker_targetspecs_Ge *c_estimator_Estimators_f2_Targe,
    trackingEKF *c_estimator_Estimators_f2_Track,
    const c_fusion_tracker_targetspecs_He *c_estimator_Estimators_f3_Targe,
    trackingEKF *c_estimator_Estimators_f3_Track, b_struct_T *pdf, real_T dT)
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  emlrtStack b_st;
  emlrtStack st;
  real_T b_a[36];
  real_T dv[36];
  real_T a;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (pdf->IsValid[0]) {
    st.site = &vt_emlrtRSI;
    b_st.site = &wt_emlrtRSI;
    ExtendedKalmanFilter_set_State(&b_st, c_estimator_Estimators_f1_Track,
                                   pdf->Hypothesis[0].State);
    b_st.site = &xt_emlrtRSI;
    c_ExtendedKalmanFilter_set_Stat(&b_st, c_estimator_Estimators_f1_Track,
                                    pdf->Hypothesis[0].StateCovariance);
    b_st.site = &yt_emlrtRSI;
    c_ExtendedKalmanFilter_set_Proc(
        &b_st, c_estimator_Estimators_f1_Track,
        c_estimator_Estimators_f1_Targe->StateTransitionModel
            .PropAccelerationVariance);
    b_st.site = &au_emlrtRSI;
    trackingEKF_predict(&b_st, c_estimator_Estimators_f1_Track, dT);
    for (i = 0; i < 6; i++) {
      pdf->Hypothesis[0].State[i] = c_estimator_Estimators_f1_Track->pState[i];
    }
    if ((!c_estimator_Estimators_f1_Track->pIsSetStateCovariance) ||
        (c_estimator_Estimators_f1_Track->pSqrtStateCovarianceScalar != -1.0)) {
      a = c_estimator_Estimators_f1_Track->pSqrtStateCovarianceScalar;
      for (i = 0; i < 36; i++) {
        c_estimator_Estimators_f1_Track->pSqrtStateCovariance[i] =
            a * (real_T)iv[i];
      }
      c_estimator_Estimators_f1_Track->pIsSetStateCovariance = true;
      c_estimator_Estimators_f1_Track->pSqrtStateCovarianceScalar = -1.0;
    }
    memcpy(&dv[0], &c_estimator_Estimators_f1_Track->pSqrtStateCovariance[0],
           36U * sizeof(real_T));
    memcpy(&b_a[0], &c_estimator_Estimators_f1_Track->pSqrtStateCovariance[0],
           36U * sizeof(real_T));
    for (b_i = 0; b_i < 6; b_i++) {
      for (i = 0; i < 6; i++) {
        pdf->Hypothesis[0].StateCovariance[i + 6 * b_i] = 0.0;
      }
      i1 = 6 * b_i + 2;
      i2 = 6 * b_i + 4;
      for (i = 0; i < 6; i++) {
        r = _mm_loadu_pd(&b_a[6 * i]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[0].StateCovariance[6 * b_i]);
        r2 = _mm_set1_pd(dv[b_i + 6 * i]);
        _mm_storeu_pd(&pdf->Hypothesis[0].StateCovariance[6 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&b_a[6 * i + 2]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[0].StateCovariance[i1]);
        _mm_storeu_pd(&pdf->Hypothesis[0].StateCovariance[i1],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&b_a[6 * i + 4]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[0].StateCovariance[i2]);
        _mm_storeu_pd(&pdf->Hypothesis[0].StateCovariance[i2],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
  }
  if (pdf->IsValid[1]) {
    st.site = &vt_emlrtRSI;
    b_st.site = &wt_emlrtRSI;
    ExtendedKalmanFilter_set_State(&b_st, c_estimator_Estimators_f2_Track,
                                   pdf->Hypothesis[1].State);
    b_st.site = &xt_emlrtRSI;
    c_ExtendedKalmanFilter_set_Stat(&b_st, c_estimator_Estimators_f2_Track,
                                    pdf->Hypothesis[1].StateCovariance);
    b_st.site = &yt_emlrtRSI;
    c_ExtendedKalmanFilter_set_Proc(
        &b_st, c_estimator_Estimators_f2_Track,
        c_estimator_Estimators_f2_Targe->StateTransitionModel
            .PropAccelerationVariance);
    b_st.site = &au_emlrtRSI;
    trackingEKF_predict(&b_st, c_estimator_Estimators_f2_Track, dT);
    for (i = 0; i < 6; i++) {
      pdf->Hypothesis[1].State[i] = c_estimator_Estimators_f2_Track->pState[i];
    }
    if ((!c_estimator_Estimators_f2_Track->pIsSetStateCovariance) ||
        (c_estimator_Estimators_f2_Track->pSqrtStateCovarianceScalar != -1.0)) {
      a = c_estimator_Estimators_f2_Track->pSqrtStateCovarianceScalar;
      for (i = 0; i < 36; i++) {
        c_estimator_Estimators_f2_Track->pSqrtStateCovariance[i] =
            a * (real_T)iv[i];
      }
      c_estimator_Estimators_f2_Track->pIsSetStateCovariance = true;
      c_estimator_Estimators_f2_Track->pSqrtStateCovarianceScalar = -1.0;
    }
    memcpy(&dv[0], &c_estimator_Estimators_f2_Track->pSqrtStateCovariance[0],
           36U * sizeof(real_T));
    memcpy(&b_a[0], &c_estimator_Estimators_f2_Track->pSqrtStateCovariance[0],
           36U * sizeof(real_T));
    for (b_i = 0; b_i < 6; b_i++) {
      for (i = 0; i < 6; i++) {
        pdf->Hypothesis[1].StateCovariance[i + 6 * b_i] = 0.0;
      }
      i1 = 6 * b_i + 2;
      i2 = 6 * b_i + 4;
      for (i = 0; i < 6; i++) {
        r = _mm_loadu_pd(&b_a[6 * i]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[1].StateCovariance[6 * b_i]);
        r2 = _mm_set1_pd(dv[b_i + 6 * i]);
        _mm_storeu_pd(&pdf->Hypothesis[1].StateCovariance[6 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&b_a[6 * i + 2]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[1].StateCovariance[i1]);
        _mm_storeu_pd(&pdf->Hypothesis[1].StateCovariance[i1],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&b_a[6 * i + 4]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[1].StateCovariance[i2]);
        _mm_storeu_pd(&pdf->Hypothesis[1].StateCovariance[i2],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
  }
  if (pdf->IsValid[2]) {
    st.site = &vt_emlrtRSI;
    b_st.site = &wt_emlrtRSI;
    ExtendedKalmanFilter_set_State(&b_st, c_estimator_Estimators_f3_Track,
                                   pdf->Hypothesis[2].State);
    b_st.site = &xt_emlrtRSI;
    c_ExtendedKalmanFilter_set_Stat(&b_st, c_estimator_Estimators_f3_Track,
                                    pdf->Hypothesis[2].StateCovariance);
    b_st.site = &yt_emlrtRSI;
    c_ExtendedKalmanFilter_set_Proc(
        &b_st, c_estimator_Estimators_f3_Track,
        c_estimator_Estimators_f3_Targe->StateTransitionModel
            .PropAccelerationVariance);
    b_st.site = &au_emlrtRSI;
    trackingEKF_predict(&b_st, c_estimator_Estimators_f3_Track, dT);
    for (i = 0; i < 6; i++) {
      pdf->Hypothesis[2].State[i] = c_estimator_Estimators_f3_Track->pState[i];
    }
    if ((!c_estimator_Estimators_f3_Track->pIsSetStateCovariance) ||
        (c_estimator_Estimators_f3_Track->pSqrtStateCovarianceScalar != -1.0)) {
      a = c_estimator_Estimators_f3_Track->pSqrtStateCovarianceScalar;
      for (i = 0; i < 36; i++) {
        c_estimator_Estimators_f3_Track->pSqrtStateCovariance[i] =
            a * (real_T)iv[i];
      }
      c_estimator_Estimators_f3_Track->pIsSetStateCovariance = true;
      c_estimator_Estimators_f3_Track->pSqrtStateCovarianceScalar = -1.0;
    }
    memcpy(&dv[0], &c_estimator_Estimators_f3_Track->pSqrtStateCovariance[0],
           36U * sizeof(real_T));
    memcpy(&b_a[0], &c_estimator_Estimators_f3_Track->pSqrtStateCovariance[0],
           36U * sizeof(real_T));
    for (b_i = 0; b_i < 6; b_i++) {
      for (i = 0; i < 6; i++) {
        pdf->Hypothesis[2].StateCovariance[i + 6 * b_i] = 0.0;
      }
      i1 = 6 * b_i + 2;
      i2 = 6 * b_i + 4;
      for (i = 0; i < 6; i++) {
        r = _mm_loadu_pd(&b_a[6 * i]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[2].StateCovariance[6 * b_i]);
        r2 = _mm_set1_pd(dv[b_i + 6 * i]);
        _mm_storeu_pd(&pdf->Hypothesis[2].StateCovariance[6 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&b_a[6 * i + 2]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[2].StateCovariance[i1]);
        _mm_storeu_pd(&pdf->Hypothesis[2].StateCovariance[i1],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&b_a[6 * i + 4]);
        r1 = _mm_loadu_pd(&pdf->Hypothesis[2].StateCovariance[i2]);
        _mm_storeu_pd(&pdf->Hypothesis[2].StateCovariance[i2],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
  }
}

real_T c_MultiModalEstimator_detection(
    const emlrtStack *sp,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_Estimators_f1_Senso,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_Estimators_f2_Senso,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_Estimators_f3_Senso,
    const struct_T pdf_Hypothesis[3], const real_T pdf_LogWeights[3],
    const boolean_T pdf_IsValid[3])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T b_pdf_Hypothesis[6];
  real_T Pdi[3];
  real_T pdf_LogWeights_data[3];
  real_T Pd;
  int32_T pdf_LogWeights_size[2];
  int32_T k;
  int32_T partialTrueCount;
  int32_T trueCount;
  int8_T tmp_data[3];
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  Pdi[0] = 0.0;
  Pdi[1] = 0.0;
  Pdi[2] = 0.0;
  if (pdf_IsValid[0]) {
    st.site = &sx_emlrtRSI;
    b_pdf_Hypothesis[0] = pdf_Hypothesis[0].State[0];
    b_pdf_Hypothesis[3] = pdf_Hypothesis[0].State[1];
    b_pdf_Hypothesis[1] = pdf_Hypothesis[0].State[2];
    b_pdf_Hypothesis[4] = pdf_Hypothesis[0].State[3];
    b_pdf_Hypothesis[2] = pdf_Hypothesis[0].State[4];
    b_pdf_Hypothesis[5] = pdf_Hypothesis[0].State[5];
    b_st.site = &vx_emlrtRSI;
    Pdi[0] = c_CompositeFieldOfViewModel_det(
        &b_st, c_estimator_Estimators_f1_Senso->DetectabilityModel.FieldsOfView,
        c_estimator_Estimators_f1_Senso->DetectabilityModel.NumModels,
        b_pdf_Hypothesis);
  }
  if (pdf_IsValid[1]) {
    st.site = &sx_emlrtRSI;
    b_pdf_Hypothesis[0] = pdf_Hypothesis[1].State[0];
    b_pdf_Hypothesis[3] = pdf_Hypothesis[1].State[1];
    b_pdf_Hypothesis[1] = pdf_Hypothesis[1].State[2];
    b_pdf_Hypothesis[4] = pdf_Hypothesis[1].State[3];
    b_pdf_Hypothesis[2] = pdf_Hypothesis[1].State[4];
    b_pdf_Hypothesis[5] = pdf_Hypothesis[1].State[5];
    b_st.site = &vx_emlrtRSI;
    Pdi[1] = c_CompositeFieldOfViewModel_det(
        &b_st, c_estimator_Estimators_f2_Senso->DetectabilityModel.FieldsOfView,
        c_estimator_Estimators_f2_Senso->DetectabilityModel.NumModels,
        b_pdf_Hypothesis);
  }
  if (pdf_IsValid[2]) {
    st.site = &sx_emlrtRSI;
    b_pdf_Hypothesis[0] = pdf_Hypothesis[2].State[0];
    b_pdf_Hypothesis[3] = pdf_Hypothesis[2].State[1];
    b_pdf_Hypothesis[1] = pdf_Hypothesis[2].State[2];
    b_pdf_Hypothesis[4] = pdf_Hypothesis[2].State[3];
    b_pdf_Hypothesis[2] = pdf_Hypothesis[2].State[4];
    b_pdf_Hypothesis[5] = pdf_Hypothesis[2].State[5];
    b_st.site = &vx_emlrtRSI;
    Pdi[2] = c_CompositeFieldOfViewModel_det(
        &b_st, c_estimator_Estimators_f3_Senso->DetectabilityModel.FieldsOfView,
        c_estimator_Estimators_f3_Senso->DetectabilityModel.NumModels,
        b_pdf_Hypothesis);
  }
  st.site = &tx_emlrtRSI;
  p = false;
  for (k = 0; k < 3; k++) {
    if (p || (Pdi[k] < 0.0)) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  Pdi[0] = muDoubleScalarLog(Pdi[0]);
  Pdi[1] = muDoubleScalarLog(Pdi[1]);
  Pdi[2] = muDoubleScalarLog(Pdi[2]);
  trueCount = 0;
  if (pdf_IsValid[0]) {
    trueCount = 1;
  }
  if (pdf_IsValid[1]) {
    trueCount++;
  }
  if (pdf_IsValid[2]) {
    trueCount++;
  }
  partialTrueCount = 0;
  if (pdf_IsValid[0]) {
    tmp_data[0] = 0;
    partialTrueCount = 1;
  }
  if (pdf_IsValid[1]) {
    tmp_data[partialTrueCount] = 1;
    partialTrueCount++;
  }
  if (pdf_IsValid[2]) {
    tmp_data[partialTrueCount] = 2;
  }
  pdf_LogWeights_size[0] = 1;
  pdf_LogWeights_size[1] = trueCount;
  for (k = 0; k < trueCount; k++) {
    int8_T i;
    i = tmp_data[k];
    pdf_LogWeights_data[k] = pdf_LogWeights[i] + Pdi[i];
  }
  st.site = &ux_emlrtRSI;
  Pd = logsumexp(&st, pdf_LogWeights_data, pdf_LogWeights_size);
  return muDoubleScalarExp(Pd);
}

real_T c_MultiModalEstimator_gateProba(const emlrtStack *sp, real_T gateSize)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T Pg;
  real_T x;
  int32_T b_i;
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
  st.site = &cx_emlrtRSI;
  b_st.site = &dx_emlrtRSI;
  c_st.site = &sd_emlrtRSI;
  d_st.site = &td_emlrtRSI;
  b_st.site = &dx_emlrtRSI;
  c_st.site = &ex_emlrtRSI;
  c_st.site = &ex_emlrtRSI;
  x = muDoubleScalarSqrt(gateSize * gateSize) / 2.0;
  d_st.site = &fx_emlrtRSI;
  e_st.site = &gx_emlrtRSI;
  if (!(x > 0.0)) {
    if (x == 0.0) {
      Pg = 0.0;
    } else {
      Pg = rtNaN;
    }
  } else if (muDoubleScalarIsInf(x)) {
    Pg = 1.0;
  } else {
    real_T a2;
    real_T a3;
    real_T b_t;
    real_T t;
    real_T xD0;
    int32_T exitg1;
    if (muDoubleScalarAbs(2.0 - x) > 0.1 * (x + 2.0)) {
      if (2.2250738585072014E-308 * x > 2.0) {
        xD0 = x;
      } else if ((x < 1.0) && (1.7976931348623157E+308 * x < 2.0)) {
        xD0 = (1.3862943611198906 - 2.0 * muDoubleScalarLog(x)) - 2.0;
      } else {
        xD0 = (2.0 * muDoubleScalarLog(2.0 / x) + x) - 2.0;
      }
    } else {
      t = x / 2.0;
      Pg = (1.0 - t) / (t + 1.0);
      a2 = Pg * Pg;
      xD0 = (2.0 - x) * Pg;
      a3 = xD0;
      Pg = 2.0 * (2.0 * Pg);
      b_t = 3.0;
      do {
        exitg1 = 0;
        Pg *= a2;
        xD0 += Pg / b_t;
        if (xD0 == a3) {
          exitg1 = 1;
        } else {
          a3 = xD0;
          b_t += 2.0;
        }
      } while (exitg1 == 0);
    }
    if (x > 1.0E+6) {
      real_T sqrtx;
      real_T tsq;
      sqrtx = muDoubleScalarSqrt(x);
      t = muDoubleScalarAbs((2.0 - x) - 1.0) / sqrtx;
      tsq = t * t;
      if (t < 15.0) {
        Pg = 0.70710678118654746 * t;
        b_t = 3.97886080735226 / (Pg + 3.97886080735226);
        b_t = 0.5 *
              ((((((((((((((((((((((0.0012710976495261409 * (b_t - 0.5) +
                                    0.00011931402283834095) *
                                       (b_t - 0.5) -
                                   0.0039638509736051354) *
                                      (b_t - 0.5) -
                                  0.00087077963531729586) *
                                     (b_t - 0.5) +
                                 0.0077367252831352668) *
                                    (b_t - 0.5) +
                                0.0038333512626488732) *
                                   (b_t - 0.5) -
                               0.012722381378212275) *
                                  (b_t - 0.5) -
                              0.013382364453346007) *
                                 (b_t - 0.5) +
                             0.016131532973325226) *
                                (b_t - 0.5) +
                            0.039097684558848406) *
                               (b_t - 0.5) +
                           0.0024936720005350331) *
                              (b_t - 0.5) -
                          0.0838864557023002) *
                             (b_t - 0.5) -
                         0.11946395996432542) *
                            (b_t - 0.5) +
                        0.016620792496936737) *
                           (b_t - 0.5) +
                       0.35752427444953105) *
                          (b_t - 0.5) +
                      0.80527640875291062) *
                         (b_t - 0.5) +
                     1.1890298290927332) *
                        (b_t - 0.5) +
                    1.3704021768233816) *
                       (b_t - 0.5) +
                   1.313146538310231) *
                      (b_t - 0.5) +
                  1.0792551515585667) *
                     (b_t - 0.5) +
                 0.77436819911953858) *
                    (b_t - 0.5) +
                0.49016508058531844) *
                   (b_t - 0.5) +
               0.27537474159737679) *
              b_t * muDoubleScalarExp(-Pg * Pg) * 2.5066282746310002 *
              muDoubleScalarExp(0.5 * tsq);
        a2 = (b_t * ((tsq - 3.0) * t) - (tsq - 4.0)) / 6.0;
        a3 = (b_t * ((tsq * tsq - 9.0) * tsq + 6.0) -
              ((tsq - 1.0) * tsq - 6.0) * t) /
             72.0;
        Pg = 5.0 * tsq;
        Pg =
            (b_t * (((((Pg + 45.0) * tsq - 81.0) * tsq - 315.0) * tsq + 270.0) *
                    t) -
             ((((Pg + 40.0) * tsq - 111.0) * tsq - 174.0) * tsq + 192.0)) /
            6480.0;
      } else {
        b_t = (((3.0 - 15.0 / tsq) / tsq - 1.0) / tsq + 1.0) / t;
        a2 = (((25.0 - 210.0 / tsq) / tsq - 4.0) / tsq + 1.0) / tsq;
        a3 = (((130.0 - 1750.0 / tsq) / tsq - 11.0) / tsq + 1.0) / (tsq * t);
        Pg = (((546.0 - 11368.0 / tsq) / tsq - 26.0) / tsq + 1.0) / (tsq * tsq);
      }
      Pg = 2.0 * (((b_t / sqrtx + a2 / x) + a3 / (x * sqrtx)) + Pg / (x * x));
      if (-1.3068528194400546 - xD0 < 709.782712893384) {
        Pg *= muDoubleScalarExp(-1.3068528194400546 - xD0);
      } else {
        f_st.site = &kx_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &f_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarExp((-1.3068528194400546 - xD0) +
                               muDoubleScalarLog(Pg));
      }
      Pg = 1.0 - Pg;
    } else if (x < 2.0) {
      int32_T i;
      b_t = 1.0;
      if (x > 4.4408920985006262E-16) {
        Pg = x / 2.0;
        b_t = 2.0;
        do {
          exitg1 = 0;
          Pg = x * Pg / ((b_t - 1.0) + 2.0);
          if (Pg < 2.2204460492503131E-16) {
            exitg1 = 1;
          } else {
            b_t++;
          }
        } while (exitg1 == 0);
      }
      Pg = 0.0;
      i = (int32_T) - ((-1.0 - (b_t - 1.0)) + 1.0);
      emlrtForLoopVectorCheckR2021a(b_t - 1.0, -1.0, 1.0, mxDOUBLE_CLASS,
                                    (int32_T) - ((-1.0 - (b_t - 1.0)) + 1.0),
                                    &bb_emlrtRTEI, &e_st);
      for (b_i = 0; b_i < i; b_i++) {
        Pg = x * (Pg + 1.0) / (((b_t - 1.0) - (real_T)b_i) + 2.0);
      }
      Pg++;
      if (-1.3068528194400546 - xD0 < 709.782712893384) {
        Pg *= muDoubleScalarExp(-1.3068528194400546 - xD0);
      } else {
        f_st.site = &jx_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &f_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarExp((-1.3068528194400546 - xD0) +
                               muDoubleScalarLog(Pg));
      }
      if (Pg > 1.0) {
        Pg = 1.0;
      }
    } else {
      int32_T i;
      Pg = 1.0;
      b_t = 1.0;
      do {
        exitg1 = 0;
        i = (int32_T)muDoubleScalarFloor(x + 2.0);
        if ((int32_T)b_t <= i) {
          Pg = (2.0 - b_t) * Pg / x;
          if (muDoubleScalarAbs(Pg) < 2.2204460492503131E-16) {
            exitg1 = 1;
          } else {
            b_t++;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if ((int32_T)b_t <= i) {
        Pg = 1.0;
      } else {
        Pg = 1.0;
        b_t = 2.0;
      }
      i = (int32_T)b_t;
      emlrtForLoopVectorCheckR2021a(b_t - 1.0, -1.0, 1.0, mxDOUBLE_CLASS,
                                    (int32_T)b_t - 1, &ab_emlrtRTEI, &e_st);
      for (b_i = 0; b_i <= i - 2; b_i++) {
        Pg = (2.0 - ((b_t - 1.0) - (real_T)b_i)) * Pg / x + 1.0;
      }
      Pg = Pg * 2.0 / x;
      if (-1.3068528194400546 - xD0 < 709.782712893384) {
        Pg *= muDoubleScalarExp(-1.3068528194400546 - xD0);
      } else {
        f_st.site = &hx_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &f_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarExp((-1.3068528194400546 - xD0) +
                               muDoubleScalarLog(Pg));
      }
      if (Pg > 1.0) {
        Pg = 1.0;
      }
      Pg = 1.0 - Pg;
    }
  }
  return Pg;
}

real_T c_MultiModalEstimator_survivalP(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_Estimators_f1_Targe,
    const c_fusion_tracker_targetspecs_Ge *c_estimator_Estimators_f2_Targe,
    const c_fusion_tracker_targetspecs_He *c_estimator_Estimators_f3_Targe,
    const real_T pdf_LogWeights[3], const boolean_T pdf_IsValid[3], real_T dT)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T Psi[3];
  real_T pdf_LogWeights_data[3];
  real_T Ps;
  int32_T pdf_LogWeights_size[2];
  int32_T k;
  int32_T partialTrueCount;
  int32_T trueCount;
  int8_T tmp_data[3];
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
  Psi[0] = 0.0;
  Psi[1] = 0.0;
  Psi[2] = 0.0;
  if (pdf_IsValid[0]) {
    st.site = &fw_emlrtRSI;
    b_st.site = &iw_emlrtRSI;
    c_st.site = &jw_emlrtRSI;
    d_st.site = &sd_emlrtRSI;
    e_st.site = &td_emlrtRSI;
    if ((c_estimator_Estimators_f1_Targe->SurvivalModel.SurvivalRate < 0.0) &&
        (!muDoubleScalarIsNaN(dT)) && (muDoubleScalarFloor(dT) != dT)) {
      emlrtErrorWithMessageIdR2018a(&e_st, &w_emlrtRTEI,
                                    "Coder:toolbox:power_domainError",
                                    "Coder:toolbox:power_domainError", 0);
    }
    Psi[0] = muDoubleScalarPower(
        c_estimator_Estimators_f1_Targe->SurvivalModel.SurvivalRate, dT);
  }
  if (pdf_IsValid[1]) {
    st.site = &fw_emlrtRSI;
    b_st.site = &iw_emlrtRSI;
    c_st.site = &jw_emlrtRSI;
    d_st.site = &sd_emlrtRSI;
    e_st.site = &td_emlrtRSI;
    if ((c_estimator_Estimators_f2_Targe->SurvivalModel.SurvivalRate < 0.0) &&
        (!muDoubleScalarIsNaN(dT)) && (muDoubleScalarFloor(dT) != dT)) {
      emlrtErrorWithMessageIdR2018a(&e_st, &w_emlrtRTEI,
                                    "Coder:toolbox:power_domainError",
                                    "Coder:toolbox:power_domainError", 0);
    }
    Psi[1] = muDoubleScalarPower(
        c_estimator_Estimators_f2_Targe->SurvivalModel.SurvivalRate, dT);
  }
  if (pdf_IsValid[2]) {
    st.site = &fw_emlrtRSI;
    b_st.site = &iw_emlrtRSI;
    c_st.site = &jw_emlrtRSI;
    d_st.site = &sd_emlrtRSI;
    e_st.site = &td_emlrtRSI;
    if ((c_estimator_Estimators_f3_Targe->SurvivalModel.SurvivalRate < 0.0) &&
        (!muDoubleScalarIsNaN(dT)) && (muDoubleScalarFloor(dT) != dT)) {
      emlrtErrorWithMessageIdR2018a(&e_st, &w_emlrtRTEI,
                                    "Coder:toolbox:power_domainError",
                                    "Coder:toolbox:power_domainError", 0);
    }
    Psi[2] = muDoubleScalarPower(
        c_estimator_Estimators_f3_Targe->SurvivalModel.SurvivalRate, dT);
  }
  st.site = &gw_emlrtRSI;
  p = false;
  for (k = 0; k < 3; k++) {
    if (p || (Psi[k] < 0.0)) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  Psi[0] = muDoubleScalarLog(Psi[0]);
  Psi[1] = muDoubleScalarLog(Psi[1]);
  Psi[2] = muDoubleScalarLog(Psi[2]);
  trueCount = 0;
  if (pdf_IsValid[0]) {
    trueCount = 1;
  }
  if (pdf_IsValid[1]) {
    trueCount++;
  }
  if (pdf_IsValid[2]) {
    trueCount++;
  }
  partialTrueCount = 0;
  if (pdf_IsValid[0]) {
    tmp_data[0] = 0;
    partialTrueCount = 1;
  }
  if (pdf_IsValid[1]) {
    tmp_data[partialTrueCount] = 1;
    partialTrueCount++;
  }
  if (pdf_IsValid[2]) {
    tmp_data[partialTrueCount] = 2;
  }
  pdf_LogWeights_size[0] = 1;
  pdf_LogWeights_size[1] = trueCount;
  for (k = 0; k < trueCount; k++) {
    int8_T i;
    i = tmp_data[k];
    pdf_LogWeights_data[k] = pdf_LogWeights[i] + Psi[i];
  }
  st.site = &hw_emlrtRSI;
  Ps = logsumexp(&st, pdf_LogWeights_data, pdf_LogWeights_size);
  return muDoubleScalarExp(Ps);
}

/* End of code generation (MultiModalEstimator.c) */
