/*
 * fusecovint.c
 *
 * Code generation for function 'fusecovint'
 *
 */

/* Include files */
#include "fusecovint.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "validateCovFusion.h"
#include "xzgetrf.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo er_emlrtRSI = {
    82,           /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo fr_emlrtRSI = {
    106,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo gr_emlrtRSI = {
    124,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo hr_emlrtRSI = {
    136,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo ir_emlrtRSI = {
    142,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo jr_emlrtRSI = {
    150,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo kr_emlrtRSI = {
    154,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo lr_emlrtRSI = {
    160,          /* lineNo */
    "fusecovint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pathName
                                                                          */
};

static emlrtRSInfo pr_emlrtRSI =
    {
        74,               /* lineNo */
        "validatestring", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pathName */
};

static emlrtRSInfo qr_emlrtRSI =
    {
        111,                  /* lineNo */
        "fullValidatestring", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pathName */
};

static emlrtRSInfo rr_emlrtRSI =
    {
        164,         /* lineNo */
        "get_match", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pathName */
};

static emlrtRSInfo sr_emlrtRSI = {
    41,    /* lineNo */
    "cat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

static emlrtRTEInfo ac_emlrtRTEI =
    {
        131,                  /* lineNo */
        9,                    /* colNo */
        "fullValidatestring", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\validatestring"
        ".m" /* pName */
};

static emlrtBCInfo ag_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    160,          /* lineNo */
    102,          /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo bg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    160,          /* lineNo */
    76,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo cg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    160,          /* lineNo */
    54,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo dg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    150,          /* lineNo */
    70,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo eg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    150,          /* lineNo */
    48,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo fg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    124,          /* lineNo */
    48,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo gg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    119,          /* lineNo */
    50,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo hg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    119,          /* lineNo */
    23,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo ig_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    132,          /* lineNo */
    23,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo jg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    132,          /* lineNo */
    7,            /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo kg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    132,          /* lineNo */
    9,            /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo lg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    133,          /* lineNo */
    28,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo mg_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    133,          /* lineNo */
    7,            /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo ng_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    133,          /* lineNo */
    9,            /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo og_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    124,          /* lineNo */
    23,           /* colNo */
    "",           /* aName */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtRTEInfo nh_emlrtRTEI = {
    112,          /* lineNo */
    1,            /* colNo */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pName
                                                                          */
};

static emlrtRTEInfo oh_emlrtRTEI = {
    112,          /* lineNo */
    13,           /* colNo */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pName
                                                                          */
};

static emlrtRTEInfo ph_emlrtRTEI = {
    129,          /* lineNo */
    1,            /* colNo */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pName
                                                                          */
};

static emlrtRTEInfo qh_emlrtRTEI = {
    245,   /* lineNo */
    14,    /* colNo */
    "cat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

static emlrtRTEInfo rh_emlrtRTEI = {
    139,          /* lineNo */
    3,            /* colNo */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pName
                                                                          */
};

static emlrtRTEInfo sh_emlrtRTEI = {
    142,          /* lineNo */
    1,            /* colNo */
    "fusecovint", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fusecovint.m" /* pName
                                                                          */
};

/* Function Definitions */
void fusecovint(const emlrtStack *sp, const emxArray_real_T *trackState,
                const emxArray_real_T *trackCov, const char_T minProp[5],
                real_T fusedState[6], real_T fusedCov[36])
{
  static const char_T b_cv[11] = {'d', 'e', 't', 'e', 'r', 'm',
                                  'i', 'n', 'a', 'n', 't'};
  static const char_T vstr[11] = {'d', 'e', 't', 'e', 'r', 'm',
                                  'i', 'n', 'a', 'n', 't'};
  static const char_T b_cv2[5] = {'t', 'r', 'a', 'c', 'e'};
  static const char_T b_vstr[5] = {'t', 'r', 'a', 'c', 'e'};
  static const char_T cv3[3] = {'d', 'e', 't'};
  __m128d r1;
  __m128d r2;
  __m128d r3;
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  emxArray_real_T *A;
  emxArray_real_T *CovMatrix;
  emxArray_real_T *Weight;
  emxArray_real_T *b_A;
  emxArray_real_T *r;
  real_T b[36];
  real_T initialFusedCov[36];
  real_T initialFusedState[6];
  const real_T *trackCov_data;
  const real_T *trackState_data;
  real_T t;
  real_T y;
  real_T *A_data;
  real_T *CovMatrix_data;
  int32_T ipiv[6];
  int32_T b_i;
  int32_T c_i;
  int32_T d_i;
  int32_T exitg1;
  int32_T fusecovint_numThreads;
  int32_T fusedCov_tmp;
  int32_T i;
  int32_T k;
  int32_T minProp_size_idx_1;
  int32_T nmatched;
  int32_T partial_match_size_idx_1;
  char_T minProp_data[11];
  char_T partial_match_data[11];
  char_T b_cv1[10];
  uint8_T u;
  boolean_T emlrtHadParallelError = false;
  boolean_T isodd;
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
  trackCov_data = trackCov->data;
  trackState_data = trackState->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &er_emlrtRSI;
  validateCovFusion(&st, trackState, trackCov);
  if (trackCov->size[2] == 1) {
    for (i = 0; i < 6; i++) {
      fusedState[i] = trackState_data[i];
      for (b_i = 0; b_i < 6; b_i++) {
        fusedCov_tmp = b_i + 6 * i;
        fusedCov[fusedCov_tmp] = trackCov_data[fusedCov_tmp];
      }
    }
  } else {
    st.site = &fr_emlrtRSI;
    b_st.site = &pr_emlrtRSI;
    c_st.site = &qr_emlrtRSI;
    partial_match_size_idx_1 = 3;
    partial_match_data[0] = ' ';
    partial_match_data[1] = ' ';
    partial_match_data[2] = ' ';
    nmatched = 0;
    d_st.site = &rr_emlrtRSI;
    e_st.site = &kg_emlrtRSI;
    f_st.site = &lg_emlrtRSI;
    isodd = false;
    fusedCov_tmp = 0;
    do {
      exitg1 = 0;
      if (fusedCov_tmp < 5) {
        g_st.site = &mg_emlrtRSI;
        u = (uint8_T)minProp[fusedCov_tmp];
        if (u > 127) {
          emlrtErrorWithMessageIdR2018a(
              &g_st, &i_emlrtRTEI, "Coder:toolbox:unsupportedString",
              "Coder:toolbox:unsupportedString", 2, 12, 127);
        }
        if (cv[u] != cv[(int32_T)b_cv[fusedCov_tmp]]) {
          exitg1 = 1;
        } else {
          fusedCov_tmp++;
        }
      } else {
        isodd = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    if (isodd) {
      partial_match_size_idx_1 = 11;
      for (i = 0; i < 11; i++) {
        partial_match_data[i] = vstr[i];
      }
      nmatched = 1;
    }
    d_st.site = &rr_emlrtRSI;
    e_st.site = &kg_emlrtRSI;
    f_st.site = &lg_emlrtRSI;
    isodd = false;
    fusedCov_tmp = 0;
    do {
      exitg1 = 0;
      if (fusedCov_tmp < 5) {
        g_st.site = &mg_emlrtRSI;
        u = (uint8_T)minProp[fusedCov_tmp];
        if (u > 127) {
          emlrtErrorWithMessageIdR2018a(
              &g_st, &i_emlrtRTEI, "Coder:toolbox:unsupportedString",
              "Coder:toolbox:unsupportedString", 2, 12, 127);
        }
        if (cv[u] != cv[(int32_T)b_cv2[fusedCov_tmp]]) {
          exitg1 = 1;
        } else {
          fusedCov_tmp++;
        }
      } else {
        isodd = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    if (isodd) {
      nmatched = 1;
      partial_match_size_idx_1 = 5;
      for (i = 0; i < 5; i++) {
        partial_match_data[i] = b_vstr[i];
      }
    } else if (nmatched == 0) {
      partial_match_size_idx_1 = 3;
      partial_match_data[0] = ' ';
      partial_match_data[1] = ' ';
      partial_match_data[2] = ' ';
    }
    if (nmatched == 0) {
      b_cv1[0] = ',';
      b_cv1[1] = ' ';
      b_cv1[2] = '\'';
      for (i = 0; i < 5; i++) {
        b_cv1[i + 3] = minProp[i];
      }
      b_cv1[8] = '\'';
      b_cv1[9] = ',';
      emlrtErrorWithMessageIdR2018a(
          &b_st, &ac_emlrtRTEI,
          "Coder:toolbox:ValidatestringUnrecognizedStringChoice",
          "MATLAB:fusecovint:unrecognizedStringChoice", 9, 4, 24,
          "input number 3, minProp,", 4, 29,
          "\'det\', \'determinant\', \'trace\'", 4, 10, &b_cv1[0]);
    } else {
      minProp_size_idx_1 = partial_match_size_idx_1;
      memcpy(&minProp_data[0], &partial_match_data[0],
             (uint32_T)partial_match_size_idx_1 * sizeof(char_T));
    }
    isodd = false;
    if (minProp_size_idx_1 == 11) {
      fusedCov_tmp = 0;
      do {
        exitg1 = 0;
        if (fusedCov_tmp < 11) {
          if (minProp_data[fusedCov_tmp] != b_cv[fusedCov_tmp]) {
            exitg1 = 1;
          } else {
            fusedCov_tmp++;
          }
        } else {
          isodd = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (isodd) {
      minProp_size_idx_1 = 3;
      minProp_data[0] = 'd';
      minProp_data[1] = 'e';
      minProp_data[2] = 't';
    }
    emxInit_real_T(sp, &CovMatrix, 2, &nh_emlrtRTEI);
    fusedCov_tmp = CovMatrix->size[0] * CovMatrix->size[1];
    CovMatrix->size[0] = 1;
    partial_match_size_idx_1 = trackCov->size[2];
    CovMatrix->size[1] = trackCov->size[2];
    emxEnsureCapacity_real_T(sp, CovMatrix, fusedCov_tmp, &nh_emlrtRTEI);
    CovMatrix_data = CovMatrix->data;
    for (i = 0; i < partial_match_size_idx_1; i++) {
      CovMatrix_data[i] = 0.0;
    }
    isodd = false;
    if (minProp_size_idx_1 == 5) {
      fusedCov_tmp = 0;
      do {
        exitg1 = 0;
        if (fusedCov_tmp < 5) {
          if (b_cv2[fusedCov_tmp] != minProp_data[fusedCov_tmp]) {
            exitg1 = 1;
          } else {
            fusedCov_tmp++;
          }
        } else {
          isodd = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (isodd) {
      fusedCov_tmp = 0;
    } else {
      isodd = false;
      if (minProp_size_idx_1 == 3) {
        fusedCov_tmp = 0;
        do {
          exitg1 = 0;
          if (fusedCov_tmp < 3) {
            if (cv3[fusedCov_tmp] != minProp_data[fusedCov_tmp]) {
              exitg1 = 1;
            } else {
              fusedCov_tmp++;
            }
          } else {
            isodd = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (isodd) {
        fusedCov_tmp = 1;
      } else {
        fusedCov_tmp = -1;
      }
    }
    switch (fusedCov_tmp) {
    case 0:
      nmatched = trackCov->size[2];
      fusedCov_tmp = CovMatrix->size[0] * CovMatrix->size[1];
      CovMatrix->size[0] = 1;
      CovMatrix->size[1] = trackCov->size[2];
      emxEnsureCapacity_real_T(sp, CovMatrix, fusedCov_tmp, &oh_emlrtRTEI);
      CovMatrix_data = CovMatrix->data;
      if (trackCov->size[2] * 6 < 800) {
        for (c_i = 0; c_i < partial_match_size_idx_1; c_i++) {
          if (c_i + 1 > partial_match_size_idx_1) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, partial_match_size_idx_1,
                                          &gg_emlrtBCI, (emlrtConstCTX)sp);
          }
          t = 0.0;
          for (k = 0; k < 6; k++) {
            t += trackCov_data[(k + 6 * k) + 36 * c_i];
          }
          if (c_i + 1 > CovMatrix->size[1]) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, CovMatrix->size[1],
                                          &hg_emlrtBCI, (emlrtConstCTX)sp);
          }
          CovMatrix_data[c_i] = t;
        }
      } else {
        emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
        emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
        fusecovint_numThreads =
            emlrtAllocRegionTLSs(sp->tls, omp_in_parallel(),
                                 omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(fusecovint_numThreads) private(               \
        t, k, h_st, emlrtJBEnviron) firstprivate(emlrtHadParallelError)
        {
          if (setjmp(emlrtJBEnviron) == 0) {
            h_st.prev = sp;
            h_st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
            h_st.site = NULL;
            emlrtSetJmpBuf(&h_st, &emlrtJBEnviron);
          } else {
            emlrtHadParallelError = true;
          }
#pragma omp for nowait
          for (c_i = 0; c_i < nmatched; c_i++) {
            if (emlrtHadParallelError) {
              continue;
            }
            if (setjmp(emlrtJBEnviron) == 0) {
              if (c_i + 1 > trackCov->size[2]) {
                emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, trackCov->size[2],
                                              &gg_emlrtBCI, &h_st);
              }
              t = 0.0;
              for (k = 0; k < 6; k++) {
                t += trackCov_data[(k + 6 * k) + 36 * c_i];
              }
              if (c_i + 1 > CovMatrix->size[1]) {
                emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, CovMatrix->size[1],
                                              &hg_emlrtBCI, &h_st);
              }
              CovMatrix_data[c_i] = t;
            } else {
              emlrtHadParallelError = true;
            }
          }
        }
        emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
        emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
      }
      break;
    case 1:
      fusedCov_tmp = CovMatrix->size[0] * CovMatrix->size[1];
      CovMatrix->size[0] = 1;
      CovMatrix->size[1] = trackCov->size[2];
      emxEnsureCapacity_real_T(sp, CovMatrix, fusedCov_tmp, &oh_emlrtRTEI);
      CovMatrix_data = CovMatrix->data;
      for (b_i = 0; b_i < partial_match_size_idx_1; b_i++) {
        st.site = &gr_emlrtRSI;
        if (b_i + 1 > partial_match_size_idx_1) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, partial_match_size_idx_1,
                                        &fg_emlrtBCI, &st);
        }
        b_st.site = &ak_emlrtRSI;
        for (i = 0; i < 36; i++) {
          initialFusedCov[i] = trackCov_data[i + b_i * 36];
        }
        c_st.site = &rj_emlrtRSI;
        xzgetrf(&c_st, initialFusedCov, ipiv);
        y = initialFusedCov[0];
        isodd = false;
        for (i = 0; i < 5; i++) {
          y *= initialFusedCov[(i + 6 * (i + 1)) + 1];
          if (ipiv[i] > i + 1) {
            isodd = !isodd;
          }
        }
        if (isodd) {
          y = -y;
        }
        if (b_i + 1 > CovMatrix->size[1]) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, CovMatrix->size[1],
                                        &og_emlrtBCI, (emlrtConstCTX)sp);
        }
        CovMatrix_data[b_i] = y;
      }
      break;
    }
    emxInit_real_T(sp, &A, 2, &ph_emlrtRTEI);
    fusedCov_tmp = A->size[0] * A->size[1];
    A->size[0] = trackCov->size[2] - 1;
    A->size[1] = partial_match_size_idx_1;
    emxEnsureCapacity_real_T(sp, A, fusedCov_tmp, &ph_emlrtRTEI);
    A_data = A->data;
    fusedCov_tmp = (trackCov->size[2] - 1) * trackCov->size[2];
    for (i = 0; i < fusedCov_tmp; i++) {
      A_data[i] = 0.0;
    }
    for (i = 0; i <= partial_match_size_idx_1 - 2; i++) {
      if (i + 1 > CovMatrix->size[1]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, CovMatrix->size[1],
                                      &ig_emlrtBCI, (emlrtConstCTX)sp);
      }
      if (i + 1 > A->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, A->size[0], &jg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i + 1 > A->size[1]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, A->size[1], &kg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      A_data[i + A->size[0] * i] = CovMatrix_data[i];
      if (i + 2 > CovMatrix->size[1]) {
        emlrtDynamicBoundsCheckR2012b(i + 2, 1, CovMatrix->size[1],
                                      &lg_emlrtBCI, (emlrtConstCTX)sp);
      }
      if (i + 1 > A->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, A->size[0], &mg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i + 2 > A->size[1]) {
        emlrtDynamicBoundsCheckR2012b(i + 2, 1, A->size[1], &ng_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      A_data[i + A->size[0] * (i + 1)] = -CovMatrix_data[i + 1];
    }
    emxFree_real_T(sp, &CovMatrix);
    st.site = &hr_emlrtRSI;
    b_st.site = &sr_emlrtRSI;
    c_st.site = &rp_emlrtRSI;
    if (A->size[1] != trackCov->size[2]) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &rb_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    emxInit_real_T(sp, &b_A, 2, &qh_emlrtRTEI);
    fusedCov_tmp = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0] + 1;
    nmatched = A->size[1];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(sp, b_A, fusedCov_tmp, &qh_emlrtRTEI);
    CovMatrix_data = b_A->data;
    for (i = 0; i < nmatched; i++) {
      fusedCov_tmp = A->size[0];
      for (b_i = 0; b_i < fusedCov_tmp; b_i++) {
        CovMatrix_data[b_i + b_A->size[0] * i] = A_data[b_i + A->size[0] * i];
      }
      CovMatrix_data[fusedCov_tmp + b_A->size[0] * i] = 1.0;
    }
    emxFree_real_T(sp, &A);
    emxInit_real_T(sp, &r, 1, &rh_emlrtRTEI);
    fusedCov_tmp = r->size[0];
    r->size[0] = partial_match_size_idx_1;
    emxEnsureCapacity_real_T(sp, r, fusedCov_tmp, &rh_emlrtRTEI);
    CovMatrix_data = r->data;
    for (i = 0; i <= partial_match_size_idx_1 - 2; i++) {
      CovMatrix_data[i] = 0.0;
    }
    CovMatrix_data[trackCov->size[2] - 1] = 1.0;
    emxInit_real_T(sp, &Weight, 1, &sh_emlrtRTEI);
    st.site = &ir_emlrtRSI;
    mldivide(&st, b_A, r, Weight);
    CovMatrix_data = Weight->data;
    emxFree_real_T(sp, &r);
    emxFree_real_T(sp, &b_A);
    memset(&initialFusedCov[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < partial_match_size_idx_1; i++) {
      if (i + 1 > Weight->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, Weight->size[0], &eg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i + 1 > partial_match_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, partial_match_size_idx_1,
                                      &dg_emlrtBCI, (emlrtConstCTX)sp);
      }
      st.site = &jr_emlrtRSI;
      b_mldivide(&st, &trackCov_data[36 * i], b);
      for (b_i = 0; b_i <= 34; b_i += 2) {
        r1 = _mm_loadu_pd(&b[b_i]);
        r2 = _mm_loadu_pd(&initialFusedCov[b_i]);
        _mm_storeu_pd(
            &initialFusedCov[b_i],
            _mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(CovMatrix_data[i]), r1)));
      }
    }
    st.site = &kr_emlrtRSI;
    b_mldivide(&st, initialFusedCov, fusedCov);
    for (i = 0; i < 6; i++) {
      initialFusedState[i] = 0.0;
    }
    for (i = 0; i < partial_match_size_idx_1; i++) {
      if (i + 1 > Weight->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, Weight->size[0], &cg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i + 1 > partial_match_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, partial_match_size_idx_1,
                                      &bg_emlrtBCI, (emlrtConstCTX)sp);
      }
      st.site = &lr_emlrtRSI;
      b_mldivide(&st, &trackCov_data[36 * i], b);
      if (i + 1 > trackState->size[1]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, trackState->size[1],
                                      &ag_emlrtBCI, (emlrtConstCTX)sp);
      }
      for (b_i = 0; b_i < 6; b_i++) {
        y = 0.0;
        for (d_i = 0; d_i < 6; d_i++) {
          y += CovMatrix_data[i] * b[b_i + 6 * d_i] *
               trackState_data[d_i + 6 * i];
        }
        initialFusedState[b_i] += y;
      }
    }
    emxFree_real_T(sp, &Weight);
    memset(&fusedState[0], 0, 6U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      r1 = _mm_loadu_pd(&fusedCov[6 * i]);
      r2 = _mm_loadu_pd(&fusedState[0]);
      r3 = _mm_set1_pd(initialFusedState[i]);
      _mm_storeu_pd(&fusedState[0], _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
      r1 = _mm_loadu_pd(&fusedCov[6 * i + 2]);
      r2 = _mm_loadu_pd(&fusedState[2]);
      _mm_storeu_pd(&fusedState[2], _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
      r1 = _mm_loadu_pd(&fusedCov[6 * i + 4]);
      r2 = _mm_loadu_pd(&fusedState[4]);
      _mm_storeu_pd(&fusedState[4], _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (fusecovint.c) */
