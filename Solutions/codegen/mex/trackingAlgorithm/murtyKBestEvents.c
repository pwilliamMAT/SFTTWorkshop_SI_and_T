/*
 * murtyKBestEvents.c
 *
 * Code generation for function 'murtyKBestEvents'
 *
 */

/* Include files */
#include "murtyKBestEvents.h"
#include "JVProblemSolutionPair.h"
#include "ProblemSolutionQueue.h"
#include "eml_int_forloop_overflow_check.h"
#include "logsumexp.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo qdb_emlrtRSI = {
    15,                 /* lineNo */
    "murtyKBestEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtRSInfo rdb_emlrtRSI = {
    21,                 /* lineNo */
    "murtyKBestEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtRSInfo sdb_emlrtRSI = {
    30,                 /* lineNo */
    "murtyKBestEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtRSInfo tdb_emlrtRSI = {
    36,                 /* lineNo */
    "murtyKBestEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtRSInfo udb_emlrtRSI = {
    47,                 /* lineNo */
    "murtyKBestEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtRSInfo vdb_emlrtRSI = {
    55,                        /* lineNo */
    "convertLikelihoodToCost", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtRSInfo xdb_emlrtRSI = {
    79,            /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo ydb_emlrtRSI = {
    102,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo aeb_emlrtRSI = {
    109,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo beb_emlrtRSI = {
    117,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo ceb_emlrtRSI = {
    120,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo deb_emlrtRSI = {
    123,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo eeb_emlrtRSI = {
    124,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo feb_emlrtRSI = {
    131,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo geb_emlrtRSI = {
    90,            /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo heb_emlrtRSI = {
    101,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo ieb_emlrtRSI = {
    105,           /* lineNo */
    "assignkbest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pathName
                                                                           */
};

static emlrtRSInfo jeb_emlrtRSI = {
    16,                       /* lineNo */
    "parseAssignkbestInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\parseAssignkbestInputs.m" /* pathName */
};

static emlrtRSInfo keb_emlrtRSI = {
    28,                       /* lineNo */
    "parseAssignkbestInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\parseAssignkbestInputs.m" /* pathName */
};

static emlrtRSInfo leb_emlrtRSI = {
    30,                       /* lineNo */
    "parseAssignkbestInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\parseAssignkbestInputs.m" /* pathName */
};

static emlrtRSInfo meb_emlrtRSI = {
    43,                       /* lineNo */
    "parseAssignkbestInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\parseAssignkbestInputs.m" /* pathName */
};

static emlrtRSInfo neb_emlrtRSI = {
    10,                   /* lineNo */
    "lapCheckCostMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapCheckCostMatrix.m" /* pathName */
};

static emlrtRSInfo xhb_emlrtRSI = {
    115,                                /* lineNo */
    "ProblemSolutionQueue/addProblems", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo uib_emlrtRSI = {
    67,                         /* lineNo */
    "convertAssignmentToEvent", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pathName */
};

static emlrtBCInfo be_emlrtBCI = {
    -1,                 /* iFirst */
    -1,                 /* iLast */
    47,                 /* lineNo */
    13,                 /* colNo */
    "",                 /* aName */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtECInfo p_emlrtECI = {
    -1,                 /* nDims */
    47,                 /* lineNo */
    5,                  /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtBCInfo ce_emlrtBCI = {
    -1,                 /* iFirst */
    -1,                 /* iLast */
    47,                 /* lineNo */
    55,                 /* colNo */
    "",                 /* aName */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtBCInfo de_emlrtBCI = {
    -1,                 /* iFirst */
    -1,                 /* iLast */
    47,                 /* lineNo */
    73,                 /* colNo */
    "",                 /* aName */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtBCInfo ee_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    58,                        /* lineNo */
    30,                        /* colNo */
    "",                        /* aName */
    "convertLikelihoodToCost", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    57,                        /* lineNo */
    28,                        /* colNo */
    "",                        /* aName */
    "convertLikelihoodToCost", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtBCInfo ge_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    56,                        /* lineNo */
    27,                        /* colNo */
    "",                        /* aName */
    "convertLikelihoodToCost", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtBCInfo he_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    56,                        /* lineNo */
    21,                        /* colNo */
    "",                        /* aName */
    "convertLikelihoodToCost", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtRTEInfo tb_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatenonnan", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonnan.m" /* pName */
};

static emlrtBCInfo ie_emlrtBCI = {
    -1,                                 /* iFirst */
    -1,                                 /* iLast */
    116,                                /* lineNo */
    40,                                 /* colNo */
    "",                                 /* aName */
    "ProblemSolutionQueue/addProblems", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo je_emlrtBCI = {
    -1,                                 /* iFirst */
    -1,                                 /* iLast */
    116,                                /* lineNo */
    58,                                 /* colNo */
    "",                                 /* aName */
    "ProblemSolutionQueue/addProblems", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo ke_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    66,                         /* lineNo */
    7,                          /* colNo */
    "",                         /* aName */
    "convertAssignmentToEvent", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtBCInfo le_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    68,                         /* lineNo */
    7,                          /* colNo */
    "",                         /* aName */
    "convertAssignmentToEvent", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m", /* pName */
    0                                            /* checkKind */
};

static emlrtRTEInfo eg_emlrtRTEI =
    {
        1,     /* lineNo */
        18,    /* colNo */
        "log", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elfun\\log.m" /* pName
                                                                          */
};

static emlrtRTEInfo fg_emlrtRTEI = {
    58,                 /* lineNo */
    1,                  /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo gg_emlrtRTEI = {
    56,                 /* lineNo */
    14,                 /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo hg_emlrtRTEI = {
    102,           /* lineNo */
    1,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

static emlrtRTEInfo ig_emlrtRTEI = {
    27,                 /* lineNo */
    1,                  /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo jg_emlrtRTEI = {
    117,           /* lineNo */
    5,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

static emlrtRTEInfo kg_emlrtRTEI = {
    123,           /* lineNo */
    5,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

static emlrtRTEInfo lg_emlrtRTEI = {
    44,                 /* lineNo */
    1,                  /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo mg_emlrtRTEI = {
    47,                 /* lineNo */
    18,                 /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo ng_emlrtRTEI = {
    47,                 /* lineNo */
    5,                  /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo og_emlrtRTEI = {
    1,                  /* lineNo */
    28,                 /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo pg_emlrtRTEI = {
    55,                 /* lineNo */
    1,                  /* colNo */
    "murtyKBestEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\murtyKBestEvents.m" /* pName */
};

static emlrtRTEInfo qg_emlrtRTEI = {
    84,            /* lineNo */
    9,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

static emlrtRTEInfo rg_emlrtRTEI = {
    101,           /* lineNo */
    1,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

static emlrtRTEInfo sg_emlrtRTEI = {
    105,           /* lineNo */
    1,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

static emlrtRTEInfo tg_emlrtRTEI = {
    120,           /* lineNo */
    5,             /* colNo */
    "assignkbest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\assignkbest.m" /* pName
                                                                           */
};

/* Function Definitions */
void murtyKBestEvents(const emlrtStack *sp,
                      const emxArray_real_T *likelihoodMatrix, real_T k,
                      emxArray_boolean_T *FJE, emxArray_real_T *FJEProbs)
{
  __m128d r;
  jmp_buf *volatile emlrtJBStack;
  c_emxArray_fusion_internal_assi *partitionSolutions;
  c_emxArray_fusion_internal_assi *solutionQueue_AllProblemList;
  c_fusion_internal_assignment_JV ps;
  c_fusion_internal_assignment_JV *c_solutionQueue_AllProblemList_;
  c_fusion_internal_assignment_JV *partitionSolutions_data;
  c_fusion_internal_assignment_Pr priorityQueue;
  cell_wrap_74 *assignments_data;
  cell_wrap_75 *unassignedRows_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_boolean_T *r1;
  emxArray_cell_wrap_74 *assignments;
  emxArray_cell_wrap_75 *unassignedCols;
  emxArray_cell_wrap_75 *unassignedRows;
  emxArray_real_T *b_cost;
  emxArray_real_T *c_cost;
  emxArray_real_T *cost;
  emxArray_real_T *costNonAssignment_f2;
  real_T costNonAssignment_f1_data[51];
  const real_T *likelihoodMatrix_data;
  real_T logsumpost;
  real_T *costNonAssignment_f2_data;
  real_T *cost_data;
  int32_T b_FJE[2];
  int32_T b_i;
  int32_T b_k;
  int32_T b_loop_ub;
  int32_T c_k;
  int32_T c_loop_ub;
  int32_T currentIteration;
  int32_T d_k;
  int32_T d_loop_ub;
  int32_T i;
  int32_T loop_ub;
  int32_T murtyKBestEvents_numThreads;
  int32_T nIn;
  int32_T nx;
  int32_T siz;
  int32_T siz_idx_0;
  int32_T siz_idx_1;
  int32_T vectorUB;
  int8_T varargin_1[2];
  int8_T varargin_2[2];
  boolean_T exitg1;
  boolean_T p;
  boolean_T *FJE_data;
  boolean_T *r2;
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
  likelihoodMatrix_data = likelihoodMatrix->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &qdb_emlrtRSI;
  emxInit_real_T(&st, &cost, 2, &pg_emlrtRTEI, true);
  b_st.site = &vdb_emlrtRSI;
  currentIteration = cost->size[0] * cost->size[1];
  cost->size[0] = likelihoodMatrix->size[0];
  loop_ub = likelihoodMatrix->size[1];
  cost->size[1] = likelihoodMatrix->size[1];
  emxEnsureCapacity_real_T(&b_st, cost, currentIteration, &eg_emlrtRTEI);
  cost_data = cost->data;
  nx = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
  for (b_k = 0; b_k < nx; b_k++) {
    cost_data[b_k] = likelihoodMatrix_data[b_k];
  }
  p = false;
  for (b_k = 0; b_k < nx; b_k++) {
    if (p || (likelihoodMatrix_data[b_k] < 0.0)) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &x_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
  }
  c_st.site = &wdb_emlrtRSI;
  d_st.site = &xr_emlrtRSI;
  if (nx > 2147483646) {
    e_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }
  if (nx < 800) {
    for (c_k = 0; c_k < nx; c_k++) {
      cost_data[c_k] = muDoubleScalarLog(cost_data[c_k]);
    }
  } else {
    emlrtEnterParallelRegion(&c_st, omp_in_parallel());
    emlrtPushJmpBuf(&c_st, &emlrtJBStack);
    murtyKBestEvents_numThreads =
        emlrtAllocRegionTLSs(c_st.tls, omp_in_parallel(), omp_get_max_threads(),
                             omp_get_num_procs());
#pragma omp parallel for num_threads(murtyKBestEvents_numThreads)

    for (c_k = 0; c_k < nx; c_k++) {
      cost_data[c_k] = muDoubleScalarLog(cost_data[c_k]);
    }
    emlrtPopJmpBuf(&c_st, &emlrtJBStack);
    emlrtExitParallelRegion(&c_st, omp_in_parallel());
  }
  currentIteration = (nx / 2) << 1;
  vectorUB = currentIteration - 2;
  for (b_k = 0; b_k <= vectorUB; b_k += 2) {
    r = _mm_loadu_pd(&cost_data[b_k]);
    _mm_storeu_pd(&cost_data[b_k], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }
  for (b_k = currentIteration; b_k < nx; b_k++) {
    cost_data[b_k] = -cost_data[b_k];
  }
  nIn = cost->size[0];
  if (cost->size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(cost->size[0], 1, cost->size[0], &he_emlrtBCI,
                                  &st);
  }
  b_loop_ub = cost->size[1];
  if (cost->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(cost->size[1], 1, cost->size[1], &ge_emlrtBCI,
                                  &st);
  }
  if (cost->size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(cost->size[0], 1, cost->size[0], &fe_emlrtBCI,
                                  &st);
  }
  nx = cost->size[0] - 1;
  for (b_k = 0; b_k <= nIn - 2; b_k++) {
    costNonAssignment_f1_data[b_k] = cost_data[b_k + 1];
  }
  if (cost->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(cost->size[1], 1, cost->size[1], &ee_emlrtBCI,
                                  &st);
  }
  emxInit_real_T(&st, &costNonAssignment_f2, 2, &og_emlrtRTEI, true);
  currentIteration =
      costNonAssignment_f2->size[0] * costNonAssignment_f2->size[1];
  costNonAssignment_f2->size[0] = 1;
  costNonAssignment_f2->size[1] = cost->size[1] - 1;
  emxEnsureCapacity_real_T(&st, costNonAssignment_f2, currentIteration,
                           &fg_emlrtRTEI);
  costNonAssignment_f2_data = costNonAssignment_f2->data;
  for (b_k = 0; b_k <= b_loop_ub - 2; b_k++) {
    costNonAssignment_f2_data[b_k] = cost_data[cost->size[0] * (b_k + 1)];
  }
  st.site = &rdb_emlrtRSI;
  b_st.site = &xdb_emlrtRSI;
  c_st.site = &jeb_emlrtRSI;
  d_st.site = &neb_emlrtRSI;
  e_st.site = &vd_emlrtRSI;
  p = true;
  currentIteration = (cost->size[0] - 1) * (cost->size[1] - 1);
  vectorUB = 0;
  exitg1 = false;
  while ((!exitg1) && (vectorUB <= currentIteration - 1)) {
    if (!muDoubleScalarIsNaN(
            cost_data[(vectorUB % (cost->size[0] - 1) +
                       cost->size[0] * (vectorUB / (cost->size[0] - 1) + 1)) +
                      1])) {
      vectorUB++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &tb_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:assignkbest:expectedNonNaN", 3, 4, 10, "COSTMATRIX");
  }
  c_st.site = &keb_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  p = true;
  currentIteration = 1;
  exitg1 = false;
  while ((!exitg1) && (currentIteration - 1 <= cost->size[0] - 2)) {
    logsumpost = cost_data[currentIteration];
    if ((!muDoubleScalarIsInf(logsumpost)) &&
        (!muDoubleScalarIsNaN(logsumpost))) {
      currentIteration++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:assignkbest:expectedFinite", 3, 4, 22,
        "costOfRowNonAssignment");
  }
  d_st.site = &vd_emlrtRSI;
  c_st.site = &leb_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  p = true;
  currentIteration = 1;
  exitg1 = false;
  while ((!exitg1) && (currentIteration - 1 <= cost->size[1] - 2)) {
    logsumpost = cost_data[cost->size[0] * currentIteration];
    if ((!muDoubleScalarIsInf(logsumpost)) &&
        (!muDoubleScalarIsNaN(logsumpost))) {
      currentIteration++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:assignkbest:expectedFinite", 3, 4, 22,
        "costOfColNonAssignment");
  }
  d_st.site = &vd_emlrtRSI;
  c_st.site = &meb_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  if (muDoubleScalarIsInf(k) || muDoubleScalarIsNaN(k)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:assignkbest:expectedFinite", 3, 4, 1, "k");
  }
  d_st.site = &vd_emlrtRSI;
  if (!(muDoubleScalarFloor(k) == k)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &mb_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:assignkbest:expectedInteger", 3, 4, 1, "k");
  }
  emxInit_real_T(&st, &b_cost, 2, &gg_emlrtRTEI, true);
  currentIteration = b_cost->size[0] * b_cost->size[1];
  b_cost->size[0] = cost->size[0] - 1;
  b_cost->size[1] = cost->size[1] - 1;
  emxEnsureCapacity_real_T(&st, b_cost, currentIteration, &gg_emlrtRTEI);
  costNonAssignment_f2_data = b_cost->data;
  for (b_k = 0; b_k <= b_loop_ub - 2; b_k++) {
    for (i = 0; i <= nIn - 2; i++) {
      costNonAssignment_f2_data[i + b_cost->size[0] * b_k] =
          cost_data[(i + cost->size[0] * (b_k + 1)) + 1];
    }
  }
  c_emxInitStruct_fusion_internal(&st, &ps, &qg_emlrtRTEI, true);
  b_st.site = &geb_emlrtRSI;
  ps.BestSolutionCost = c_JVProblemSolutionPair_JVProbl(
      &b_st, b_cost, costNonAssignment_f1_data, nx, costNonAssignment_f2,
      ps.PaddedCostMatrix, ps.RowSoln, ps.ColSoln, ps.IsEnforced, ps.CostSize,
      ps.IsDummySolution, ps.ColReduction, ps.RowReduction, &ps.IsSolved,
      &ps.LowerBound);
  emxFree_real_T(&st, &b_cost);
  emxFree_real_T(&st, &costNonAssignment_f2);
  currentIteration = 0;
  d_emxInitStruct_fusion_internal(&st, &priorityQueue, &rg_emlrtRTEI);
  b_st.site = &heb_emlrtRSI;
  priorityQueue.NumProblems = c_ProblemSolutionQueue_ProblemS(
      &b_st, &ps, ((cost->size[0] + cost->size[1]) - 2) * (int32_T)k + 1,
      priorityQueue.AllProblemList, &priorityQueue.MaxNumSubProblems);
  emxFree_real_T(&st, &cost);
  b_st.site = &ydb_emlrtRSI;
  if ((priorityQueue.NumProblems < 0) ||
      (priorityQueue.NumProblems > priorityQueue.AllProblemList->size[0] - 1)) {
    emlrtDynamicBoundsCheckR2012b(priorityQueue.NumProblems, 0,
                                  priorityQueue.AllProblemList->size[0] - 1,
                                  &ie_emlrtBCI, &b_st);
  }
  c_emxCopyStruct_fusion_internal(
      &b_st, &priorityQueue.AllProblemList->data[priorityQueue.NumProblems],
      &ps, &hg_emlrtRTEI);
  priorityQueue.NumProblems++;
  c_emxInit_fusion_internal_assig(&st, &solutionQueue_AllProblemList,
                                  &sg_emlrtRTEI);
  b_st.site = &ieb_emlrtRSI;
  nx = c_ProblemSolutionQueue_ProblemS(&b_st, &ps, (int32_T)k,
                                       solutionQueue_AllProblemList, &vectorUB);
  c_solutionQueue_AllProblemList_ = solutionQueue_AllProblemList->data;
  c_emxInit_fusion_internal_assig(&st, &partitionSolutions, &tg_emlrtRTEI);
  exitg1 = false;
  while ((!exitg1) && ((priorityQueue.NumProblems != 0) &&
                       (currentIteration < (int32_T)k))) {
    b_st.site = &aeb_emlrtRSI;
    p = c_ProblemSolutionQueue_extractB(&b_st, &priorityQueue, &ps);
    if (!p) {
      exitg1 = true;
    } else {
      b_st.site = &beb_emlrtRSI;
      if ((nx < 0) || (nx > solutionQueue_AllProblemList->size[0] - 1)) {
        emlrtDynamicBoundsCheckR2012b(nx, 0,
                                      solutionQueue_AllProblemList->size[0] - 1,
                                      &ie_emlrtBCI, &b_st);
      }
      c_emxCopyStruct_fusion_internal(
          &b_st, &c_solutionQueue_AllProblemList_[nx], &ps, &jg_emlrtRTEI);
      nx++;
      b_st.site = &ceb_emlrtRSI;
      JVProblemSolutionPair_partition(
          &b_st, ps.PaddedCostMatrix, ps.RowSoln, ps.ColSoln, ps.IsEnforced,
          ps.CostSize, ps.BestSolutionCost, ps.IsDummySolution, ps.ColReduction,
          ps.RowReduction, partitionSolutions);
      partitionSolutions_data = partitionSolutions->data;
      b_st.site = &deb_emlrtRSI;
      nIn = partitionSolutions->size[0];
      c_st.site = &xhb_emlrtRSI;
      if (partitionSolutions->size[0] > 2147483646) {
        d_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      for (b_k = 0; b_k < nIn; b_k++) {
        if (b_k > partitionSolutions->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(b_k, 0, partitionSolutions->size[0] - 1,
                                        &je_emlrtBCI, &b_st);
        }
        b_loop_ub = priorityQueue.NumProblems + b_k;
        if ((b_loop_ub < 0) ||
            (b_loop_ub > priorityQueue.AllProblemList->size[0] - 1)) {
          emlrtDynamicBoundsCheckR2012b(
              b_loop_ub, 0, priorityQueue.AllProblemList->size[0] - 1,
              &ie_emlrtBCI, &b_st);
        }
        c_emxCopyStruct_fusion_internal(
            &b_st, &priorityQueue.AllProblemList->data[b_loop_ub],
            &partitionSolutions_data[b_k], &kg_emlrtRTEI);
      }
      priorityQueue.NumProblems += partitionSolutions->size[0];
      b_st.site = &eeb_emlrtRSI;
      ProblemSolutionQueue_sortQueue(&b_st, &priorityQueue);
      currentIteration++;
    }
  }
  c_emxFree_fusion_internal_assig(&st, &partitionSolutions);
  d_emxFreeStruct_fusion_internal(&st, &priorityQueue);
  c_emxFreeStruct_fusion_internal(&st, &ps);
  emxInit_cell_wrap_74(&st, &assignments, &og_emlrtRTEI);
  emxInit_cell_wrap_75(&st, &unassignedRows, &og_emlrtRTEI);
  emxInit_cell_wrap_75(&st, &unassignedCols, &og_emlrtRTEI);
  emxInit_real_T(&st, &c_cost, 1, &og_emlrtRTEI, true);
  b_st.site = &feb_emlrtRSI;
  c_ProblemSolutionQueue_formatSo(&b_st, nx, vectorUB,
                                  solutionQueue_AllProblemList, assignments,
                                  unassignedRows, unassignedCols, c_cost);
  costNonAssignment_f2_data = c_cost->data;
  unassignedRows_data = unassignedRows->data;
  assignments_data = assignments->data;
  emxFree_cell_wrap_75(&st, &unassignedCols);
  c_emxFree_fusion_internal_assig(&st, &solutionQueue_AllProblemList);
  c_loop_ub = c_cost->size[0];
  currentIteration = FJEProbs->size[0];
  FJEProbs->size[0] = c_cost->size[0];
  emxEnsureCapacity_real_T(sp, FJEProbs, currentIteration, &ig_emlrtRTEI);
  cost_data = FJEProbs->data;
  currentIteration = (c_cost->size[0] / 2) << 1;
  vectorUB = currentIteration - 2;
  for (b_k = 0; b_k <= vectorUB; b_k += 2) {
    r = _mm_loadu_pd(&costNonAssignment_f2_data[b_k]);
    _mm_storeu_pd(&cost_data[b_k], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }
  for (b_k = currentIteration; b_k < c_loop_ub; b_k++) {
    cost_data[b_k] = -costNonAssignment_f2_data[b_k];
  }
  st.site = &sdb_emlrtRSI;
  logsumpost = logsumexp(&st, FJEProbs);
  currentIteration = FJEProbs->size[0];
  vectorUB = (FJEProbs->size[0] / 2) << 1;
  nx = vectorUB - 2;
  for (b_k = 0; b_k <= nx; b_k += 2) {
    r = _mm_loadu_pd(&cost_data[b_k]);
    _mm_storeu_pd(&cost_data[b_k], _mm_sub_pd(r, _mm_set1_pd(logsumpost)));
  }
  for (b_k = vectorUB; b_k < currentIteration; b_k++) {
    cost_data[b_k] -= logsumpost;
  }
  st.site = &tdb_emlrtRSI;
  b_st.site = &tib_emlrtRSI;
  currentIteration = FJEProbs->size[0];
  c_st.site = &xr_emlrtRSI;
  if (FJEProbs->size[0] > 2147483646) {
    d_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }
  if (FJEProbs->size[0] < 800) {
    for (d_k = 0; d_k < currentIteration; d_k++) {
      cost_data[d_k] = muDoubleScalarExp(cost_data[d_k]);
    }
  } else {
    emlrtEnterParallelRegion(&b_st, omp_in_parallel());
    emlrtPushJmpBuf(&b_st, &emlrtJBStack);
    murtyKBestEvents_numThreads =
        emlrtAllocRegionTLSs(b_st.tls, omp_in_parallel(), omp_get_max_threads(),
                             omp_get_num_procs());
#pragma omp parallel for num_threads(murtyKBestEvents_numThreads)

    for (d_k = 0; d_k < currentIteration; d_k++) {
      cost_data[d_k] = muDoubleScalarExp(cost_data[d_k]);
    }
    emlrtPopJmpBuf(&b_st, &emlrtJBStack);
    emlrtExitParallelRegion(&b_st, omp_in_parallel());
  }
  b_loop_ub = likelihoodMatrix->size[0] - 1;
  currentIteration = FJE->size[0] * FJE->size[1] * FJE->size[2];
  FJE->size[0] = likelihoodMatrix->size[0] - 1;
  FJE->size[1] = loop_ub;
  FJE->size[2] = c_loop_ub;
  emxEnsureCapacity_boolean_T(sp, FJE, currentIteration, &lg_emlrtRTEI);
  FJE_data = FJE->data;
  nx = (likelihoodMatrix->size[0] - 1) * likelihoodMatrix->size[1];
  currentIteration = nx * c_cost->size[0];
  emxFree_real_T(sp, &c_cost);
  for (b_k = 0; b_k < currentIteration; b_k++) {
    FJE_data[b_k] = false;
  }
  nIn = assignments->size[0];
  if (assignments->size[0] - 1 >= 0) {
    d_loop_ub = nx;
    siz_idx_0 = likelihoodMatrix->size[0] - 1;
    siz_idx_1 = loop_ub;
    varargin_1[1] = 1;
    varargin_2[1] = 1;
    siz = likelihoodMatrix->size[0] - 1;
  }
  emxInit_boolean_T(sp, &r1, 2, &ng_emlrtRTEI, true);
  for (b_i = 0; b_i < nIn; b_i++) {
    int32_T tmp_data[51];
    uint32_T varargin_2_data[51];
    if (b_i + 1 > c_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, c_loop_ub, &be_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    st.site = &udb_emlrtRSI;
    if (b_i > assignments->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, assignments->size[0] - 1,
                                    &ce_emlrtBCI, &st);
    }
    if (b_i > unassignedRows->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, unassignedRows->size[0] - 1,
                                    &de_emlrtBCI, &st);
    }
    vectorUB = r1->size[0] * r1->size[1];
    r1->size[0] = b_loop_ub;
    r1->size[1] = loop_ub;
    emxEnsureCapacity_boolean_T(&st, r1, vectorUB, &mg_emlrtRTEI);
    r2 = r1->data;
    for (b_k = 0; b_k < d_loop_ub; b_k++) {
      r2[b_k] = false;
    }
    currentIteration = unassignedRows_data[b_i].f1->size[0];
    for (b_k = 0; b_k < currentIteration; b_k++) {
      vectorUB = (int32_T)unassignedRows_data[b_i].f1->data[b_k];
      if ((vectorUB < 1) || (vectorUB > b_loop_ub)) {
        emlrtDynamicBoundsCheckR2012b(vectorUB, 1, b_loop_ub, &ke_emlrtBCI,
                                      &st);
      }
      tmp_data[b_k] = vectorUB;
    }
    for (b_k = 0; b_k < currentIteration; b_k++) {
      r2[tmp_data[b_k] - 1] = true;
    }
    b_st.site = &uib_emlrtRSI;
    currentIteration = assignments_data[b_i].f1.size[0];
    for (b_k = 0; b_k < currentIteration; b_k++) {
      uint32_T q0;
      uint32_T qY;
      q0 =
          assignments_data[b_i].f1.data[b_k + assignments_data[b_i].f1.size[0]];
      qY = q0 + 1U;
      if (q0 + 1U < q0) {
        qY = MAX_uint32_T;
      }
      varargin_2_data[b_k] = qY;
    }
    c_st.site = &wgb_emlrtRSI;
    vectorUB = 0;
    exitg1 = false;
    while ((!exitg1) && (vectorUB <= currentIteration - 1)) {
      if ((assignments_data[b_i].f1.data[vectorUB] >= 1U) &&
          (assignments_data[b_i].f1.data[vectorUB] <= (uint32_T)siz_idx_0)) {
        vectorUB++;
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
                                      "MATLAB:sub2ind:IndexOutOfRange",
                                      "MATLAB:sub2ind:IndexOutOfRange", 0);
      }
    }
    varargin_1[0] = (int8_T)currentIteration;
    varargin_2[0] = (int8_T)currentIteration;
    p = true;
    vectorUB = 0;
    exitg1 = false;
    while ((!exitg1) && (vectorUB < 2)) {
      if (varargin_1[vectorUB] != varargin_2[vectorUB]) {
        p = false;
        exitg1 = true;
      } else {
        vectorUB++;
      }
    }
    if (!p) {
      emlrtErrorWithMessageIdR2018a(&c_st, &sb_emlrtRTEI,
                                    "MATLAB:sub2ind:SubscriptVectorSize",
                                    "MATLAB:sub2ind:SubscriptVectorSize", 0);
    }
    vectorUB = 0;
    exitg1 = false;
    while ((!exitg1) && (vectorUB <= currentIteration - 1)) {
      if (varargin_2_data[vectorUB] > (uint32_T)siz_idx_1) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
                                      "MATLAB:sub2ind:IndexOutOfRange",
                                      "MATLAB:sub2ind:IndexOutOfRange", 0);
      } else {
        vectorUB++;
      }
    }
    for (b_k = 0; b_k < currentIteration; b_k++) {
      vectorUB = (int32_T)assignments_data[b_i].f1.data[b_k] +
                 siz * ((int32_T)varargin_2_data[b_k] - 1);
      if ((vectorUB < 1) || (vectorUB > nx)) {
        emlrtDynamicBoundsCheckR2012b(vectorUB, 1, nx, &le_emlrtBCI, &st);
      }
      tmp_data[b_k] = vectorUB;
    }
    for (b_k = 0; b_k < currentIteration; b_k++) {
      r2[tmp_data[b_k] - 1] = true;
    }
    b_FJE[0] = b_loop_ub;
    b_FJE[1] = loop_ub;
    emlrtSubAssignSizeCheckR2012b(&b_FJE[0], 2, &r1->size[0], 2, &p_emlrtECI,
                                  (emlrtCTX)sp);
    for (b_k = 0; b_k < loop_ub; b_k++) {
      for (i = 0; i < b_loop_ub; i++) {
        FJE_data[(i + FJE->size[0] * b_k) + FJE->size[0] * FJE->size[1] * b_i] =
            r2[i + b_loop_ub * b_k];
      }
    }
  }
  emxFree_cell_wrap_75(sp, &unassignedRows);
  emxFree_cell_wrap_74(sp, &assignments);
  emxFree_boolean_T(sp, &r1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (murtyKBestEvents.c) */
