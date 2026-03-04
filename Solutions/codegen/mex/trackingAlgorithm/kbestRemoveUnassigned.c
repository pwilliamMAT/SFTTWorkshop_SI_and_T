/*
 * kbestRemoveUnassigned.c
 *
 * Code generation for function 'kbestRemoveUnassigned'
 *
 */

/* Include files */
#include "kbestRemoveUnassigned.h"
#include "colon.h"
#include "eml_int_forloop_overflow_check.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "omp.h"

/* Variable Definitions */
static emlrtRSInfo jjb_emlrtRSI = {
    30,                      /* lineNo */
    "kbestRemoveUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pathName */
};

static emlrtRSInfo kjb_emlrtRSI = {
    28,                      /* lineNo */
    "kbestRemoveUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pathName */
};

static emlrtRSInfo ljb_emlrtRSI = {
    27,                      /* lineNo */
    "kbestRemoveUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pathName */
};

static emlrtRSInfo mjb_emlrtRSI = {
    23,                      /* lineNo */
    "kbestRemoveUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pathName */
};

static emlrtRSInfo njb_emlrtRSI = {
    21,                      /* lineNo */
    "kbestRemoveUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pathName */
};

static emlrtRSInfo ojb_emlrtRSI = {
    27,                                                 /* lineNo */
    "sortrows",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sortrows.m" /* pathName */
};

static emlrtRSInfo pjb_emlrtRSI = {
    28,                                                 /* lineNo */
    "sortrows",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sortrows.m" /* pathName */
};

static emlrtRSInfo qjb_emlrtRSI = {
    86,                                                  /* lineNo */
    "sortIdx",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sortIdx.m" /* pathName */
};

static emlrtRSInfo rjb_emlrtRSI = {
    57,                                                    /* lineNo */
    "mergesort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/mergesort.m" /* pathName */
};

static emlrtRSInfo sjb_emlrtRSI = {
    113,                                                   /* lineNo */
    "mergesort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/mergesort.m" /* pathName */
};

static emlrtRSInfo tjb_emlrtRSI = {
    39,                                                 /* lineNo */
    "apply_row_permutation",                            /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sortrows.m" /* pathName */
};

static emlrtRSInfo ujb_emlrtRSI = {
    42,                                                 /* lineNo */
    "apply_row_permutation",                            /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sortrows.m" /* pathName */
};

static emlrtBCInfo kj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    23,                      /* lineNo */
    21,                      /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo lj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    30,                      /* lineNo */
    25,                      /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo mj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    36,                      /* lineNo */
    27,                      /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo nj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    24,                      /* lineNo */
    1,                       /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo oj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    29,                      /* lineNo */
    26,                      /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo pj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    31,                      /* lineNo */
    5,                       /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo qj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    33,                      /* lineNo */
    5,                       /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo rj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    34,                      /* lineNo */
    5,                       /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo sj_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    37,                      /* lineNo */
    25,                      /* colNo */
    "",                      /* aName */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m", /* pName */
    0                          /* checkKind */
};

static emlrtRTEInfo ai_emlrtRTEI = {
    27,                                                 /* lineNo */
    1,                                                  /* colNo */
    "sortrows",                                         /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sortrows.m" /* pName */
};

static emlrtRTEInfo bi_emlrtRTEI = {
    52,                                                    /* lineNo */
    9,                                                     /* colNo */
    "mergesort",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/mergesort.m" /* pName */
};

static emlrtRTEInfo yk_emlrtRTEI = {
    23,                      /* lineNo */
    1,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo al_emlrtRTEI = {
    27,                      /* lineNo */
    5,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo bl_emlrtRTEI = {
    29,                      /* lineNo */
    5,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo cl_emlrtRTEI = {
    30,                      /* lineNo */
    5,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo dl_emlrtRTEI = {
    34,                      /* lineNo */
    5,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo el_emlrtRTEI = {
    36,                      /* lineNo */
    1,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo fl_emlrtRTEI = {
    37,                      /* lineNo */
    14,                      /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo gl_emlrtRTEI = {
    37,                      /* lineNo */
    1,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo hl_emlrtRTEI = {
    22,                      /* lineNo */
    1,                       /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo il_emlrtRTEI = {
    21,                      /* lineNo */
    17,                      /* colNo */
    "kbestRemoveUnassigned", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "kbestRemoveUnassigned.m" /* pName */
};

static emlrtRTEInfo jl_emlrtRTEI = {
    52,                                                    /* lineNo */
    1,                                                     /* colNo */
    "mergesort",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/mergesort.m" /* pName */
};

/* Function Definitions */
int32_T kbestRemoveUnassigned(const emlrtStack *sp,
                              emxArray_uint32_T *assignment,
                              const int32_T costSize[2],
                              uint32_T unassignedRows_data[],
                              emxArray_uint32_T *unassignedCols)
{
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_int32_T *idx;
  emxArray_int32_T *iwork;
  emxArray_int32_T *y;
  emxArray_uint32_T *b_colSoln;
  emxArray_uint32_T *colSoln;
  emxArray_uint32_T *rowSoln;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T i;
  int32_T i1;
  int32_T kbestRemoveUnassigned_numThreads;
  int32_T loop_ub;
  int32_T n;
  int32_T p;
  int32_T pEnd;
  int32_T q;
  int32_T qEnd;
  int32_T rowIdx_size;
  int32_T unassignedRows_size;
  int32_T *idx_data;
  int32_T *iwork_data;
  uint32_T rowIdx_data[51];
  uint32_T u;
  uint32_T u1;
  uint32_T *assignment_data;
  uint32_T *b_colSoln_data;
  uint32_T *colSoln_data;
  uint32_T *rowSoln_data;
  int8_T tmp_data[51];
  boolean_T isRowAssigned_data[51];
  boolean_T b_p;
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
  assignment_data = assignment->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_int32_T(sp, &y, 2, &il_emlrtRTEI);
  st.site = &njb_emlrtRSI;
  b_st.site = &jdb_emlrtRSI;
  c_st.site = &kdb_emlrtRSI;
  eml_integer_colon_dispatcher(&c_st, costSize[0], y);
  idx_data = y->data;
  pEnd = y->size[1];
  rowIdx_size = y->size[1];
  for (i = 0; i < pEnd; i++) {
    p = idx_data[i];
    if (p < 0) {
      p = 0;
    }
    rowIdx_data[i] = (uint32_T)p;
  }
  loop_ub = assignment->size[0];
  if ((costSize[0] < 1) || (costSize[0] > loop_ub)) {
    emlrtDynamicBoundsCheckR2012b(costSize[0], 1, loop_ub, &kj_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  b_iv[0] = 1;
  b_iv[1] = costSize[0];
  st.site = &mjb_emlrtRSI;
  b_indexShapeCheck(&st, assignment->size[0], b_iv);
  p = costSize[0];
  emxInit_uint32_T(sp, &rowSoln, 1, &hl_emlrtRTEI, true);
  pEnd = rowSoln->size[0];
  rowSoln->size[0] = costSize[0];
  emxEnsureCapacity_uint32_T(sp, rowSoln, pEnd, &yk_emlrtRTEI);
  rowSoln_data = rowSoln->data;
  for (i = 0; i < p; i++) {
    rowSoln_data[i] = assignment_data[i + assignment->size[0]];
  }
  pEnd = rowSoln->size[0];
  for (i = 0; i < pEnd; i++) {
    if (rowSoln_data[i] > (uint32_T)costSize[1]) {
      if (((int8_T)i < 0) || ((int8_T)i > costSize[0] - 1)) {
        emlrtDynamicBoundsCheckR2012b((int8_T)i, 0, costSize[0] - 1,
                                      &nj_emlrtBCI, (emlrtConstCTX)sp);
      }
      rowSoln_data[(int8_T)i] = 0U;
    }
  }
  b_loop_ub = rowSoln->size[0];
  pEnd = rowSoln->size[0];
  if (rowSoln->size[0] < 1600) {
    for (b_i = 0; b_i < b_loop_ub; b_i++) {
      isRowAssigned_data[b_i] = (rowSoln_data[b_i] != 0U);
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    kbestRemoveUnassigned_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(kbestRemoveUnassigned_numThreads)

    for (b_i = 0; b_i < pEnd; b_i++) {
      isRowAssigned_data[b_i] = (rowSoln_data[b_i] != 0U);
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  st.site = &ljb_emlrtRSI;
  b_st.site = &jdb_emlrtRSI;
  c_st.site = &kdb_emlrtRSI;
  eml_integer_colon_dispatcher(&c_st, costSize[1], y);
  idx_data = y->data;
  p = y->size[1];
  pEnd = rowSoln->size[0];
  rowSoln->size[0] = y->size[1];
  emxEnsureCapacity_uint32_T(sp, rowSoln, pEnd, &al_emlrtRTEI);
  rowSoln_data = rowSoln->data;
  for (i = 0; i < p; i++) {
    pEnd = idx_data[i];
    if (pEnd < 0) {
      pEnd = 0;
    }
    rowSoln_data[i] = (uint32_T)pEnd;
  }
  emxFree_int32_T(sp, &y);
  st.site = &kjb_emlrtRSI;
  b_st.site = &ojb_emlrtRSI;
  n = assignment->size[0] + 1;
  emxInit_int32_T(&b_st, &idx, 1, &ai_emlrtRTEI);
  pEnd = idx->size[0];
  idx->size[0] = loop_ub;
  emxEnsureCapacity_int32_T(&b_st, idx, pEnd, &ai_emlrtRTEI);
  idx_data = idx->data;
  for (i = 0; i < loop_ub; i++) {
    idx_data[i] = 0;
  }
  c_st.site = &qjb_emlrtRSI;
  emxInit_int32_T(&c_st, &iwork, 1, &jl_emlrtRTEI);
  pEnd = iwork->size[0];
  iwork->size[0] = loop_ub;
  emxEnsureCapacity_int32_T(&c_st, iwork, pEnd, &bi_emlrtRTEI);
  iwork_data = iwork->data;
  pEnd = assignment->size[0] - 1;
  d_st.site = &rjb_emlrtRSI;
  if (assignment->size[0] - 1 > 2147483645) {
    e_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }
  for (i = 1; i <= pEnd; i += 2) {
    b_p = true;
    u = assignment_data[(i + assignment->size[0]) - 1];
    u1 = assignment_data[i + assignment->size[0]];
    if (u != u1) {
      b_p = (u <= u1);
    }
    if (b_p) {
      idx_data[i - 1] = i;
      idx_data[i] = i + 1;
    } else {
      idx_data[i - 1] = i + 1;
      idx_data[i] = i;
    }
  }
  if (((uint32_T)assignment->size[0] & 1U) != 0U) {
    idx_data[pEnd] = loop_ub;
  }
  c_i = 2;
  while (c_i < n - 1) {
    int32_T i2;
    int32_T j;
    i2 = c_i << 1;
    j = 1;
    for (pEnd = c_i + 1; pEnd < n; pEnd = qEnd + c_i) {
      int32_T kEnd;
      p = j - 1;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > n) {
        qEnd = n;
      }
      unassignedRows_size = 0;
      kEnd = qEnd - j;
      while (unassignedRows_size < kEnd) {
        b_p = true;
        u = assignment_data[(idx_data[p] + assignment->size[0]) - 1];
        u1 = assignment_data[(idx_data[q] + assignment->size[0]) - 1];
        if (u != u1) {
          b_p = (u <= u1);
        }
        if (b_p) {
          iwork_data[unassignedRows_size] = idx_data[p];
          p++;
          if (p + 1 == pEnd) {
            while (q + 1 < qEnd) {
              unassignedRows_size++;
              iwork_data[unassignedRows_size] = idx_data[q];
              q++;
            }
          }
        } else {
          iwork_data[unassignedRows_size] = idx_data[q];
          q++;
          if (q + 1 == qEnd) {
            while (p + 1 < pEnd) {
              unassignedRows_size++;
              iwork_data[unassignedRows_size] = idx_data[p];
              p++;
            }
          }
        }
        unassignedRows_size++;
      }
      d_st.site = &sjb_emlrtRSI;
      for (i = 0; i < kEnd; i++) {
        idx_data[(j + i) - 1] = iwork_data[i];
      }
      j = qEnd;
    }
    c_i = i2;
  }
  emxFree_int32_T(&c_st, &iwork);
  b_st.site = &pjb_emlrtRSI;
  c_st.site = &tjb_emlrtRSI;
  if (assignment->size[0] > 2147483646) {
    d_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }
  c_st.site = &ujb_emlrtRSI;
  if (assignment->size[0] > 2147483646) {
    d_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }
  emxInit_uint32_T(sp, &colSoln, 2, &bl_emlrtRTEI, true);
  pEnd = colSoln->size[0] * colSoln->size[1];
  colSoln->size[0] = loop_ub;
  colSoln->size[1] = 2;
  emxEnsureCapacity_uint32_T(sp, colSoln, pEnd, &bl_emlrtRTEI);
  colSoln_data = colSoln->data;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      if ((idx_data[i1] < 1) || (idx_data[i1] > loop_ub)) {
        emlrtDynamicBoundsCheckR2012b(idx_data[i1], 1, loop_ub, &oj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      colSoln_data[i1 + colSoln->size[0] * i] =
          assignment_data[(idx_data[i1] + assignment->size[0] * i) - 1];
    }
  }
  pEnd = idx->size[0] << 1;
  emxFree_int32_T(sp, &idx);
  if ((costSize[1] < 1) || (costSize[1] > pEnd)) {
    emlrtDynamicBoundsCheckR2012b(costSize[1], 1, pEnd, &lj_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  st.site = &jjb_emlrtRSI;
  c_indexShapeCheck();
  p = costSize[1];
  emxInit_uint32_T(sp, &b_colSoln, 2, &bl_emlrtRTEI, true);
  pEnd = b_colSoln->size[0] * b_colSoln->size[1];
  b_colSoln->size[0] = 1;
  b_colSoln->size[1] = costSize[1];
  emxEnsureCapacity_uint32_T(sp, b_colSoln, pEnd, &cl_emlrtRTEI);
  b_colSoln_data = b_colSoln->data;
  for (i = 0; i < p; i++) {
    b_colSoln_data[i] = colSoln_data[i];
  }
  pEnd = costSize[1];
  for (i = 0; i < pEnd; i++) {
    if (colSoln_data[i] > (uint32_T)costSize[0]) {
      if (i > costSize[1] - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, costSize[1] - 1, &pj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      b_colSoln_data[i] = 0U;
    }
  }
  unassignedRows_size = 0;
  for (i = 0; i < b_loop_ub; i++) {
    if (!isRowAssigned_data[i]) {
      unassignedRows_size++;
    }
  }
  pEnd = 0;
  for (i = 0; i < b_loop_ub; i++) {
    if (!isRowAssigned_data[i]) {
      if (i > rowIdx_size - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, rowIdx_size - 1, &qj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      unassignedRows_data[pEnd] = rowIdx_data[i];
      pEnd++;
    }
  }
  q = b_colSoln->size[1];
  p = 0;
  for (i = 0; i < q; i++) {
    if (b_colSoln_data[i] == 0U) {
      p++;
    }
  }
  pEnd = unassignedCols->size[0];
  unassignedCols->size[0] = p;
  emxEnsureCapacity_uint32_T(sp, unassignedCols, pEnd, &dl_emlrtRTEI);
  colSoln_data = unassignedCols->data;
  pEnd = 0;
  for (i = 0; i < q; i++) {
    if (b_colSoln_data[i] == 0U) {
      if (i > rowSoln->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, rowSoln->size[0] - 1, &rj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      colSoln_data[pEnd] = rowSoln_data[i];
      pEnd++;
    }
  }
  emxFree_uint32_T(sp, &b_colSoln);
  emxFree_uint32_T(sp, &rowSoln);
  if (costSize[0] > loop_ub) {
    emlrtDynamicBoundsCheckR2012b(costSize[0], 1, loop_ub, &mj_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  pEnd = costSize[0];
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < pEnd; i1++) {
      assignment_data[i1 + pEnd * i] =
          assignment_data[i1 + assignment->size[0] * i];
    }
  }
  pEnd = assignment->size[0] * assignment->size[1];
  assignment->size[0] = costSize[0];
  assignment->size[1] = 2;
  emxEnsureCapacity_uint32_T(sp, assignment, pEnd, &el_emlrtRTEI);
  assignment_data = assignment->data;
  q = 0;
  pEnd = 0;
  for (i = 0; i < b_loop_ub; i++) {
    if (isRowAssigned_data[i]) {
      q++;
      tmp_data[pEnd] = (int8_T)i;
      pEnd++;
    }
  }
  p = assignment->size[0] - 1;
  pEnd = colSoln->size[0] * colSoln->size[1];
  colSoln->size[0] = q;
  colSoln->size[1] = 2;
  emxEnsureCapacity_uint32_T(sp, colSoln, pEnd, &fl_emlrtRTEI);
  colSoln_data = colSoln->data;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < q; i1++) {
      int8_T b_i2;
      b_i2 = tmp_data[i1];
      if (b_i2 > p) {
        emlrtDynamicBoundsCheckR2012b(b_i2, 0, p, &sj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      colSoln_data[i1 + colSoln->size[0] * i] =
          assignment_data[b_i2 + assignment->size[0] * i];
    }
  }
  p = colSoln->size[0];
  pEnd = assignment->size[0] * assignment->size[1];
  assignment->size[0] = colSoln->size[0];
  assignment->size[1] = 2;
  emxEnsureCapacity_uint32_T(sp, assignment, pEnd, &gl_emlrtRTEI);
  assignment_data = assignment->data;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < p; i1++) {
      assignment_data[i1 + assignment->size[0] * i] =
          colSoln_data[i1 + colSoln->size[0] * i];
    }
  }
  emxFree_uint32_T(sp, &colSoln);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return unassignedRows_size;
}

/* End of code generation (kbestRemoveUnassigned.c) */
