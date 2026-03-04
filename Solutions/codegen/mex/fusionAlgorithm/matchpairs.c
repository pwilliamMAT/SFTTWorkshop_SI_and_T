/*
 * matchpairs.c
 *
 * Code generation for function 'matchpairs'
 *
 */

/* Include files */
#include "matchpairs.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "indexShapeCheck.h"
#include "minPriorityQueue.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo hn_emlrtRSI =
    {
        179,                     /* lineNo */
        "matlabPerfectMatching", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo in_emlrtRSI =
    {
        181,                     /* lineNo */
        "matlabPerfectMatching", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo jn_emlrtRSI =
    {
        230,                     /* lineNo */
        "matlabPerfectMatching", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo kn_emlrtRSI = {
    17,    /* lineNo */
    "min", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\min.m" /* pathName
                                                                        */
};

static emlrtRSInfo ln_emlrtRSI =
    {
        69,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo mn_emlrtRSI =
    {
        119,       /* lineNo */
        "minimum", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo nn_emlrtRSI = {
    288,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo on_emlrtRSI = {
    381,                     /* lineNo */
    "unaryMinOrMaxDispatch", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo pn_emlrtRSI = {
    455,          /* lineNo */
    "minOrMax2D", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo qn_emlrtRSI = {
    562,                         /* lineNo */
    "minOrMax2DColumnMajorDim2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo rn_emlrtRSI = {
    561,                         /* lineNo */
    "minOrMax2DColumnMajorDim2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo sn_emlrtRSI = {
    558,                         /* lineNo */
    "minOrMax2DColumnMajorDim2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo tn_emlrtRSI =
    {
        288,                     /* lineNo */
        "augmentedShortestPath", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo un_emlrtRSI =
    {
        305,                     /* lineNo */
        "augmentedShortestPath", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo vn_emlrtRSI =
    {
        308,                     /* lineNo */
        "augmentedShortestPath", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo wn_emlrtRSI =
    {
        323,                     /* lineNo */
        "augmentedShortestPath", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo xn_emlrtRSI =
    {
        373,                     /* lineNo */
        "augmentedShortestPath", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo yn_emlrtRSI =
    {
        379,                     /* lineNo */
        "augmentedShortestPath", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo ao_emlrtRSI = {
    46,                      /* lineNo */
    "minPriorityQueue/push", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtRSInfo do_emlrtRSI = {
    69,                        /* lineNo */
    "minPriorityQueue/update", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtRSInfo eo_emlrtRSI = {
    56,                     /* lineNo */
    "minPriorityQueue/pop", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtRSInfo fo_emlrtRSI = {
    99,                          /* lineNo */
    "minPriorityQueue/percDown", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtRSInfo go_emlrtRSI = {
    105,                         /* lineNo */
    "minPriorityQueue/percDown", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtRSInfo ho_emlrtRSI = {
    107,                         /* lineNo */
    "minPriorityQueue/percDown", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtBCInfo lc_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    68,                        /* lineNo */
    35,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/update", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtRTEInfo ib_emlrtRTEI = {
    198,             /* lineNo */
    27,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtBCInfo mc_emlrtBCI = {
    -1,                     /* iFirst */
    -1,                     /* iLast */
    53,                     /* lineNo */
    38,                     /* colNo */
    "",                     /* aName */
    "minPriorityQueue/pop", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo nc_emlrtBCI = {
    -1,                     /* iFirst */
    -1,                     /* iLast */
    54,                     /* lineNo */
    29,                     /* colNo */
    "",                     /* aName */
    "minPriorityQueue/pop", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo oc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    44,                      /* lineNo */
    22,                      /* colNo */
    "",                      /* aName */
    "minPriorityQueue/push", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo pc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    45,                      /* lineNo */
    29,                      /* colNo */
    "",                      /* aName */
    "minPriorityQueue/push", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo uc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    106,                         /* lineNo */
    53,                          /* colNo */
    "",                          /* aName */
    "minPriorityQueue/percDown", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo vc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    106,                         /* lineNo */
    30,                          /* colNo */
    "",                          /* aName */
    "minPriorityQueue/percDown", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo wc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    107,                         /* lineNo */
    86,                          /* colNo */
    "",                          /* aName */
    "minPriorityQueue/percDown", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo xc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    107,                         /* lineNo */
    46,                          /* colNo */
    "",                          /* aName */
    "minPriorityQueue/percDown", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo yc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    107,                         /* lineNo */
    21,                          /* colNo */
    "",                          /* aName */
    "minPriorityQueue/percDown", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo ad_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    107,                         /* lineNo */
    77,                          /* colNo */
    "",                          /* aName */
    "minPriorityQueue/percDown", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtRTEInfo ke_emlrtRTEI =
    {
        167,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo le_emlrtRTEI =
    {
        168,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo me_emlrtRTEI =
    {
        169,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ne_emlrtRTEI = {
    550,             /* lineNo */
    21,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtRTEInfo oe_emlrtRTEI = {
    552,             /* lineNo */
    5,               /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtRTEInfo pe_emlrtRTEI = {
    18,    /* lineNo */
    5,     /* colNo */
    "min", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\min.m" /* pName
                                                                        */
};

static emlrtRTEInfo qe_emlrtRTEI =
    {
        188,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo re_emlrtRTEI = {
    26,                 /* lineNo */
    39,                 /* colNo */
    "minPriorityQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pName */
};

static emlrtRTEInfo se_emlrtRTEI = {
    27,                 /* lineNo */
    46,                 /* colNo */
    "minPriorityQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pName */
};

static emlrtRTEInfo te_emlrtRTEI =
    {
        233,          /* lineNo */
        61,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ue_emlrtRTEI =
    {
        254,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ve_emlrtRTEI =
    {
        258,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo we_emlrtRTEI =
    {
        263,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo xe_emlrtRTEI =
    {
        267,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ye_emlrtRTEI =
    {
        218,          /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo af_emlrtRTEI =
    {
        233,          /* lineNo */
        9,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo bf_emlrtRTEI =
    {
        233,          /* lineNo */
        25,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo cf_emlrtRTEI =
    {
        233,          /* lineNo */
        40,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo df_emlrtRTEI =
    {
        233,          /* lineNo */
        52,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ef_emlrtRTEI =
    {
        162,          /* lineNo */
        66,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

/* Function Definitions */
boolean_T
matlabPerfectMatching(const emlrtStack *sp, const emxArray_real_T *matrixRep,
                      emxArray_int32_T *matchCtoR, emxArray_int32_T *matchRtoC,
                      emxArray_real_T *rowWeight, emxArray_real_T *colWeight)
{
  c_matlab_internal_coder_minPrio queue;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  emxArray_int32_T *idx;
  emxArray_real_T *distancesR;
  emxArray_real_T *minIndices;
  emxArray_uint8_T *colorsR;
  const real_T *matrixRep_data;
  real_T *colWeight_data;
  real_T *distancesR_data;
  real_T *minIndices_data;
  real_T *pairWeightR_data;
  real_T *rowWeight_data;
  int32_T b_i;
  int32_T c;
  int32_T i;
  int32_T n;
  int32_T *idx_data;
  int32_T *matchCtoR_data;
  int32_T *matchRtoC_data;
  int32_T *queue_heap_data;
  int32_T *queue_indexToHeap_data;
  uint8_T *colorsR_data;
  boolean_T success;
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
  matrixRep_data = matrixRep->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  n = matrixRep->size[0];
  c = matchCtoR->size[0];
  matchCtoR->size[0] = matrixRep->size[0];
  emxEnsureCapacity_int32_T(sp, matchCtoR, c, &ke_emlrtRTEI);
  matchCtoR_data = matchCtoR->data;
  c = matchRtoC->size[0];
  matchRtoC->size[0] = matrixRep->size[0];
  emxEnsureCapacity_int32_T(sp, matchRtoC, c, &le_emlrtRTEI);
  matchRtoC_data = matchRtoC->data;
  c = colWeight->size[0];
  colWeight->size[0] = matrixRep->size[0];
  emxEnsureCapacity_real_T(sp, colWeight, c, &me_emlrtRTEI);
  colWeight_data = colWeight->data;
  for (i = 0; i < n; i++) {
    matchCtoR_data[i] = 0;
    matchRtoC_data[i] = 0;
    colWeight_data[i] = rtInf;
  }
  emxInit_real_T(sp, &colWeight, 1, &qe_emlrtRTEI);
  emxInit_int32_T(sp, &matchCtoR, 1, &ye_emlrtRTEI);
  emxInit_int32_T(sp, &matchRtoC, 1, &ye_emlrtRTEI);
  emxInit_real_T(sp, &minIndices, 1, &af_emlrtRTEI);
  emxInit_int32_T(sp, &idx, 1, &bf_emlrtRTEI);
  emxInit_real_T(sp, &distancesR, 1, &cf_emlrtRTEI);
  emxInit_uint8_T(sp, &colorsR, 1, &df_emlrtRTEI);
  c_emxInitStruct_matlab_internal(sp, &queue, &ef_emlrtRTEI);
  if (n == 0) {
    rowWeight->size[0] = 0;
    success = true;
  } else {
    real_T edge_weight_shifted;
    int32_T b_n;
    st.site = &hn_emlrtRSI;
    b_st.site = &kn_emlrtRSI;
    c_st.site = &ln_emlrtRSI;
    d_st.site = &mn_emlrtRSI;
    if (matrixRep->size[1] < 1) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &ib_emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }
    e_st.site = &nn_emlrtRSI;
    f_st.site = &on_emlrtRSI;
    g_st.site = &pn_emlrtRSI;
    b_n = matrixRep->size[1];
    c = rowWeight->size[0];
    rowWeight->size[0] = n;
    emxEnsureCapacity_real_T(&g_st, rowWeight, c, &ne_emlrtRTEI);
    rowWeight_data = rowWeight->data;
    c = idx->size[0];
    idx->size[0] = n;
    emxEnsureCapacity_int32_T(&g_st, idx, c, &oe_emlrtRTEI);
    idx_data = idx->data;
    for (i = 0; i < n; i++) {
      idx_data[i] = 1;
    }
    if (matrixRep->size[0] >= 1) {
      h_st.site = &sn_emlrtRSI;
      if (matrixRep->size[0] > 2147483646) {
        i_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (i = 0; i < n; i++) {
        rowWeight_data[i] = matrixRep_data[i];
      }
      h_st.site = &rn_emlrtRSI;
      if (matrixRep->size[1] > 2147483646) {
        i_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (i = 2; i <= b_n; i++) {
        h_st.site = &qn_emlrtRSI;
        if (n > 2147483646) {
          i_st.site = &tb_emlrtRSI;
          check_forloop_overflow_error(&i_st);
        }
        for (b_i = 0; b_i < n; b_i++) {
          edge_weight_shifted =
              matrixRep_data[b_i + matrixRep->size[0] * (i - 1)];
          if (muDoubleScalarIsNaN(edge_weight_shifted)) {
            success = false;
          } else if (muDoubleScalarIsNaN(rowWeight_data[b_i])) {
            success = true;
          } else {
            success = (rowWeight_data[b_i] > edge_weight_shifted);
          }
          if (success) {
            rowWeight_data[b_i] = edge_weight_shifted;
            idx_data[b_i] = i;
          }
        }
      }
    }
    c = minIndices->size[0];
    minIndices->size[0] = n;
    emxEnsureCapacity_real_T(&st, minIndices, c, &pe_emlrtRTEI);
    minIndices_data = minIndices->data;
    for (i = 0; i < n; i++) {
      minIndices_data[i] = idx_data[i];
    }
    st.site = &in_emlrtRSI;
    b_st.site = &ed_emlrtRSI;
    success = true;
    c_st.site = &fd_emlrtRSI;
    if (rowWeight->size[0] > 2147483646) {
      d_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    for (i = 0; i < n; i++) {
      if ((!success) || (muDoubleScalarIsInf(rowWeight_data[i]) ||
                         muDoubleScalarIsNaN(rowWeight_data[i]))) {
        success = false;
      }
    }
    if (!success) {
      success = false;
    } else {
      int32_T colStart;
      c = colWeight->size[0];
      colWeight->size[0] = n;
      emxEnsureCapacity_real_T(sp, colWeight, c, &qe_emlrtRTEI);
      pairWeightR_data = colWeight->data;
      for (b_i = 0; b_i < n; b_i++) {
        pairWeightR_data[b_i] = 0.0;
        c = (int32_T)minIndices_data[b_i];
        if (matchCtoR_data[c - 1] == 0) {
          matchRtoC_data[b_i] = c;
          matchCtoR_data[c - 1] = b_i + 1;
          pairWeightR_data[b_i] = rowWeight_data[b_i];
        }
        for (i = 0; i < n; i++) {
          edge_weight_shifted =
              matrixRep_data[i + matrixRep->size[0] * b_i] - rowWeight_data[i];
          if (edge_weight_shifted < colWeight_data[b_i]) {
            colWeight_data[b_i] = edge_weight_shifted;
          }
        }
      }
      c = matchCtoR->size[0];
      matchCtoR->size[0] = n;
      emxEnsureCapacity_int32_T(sp, matchCtoR, c, &re_emlrtRTEI);
      queue_heap_data = matchCtoR->data;
      c = matchRtoC->size[0];
      matchRtoC->size[0] = n;
      emxEnsureCapacity_int32_T(sp, matchRtoC, c, &se_emlrtRTEI);
      queue_indexToHeap_data = matchRtoC->data;
      colStart = 0;
      int32_T exitg1;
      do {
        exitg1 = 0;
        if (colStart <= n - 1) {
          if (matchCtoR_data[colStart] != 0) {
            colStart++;
          } else {
            real_T last_weight_sap;
            real_T lsap;
            int32_T c_n;
            int32_T clast;
            int32_T exitg2;
            int32_T rlast;
            boolean_T guard1;
            st.site = &jn_emlrtRSI;
            c = queue.heap->size[0];
            queue.heap->size[0] = n;
            emxEnsureCapacity_int32_T(&st, queue.heap, c, &te_emlrtRTEI);
            c = queue.indexToHeap->size[0];
            queue.indexToHeap->size[0] = n;
            emxEnsureCapacity_int32_T(&st, queue.indexToHeap, c, &te_emlrtRTEI);
            for (i = 0; i < n; i++) {
              queue.heap->data[i] = queue_heap_data[i];
              queue.indexToHeap->data[i] = queue_indexToHeap_data[i];
            }
            c_n = matrixRep->size[0] - 1;
            c = minIndices->size[0];
            minIndices->size[0] = n;
            emxEnsureCapacity_real_T(&st, minIndices, c, &ue_emlrtRTEI);
            minIndices_data = minIndices->data;
            c = idx->size[0];
            idx->size[0] = n;
            emxEnsureCapacity_int32_T(&st, idx, c, &ve_emlrtRTEI);
            idx_data = idx->data;
            for (i = 0; i < n; i++) {
              minIndices_data[i] = 0.0;
              idx_data[i] = 0;
            }
            idx_data[colStart] = 0;
            c = distancesR->size[0];
            distancesR->size[0] = n;
            emxEnsureCapacity_real_T(&st, distancesR, c, &we_emlrtRTEI);
            distancesR_data = distancesR->data;
            c = colorsR->size[0];
            colorsR->size[0] = n;
            emxEnsureCapacity_uint8_T(&st, colorsR, c, &xe_emlrtRTEI);
            colorsR_data = colorsR->data;
            for (i = 0; i < n; i++) {
              distancesR_data[i] = rtInf;
              colorsR_data[i] = 0U;
            }
            queue.len = 0;
            edge_weight_shifted = 0.0;
            lsap = rtInf;
            rlast = -1;
            clast = -1;
            last_weight_sap = rtInf;
            c = colStart;
            guard1 = false;
            int32_T exitg3;
            do {
              real_T edge_weight;
              exitg3 = 0;
              b_st.site = &tn_emlrtRSI;
              b_n = 0;
              do {
                exitg2 = 0;
                if (b_n <= c_n) {
                  if (colorsR_data[b_n] == 2) {
                    b_n++;
                  } else {
                    real_T dnew;
                    edge_weight = matrixRep_data[b_n + matrixRep->size[0] * c];
                    dnew = edge_weight_shifted +
                           ((edge_weight - rowWeight_data[b_n]) -
                            colWeight_data[c]);
                    if (dnew < lsap) {
                      if (matchRtoC_data[b_n] == 0) {
                        lsap = dnew;
                        rlast = b_n;
                        clast = c;
                        last_weight_sap = edge_weight;
                      } else if (dnew < distancesR_data[b_n]) {
                        distancesR_data[b_n] = dnew;
                        idx_data[matchRtoC_data[b_n] - 1] = c + 1;
                        minIndices_data[b_n] = edge_weight;
                        if (colorsR_data[b_n] == 0) {
                          b_st.site = &un_emlrtRSI;
                          queue.len++;
                          if ((queue.len < 1) ||
                              (queue.len > queue.heap->size[0])) {
                            emlrtDynamicBoundsCheckR2012b(queue.len, 1,
                                                          queue.heap->size[0],
                                                          &oc_emlrtBCI, &b_st);
                          }
                          queue.heap->data[queue.len - 1] = b_n + 1;
                          if (b_n + 1 > queue.indexToHeap->size[0]) {
                            emlrtDynamicBoundsCheckR2012b(
                                b_n + 1, 1, queue.indexToHeap->size[0],
                                &pc_emlrtBCI, &b_st);
                          }
                          queue.indexToHeap->data[b_n] = queue.len;
                          c_st.site = &ao_emlrtRSI;
                          minPriorityQueue_percUp(&c_st, &queue, queue.len,
                                                  distancesR);
                          colorsR_data[b_n] = 1U;
                        } else {
                          b_st.site = &vn_emlrtRSI;
                          if (b_n + 1 > queue.indexToHeap->size[0]) {
                            emlrtDynamicBoundsCheckR2012b(
                                b_n + 1, 1, queue.indexToHeap->size[0],
                                &lc_emlrtBCI, &b_st);
                          }
                          c_st.site = &do_emlrtRSI;
                          minPriorityQueue_percUp(&c_st, &queue,
                                                  queue.indexToHeap->data[b_n],
                                                  distancesR);
                        }
                      }
                      b_n++;
                    } else if (((dnew >= -1.7976931348623157E+308) &&
                                (dnew <= 1.7976931348623157E+308)) ||
                               (!(edge_weight >= -1.7976931348623157E+308)) ||
                               (!(edge_weight <= 1.7976931348623157E+308))) {
                      b_n++;
                    } else {
                      success = false;
                      exitg2 = 2;
                    }
                  }
                } else {
                  exitg2 = 1;
                }
              } while (exitg2 == 0);
              if (exitg2 == 1) {
                if (queue.len == 0) {
                  guard1 = true;
                  exitg3 = 1;
                } else {
                  int32_T c_i;
                  int32_T d_i;
                  int32_T rCandidate;
                  b_st.site = &wn_emlrtRSI;
                  rCandidate = queue.heap->data[0] - 1;
                  if ((queue.len < 1) || (queue.len > queue.heap->size[0])) {
                    emlrtDynamicBoundsCheckR2012b(
                        queue.len, 1, queue.heap->size[0], &mc_emlrtBCI, &b_st);
                  }
                  c_i = queue.len - 1;
                  queue.heap->data[0] = queue.heap->data[queue.len - 1];
                  if ((queue.heap->data[0] < 1) ||
                      (queue.heap->data[0] > queue.indexToHeap->size[0])) {
                    emlrtDynamicBoundsCheckR2012b(queue.heap->data[0], 1,
                                                  queue.indexToHeap->size[0],
                                                  &nc_emlrtBCI, &b_st);
                  }
                  queue.indexToHeap->data[queue.heap->data[0] - 1] = 1;
                  queue.len--;
                  c_st.site = &eo_emlrtRSI;
                  d_i = 1;
                  int32_T exitg4;
                  int32_T ichild;
                  do {
                    exitg4 = 0;
                    ichild = d_i << 1;
                    if (ichild <= c_i) {
                      if (ichild + 1 <= c_i) {
                        d_st.site = &fo_emlrtRSI;
                        if ((ichild < 1) || (ichild > queue.heap->size[0])) {
                          emlrtDynamicBoundsCheckR2012b(ichild, 1,
                                                        queue.heap->size[0],
                                                        &kc_emlrtBCI, &d_st);
                        }
                        if (ichild + 1 > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(ichild + 1, 1,
                                                        queue.heap->size[0],
                                                        &jc_emlrtBCI, &d_st);
                        }
                        c = queue.heap->data[ichild - 1];
                        if ((c < 1) || (c > n)) {
                          emlrtDynamicBoundsCheckR2012b(c, 1, n, &qc_emlrtBCI,
                                                        &d_st);
                        }
                        if ((queue.heap->data[ichild] < 1) ||
                            (queue.heap->data[ichild] > n)) {
                          emlrtDynamicBoundsCheckR2012b(
                              queue.heap->data[ichild], 1, n, &rc_emlrtBCI,
                              &d_st);
                        }
                        edge_weight_shifted = distancesR_data[c - 1];
                        edge_weight =
                            distancesR_data[queue.heap->data[ichild] - 1];
                        if (edge_weight_shifted < edge_weight) {
                          success = true;
                        } else {
                          if (c > n) {
                            emlrtDynamicBoundsCheckR2012b(c, 1, n, &sc_emlrtBCI,
                                                          &d_st);
                          }
                          if ((queue.heap->data[ichild] < 1) ||
                              (queue.heap->data[ichild] > n)) {
                            emlrtDynamicBoundsCheckR2012b(
                                queue.heap->data[ichild], 1, n, &tc_emlrtBCI,
                                &d_st);
                          }
                          if ((edge_weight_shifted == edge_weight) &&
                              (c <= queue.heap->data[ichild])) {
                            success = true;
                          } else {
                            success = false;
                          }
                        }
                        if (!success) {
                          ichild++;
                        }
                      }
                      d_st.site = &go_emlrtRSI;
                      if (d_i > queue.heap->size[0]) {
                        emlrtDynamicBoundsCheckR2012b(
                            d_i, 1, queue.heap->size[0], &kc_emlrtBCI, &d_st);
                      }
                      if ((ichild < 1) || (ichild > queue.heap->size[0])) {
                        emlrtDynamicBoundsCheckR2012b(ichild, 1,
                                                      queue.heap->size[0],
                                                      &jc_emlrtBCI, &d_st);
                      }
                      c = queue.heap->data[d_i - 1];
                      if ((c < 1) || (c > n)) {
                        emlrtDynamicBoundsCheckR2012b(c, 1, n, &qc_emlrtBCI,
                                                      &d_st);
                      }
                      b_n = queue.heap->data[ichild - 1];
                      if ((b_n < 1) || (b_n > n)) {
                        emlrtDynamicBoundsCheckR2012b(b_n, 1, n, &rc_emlrtBCI,
                                                      &d_st);
                      }
                      edge_weight_shifted = distancesR_data[c - 1];
                      edge_weight = distancesR_data[b_n - 1];
                      if (edge_weight_shifted < edge_weight) {
                        success = true;
                      } else {
                        if (c > n) {
                          emlrtDynamicBoundsCheckR2012b(c, 1, n, &sc_emlrtBCI,
                                                        &d_st);
                        }
                        if (b_n > n) {
                          emlrtDynamicBoundsCheckR2012b(b_n, 1, n, &tc_emlrtBCI,
                                                        &d_st);
                        }
                        if ((edge_weight_shifted == edge_weight) &&
                            (c <= b_n)) {
                          success = true;
                        } else {
                          success = false;
                        }
                      }
                      if (!success) {
                        if (ichild > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(ichild, 1,
                                                        queue.heap->size[0],
                                                        &uc_emlrtBCI, &c_st);
                        }
                        if (d_i > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(
                              d_i, 1, queue.heap->size[0], &uc_emlrtBCI, &c_st);
                        }
                        if (ichild > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(ichild, 1,
                                                        queue.heap->size[0],
                                                        &vc_emlrtBCI, &c_st);
                        }
                        c = queue.heap->data[d_i - 1];
                        queue.heap->data[d_i - 1] =
                            queue.heap->data[ichild - 1];
                        queue.heap->data[ichild - 1] = c;
                        d_st.site = &ho_emlrtRSI;
                        b_indexShapeCheck(&d_st, queue.heap->size[0]);
                        if (ichild > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(ichild, 1,
                                                        queue.heap->size[0],
                                                        &wc_emlrtBCI, &c_st);
                        }
                        if (d_i > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(
                              d_i, 1, queue.heap->size[0], &wc_emlrtBCI, &c_st);
                        }
                        if (ichild > queue.heap->size[0]) {
                          emlrtDynamicBoundsCheckR2012b(ichild, 1,
                                                        queue.heap->size[0],
                                                        &xc_emlrtBCI, &c_st);
                        }
                        if ((queue.heap->data[ichild - 1] < 1) ||
                            (queue.heap->data[ichild - 1] >
                             queue.indexToHeap->size[0])) {
                          emlrtDynamicBoundsCheckR2012b(
                              queue.heap->data[ichild - 1], 1,
                              queue.indexToHeap->size[0], &ad_emlrtBCI, &c_st);
                        }
                        if ((queue.heap->data[d_i - 1] < 1) ||
                            (queue.heap->data[d_i - 1] >
                             queue.indexToHeap->size[0])) {
                          emlrtDynamicBoundsCheckR2012b(
                              queue.heap->data[d_i - 1], 1,
                              queue.indexToHeap->size[0], &ad_emlrtBCI, &c_st);
                        }
                        c = queue.indexToHeap
                                ->data[queue.heap->data[d_i - 1] - 1];
                        if ((queue.heap->data[d_i - 1] < 1) ||
                            (queue.heap->data[d_i - 1] >
                             queue.indexToHeap->size[0])) {
                          emlrtDynamicBoundsCheckR2012b(
                              queue.heap->data[d_i - 1], 1,
                              queue.indexToHeap->size[0], &yc_emlrtBCI, &c_st);
                        }
                        queue.indexToHeap->data[queue.heap->data[d_i - 1] - 1] =
                            queue.indexToHeap
                                ->data[queue.heap->data[ichild - 1] - 1];
                        if ((queue.heap->data[ichild - 1] < 1) ||
                            (queue.heap->data[ichild - 1] >
                             queue.indexToHeap->size[0])) {
                          emlrtDynamicBoundsCheckR2012b(
                              queue.heap->data[ichild - 1], 1,
                              queue.indexToHeap->size[0], &yc_emlrtBCI, &c_st);
                        }
                        queue.indexToHeap
                            ->data[queue.heap->data[ichild - 1] - 1] = c;
                        d_i = ichild;
                      } else {
                        exitg4 = 1;
                      }
                    } else {
                      exitg4 = 1;
                    }
                  } while (exitg4 == 0);
                  edge_weight_shifted = distancesR_data[rCandidate];
                  if (lsap <= distancesR_data[rCandidate]) {
                    guard1 = true;
                    exitg3 = 1;
                  } else {
                    colorsR_data[rCandidate] = 2U;
                    c = matchRtoC_data[rCandidate] - 1;
                    guard1 = false;
                  }
                }
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);
            if (guard1) {
              success = (lsap < rtInf);
              if (success) {
                c = rlast + 1;
                do {
                  exitg2 = 0;
                  b_n = matchCtoR_data[clast];
                  matchRtoC_data[c - 1] = clast + 1;
                  matchCtoR_data[clast] = c;
                  pairWeightR_data[c - 1] = last_weight_sap;
                  if (idx_data[clast] == 0) {
                    exitg2 = 1;
                  } else {
                    c = b_n;
                    clast = idx_data[clast] - 1;
                    last_weight_sap = minIndices_data[b_n - 1];
                  }
                } while (exitg2 == 0);
                b_st.site = &xn_emlrtRSI;
                for (i = 0; i <= c_n; i++) {
                  if (colorsR_data[i] == 2) {
                    rowWeight_data[i] =
                        (rowWeight_data[i] - lsap) + distancesR_data[i];
                  }
                }
                b_st.site = &yn_emlrtRSI;
                for (i = 0; i <= c_n; i++) {
                  if (matchCtoR_data[i] != 0) {
                    colWeight_data[i] =
                        pairWeightR_data[matchCtoR_data[i] - 1] -
                        rowWeight_data[matchCtoR_data[i] - 1];
                  }
                }
              }
            }
            if (!success) {
              success = false;
              exitg1 = 1;
            } else {
              colStart++;
            }
          }
        } else {
          success = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
  }
  c_emxFreeStruct_matlab_internal(sp, &queue);
  emxFree_uint8_T(sp, &colorsR);
  emxFree_real_T(sp, &distancesR);
  emxFree_int32_T(sp, &idx);
  emxFree_real_T(sp, &minIndices);
  emxFree_int32_T(sp, &matchRtoC);
  emxFree_int32_T(sp, &matchCtoR);
  emxFree_real_T(sp, &colWeight);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return success;
}

/* End of code generation (matchpairs.c) */
