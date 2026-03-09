/*
 * AssignerGNN.c
 *
 * Code generation for function 'AssignerGNN'
 *
 */

/* Include files */
#include "AssignerGNN.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "matchpairs.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>

/* Variable Definitions */
static emlrtRSInfo ud_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRSInfo vd_emlrtRSI =
    {
        71,      /* lineNo */
        "power", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pathName
                                                                          */
};

static emlrtRSInfo lm_emlrtRSI = {
    193,                    /* lineNo */
    "AssignerGNN/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pathName */
};

static emlrtRSInfo mm_emlrtRSI = {
    198,                          /* lineNo */
    "AssignerGNN/getAssignments", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pathName */
};

static emlrtRSInfo nm_emlrtRSI = {
    205,                                  /* lineNo */
    "AssignerGNN/solveAssignmentProblem", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pathName */
};

static emlrtRSInfo om_emlrtRSI =
    {
        16,           /* lineNo */
        "matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo pm_emlrtRSI =
    {
        46,           /* lineNo */
        "matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo qm_emlrtRSI =
    {
        67,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo rm_emlrtRSI =
    {
        68,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo sm_emlrtRSI =
    {
        72,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo tm_emlrtRSI =
    {
        73,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo um_emlrtRSI =
    {
        77,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo vm_emlrtRSI =
    {
        80,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo wm_emlrtRSI =
    {
        83,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo xm_emlrtRSI =
    {
        88,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo ym_emlrtRSI =
    {
        90,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo an_emlrtRSI =
    {
        92,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo bn_emlrtRSI =
    {
        99,               /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo cn_emlrtRSI =
    {
        115,              /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo dn_emlrtRSI =
    {
        123,              /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo en_emlrtRSI =
    {
        133,              /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo fn_emlrtRSI =
    {
        138,              /* lineNo */
        "eml_matchpairs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo gn_emlrtRSI =
    {
        153,               /* lineNo */
        "perfectMatching", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRSInfo io_emlrtRSI = {
    44,               /* lineNo */
    "reduceToScalar", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reduceToScalar.m" /* pathName */
};

static emlrtRSInfo jo_emlrtRSI = {
    52,      /* lineNo */
    "floop", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reduceToScalar.m" /* pathName */
};

static emlrtRSInfo ko_emlrtRSI =
    {
        398,                 /* lineNo */
        "findAGreaterThanB", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pathName */
};

static emlrtRTEInfo fb_emlrtRTEI =
    {
        120,              /* lineNo */
        1,                /* colNo */
        "eml_matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo gb_emlrtRTEI =
    {
        19,           /* lineNo */
        23,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo hb_emlrtRTEI =
    {
        16,           /* lineNo */
        23,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo xd_emlrtRTEI =
    {
        65,           /* lineNo */
        1,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo yd_emlrtRTEI =
    {
        121,          /* lineNo */
        28,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ae_emlrtRTEI =
    {
        395,          /* lineNo */
        22,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo be_emlrtRTEI =
    {
        76,                  /* lineNo */
        13,                  /* colNo */
        "eml_mtimes_helper", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo ce_emlrtRTEI =
    {
        133,          /* lineNo */
        5,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo de_emlrtRTEI =
    {
        138,          /* lineNo */
        5,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo ee_emlrtRTEI = {
    206,           /* lineNo */
    13,            /* colNo */
    "AssignerGNN", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pName */
};

static emlrtRTEInfo fe_emlrtRTEI = {
    207,           /* lineNo */
    13,            /* colNo */
    "AssignerGNN", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pName */
};

static emlrtRTEInfo ge_emlrtRTEI = {
    208,           /* lineNo */
    13,            /* colNo */
    "AssignerGNN", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pName */
};

static emlrtRTEInfo he_emlrtRTEI = {
    189,           /* lineNo */
    17,            /* colNo */
    "AssignerGNN", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\AssignerG"
    "NN.m" /* pName */
};

static emlrtRTEInfo ie_emlrtRTEI =
    {
        83,           /* lineNo */
        2,            /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

static emlrtRTEInfo je_emlrtRTEI =
    {
        83,           /* lineNo */
        12,           /* colNo */
        "matchpairs", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\matchpairs."
        "m" /* pName */
};

/* Function Definitions */
void AssignerGNN_stepImpl(const emlrtStack *sp,
                          c_matlabshared_tracking_interna *obj,
                          const emxArray_real_T *costMatrix,
                          emxArray_uint32_T *assignments,
                          emxArray_uint32_T *unassignedRows,
                          emxArray_uint32_T *unassignedColumns)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  emxArray_int32_T *b_unassignedRows;
  emxArray_int32_T *colToRow;
  emxArray_int32_T *matchings;
  emxArray_int32_T *rowToCol;
  emxArray_real_T *a__1;
  emxArray_real_T *a__2;
  emxArray_real_T *b_paddedCost;
  emxArray_real_T *paddedCost;
  const real_T *costMatrix_data;
  real_T costUnmatched;
  real_T *b_paddedCost_data;
  real_T *paddedCost_data;
  int32_T b_nOut;
  int32_T ii;
  int32_T k;
  int32_T m;
  int32_T n;
  int32_T nOut;
  int32_T nr;
  int32_T nx;
  int32_T *colToRow_data;
  int32_T *matchings_data;
  int32_T *rowToCol_data;
  int32_T *unassignedRows_data;
  uint32_T *assignments_data;
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
  costMatrix_data = costMatrix->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  costUnmatched = obj->AssignmentThreshold[0] / 2.0;
  obj->pCostOfNonAssignment = costUnmatched;
  st.site = &lm_emlrtRSI;
  b_st.site = &mm_emlrtRSI;
  c_st.site = &nm_emlrtRSI;
  d_st.site = &om_emlrtRSI;
  e_st.site = &ed_emlrtRSI;
  nx = costMatrix->size[0] * costMatrix->size[1];
  p = false;
  f_st.site = &fd_emlrtRSI;
  if (nx > 2147483646) {
    g_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&g_st);
  }
  for (k = 0; k < nx; k++) {
    if (p || muDoubleScalarIsNaN(costMatrix_data[k])) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(&c_st, &hb_emlrtRTEI,
                                  "MATLAB:matchpairs:NonFiniteCost",
                                  "MATLAB:matchpairs:NonFiniteCost", 0);
  }
  if (muDoubleScalarIsInf(costUnmatched) ||
      muDoubleScalarIsNaN(costUnmatched)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &gb_emlrtRTEI, "MATLAB:matchpairs:NonFiniteCostUnmatched",
        "MATLAB:matchpairs:NonFiniteCostUnmatched", 0);
  }
  d_st.site = &pm_emlrtRSI;
  m = costMatrix->size[0];
  n = costMatrix->size[1];
  nOut = costMatrix->size[0] + costMatrix->size[1];
  emxInit_real_T(&d_st, &paddedCost, 2, &xd_emlrtRTEI);
  nx = paddedCost->size[0] * paddedCost->size[1];
  paddedCost->size[0] = nOut;
  paddedCost->size[1] = nOut;
  emxEnsureCapacity_real_T(&d_st, paddedCost, nx, &xd_emlrtRTEI);
  paddedCost_data = paddedCost->data;
  b_nOut = nOut * nOut;
  for (k = 0; k < b_nOut; k++) {
    paddedCost_data[k] = rtInf;
  }
  e_st.site = &qm_emlrtRSI;
  if (costMatrix->size[1] > 2147483646) {
    f_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&f_st);
  }
  for (k = 0; k < n; k++) {
    e_st.site = &rm_emlrtRSI;
    if (m > 2147483646) {
      f_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }
    for (ii = 0; ii < m; ii++) {
      paddedCost_data[ii + paddedCost->size[0] * k] =
          costMatrix_data[ii + costMatrix->size[0] * k];
    }
  }
  e_st.site = &sm_emlrtRSI;
  if (costMatrix->size[0] > 2147483646) {
    f_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&f_st);
  }
  for (k = 0; k < m; k++) {
    e_st.site = &tm_emlrtRSI;
    if (n > 2147483646) {
      f_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }
    for (ii = 0; ii < n; ii++) {
      paddedCost_data[(m + ii) + paddedCost->size[0] * (n + k)] =
          costMatrix_data[k + costMatrix->size[0] * ii];
    }
  }
  e_st.site = &um_emlrtRSI;
  for (k = 0; k < m; k++) {
    paddedCost_data[k + paddedCost->size[0] * (n + k)] = 2.0 * costUnmatched;
  }
  e_st.site = &vm_emlrtRSI;
  for (k = 0; k < n; k++) {
    paddedCost_data[(m + k) + paddedCost->size[0] * k] = 2.0 * costUnmatched;
  }
  e_st.site = &wm_emlrtRSI;
  emxInit_int32_T(&e_st, &colToRow, 1, &ie_emlrtRTEI);
  emxInit_int32_T(&e_st, &rowToCol, 1, &je_emlrtRTEI);
  emxInit_real_T(&e_st, &a__1, 1, &he_emlrtRTEI);
  emxInit_real_T(&e_st, &a__2, 1, &he_emlrtRTEI);
  f_st.site = &gn_emlrtRSI;
  p = matlabPerfectMatching(&f_st, paddedCost, colToRow, rowToCol, a__1, a__2);
  rowToCol_data = rowToCol->data;
  colToRow_data = colToRow->data;
  if (!p) {
    colToRow->size[0] = 0;
    rowToCol->size[0] = 0;
  }
  emxInit_real_T(&d_st, &b_paddedCost, 2, &be_emlrtRTEI);
  if ((colToRow->size[0] == 0) && (nOut > 0)) {
    real_T absNewVal;
    real_T maxVal;
    e_st.site = &xm_emlrtRSI;
    f_st.site = &io_emlrtRSI;
    maxVal = 0.0;
    g_st.site = &jo_emlrtRSI;
    if (b_nOut > 2147483646) {
      h_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&h_st);
    }
    for (k = 0; k < b_nOut; k++) {
      absNewVal = muDoubleScalarAbs(paddedCost_data[k]);
      if ((absNewVal > maxVal) && (!muDoubleScalarIsInf(absNewVal))) {
        maxVal = absNewVal;
      }
    }
    if (!muDoubleScalarIsInf(maxVal)) {
      absNewVal = frexp(maxVal, &nr);
      maxVal = nr;
      if (absNewVal == 0.5) {
        maxVal = (real_T)nr - 1.0;
      }
    }
    e_st.site = &ym_emlrtRSI;
    f_st.site = &ud_emlrtRSI;
    g_st.site = &vd_emlrtRSI;
    absNewVal = muDoubleScalarPower(2.0, -maxVal);
    e_st.site = &an_emlrtRSI;
    nx = b_paddedCost->size[0] * b_paddedCost->size[1];
    b_paddedCost->size[0] = nOut;
    b_paddedCost->size[1] = nOut;
    emxEnsureCapacity_real_T(&e_st, b_paddedCost, nx, &be_emlrtRTEI);
    b_paddedCost_data = b_paddedCost->data;
    nx = (b_nOut / 2) << 1;
    nr = nx - 2;
    for (k = 0; k <= nr; k += 2) {
      __m128d r;
      r = _mm_loadu_pd(&paddedCost_data[k]);
      _mm_storeu_pd(&b_paddedCost_data[k],
                    _mm_mul_pd(r, _mm_set1_pd(absNewVal)));
    }
    for (k = nx; k < b_nOut; k++) {
      b_paddedCost_data[k] = paddedCost_data[k] * absNewVal;
    }
    f_st.site = &gn_emlrtRSI;
    p = matlabPerfectMatching(&f_st, b_paddedCost, colToRow, rowToCol, a__1,
                              a__2);
    rowToCol_data = rowToCol->data;
    colToRow_data = colToRow->data;
    if (!p) {
      colToRow->size[0] = 0;
      rowToCol->size[0] = 0;
    }
  }
  emxFree_real_T(&d_st, &b_paddedCost);
  emxFree_real_T(&d_st, &a__2);
  emxFree_real_T(&d_st, &a__1);
  emxFree_real_T(&d_st, &paddedCost);
  e_st.site = &bn_emlrtRSI;
  for (k = 0; k < n; k++) {
    nx = colToRow_data[k];
    if ((colToRow_data[k] <= m) &&
        (costMatrix_data[(colToRow_data[k] + costMatrix->size[0] * k) - 1] ==
         2.0 * costUnmatched)) {
      colToRow_data[k] = nOut + 1;
      rowToCol_data[nx - 1] = nOut + 1;
    }
  }
  nr = 0;
  e_st.site = &cn_emlrtRSI;
  for (k = 0; k < n; k++) {
    if (colToRow_data[k] <= m) {
      nr++;
    }
  }
  if (nr > costMatrix->size[1]) {
    emlrtErrorWithMessageIdR2018a(&d_st, &fb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  emxInit_int32_T(&d_st, &matchings, 2, &he_emlrtRTEI);
  nx = matchings->size[0] * matchings->size[1];
  matchings->size[0] = nr;
  matchings->size[1] = 2;
  emxEnsureCapacity_int32_T(&d_st, matchings, nx, &yd_emlrtRTEI);
  matchings_data = matchings->data;
  nx = -1;
  e_st.site = &dn_emlrtRSI;
  for (k = 0; k < n; k++) {
    if (colToRow_data[k] <= m) {
      nx++;
      matchings_data[nx] = colToRow_data[k];
      matchings_data[nx + matchings->size[0]] = k + 1;
    }
  }
  e_st.site = &en_emlrtRSI;
  emxInit_int32_T(&e_st, &b_unassignedRows, 1, &he_emlrtRTEI);
  nx = b_unassignedRows->size[0];
  b_unassignedRows->size[0] = costMatrix->size[0];
  emxEnsureCapacity_int32_T(&e_st, b_unassignedRows, nx, &ae_emlrtRTEI);
  unassignedRows_data = b_unassignedRows->data;
  nOut = 0;
  f_st.site = &ko_emlrtRSI;
  for (k = 0; k < m; k++) {
    if (rowToCol_data[k] > n) {
      nOut++;
      unassignedRows_data[nOut - 1] = k + 1;
    }
  }
  nx = b_unassignedRows->size[0];
  b_unassignedRows->size[0] = nOut;
  emxEnsureCapacity_int32_T(&e_st, b_unassignedRows, nx, &ce_emlrtRTEI);
  unassignedRows_data = b_unassignedRows->data;
  e_st.site = &fn_emlrtRSI;
  nx = rowToCol->size[0];
  rowToCol->size[0] = costMatrix->size[1];
  emxEnsureCapacity_int32_T(&e_st, rowToCol, nx, &ae_emlrtRTEI);
  rowToCol_data = rowToCol->data;
  b_nOut = 0;
  f_st.site = &ko_emlrtRSI;
  for (k = 0; k < n; k++) {
    if (colToRow_data[k] > m) {
      b_nOut++;
      rowToCol_data[b_nOut - 1] = k + 1;
    }
  }
  emxFree_int32_T(&e_st, &colToRow);
  nx = rowToCol->size[0];
  rowToCol->size[0] = b_nOut;
  emxEnsureCapacity_int32_T(&e_st, rowToCol, nx, &de_emlrtRTEI);
  rowToCol_data = rowToCol->data;
  nx = assignments->size[0] * assignments->size[1];
  assignments->size[0] = matchings->size[0];
  assignments->size[1] = 2;
  emxEnsureCapacity_uint32_T(&b_st, assignments, nx, &ee_emlrtRTEI);
  assignments_data = assignments->data;
  nx = matchings->size[0] << 1;
  for (k = 0; k < nx; k++) {
    nr = matchings_data[k];
    if (nr < 0) {
      nr = 0;
    }
    assignments_data[k] = (uint32_T)nr;
  }
  emxFree_int32_T(&b_st, &matchings);
  nx = unassignedRows->size[0];
  unassignedRows->size[0] = nOut;
  emxEnsureCapacity_uint32_T(&b_st, unassignedRows, nx, &fe_emlrtRTEI);
  assignments_data = unassignedRows->data;
  for (k = 0; k < nOut; k++) {
    assignments_data[k] = (uint32_T)unassignedRows_data[k];
  }
  emxFree_int32_T(&b_st, &b_unassignedRows);
  nx = unassignedColumns->size[0];
  unassignedColumns->size[0] = b_nOut;
  emxEnsureCapacity_uint32_T(&b_st, unassignedColumns, nx, &ge_emlrtRTEI);
  assignments_data = unassignedColumns->data;
  for (k = 0; k < b_nOut; k++) {
    assignments_data[k] = (uint32_T)rowToCol_data[k];
  }
  emxFree_int32_T(&b_st, &rowToCol);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (AssignerGNN.c) */
