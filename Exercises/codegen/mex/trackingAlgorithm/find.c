/*
 * find.c
 *
 * Code generation for function 'find'
 *
 */

/* Include files */
#include "find.h"
#include "eml_int_forloop_overflow_check.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo odb_emlrtRSI = {
    138,                                          /* lineNo */
    "eml_find",                                   /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pathName */
};

static emlrtRSInfo pdb_emlrtRSI = {
    376,                                          /* lineNo */
    "find_first_indices",                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pathName */
};

static emlrtRSInfo qhb_emlrtRSI = {
    396,                                          /* lineNo */
    "find_first_indices",                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pathName */
};

static emlrtRTEInfo hb_emlrtRTEI = {
    386,                                          /* lineNo */
    1,                                            /* colNo */
    "find_first_indices",                         /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

static emlrtRTEInfo xb_emlrtRTEI = {
    81,                                           /* lineNo */
    1,                                            /* colNo */
    "eml_find",                                   /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

static emlrtRTEInfo se_emlrtRTEI = {
    358,                                          /* lineNo */
    24,                                           /* colNo */
    "find",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

static emlrtRTEInfo te_emlrtRTEI = {
    138,                                          /* lineNo */
    9,                                            /* colNo */
    "find",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

static emlrtRTEInfo ch_emlrtRTEI = {
    363,                                          /* lineNo */
    24,                                           /* colNo */
    "find",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

/* Function Definitions */
void b_eml_find(const emlrtStack *sp, const emxArray_boolean_T *x,
                emxArray_int32_T *i)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T b_iv[2];
  int32_T idx;
  int32_T ii;
  int32_T nx;
  int32_T *i_data;
  const boolean_T *x_data;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  nx = x->size[0];
  st.site = &odb_emlrtRSI;
  idx = 0;
  ii = i->size[0];
  i->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&st, i, ii, &ch_emlrtRTEI);
  i_data = i->data;
  b_st.site = &pdb_emlrtRSI;
  if (x->size[0] > 2147483646) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx > x->size[0]) {
    emlrtErrorWithMessageIdR2018a(&st, &hb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (idx < 1) {
    idx = 0;
  }
  b_iv[0] = 1;
  b_iv[1] = idx;
  b_st.site = &qhb_emlrtRSI;
  b_indexShapeCheck(&b_st, i->size[0], b_iv);
  ii = i->size[0];
  i->size[0] = idx;
  emxEnsureCapacity_int32_T(&st, i, ii, &te_emlrtRTEI);
}

void c_eml_find(const emlrtStack *sp, const emxArray_boolean_T *x,
                emxArray_int32_T *i)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T idx;
  int32_T ii;
  int32_T k;
  int32_T *i_data;
  const boolean_T *x_data;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  k = (x->size[1] >= 1);
  if (k > x->size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &xb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  st.site = &odb_emlrtRSI;
  idx = 0;
  ii = i->size[0] * i->size[1];
  i->size[0] = 1;
  i->size[1] = k;
  emxEnsureCapacity_int32_T(&st, i, ii, &se_emlrtRTEI);
  i_data = i->data;
  b_st.site = &pdb_emlrtRSI;
  if (x->size[1] > 2147483646) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x->size[1] - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= k) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx > k) {
    emlrtErrorWithMessageIdR2018a(&st, &hb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (k == 1) {
    if (idx == 0) {
      i->size[0] = 1;
      i->size[1] = 0;
    }
  } else {
    ii = i->size[0] * i->size[1];
    i->size[1] = (idx >= 1);
    emxEnsureCapacity_int32_T(&st, i, ii, &te_emlrtRTEI);
  }
}

int32_T d_eml_find(const emlrtStack *sp, const real_T x_data[], int32_T x_size,
                   int32_T i_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i_size;
  int32_T idx;
  int32_T ii;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i_size = (x_size >= 1);
  if (i_size > x_size) {
    emlrtErrorWithMessageIdR2018a(sp, &xb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  st.site = &odb_emlrtRSI;
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x_size - 1)) {
    if (x_data[ii] != 0.0) {
      idx = 1;
      i_data[0] = ii + 1;
      exitg1 = true;
    } else {
      ii++;
    }
  }
  if (idx > i_size) {
    emlrtErrorWithMessageIdR2018a(&st, &hb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (i_size == 1) {
    if (idx == 0) {
      i_size = 0;
    }
  } else {
    int32_T b_iv[2];
    i_size = (idx >= 1);
    b_iv[0] = 1;
    b_iv[1] = i_size;
    b_st.site = &qhb_emlrtRSI;
    b_indexShapeCheck(&b_st, 0, b_iv);
  }
  return i_size;
}

void eml_find(const emlrtStack *sp, const emxArray_boolean_T *x,
              emxArray_int32_T *i)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T idx;
  int32_T ii;
  int32_T nx;
  int32_T *i_data;
  const boolean_T *x_data;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  nx = x->size[1];
  st.site = &odb_emlrtRSI;
  idx = 0;
  ii = i->size[0] * i->size[1];
  i->size[0] = 1;
  i->size[1] = x->size[1];
  emxEnsureCapacity_int32_T(&st, i, ii, &se_emlrtRTEI);
  i_data = i->data;
  b_st.site = &pdb_emlrtRSI;
  if (x->size[1] > 2147483646) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx > x->size[1]) {
    emlrtErrorWithMessageIdR2018a(&st, &hb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (x->size[1] == 1) {
    if (idx == 0) {
      i->size[0] = 1;
      i->size[1] = 0;
    }
  } else {
    ii = i->size[0] * i->size[1];
    if (idx < 1) {
      i->size[1] = 0;
    } else {
      i->size[1] = idx;
    }
    emxEnsureCapacity_int32_T(&st, i, ii, &te_emlrtRTEI);
  }
}

/* End of code generation (find.c) */
