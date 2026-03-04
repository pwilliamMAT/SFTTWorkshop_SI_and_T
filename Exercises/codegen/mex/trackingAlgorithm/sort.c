/*
 * sort.c
 *
 * Code generation for function 'sort'
 *
 */

/* Include files */
#include "sort.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo bo_emlrtRSI = {
    76,                                               /* lineNo */
    "sort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pathName */
};

static emlrtRSInfo co_emlrtRSI = {
    79,                                               /* lineNo */
    "sort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pathName */
};

static emlrtRSInfo do_emlrtRSI = {
    81,                                               /* lineNo */
    "sort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pathName */
};

static emlrtRSInfo eo_emlrtRSI = {
    84,                                               /* lineNo */
    "sort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pathName */
};

static emlrtRSInfo fo_emlrtRSI = {
    87,                                               /* lineNo */
    "sort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pathName */
};

static emlrtRSInfo go_emlrtRSI = {
    90,                                               /* lineNo */
    "sort",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pathName */
};

static emlrtRTEInfo mi_emlrtRTEI = {
    56,                                               /* lineNo */
    24,                                               /* colNo */
    "sort",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pName */
};

static emlrtRTEInfo ni_emlrtRTEI = {
    75,                                               /* lineNo */
    26,                                               /* colNo */
    "sort",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pName */
};

static emlrtRTEInfo oi_emlrtRTEI = {
    56,                                               /* lineNo */
    1,                                                /* colNo */
    "sort",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pName */
};

static emlrtRTEInfo pi_emlrtRTEI = {
    1,                                                /* lineNo */
    20,                                               /* colNo */
    "sort",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sort.m" /* pName */
};

/* Function Definitions */
void b_sort(const emlrtStack *sp, emxArray_real_T *x)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_int32_T *dpb_emlrtRSI;
  emxArray_real_T *vwork;
  real_T *vwork_data;
  real_T *x_data;
  int32_T dim;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T k;
  int32_T vstride;
  boolean_T b_overflow;
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    i = x->size[0];
  } else {
    i = 1;
  }
  emxInit_real_T(sp, &vwork, 1, &oi_emlrtRTEI, true);
  i1 = vwork->size[0];
  vwork->size[0] = i;
  emxEnsureCapacity_real_T(sp, vwork, i1, &mi_emlrtRTEI);
  vwork_data = vwork->data;
  st.site = &bo_emlrtRSI;
  vstride = 1;
  i1 = dim - 2;
  for (k = 0; k <= i1; k++) {
    vstride *= x->size[0];
  }
  st.site = &co_emlrtRSI;
  st.site = &do_emlrtRSI;
  if (vstride > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  overflow = (i > 2147483646);
  b_overflow = (i > 2147483646);
  emxInit_int32_T(sp, &dpb_emlrtRSI, 1, &pi_emlrtRTEI);
  for (j = 0; j < vstride; j++) {
    st.site = &eo_emlrtRSI;
    if (overflow) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = 0; k < i; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    st.site = &fo_emlrtRSI;
    sortIdx(&st, vwork, dpb_emlrtRSI);
    vwork_data = vwork->data;
    st.site = &go_emlrtRSI;
    if (b_overflow) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = 0; k < i; k++) {
      x_data[j + k * vstride] = vwork_data[k];
    }
  }
  emxFree_int32_T(sp, &dpb_emlrtRSI);
  emxFree_real_T(sp, &vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_sort(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *vwork;
  real_T *vwork_data;
  real_T *x_data;
  int32_T dim;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T k;
  int32_T vstride;
  int32_T *idx_data;
  int32_T *iidx_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    i = x->size[0];
  } else {
    i = 1;
  }
  emxInit_real_T(sp, &vwork, 1, &oi_emlrtRTEI, true);
  i1 = vwork->size[0];
  vwork->size[0] = i;
  emxEnsureCapacity_real_T(sp, vwork, i1, &mi_emlrtRTEI);
  vwork_data = vwork->data;
  i1 = idx->size[0];
  idx->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(sp, idx, i1, &ni_emlrtRTEI);
  idx_data = idx->data;
  st.site = &bo_emlrtRSI;
  vstride = 1;
  i1 = dim - 2;
  for (k = 0; k <= i1; k++) {
    vstride *= x->size[0];
  }
  st.site = &co_emlrtRSI;
  st.site = &do_emlrtRSI;
  if (vstride > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  emxInit_int32_T(sp, &idx, 1, &pi_emlrtRTEI);
  for (j = 0; j < vstride; j++) {
    st.site = &eo_emlrtRSI;
    if (i > 2147483646) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = 0; k < i; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    st.site = &fo_emlrtRSI;
    b_sortIdx(&st, vwork, idx);
    iidx_data = idx->data;
    vwork_data = vwork->data;
    st.site = &go_emlrtRSI;
    for (k = 0; k < i; k++) {
      i1 = j + k * vstride;
      x_data[i1] = vwork_data[k];
      idx_data[i1] = iidx_data[k];
    }
  }
  emxFree_int32_T(sp, &idx);
  emxFree_real_T(sp, &vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void sort(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *vwork;
  real_T *vwork_data;
  real_T *x_data;
  int32_T dim;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T k;
  int32_T vstride;
  int32_T *idx_data;
  int32_T *iidx_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    i = x->size[0];
  } else {
    i = 1;
  }
  emxInit_real_T(sp, &vwork, 1, &oi_emlrtRTEI, true);
  i1 = vwork->size[0];
  vwork->size[0] = i;
  emxEnsureCapacity_real_T(sp, vwork, i1, &mi_emlrtRTEI);
  vwork_data = vwork->data;
  i1 = idx->size[0];
  idx->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(sp, idx, i1, &ni_emlrtRTEI);
  idx_data = idx->data;
  st.site = &bo_emlrtRSI;
  vstride = 1;
  i1 = dim - 2;
  for (k = 0; k <= i1; k++) {
    vstride *= x->size[0];
  }
  st.site = &co_emlrtRSI;
  st.site = &do_emlrtRSI;
  if (vstride > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  emxInit_int32_T(sp, &idx, 1, &pi_emlrtRTEI);
  for (j = 0; j < vstride; j++) {
    st.site = &eo_emlrtRSI;
    if (i > 2147483646) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = 0; k < i; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    st.site = &fo_emlrtRSI;
    sortIdx(&st, vwork, idx);
    iidx_data = idx->data;
    vwork_data = vwork->data;
    st.site = &go_emlrtRSI;
    for (k = 0; k < i; k++) {
      i1 = j + k * vstride;
      x_data[i1] = vwork_data[k];
      idx_data[i1] = iidx_data[k];
    }
  }
  emxFree_int32_T(sp, &idx);
  emxFree_real_T(sp, &vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (sort.c) */
