/*
 * sortIdx.c
 *
 * Code generation for function 'sortIdx'
 *
 */

/* Include files */
#include "sortIdx.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo
    gp_emlrtRSI =
        {
            488,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    ip_emlrtRSI =
        {
            496,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    jp_emlrtRSI =
        {
            503,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    kp_emlrtRSI =
        {
            550,     /* lineNo */
            "merge", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    lp_emlrtRSI =
        {
            519,     /* lineNo */
            "merge", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

/* Function Declarations */
static void b_merge(const emlrtStack *sp, emxArray_int32_T *idx,
                    emxArray_real_T *x, int32_T offset, int32_T np, int32_T nq,
                    emxArray_int32_T *iwork, emxArray_real_T *xwork);

static void merge(const emlrtStack *sp, emxArray_int32_T *idx,
                  emxArray_uint32_T *x, int32_T offset, int32_T np, int32_T nq,
                  emxArray_int32_T *iwork, emxArray_uint32_T *xwork);

/* Function Definitions */
static void b_merge(const emlrtStack *sp, emxArray_int32_T *idx,
                    emxArray_real_T *x, int32_T offset, int32_T np, int32_T nq,
                    emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T *x_data;
  real_T *xwork_data;
  int32_T j;
  int32_T *idx_data;
  int32_T *iwork_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  xwork_data = xwork->data;
  iwork_data = iwork->data;
  x_data = x->data;
  idx_data = idx->data;
  if (nq != 0) {
    int32_T iout;
    int32_T n;
    int32_T p;
    int32_T q;
    n = np + nq;
    st.site = &lp_emlrtRSI;
    if (n > 2147483646) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (j = 0; j < n; j++) {
      q = offset + j;
      iwork_data[j] = idx_data[q];
      xwork_data[j] = x_data[q];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n) {
          q++;
        } else {
          q = iout - p;
          st.site = &kp_emlrtRSI;
          if ((p + 1 <= np) && (np > 2147483646)) {
            b_st.site = &tb_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

static void merge(const emlrtStack *sp, emxArray_int32_T *idx,
                  emxArray_uint32_T *x, int32_T offset, int32_T np, int32_T nq,
                  emxArray_int32_T *iwork, emxArray_uint32_T *xwork)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T j;
  int32_T *idx_data;
  int32_T *iwork_data;
  uint32_T *x_data;
  uint32_T *xwork_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  xwork_data = xwork->data;
  iwork_data = iwork->data;
  x_data = x->data;
  idx_data = idx->data;
  if (nq != 0) {
    int32_T iout;
    int32_T n;
    int32_T p;
    int32_T q;
    n = np + nq;
    st.site = &lp_emlrtRSI;
    if (n > 2147483646) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (j = 0; j < n; j++) {
      q = offset + j;
      iwork_data[j] = idx_data[q];
      xwork_data[j] = x_data[q];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n) {
          q++;
        } else {
          q = iout - p;
          st.site = &kp_emlrtRSI;
          if ((p + 1 <= np) && (np > 2147483646)) {
            b_st.site = &tb_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

void b_merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
                   emxArray_real_T *x, int32_T offset, int32_T n,
                   int32_T preSortLevel, emxArray_int32_T *iwork,
                   emxArray_real_T *xwork)
{
  emlrtStack st;
  int32_T bLen;
  int32_T k;
  int32_T nPairs;
  st.prev = sp;
  st.tls = sp->tls;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    int32_T tailOffset;
    if (((uint32_T)nPairs & 1U) != 0U) {
      int32_T nTail;
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        st.site = &gp_emlrtRSI;
        b_merge(&st, idx, x, offset + tailOffset, bLen, nTail - bLen, iwork,
                xwork);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (k = 0; k < nPairs; k++) {
      st.site = &ip_emlrtRSI;
      b_merge(&st, idx, x, offset + k * tailOffset, bLen, bLen, iwork, xwork);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    st.site = &jp_emlrtRSI;
    b_merge(&st, idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

void merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
                 emxArray_uint32_T *x, int32_T offset, int32_T n,
                 int32_T preSortLevel, emxArray_int32_T *iwork,
                 emxArray_uint32_T *xwork)
{
  emlrtStack st;
  int32_T bLen;
  int32_T k;
  int32_T nPairs;
  st.prev = sp;
  st.tls = sp->tls;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    int32_T tailOffset;
    if (((uint32_T)nPairs & 1U) != 0U) {
      int32_T nTail;
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        st.site = &gp_emlrtRSI;
        merge(&st, idx, x, offset + tailOffset, bLen, nTail - bLen, iwork,
              xwork);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (k = 0; k < nPairs; k++) {
      st.site = &ip_emlrtRSI;
      merge(&st, idx, x, offset + k * tailOffset, bLen, bLen, iwork, xwork);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    st.site = &jp_emlrtRSI;
    merge(&st, idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

/* End of code generation (sortIdx.c) */
