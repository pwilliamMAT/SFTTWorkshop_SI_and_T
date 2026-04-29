/*
 * sum.c
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ocb_emlrtRSI = {
    107,                /* lineNo */
    "blockedSummation", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\blocke"
    "dSummation.m" /* pathName */
};

static emlrtRSInfo pcb_emlrtRSI = {
    22,                    /* lineNo */
    "sumMatrixIncludeNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo qcb_emlrtRSI = {
    42,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo rcb_emlrtRSI = {
    41,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo ygb_emlrtRSI = {
    57,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRTEInfo bg_emlrtRTEI = {
    20,    /* lineNo */
    1,     /* colNo */
    "sum", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\sum.m" /* pName
                                                                        */
};

static emlrtRTEInfo cg_emlrtRTEI = {
    35,                    /* lineNo */
    20,                    /* colNo */
    "sumMatrixIncludeNaN", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pName */
};

/* Function Definitions */
int32_T b_sum(const emxArray_real_T *x, real_T y_data[])
{
  real_T bsum_data[51];
  const real_T *x_data;
  int32_T b_xj;
  int32_T ib;
  int32_T xj;
  int32_T y_size;
  x_data = x->data;
  if (x->size[1] == 0) {
    int32_T hi;
    y_size = (int8_T)x->size[0];
    hi = (int8_T)x->size[0];
    if (hi - 1 >= 0) {
      memset(&y_data[0], 0, (uint32_T)hi * sizeof(real_T));
    }
  } else {
    __m128d r;
    int32_T bvstride;
    int32_T firstBlockLength;
    int32_T hi;
    int32_T lastBlockLength;
    int32_T nblocks;
    int32_T vectorUB;
    int32_T vstride;
    int32_T xoffset;
    vstride = x->size[0] - 1;
    bvstride = x->size[0] << 10;
    y_size = x->size[0];
    if (x->size[1] <= 1024) {
      firstBlockLength = x->size[1];
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = (int32_T)((uint32_T)x->size[1] >> 10);
      lastBlockLength = x->size[1] - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    for (xj = 0; xj <= vstride; xj++) {
      y_data[xj] = x_data[xj];
      bsum_data[xj] = 0.0;
    }
    for (xj = 2; xj <= firstBlockLength; xj++) {
      xoffset = (xj - 1) * (vstride + 1);
      hi = ((vstride + 1) / 2) << 1;
      vectorUB = hi - 2;
      for (b_xj = 0; b_xj <= vectorUB; b_xj += 2) {
        r = _mm_loadu_pd(&y_data[b_xj]);
        _mm_storeu_pd(&y_data[b_xj],
                      _mm_add_pd(r, _mm_loadu_pd(&x_data[xoffset + b_xj])));
      }
      for (b_xj = hi; b_xj <= vstride; b_xj++) {
        y_data[b_xj] += x_data[xoffset + b_xj];
      }
    }
    for (ib = 2; ib <= nblocks; ib++) {
      int32_T xblockoffset;
      xblockoffset = (ib - 1) * bvstride;
      for (xj = 0; xj <= vstride; xj++) {
        bsum_data[xj] = x_data[xblockoffset + xj];
      }
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }
      for (xj = 2; xj <= hi; xj++) {
        xoffset = xblockoffset + (xj - 1) * (vstride + 1);
        vectorUB = ((vstride + 1) / 2) << 1;
        firstBlockLength = vectorUB - 2;
        for (b_xj = 0; b_xj <= firstBlockLength; b_xj += 2) {
          r = _mm_loadu_pd(&bsum_data[b_xj]);
          _mm_storeu_pd(&bsum_data[b_xj],
                        _mm_add_pd(r, _mm_loadu_pd(&x_data[xoffset + b_xj])));
        }
        for (b_xj = vectorUB; b_xj <= vstride; b_xj++) {
          bsum_data[b_xj] += x_data[xoffset + b_xj];
        }
      }
      hi = ((vstride + 1) / 2) << 1;
      vectorUB = hi - 2;
      for (xj = 0; xj <= vectorUB; xj += 2) {
        __m128d r1;
        r = _mm_loadu_pd(&y_data[xj]);
        r1 = _mm_loadu_pd(&bsum_data[xj]);
        _mm_storeu_pd(&y_data[xj], _mm_add_pd(r, r1));
      }
      for (xj = hi; xj <= vstride; xj++) {
        y_data[xj] += bsum_data[xj];
      }
    }
  }
  return y_size;
}

real_T c_sum(const emlrtStack *sp, const emxArray_real_T *x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T y;
  int32_T ib;
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
  st.site = &mcb_emlrtRSI;
  b_st.site = &pbb_emlrtRSI;
  c_st.site = &ncb_emlrtRSI;
  if (x->size[0] == 0) {
    y = 0.0;
  } else {
    d_st.site = &ocb_emlrtRSI;
    e_st.site = &pcb_emlrtRSI;
    if (x->size[0] < 4096) {
      f_st.site = &qcb_emlrtRSI;
      y = b_sumColumnB(&f_st, x, x->size[0]);
    } else {
      int32_T inb;
      int32_T nfb;
      int32_T nleft;
      nfb = (int32_T)((uint32_T)x->size[0] >> 12);
      inb = nfb << 12;
      nleft = x->size[0] - inb;
      y = sumColumnB4(x, 1);
      for (ib = 2; ib <= nfb; ib++) {
        y += sumColumnB4(x, ((ib - 1) << 12) + 1);
      }
      if (nleft > 0) {
        f_st.site = &ygb_emlrtRSI;
        y += c_sumColumnB(&f_st, x, nleft, inb + 1);
      }
    }
  }
  return y;
}

void sum(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T *y_data;
  int32_T col;
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
  st.site = &mcb_emlrtRSI;
  b_st.site = &pbb_emlrtRSI;
  c_st.site = &ncb_emlrtRSI;
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    int32_T loop_ub;
    loop_ub = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&c_st, y, loop_ub, &bg_emlrtRTEI);
    y_data = y->data;
    loop_ub = x->size[1];
    for (col = 0; col < loop_ub; col++) {
      y_data[col] = 0.0;
    }
  } else {
    int32_T i;
    int32_T loop_ub;
    d_st.site = &ocb_emlrtRSI;
    e_st.site = &pcb_emlrtRSI;
    loop_ub = y->size[0] * y->size[1];
    y->size[0] = 1;
    i = x->size[1];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&e_st, y, loop_ub, &cg_emlrtRTEI);
    y_data = y->data;
    f_st.site = &rcb_emlrtRSI;
    if (x->size[1] > 2147483646) {
      g_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&g_st);
    }
    for (col = 0; col < i; col++) {
      f_st.site = &qcb_emlrtRSI;
      y_data[col] = sumColumnB(&f_st, x, col + 1, x->size[0]);
    }
  }
}

/* End of code generation (sum.c) */
