/*
 * sort.c
 *
 * Code generation for function 'sort'
 *
 */

/* Include files */
#include "sort.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo cn_emlrtRSI = {
    76,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo dn_emlrtRSI = {
    79,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo en_emlrtRSI = {
    81,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo fn_emlrtRSI = {
    84,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo gn_emlrtRSI = {
    87,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo hn_emlrtRSI = {
    90,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo in_emlrtRSI = {
    55,         /* lineNo */
    "prodsize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\prodsize.m" /* pathName */
};

static emlrtRSInfo
    jn_emlrtRSI =
        {
            105,       /* lineNo */
            "sortIdx", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    kn_emlrtRSI =
        {
            301,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    ln_emlrtRSI =
        {
            309,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    mn_emlrtRSI =
        {
            310,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    nn_emlrtRSI =
        {
            318,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    on_emlrtRSI =
        {
            326,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    pn_emlrtRSI =
        {
            432,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    qn_emlrtRSI =
        {
            457,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    rn_emlrtRSI =
        {
            462,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    sn_emlrtRSI =
        {
            576,                /* lineNo */
            "merge_pow2_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    tn_emlrtRSI =
        {
            578,                /* lineNo */
            "merge_pow2_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    un_emlrtRSI =
        {
            606,                /* lineNo */
            "merge_pow2_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo ar_emlrtRSI = {
    72,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo
    br_emlrtRSI =
        {
            381,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    cr_emlrtRSI =
        {
            409,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    dr_emlrtRSI =
        {
            416,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRTEInfo rj_emlrtRTEI = {
    56,     /* lineNo */
    24,     /* colNo */
    "sort", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pName
                                                                           */
};

static emlrtRTEInfo
    sj_emlrtRTEI =
        {
            61,        /* lineNo */
            5,         /* colNo */
            "sortIdx", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pName */
};

static emlrtRTEInfo
    tj_emlrtRTEI =
        {
            296,       /* lineNo */
            1,         /* colNo */
            "sortIdx", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pName */
};

static emlrtRTEInfo
    uj_emlrtRTEI =
        {
            298,       /* lineNo */
            24,        /* colNo */
            "sortIdx", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pName */
};

static emlrtRTEInfo vj_emlrtRTEI = {
    56,     /* lineNo */
    1,      /* colNo */
    "sort", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pName
                                                                           */
};

static emlrtRTEInfo wj_emlrtRTEI = {
    1,      /* lineNo */
    20,     /* colNo */
    "sort", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pName
                                                                           */
};

static emlrtRTEInfo
    xj_emlrtRTEI =
        {
            298,       /* lineNo */
            1,         /* colNo */
            "sortIdx", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pName */
};

static emlrtRTEInfo
    jk_emlrtRTEI =
        {
            56,        /* lineNo */
            5,         /* colNo */
            "sortIdx", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pName */
};

/* Function Definitions */
void b_sort(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  real_T *x_data;
  real_T *xwork_data;
  int32_T b;
  int32_T b_b;
  int32_T b_k;
  int32_T i1;
  int32_T k;
  int32_T nBlocks;
  int32_T *idx_data;
  int32_T *iwork_data;
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
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ar_emlrtRSI;
  i1 = idx->size[0] * idx->size[1];
  idx->size[0] = 1;
  nBlocks = x->size[1];
  idx->size[1] = nBlocks;
  emxEnsureCapacity_int32_T(&st, idx, i1, &jk_emlrtRTEI);
  idx_data = idx->data;
  for (k = 0; k < nBlocks; k++) {
    idx_data[k] = 0;
  }
  if (x->size[1] != 0) {
    real_T x4[4];
    int32_T idx4[4];
    int32_T bLen;
    int32_T bLen2;
    int32_T i2;
    int32_T i4;
    int32_T ib;
    int32_T nNaNs;
    int32_T quartetOffset;
    int32_T wOffset;
    b_st.site = &jn_emlrtRSI;
    emxInit_int32_T(&b_st, &iwork, 1, &tj_emlrtRTEI);
    i1 = iwork->size[0];
    iwork->size[0] = nBlocks;
    emxEnsureCapacity_int32_T(&b_st, iwork, i1, &tj_emlrtRTEI);
    iwork_data = iwork->data;
    for (k = 0; k < nBlocks; k++) {
      iwork_data[k] = 0;
    }
    emxInit_real_T(&b_st, &xwork, 1, &xj_emlrtRTEI);
    i1 = xwork->size[0];
    xwork->size[0] = nBlocks;
    emxEnsureCapacity_real_T(&b_st, xwork, i1, &uj_emlrtRTEI);
    xwork_data = xwork->data;
    c_st.site = &kn_emlrtRSI;
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    nNaNs = 0;
    ib = 0;
    d_st.site = &br_emlrtRSI;
    if (x->size[1] > 2147483646) {
      e_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    for (k = 0; k < nBlocks; k++) {
      if (muDoubleScalarIsNaN(x_data[k])) {
        i1 = (nBlocks - nNaNs) - 1;
        idx_data[i1] = k + 1;
        xwork_data[i1] = x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = k + 1;
        x4[ib - 1] = x_data[k];
        if (ib == 4) {
          real_T d;
          real_T d1;
          quartetOffset = k - nNaNs;
          if (x4[0] <= x4[1]) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (x4[2] <= x4[3]) {
            ib = 3;
            i4 = 4;
          } else {
            ib = 4;
            i4 = 3;
          }
          d = x4[i1 - 1];
          d1 = x4[ib - 1];
          if (d <= d1) {
            if (x4[i2 - 1] <= d1) {
              bLen = i1;
              bLen2 = i2;
              i1 = ib;
              i2 = i4;
            } else if (x4[i2 - 1] <= x4[i4 - 1]) {
              bLen = i1;
              bLen2 = ib;
              i1 = i2;
              i2 = i4;
            } else {
              bLen = i1;
              bLen2 = ib;
              i1 = i4;
            }
          } else if (d <= x4[i4 - 1]) {
            if (x4[i2 - 1] <= x4[i4 - 1]) {
              bLen = ib;
              bLen2 = i1;
              i1 = i2;
              i2 = i4;
            } else {
              bLen = ib;
              bLen2 = i1;
              i1 = i4;
            }
          } else {
            bLen = ib;
            bLen2 = i4;
          }
          idx_data[quartetOffset - 3] = idx4[bLen - 1];
          idx_data[quartetOffset - 2] = idx4[bLen2 - 1];
          idx_data[quartetOffset - 1] = idx4[i1 - 1];
          idx_data[quartetOffset] = idx4[i2 - 1];
          x_data[quartetOffset - 3] = x4[bLen - 1];
          x_data[quartetOffset - 2] = x4[bLen2 - 1];
          x_data[quartetOffset - 1] = x4[i1 - 1];
          x_data[quartetOffset] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    wOffset = x->size[1] - nNaNs;
    if (ib > 0) {
      int8_T perm[4];
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }
      d_st.site = &cr_emlrtRSI;
      if (ib > 2147483646) {
        e_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&e_st);
      }
      i1 = (uint8_T)ib;
      for (k = 0; k < i1; k++) {
        quartetOffset = (wOffset - ib) + k;
        i2 = perm[k];
        idx_data[quartetOffset] = idx4[i2 - 1];
        x_data[quartetOffset] = x4[i2 - 1];
      }
    }
    i1 = nNaNs >> 1;
    d_st.site = &dr_emlrtRSI;
    for (k = 0; k < i1; k++) {
      quartetOffset = wOffset + k;
      i2 = idx_data[quartetOffset];
      ib = (nBlocks - k) - 1;
      idx_data[quartetOffset] = idx_data[ib];
      idx_data[ib] = i2;
      x_data[quartetOffset] = xwork_data[ib];
      x_data[ib] = xwork_data[quartetOffset];
    }
    if (((uint32_T)nNaNs & 1U) != 0U) {
      i1 += wOffset;
      x_data[i1] = xwork_data[i1];
    }
    i1 = 2;
    if (wOffset > 1) {
      if (x->size[1] >= 256) {
        nBlocks = wOffset >> 8;
        if (nBlocks > 0) {
          c_st.site = &ln_emlrtRSI;
          for (b = 0; b < nBlocks; b++) {
            real_T b_xwork[256];
            int32_T b_iwork[256];
            c_st.site = &mn_emlrtRSI;
            i4 = (b << 8) - 1;
            for (b_b = 0; b_b < 6; b_b++) {
              bLen = 1 << (b_b + 2);
              bLen2 = bLen << 1;
              nNaNs = 256 >> (b_b + 3);
              d_st.site = &sn_emlrtRSI;
              for (b_k = 0; b_k < nNaNs; b_k++) {
                i1 = (i4 + b_k * bLen2) + 1;
                d_st.site = &tn_emlrtRSI;
                for (k = 0; k < bLen2; k++) {
                  ib = i1 + k;
                  b_iwork[k] = idx_data[ib];
                  b_xwork[k] = x_data[ib];
                }
                i2 = 0;
                quartetOffset = bLen;
                ib = i1 - 1;
                int32_T exitg1;
                do {
                  exitg1 = 0;
                  ib++;
                  if (b_xwork[i2] <= b_xwork[quartetOffset]) {
                    idx_data[ib] = b_iwork[i2];
                    x_data[ib] = b_xwork[i2];
                    if (i2 + 1 < bLen) {
                      i2++;
                    } else {
                      exitg1 = 1;
                    }
                  } else {
                    idx_data[ib] = b_iwork[quartetOffset];
                    x_data[ib] = b_xwork[quartetOffset];
                    if (quartetOffset + 1 < bLen2) {
                      quartetOffset++;
                    } else {
                      ib -= i2;
                      d_st.site = &un_emlrtRSI;
                      for (k = i2 + 1; k <= bLen; k++) {
                        quartetOffset = ib + k;
                        idx_data[quartetOffset] = b_iwork[k - 1];
                        x_data[quartetOffset] = b_xwork[k - 1];
                      }
                      exitg1 = 1;
                    }
                  }
                } while (exitg1 == 0);
              }
            }
          }
          i1 = nBlocks << 8;
          quartetOffset = wOffset - i1;
          if (quartetOffset > 0) {
            c_st.site = &nn_emlrtRSI;
            b_merge_block(&c_st, idx, x, i1, quartetOffset, 2, iwork, xwork);
          }
          i1 = 8;
        }
      }
      c_st.site = &on_emlrtRSI;
      b_merge_block(&c_st, idx, x, 0, wOffset, i1, iwork, xwork);
    }
    emxFree_real_T(&b_st, &xwork);
    emxFree_int32_T(&b_st, &iwork);
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void sort(const emlrtStack *sp, emxArray_uint32_T *x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_int32_T *iidx;
  emxArray_int32_T *iwork;
  emxArray_uint32_T *vwork;
  int32_T b;
  int32_T b_b;
  int32_T b_i;
  int32_T b_k;
  int32_T dim;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T vstride;
  int32_T *iidx_data;
  int32_T *iwork_data;
  uint32_T *vwork_data;
  uint32_T *x_data;
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
  emxInit_uint32_T(sp, &vwork, 1, &vj_emlrtRTEI);
  b_i = vwork->size[0];
  vwork->size[0] = i;
  emxEnsureCapacity_uint32_T(sp, vwork, b_i, &rj_emlrtRTEI);
  vwork_data = vwork->data;
  st.site = &cn_emlrtRSI;
  vstride = 1;
  b_i = dim - 2;
  b_st.site = &in_emlrtRSI;
  for (k = 0; k <= b_i; k++) {
    vstride *= x->size[0];
  }
  st.site = &dn_emlrtRSI;
  st.site = &en_emlrtRSI;
  if (vstride > 2147483646) {
    b_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  emxInit_int32_T(sp, &iidx, 1, &wj_emlrtRTEI);
  emxInit_int32_T(sp, &iwork, 1, &tj_emlrtRTEI);
  emxInit_uint32_T(sp, &x, 1, &xj_emlrtRTEI);
  for (j = 0; j < vstride; j++) {
    st.site = &fn_emlrtRSI;
    if (i > 2147483646) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = 0; k < i; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    st.site = &gn_emlrtRSI;
    b_i = vwork->size[0];
    dim = iidx->size[0];
    iidx->size[0] = vwork->size[0];
    emxEnsureCapacity_int32_T(&st, iidx, dim, &sj_emlrtRTEI);
    iidx_data = iidx->data;
    for (k = 0; k < b_i; k++) {
      iidx_data[k] = 0;
    }
    if (vwork->size[0] != 0) {
      int32_T idx4[4];
      int32_T bLen;
      int32_T bLen2;
      int32_T i1;
      int32_T i2;
      int32_T i3;
      int32_T i4;
      uint32_T x4[4];
      b_st.site = &jn_emlrtRSI;
      dim = iwork->size[0];
      iwork->size[0] = vwork->size[0];
      emxEnsureCapacity_int32_T(&b_st, iwork, dim, &tj_emlrtRTEI);
      iwork_data = iwork->data;
      for (k = 0; k < b_i; k++) {
        iwork_data[k] = 0;
      }
      b_i = x->size[0];
      x->size[0] = vwork->size[0];
      emxEnsureCapacity_uint32_T(&b_st, x, b_i, &uj_emlrtRTEI);
      c_st.site = &kn_emlrtRSI;
      x4[0] = 0U;
      idx4[0] = 0;
      x4[1] = 0U;
      idx4[1] = 0;
      x4[2] = 0U;
      idx4[2] = 0;
      x4[3] = 0U;
      idx4[3] = 0;
      dim = vwork->size[0] >> 2;
      d_st.site = &pn_emlrtRSI;
      for (k = 0; k < dim; k++) {
        uint32_T b_x4_tmp;
        uint32_T c_x4_tmp;
        uint32_T x4_tmp;
        b_i = k << 2;
        idx4[0] = b_i + 1;
        idx4[1] = b_i + 2;
        idx4[2] = b_i + 3;
        idx4[3] = b_i + 4;
        x4[0] = vwork_data[b_i];
        x4_tmp = vwork_data[b_i + 1];
        x4[1] = x4_tmp;
        b_x4_tmp = vwork_data[b_i + 2];
        x4[2] = b_x4_tmp;
        c_x4_tmp = vwork_data[b_i + 3];
        x4[3] = c_x4_tmp;
        if (vwork_data[b_i] <= x4_tmp) {
          i1 = 1;
          i2 = 2;
        } else {
          i1 = 2;
          i2 = 1;
        }
        if (b_x4_tmp <= c_x4_tmp) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }
        x4_tmp = x4[i1 - 1];
        b_x4_tmp = x4[i3 - 1];
        if (x4_tmp <= b_x4_tmp) {
          if (x4[i2 - 1] <= b_x4_tmp) {
            bLen = i1;
            bLen2 = i2;
            i1 = i3;
            i2 = i4;
          } else if (x4[i2 - 1] <= x4[i4 - 1]) {
            bLen = i1;
            bLen2 = i3;
            i1 = i2;
            i2 = i4;
          } else {
            bLen = i1;
            bLen2 = i3;
            i1 = i4;
          }
        } else if (x4_tmp <= x4[i4 - 1]) {
          if (x4[i2 - 1] <= x4[i4 - 1]) {
            bLen = i3;
            bLen2 = i1;
            i1 = i2;
            i2 = i4;
          } else {
            bLen = i3;
            bLen2 = i1;
            i1 = i4;
          }
        } else {
          bLen = i3;
          bLen2 = i4;
        }
        iidx_data[b_i] = idx4[bLen - 1];
        iidx_data[b_i + 1] = idx4[bLen2 - 1];
        iidx_data[b_i + 2] = idx4[i1 - 1];
        iidx_data[b_i + 3] = idx4[i2 - 1];
        vwork_data[b_i] = x4[bLen - 1];
        vwork_data[b_i + 1] = x4[bLen2 - 1];
        vwork_data[b_i + 2] = x4[i1 - 1];
        vwork_data[b_i + 3] = x4[i2 - 1];
      }
      i1 = dim << 2;
      i2 = vwork->size[0] - i1;
      if (i2 > 0) {
        int8_T perm[4];
        d_st.site = &qn_emlrtRSI;
        if (i2 > 2147483646) {
          e_st.site = &tb_emlrtRSI;
          check_forloop_overflow_error(&e_st);
        }
        for (k = 0; k < i2; k++) {
          dim = i1 + k;
          idx4[k] = dim + 1;
          x4[k] = vwork_data[dim];
        }
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (i2 == 1) {
          perm[0] = 1;
        } else if (i2 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }
        d_st.site = &rn_emlrtRSI;
        for (k = 0; k < i2; k++) {
          dim = i1 + k;
          b_i = perm[k];
          iidx_data[dim] = idx4[b_i - 1];
          vwork_data[dim] = x4[b_i - 1];
        }
      }
      b_i = 2;
      if (vwork->size[0] > 1) {
        if (vwork->size[0] >= 256) {
          i3 = vwork->size[0] >> 8;
          c_st.site = &ln_emlrtRSI;
          for (b = 0; b < i3; b++) {
            int32_T b_iwork[256];
            uint32_T xwork[256];
            c_st.site = &mn_emlrtRSI;
            i4 = (b << 8) - 1;
            for (b_b = 0; b_b < 6; b_b++) {
              int32_T b_i1;
              bLen = 1 << (b_b + 2);
              bLen2 = bLen << 1;
              b_i1 = 256 >> (b_b + 3);
              d_st.site = &sn_emlrtRSI;
              for (b_k = 0; b_k < b_i1; b_k++) {
                i1 = (i4 + b_k * bLen2) + 1;
                d_st.site = &tn_emlrtRSI;
                for (k = 0; k < bLen2; k++) {
                  dim = i1 + k;
                  b_iwork[k] = iidx_data[dim];
                  xwork[k] = vwork_data[dim];
                }
                i2 = 0;
                b_i = bLen;
                dim = i1 - 1;
                int32_T exitg1;
                do {
                  exitg1 = 0;
                  dim++;
                  if (xwork[i2] <= xwork[b_i]) {
                    iidx_data[dim] = b_iwork[i2];
                    vwork_data[dim] = xwork[i2];
                    if (i2 + 1 < bLen) {
                      i2++;
                    } else {
                      exitg1 = 1;
                    }
                  } else {
                    iidx_data[dim] = b_iwork[b_i];
                    vwork_data[dim] = xwork[b_i];
                    if (b_i + 1 < bLen2) {
                      b_i++;
                    } else {
                      dim -= i2;
                      d_st.site = &un_emlrtRSI;
                      for (k = i2 + 1; k <= bLen; k++) {
                        b_i = dim + k;
                        iidx_data[b_i] = b_iwork[k - 1];
                        vwork_data[b_i] = xwork[k - 1];
                      }
                      exitg1 = 1;
                    }
                  }
                } while (exitg1 == 0);
              }
            }
          }
          dim = i3 << 8;
          b_i = vwork->size[0] - dim;
          if (b_i > 0) {
            c_st.site = &nn_emlrtRSI;
            merge_block(&c_st, iidx, vwork, dim, b_i, 2, iwork, x);
          }
          b_i = 8;
        }
        c_st.site = &on_emlrtRSI;
        merge_block(&c_st, iidx, vwork, 0, vwork->size[0], b_i, iwork, x);
        vwork_data = vwork->data;
      }
    }
    st.site = &hn_emlrtRSI;
    for (k = 0; k < i; k++) {
      x_data[j + k * vstride] = vwork_data[k];
    }
  }
  emxFree_uint32_T(sp, &x);
  emxFree_int32_T(sp, &iwork);
  emxFree_int32_T(sp, &iidx);
  emxFree_uint32_T(sp, &vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (sort.c) */
