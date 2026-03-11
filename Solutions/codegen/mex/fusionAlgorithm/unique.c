/*
 * unique.c
 *
 * Code generation for function 'unique'
 *
 */

/* Include files */
#include "unique.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo jb_emlrtRSI = {
    161,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo kb_emlrtRSI = {
    163,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo lb_emlrtRSI = {
    180,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo mb_emlrtRSI = {
    211,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo nb_emlrtRSI = {
    224,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo ob_emlrtRSI = {
    235,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo pb_emlrtRSI = {
    249,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo
    qb_emlrtRSI =
        {
            145,       /* lineNo */
            "sortIdx", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo rb_emlrtRSI =
    {
        57,          /* lineNo */
        "mergesort", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\mergesort.m" /* pathName */
};

static emlrtRSInfo sb_emlrtRSI =
    {
        113,         /* lineNo */
        "mergesort", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\mergesort.m" /* pathName */
};

static emlrtRSInfo jo_emlrtRSI = {
    243,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRTEInfo e_emlrtRTEI = {
    242,             /* lineNo */
    1,               /* colNo */
    "unique_vector", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo mc_emlrtRTEI = {
    161,      /* lineNo */
    1,        /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo nc_emlrtRTEI =
    {
        52,          /* lineNo */
        9,           /* colNo */
        "mergesort", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\mergesort.m" /* pName */
};

static emlrtRTEInfo oc_emlrtRTEI = {
    162,      /* lineNo */
    20,       /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo pc_emlrtRTEI = {
    243,      /* lineNo */
    1,        /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo qc_emlrtRTEI =
    {
        52,          /* lineNo */
        1,           /* colNo */
        "mergesort", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\mergesort.m" /* pName */
};

/* Function Definitions */
void b_unique_vector(const emlrtStack *sp, const emxArray_uint32_T *a,
                     emxArray_uint32_T *b)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_int32_T *idx;
  emxArray_int32_T *iwork;
  int32_T b_iv[2];
  int32_T k;
  int32_T n;
  int32_T na;
  int32_T p;
  int32_T pEnd;
  int32_T q;
  int32_T qEnd;
  int32_T *idx_data;
  int32_T *iwork_data;
  const uint32_T *a_data;
  uint32_T *b_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  a_data = a->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  na = a->size[0];
  st.site = &jb_emlrtRSI;
  n = a->size[0] + 1;
  emxInit_int32_T(&st, &idx, 1, &mc_emlrtRTEI);
  pEnd = idx->size[0];
  idx->size[0] = a->size[0];
  emxEnsureCapacity_int32_T(&st, idx, pEnd, &mc_emlrtRTEI);
  idx_data = idx->data;
  for (k = 0; k < na; k++) {
    idx_data[k] = 0;
  }
  if (a->size[0] != 0) {
    int32_T i;
    b_st.site = &qb_emlrtRSI;
    emxInit_int32_T(&b_st, &iwork, 1, &qc_emlrtRTEI);
    pEnd = iwork->size[0];
    iwork->size[0] = a->size[0];
    emxEnsureCapacity_int32_T(&b_st, iwork, pEnd, &nc_emlrtRTEI);
    iwork_data = iwork->data;
    pEnd = a->size[0] - 1;
    c_st.site = &rb_emlrtRSI;
    if (a->size[0] - 1 > 2147483645) {
      d_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    for (k = 1; k <= pEnd; k += 2) {
      if (a_data[k - 1] <= a_data[k]) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if (((uint32_T)a->size[0] & 1U) != 0U) {
      idx_data[a->size[0] - 1] = a->size[0];
    }
    i = 2;
    while (i < n - 1) {
      int32_T i2;
      int32_T j;
      i2 = i << 1;
      j = 1;
      for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
        int32_T b_k;
        int32_T kEnd;
        p = j;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        b_k = 0;
        kEnd = qEnd - j;
        while (b_k < kEnd) {
          int32_T b_i;
          int32_T i1;
          b_i = idx_data[q - 1];
          i1 = idx_data[p - 1];
          if (a_data[i1 - 1] <= a_data[b_i - 1]) {
            iwork_data[b_k] = i1;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[b_k] = b_i;
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[p - 1];
                p++;
              }
            }
          }
          b_k++;
        }
        c_st.site = &sb_emlrtRSI;
        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      i = i2;
    }
    emxFree_int32_T(&b_st, &iwork);
  }
  pEnd = b->size[0];
  b->size[0] = a->size[0];
  emxEnsureCapacity_uint32_T(sp, b, pEnd, &oc_emlrtRTEI);
  b_data = b->data;
  st.site = &kb_emlrtRSI;
  if (a->size[0] > 2147483646) {
    b_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (k = 0; k < na; k++) {
    b_data[k] = a_data[idx_data[k] - 1];
  }
  emxFree_int32_T(sp, &idx);
  q = 0;
  pEnd = 0;
  while (pEnd + 1 <= na) {
    uint32_T x;
    x = b_data[pEnd];
    p = pEnd;
    do {
      pEnd++;
    } while (!((pEnd + 1 > na) || (b_data[pEnd] != x)));
    q++;
    b_data[q - 1] = x;
    st.site = &mb_emlrtRSI;
    if ((p + 1 <= pEnd) && (pEnd > 2147483646)) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
  }
  st.site = &ob_emlrtRSI;
  if (q > a->size[0]) {
    emlrtErrorWithMessageIdR2018a(sp, &e_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (q < 1) {
    q = 0;
  }
  b_iv[0] = 1;
  b_iv[1] = q;
  st.site = &jo_emlrtRSI;
  indexShapeCheck(&st, b->size[0], b_iv);
  pEnd = b->size[0];
  b->size[0] = q;
  emxEnsureCapacity_uint32_T(sp, b, pEnd, &pc_emlrtRTEI);
  st.site = &pb_emlrtRSI;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_unique_vector(const emlrtStack *sp, const emxArray_uint32_T *a,
                     emxArray_uint32_T *b)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_int32_T *idx;
  emxArray_int32_T *iwork;
  int32_T k;
  int32_T n;
  int32_T na;
  int32_T p;
  int32_T pEnd;
  int32_T q;
  int32_T qEnd;
  int32_T *idx_data;
  int32_T *iwork_data;
  const uint32_T *a_data;
  uint32_T *b_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  a_data = a->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  na = a->size[1];
  st.site = &jb_emlrtRSI;
  n = a->size[1] + 1;
  emxInit_int32_T(&st, &idx, 2, &mc_emlrtRTEI);
  pEnd = idx->size[0] * idx->size[1];
  idx->size[0] = 1;
  idx->size[1] = a->size[1];
  emxEnsureCapacity_int32_T(&st, idx, pEnd, &mc_emlrtRTEI);
  idx_data = idx->data;
  for (k = 0; k < na; k++) {
    idx_data[k] = 0;
  }
  if (a->size[1] != 0) {
    int32_T i;
    b_st.site = &qb_emlrtRSI;
    emxInit_int32_T(&b_st, &iwork, 1, &qc_emlrtRTEI);
    pEnd = iwork->size[0];
    iwork->size[0] = a->size[1];
    emxEnsureCapacity_int32_T(&b_st, iwork, pEnd, &nc_emlrtRTEI);
    iwork_data = iwork->data;
    pEnd = a->size[1] - 1;
    c_st.site = &rb_emlrtRSI;
    if (a->size[1] - 1 > 2147483645) {
      d_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    for (k = 1; k <= pEnd; k += 2) {
      if (a_data[k - 1] <= a_data[k]) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if (((uint32_T)a->size[1] & 1U) != 0U) {
      idx_data[a->size[1] - 1] = a->size[1];
    }
    i = 2;
    while (i < n - 1) {
      int32_T i2;
      int32_T j;
      i2 = i << 1;
      j = 1;
      for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
        int32_T b_k;
        int32_T kEnd;
        p = j;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        b_k = 0;
        kEnd = qEnd - j;
        while (b_k < kEnd) {
          int32_T b_i;
          int32_T i1;
          b_i = idx_data[q - 1];
          i1 = idx_data[p - 1];
          if (a_data[i1 - 1] <= a_data[b_i - 1]) {
            iwork_data[b_k] = i1;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[b_k] = b_i;
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[p - 1];
                p++;
              }
            }
          }
          b_k++;
        }
        c_st.site = &sb_emlrtRSI;
        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      i = i2;
    }
    emxFree_int32_T(&b_st, &iwork);
  }
  pEnd = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = a->size[1];
  emxEnsureCapacity_uint32_T(sp, b, pEnd, &oc_emlrtRTEI);
  b_data = b->data;
  st.site = &kb_emlrtRSI;
  if (a->size[1] > 2147483646) {
    b_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (k = 0; k < na; k++) {
    b_data[k] = a_data[idx_data[k] - 1];
  }
  emxFree_int32_T(sp, &idx);
  q = 0;
  pEnd = 0;
  while (pEnd + 1 <= na) {
    uint32_T x;
    x = b_data[pEnd];
    p = pEnd;
    do {
      pEnd++;
    } while (!((pEnd + 1 > na) || (b_data[pEnd] != x)));
    q++;
    b_data[q - 1] = x;
    st.site = &mb_emlrtRSI;
    if ((p + 1 <= pEnd) && (pEnd > 2147483646)) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
  }
  st.site = &ob_emlrtRSI;
  if (q > a->size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &e_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  pEnd = b->size[0] * b->size[1];
  if (q < 1) {
    b->size[1] = 0;
  } else {
    b->size[1] = q;
  }
  emxEnsureCapacity_uint32_T(sp, b, pEnd, &pc_emlrtRTEI);
  st.site = &pb_emlrtRSI;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void unique_vector(const emlrtStack *sp, const emxArray_real_T *a,
                   emxArray_real_T *b)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_int32_T *idx;
  emxArray_int32_T *iwork;
  const real_T *a_data;
  real_T x;
  real_T *b_data;
  int32_T b_k;
  int32_T i;
  int32_T k;
  int32_T n;
  int32_T nNaN;
  int32_T na;
  int32_T p;
  int32_T pEnd;
  int32_T q;
  int32_T qEnd;
  int32_T *idx_data;
  int32_T *iwork_data;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  a_data = a->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  na = a->size[1];
  st.site = &jb_emlrtRSI;
  n = a->size[1] + 1;
  emxInit_int32_T(&st, &idx, 2, &mc_emlrtRTEI);
  pEnd = idx->size[0] * idx->size[1];
  idx->size[0] = 1;
  idx->size[1] = a->size[1];
  emxEnsureCapacity_int32_T(&st, idx, pEnd, &mc_emlrtRTEI);
  idx_data = idx->data;
  for (k = 0; k < na; k++) {
    idx_data[k] = 0;
  }
  if (a->size[1] != 0) {
    b_st.site = &qb_emlrtRSI;
    emxInit_int32_T(&b_st, &iwork, 1, &qc_emlrtRTEI);
    pEnd = iwork->size[0];
    iwork->size[0] = a->size[1];
    emxEnsureCapacity_int32_T(&b_st, iwork, pEnd, &nc_emlrtRTEI);
    iwork_data = iwork->data;
    pEnd = a->size[1] - 1;
    c_st.site = &rb_emlrtRSI;
    if (a->size[1] - 1 > 2147483645) {
      d_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    for (k = 1; k <= pEnd; k += 2) {
      x = a_data[k];
      if ((a_data[k - 1] <= x) || muDoubleScalarIsNaN(x)) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if (((uint32_T)a->size[1] & 1U) != 0U) {
      idx_data[a->size[1] - 1] = a->size[1];
    }
    i = 2;
    while (i < n - 1) {
      int32_T i2;
      int32_T j;
      i2 = i << 1;
      j = 1;
      for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
        int32_T kEnd;
        p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        b_k = 0;
        kEnd = qEnd - j;
        while (b_k < kEnd) {
          x = a_data[idx_data[q] - 1];
          nNaN = idx_data[p - 1];
          if ((a_data[nNaN - 1] <= x) || muDoubleScalarIsNaN(x)) {
            iwork_data[b_k] = nNaN;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[b_k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                b_k++;
                iwork_data[b_k] = idx_data[p - 1];
                p++;
              }
            }
          }
          b_k++;
        }
        c_st.site = &sb_emlrtRSI;
        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      i = i2;
    }
    emxFree_int32_T(&b_st, &iwork);
  }
  pEnd = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = a->size[1];
  emxEnsureCapacity_real_T(sp, b, pEnd, &oc_emlrtRTEI);
  b_data = b->data;
  st.site = &kb_emlrtRSI;
  if (a->size[1] > 2147483646) {
    b_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (k = 0; k < na; k++) {
    b_data[k] = a_data[idx_data[k] - 1];
  }
  emxFree_int32_T(sp, &idx);
  q = 0;
  while ((q + 1 <= na) && muDoubleScalarIsInf(b_data[q]) && (b_data[q] < 0.0)) {
    q++;
  }
  b_k = q;
  q = a->size[1];
  while ((q >= 1) && muDoubleScalarIsNaN(b_data[q - 1])) {
    q--;
  }
  nNaN = a->size[1] - q;
  exitg1 = false;
  while ((!exitg1) && (q >= 1)) {
    x = b_data[q - 1];
    if (muDoubleScalarIsInf(x) && (x > 0.0)) {
      q--;
    } else {
      exitg1 = true;
    }
  }
  pEnd = (a->size[1] - q) - nNaN;
  i = 0;
  if (b_k > 0) {
    i = 1;
    st.site = &lb_emlrtRSI;
    if (b_k > 2147483646) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
  }
  while (b_k + 1 <= q) {
    x = b_data[b_k];
    p = b_k;
    do {
      b_k++;
    } while (!((b_k + 1 > q) || (b_data[b_k] != x)));
    i++;
    b_data[i - 1] = x;
    st.site = &mb_emlrtRSI;
    if ((p + 1 <= b_k) && (b_k > 2147483646)) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
  }
  if (pEnd > 0) {
    i++;
    b_data[i - 1] = b_data[q];
    st.site = &nb_emlrtRSI;
    if (pEnd > 2147483646) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
  }
  b_k = q + pEnd;
  st.site = &ob_emlrtRSI;
  for (k = 0; k < nNaN; k++) {
    b_data[i + k] = b_data[b_k + k];
  }
  if (nNaN - 1 >= 0) {
    i += nNaN;
  }
  if (i > a->size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &e_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  pEnd = b->size[0] * b->size[1];
  if (i < 1) {
    b->size[1] = 0;
  } else {
    b->size[1] = i;
  }
  emxEnsureCapacity_real_T(sp, b, pEnd, &pc_emlrtRTEI);
  st.site = &pb_emlrtRSI;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (unique.c) */
