/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "xzgetrf.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dt_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo et_emlrtRSI = {
    44,      /* lineNo */
    "mldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo ft_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo
    gt_emlrtRSI =
        {
            109,          /* lineNo */
            "lusolveNxN", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    ht_emlrtRSI =
        {
            124,          /* lineNo */
            "InvAtimesX", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo it_emlrtRSI = {
    19,        /* lineNo */
    "xgetrfs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo jt_emlrtRSI = {
    108,      /* lineNo */
    "cmldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo kt_emlrtRSI = {
    70,       /* lineNo */
    "cmldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo
    nt_emlrtRSI =
        {
            61,        /* lineNo */
            "qrsolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    ot_emlrtRSI =
        {
            72,        /* lineNo */
            "qrsolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    pt_emlrtRSI =
        {
            85,        /* lineNo */
            "qrsolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo qt_emlrtRSI = {
    63,       /* lineNo */
    "xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo rt_emlrtRSI = {
    158,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo st_emlrtRSI = {
    154,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo tt_emlrtRSI = {
    151,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo ut_emlrtRSI = {
    148,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo vt_emlrtRSI = {
    143,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo wt_emlrtRSI = {
    141,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo xt_emlrtRSI = {
    138,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo
    yt_emlrtRSI =
        {
            172,          /* lineNo */
            "rankFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    au_emlrtRSI =
        {
            173,          /* lineNo */
            "rankFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    bu_emlrtRSI =
        {
            119,         /* lineNo */
            "LSQFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    cu_emlrtRSI =
        {
            128,         /* lineNo */
            "LSQFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    du_emlrtRSI =
        {
            138,         /* lineNo */
            "LSQFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo eu_emlrtRSI = {
    40,         /* lineNo */
    "xunormqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xunormqr.m" /* pathName */
};

static emlrtRSInfo fu_emlrtRSI = {
    106,              /* lineNo */
    "ceval_xunormqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xunormqr.m" /* pathName */
};

static emlrtRSInfo gu_emlrtRSI = {
    26,        /* lineNo */
    "xgetrfs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo hu_emlrtRSI = {
    27,        /* lineNo */
    "xgetrfs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtMCInfo c_emlrtMCI = {
    53,        /* lineNo */
    19,        /* colNo */
    "flt2str", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\flt2str.m" /* pName */
};

static emlrtRTEInfo yb_emlrtRTEI = {
    16,         /* lineNo */
    19,         /* colNo */
    "mldivide", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pName
                                                                         */
};

static emlrtRTEInfo eg_emlrtRTEI = {
    20,         /* lineNo */
    5,          /* colNo */
    "mldivide", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pName
                                                                         */
};

static emlrtRTEInfo fg_emlrtRTEI = {
    1,        /* lineNo */
    32,       /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo gg_emlrtRTEI = {
    61,       /* lineNo */
    9,        /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo hg_emlrtRTEI = {
    48,        /* lineNo */
    37,        /* colNo */
    "xgetrfs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pName */
};

static emlrtRTEInfo ig_emlrtRTEI = {
    92,       /* lineNo */
    22,       /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo jg_emlrtRTEI = {
    105,      /* lineNo */
    1,        /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo kg_emlrtRTEI = {
    70,        /* lineNo */
    23,        /* colNo */
    "xgetrfs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pName */
};

static emlrtRTEInfo
    lg_emlrtRTEI =
        {
            85,        /* lineNo */
            26,        /* colNo */
            "qrsolve", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pName */
};

static emlrtRTEInfo
    mg_emlrtRTEI =
        {
            119,       /* lineNo */
            5,         /* colNo */
            "qrsolve", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pName */
};

static emlrtRTEInfo ng_emlrtRTEI = {
    1,          /* lineNo */
    14,         /* colNo */
    "mldivide", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pName
                                                                         */
};

static emlrtRSInfo hw_emlrtRSI = {
    53,        /* lineNo */
    "flt2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\flt2str.m" /* pathName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y[14]);

static const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *m,
                                const mxArray *m1, emlrtMCInfo *location);

static void emlrt_marshallIn(const emlrtStack *sp,
                             const mxArray *a__output_of_sprintf_,
                             const char_T *identifier, char_T y[14]);

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[14]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[14])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *m,
                                const mxArray *m1, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m2;
  pArrays[0] = m;
  pArrays[1] = m1;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m2, 2, &pArrays[0],
                               "sprintf", true, location);
}

static void emlrt_marshallIn(const emlrtStack *sp,
                             const mxArray *a__output_of_sprintf_,
                             const char_T *identifier, char_T y[14])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[14])
{
  static const int32_T dims[2] = {1, 14};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 14);
  emlrtDestroyArray(&src);
}

void b_mldivide(const emlrtStack *sp, const real_T A[36], real_T Y[36])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T b_A[36];
  int32_T ipiv[6];
  int32_T b_i;
  int32_T b_kAcol;
  int32_T i;
  int32_T info;
  int32_T j;
  int32_T jBcol;
  int32_T kAcol;
  int32_T temp;
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
  st.site = &dt_emlrtRSI;
  b_st.site = &ft_emlrtRSI;
  c_st.site = &wk_emlrtRSI;
  d_st.site = &gt_emlrtRSI;
  e_st.site = &ht_emlrtRSI;
  f_st.site = &gu_emlrtRSI;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  g_st.site = &cl_emlrtRSI;
  info = xzgetrf(&g_st, b_A, ipiv);
  f_st.site = &hu_emlrtRSI;
  for (i = 0; i < 36; i++) {
    Y[i] = iv[i];
  }
  for (i = 0; i < 5; i++) {
    jBcol = ipiv[i];
    if (jBcol != i + 1) {
      for (j = 0; j < 6; j++) {
        kAcol = i + 6 * j;
        temp = (int32_T)Y[kAcol];
        b_kAcol = (jBcol + 6 * j) - 1;
        Y[kAcol] = Y[b_kAcol];
        Y[b_kAcol] = temp;
      }
    }
  }
  for (i = 0; i < 6; i++) {
    temp = 6 * i;
    for (j = 0; j < 6; j++) {
      b_kAcol = 6 * j;
      jBcol = j + temp;
      if (Y[jBcol] != 0.0) {
        kAcol = j + 2;
        for (b_i = kAcol; b_i < 7; b_i++) {
          int32_T Y_tmp;
          Y_tmp = (b_i + temp) - 1;
          Y[Y_tmp] -= Y[jBcol] * b_A[(b_i + b_kAcol) - 1];
        }
      }
    }
  }
  for (i = 0; i < 6; i++) {
    jBcol = 6 * i;
    for (j = 5; j >= 0; j--) {
      real_T d;
      kAcol = 6 * j;
      temp = j + jBcol;
      d = Y[temp];
      if (d != 0.0) {
        Y[temp] = d / b_A[j + kAcol];
        for (b_i = 0; b_i < j; b_i++) {
          b_kAcol = b_i + jBcol;
          Y[b_kAcol] -= Y[temp] * b_A[b_i + kAcol];
        }
      }
    }
  }
  if (info > 0) {
    d_st.site = &yk_emlrtRSI;
    if (!emlrtSetWarningFlag(&d_st)) {
      e_st.site = &jl_emlrtRSI;
      c_warning(&e_st);
    }
  }
}

void mldivide(const emlrtStack *sp, const emxArray_real_T *A,
              const emxArray_real_T *B, emxArray_real_T *Y)
{
  static const int32_T b_iv[2] = {1, 6};
  static const char_T b_fname[19] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                     '_', 'd', 'g', 'e', 't', 'r', 'f',
                                     '_', 'w', 'o', 'r', 'k'};
  static const char_T c_fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                     '_', 'd', 'o', 'r', 'm', 'q', 'r'};
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'g', 'e', 'q', 'p', '3'};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  ptrdiff_t *jpvt_t_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_int32_T *jpvt;
  emxArray_ptrdiff_t *jpvt_t;
  emxArray_real_T *b_A;
  emxArray_real_T *b_B;
  emxArray_real_T *tau;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  const real_T *A_data;
  const real_T *B_data;
  real_T *Y_data;
  real_T *b_A_data;
  real_T *b_B_data;
  real_T *tau_data;
  int32_T i;
  int32_T j;
  int32_T *jpvt_data;
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
  B_data = B->data;
  A_data = A->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (B->size[0] != A->size[0]) {
    emlrtErrorWithMessageIdR2018a(sp, &yb_emlrtRTEI, "MATLAB:dimagree",
                                  "MATLAB:dimagree", 0);
  }
  st.site = &dt_emlrtRSI;
  emxInit_real_T(&st, &b_A, 2, &ng_emlrtRTEI);
  emxInit_ptrdiff_t(&st, &jpvt_t, &jg_emlrtRTEI);
  if (A->size[0] == A->size[1]) {
    ptrdiff_t INFO;
    ptrdiff_t LDA;
    ptrdiff_t N;
    int32_T ma;
    int32_T na;
    int32_T rankA;
    b_st.site = &ft_emlrtRSI;
    c_st.site = &wk_emlrtRSI;
    rankA = B->size[0];
    ma = Y->size[0];
    Y->size[0] = B->size[0];
    emxEnsureCapacity_real_T(&c_st, Y, ma, &eg_emlrtRTEI);
    Y_data = Y->data;
    for (j = 0; j < rankA; j++) {
      Y_data[j] = B_data[j];
    }
    d_st.site = &gt_emlrtRSI;
    e_st.site = &ht_emlrtRSI;
    ma = A->size[0];
    na = A->size[1];
    ma = muIntScalarMin_sint32(ma, na);
    na = muIntScalarMin_sint32(rankA, ma);
    f_st.site = &it_emlrtRSI;
    ma = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(&f_st, b_A, ma, &hg_emlrtRTEI);
    b_A_data = b_A->data;
    ma = A->size[0] * A->size[1];
    for (j = 0; j < ma; j++) {
      b_A_data[j] = A_data[j];
    }
    g_st.site = &kt_emlrtRSI;
    ma = jpvt_t->size[0];
    jpvt_t->size[0] = na;
    emxEnsureCapacity_ptrdiff_t(&g_st, jpvt_t, ma, &kg_emlrtRTEI);
    jpvt_t_data = jpvt_t->data;
    N = (ptrdiff_t)na;
    LDA = (ptrdiff_t)b_A->size[0];
    INFO = LAPACKE_dgetrf_work(102, N, N, &b_A_data[0], LDA, &jpvt_t_data[0]);
    g_st.site = &jt_emlrtRSI;
    if ((int32_T)INFO < 0) {
      if ((int32_T)INFO == -1010) {
        emlrtErrorWithMessageIdR2018a(&g_st, &p_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&g_st, &q_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      19, &b_fname[0], 12, (int32_T)INFO);
      }
    }
    LAPACKE_dgetrs_work(102, 'N', N, (ptrdiff_t)1, &b_A_data[0], LDA,
                        &jpvt_t_data[0], &Y_data[0], (ptrdiff_t)B->size[0]);
    if ((int32_T)INFO > 0) {
      d_st.site = &yk_emlrtRSI;
      if (!emlrtSetWarningFlag(&d_st)) {
        e_st.site = &jl_emlrtRSI;
        c_warning(&e_st);
      }
    }
  } else {
    ptrdiff_t N;
    real_T tol;
    int32_T loop_ub;
    int32_T ma;
    int32_T na;
    int32_T rankA;
    boolean_T overflow;
    b_st.site = &et_emlrtRSI;
    c_st.site = &nt_emlrtRSI;
    rankA = A->size[0];
    ma = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    loop_ub = A->size[1];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(&c_st, b_A, ma, &fg_emlrtRTEI);
    b_A_data = b_A->data;
    ma = A->size[0] * A->size[1];
    for (j = 0; j < ma; j++) {
      b_A_data[j] = A_data[j];
    }
    emxInit_int32_T(&c_st, &jpvt, 2, &ng_emlrtRTEI);
    ma = jpvt->size[0] * jpvt->size[1];
    jpvt->size[0] = 1;
    jpvt->size[1] = A->size[1];
    emxEnsureCapacity_int32_T(&c_st, jpvt, ma, &gg_emlrtRTEI);
    jpvt_data = jpvt->data;
    for (j = 0; j < loop_ub; j++) {
      jpvt_data[j] = 0;
    }
    d_st.site = &qt_emlrtRSI;
    na = muIntScalarMin_sint32(rankA, loop_ub);
    emxInit_real_T(&d_st, &tau, 1, &ng_emlrtRTEI);
    ma = tau->size[0];
    tau->size[0] = na;
    emxEnsureCapacity_real_T(&d_st, tau, ma, &ig_emlrtRTEI);
    tau_data = tau->data;
    ma = jpvt_t->size[0];
    jpvt_t->size[0] = A->size[1];
    emxEnsureCapacity_ptrdiff_t(&d_st, jpvt_t, ma, &jg_emlrtRTEI);
    jpvt_t_data = jpvt_t->data;
    for (j = 0; j < loop_ub; j++) {
      jpvt_t_data[j] = (ptrdiff_t)0;
    }
    N = LAPACKE_dgeqp3(102, (ptrdiff_t)b_A->size[0], (ptrdiff_t)b_A->size[1],
                       &b_A_data[0], (ptrdiff_t)b_A->size[0], &jpvt_t_data[0],
                       &tau_data[0]);
    e_st.site = &xt_emlrtRSI;
    if ((int32_T)N != 0) {
      overflow = true;
      if ((int32_T)N != -4) {
        if ((int32_T)N == -1010) {
          emlrtErrorWithMessageIdR2018a(&e_st, &p_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &fname[0], 12, (int32_T)N);
        }
      }
    } else {
      overflow = false;
    }
    if (overflow) {
      e_st.site = &wt_emlrtRSI;
      if (b_A->size[1] > 2147483646) {
        f_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&f_st);
      }
      overflow = (b_A->size[0] > 2147483646);
      for (j = 0; j < loop_ub; j++) {
        e_st.site = &vt_emlrtRSI;
        if (overflow) {
          f_st.site = &tb_emlrtRSI;
          check_forloop_overflow_error(&f_st);
        }
        for (i = 0; i < rankA; i++) {
          b_A_data[j * rankA + i] = rtNaN;
        }
      }
      e_st.site = &ut_emlrtRSI;
      if (na > 2147483646) {
        f_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&f_st);
      }
      for (j = 0; j < na; j++) {
        tau_data[j] = rtNaN;
      }
      ma = na + 1;
      e_st.site = &tt_emlrtRSI;
      for (j = ma; j <= na; j++) {
        tau_data[j - 1] = 0.0;
      }
      e_st.site = &st_emlrtRSI;
      ma = (b_A->size[1] / 4) << 2;
      na = ma - 4;
      for (j = 0; j <= na; j += 4) {
        _mm_storeu_si128(
            (__m128i *)&jpvt_data[j],
            _mm_add_epi32(
                _mm_add_epi32(_mm_set1_epi32(j),
                              _mm_loadu_si128((const __m128i *)&iv1[0])),
                _mm_set1_epi32(1)));
      }
      for (j = ma; j < loop_ub; j++) {
        jpvt_data[j] = j + 1;
      }
    } else {
      e_st.site = &rt_emlrtRSI;
      if (b_A->size[1] > 2147483646) {
        f_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&f_st);
      }
      for (j = 0; j < loop_ub; j++) {
        jpvt_data[j] = (int32_T)jpvt_t_data[j];
      }
    }
    c_st.site = &ot_emlrtRSI;
    rankA = 0;
    if (b_A->size[0] < b_A->size[1]) {
      ma = b_A->size[0];
      na = b_A->size[1];
    } else {
      ma = b_A->size[1];
      na = b_A->size[0];
    }
    tol = muDoubleScalarMin(1.4901161193847656E-8,
                            2.2204460492503131E-15 * (real_T)na) *
          muDoubleScalarAbs(b_A_data[0]);
    while (
        (rankA < ma) &&
        (!(muDoubleScalarAbs(b_A_data[rankA + b_A->size[0] * rankA]) <= tol))) {
      rankA++;
    }
    if ((rankA < ma) && (!emlrtSetWarningFlag(&c_st))) {
      char_T str[14];
      d_st.site = &au_emlrtRSI;
      y = NULL;
      m = emlrtCreateCharArray(2, &b_iv[0]);
      emlrtInitCharArrayR2013a(&d_st, 6, m, &rfmt[0]);
      emlrtAssign(&y, m);
      b_y = NULL;
      m = emlrtCreateDoubleScalar(tol);
      emlrtAssign(&b_y, m);
      e_st.site = &hw_emlrtRSI;
      emlrt_marshallIn(&e_st, b_sprintf(&e_st, y, b_y, &c_emlrtMCI),
                       "<output of sprintf>", str);
      d_st.site = &yt_emlrtRSI;
      d_warning(&d_st, rankA, str);
    }
    c_st.site = &pt_emlrtRSI;
    emxInit_real_T(&c_st, &b_B, 1, &lg_emlrtRTEI);
    na = B->size[0];
    ma = b_B->size[0];
    b_B->size[0] = B->size[0];
    emxEnsureCapacity_real_T(&c_st, b_B, ma, &lg_emlrtRTEI);
    b_B_data = b_B->data;
    for (j = 0; j < na; j++) {
      b_B_data[j] = B_data[j];
    }
    na = b_A->size[1];
    ma = Y->size[0];
    Y->size[0] = b_A->size[1];
    emxEnsureCapacity_real_T(&c_st, Y, ma, &eg_emlrtRTEI);
    Y_data = Y->data;
    for (j = 0; j < na; j++) {
      Y_data[j] = 0.0;
    }
    d_st.site = &bu_emlrtRSI;
    e_st.site = &eu_emlrtRSI;
    N = (ptrdiff_t)b_B->size[0];
    N = LAPACKE_dormqr(
        102, 'L', 'T', N, (ptrdiff_t)1,
        (ptrdiff_t)muIntScalarMin_sint32(b_A->size[0], b_A->size[1]),
        &b_A_data[0], (ptrdiff_t)b_A->size[0], &tau_data[0], &b_B_data[0], N);
    emxFree_real_T(&e_st, &tau);
    f_st.site = &fu_emlrtRSI;
    if ((int32_T)N != 0) {
      boolean_T p;
      overflow = true;
      p = false;
      if ((int32_T)N == -7) {
        p = true;
      } else if ((int32_T)N == -9) {
        p = true;
      } else if ((int32_T)N == -10) {
        p = true;
      }
      if (!p) {
        if ((int32_T)N == -1010) {
          emlrtErrorWithMessageIdR2018a(&f_st, &p_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&f_st, &q_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &c_fname[0], 12, (int32_T)N);
        }
      }
    } else {
      overflow = false;
    }
    if (overflow) {
      na = b_B->size[0];
      ma = b_B->size[0];
      b_B->size[0] = na;
      emxEnsureCapacity_real_T(&e_st, b_B, ma, &mg_emlrtRTEI);
      b_B_data = b_B->data;
      for (j = 0; j < na; j++) {
        b_B_data[j] = rtNaN;
      }
    }
    d_st.site = &cu_emlrtRSI;
    if (rankA > 2147483646) {
      e_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    for (j = 0; j < rankA; j++) {
      Y_data[jpvt_data[j] - 1] = b_B_data[j];
    }
    emxFree_real_T(&c_st, &b_B);
    for (j = rankA; j >= 1; j--) {
      ma = jpvt_data[j - 1];
      Y_data[ma - 1] /= b_A_data[(j + b_A->size[0] * (j - 1)) - 1];
      d_st.site = &du_emlrtRSI;
      for (i = 0; i <= j - 2; i++) {
        Y_data[jpvt_data[i] - 1] -=
            Y_data[ma - 1] * b_A_data[i + b_A->size[0] * (j - 1)];
      }
    }
    emxFree_int32_T(&c_st, &jpvt);
  }
  emxFree_ptrdiff_t(&st, &jpvt_t);
  emxFree_real_T(&st, &b_A);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (mldivide.c) */
