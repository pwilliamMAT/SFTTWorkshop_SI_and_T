/*
 * gaussEKFilter.c
 *
 * Code generation for function 'gaussEKFilter'
 *
 */

/* Include files */
#include "gaussEKFilter.h"
#include "constvel.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "validateattributes.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ej_emlrtRSI = {
    19,                      /* lineNo */
    "gaussEKFilter/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m" /* pathName */
};

static emlrtECInfo i_emlrtECI =
    {
        -1,         /* nDims */
        166,        /* lineNo */
        5,          /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

static emlrtECInfo j_emlrtECI =
    {
        2,          /* nDims */
        166,        /* lineNo */
        23,         /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

static emlrtECInfo k_emlrtECI =
    {
        -1,         /* nDims */
        165,        /* lineNo */
        5,          /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

static emlrtECInfo l_emlrtECI =
    {
        2,          /* nDims */
        165,        /* lineNo */
        25,         /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

static emlrtRTEInfo jc_emlrtRTEI =
    {
        150,        /* lineNo */
        27,         /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

static emlrtBCInfo si_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    31,                      /* lineNo */
    84,                      /* colNo */
    "",                      /* aName */
    "gaussEKFilter/predict", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ti_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    30,                      /* lineNo */
    84,                      /* colNo */
    "",                      /* aName */
    "gaussEKFilter/predict", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ui_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    20,                      /* lineNo */
    28,                      /* colNo */
    "",                      /* aName */
    "gaussEKFilter/predict", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo vi_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    37,                      /* lineNo */
    23,                      /* colNo */
    "",                      /* aName */
    "gaussEKFilter/predict", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo oj_emlrtRTEI = {
    17,              /* lineNo */
    17,              /* colNo */
    "gaussEKFilter", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m" /* pName */
};

static emlrtRSInfo yu_emlrtRSI =
    {
        166,        /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

/* Function Definitions */
void b_gaussEKFilter_predict(const emlrtStack *sp, const real_T x[6],
                             real_T P[36], const real_T Q[9], real_T varargin_1,
                             real_T xk[6])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T F[36];
  real_T b_F[36];
  real_T U[18];
  real_T b_U[18];
  real_T imvec[6];
  real_T z[6];
  real_T specvec_f2[3];
  real_T epsilon;
  real_T xk_tmp;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T i1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &xg_emlrtRSI;
  for (i = 0; i < 6; i++) {
    xk[i] = x[i];
  }
  b_st.site = &bh_emlrtRSI;
  d_validateattributes(&b_st, x);
  b_st.site = &fh_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_1) || muDoubleScalarIsNaN(varargin_1)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constvel:expectedFinite", 3, 4, 19, "input number 3, dt,");
  }
  b_st.site = &gh_emlrtRSI;
  xk_tmp = 0.5 * (varargin_1 * varargin_1) * 0.0;
  xk[0] = (xk[0] + xk[1] * varargin_1) + xk_tmp;
  epsilon = 0.0 * varargin_1;
  xk[1] += epsilon;
  b_st.site = &gh_emlrtRSI;
  xk[2] = (xk[2] + xk[3] * varargin_1) + xk_tmp;
  xk[3] += epsilon;
  b_st.site = &gh_emlrtRSI;
  xk[4] = (xk[4] + xk[5] * varargin_1) + xk_tmp;
  xk[5] += epsilon;
  specvec_f2[0] = 0.0;
  specvec_f2[1] = 0.0;
  specvec_f2[2] = 0.0;
  st.site = &yg_emlrtRSI;
  for (i = 0; i < 6; i++) {
    z[i] = x[i];
  }
  b_st.site = &ch_emlrtRSI;
  constvel(&b_st, z, specvec_f2, varargin_1);
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      imvec[b_i] = x[b_i];
    }
    xk_tmp = x[i];
    epsilon =
        muDoubleScalarMax(1.4901161193847656E-8,
                          1.4901161193847656E-8 * muDoubleScalarAbs(xk_tmp));
    imvec[i] = xk_tmp + epsilon;
    b_st.site = &dh_emlrtRSI;
    constvel(&b_st, imvec, specvec_f2, varargin_1);
    r = _mm_loadu_pd(&imvec[0]);
    r1 = _mm_loadu_pd(&z[0]);
    r2 = _mm_set1_pd(epsilon);
    _mm_storeu_pd(&F[6 * i], _mm_div_pd(_mm_sub_pd(r, r1), r2));
    r = _mm_loadu_pd(&imvec[2]);
    r1 = _mm_loadu_pd(&z[2]);
    _mm_storeu_pd(&F[6 * i + 2], _mm_div_pd(_mm_sub_pd(r, r1), r2));
    r = _mm_loadu_pd(&imvec[4]);
    r1 = _mm_loadu_pd(&z[4]);
    _mm_storeu_pd(&F[6 * i + 4], _mm_div_pd(_mm_sub_pd(r, r1), r2));
  }
  st.site = &ah_emlrtRSI;
  for (i = 0; i < 6; i++) {
    z[i] = x[i];
  }
  b_st.site = &ch_emlrtRSI;
  constvel(&b_st, z, specvec_f2, varargin_1);
  r = _mm_set1_pd(1.4901161193847656E-8);
  for (i = 0; i < 3; i++) {
    specvec_f2[0] = 0.0;
    specvec_f2[1] = 0.0;
    specvec_f2[2] = 0.0;
    specvec_f2[i] = 1.4901161193847656E-8;
    for (b_i = 0; b_i < 6; b_i++) {
      imvec[b_i] = x[b_i];
    }
    b_st.site = &dh_emlrtRSI;
    constvel(&b_st, imvec, specvec_f2, varargin_1);
    r1 = _mm_loadu_pd(&imvec[0]);
    r2 = _mm_loadu_pd(&z[0]);
    _mm_storeu_pd(&U[6 * i], _mm_div_pd(_mm_sub_pd(r1, r2), r));
    r1 = _mm_loadu_pd(&imvec[2]);
    r2 = _mm_loadu_pd(&z[2]);
    _mm_storeu_pd(&U[6 * i + 2], _mm_div_pd(_mm_sub_pd(r1, r2), r));
    r1 = _mm_loadu_pd(&imvec[4]);
    r2 = _mm_loadu_pd(&z[4]);
    _mm_storeu_pd(&U[6 * i + 4], _mm_div_pd(_mm_sub_pd(r1, r2), r));
  }
  memset(&b_F[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    c_i = 6 * i + 2;
    i1 = 6 * i + 4;
    for (b_i = 0; b_i < 6; b_i++) {
      r = _mm_loadu_pd(&F[6 * b_i]);
      r1 = _mm_loadu_pd(&b_F[6 * i]);
      r2 = _mm_set1_pd(P[b_i + 6 * i]);
      _mm_storeu_pd(&b_F[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&F[6 * b_i + 2]);
      r1 = _mm_loadu_pd(&b_F[c_i]);
      _mm_storeu_pd(&b_F[c_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&F[6 * b_i + 4]);
      r1 = _mm_loadu_pd(&b_F[i1]);
      _mm_storeu_pd(&b_F[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  memset(&b_U[0], 0, 18U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    c_i = 6 * i + 2;
    i1 = 6 * i + 4;
    for (b_i = 0; b_i < 3; b_i++) {
      r = _mm_loadu_pd(&U[6 * b_i]);
      r1 = _mm_loadu_pd(&b_U[6 * i]);
      r2 = _mm_set1_pd(Q[b_i + 3 * i]);
      _mm_storeu_pd(&b_U[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&U[6 * b_i + 2]);
      r1 = _mm_loadu_pd(&b_U[c_i]);
      _mm_storeu_pd(&b_U[c_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&U[6 * b_i + 4]);
      r1 = _mm_loadu_pd(&b_U[i1]);
      _mm_storeu_pd(&b_U[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  memset(&P[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    c_i = 6 * i + 2;
    i1 = 6 * i + 4;
    for (b_i = 0; b_i < 6; b_i++) {
      r = _mm_loadu_pd(&b_F[6 * b_i]);
      r1 = _mm_loadu_pd(&P[6 * i]);
      r2 = _mm_set1_pd(F[i + 6 * b_i]);
      _mm_storeu_pd(&P[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&b_F[6 * b_i + 2]);
      r1 = _mm_loadu_pd(&P[c_i]);
      _mm_storeu_pd(&P[c_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&b_F[6 * b_i + 4]);
      r1 = _mm_loadu_pd(&P[i1]);
      _mm_storeu_pd(&P[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  memset(&F[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    c_i = 6 * i + 2;
    i1 = 6 * i + 4;
    for (b_i = 0; b_i < 3; b_i++) {
      r = _mm_loadu_pd(&b_U[6 * b_i]);
      r1 = _mm_loadu_pd(&F[6 * i]);
      r2 = _mm_set1_pd(U[i + 6 * b_i]);
      _mm_storeu_pd(&F[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&b_U[6 * b_i + 2]);
      r1 = _mm_loadu_pd(&F[c_i]);
      _mm_storeu_pd(&F[c_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&b_U[6 * b_i + 4]);
      r1 = _mm_loadu_pd(&F[i1]);
      _mm_storeu_pd(&F[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&P[i]);
    r1 = _mm_loadu_pd(&F[i]);
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&P[i], r);
  }
}

void gaussEKFilter_predict(const emlrtStack *sp, const emxArray_real_T *x,
                           emxArray_real_T *P, const real_T Q[9],
                           real_T varargin_1, emxArray_real_T *xk)
{
  __m128d r4;
  __m128d r5;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  real_T F[36];
  real_T b_F[36];
  real_T c_F[36];
  real_T U[18];
  real_T b_U[18];
  real_T imvec[6];
  real_T z[6];
  real_T specvec_f2[3];
  const real_T *x_data;
  real_T a;
  real_T *P_data;
  real_T *r2;
  real_T *r3;
  real_T *xk_data;
  int32_T varargin_2[2];
  int32_T acoef;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T k;
  int32_T loop_ub;
  int32_T n;
  uint32_T b_varargin_1[2];
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  P_data = P->data;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  n = x->size[1];
  st.site = &xg_emlrtRSI;
  acoef = xk->size[0] * xk->size[1];
  xk->size[0] = 6;
  xk->size[1] = x->size[1];
  emxEnsureCapacity_real_T(&st, xk, acoef, &oj_emlrtRTEI);
  xk_data = xk->data;
  loop_ub = 6 * x->size[1];
  for (k = 0; k < loop_ub; k++) {
    xk_data[k] = x_data[k];
  }
  b_st.site = &bh_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  p = true;
  acoef = 0;
  exitg1 = false;
  while ((!exitg1) && (acoef <= loop_ub - 1)) {
    if ((!muDoubleScalarIsInf(x_data[acoef])) &&
        (!muDoubleScalarIsNaN(x_data[acoef]))) {
      acoef++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constvel:expectedFinite", 3, 4, 22, "input number 1, state,");
  }
  b_st.site = &eh_emlrtRSI;
  b_varargin_1[0] = 3U;
  b_varargin_1[1] = (uint32_T)x->size[1];
  varargin_2[0] = 3;
  varargin_2[1] = x->size[1];
  p = true;
  acoef = 0;
  exitg1 = false;
  while ((!exitg1) && (acoef < 2)) {
    if ((int32_T)b_varargin_1[acoef] != varargin_2[acoef]) {
      p = false;
      exitg1 = true;
    } else {
      acoef++;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(&st, &jc_emlrtRTEI,
                                  "shared_tracking:motion:incorrectNoiseDim",
                                  "shared_tracking:motion:incorrectNoiseDim", 7,
                                  4, 1, "w", 12, 3, 12, x->size[1]);
  }
  b_st.site = &fh_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_1) || muDoubleScalarIsNaN(varargin_1)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constvel:expectedFinite", 3, 4, 19, "input number 3, dt,");
  }
  a = 0.5 * (varargin_1 * varargin_1);
  emxInit_real_T(&st, &r, 2, &pj_emlrtRTEI);
  emxInit_real_T(&st, &r1, 2, &pj_emlrtRTEI);
  for (i = 0; i < 3; i++) {
    int32_T b_i;
    b_i = ((i + 1) << 1) - 1;
    acoef = r->size[0] * r->size[1];
    r->size[0] = 1;
    loop_ub = xk->size[1];
    r->size[1] = xk->size[1];
    emxEnsureCapacity_real_T(&st, r, acoef, &ej_emlrtRTEI);
    r2 = r->data;
    if (xk->size[1] != 0) {
      acoef = (xk->size[1] != 1);
      for (k = 0; k < loop_ub; k++) {
        r2[k] = xk_data[b_i + 6 * (acoef * k)] * varargin_1;
      }
    }
    if (xk->size[1] == r->size[1]) {
      acoef = r->size[0] * r->size[1];
      r->size[0] = 1;
      r->size[1] = xk->size[1];
      emxEnsureCapacity_real_T(&st, r, acoef, &pj_emlrtRTEI);
      r2 = r->data;
      for (k = 0; k < loop_ub; k++) {
        r2[k] += xk_data[(b_i + 6 * k) - 1];
      }
    } else {
      b_st.site = &gh_emlrtRSI;
      binary_expand_op_1(&b_st, r, xk, b_i);
    }
    b_st.site = &gh_emlrtRSI;
    acoef = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = n;
    emxEnsureCapacity_real_T(&st, r1, acoef, &ej_emlrtRTEI);
    r3 = r1->data;
    if (n != 0) {
      for (k = 0; k < n; k++) {
        r3[k] = a * 0.0;
      }
    }
    if ((r->size[1] != n) && ((r->size[1] != 1) && (n != 1))) {
      emlrtDimSizeImpxCheckR2021b(r->size[1], n, &l_emlrtECI, &st);
    }
    if (r->size[1] == r1->size[1]) {
      b_loop_ub = r->size[1] - 1;
      acoef = r->size[0] * r->size[1];
      r->size[0] = 1;
      emxEnsureCapacity_real_T(&st, r, acoef, &pj_emlrtRTEI);
      r2 = r->data;
      loop_ub = (r->size[1] / 2) << 1;
      acoef = loop_ub - 2;
      for (i1 = 0; i1 <= acoef; i1 += 2) {
        r4 = _mm_loadu_pd(&r2[i1]);
        r5 = _mm_loadu_pd(&r3[i1]);
        _mm_storeu_pd(&r2[i1], _mm_add_pd(r4, r5));
      }
      for (i1 = loop_ub; i1 <= b_loop_ub; i1++) {
        r2[i1] += r3[i1];
      }
    } else {
      b_st.site = &gh_emlrtRSI;
      plus(&b_st, r, r1);
      r2 = r->data;
    }
    varargin_2[0] = 1;
    loop_ub = xk->size[1];
    varargin_2[1] = xk->size[1];
    emlrtSubAssignSizeCheckR2012b(&varargin_2[0], 2, &r->size[0], 2,
                                  &k_emlrtECI, &st);
    for (k = 0; k < loop_ub; k++) {
      xk_data[(b_i + 6 * k) - 1] = r2[k];
    }
    acoef = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = n;
    emxEnsureCapacity_real_T(&st, r, acoef, &ej_emlrtRTEI);
    r2 = r->data;
    if (n != 0) {
      for (k = 0; k < n; k++) {
        r2[k] = 0.0 * varargin_1;
      }
    }
    if ((xk->size[1] != n) && ((xk->size[1] != 1) && (n != 1))) {
      emlrtDimSizeImpxCheckR2021b(xk->size[1], n, &j_emlrtECI, &st);
    }
    if (xk->size[1] == r->size[1]) {
      acoef = r->size[0] * r->size[1];
      r->size[0] = 1;
      r->size[1] = xk->size[1];
      emxEnsureCapacity_real_T(&st, r, acoef, &qj_emlrtRTEI);
      r2 = r->data;
      for (k = 0; k < loop_ub; k++) {
        r2[k] += xk_data[b_i + 6 * k];
      }
    } else {
      b_st.site = &yu_emlrtRSI;
      binary_expand_op(&b_st, r, xk, b_i);
      r2 = r->data;
    }
    varargin_2[0] = 1;
    acoef = xk->size[1];
    varargin_2[1] = xk->size[1];
    emlrtSubAssignSizeCheckR2012b(&varargin_2[0], 2, &r->size[0], 2,
                                  &i_emlrtECI, &st);
    for (k = 0; k < acoef; k++) {
      xk_data[b_i + 6 * k] = r2[k];
    }
  }
  emxFree_real_T(&st, &r1);
  emxFree_real_T(&st, &r);
  st.site = &ej_emlrtRSI;
  if (x->size[1] > 2147483646) {
    b_st.site = &tb_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (i = 0; i < n; i++) {
    __m128d r6;
    b_loop_ub = P->size[2];
    if (i + 1 > b_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &ui_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (i + 1 > n) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, n, &ti_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    st.site = &yg_emlrtRSI;
    for (k = 0; k < 6; k++) {
      z[k] = x_data[k + 6 * i];
    }
    specvec_f2[0] = 0.0;
    specvec_f2[1] = 0.0;
    specvec_f2[2] = 0.0;
    b_st.site = &ch_emlrtRSI;
    constvel(&b_st, z, specvec_f2, varargin_1);
    specvec_f2[0] = 0.0;
    specvec_f2[1] = 0.0;
    specvec_f2[2] = 0.0;
    for (k = 0; k < 6; k++) {
      for (i1 = 0; i1 < 6; i1++) {
        imvec[i1] = x_data[i1 + 6 * i];
      }
      real_T epsilon;
      a = x_data[k + 6 * i];
      epsilon = muDoubleScalarMax(1.4901161193847656E-8,
                                  1.4901161193847656E-8 * muDoubleScalarAbs(a));
      imvec[k] = a + epsilon;
      b_st.site = &dh_emlrtRSI;
      constvel(&b_st, imvec, specvec_f2, varargin_1);
      r4 = _mm_loadu_pd(&imvec[0]);
      r5 = _mm_loadu_pd(&z[0]);
      r6 = _mm_set1_pd(epsilon);
      _mm_storeu_pd(&F[6 * k], _mm_div_pd(_mm_sub_pd(r4, r5), r6));
      r4 = _mm_loadu_pd(&imvec[2]);
      r5 = _mm_loadu_pd(&z[2]);
      _mm_storeu_pd(&F[6 * k + 2], _mm_div_pd(_mm_sub_pd(r4, r5), r6));
      r4 = _mm_loadu_pd(&imvec[4]);
      r5 = _mm_loadu_pd(&z[4]);
      _mm_storeu_pd(&F[6 * k + 4], _mm_div_pd(_mm_sub_pd(r4, r5), r6));
    }
    if (i + 1 > n) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, n, &si_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    st.site = &ah_emlrtRSI;
    for (k = 0; k < 6; k++) {
      z[k] = x_data[k + 6 * i];
    }
    specvec_f2[0] = 0.0;
    specvec_f2[1] = 0.0;
    specvec_f2[2] = 0.0;
    b_st.site = &ch_emlrtRSI;
    constvel(&b_st, z, specvec_f2, varargin_1);
    r4 = _mm_set1_pd(1.4901161193847656E-8);
    for (k = 0; k < 3; k++) {
      specvec_f2[0] = 0.0;
      specvec_f2[1] = 0.0;
      specvec_f2[2] = 0.0;
      specvec_f2[k] = 1.4901161193847656E-8;
      for (i1 = 0; i1 < 6; i1++) {
        imvec[i1] = x_data[i1 + 6 * i];
      }
      b_st.site = &dh_emlrtRSI;
      constvel(&b_st, imvec, specvec_f2, varargin_1);
      r5 = _mm_loadu_pd(&imvec[0]);
      r6 = _mm_loadu_pd(&z[0]);
      _mm_storeu_pd(&U[6 * k], _mm_div_pd(_mm_sub_pd(r5, r6), r4));
      r5 = _mm_loadu_pd(&imvec[2]);
      r6 = _mm_loadu_pd(&z[2]);
      _mm_storeu_pd(&U[6 * k + 2], _mm_div_pd(_mm_sub_pd(r5, r6), r4));
      r5 = _mm_loadu_pd(&imvec[4]);
      r6 = _mm_loadu_pd(&z[4]);
      _mm_storeu_pd(&U[6 * k + 4], _mm_div_pd(_mm_sub_pd(r5, r6), r4));
    }
    memset(&b_F[0], 0, 36U * sizeof(real_T));
    for (k = 0; k < 6; k++) {
      loop_ub = 6 * k + 2;
      acoef = 6 * k + 4;
      for (i1 = 0; i1 < 6; i1++) {
        r4 = _mm_loadu_pd(&F[6 * i1]);
        r5 = _mm_loadu_pd(&b_F[6 * k]);
        r6 = _mm_set1_pd(P_data[(i1 + 6 * k) + 36 * i]);
        _mm_storeu_pd(&b_F[6 * k], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&F[6 * i1 + 2]);
        r5 = _mm_loadu_pd(&b_F[loop_ub]);
        _mm_storeu_pd(&b_F[loop_ub], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&F[6 * i1 + 4]);
        r5 = _mm_loadu_pd(&b_F[acoef]);
        _mm_storeu_pd(&b_F[acoef], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
      }
    }
    memset(&b_U[0], 0, 18U * sizeof(real_T));
    for (k = 0; k < 3; k++) {
      loop_ub = 6 * k + 2;
      acoef = 6 * k + 4;
      for (i1 = 0; i1 < 3; i1++) {
        r4 = _mm_loadu_pd(&U[6 * i1]);
        r5 = _mm_loadu_pd(&b_U[6 * k]);
        r6 = _mm_set1_pd(Q[i1 + 3 * k]);
        _mm_storeu_pd(&b_U[6 * k], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&U[6 * i1 + 2]);
        r5 = _mm_loadu_pd(&b_U[loop_ub]);
        _mm_storeu_pd(&b_U[loop_ub], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&U[6 * i1 + 4]);
        r5 = _mm_loadu_pd(&b_U[acoef]);
        _mm_storeu_pd(&b_U[acoef], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
      }
    }
    memset(&c_F[0], 0, 36U * sizeof(real_T));
    for (k = 0; k < 6; k++) {
      loop_ub = 6 * k + 2;
      acoef = 6 * k + 4;
      for (i1 = 0; i1 < 6; i1++) {
        r4 = _mm_loadu_pd(&b_F[6 * i1]);
        r5 = _mm_loadu_pd(&c_F[6 * k]);
        r6 = _mm_set1_pd(F[k + 6 * i1]);
        _mm_storeu_pd(&c_F[6 * k], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&b_F[6 * i1 + 2]);
        r5 = _mm_loadu_pd(&c_F[loop_ub]);
        _mm_storeu_pd(&c_F[loop_ub], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&b_F[6 * i1 + 4]);
        r5 = _mm_loadu_pd(&c_F[acoef]);
        _mm_storeu_pd(&c_F[acoef], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
      }
    }
    memset(&F[0], 0, 36U * sizeof(real_T));
    for (k = 0; k < 6; k++) {
      loop_ub = 6 * k + 2;
      acoef = 6 * k + 4;
      for (i1 = 0; i1 < 3; i1++) {
        r4 = _mm_loadu_pd(&b_U[6 * i1]);
        r5 = _mm_loadu_pd(&F[6 * k]);
        r6 = _mm_set1_pd(U[k + 6 * i1]);
        _mm_storeu_pd(&F[6 * k], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&b_U[6 * i1 + 2]);
        r5 = _mm_loadu_pd(&F[loop_ub]);
        _mm_storeu_pd(&F[loop_ub], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
        r4 = _mm_loadu_pd(&b_U[6 * i1 + 4]);
        r5 = _mm_loadu_pd(&F[acoef]);
        _mm_storeu_pd(&F[acoef], _mm_add_pd(r5, _mm_mul_pd(r4, r6)));
      }
    }
    if (i + 1 > b_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &vi_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    for (i1 = 0; i1 < 6; i1++) {
      r4 = _mm_loadu_pd(&c_F[6 * i1]);
      r5 = _mm_loadu_pd(&F[6 * i1]);
      loop_ub = 6 * i1 + 36 * i;
      _mm_storeu_pd(&P_data[loop_ub], _mm_add_pd(r4, r5));
      acoef = 6 * i1 + 2;
      r4 = _mm_loadu_pd(&c_F[acoef]);
      r5 = _mm_loadu_pd(&F[acoef]);
      _mm_storeu_pd(&P_data[loop_ub + 2], _mm_add_pd(r4, r5));
      acoef = 6 * i1 + 4;
      r4 = _mm_loadu_pd(&c_F[acoef]);
      r5 = _mm_loadu_pd(&F[acoef]);
      _mm_storeu_pd(&P_data[loop_ub + 4], _mm_add_pd(r4, r5));
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (gaussEKFilter.c) */
