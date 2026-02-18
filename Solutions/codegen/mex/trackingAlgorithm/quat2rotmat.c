/*
 * quat2rotmat.c
 *
 * Code generation for function 'quat2rotmat'
 *
 */

/* Include files */
#include "quat2rotmat.h"
#include "div.h"
#include "normalize.h"
#include "rotmat.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ir_emlrtRSI = {
    19,                      /* lineNo */
    "quaternionBase/rotmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pathName */
};

static emlrtRSInfo jr_emlrtRSI = {
    10,                         /* lineNo */
    "quaternionBase/normalize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pathName */
};

static emlrtRSInfo os_emlrtRSI = {
    47,                      /* lineNo */
    "quaternionBase/rotmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pathName */
};

static emlrtRSInfo ps_emlrtRSI = {
    11,                         /* lineNo */
    "quaternionBase/normalize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pathName */
};

static emlrtRSInfo qs_emlrtRSI = {
    12,                         /* lineNo */
    "quaternionBase/normalize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pathName */
};

static emlrtRSInfo rs_emlrtRSI = {
    13,                         /* lineNo */
    "quaternionBase/normalize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pathName */
};

static emlrtRSInfo ss_emlrtRSI = {
    14,                         /* lineNo */
    "quaternionBase/normalize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pathName */
};

static emlrtRSInfo ts_emlrtRSI = {
    16,     /* lineNo */
    "sqrt", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m" /* pathName
                                                                       */
};

static emlrtRSInfo us_emlrtRSI = {
    34,               /* lineNo */
    "rdivide_helper", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\rdivide_"
    "helper.m" /* pathName */
};

static emlrtRSInfo vs_emlrtRSI = {
    53,    /* lineNo */
    "div", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\div.m" /* pathName
                                                                          */
};

static emlrtRSInfo ws_emlrtRSI = {
    38,        /* lineNo */
    "squeeze", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\squeeze.m" /* pathName
                                                                          */
};

static emlrtECInfo f_emlrtECI = {
    1,                          /* nDims */
    10,                         /* lineNo */
    10,                         /* colNo */
    "quaternionBase/normalize", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pName */
};

static emlrtBCInfo oc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    45,                      /* lineNo */
    14,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo pc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    44,                      /* lineNo */
    16,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo qc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    43,                      /* lineNo */
    16,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo rc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    42,                      /* lineNo */
    16,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo sc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    41,                      /* lineNo */
    16,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo tc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    40,                      /* lineNo */
    16,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo uc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    39,                      /* lineNo */
    16,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo vc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    38,                      /* lineNo */
    18,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtECInfo g_emlrtECI = {
    1,                       /* nDims */
    31,                      /* lineNo */
    7,                       /* colNo */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pName */
};

static emlrtECInfo h_emlrtECI = {
    1,                       /* nDims */
    30,                      /* lineNo */
    7,                       /* colNo */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pName */
};

static emlrtECInfo i_emlrtECI = {
    1,                       /* nDims */
    29,                      /* lineNo */
    7,                       /* colNo */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pName */
};

static emlrtECInfo j_emlrtECI = {
    1,                       /* nDims */
    28,                      /* lineNo */
    7,                       /* colNo */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pName */
};

static emlrtECInfo k_emlrtECI = {
    1,                       /* nDims */
    27,                      /* lineNo */
    7,                       /* colNo */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pName */
};

static emlrtECInfo l_emlrtECI = {
    1,                       /* nDims */
    26,                      /* lineNo */
    7,                       /* colNo */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI = {
    13,                     /* lineNo */
    27,                     /* colNo */
    "assertCompatibleDims", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\assertCompatibleDims.m" /* pName */
};

static emlrtBCInfo wc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    45,                      /* lineNo */
    23,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo xc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    45,                      /* lineNo */
    85,                      /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

static emlrtBCInfo yc_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    45,                      /* lineNo */
    147,                     /* colNo */
    "",                      /* aName */
    "quaternionBase/rotmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\rotmat.m", /* pName */
    0               /* checkKind */
};

/* Function Definitions */
void quat2rotmat(const emlrtStack *sp, const real_T q_a_data[],
                 int32_T q_a_size, const real_T q_b_data[], int32_T q_b_size,
                 const real_T q_c_data[], int32_T q_c_size,
                 const real_T q_d_data[], int32_T q_d_size, real_T rot_data[],
                 int32_T rot_size[3])
{
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T ab2_data[100];
  real_T ac2_data[100];
  real_T ad2_data[100];
  real_T b_q_a_data[100];
  real_T bbsq_data[100];
  real_T bc2_data[100];
  real_T bd2_data[100];
  real_T ccsq_data[100];
  real_T cd2_data[100];
  real_T ddsq_data[100];
  real_T n_data[100];
  real_T d;
  int32_T ab2_size;
  int32_T ac2_size;
  int32_T b_q_a_size;
  int32_T b_scalarLB;
  int32_T bbsq_size;
  int32_T c_scalarLB;
  int32_T ccsq_size;
  int32_T d_scalarLB;
  int32_T ddsq_size;
  int32_T k;
  int32_T n;
  int32_T n_size;
  int32_T nx;
  int32_T scalarLB;
  int8_T szb_idx_2;
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
  st.site = &hr_emlrtRSI;
  b_st.site = &ir_emlrtRSI;
  n_size = q_a_size;
  scalarLB = (q_a_size / 2) << 1;
  nx = scalarLB - 2;
  for (k = 0; k <= nx; k += 2) {
    r = _mm_loadu_pd(&q_a_data[k]);
    _mm_storeu_pd(&n_data[k], _mm_mul_pd(r, r));
  }
  for (k = scalarLB; k < q_a_size; k++) {
    d = q_a_data[k];
    n_data[k] = d * d;
  }
  b_scalarLB = (q_b_size / 2) << 1;
  nx = b_scalarLB - 2;
  for (k = 0; k <= nx; k += 2) {
    r = _mm_loadu_pd(&q_b_data[k]);
    _mm_storeu_pd(&ab2_data[k], _mm_mul_pd(r, r));
  }
  for (k = b_scalarLB; k < q_b_size; k++) {
    d = q_b_data[k];
    ab2_data[k] = d * d;
  }
  if ((q_a_size != q_b_size) && ((q_a_size != 1) && (q_b_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(q_a_size, q_b_size, &f_emlrtECI, &b_st);
  }
  if (q_a_size == q_b_size) {
    nx = scalarLB - 2;
    for (k = 0; k <= nx; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      r1 = _mm_loadu_pd(&ab2_data[k]);
      _mm_storeu_pd(&n_data[k], _mm_add_pd(r, r1));
    }
    for (k = scalarLB; k < q_a_size; k++) {
      n_data[k] += ab2_data[k];
    }
  } else {
    plus(n_data, &n_size, ab2_data, &q_b_size);
  }
  c_scalarLB = (q_c_size / 2) << 1;
  nx = c_scalarLB - 2;
  for (k = 0; k <= nx; k += 2) {
    r = _mm_loadu_pd(&q_c_data[k]);
    _mm_storeu_pd(&ab2_data[k], _mm_mul_pd(r, r));
  }
  for (k = c_scalarLB; k < q_c_size; k++) {
    d = q_c_data[k];
    ab2_data[k] = d * d;
  }
  if ((n_size != q_c_size) && ((n_size != 1) && (q_c_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(n_size, q_c_size, &f_emlrtECI, &b_st);
  }
  if (n_size == q_c_size) {
    nx = (n_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      r1 = _mm_loadu_pd(&ab2_data[k]);
      _mm_storeu_pd(&n_data[k], _mm_add_pd(r, r1));
    }
    for (k = nx; k < n_size; k++) {
      n_data[k] += ab2_data[k];
    }
  } else {
    plus(n_data, &n_size, ab2_data, &q_c_size);
  }
  d_scalarLB = (q_d_size / 2) << 1;
  nx = d_scalarLB - 2;
  for (k = 0; k <= nx; k += 2) {
    r = _mm_loadu_pd(&q_d_data[k]);
    _mm_storeu_pd(&ab2_data[k], _mm_mul_pd(r, r));
  }
  for (k = d_scalarLB; k < q_d_size; k++) {
    d = q_d_data[k];
    ab2_data[k] = d * d;
  }
  if ((n_size != q_d_size) && ((n_size != 1) && (q_d_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(n_size, q_d_size, &f_emlrtECI, &b_st);
  }
  c_st.site = &jr_emlrtRSI;
  if (n_size == q_d_size) {
    nx = (n_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      r1 = _mm_loadu_pd(&ab2_data[k]);
      _mm_storeu_pd(&n_data[k], _mm_add_pd(r, r1));
    }
    for (k = nx; k < n_size; k++) {
      n_data[k] += ab2_data[k];
    }
  } else {
    plus(n_data, &n_size, ab2_data, &q_d_size);
  }
  p = false;
  for (k = 0; k < n_size; k++) {
    if (p || (n_data[k] < 0.0)) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  d_st.site = &ts_emlrtRSI;
  nx = (n_size / 2) << 1;
  n = nx - 2;
  for (k = 0; k <= n; k += 2) {
    r = _mm_loadu_pd(&n_data[k]);
    _mm_storeu_pd(&n_data[k], _mm_sqrt_pd(r));
  }
  for (k = nx; k < n_size; k++) {
    n_data[k] = muDoubleScalarSqrt(n_data[k]);
  }
  c_st.site = &ps_emlrtRSI;
  d_st.site = &us_emlrtRSI;
  e_st.site = &vs_emlrtRSI;
  if ((q_a_size != 1) && (n_size != 1) && (q_a_size != n_size)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                  "MATLAB:sizeDimensionsMustMatch",
                                  "MATLAB:sizeDimensionsMustMatch", 0);
  }
  if (q_a_size == n_size) {
    b_q_a_size = q_a_size;
    nx = scalarLB - 2;
    for (k = 0; k <= nx; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      _mm_storeu_pd(&b_q_a_data[k], _mm_div_pd(_mm_loadu_pd(&q_a_data[k]), r));
    }
    for (k = scalarLB; k < q_a_size; k++) {
      b_q_a_data[k] = q_a_data[k] / n_data[k];
    }
  } else {
    b_q_a_size = rdivide(b_q_a_data, q_a_data, &q_a_size, n_data, &n_size);
  }
  c_st.site = &qs_emlrtRSI;
  d_st.site = &us_emlrtRSI;
  e_st.site = &vs_emlrtRSI;
  if ((q_b_size != 1) && (n_size != 1) && (q_b_size != n_size)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                  "MATLAB:sizeDimensionsMustMatch",
                                  "MATLAB:sizeDimensionsMustMatch", 0);
  }
  if (q_b_size == n_size) {
    bbsq_size = q_b_size;
    nx = b_scalarLB - 2;
    for (k = 0; k <= nx; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      _mm_storeu_pd(&bbsq_data[k], _mm_div_pd(_mm_loadu_pd(&q_b_data[k]), r));
    }
    for (k = b_scalarLB; k < q_b_size; k++) {
      bbsq_data[k] = q_b_data[k] / n_data[k];
    }
  } else {
    bbsq_size = rdivide(bbsq_data, q_b_data, &q_b_size, n_data, &n_size);
  }
  c_st.site = &rs_emlrtRSI;
  d_st.site = &us_emlrtRSI;
  e_st.site = &vs_emlrtRSI;
  if ((q_c_size != 1) && (n_size != 1) && (q_c_size != n_size)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                  "MATLAB:sizeDimensionsMustMatch",
                                  "MATLAB:sizeDimensionsMustMatch", 0);
  }
  if (q_c_size == n_size) {
    ccsq_size = q_c_size;
    nx = c_scalarLB - 2;
    for (k = 0; k <= nx; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      _mm_storeu_pd(&ccsq_data[k], _mm_div_pd(_mm_loadu_pd(&q_c_data[k]), r));
    }
    for (k = c_scalarLB; k < q_c_size; k++) {
      ccsq_data[k] = q_c_data[k] / n_data[k];
    }
  } else {
    ccsq_size = rdivide(ccsq_data, q_c_data, &q_c_size, n_data, &n_size);
  }
  c_st.site = &ss_emlrtRSI;
  d_st.site = &us_emlrtRSI;
  e_st.site = &vs_emlrtRSI;
  if ((q_d_size != 1) && (n_size != 1) && (q_d_size != n_size)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                  "MATLAB:sizeDimensionsMustMatch",
                                  "MATLAB:sizeDimensionsMustMatch", 0);
  }
  if (q_d_size == n_size) {
    ddsq_size = q_d_size;
    nx = d_scalarLB - 2;
    for (k = 0; k <= nx; k += 2) {
      r = _mm_loadu_pd(&n_data[k]);
      _mm_storeu_pd(&ddsq_data[k], _mm_div_pd(_mm_loadu_pd(&q_d_data[k]), r));
    }
    for (k = d_scalarLB; k < q_d_size; k++) {
      ddsq_data[k] = q_d_data[k] / n_data[k];
    }
  } else {
    ddsq_size = rdivide(ddsq_data, q_d_data, &q_d_size, n_data, &n_size);
  }
  if ((b_q_a_size != bbsq_size) && ((b_q_a_size != 1) && (bbsq_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_q_a_size, bbsq_size, &l_emlrtECI, &st);
  }
  if (b_q_a_size == bbsq_size) {
    ab2_size = b_q_a_size;
    nx = (b_q_a_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&b_q_a_data[k]);
      r1 = _mm_loadu_pd(&bbsq_data[k]);
      _mm_storeu_pd(&ab2_data[k],
                    _mm_mul_pd(_mm_mul_pd(r, r1), _mm_set1_pd(2.0)));
    }
    for (k = nx; k < b_q_a_size; k++) {
      ab2_data[k] = b_q_a_data[k] * bbsq_data[k] * 2.0;
    }
  } else {
    ab2_size = binary_expand_op(ab2_data, b_q_a_data, &b_q_a_size, bbsq_data,
                                &bbsq_size);
  }
  if ((b_q_a_size != ccsq_size) && ((b_q_a_size != 1) && (ccsq_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_q_a_size, ccsq_size, &k_emlrtECI, &st);
  }
  if (b_q_a_size == ccsq_size) {
    ac2_size = b_q_a_size;
    nx = (b_q_a_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&b_q_a_data[k]);
      r1 = _mm_loadu_pd(&ccsq_data[k]);
      _mm_storeu_pd(&ac2_data[k],
                    _mm_mul_pd(_mm_mul_pd(r, r1), _mm_set1_pd(2.0)));
    }
    for (k = nx; k < b_q_a_size; k++) {
      ac2_data[k] = b_q_a_data[k] * ccsq_data[k] * 2.0;
    }
  } else {
    ac2_size = binary_expand_op(ac2_data, b_q_a_data, &b_q_a_size, ccsq_data,
                                &ccsq_size);
  }
  if ((b_q_a_size != ddsq_size) && ((b_q_a_size != 1) && (ddsq_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_q_a_size, ddsq_size, &j_emlrtECI, &st);
  }
  if (b_q_a_size == ddsq_size) {
    b_scalarLB = b_q_a_size;
    nx = (b_q_a_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&b_q_a_data[k]);
      r1 = _mm_loadu_pd(&ddsq_data[k]);
      _mm_storeu_pd(&ad2_data[k],
                    _mm_mul_pd(_mm_mul_pd(r, r1), _mm_set1_pd(2.0)));
    }
    for (k = nx; k < b_q_a_size; k++) {
      ad2_data[k] = b_q_a_data[k] * ddsq_data[k] * 2.0;
    }
  } else {
    b_scalarLB = binary_expand_op(ad2_data, b_q_a_data, &b_q_a_size, ddsq_data,
                                  &ddsq_size);
  }
  if ((bbsq_size != ccsq_size) && ((bbsq_size != 1) && (ccsq_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(bbsq_size, ccsq_size, &i_emlrtECI, &st);
  }
  if (bbsq_size == ccsq_size) {
    c_scalarLB = bbsq_size;
    nx = (bbsq_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&bbsq_data[k]);
      r1 = _mm_loadu_pd(&ccsq_data[k]);
      _mm_storeu_pd(&bc2_data[k],
                    _mm_mul_pd(_mm_mul_pd(r, r1), _mm_set1_pd(2.0)));
    }
    for (k = nx; k < bbsq_size; k++) {
      bc2_data[k] = bbsq_data[k] * ccsq_data[k] * 2.0;
    }
  } else {
    c_scalarLB = binary_expand_op(bc2_data, bbsq_data, &bbsq_size, ccsq_data,
                                  &ccsq_size);
  }
  if ((bbsq_size != ddsq_size) && ((bbsq_size != 1) && (ddsq_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(bbsq_size, ddsq_size, &h_emlrtECI, &st);
  }
  if (bbsq_size == ddsq_size) {
    scalarLB = bbsq_size;
    nx = (bbsq_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&bbsq_data[k]);
      r1 = _mm_loadu_pd(&ddsq_data[k]);
      _mm_storeu_pd(&bd2_data[k],
                    _mm_mul_pd(_mm_mul_pd(r, r1), _mm_set1_pd(2.0)));
    }
    for (k = nx; k < bbsq_size; k++) {
      bd2_data[k] = bbsq_data[k] * ddsq_data[k] * 2.0;
    }
  } else {
    scalarLB = binary_expand_op(bd2_data, bbsq_data, &bbsq_size, ddsq_data,
                                &ddsq_size);
  }
  if ((ccsq_size != ddsq_size) && ((ccsq_size != 1) && (ddsq_size != 1))) {
    emlrtDimSizeImpxCheckR2021b(ccsq_size, ddsq_size, &g_emlrtECI, &st);
  }
  if (ccsq_size == ddsq_size) {
    d_scalarLB = ccsq_size;
    nx = (ccsq_size / 2) << 1;
    n = nx - 2;
    for (k = 0; k <= n; k += 2) {
      r = _mm_loadu_pd(&ccsq_data[k]);
      r1 = _mm_loadu_pd(&ddsq_data[k]);
      _mm_storeu_pd(&cd2_data[k],
                    _mm_mul_pd(_mm_mul_pd(r, r1), _mm_set1_pd(2.0)));
    }
    for (k = nx; k < ccsq_size; k++) {
      cd2_data[k] = ccsq_data[k] * ddsq_data[k] * 2.0;
    }
  } else {
    d_scalarLB = binary_expand_op(cd2_data, ccsq_data, &ccsq_size, ddsq_data,
                                  &ddsq_size);
  }
  nx = (b_q_a_size / 2) << 1;
  n = nx - 2;
  for (k = 0; k <= n; k += 2) {
    r = _mm_loadu_pd(&b_q_a_data[k]);
    _mm_storeu_pd(&n_data[k],
                  _mm_sub_pd(_mm_mul_pd(_mm_mul_pd(r, r), _mm_set1_pd(2.0)),
                             _mm_set1_pd(1.0)));
  }
  for (k = nx; k < b_q_a_size; k++) {
    d = b_q_a_data[k];
    n_data[k] = d * d * 2.0 - 1.0;
  }
  nx = (bbsq_size / 2) << 1;
  n = nx - 2;
  for (k = 0; k <= n; k += 2) {
    r = _mm_loadu_pd(&bbsq_data[k]);
    _mm_storeu_pd(&bbsq_data[k],
                  _mm_mul_pd(_mm_mul_pd(r, r), _mm_set1_pd(2.0)));
  }
  for (k = nx; k < bbsq_size; k++) {
    d = bbsq_data[k];
    d = d * d * 2.0;
    bbsq_data[k] = d;
  }
  nx = (ccsq_size / 2) << 1;
  n = nx - 2;
  for (k = 0; k <= n; k += 2) {
    r = _mm_loadu_pd(&ccsq_data[k]);
    _mm_storeu_pd(&ccsq_data[k],
                  _mm_mul_pd(_mm_mul_pd(r, r), _mm_set1_pd(2.0)));
  }
  for (k = nx; k < ccsq_size; k++) {
    d = ccsq_data[k];
    d = d * d * 2.0;
    ccsq_data[k] = d;
  }
  nx = (ddsq_size / 2) << 1;
  n = nx - 2;
  for (k = 0; k <= n; k += 2) {
    r = _mm_loadu_pd(&ddsq_data[k]);
    _mm_storeu_pd(&ddsq_data[k],
                  _mm_mul_pd(_mm_mul_pd(r, r), _mm_set1_pd(2.0)));
  }
  for (k = nx; k < ddsq_size; k++) {
    d = ddsq_data[k];
    d = d * d * 2.0;
    ddsq_data[k] = d;
  }
  nx = 9 * b_q_a_size;
  if (nx - 1 >= 0) {
    memset(&rot_data[0], 0, (uint32_T)nx * sizeof(real_T));
  }
  if (q_a_size != 1) {
    n_size = q_a_size;
  }
  for (k = 0; k < n_size; k++) {
    real_T d1;
    real_T d2;
    real_T d3;
    real_T d4;
    if (k + 1 > b_q_a_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_q_a_size, &vc_emlrtBCI, &st);
    }
    if (k + 1 > c_scalarLB) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, c_scalarLB, &uc_emlrtBCI, &st);
    }
    if (k + 1 > scalarLB) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, scalarLB, &tc_emlrtBCI, &st);
    }
    if (k + 1 > ac2_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, ac2_size, &sc_emlrtBCI, &st);
    }
    if (k + 1 > b_scalarLB) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_scalarLB, &rc_emlrtBCI, &st);
    }
    if (k + 1 > d_scalarLB) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, d_scalarLB, &qc_emlrtBCI, &st);
    }
    if (k + 1 > ab2_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, ab2_size, &pc_emlrtBCI, &st);
    }
    if (k + 1 > b_q_a_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_q_a_size, &oc_emlrtBCI, &st);
    }
    if (k + 1 > bbsq_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, bbsq_size, &wc_emlrtBCI, &st);
    }
    d1 = n_data[k];
    rot_data[9 * k] = d1 + bbsq_data[k];
    d = bc2_data[k];
    d2 = ad2_data[k];
    rot_data[9 * k + 3] = d + d2;
    d3 = bd2_data[k];
    d4 = ac2_data[k];
    rot_data[9 * k + 6] = d3 - d4;
    rot_data[9 * k + 1] = d - d2;
    if (k + 1 > ccsq_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, ccsq_size, &xc_emlrtBCI, &st);
    }
    rot_data[9 * k + 4] = d1 + ccsq_data[k];
    d = cd2_data[k];
    d2 = ab2_data[k];
    rot_data[9 * k + 7] = d + d2;
    rot_data[9 * k + 2] = d3 + d4;
    rot_data[9 * k + 5] = d - d2;
    if (k + 1 > ddsq_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, ddsq_size, &yc_emlrtBCI, &st);
    }
    rot_data[9 * k + 8] = d1 + ddsq_data[k];
  }
  b_st.site = &os_emlrtRSI;
  szb_idx_2 = 1;
  if (b_q_a_size != 1) {
    szb_idx_2 = (int8_T)b_q_a_size;
  }
  c_st.site = &ws_emlrtRSI;
  nx = 9 * b_q_a_size;
  d_st.site = &xs_emlrtRSI;
  n = 3;
  if (b_q_a_size > 3) {
    n = b_q_a_size;
  }
  if (szb_idx_2 > muIntScalarMax_sint32(nx, n)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &o_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (9 * szb_idx_2 != nx) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &p_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  rot_size[0] = 3;
  rot_size[1] = 3;
  rot_size[2] = szb_idx_2;
}

/* End of code generation (quat2rotmat.c) */
