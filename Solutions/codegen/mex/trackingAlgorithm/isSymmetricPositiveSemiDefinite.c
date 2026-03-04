/*
 * isSymmetricPositiveSemiDefinite.c
 *
 * Code generation for function 'isSymmetricPositiveSemiDefinite'
 *
 */

/* Include files */
#include "isSymmetricPositiveSemiDefinite.h"
#include "eigSkewHermitianStandard.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "warning.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo yk_emlrtRSI = {
    159,                                                       /* lineNo */
    "ceval_xgeev",                                             /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xgeev.m" /* pathName */
};

static emlrtRSInfo al_emlrtRSI = {
    42,                                                        /* lineNo */
    "xgeev",                                                   /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xgeev.m" /* pathName */
};

static const char_T cv2[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                               '_', 'd', 'g', 'e', 'e', 'v', 'x'};

/* Function Definitions */
void b_isSymmetricPositiveSemiDefini(const emlrtStack *sp,
                                     const real_T b_value[16])
{
  __m128d r;
  ptrdiff_t ihi_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  creal_T d[4];
  real_T A[16];
  real_T y[16];
  real_T scale[4];
  real_T wimag[4];
  real_T wreal[4];
  real_T dv[2];
  real_T dv1[2];
  real_T abnrm;
  real_T absx;
  real_T rconde;
  real_T rcondv;
  real_T tol;
  real_T vright;
  int32_T i1;
  int32_T i2;
  int32_T idx;
  int32_T k;
  boolean_T x_data[6];
  boolean_T b_y[4];
  boolean_T c_y;
  boolean_T exitg1;
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
  wreal[0] = muDoubleScalarAbs(b_value[0]);
  wreal[1] = muDoubleScalarAbs(b_value[5]);
  wreal[2] = muDoubleScalarAbs(b_value[10]);
  wreal[3] = muDoubleScalarAbs(b_value[15]);
  if (!muDoubleScalarIsNaN(wreal[0])) {
    idx = 1;
  } else {
    idx = 0;
    i2 = 2;
    exitg1 = false;
    while ((!exitg1) && (i2 < 5)) {
      if (!muDoubleScalarIsNaN(wreal[i2 - 1])) {
        idx = i2;
        exitg1 = true;
      } else {
        i2++;
      }
    }
  }
  if (idx == 0) {
    absx = wreal[0];
  } else {
    absx = wreal[idx - 1];
    i2 = idx + 1;
    for (k = i2; k < 5; k++) {
      vright = wreal[k - 1];
      if (absx < vright) {
        absx = vright;
      }
    }
  }
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    absx = rtNaN;
  } else if (absx < 4.4501477170144028E-308) {
    absx = 4.94065645841247E-324;
  } else {
    frexp(absx, &i1);
    absx = ldexp(1.0, i1 - 53);
  }
  tol = 100.0 * absx;
  for (k = 0; k < 4; k++) {
    idx = k << 2;
    A[idx] = b_value[k];
    A[idx + 1] = b_value[k + 4];
    A[idx + 2] = b_value[k + 8];
    A[idx + 3] = b_value[k + 12];
  }
  for (k = 0; k <= 14; k += 2) {
    r = _mm_loadu_pd(&A[k]);
    _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[k]), r));
    dv1[0] = muDoubleScalarAbs(dv[0]);
    dv1[1] = muDoubleScalarAbs(dv[1]);
    r = _mm_loadu_pd(&dv1[0]);
    _mm_storeu_pd(&y[k], r);
  }
  st.site = &he_emlrtRSI;
  absx = muDoubleScalarSqrt(tol);
  st.site = &he_emlrtRSI;
  b_st.site = &je_emlrtRSI;
  b_y[0] = true;
  b_y[1] = true;
  b_y[2] = true;
  b_y[3] = true;
  i2 = 4;
  for (k = 0; k < 4; k++) {
    idx = i2;
    i1 = i2 - 3;
    i2 += 4;
    c_st.site = &ke_emlrtRSI;
    if ((i1 <= idx) && (idx > 2147483646)) {
      d_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    exitg1 = false;
    while ((!exitg1) && (i1 <= idx)) {
      if (!(y[i1 - 1] < absx)) {
        b_y[k] = false;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
  c_y = true;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= 3)) {
    if (!b_y[idx]) {
      c_y = false;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  st.site = &ie_emlrtRSI;
  for (k = 0; k <= 14; k += 2) {
    r = _mm_loadu_pd(&A[k]);
    _mm_storeu_pd(&A[k], _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[k]), r),
                                    _mm_set1_pd(2.0)));
  }
  b_st.site = &le_emlrtRSI;
  c_st.site = &pe_emlrtRSI;
  d_st.site = &qe_emlrtRSI;
  p = true;
  for (k = 0; k < 16; k++) {
    if (p) {
      absx = A[k];
      if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    d[0].re = rtNaN;
    d[1].re = rtNaN;
    d[2].re = rtNaN;
    d[3].re = rtNaN;
  } else {
    int32_T exitg2;
    p = true;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx < 4)) {
      i2 = 0;
      do {
        exitg2 = 0;
        if (i2 <= idx) {
          if (!(A[i2 + (idx << 2)] == A[idx + (i2 << 2)])) {
            p = false;
            exitg2 = 1;
          } else {
            i2++;
          }
        } else {
          idx++;
          exitg2 = 2;
        }
      } while (exitg2 == 0);
      if (exitg2 == 1) {
        exitg1 = true;
      }
    }
    if (p) {
      b_st.site = &me_emlrtRSI;
      c_st.site = &se_emlrtRSI;
      d_st.site = &ue_emlrtRSI;
      n_t = (ptrdiff_t)4;
      n_t = LAPACKE_dsyev(102, 'N', 'L', n_t, &A[0], n_t, &scale[0]);
      e_st.site = &bd_emlrtRSI;
      if ((int32_T)n_t < 0) {
        if ((int32_T)n_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&e_st, &h_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&e_st, &i_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &cv[0], 12, (int32_T)n_t);
        }
      }
      d[0].re = scale[0];
      d[1].re = scale[1];
      d[2].re = scale[2];
      d[3].re = scale[3];
      if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&b_st))) {
        c_st.site = &te_emlrtRSI;
        warning(&c_st);
      }
    } else {
      p = true;
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx < 4)) {
        i2 = 0;
        do {
          exitg2 = 0;
          if (i2 <= idx) {
            if (!(A[i2 + (idx << 2)] == -A[idx + (i2 << 2)])) {
              p = false;
              exitg2 = 1;
            } else {
              i2++;
            }
          } else {
            idx++;
            exitg2 = 2;
          }
        } while (exitg2 == 0);
        if (exitg2 == 1) {
          exitg1 = true;
        }
      }
      if (p) {
        b_st.site = &ne_emlrtRSI;
        b_eigSkewHermitianStandard(&b_st, A, d);
      } else {
        b_st.site = &oe_emlrtRSI;
        c_st.site = &qg_emlrtRSI;
        d_st.site = &al_emlrtRSI;
        n_t = LAPACKE_dgeevx(102, 'B', 'N', 'N', 'N', (ptrdiff_t)4, &A[0],
                             (ptrdiff_t)4, &wreal[0], &wimag[0], &absx,
                             (ptrdiff_t)1, &vright, (ptrdiff_t)1, &n_t, &ihi_t,
                             &scale[0], &abnrm, &rconde, &rcondv);
        e_st.site = &yk_emlrtRSI;
        if ((int32_T)n_t < 0) {
          if ((int32_T)n_t == -1010) {
            emlrtErrorWithMessageIdR2018a(&e_st, &h_emlrtRTEI, "MATLAB:nomem",
                                          "MATLAB:nomem", 0);
          } else {
            emlrtErrorWithMessageIdR2018a(&e_st, &i_emlrtRTEI,
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          5, 4, 14, &cv2[0], 12, (int32_T)n_t);
          }
        }
        d[0].re = wreal[0];
        d[1].re = wreal[1];
        d[2].re = wreal[2];
        d[3].re = wreal[3];
        if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&b_st))) {
          c_st.site = &rg_emlrtRSI;
          warning(&c_st);
        }
      }
    }
  }
  x_data[0] = (d[0].re < -tol);
  x_data[1] = (d[1].re < -tol);
  x_data[2] = (d[2].re < -tol);
  x_data[3] = (d[3].re < -tol);
  p = false;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= 3)) {
    if (x_data[idx]) {
      p = true;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  if (p || (!c_y)) {
    emlrtErrorWithMessageIdR2018a(
        sp, &g_emlrtRTEI,
        "shared_tracking:KalmanFilter:invalidCovarianceValues",
        "shared_tracking:KalmanFilter:invalidCovarianceValues", 3, 4, 16,
        "MeasurementNoise");
  }
}

void isSymmetricPositiveSemiDefinite(const emlrtStack *sp,
                                     const real_T b_value[36])
{
  __m128d r;
  ptrdiff_t ihi_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  creal_T d[6];
  real_T A[36];
  real_T y[36];
  real_T scale[6];
  real_T wimag[6];
  real_T wreal[6];
  real_T dv[2];
  real_T dv1[2];
  real_T abnrm;
  real_T absx;
  real_T rconde;
  real_T rcondv;
  real_T tol;
  real_T vright;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T idx;
  boolean_T b_y[6];
  boolean_T c_y;
  boolean_T exitg1;
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
  for (i = 0; i < 6; i++) {
    wreal[i] = muDoubleScalarAbs(b_value[i + 6 * i]);
  }
  if (!muDoubleScalarIsNaN(wreal[0])) {
    idx = 1;
  } else {
    idx = 0;
    i2 = 2;
    exitg1 = false;
    while ((!exitg1) && (i2 < 7)) {
      if (!muDoubleScalarIsNaN(wreal[i2 - 1])) {
        idx = i2;
        exitg1 = true;
      } else {
        i2++;
      }
    }
  }
  if (idx == 0) {
    absx = wreal[0];
  } else {
    absx = wreal[idx - 1];
    i2 = idx + 1;
    for (i = i2; i < 7; i++) {
      vright = wreal[i - 1];
      if (absx < vright) {
        absx = vright;
      }
    }
  }
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    absx = rtNaN;
  } else if (absx < 4.4501477170144028E-308) {
    absx = 4.94065645841247E-324;
  } else {
    frexp(absx, &i1);
    absx = ldexp(1.0, i1 - 53);
  }
  tol = 100.0 * absx;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      A[b_i + 6 * i] = b_value[i + 6 * b_i];
    }
  }
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&A[i]);
    _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[i]), r));
    dv1[0] = muDoubleScalarAbs(dv[0]);
    dv1[1] = muDoubleScalarAbs(dv[1]);
    r = _mm_loadu_pd(&dv1[0]);
    _mm_storeu_pd(&y[i], r);
  }
  st.site = &he_emlrtRSI;
  absx = muDoubleScalarSqrt(tol);
  st.site = &he_emlrtRSI;
  b_st.site = &je_emlrtRSI;
  for (i = 0; i < 6; i++) {
    b_y[i] = true;
  }
  i2 = 6;
  for (i = 0; i < 6; i++) {
    idx = i2;
    i1 = i2 - 5;
    i2 += 6;
    c_st.site = &ke_emlrtRSI;
    if ((i1 <= idx) && (idx > 2147483646)) {
      d_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    exitg1 = false;
    while ((!exitg1) && (i1 <= idx)) {
      if (!(y[i1 - 1] < absx)) {
        b_y[i] = false;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
  c_y = true;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= 5)) {
    if (!b_y[idx]) {
      c_y = false;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  st.site = &ie_emlrtRSI;
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&A[i]);
    _mm_storeu_pd(&A[i], _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[i]), r),
                                    _mm_set1_pd(2.0)));
  }
  b_st.site = &le_emlrtRSI;
  c_st.site = &pe_emlrtRSI;
  d_st.site = &qe_emlrtRSI;
  p = true;
  for (i = 0; i < 36; i++) {
    if (p) {
      absx = A[i];
      if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (i = 0; i < 6; i++) {
      d[i].re = rtNaN;
      d[i].im = 0.0;
    }
  } else {
    int32_T exitg2;
    p = true;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx < 6)) {
      i2 = 0;
      do {
        exitg2 = 0;
        if (i2 <= idx) {
          if (!(A[i2 + 6 * idx] == A[idx + 6 * i2])) {
            p = false;
            exitg2 = 1;
          } else {
            i2++;
          }
        } else {
          idx++;
          exitg2 = 2;
        }
      } while (exitg2 == 0);
      if (exitg2 == 1) {
        exitg1 = true;
      }
    }
    if (p) {
      b_st.site = &me_emlrtRSI;
      c_st.site = &se_emlrtRSI;
      d_st.site = &ue_emlrtRSI;
      n_t = (ptrdiff_t)6;
      n_t = LAPACKE_dsyev(102, 'N', 'L', n_t, &A[0], n_t, &scale[0]);
      e_st.site = &bd_emlrtRSI;
      if ((int32_T)n_t < 0) {
        if ((int32_T)n_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&e_st, &h_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&e_st, &i_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &cv[0], 12, (int32_T)n_t);
        }
      }
      for (i = 0; i < 6; i++) {
        d[i].re = scale[i];
        d[i].im = 0.0;
      }
      if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&b_st))) {
        c_st.site = &te_emlrtRSI;
        warning(&c_st);
      }
    } else {
      p = true;
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx < 6)) {
        i2 = 0;
        do {
          exitg2 = 0;
          if (i2 <= idx) {
            if (!(A[i2 + 6 * idx] == -A[idx + 6 * i2])) {
              p = false;
              exitg2 = 1;
            } else {
              i2++;
            }
          } else {
            idx++;
            exitg2 = 2;
          }
        } while (exitg2 == 0);
        if (exitg2 == 1) {
          exitg1 = true;
        }
      }
      if (p) {
        b_st.site = &ne_emlrtRSI;
        eigSkewHermitianStandard(&b_st, A, d);
      } else {
        b_st.site = &oe_emlrtRSI;
        c_st.site = &qg_emlrtRSI;
        d_st.site = &al_emlrtRSI;
        n_t = LAPACKE_dgeevx(102, 'B', 'N', 'N', 'N', (ptrdiff_t)6, &A[0],
                             (ptrdiff_t)6, &wreal[0], &wimag[0], &absx,
                             (ptrdiff_t)1, &vright, (ptrdiff_t)1, &n_t, &ihi_t,
                             &scale[0], &abnrm, &rconde, &rcondv);
        e_st.site = &yk_emlrtRSI;
        if ((int32_T)n_t < 0) {
          if ((int32_T)n_t == -1010) {
            emlrtErrorWithMessageIdR2018a(&e_st, &h_emlrtRTEI, "MATLAB:nomem",
                                          "MATLAB:nomem", 0);
          } else {
            emlrtErrorWithMessageIdR2018a(&e_st, &i_emlrtRTEI,
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          5, 4, 14, &cv2[0], 12, (int32_T)n_t);
          }
        }
        for (i = 0; i < 6; i++) {
          d[i].re = wreal[i];
          d[i].im = wimag[i];
        }
        if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&b_st))) {
          c_st.site = &rg_emlrtRSI;
          warning(&c_st);
        }
      }
    }
  }
  for (i = 0; i < 6; i++) {
    b_y[i] = (d[i].re < -tol);
  }
  p = false;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= 5)) {
    if (b_y[idx]) {
      p = true;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  if (p || (!c_y)) {
    emlrtErrorWithMessageIdR2018a(
        sp, &g_emlrtRTEI,
        "shared_tracking:KalmanFilter:invalidCovarianceValues",
        "shared_tracking:KalmanFilter:invalidCovarianceValues", 3, 4, 15,
        "StateCovariance");
  }
}

/* End of code generation (isSymmetricPositiveSemiDefinite.c) */
