/*
 * xnrm2.c
 *
 * Code generation for function 'xnrm2'
 *
 */

/* Include files */
#include "xnrm2.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo id_emlrtRSI = {
    23,                                                      /* lineNo */
    "xnrm2",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xnrm2.m" /* pathName */
};

static emlrtRSInfo jd_emlrtRSI = {
    38,                                                         /* lineNo */
    "xnrm2",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xnrm2.m" /* pathName */
};

static emlrtRSInfo kd_emlrtRSI = {
    64,                                                         /* lineNo */
    "xnrm2",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xnrm2.m" /* pathName */
};

/* Function Definitions */
real_T b_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[9], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T y;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = muDoubleScalarAbs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      b_st.site = &jd_emlrtRSI;
      if ((ix0 <= kend) && (kend > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = muDoubleScalarAbs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * muDoubleScalarSqrt(y);
      if (muDoubleScalarIsNaN(y)) {
        int32_T b_k;
        b_st.site = &kd_emlrtRSI;
        b_k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (b_k <= kend) {
            if (muDoubleScalarIsNaN(x[b_k - 1])) {
              exitg1 = 1;
            } else {
              b_k++;
            }
          } else {
            y = rtInf;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }
  return y;
}

real_T c_xnrm2(int32_T n, const real_T x[3])
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = muDoubleScalarAbs(x[1]);
    } else {
      real_T absxk;
      real_T scale;
      real_T t;
      scale = 3.3121686421112381E-170;
      absxk = muDoubleScalarAbs(x[1]);
      if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
      }
      absxk = muDoubleScalarAbs(x[2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
      y = scale * muDoubleScalarSqrt(y);
      if (muDoubleScalarIsNaN(y)) {
        int32_T k;
        k = 2;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (k < 4) {
            if (muDoubleScalarIsNaN(x[k - 1])) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            y = rtInf;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }
  return y;
}

real_T d_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[6], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  b_st.site = &jd_emlrtRSI;
  if ((ix0 <= kend) && (kend > 2147483646)) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_st.site = &kd_emlrtRSI;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T e_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[54], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  b_st.site = &jd_emlrtRSI;
  if ((ix0 <= kend) && (kend > 2147483646)) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_st.site = &kd_emlrtRSI;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T f_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[16], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T y;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = muDoubleScalarAbs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      b_st.site = &jd_emlrtRSI;
      if ((ix0 <= kend) && (kend > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = muDoubleScalarAbs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * muDoubleScalarSqrt(y);
      if (muDoubleScalarIsNaN(y)) {
        int32_T b_k;
        b_st.site = &kd_emlrtRSI;
        b_k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (b_k <= kend) {
            if (muDoubleScalarIsNaN(x[b_k - 1])) {
              exitg1 = 1;
            } else {
              b_k++;
            }
          } else {
            y = rtInf;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }
  return y;
}

real_T g_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[4], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  b_st.site = &jd_emlrtRSI;
  if ((ix0 <= kend) && (kend > 2147483646)) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_st.site = &kd_emlrtRSI;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T h_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[40], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  b_st.site = &jd_emlrtRSI;
  if ((ix0 <= kend) && (kend > 2147483646)) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_st.site = &kd_emlrtRSI;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T i_xnrm2(const emlrtStack *sp, int32_T n, const real_T x[60], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  boolean_T b;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  b_st.site = &jd_emlrtRSI;
  if ((ix0 <= kend) && (kend > 2147483646)) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = ix0; k <= kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_st.site = &kd_emlrtRSI;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T j_xnrm2(const real_T x[3])
{
  real_T scale;
  real_T y;
  int32_T k;
  boolean_T b;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 2; k < 4; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_k = 2;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= 3) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T xnrm2(const emlrtStack *sp, int32_T n, const real_T x[36], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T y;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &id_emlrtRSI;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = muDoubleScalarAbs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      b_st.site = &jd_emlrtRSI;
      if ((ix0 <= kend) && (kend > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = muDoubleScalarAbs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * muDoubleScalarSqrt(y);
      if (muDoubleScalarIsNaN(y)) {
        int32_T b_k;
        b_st.site = &kd_emlrtRSI;
        b_k = ix0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (b_k <= kend) {
            if (muDoubleScalarIsNaN(x[b_k - 1])) {
              exitg1 = 1;
            } else {
              b_k++;
            }
          } else {
            y = rtInf;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
    }
  }
  return y;
}

/* End of code generation (xnrm2.c) */
