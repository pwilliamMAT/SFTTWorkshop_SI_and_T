/*
 * svd.c
 *
 * Code generation for function 'svd'
 *
 */

/* Include files */
#include "svd.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xscal.h"
#include "xswap.h"
#include "xzlangeM.h"
#include "xzlascl.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo li_emlrtRSI = {
    52,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo mi_emlrtRSI = {
    107,          /* lineNo */
    "callLAPACK", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo ni_emlrtRSI = {
    34,       /* lineNo */
    "xgesvd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesvd.m" /* pathName */
};

static emlrtRSInfo oi_emlrtRSI = {
    452,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo pi_emlrtRSI = {
    431,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo qi_emlrtRSI = {
    418,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo ri_emlrtRSI = {
    415,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo si_emlrtRSI = {
    404,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo ti_emlrtRSI = {
    377,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo ui_emlrtRSI = {
    375,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo vi_emlrtRSI = {
    358,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo wi_emlrtRSI = {
    275,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo xi_emlrtRSI = {
    264,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo yi_emlrtRSI = {
    239,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo aj_emlrtRSI = {
    218,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo bj_emlrtRSI = {
    208,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo cj_emlrtRSI = {
    144,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo dj_emlrtRSI = {
    138,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo ej_emlrtRSI = {
    118,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo fj_emlrtRSI = {
    106,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo gj_emlrtRSI = {
    101,      /* lineNo */
    "xzsvdc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pathName */
};

static emlrtRSInfo hj_emlrtRSI = {
    21,                   /* lineNo */
    "scaleVectorByRecip", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\scaleVectorByRecip.m" /* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    293,      /* lineNo */
    13,       /* colNo */
    "xzsvdc", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" /* pName */
};

/* Function Definitions */
void b_svd(const emlrtStack *sp, const real_T A[36], real_T U[36], real_T s[6],
           real_T V[36])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b_A[36];
  real_T b_s[6];
  real_T e[6];
  real_T work[6];
  real_T anrm;
  real_T b;
  real_T cscale;
  real_T f;
  real_T nrm;
  real_T rt;
  real_T scale;
  real_T sm;
  real_T snorm;
  int32_T i;
  int32_T iter;
  int32_T m;
  int32_T q;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T qs;
  boolean_T doscale;
  boolean_T exitg1;
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
  st.site = &li_emlrtRSI;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  b_st.site = &mi_emlrtRSI;
  c_st.site = &ni_emlrtRSI;
  for (i = 0; i < 6; i++) {
    b_s[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }
  memset(&U[0], 0, 36U * sizeof(real_T));
  memset(&V[0], 0, 36U * sizeof(real_T));
  doscale = false;
  anrm = b_xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    g_xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    g_xzlascl(anrm, cscale, b_A);
  }
  for (q = 0; q < 5; q++) {
    boolean_T apply_transform;
    qp1 = q + 2;
    iter = q + 6 * q;
    qq = iter + 1;
    apply_transform = false;
    d_st.site = &gj_emlrtRSI;
    nrm = xnrm2(&d_st, 6 - q, b_A, iter + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        nrm = -nrm;
      }
      b_s[q] = nrm;
      d_st.site = &fj_emlrtRSI;
      if (muDoubleScalarAbs(nrm) >= 1.0020841800044864E-292) {
        e_st.site = &hj_emlrtRSI;
        xscal(&e_st, 6 - q, 1.0 / nrm, b_A, iter + 1);
      } else {
        qjj = (iter - q) + 6;
        qp1jj = ((((qjj - iter) / 2) << 1) + iter) + 1;
        qs = qp1jj - 2;
        for (i = qq; i <= qs; i += 2) {
          r = _mm_loadu_pd(&b_A[i - 1]);
          _mm_storeu_pd(&b_A[i - 1], _mm_div_pd(r, _mm_set1_pd(b_s[q])));
        }
        for (i = qp1jj; i <= qjj; i++) {
          b_A[i - 1] /= b_s[q];
        }
      }
      b_A[iter]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0;
    }
    for (i = qp1; i < 7; i++) {
      qjj = q + 6 * (i - 1);
      if (apply_transform) {
        d_st.site = &ej_emlrtRSI;
        nrm = xdotc(&d_st, 6 - q, b_A, iter + 1, b_A, qjj + 1);
        nrm = -(nrm / b_A[iter]);
        b_xaxpy(6 - q, nrm, iter + 1, b_A, qjj + 1);
      }
      e[i - 1] = b_A[qjj];
    }
    for (i = q + 1; i < 7; i++) {
      qp1jj = (i + 6 * q) - 1;
      U[qp1jj] = b_A[qp1jj];
    }
    if (q + 1 <= 4) {
      d_st.site = &dj_emlrtRSI;
      nrm = d_xnrm2(&d_st, 5 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        d_st.site = &cj_emlrtRSI;
        nrm = e[q];
        if (muDoubleScalarAbs(e[q]) >= 1.0020841800044864E-292) {
          e_st.site = &hj_emlrtRSI;
          e_xscal(&e_st, 5 - q, 1.0 / e[q], e, q + 2);
        } else {
          qp1jj = ((((5 - q) / 2) << 1) + q) + 2;
          qjj = qp1jj - 2;
          for (i = qp1; i <= qjj; i += 2) {
            r = _mm_loadu_pd(&e[i - 1]);
            _mm_storeu_pd(&e[i - 1], _mm_div_pd(r, _mm_set1_pd(nrm)));
          }
          for (i = qp1jj; i < 7; i++) {
            e[i - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (i = qp1; i < 7; i++) {
          work[i - 1] = 0.0;
        }
        for (i = qp1; i < 7; i++) {
          c_xaxpy(5 - q, e[i - 1], b_A, (q + 6 * (i - 1)) + 2, work, q + 2);
        }
        for (i = qp1; i < 7; i++) {
          d_xaxpy(5 - q, -e[i - 1] / e[q + 1], work, q + 2, b_A,
                  (q + 6 * (i - 1)) + 2);
        }
      }
      for (i = qp1; i < 7; i++) {
        V[(i + 6 * q) - 1] = e[i - 1];
      }
    }
  }
  m = 4;
  b_s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0;
  for (i = 0; i < 6; i++) {
    U[i + 30] = 0.0;
  }
  U[35] = 1.0;
  for (q = 4; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 6 * q;
    if (b_s[q] != 0.0) {
      for (i = qp1; i < 7; i++) {
        qjj = (q + 6 * (i - 1)) + 1;
        d_st.site = &bj_emlrtRSI;
        nrm = xdotc(&d_st, 6 - q, U, qq + 1, U, qjj);
        nrm = -(nrm / U[qq]);
        b_xaxpy(6 - q, nrm, qq + 1, U, qjj);
      }
      qs = ((((6 - q) / 2) << 1) + q) + 1;
      qjj = qs - 2;
      for (i = q + 1; i <= qjj; i += 2) {
        qp1jj = (i + 6 * q) - 1;
        r = _mm_loadu_pd(&U[qp1jj]);
        _mm_storeu_pd(&U[qp1jj], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (i = qs; i < 7; i++) {
        qjj = (i + 6 * q) - 1;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      d_st.site = &aj_emlrtRSI;
      for (i = 0; i < q; i++) {
        U[i + 6 * q] = 0.0;
      }
    } else {
      for (i = 0; i < 6; i++) {
        U[i + 6 * q] = 0.0;
      }
      U[qq] = 1.0;
    }
  }
  for (q = 5; q >= 0; q--) {
    if ((q + 1 <= 4) && (e[q] != 0.0)) {
      qp1 = q + 2;
      qjj = (q + 6 * q) + 2;
      for (i = qp1; i < 7; i++) {
        qp1jj = (q + 6 * (i - 1)) + 2;
        d_st.site = &yi_emlrtRSI;
        nrm = xdotc(&d_st, 5 - q, V, qjj, V, qp1jj);
        nrm = -(nrm / V[qjj - 1]);
        b_xaxpy(5 - q, nrm, qjj, V, qp1jj);
      }
    }
    for (i = 0; i < 6; i++) {
      V[i + 6 * q] = 0.0;
    }
    V[q + 6 * q] = 1.0;
  }
  for (i = 0; i < 6; i++) {
    nrm = b_s[i];
    if (nrm != 0.0) {
      rt = muDoubleScalarAbs(nrm);
      nrm /= rt;
      b_s[i] = rt;
      if (i + 1 < 6) {
        e[i] /= nrm;
      }
      d_st.site = &xi_emlrtRSI;
      f_xscal(&d_st, nrm, U, 6 * i + 1);
    }
    if (i + 1 < 6) {
      nrm = e[i];
      if (nrm != 0.0) {
        rt = muDoubleScalarAbs(nrm);
        nrm = rt / nrm;
        e[i] = rt;
        b_s[i + 1] *= nrm;
        d_st.site = &wi_emlrtRSI;
        f_xscal(&d_st, nrm, V, 6 * (i + 1) + 1);
      }
    }
  }
  iter = 0;
  snorm = 0.0;
  for (i = 0; i < 6; i++) {
    snorm =
        muDoubleScalarMax(snorm, muDoubleScalarMax(muDoubleScalarAbs(b_s[i]),
                                                   muDoubleScalarAbs(e[i])));
  }
  exitg1 = false;
  while ((!exitg1) && (m + 2 > 0)) {
    if (iter >= 75) {
      emlrtErrorWithMessageIdR2018a(&c_st, &j_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    } else {
      boolean_T exitg2;
      qq = m + 1;
      exitg2 = false;
      while (!(exitg2 || (qq == 0))) {
        nrm = muDoubleScalarAbs(e[qq - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (muDoubleScalarAbs(b_s[qq - 1]) +
                                              muDoubleScalarAbs(b_s[qq]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[qq - 1] = 0.0;
          exitg2 = true;
        } else {
          qq--;
        }
      }
      if (qq == m + 1) {
        qp1jj = 4;
      } else {
        qs = m + 2;
        qjj = m + 2;
        exitg2 = false;
        while ((!exitg2) && (qjj >= qq)) {
          qs = qjj;
          if (qjj == qq) {
            exitg2 = true;
          } else {
            nrm = 0.0;
            if (qjj < m + 2) {
              nrm = muDoubleScalarAbs(e[qjj - 1]);
            }
            if (qjj > qq + 1) {
              nrm += muDoubleScalarAbs(e[qjj - 2]);
            }
            rt = muDoubleScalarAbs(b_s[qjj - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              b_s[qjj - 1] = 0.0;
              exitg2 = true;
            } else {
              qjj--;
            }
          }
        }
        if (qs == qq) {
          qp1jj = 3;
        } else if (qs == m + 2) {
          qp1jj = 1;
        } else {
          qp1jj = 2;
          qq = qs;
        }
      }
      switch (qp1jj) {
      case 1:
        f = e[m];
        e[m] = 0.0;
        qjj = m + 1;
        for (i = qjj; i >= qq + 1; i--) {
          d_st.site = &vi_emlrtRSI;
          nrm = 0.0;
          rt = 0.0;
          drotg(&b_s[i - 1], &f, &nrm, &rt);
          if (i > qq + 1) {
            b = e[i - 2];
            f = -rt * b;
            e[i - 2] = b * nrm;
          }
          b_xrot(V, 6 * (i - 1) + 1, 6 * (m + 1) + 1, nrm, rt);
        }
        break;
      case 2:
        f = e[qq - 1];
        e[qq - 1] = 0.0;
        d_st.site = &ui_emlrtRSI;
        for (i = qq + 1; i <= m + 2; i++) {
          d_st.site = &ti_emlrtRSI;
          nrm = 0.0;
          rt = 0.0;
          drotg(&b_s[i - 1], &f, &nrm, &rt);
          b = e[i - 1];
          f = -rt * b;
          e[i - 1] = b * nrm;
          b_xrot(U, 6 * (i - 1) + 1, 6 * (qq - 1) + 1, nrm, rt);
        }
        break;
      case 3: {
        real_T sqds;
        qjj = m + 1;
        nrm = b_s[m + 1];
        scale = muDoubleScalarMax(
            muDoubleScalarMax(
                muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarAbs(nrm),
                                                    muDoubleScalarAbs(b_s[m])),
                                  muDoubleScalarAbs(e[m])),
                muDoubleScalarAbs(b_s[qq])),
            muDoubleScalarAbs(e[qq]));
        sm = nrm / scale;
        nrm = b_s[m] / scale;
        rt = e[m] / scale;
        sqds = b_s[qq] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          d_st.site = &si_emlrtRSI;
          rt = muDoubleScalarSqrt(b * b + nrm);
          if (b < 0.0) {
            rt = -rt;
          }
          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }
        f = (sqds + sm) * (sqds - sm) + rt;
        nrm = sqds * (e[qq] / scale);
        d_st.site = &ri_emlrtRSI;
        for (i = qq + 1; i <= qjj; i++) {
          d_st.site = &qi_emlrtRSI;
          b = 0.0;
          sm = 0.0;
          drotg(&f, &nrm, &b, &sm);
          if (i > qq + 1) {
            e[i - 2] = f;
          }
          nrm = e[i - 1];
          rt = b_s[i - 1];
          scale = b * rt + sm * nrm;
          e[i - 1] = b * nrm - sm * rt;
          nrm = b_s[i];
          rt = sm * nrm;
          sqds = nrm * b;
          qp1jj = 6 * (i - 1) + 1;
          qs = 6 * i + 1;
          b_xrot(V, qp1jj, qs, b, sm);
          d_st.site = &pi_emlrtRSI;
          b = 0.0;
          sm = 0.0;
          drotg(&scale, &rt, &b, &sm);
          b_s[i - 1] = scale;
          nrm = e[i - 1];
          f = b * nrm + sm * sqds;
          sqds = -sm * nrm + b * sqds;
          b_s[i] = sqds;
          nrm = sm * e[i];
          e[i] *= b;
          b_xrot(U, qp1jj, qs, b, sm);
        }
        e[m] = f;
        iter++;
      } break;
      default:
        if (b_s[qq] < 0.0) {
          b_s[qq] = -b_s[qq];
          d_st.site = &oi_emlrtRSI;
          g_xscal(&d_st, V, 6 * qq + 1);
        }
        qp1 = qq + 1;
        while ((qq + 1 < 6) && (b_s[qq] < b_s[qp1])) {
          rt = b_s[qq];
          b_s[qq] = b_s[qp1];
          b_s[qp1] = rt;
          qjj = 6 * qq + 1;
          qp1jj = 6 * (qq + 1) + 1;
          c_xswap(V, qjj, qp1jj);
          c_xswap(U, qjj, qp1jj);
          qq = qp1;
          qp1++;
        }
        iter = 0;
        m--;
        break;
      }
    }
  }
  for (i = 0; i < 6; i++) {
    s[i] = b_s[i];
  }
  if (doscale) {
    d_xzlascl(cscale, anrm, s);
  }
}

void c_svd(const emlrtStack *sp, const real_T A[16], real_T U[16], real_T s[4],
           real_T V[16])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b_A[16];
  real_T e[4];
  real_T work[4];
  real_T anrm;
  real_T b;
  real_T cscale;
  real_T f;
  real_T nrm;
  real_T rt;
  real_T scale;
  real_T sm;
  real_T snorm;
  int32_T iter;
  int32_T jj;
  int32_T m;
  int32_T q;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T vectorUB;
  boolean_T doscale;
  boolean_T exitg1;
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
  st.site = &li_emlrtRSI;
  memcpy(&b_A[0], &A[0], 16U * sizeof(real_T));
  b_st.site = &mi_emlrtRSI;
  c_st.site = &ni_emlrtRSI;
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  s[3] = 0.0;
  e[3] = 0.0;
  work[3] = 0.0;
  memset(&U[0], 0, 16U * sizeof(real_T));
  memset(&V[0], 0, 16U * sizeof(real_T));
  doscale = false;
  anrm = c_xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    h_xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    h_xzlascl(anrm, cscale, b_A);
  }
  for (q = 0; q < 3; q++) {
    boolean_T apply_transform;
    qp1 = q + 2;
    m = q << 2;
    iter = q + m;
    qq = iter + 1;
    apply_transform = false;
    d_st.site = &gj_emlrtRSI;
    nrm = f_xnrm2(&d_st, 4 - q, b_A, iter + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      d_st.site = &fj_emlrtRSI;
      if (muDoubleScalarAbs(nrm) >= 1.0020841800044864E-292) {
        e_st.site = &hj_emlrtRSI;
        h_xscal(&e_st, 4 - q, 1.0 / nrm, b_A, iter + 1);
      } else {
        qjj = (iter - q) + 4;
        qp1jj = ((((qjj - iter) / 2) << 1) + iter) + 1;
        vectorUB = qp1jj - 2;
        for (jj = qq; jj <= vectorUB; jj += 2) {
          r = _mm_loadu_pd(&b_A[jj - 1]);
          _mm_storeu_pd(&b_A[jj - 1], _mm_div_pd(r, _mm_set1_pd(s[q])));
        }
        for (jj = qp1jj; jj <= qjj; jj++) {
          b_A[jj - 1] /= s[q];
        }
      }
      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (jj = qp1; jj < 5; jj++) {
      qjj = q + ((jj - 1) << 2);
      if (apply_transform) {
        d_st.site = &ej_emlrtRSI;
        nrm = c_xdotc(&d_st, 4 - q, b_A, iter + 1, b_A, qjj + 1);
        nrm = -(nrm / b_A[iter]);
        e_xaxpy(4 - q, nrm, iter + 1, b_A, qjj + 1);
      }
      e[jj - 1] = b_A[qjj];
    }
    for (jj = q + 1; jj < 5; jj++) {
      qjj = (jj + m) - 1;
      U[qjj] = b_A[qjj];
    }
    if (q + 1 <= 2) {
      d_st.site = &dj_emlrtRSI;
      nrm = g_xnrm2(&d_st, 3 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        d_st.site = &cj_emlrtRSI;
        nrm = e[q];
        if (muDoubleScalarAbs(e[q]) >= 1.0020841800044864E-292) {
          e_st.site = &hj_emlrtRSI;
          i_xscal(&e_st, 3 - q, 1.0 / e[q], e, q + 2);
        } else {
          qjj = ((((3 - q) / 2) << 1) + q) + 2;
          qp1jj = qjj - 2;
          for (jj = qp1; jj <= qp1jj; jj += 2) {
            r = _mm_loadu_pd(&e[jj - 1]);
            _mm_storeu_pd(&e[jj - 1], _mm_div_pd(r, _mm_set1_pd(nrm)));
          }
          for (jj = qjj; jj < 5; jj++) {
            e[jj - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (jj = qp1; jj < 5; jj++) {
          work[jj - 1] = 0.0;
        }
        for (jj = qp1; jj < 5; jj++) {
          f_xaxpy(3 - q, e[jj - 1], b_A, (q + ((jj - 1) << 2)) + 2, work,
                  q + 2);
        }
        for (jj = qp1; jj < 5; jj++) {
          g_xaxpy(3 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A,
                  (q + ((jj - 1) << 2)) + 2);
        }
      }
      for (jj = qp1; jj < 5; jj++) {
        V[(jj + m) - 1] = e[jj - 1];
      }
    }
  }
  m = 2;
  s[3] = b_A[15];
  e[2] = b_A[14];
  e[3] = 0.0;
  U[12] = 0.0;
  U[13] = 0.0;
  U[14] = 0.0;
  U[15] = 1.0;
  for (q = 2; q >= 0; q--) {
    qp1 = q + 2;
    iter = q << 2;
    qq = q + iter;
    if (s[q] != 0.0) {
      for (jj = qp1; jj < 5; jj++) {
        qjj = (q + ((jj - 1) << 2)) + 1;
        d_st.site = &bj_emlrtRSI;
        nrm = c_xdotc(&d_st, 4 - q, U, qq + 1, U, qjj);
        nrm = -(nrm / U[qq]);
        e_xaxpy(4 - q, nrm, qq + 1, U, qjj);
      }
      vectorUB = ((((4 - q) / 2) << 1) + q) + 1;
      qjj = vectorUB - 2;
      for (jj = q + 1; jj <= qjj; jj += 2) {
        qp1jj = (jj + iter) - 1;
        r = _mm_loadu_pd(&U[qp1jj]);
        _mm_storeu_pd(&U[qp1jj], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (jj = vectorUB; jj < 5; jj++) {
        qjj = (jj + iter) - 1;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      d_st.site = &aj_emlrtRSI;
      for (jj = 0; jj < q; jj++) {
        U[jj + iter] = 0.0;
      }
    } else {
      U[iter] = 0.0;
      U[iter + 1] = 0.0;
      U[iter + 2] = 0.0;
      U[iter + 3] = 0.0;
      U[qq] = 1.0;
    }
  }
  for (q = 3; q >= 0; q--) {
    if ((q + 1 <= 2) && (e[q] != 0.0)) {
      qp1 = q + 2;
      qjj = (q + (q << 2)) + 2;
      for (jj = qp1; jj < 5; jj++) {
        qp1jj = (q + ((jj - 1) << 2)) + 2;
        d_st.site = &yi_emlrtRSI;
        nrm = c_xdotc(&d_st, 3 - q, V, qjj, V, qp1jj);
        nrm = -(nrm / V[qjj - 1]);
        e_xaxpy(3 - q, nrm, qjj, V, qp1jj);
      }
    }
    qjj = q << 2;
    V[qjj] = 0.0;
    V[qjj + 1] = 0.0;
    V[qjj + 2] = 0.0;
    V[qjj + 3] = 0.0;
    V[q + qjj] = 1.0;
  }
  if (s[0] != 0.0) {
    rt = muDoubleScalarAbs(s[0]);
    nrm = s[0] / rt;
    s[0] = rt;
    e[0] /= nrm;
    d_st.site = &xi_emlrtRSI;
    j_xscal(&d_st, nrm, U, 1);
  }
  if (e[0] != 0.0) {
    rt = muDoubleScalarAbs(e[0]);
    nrm = rt / e[0];
    e[0] = rt;
    s[1] *= nrm;
    d_st.site = &wi_emlrtRSI;
    j_xscal(&d_st, nrm, V, 5);
  }
  if (s[1] != 0.0) {
    rt = muDoubleScalarAbs(s[1]);
    nrm = s[1] / rt;
    s[1] = rt;
    e[1] /= nrm;
    d_st.site = &xi_emlrtRSI;
    j_xscal(&d_st, nrm, U, 5);
  }
  if (e[1] != 0.0) {
    rt = muDoubleScalarAbs(e[1]);
    nrm = rt / e[1];
    e[1] = rt;
    s[2] *= nrm;
    d_st.site = &wi_emlrtRSI;
    j_xscal(&d_st, nrm, V, 9);
  }
  if (s[2] != 0.0) {
    rt = muDoubleScalarAbs(s[2]);
    nrm = s[2] / rt;
    s[2] = rt;
    e[2] = b_A[14] / nrm;
    d_st.site = &xi_emlrtRSI;
    j_xscal(&d_st, nrm, U, 9);
  }
  if (e[2] != 0.0) {
    rt = muDoubleScalarAbs(e[2]);
    nrm = rt / e[2];
    e[2] = rt;
    s[3] = b_A[15] * nrm;
    d_st.site = &wi_emlrtRSI;
    j_xscal(&d_st, nrm, V, 13);
  }
  if (s[3] != 0.0) {
    rt = muDoubleScalarAbs(s[3]);
    nrm = s[3] / rt;
    s[3] = rt;
    d_st.site = &xi_emlrtRSI;
    j_xscal(&d_st, nrm, U, 13);
  }
  iter = 0;
  snorm = muDoubleScalarMax(
      muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarMax(s[0], e[0]),
                                          muDoubleScalarMax(s[1], e[1])),
                        muDoubleScalarMax(s[2], e[2])),
      muDoubleScalarMax(s[3], 0.0));
  exitg1 = false;
  while ((!exitg1) && (m + 2 > 0)) {
    if (iter >= 75) {
      emlrtErrorWithMessageIdR2018a(&c_st, &j_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    } else {
      boolean_T exitg2;
      qq = m + 1;
      exitg2 = false;
      while (!(exitg2 || (qq == 0))) {
        nrm = muDoubleScalarAbs(e[qq - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (muDoubleScalarAbs(s[qq - 1]) +
                                              muDoubleScalarAbs(s[qq]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[qq - 1] = 0.0;
          exitg2 = true;
        } else {
          qq--;
        }
      }
      if (qq == m + 1) {
        qjj = 4;
      } else {
        qp1jj = m + 2;
        qjj = m + 2;
        exitg2 = false;
        while ((!exitg2) && (qjj >= qq)) {
          qp1jj = qjj;
          if (qjj == qq) {
            exitg2 = true;
          } else {
            nrm = 0.0;
            if (qjj < m + 2) {
              nrm = muDoubleScalarAbs(e[qjj - 1]);
            }
            if (qjj > qq + 1) {
              nrm += muDoubleScalarAbs(e[qjj - 2]);
            }
            rt = muDoubleScalarAbs(s[qjj - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              s[qjj - 1] = 0.0;
              exitg2 = true;
            } else {
              qjj--;
            }
          }
        }
        if (qp1jj == qq) {
          qjj = 3;
        } else if (qp1jj == m + 2) {
          qjj = 1;
        } else {
          qjj = 2;
          qq = qp1jj;
        }
      }
      switch (qjj) {
      case 1:
        f = e[m];
        e[m] = 0.0;
        qjj = m + 1;
        for (jj = qjj; jj >= qq + 1; jj--) {
          d_st.site = &vi_emlrtRSI;
          nrm = 0.0;
          rt = 0.0;
          drotg(&s[jj - 1], &f, &nrm, &rt);
          if (jj > qq + 1) {
            b = e[jj - 2];
            f = -rt * b;
            e[jj - 2] = b * nrm;
          }
          c_xrot(V, ((jj - 1) << 2) + 1, ((m + 1) << 2) + 1, nrm, rt);
        }
        break;
      case 2:
        f = e[qq - 1];
        e[qq - 1] = 0.0;
        d_st.site = &ui_emlrtRSI;
        for (jj = qq + 1; jj <= m + 2; jj++) {
          d_st.site = &ti_emlrtRSI;
          nrm = 0.0;
          rt = 0.0;
          drotg(&s[jj - 1], &f, &nrm, &rt);
          b = e[jj - 1];
          f = -rt * b;
          e[jj - 1] = b * nrm;
          c_xrot(U, ((jj - 1) << 2) + 1, ((qq - 1) << 2) + 1, nrm, rt);
        }
        break;
      case 3: {
        real_T sqds;
        qjj = m + 1;
        nrm = s[m + 1];
        scale = muDoubleScalarMax(
            muDoubleScalarMax(
                muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarAbs(nrm),
                                                    muDoubleScalarAbs(s[m])),
                                  muDoubleScalarAbs(e[m])),
                muDoubleScalarAbs(s[qq])),
            muDoubleScalarAbs(e[qq]));
        sm = nrm / scale;
        nrm = s[m] / scale;
        rt = e[m] / scale;
        sqds = s[qq] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          d_st.site = &si_emlrtRSI;
          rt = muDoubleScalarSqrt(b * b + nrm);
          if (b < 0.0) {
            rt = -rt;
          }
          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }
        f = (sqds + sm) * (sqds - sm) + rt;
        nrm = sqds * (e[qq] / scale);
        d_st.site = &ri_emlrtRSI;
        for (jj = qq + 1; jj <= qjj; jj++) {
          d_st.site = &qi_emlrtRSI;
          b = 0.0;
          sm = 0.0;
          drotg(&f, &nrm, &b, &sm);
          if (jj > qq + 1) {
            e[jj - 2] = f;
          }
          nrm = e[jj - 1];
          rt = s[jj - 1];
          scale = b * rt + sm * nrm;
          e[jj - 1] = b * nrm - sm * rt;
          nrm = s[jj];
          rt = sm * nrm;
          sqds = nrm * b;
          qp1jj = ((jj - 1) << 2) + 1;
          vectorUB = (jj << 2) + 1;
          c_xrot(V, qp1jj, vectorUB, b, sm);
          d_st.site = &pi_emlrtRSI;
          b = 0.0;
          sm = 0.0;
          drotg(&scale, &rt, &b, &sm);
          s[jj - 1] = scale;
          nrm = e[jj - 1];
          f = b * nrm + sm * sqds;
          sqds = -sm * nrm + b * sqds;
          s[jj] = sqds;
          nrm = sm * e[jj];
          e[jj] *= b;
          c_xrot(U, qp1jj, vectorUB, b, sm);
        }
        e[m] = f;
        iter++;
      } break;
      default:
        if (s[qq] < 0.0) {
          s[qq] = -s[qq];
          d_st.site = &oi_emlrtRSI;
          k_xscal(&d_st, V, (qq << 2) + 1);
        }
        qp1 = qq + 1;
        while ((qq + 1 < 4) && (s[qq] < s[qp1])) {
          rt = s[qq];
          s[qq] = s[qp1];
          s[qp1] = rt;
          qp1jj = (qq << 2) + 1;
          qjj = ((qq + 1) << 2) + 1;
          d_xswap(V, qp1jj, qjj);
          d_xswap(U, qp1jj, qjj);
          qq = qp1;
          qp1++;
        }
        iter = 0;
        m--;
        break;
      }
    }
  }
  if (doscale) {
    e_xzlascl(cscale, anrm, s);
  }
}

void svd(const emlrtStack *sp, const real_T A[9], real_T U[9], real_T s[3],
         real_T V[9])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b_A[9];
  real_T e[3];
  real_T work[3];
  real_T anrm;
  real_T b;
  real_T cscale;
  real_T f;
  real_T nrm;
  real_T rt;
  real_T scale;
  real_T sm;
  real_T snorm;
  int32_T iter;
  int32_T jj;
  int32_T m;
  int32_T q;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  int32_T qs;
  int32_T vectorUB;
  boolean_T doscale;
  boolean_T exitg1;
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
  st.site = &li_emlrtRSI;
  memcpy(&b_A[0], &A[0], 9U * sizeof(real_T));
  b_st.site = &mi_emlrtRSI;
  c_st.site = &ni_emlrtRSI;
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  memset(&U[0], 0, 9U * sizeof(real_T));
  memset(&V[0], 0, 9U * sizeof(real_T));
  doscale = false;
  anrm = xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
    doscale = true;
    cscale = 6.7178761075670888E-139;
    f_xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.4885657073574029E+138) {
    doscale = true;
    cscale = 1.4885657073574029E+138;
    f_xzlascl(anrm, cscale, b_A);
  }
  for (q = 0; q < 2; q++) {
    boolean_T apply_transform;
    qp1 = q + 2;
    iter = q + 3 * q;
    qq = iter + 1;
    apply_transform = false;
    d_st.site = &gj_emlrtRSI;
    nrm = b_xnrm2(&d_st, 3 - q, b_A, iter + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      d_st.site = &fj_emlrtRSI;
      if (muDoubleScalarAbs(nrm) >= 1.0020841800044864E-292) {
        e_st.site = &hj_emlrtRSI;
        b_xscal(&e_st, 3 - q, 1.0 / nrm, b_A, iter + 1);
      } else {
        qjj = (iter - q) + 3;
        qs = ((((qjj - iter) / 2) << 1) + iter) + 1;
        vectorUB = qs - 2;
        for (jj = qq; jj <= vectorUB; jj += 2) {
          r = _mm_loadu_pd(&b_A[jj - 1]);
          _mm_storeu_pd(&b_A[jj - 1], _mm_div_pd(r, _mm_set1_pd(s[q])));
        }
        for (jj = qs; jj <= qjj; jj++) {
          b_A[jj - 1] /= s[q];
        }
      }
      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (jj = qp1; jj < 4; jj++) {
      qjj = q + 3 * (jj - 1);
      if (apply_transform) {
        d_st.site = &ej_emlrtRSI;
        nrm = b_xdotc(&d_st, 3 - q, b_A, iter + 1, b_A, qjj + 1);
        nrm = -(nrm / b_A[iter]);
        xaxpy(3 - q, nrm, iter + 1, b_A, qjj + 1);
      }
      e[jj - 1] = b_A[qjj];
    }
    for (jj = q + 1; jj < 4; jj++) {
      qjj = (jj + 3 * q) - 1;
      U[qjj] = b_A[qjj];
    }
    if (q <= 0) {
      d_st.site = &dj_emlrtRSI;
      nrm = j_xnrm2(e);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        d_st.site = &cj_emlrtRSI;
        nrm = e[0];
        if (muDoubleScalarAbs(e[0]) >= 1.0020841800044864E-292) {
          l_xscal(1.0 / e[0], e);
        } else {
          for (jj = qp1; jj <= 2; jj += 2) {
            r = _mm_loadu_pd(&e[1]);
            _mm_storeu_pd(&e[1], _mm_div_pd(r, _mm_set1_pd(nrm)));
          }
        }
        e[1]++;
        e[0] = -e[0];
        for (jj = qp1; jj < 4; jj++) {
          work[jj - 1] = 0.0;
        }
        for (jj = qp1; jj < 4; jj++) {
          h_xaxpy(e[jj - 1], b_A, 3 * (jj - 1) + 2, work);
        }
        for (jj = qp1; jj < 4; jj++) {
          i_xaxpy(-e[jj - 1] / e[1], work, b_A, 3 * (jj - 1) + 2);
        }
      }
      for (jj = qp1; jj < 4; jj++) {
        V[jj - 1] = e[jj - 1];
      }
    }
  }
  m = 1;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (q = 1; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 3 * q;
    if (s[q] != 0.0) {
      for (jj = qp1; jj < 4; jj++) {
        qjj = (q + 3 * (jj - 1)) + 1;
        d_st.site = &bj_emlrtRSI;
        nrm = b_xdotc(&d_st, 3 - q, U, qq + 1, U, qjj);
        nrm = -(nrm / U[qq]);
        xaxpy(3 - q, nrm, qq + 1, U, qjj);
      }
      vectorUB = q + 3;
      qjj = q + 1;
      for (jj = q + 1; jj <= qjj; jj += 2) {
        qs = (jj + 3 * q) - 1;
        r = _mm_loadu_pd(&U[qs]);
        _mm_storeu_pd(&U[qs], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (jj = vectorUB; jj < 4; jj++) {
        qjj = 3 * q + 2;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      d_st.site = &aj_emlrtRSI;
      if (q - 1 >= 0) {
        U[3 * q] = 0.0;
      }
    } else {
      U[3 * q] = 0.0;
      U[3 * q + 1] = 0.0;
      U[3 * q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }
  for (jj = 2; jj >= 0; jj--) {
    if ((jj <= 0) && (e[0] != 0.0)) {
      d_st.site = &yi_emlrtRSI;
      nrm = b_xdotc(&d_st, 2, V, 2, V, 5);
      nrm = -(nrm / V[1]);
      xaxpy(2, nrm, 2, V, 5);
      d_st.site = &yi_emlrtRSI;
      nrm = b_xdotc(&d_st, 2, V, 2, V, 8);
      nrm = -(nrm / V[1]);
      xaxpy(2, nrm, 2, V, 8);
    }
    V[3 * jj] = 0.0;
    V[3 * jj + 1] = 0.0;
    V[3 * jj + 2] = 0.0;
    V[jj + 3 * jj] = 1.0;
  }
  if (s[0] != 0.0) {
    rt = muDoubleScalarAbs(s[0]);
    nrm = s[0] / rt;
    s[0] = rt;
    e[0] /= nrm;
    d_st.site = &xi_emlrtRSI;
    c_xscal(&d_st, nrm, U, 1);
  }
  if (e[0] != 0.0) {
    rt = muDoubleScalarAbs(e[0]);
    nrm = rt / e[0];
    e[0] = rt;
    s[1] *= nrm;
    d_st.site = &wi_emlrtRSI;
    c_xscal(&d_st, nrm, V, 4);
  }
  if (s[1] != 0.0) {
    rt = muDoubleScalarAbs(s[1]);
    nrm = s[1] / rt;
    s[1] = rt;
    e[1] = b_A[7] / nrm;
    d_st.site = &xi_emlrtRSI;
    c_xscal(&d_st, nrm, U, 4);
  }
  if (e[1] != 0.0) {
    rt = muDoubleScalarAbs(e[1]);
    nrm = rt / e[1];
    e[1] = rt;
    s[2] = b_A[8] * nrm;
    d_st.site = &wi_emlrtRSI;
    c_xscal(&d_st, nrm, V, 7);
  }
  if (s[2] != 0.0) {
    rt = muDoubleScalarAbs(s[2]);
    nrm = s[2] / rt;
    s[2] = rt;
    d_st.site = &xi_emlrtRSI;
    c_xscal(&d_st, nrm, U, 7);
  }
  iter = 0;
  snorm = muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarMax(s[0], e[0]),
                                              muDoubleScalarMax(s[1], e[1])),
                            muDoubleScalarMax(s[2], 0.0));
  exitg1 = false;
  while ((!exitg1) && (m + 2 > 0)) {
    if (iter >= 75) {
      emlrtErrorWithMessageIdR2018a(&c_st, &j_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    } else {
      boolean_T exitg2;
      qq = m + 1;
      exitg2 = false;
      while (!(exitg2 || (qq == 0))) {
        nrm = muDoubleScalarAbs(e[qq - 1]);
        if ((nrm <= 2.2204460492503131E-16 * (muDoubleScalarAbs(s[qq - 1]) +
                                              muDoubleScalarAbs(s[qq]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[qq - 1] = 0.0;
          exitg2 = true;
        } else {
          qq--;
        }
      }
      if (qq == m + 1) {
        qjj = 4;
      } else {
        qs = m + 2;
        qjj = m + 2;
        exitg2 = false;
        while ((!exitg2) && (qjj >= qq)) {
          qs = qjj;
          if (qjj == qq) {
            exitg2 = true;
          } else {
            nrm = 0.0;
            if (qjj < m + 2) {
              nrm = muDoubleScalarAbs(e[qjj - 1]);
            }
            if (qjj > qq + 1) {
              nrm += muDoubleScalarAbs(e[qjj - 2]);
            }
            rt = muDoubleScalarAbs(s[qjj - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              s[qjj - 1] = 0.0;
              exitg2 = true;
            } else {
              qjj--;
            }
          }
        }
        if (qs == qq) {
          qjj = 3;
        } else if (qs == m + 2) {
          qjj = 1;
        } else {
          qjj = 2;
          qq = qs;
        }
      }
      switch (qjj) {
      case 1:
        f = e[m];
        e[m] = 0.0;
        qjj = m + 1;
        for (jj = qjj; jj >= qq + 1; jj--) {
          d_st.site = &vi_emlrtRSI;
          nrm = 0.0;
          rt = 0.0;
          drotg(&s[jj - 1], &f, &nrm, &rt);
          if (jj > qq + 1) {
            f = -rt * e[0];
            e[0] *= nrm;
          }
          xrot(V, 3 * (jj - 1) + 1, 3 * (m + 1) + 1, nrm, rt);
        }
        break;
      case 2:
        f = e[qq - 1];
        e[qq - 1] = 0.0;
        d_st.site = &ui_emlrtRSI;
        for (jj = qq + 1; jj <= m + 2; jj++) {
          d_st.site = &ti_emlrtRSI;
          nrm = 0.0;
          rt = 0.0;
          drotg(&s[jj - 1], &f, &nrm, &rt);
          b = e[jj - 1];
          f = -rt * b;
          e[jj - 1] = b * nrm;
          xrot(U, 3 * (jj - 1) + 1, 3 * (qq - 1) + 1, nrm, rt);
        }
        break;
      case 3: {
        real_T sqds;
        qjj = m + 1;
        nrm = s[m + 1];
        scale = muDoubleScalarMax(
            muDoubleScalarMax(
                muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarAbs(nrm),
                                                    muDoubleScalarAbs(s[m])),
                                  muDoubleScalarAbs(e[m])),
                muDoubleScalarAbs(s[qq])),
            muDoubleScalarAbs(e[qq]));
        sm = nrm / scale;
        nrm = s[m] / scale;
        rt = e[m] / scale;
        sqds = s[qq] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          d_st.site = &si_emlrtRSI;
          rt = muDoubleScalarSqrt(b * b + nrm);
          if (b < 0.0) {
            rt = -rt;
          }
          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }
        f = (sqds + sm) * (sqds - sm) + rt;
        nrm = sqds * (e[qq] / scale);
        d_st.site = &ri_emlrtRSI;
        for (jj = qq + 1; jj <= qjj; jj++) {
          d_st.site = &qi_emlrtRSI;
          b = 0.0;
          sm = 0.0;
          drotg(&f, &nrm, &b, &sm);
          if (jj > qq + 1) {
            e[0] = f;
          }
          nrm = e[jj - 1];
          rt = s[jj - 1];
          scale = b * rt + sm * nrm;
          e[jj - 1] = b * nrm - sm * rt;
          nrm = s[jj];
          rt = sm * nrm;
          sqds = nrm * b;
          qs = 3 * (jj - 1) + 1;
          vectorUB = 3 * jj + 1;
          xrot(V, qs, vectorUB, b, sm);
          d_st.site = &pi_emlrtRSI;
          b = 0.0;
          sm = 0.0;
          drotg(&scale, &rt, &b, &sm);
          s[jj - 1] = scale;
          nrm = e[jj - 1];
          f = b * nrm + sm * sqds;
          sqds = -sm * nrm + b * sqds;
          s[jj] = sqds;
          nrm = sm * e[jj];
          e[jj] *= b;
          xrot(U, qs, vectorUB, b, sm);
        }
        e[m] = f;
        iter++;
      } break;
      default:
        if (s[qq] < 0.0) {
          s[qq] = -s[qq];
          d_st.site = &oi_emlrtRSI;
          d_xscal(&d_st, V, 3 * qq + 1);
        }
        qp1 = qq + 1;
        while ((qq + 1 < 3) && (s[qq] < s[qp1])) {
          rt = s[qq];
          s[qq] = s[qp1];
          s[qp1] = rt;
          qs = 3 * qq + 1;
          qjj = 3 * (qq + 1) + 1;
          b_xswap(V, qs, qjj);
          b_xswap(U, qs, qjj);
          qq = qp1;
          qp1++;
        }
        iter = 0;
        m--;
        break;
      }
    }
  }
  if (doscale) {
    c_xzlascl(cscale, anrm, s);
  }
}

/* End of code generation (svd.c) */
