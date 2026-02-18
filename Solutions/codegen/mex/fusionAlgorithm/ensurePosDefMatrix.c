/*
 * ensurePosDefMatrix.c
 *
 * Code generation for function 'ensurePosDefMatrix'
 *
 */

/* Include files */
#include "ensurePosDefMatrix.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "xhseqr.h"
#include "xzgehrd.h"
#include "xzlarf.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ru_emlrtRSI = {
    19,                   /* lineNo */
    "ensurePosDefMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\ensurePosDefMatrix.m" /* pathName */
};

static emlrtRSInfo su_emlrtRSI = {
    125,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo tu_emlrtRSI = {
    133,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo uu_emlrtRSI = {
    141,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo vu_emlrtRSI = {
    27,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

static emlrtRSInfo wu_emlrtRSI = {
    10,        /* lineNo */
    "xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

static emlrtRSInfo xu_emlrtRSI = {
    34,        /* lineNo */
    "xzungqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzungqr.m" /* pathName */
};

static emlrtRSInfo bv_emlrtRSI = {
    12,                         /* lineNo */
    "eigSkewHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigSkew"
    "HermitianStandard.m" /* pathName */
};

static emlrtRSInfo cv_emlrtRSI = {
    22,                             /* lineNo */
    "eigRealSkewSymmetricStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigReal"
    "SkewSymmetricStandard.m" /* pathName */
};

static emlrtRSInfo dv_emlrtRSI = {
    66,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo ev_emlrtRSI = {
    69,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo fv_emlrtRSI = {
    70,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo gv_emlrtRSI = {
    14,          /* lineNo */
    "xungorghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xungorghr.m" /* pathName */
};

static emlrtRSInfo hv_emlrtRSI = {
    15,        /* lineNo */
    "xzunghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzunghr.m" /* pathName */
};

static emlrtRSInfo iv_emlrtRSI = {
    53,        /* lineNo */
    "xzunghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzunghr.m" /* pathName */
};

static emlrtRSInfo lv_emlrtRSI = {
    26,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

static emlrtRSInfo mv_emlrtRSI = {
    40,      /* lineNo */
    "xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

/* Function Definitions */
void ensurePosDefMatrix(const emlrtStack *sp, real_T P[36])
{
  ptrdiff_t ihi_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  creal_T D[36];
  creal_T V[36];
  creal_T b_V[36];
  real_T A[36];
  real_T vright[36];
  real_T scale[6];
  real_T wimag[6];
  real_T wreal[6];
  real_T abnrm;
  real_T lambda;
  real_T rconde;
  real_T rcondv;
  int32_T A_tmp;
  int32_T b_i;
  int32_T i;
  int32_T istart;
  int32_T j;
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  st.site = &ru_emlrtRSI;
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      A_tmp = j + 6 * i;
      A[A_tmp] = (P[A_tmp] + P[i + 6 * j]) / 2.0;
    }
  }
  b_st.site = &yc_emlrtRSI;
  c_st.site = &dd_emlrtRSI;
  d_st.site = &ed_emlrtRSI;
  p = true;
  e_st.site = &fd_emlrtRSI;
  for (i = 0; i < 36; i++) {
    if (p) {
      lambda = A[i];
      if (muDoubleScalarIsInf(lambda) || muDoubleScalarIsNaN(lambda)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (i = 0; i < 36; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
      D[i].re = 0.0;
      D[i].im = 0.0;
    }
    for (i = 0; i < 6; i++) {
      istart = i + 6 * i;
      D[istart].re = rtNaN;
      D[istart].im = 0.0;
    }
  } else {
    int32_T exitg1;
    boolean_T exitg2;
    p = true;
    istart = 0;
    exitg2 = false;
    while ((!exitg2) && (istart < 6)) {
      A_tmp = 0;
      do {
        exitg1 = 0;
        if (A_tmp <= istart) {
          if (!(A[A_tmp + 6 * istart] == A[istart + 6 * A_tmp])) {
            p = false;
            exitg1 = 1;
          } else {
            A_tmp++;
          }
        } else {
          istart++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (p) {
      b_st.site = &su_emlrtRSI;
      c_st.site = &vu_emlrtRSI;
      d_st.site = &wu_emlrtRSI;
      n_t = (ptrdiff_t)6;
      n_t = LAPACKE_dsyev(102, 'V', 'L', n_t, &A[0], n_t, &scale[0]);
      e_st.site = &jd_emlrtRSI;
      if ((int32_T)n_t < 0) {
        if ((int32_T)n_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&e_st, &p_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &cv1[0], 12, (int32_T)n_t);
        }
      }
      memset(&D[0], 0, 36U * sizeof(creal_T));
      for (i = 0; i < 6; i++) {
        istart = i + 6 * i;
        D[istart].re = scale[i];
        D[istart].im = 0.0;
      }
      for (i = 0; i < 36; i++) {
        V[i].re = A[i];
        V[i].im = 0.0;
      }
      if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&b_st))) {
        c_st.site = &hd_emlrtRSI;
        warning(&c_st);
      }
    } else {
      p = true;
      istart = 0;
      exitg2 = false;
      while ((!exitg2) && (istart < 6)) {
        A_tmp = 0;
        do {
          exitg1 = 0;
          if (A_tmp <= istart) {
            if (!(A[A_tmp + 6 * istart] == -A[istart + 6 * A_tmp])) {
              p = false;
              exitg1 = 1;
            } else {
              A_tmp++;
            }
          } else {
            istart++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (p) {
        int32_T itau;
        int32_T sgn;
        int32_T vectorUB;
        b_st.site = &tu_emlrtRSI;
        c_st.site = &bv_emlrtRSI;
        d_st.site = &cv_emlrtRSI;
        e_st.site = &yd_emlrtRSI;
        f_st.site = &dd_emlrtRSI;
        g_st.site = &ed_emlrtRSI;
        p = true;
        for (i = 0; i < 36; i++) {
          if (p) {
            lambda = A[i];
            if (muDoubleScalarIsInf(lambda) || muDoubleScalarIsNaN(lambda)) {
              p = false;
            }
          } else {
            p = false;
          }
        }
        if (!p) {
          for (i = 0; i < 36; i++) {
            vright[i] = rtNaN;
          }
          istart = 2;
          for (i = 0; i < 5; i++) {
            if (istart <= 6) {
              memset(&vright[(i * 6 + istart) + -1], 0,
                     (uint32_T)(-istart + 7) * sizeof(real_T));
            }
            istart++;
          }
          for (i = 0; i < 36; i++) {
            A[i] = rtNaN;
          }
        } else {
          real_T tau[5];
          e_st.site = &dv_emlrtRSI;
          f_st.site = &de_emlrtRSI;
          xzgehrd(&f_st, A, tau);
          e_st.site = &ev_emlrtRSI;
          memcpy(&vright[0], &A[0], 36U * sizeof(real_T));
          f_st.site = &gv_emlrtRSI;
          for (j = 4; j >= 0; j--) {
            sgn = (j + 1) * 6;
            g_st.site = &hv_emlrtRSI;
            for (i = 0; i <= j; i++) {
              vright[sgn + i] = 0.0;
            }
            istart = j + 3;
            for (i = istart; i < 7; i++) {
              A_tmp = sgn + i;
              vright[A_tmp - 1] = vright[A_tmp - 7];
            }
          }
          for (i = 0; i < 6; i++) {
            vright[i] = 0.0;
          }
          vright[0] = 1.0;
          g_st.site = &iv_emlrtRSI;
          itau = 4;
          for (i = 0; i < 6; i++) {
            scale[i] = 0.0;
          }
          for (b_i = 4; b_i >= 0; b_i--) {
            int32_T iaii;
            iaii = (b_i + b_i * 6) + 7;
            if (b_i + 1 < 5) {
              vright[iaii] = 1.0;
              h_st.site = &xu_emlrtRSI;
              xzlarf(&h_st, 5 - b_i, 4 - b_i, iaii + 1, tau[itau], vright,
                     iaii + 7, scale);
              istart = iaii + 2;
              A_tmp = (iaii - b_i) + 5;
              sgn = (((((A_tmp - iaii) - 1) / 2) << 1) + iaii) + 2;
              vectorUB = sgn - 2;
              for (i = istart; i <= vectorUB; i += 2) {
                __m128d r;
                r = _mm_loadu_pd(&vright[i - 1]);
                _mm_storeu_pd(&vright[i - 1],
                              _mm_mul_pd(_mm_set1_pd(-tau[itau]), r));
              }
              for (i = sgn; i <= A_tmp; i++) {
                vright[i - 1] *= -tau[itau];
              }
            }
            vright[iaii] = 1.0 - tau[itau];
            for (i = 0; i < b_i; i++) {
              vright[(iaii - i) - 1] = 0.0;
            }
            itau = b_i - 1;
          }
          e_st.site = &fv_emlrtRSI;
          istart = xhseqr(&e_st, A, vright);
          if ((istart != 0) && (!emlrtSetWarningFlag(&d_st))) {
            e_st.site = &ce_emlrtRSI;
            b_warning(&e_st);
          }
        }
        memset(&D[0], 0, 36U * sizeof(creal_T));
        istart = 1;
        do {
          exitg1 = 0;
          if (istart <= 6) {
            if (istart != 6) {
              A_tmp = istart + 6 * (istart - 1);
              lambda = A[A_tmp];
              if (lambda != 0.0) {
                lambda = muDoubleScalarAbs(lambda);
                D[A_tmp - 1].re = 0.0;
                D[A_tmp - 1].im = lambda;
                A_tmp = istart + 6 * istart;
                D[A_tmp].re = 0.0;
                D[A_tmp].im = -lambda;
                istart += 2;
              } else {
                istart++;
              }
            } else {
              istart++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        for (i = 0; i < 36; i++) {
          V[i].re = vright[i];
          V[i].im = 0.0;
        }
        istart = 1;
        do {
          exitg1 = 0;
          if (istart <= 6) {
            if (istart != 6) {
              A_tmp = 6 * (istart - 1);
              lambda = A[istart + A_tmp];
              if (lambda != 0.0) {
                if (lambda < 0.0) {
                  sgn = 1;
                } else {
                  sgn = -1;
                }
                for (i = 0; i < 6; i++) {
                  vectorUB = i + A_tmp;
                  lambda = V[vectorUB].re;
                  itau = i + 6 * istart;
                  abnrm = (real_T)sgn * V[itau].re;
                  if (abnrm == 0.0) {
                    V[vectorUB].re = lambda / 1.4142135623730951;
                    V[vectorUB].im = 0.0;
                  } else if (lambda == 0.0) {
                    V[vectorUB].re = 0.0;
                    V[vectorUB].im = abnrm / 1.4142135623730951;
                  } else {
                    V[vectorUB].re = lambda / 1.4142135623730951;
                    V[vectorUB].im = abnrm / 1.4142135623730951;
                  }
                  V[itau].re = V[vectorUB].re;
                  V[itau].im = -V[vectorUB].im;
                }
                istart += 2;
              } else {
                istart++;
              }
            } else {
              istart++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        creal_T d[6];
        b_st.site = &uu_emlrtRSI;
        c_st.site = &lv_emlrtRSI;
        d_st.site = &mv_emlrtRSI;
        n_t = LAPACKE_dgeevx(102, 'B', 'N', 'V', 'N', (ptrdiff_t)6, &A[0],
                             (ptrdiff_t)6, &wreal[0], &wimag[0], &lambda,
                             (ptrdiff_t)1, &vright[0], (ptrdiff_t)6, &n_t,
                             &ihi_t, &scale[0], &abnrm, &rconde, &rcondv);
        e_st.site = &tf_emlrtRSI;
        if ((int32_T)n_t < 0) {
          if ((int32_T)n_t == -1010) {
            emlrtErrorWithMessageIdR2018a(&e_st, &p_emlrtRTEI, "MATLAB:nomem",
                                          "MATLAB:nomem", 0);
          } else {
            emlrtErrorWithMessageIdR2018a(&e_st, &q_emlrtRTEI,
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          5, 4, 14, &cv2[0], 12, (int32_T)n_t);
          }
        }
        for (i = 0; i < 6; i++) {
          d[i].re = wreal[i];
          d[i].im = wimag[i];
        }
        for (i = 0; i < 36; i++) {
          V[i].re = vright[i];
          V[i].im = 0.0;
        }
        for (i = 0; i < 5; i++) {
          if ((wimag[i] > 0.0) && (wimag[i + 1] < 0.0)) {
            for (j = 0; j < 6; j++) {
              istart = j + 6 * i;
              A_tmp = j + 6 * (i + 1);
              lambda = V[A_tmp].re;
              V[istart].im = lambda;
              V[A_tmp].re = V[istart].re;
              V[A_tmp].im = -lambda;
            }
          }
        }
        memset(&D[0], 0, 36U * sizeof(creal_T));
        for (i = 0; i < 6; i++) {
          D[i + 6 * i] = d[i];
        }
        if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&b_st))) {
          c_st.site = &sf_emlrtRSI;
          warning(&c_st);
        }
      }
    }
  }
  for (i = 0; i < 6; i++) {
    wreal[i] = muDoubleScalarMax(D[i + 6 * i].re, 2.2204460492503131E-16);
  }
  memset(&vright[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    vright[i + 6 * i] = wreal[i];
  }
  for (i = 0; i < 36; i++) {
    D[i].re = vright[i];
    D[i].im = 0.0;
  }
  for (j = 0; j < 6; j++) {
    memset(&b_V[j * 6], 0, 6U * sizeof(creal_T));
    for (b_i = 0; b_i < 6; b_i++) {
      istart = b_i + 6 * j;
      lambda = D[istart].re;
      abnrm = D[istart].im;
      for (i = 0; i < 6; i++) {
        istart = i + 6 * b_i;
        rconde = V[istart].re;
        rcondv = V[istart].im;
        istart = i + 6 * j;
        b_V[istart].re += rconde * lambda - rcondv * abnrm;
        b_V[istart].im += rconde * abnrm + rcondv * lambda;
      }
    }
  }
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      lambda = 0.0;
      for (b_i = 0; b_i < 6; b_i++) {
        istart = j + 6 * b_i;
        A_tmp = i + 6 * b_i;
        lambda += b_V[A_tmp].re * V[istart].re - b_V[A_tmp].im * -V[istart].im;
      }
      P[i + 6 * j] = lambda;
    }
  }
}

/* End of code generation (ensurePosDefMatrix.c) */
