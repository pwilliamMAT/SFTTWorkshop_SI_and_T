/*
 * eig.c
 *
 * Code generation for function 'eig'
 *
 */

/* Include files */
#include "eig.h"
#include "eigSkewHermitianStandard.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bd_emlrtRSI = {
    127,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo cd_emlrtRSI = {
    135,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo dd_emlrtRSI = {
    143,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo hd_emlrtRSI = {
    13,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

static emlrtRSInfo jd_emlrtRSI = {
    8,         /* lineNo */
    "xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

static emlrtRSInfo sf_emlrtRSI = {
    34,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

static emlrtRSInfo vf_emlrtRSI = {
    42,      /* lineNo */
    "xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

/* Function Definitions */
void eig(const emlrtStack *sp, const real_T A[36], creal_T V[6])
{
  ptrdiff_t ihi_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T b_A[36];
  real_T scale[6];
  real_T wimag[6];
  real_T wreal[6];
  real_T abnrm;
  real_T rconde;
  real_T rcondv;
  real_T vleft;
  real_T vright;
  int32_T i;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &ad_emlrtRSI;
  b_st.site = &ed_emlrtRSI;
  c_st.site = &fd_emlrtRSI;
  p = true;
  for (i = 0; i < 36; i++) {
    if (p) {
      vleft = A[i];
      if (muDoubleScalarIsInf(vleft) || muDoubleScalarIsNaN(vleft)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (i = 0; i < 6; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
  } else {
    int32_T b_i;
    int32_T exitg1;
    int32_T j;
    boolean_T exitg2;
    p = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 6)) {
      b_i = 0;
      do {
        exitg1 = 0;
        if (b_i <= j) {
          if (!(A[b_i + 6 * j] == A[j + 6 * b_i])) {
            p = false;
            exitg1 = 1;
          } else {
            b_i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (p) {
      st.site = &bd_emlrtRSI;
      b_st.site = &hd_emlrtRSI;
      c_st.site = &jd_emlrtRSI;
      memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
      n_t = (ptrdiff_t)6;
      n_t = LAPACKE_dsyev(102, 'N', 'L', n_t, &b_A[0], n_t, &scale[0]);
      d_st.site = &kd_emlrtRSI;
      if ((int32_T)n_t < 0) {
        if ((int32_T)n_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&d_st, &t_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &cv1[0], 12, (int32_T)n_t);
        }
      }
      for (i = 0; i < 6; i++) {
        V[i].re = scale[i];
        V[i].im = 0.0;
      }
      if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&st))) {
        b_st.site = &id_emlrtRSI;
        warning(&b_st);
      }
    } else {
      p = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 6)) {
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= j) {
            if (!(A[b_i + 6 * j] == -A[j + 6 * b_i])) {
              p = false;
              exitg1 = 1;
            } else {
              b_i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (p) {
        st.site = &cd_emlrtRSI;
        eigSkewHermitianStandard(&st, A, V);
      } else {
        st.site = &dd_emlrtRSI;
        b_st.site = &sf_emlrtRSI;
        c_st.site = &vf_emlrtRSI;
        memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
        n_t = LAPACKE_dgeevx(102, 'B', 'N', 'N', 'N', (ptrdiff_t)6, &b_A[0],
                             (ptrdiff_t)6, &wreal[0], &wimag[0], &vleft,
                             (ptrdiff_t)1, &vright, (ptrdiff_t)1, &n_t, &ihi_t,
                             &scale[0], &abnrm, &rconde, &rcondv);
        d_st.site = &uf_emlrtRSI;
        if ((int32_T)n_t < 0) {
          if ((int32_T)n_t == -1010) {
            emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI, "MATLAB:nomem",
                                          "MATLAB:nomem", 0);
          } else {
            emlrtErrorWithMessageIdR2018a(&d_st, &t_emlrtRTEI,
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          "Coder:toolbox:LAPACKCallErrorInfo",
                                          5, 4, 14, &cv2[0], 12, (int32_T)n_t);
          }
        }
        for (i = 0; i < 6; i++) {
          V[i].re = wreal[i];
          V[i].im = wimag[i];
        }
        if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&st))) {
          b_st.site = &tf_emlrtRSI;
          warning(&b_st);
        }
      }
    }
  }
}

/* End of code generation (eig.c) */
