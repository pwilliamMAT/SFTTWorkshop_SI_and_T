/*
 * eig.c
 *
 * Code generation for function 'eig'
 *
 */

/* Include files */
#include "eig.h"
#include "anyNonFinite.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "warning.h"
#include "xdlahqr.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xhseqr.h"
#include "xzgebal.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ig_emlrtRSI = {
    54,      /* lineNo */
    "xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

static emlrtRSInfo jg_emlrtRSI = {
    49,       /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo kg_emlrtRSI = {
    67,       /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo lg_emlrtRSI = {
    73,       /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo mg_emlrtRSI = {
    130,      /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo ng_emlrtRSI = {
    131,      /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo og_emlrtRSI = {
    133,      /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo pg_emlrtRSI = {
    134,      /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo qg_emlrtRSI = {
    139,      /* lineNo */
    "xdgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdgeev.m" /* pathName */
};

static emlrtRSInfo hh_emlrtRSI = {
    28,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo ih_emlrtRSI = {
    37,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

/* Function Definitions */
void eig(const emlrtStack *sp, const real_T A[9], creal_T V[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T T[9];
  real_T work[3];
  real_T tau[2];
  real_T alpha1;
  real_T lambda;
  int32_T ihi;
  int32_T j;
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
  st.site = &be_emlrtRSI;
  if (anyNonFinite(A)) {
    V[0].re = rtNaN;
    V[0].im = 0.0;
    V[1].re = rtNaN;
    V[1].im = 0.0;
    V[2].re = rtNaN;
    V[2].im = 0.0;
  } else {
    int32_T exitg1;
    int32_T i;
    int32_T istart;
    boolean_T exitg2;
    boolean_T scalea;
    scalea = true;
    istart = 0;
    exitg2 = false;
    while ((!exitg2) && (istart < 3)) {
      i = 0;
      do {
        exitg1 = 0;
        if (i <= istart) {
          if (!(A[i + 3 * istart] == A[istart + 3 * i])) {
            scalea = false;
            exitg1 = 1;
          } else {
            i++;
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
    if (scalea) {
      ptrdiff_t n_t;
      st.site = &ce_emlrtRSI;
      b_st.site = &ie_emlrtRSI;
      c_st.site = &ke_emlrtRSI;
      memcpy(&T[0], &A[0], 9U * sizeof(real_T));
      n_t = (ptrdiff_t)3;
      n_t = LAPACKE_dsyev(102, 'N', 'L', n_t, &T[0], n_t, &work[0]);
      d_st.site = &rc_emlrtRSI;
      if ((int32_T)n_t < 0) {
        if ((int32_T)n_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&d_st, &h_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&d_st, &i_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &cv[0], 12, (int32_T)n_t);
        }
      }
      V[0].re = work[0];
      V[0].im = 0.0;
      V[1].re = work[1];
      V[1].im = 0.0;
      V[2].re = work[2];
      V[2].im = 0.0;
      if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&st))) {
        b_st.site = &je_emlrtRSI;
        warning(&b_st);
      }
    } else {
      scalea = true;
      istart = 0;
      exitg2 = false;
      while ((!exitg2) && (istart < 3)) {
        i = 0;
        do {
          exitg1 = 0;
          if (i <= istart) {
            if (!(A[i + 3 * istart] == -A[istart + 3 * i])) {
              scalea = false;
              exitg1 = 1;
            } else {
              i++;
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
      if (scalea) {
        st.site = &de_emlrtRSI;
        b_st.site = &le_emlrtRSI;
        c_st.site = &me_emlrtRSI;
        memcpy(&T[0], &A[0], 9U * sizeof(real_T));
        d_st.site = &ne_emlrtRSI;
        if (anyNonFinite(A)) {
          for (j = 0; j < 9; j++) {
            T[j] = rtNaN;
          }
          istart = 2;
          for (j = 0; j < 2; j++) {
            if (istart <= 3) {
              memset(&T[(j * 3 + istart) + -1], 0,
                     (uint32_T)(-istart + 4) * sizeof(real_T));
            }
            istart++;
          }
        } else {
          d_st.site = &oe_emlrtRSI;
          e_st.site = &re_emlrtRSI;
          work[0] = 0.0;
          work[1] = 0.0;
          work[2] = 0.0;
          for (j = 0; j < 2; j++) {
            int32_T b_in;
            int32_T in;
            int32_T lastv;
            in = (j + 1) * 3;
            b_in = j + 3 * j;
            lambda = T[b_in + 1];
            f_st.site = &se_emlrtRSI;
            alpha1 = xzlarfg(&f_st, 2 - j, &lambda, T, j * 3 + 3);
            tau[j] = alpha1;
            T[b_in + 1] = 1.0;
            f_st.site = &te_emlrtRSI;
            if (alpha1 != 0.0) {
              lastv = 2 - j;
              i = (b_in - j) + 2;
              while ((lastv > 0) && (T[i] == 0.0)) {
                lastv--;
                i--;
              }
              istart = 3;
              exitg2 = false;
              while ((!exitg2) && (istart > 0)) {
                int32_T ia;
                i = in + istart;
                ia = i;
                do {
                  exitg1 = 0;
                  if (ia <= i + (lastv - 1) * 3) {
                    if (T[ia - 1] != 0.0) {
                      exitg1 = 1;
                    } else {
                      ia += 3;
                    }
                  } else {
                    istart--;
                    exitg1 = 2;
                  }
                } while (exitg1 == 0);
                if (exitg1 == 1) {
                  exitg2 = true;
                }
              }
            } else {
              lastv = 0;
              istart = 0;
            }
            if (lastv > 0) {
              g_st.site = &ve_emlrtRSI;
              xgemv(&g_st, istart, lastv, T, in + 1, T, b_in + 2, work);
              g_st.site = &we_emlrtRSI;
              xgerc(&g_st, istart, lastv, -tau[j], work, b_in + 2, T, in + 1);
            }
            f_st.site = &ue_emlrtRSI;
            xzlarf(&f_st, 2 - j, 2 - j, b_in + 2, tau[j], T, (j + in) + 2,
                   work);
            T[b_in + 1] = lambda;
          }
          d_st.site = &pe_emlrtRSI;
          istart = xhseqr(&d_st, T);
          if ((istart != 0) && (!emlrtSetWarningFlag(&c_st))) {
            d_st.site = &qe_emlrtRSI;
            b_warning(&d_st);
          }
        }
        istart = 1;
        do {
          exitg1 = 0;
          if (istart <= 3) {
            boolean_T guard1;
            guard1 = false;
            if (istart != 3) {
              lambda = T[istart + 3 * (istart - 1)];
              if (lambda != 0.0) {
                lambda = muDoubleScalarAbs(lambda);
                V[istart - 1].re = 0.0;
                V[istart - 1].im = lambda;
                V[istart].re = 0.0;
                V[istart].im = -lambda;
                istart += 2;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
            if (guard1) {
              V[istart - 1].re = 0.0;
              V[istart - 1].im = 0.0;
              istart++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        real_T anrm;
        st.site = &ee_emlrtRSI;
        b_st.site = &gg_emlrtRSI;
        c_st.site = &ig_emlrtRSI;
        memcpy(&T[0], &A[0], 9U * sizeof(real_T));
        istart = 0;
        anrm = 0.0;
        i = 0;
        exitg2 = false;
        while ((!exitg2) && (i < 9)) {
          lambda = muDoubleScalarAbs(A[i]);
          if (muDoubleScalarIsNaN(lambda)) {
            anrm = rtNaN;
            exitg2 = true;
          } else {
            if (lambda > anrm) {
              anrm = lambda;
            }
            i++;
          }
        }
        if (muDoubleScalarIsInf(anrm) || muDoubleScalarIsNaN(anrm)) {
          V[0].re = rtNaN;
          V[0].im = 0.0;
          V[1].re = rtNaN;
          V[1].im = 0.0;
          V[2].re = rtNaN;
          V[2].im = 0.0;
        } else {
          real_T wi[3];
          int32_T ilo;
          lambda = anrm;
          scalea = false;
          if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
            scalea = true;
            lambda = 6.7178761075670888E-139;
            f_xzlascl(anrm, lambda, T);
          } else if (anrm > 1.4885657073574029E+138) {
            scalea = true;
            lambda = 1.4885657073574029E+138;
            f_xzlascl(anrm, lambda, T);
          }
          d_st.site = &jg_emlrtRSI;
          ilo = xzgebal(&d_st, T, &ihi, work);
          d_st.site = &kg_emlrtRSI;
          if ((ihi - ilo) + 1 > 1) {
            e_st.site = &hh_emlrtRSI;
            if (ilo - 1 > 2147483646) {
              f_st.site = &k_emlrtRSI;
              check_forloop_overflow_error(&f_st);
            }
            i = (uint8_T)(ilo - 1);
            if (i - 1 >= 0) {
              memset(&tau[0], 0, (uint32_T)i * sizeof(real_T));
            }
            for (j = ihi; j < 3; j++) {
              tau[j - 1] = 0.0;
            }
            work[0] = 0.0;
            work[1] = 0.0;
            work[2] = 0.0;
            e_st.site = &ih_emlrtRSI;
            if ((ilo <= ihi - 1) && (ihi - 1 > 2147483646)) {
              f_st.site = &k_emlrtRSI;
              check_forloop_overflow_error(&f_st);
            }
            for (j = ilo; j < ihi; j++) {
              real_T d;
              int32_T alpha1_tmp;
              int32_T b_in;
              int32_T in;
              int32_T lastv;
              istart = (j - 1) * 3;
              b_in = j * 3 + 1;
              lastv = ihi - j;
              alpha1_tmp = j + istart;
              alpha1 = T[alpha1_tmp];
              e_st.site = &se_emlrtRSI;
              d = xzlarfg(&e_st, lastv, &alpha1, T, istart + 3);
              tau[j - 1] = d;
              T[alpha1_tmp] = 1.0;
              e_st.site = &te_emlrtRSI;
              if (d != 0.0) {
                in = lastv;
                istart = (alpha1_tmp + lastv) + 1;
                while ((in > 0) && (T[istart - 2] == 0.0)) {
                  in--;
                  istart--;
                }
                istart = ihi;
                exitg2 = false;
                while ((!exitg2) && (istart > 0)) {
                  int32_T ia;
                  i = (b_in + istart) - 1;
                  ia = i;
                  do {
                    exitg1 = 0;
                    if (ia <= i + (in - 1) * 3) {
                      if (T[ia - 1] != 0.0) {
                        exitg1 = 1;
                      } else {
                        ia += 3;
                      }
                    } else {
                      istart--;
                      exitg1 = 2;
                    }
                  } while (exitg1 == 0);
                  if (exitg1 == 1) {
                    exitg2 = true;
                  }
                }
              } else {
                in = 0;
                istart = 0;
              }
              if (in > 0) {
                f_st.site = &ve_emlrtRSI;
                xgemv(&f_st, istart, in, T, b_in, T, alpha1_tmp + 1, work);
                f_st.site = &we_emlrtRSI;
                xgerc(&f_st, istart, in, -tau[j - 1], work, alpha1_tmp + 1, T,
                      b_in);
              }
              e_st.site = &ue_emlrtRSI;
              xzlarf(&e_st, lastv, 3 - j, alpha1_tmp + 1, tau[j - 1], T,
                     j + b_in, work);
              T[alpha1_tmp] = alpha1;
            }
          }
          d_st.site = &lg_emlrtRSI;
          istart = xdlahqr(&d_st, ilo, ihi, T, work, wi);
          if (scalea) {
            d_st.site = &mg_emlrtRSI;
            xzlascl(&d_st, lambda, anrm, 3 - istart, work, istart + 1);
            d_st.site = &ng_emlrtRSI;
            xzlascl(&d_st, lambda, anrm, 3 - istart, wi, istart + 1);
            if (istart != 0) {
              d_st.site = &og_emlrtRSI;
              b_xzlascl(&d_st, lambda, anrm, ilo - 1, work);
              d_st.site = &pg_emlrtRSI;
              b_xzlascl(&d_st, lambda, anrm, ilo - 1, wi);
            }
          }
          if (istart != 0) {
            d_st.site = &qg_emlrtRSI;
            if ((ilo <= istart) && (istart > 2147483646)) {
              e_st.site = &k_emlrtRSI;
              check_forloop_overflow_error(&e_st);
            }
            for (j = ilo; j <= istart; j++) {
              work[j - 1] = rtNaN;
              wi[j - 1] = 0.0;
            }
          }
          V[0].re = work[0];
          V[0].im = wi[0];
          V[1].re = work[1];
          V[1].im = wi[1];
          V[2].re = work[2];
          V[2].im = wi[2];
        }
        if ((istart != 0) && (!emlrtSetWarningFlag(&st))) {
          b_st.site = &hg_emlrtRSI;
          warning(&b_st);
        }
      }
    }
  }
}

/* End of code generation (eig.c) */
