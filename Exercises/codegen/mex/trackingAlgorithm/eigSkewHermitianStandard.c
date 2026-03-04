/*
 * eigSkewHermitianStandard.c
 *
 * Code generation for function 'eigSkewHermitianStandard'
 *
 */

/* Include files */
#include "eigSkewHermitianStandard.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "warning.h"
#include "xhseqr.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo re_emlrtRSI = {
    121,                                                   /* lineNo */
    "flatVectorAllOrAny",                                  /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/vAllOrAny.m" /* pathName */
};

/* Function Definitions */
void b_eigSkewHermitianStandard(const emlrtStack *sp, const real_T A[16],
                                creal_T V[4])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack st;
  real_T T[16];
  real_T work[4];
  real_T lambda;
  int32_T exitg1;
  int32_T i;
  int32_T ia;
  int32_T istart;
  int32_T k;
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
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  st.site = &ve_emlrtRSI;
  b_st.site = &we_emlrtRSI;
  memcpy(&T[0], &A[0], 16U * sizeof(real_T));
  c_st.site = &xe_emlrtRSI;
  d_st.site = &pe_emlrtRSI;
  e_st.site = &qe_emlrtRSI;
  p = true;
  f_st.site = &re_emlrtRSI;
  for (k = 0; k < 16; k++) {
    if (p) {
      lambda = A[k];
      if (muDoubleScalarIsInf(lambda) || muDoubleScalarIsNaN(lambda)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (k = 0; k < 16; k++) {
      T[k] = rtNaN;
    }
    istart = 2;
    for (k = 0; k < 3; k++) {
      if (istart <= 4) {
        memset(&T[(k * 4 + istart) + -1], 0,
               (uint32_T)(-istart + 5) * sizeof(real_T));
      }
      istart++;
    }
  } else {
    real_T tau[3];
    c_st.site = &ye_emlrtRSI;
    d_st.site = &cf_emlrtRSI;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (i = 0; i < 3; i++) {
      __m128d r;
      real_T alpha1;
      int32_T alpha1_tmp;
      int32_T c;
      int32_T in;
      int32_T ix0;
      int32_T knt;
      int32_T lastc;
      int32_T lastv;
      int32_T scalarLB;
      istart = i << 2;
      in = (i + 1) << 2;
      alpha1_tmp = (i + istart) + 1;
      alpha1 = T[alpha1_tmp];
      c = i + 3;
      ix0 = muIntScalarMin_sint32(c, 4) + istart;
      e_st.site = &df_emlrtRSI;
      tau[i] = 0.0;
      f_st.site = &cd_emlrtRSI;
      lambda = f_xnrm2(&f_st, 2 - i, T, ix0);
      if (lambda != 0.0) {
        real_T beta1;
        beta1 = muDoubleScalarHypot(alpha1, lambda);
        if (alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          c = (ix0 - i) + 1;
          scalarLB = ((((c - ix0) + 1) / 2) << 1) + ix0;
          istart = scalarLB - 2;
          do {
            knt++;
            f_st.site = &dd_emlrtRSI;
            g_st.site = &ld_emlrtRSI;
            h_st.site = &md_emlrtRSI;
            for (k = ix0; k <= istart; k += 2) {
              r = _mm_loadu_pd(&T[k - 1]);
              _mm_storeu_pd(&T[k - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (k = scalarLB; k <= c; k++) {
              T[k - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            alpha1 *= 9.9792015476736E+291;
          } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                   (knt < 20));
          f_st.site = &ed_emlrtRSI;
          lambda = f_xnrm2(&f_st, 2 - i, T, ix0);
          beta1 = muDoubleScalarHypot(alpha1, lambda);
          if (alpha1 >= 0.0) {
            beta1 = -beta1;
          }
          tau[i] = (beta1 - alpha1) / beta1;
          lambda = 1.0 / (alpha1 - beta1);
          f_st.site = &fd_emlrtRSI;
          g_st.site = &ld_emlrtRSI;
          h_st.site = &md_emlrtRSI;
          istart = scalarLB - 2;
          for (k = ix0; k <= istart; k += 2) {
            r = _mm_loadu_pd(&T[k - 1]);
            _mm_storeu_pd(&T[k - 1], _mm_mul_pd(_mm_set1_pd(lambda), r));
          }
          for (k = scalarLB; k <= c; k++) {
            T[k - 1] *= lambda;
          }
          f_st.site = &gd_emlrtRSI;
          for (k = 0; k < knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }
          alpha1 = beta1;
        } else {
          tau[i] = (beta1 - alpha1) / beta1;
          lambda = 1.0 / (alpha1 - beta1);
          f_st.site = &hd_emlrtRSI;
          g_st.site = &ld_emlrtRSI;
          istart = (ix0 - i) + 1;
          h_st.site = &md_emlrtRSI;
          c = ((((istart - ix0) + 1) / 2) << 1) + ix0;
          scalarLB = c - 2;
          for (k = ix0; k <= scalarLB; k += 2) {
            r = _mm_loadu_pd(&T[k - 1]);
            _mm_storeu_pd(&T[k - 1], _mm_mul_pd(_mm_set1_pd(lambda), r));
          }
          for (k = c; k <= istart; k++) {
            T[k - 1] *= lambda;
          }
          alpha1 = beta1;
        }
      }
      T[alpha1_tmp] = 1.0;
      ix0 = in + 1;
      e_st.site = &ef_emlrtRSI;
      if (tau[i] != 0.0) {
        boolean_T exitg2;
        lastv = 2 - i;
        istart = (alpha1_tmp - i) + 2;
        while ((lastv + 1 > 0) && (T[istart] == 0.0)) {
          lastv--;
          istart--;
        }
        lastc = 4;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          istart = in + lastc;
          c = istart;
          do {
            exitg1 = 0;
            if (c <= istart + (lastv << 2)) {
              if (T[c - 1] != 0.0) {
                exitg1 = 1;
              } else {
                c += 4;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        lastc = 0;
      }
      if (lastv + 1 > 0) {
        f_st.site = &gf_emlrtRSI;
        g_st.site = &if_emlrtRSI;
        if (lastc != 0) {
          h_st.site = &kf_emlrtRSI;
          memset(&work[0], 0, (uint32_T)lastc * sizeof(real_T));
          istart = alpha1_tmp;
          c = (in + (lastv << 2)) + 1;
          for (k = ix0; k <= c; k += 4) {
            scalarLB = k + lastc;
            h_st.site = &jf_emlrtRSI;
            for (ia = k; ia < scalarLB; ia++) {
              knt = ia - k;
              work[knt] += T[ia - 1] * T[istart];
            }
            istart++;
          }
        }
        f_st.site = &hf_emlrtRSI;
        g_st.site = &lf_emlrtRSI;
        h_st.site = &mf_emlrtRSI;
        i_st.site = &nf_emlrtRSI;
        if (!(-tau[i] == 0.0)) {
          istart = in;
          j_st.site = &of_emlrtRSI;
          for (k = 0; k <= lastv; k++) {
            lambda = T[alpha1_tmp + k];
            if (lambda != 0.0) {
              lambda *= -tau[i];
              c = istart + 1;
              scalarLB = lastc + istart;
              j_st.site = &pf_emlrtRSI;
              if ((istart + 1 <= scalarLB) && (scalarLB > 2147483646)) {
                k_st.site = &k_emlrtRSI;
                check_forloop_overflow_error(&k_st);
              }
              knt = ((((scalarLB - istart) / 2) << 1) + istart) + 1;
              ix0 = knt - 2;
              for (ia = c; ia <= ix0; ia += 2) {
                __m128d r1;
                r = _mm_loadu_pd(&work[(ia - istart) - 1]);
                r1 = _mm_loadu_pd(&T[ia - 1]);
                _mm_storeu_pd(
                    &T[ia - 1],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(lambda))));
              }
              for (ia = knt; ia <= scalarLB; ia++) {
                T[ia - 1] += work[(ia - istart) - 1] * lambda;
              }
            }
            istart += 4;
          }
        }
      }
      e_st.site = &ff_emlrtRSI;
      c_xzlarf(&e_st, 3 - i, 3 - i, alpha1_tmp + 1, tau[i], T, (i + in) + 2,
               work);
      T[alpha1_tmp] = alpha1;
    }
    c_st.site = &af_emlrtRSI;
    istart = c_xhseqr(&c_st, T);
    if ((istart != 0) && (!emlrtSetWarningFlag(&b_st))) {
      c_st.site = &bf_emlrtRSI;
      b_warning(&c_st);
    }
  }
  istart = 1;
  do {
    exitg1 = 0;
    if (istart <= 4) {
      boolean_T guard1;
      guard1 = false;
      if (istart != 4) {
        lambda = T[istart + ((istart - 1) << 2)];
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
}

void eigSkewHermitianStandard(const emlrtStack *sp, const real_T A[36],
                              creal_T V[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack st;
  real_T T[36];
  real_T work[6];
  real_T lambda;
  int32_T b_i;
  int32_T b_ia;
  int32_T exitg1;
  int32_T i;
  int32_T istart;
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
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  st.site = &ve_emlrtRSI;
  b_st.site = &we_emlrtRSI;
  memcpy(&T[0], &A[0], 36U * sizeof(real_T));
  c_st.site = &xe_emlrtRSI;
  d_st.site = &pe_emlrtRSI;
  e_st.site = &qe_emlrtRSI;
  p = true;
  f_st.site = &re_emlrtRSI;
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
      T[i] = rtNaN;
    }
    istart = 2;
    for (i = 0; i < 5; i++) {
      if (istart <= 6) {
        memset(&T[(i * 6 + istart) + -1], 0,
               (uint32_T)(-istart + 7) * sizeof(real_T));
      }
      istart++;
    }
  } else {
    real_T tau[5];
    c_st.site = &ye_emlrtRSI;
    d_st.site = &cf_emlrtRSI;
    for (i = 0; i < 6; i++) {
      work[i] = 0.0;
    }
    for (b_i = 0; b_i < 5; b_i++) {
      __m128d r;
      real_T alpha1;
      int32_T alpha1_tmp;
      int32_T ia;
      int32_T in;
      int32_T ix0;
      int32_T knt;
      int32_T lastc;
      int32_T lastv;
      int32_T scalarLB;
      in = (b_i + 1) * 6;
      alpha1_tmp = (b_i + 6 * b_i) + 1;
      alpha1 = T[alpha1_tmp];
      istart = b_i + 3;
      ix0 = muIntScalarMin_sint32(istart, 6) + b_i * 6;
      e_st.site = &df_emlrtRSI;
      tau[b_i] = 0.0;
      f_st.site = &cd_emlrtRSI;
      lambda = xnrm2(&f_st, 4 - b_i, T, ix0);
      if (lambda != 0.0) {
        real_T beta1;
        beta1 = muDoubleScalarHypot(alpha1, lambda);
        if (alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          ia = (ix0 - b_i) + 3;
          scalarLB = ((((ia - ix0) + 1) / 2) << 1) + ix0;
          istart = scalarLB - 2;
          do {
            knt++;
            f_st.site = &dd_emlrtRSI;
            g_st.site = &ld_emlrtRSI;
            h_st.site = &md_emlrtRSI;
            for (i = ix0; i <= istart; i += 2) {
              r = _mm_loadu_pd(&T[i - 1]);
              _mm_storeu_pd(&T[i - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (i = scalarLB; i <= ia; i++) {
              T[i - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            alpha1 *= 9.9792015476736E+291;
          } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                   (knt < 20));
          f_st.site = &ed_emlrtRSI;
          lambda = xnrm2(&f_st, 4 - b_i, T, ix0);
          beta1 = muDoubleScalarHypot(alpha1, lambda);
          if (alpha1 >= 0.0) {
            beta1 = -beta1;
          }
          tau[b_i] = (beta1 - alpha1) / beta1;
          lambda = 1.0 / (alpha1 - beta1);
          f_st.site = &fd_emlrtRSI;
          g_st.site = &ld_emlrtRSI;
          h_st.site = &md_emlrtRSI;
          istart = scalarLB - 2;
          for (i = ix0; i <= istart; i += 2) {
            r = _mm_loadu_pd(&T[i - 1]);
            _mm_storeu_pd(&T[i - 1], _mm_mul_pd(_mm_set1_pd(lambda), r));
          }
          for (i = scalarLB; i <= ia; i++) {
            T[i - 1] *= lambda;
          }
          f_st.site = &gd_emlrtRSI;
          for (i = 0; i < knt; i++) {
            beta1 *= 1.0020841800044864E-292;
          }
          alpha1 = beta1;
        } else {
          tau[b_i] = (beta1 - alpha1) / beta1;
          lambda = 1.0 / (alpha1 - beta1);
          f_st.site = &hd_emlrtRSI;
          g_st.site = &ld_emlrtRSI;
          istart = (ix0 - b_i) + 3;
          h_st.site = &md_emlrtRSI;
          ia = ((((istart - ix0) + 1) / 2) << 1) + ix0;
          scalarLB = ia - 2;
          for (i = ix0; i <= scalarLB; i += 2) {
            r = _mm_loadu_pd(&T[i - 1]);
            _mm_storeu_pd(&T[i - 1], _mm_mul_pd(_mm_set1_pd(lambda), r));
          }
          for (i = ia; i <= istart; i++) {
            T[i - 1] *= lambda;
          }
          alpha1 = beta1;
        }
      }
      T[alpha1_tmp] = 1.0;
      ix0 = in + 1;
      e_st.site = &ef_emlrtRSI;
      if (tau[b_i] != 0.0) {
        boolean_T exitg2;
        lastv = 4 - b_i;
        istart = (alpha1_tmp - b_i) + 4;
        while ((lastv + 1 > 0) && (T[istart] == 0.0)) {
          lastv--;
          istart--;
        }
        lastc = 6;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          istart = in + lastc;
          ia = istart;
          do {
            exitg1 = 0;
            if (ia <= istart + lastv * 6) {
              if (T[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia += 6;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        lastc = 0;
      }
      if (lastv + 1 > 0) {
        f_st.site = &gf_emlrtRSI;
        g_st.site = &if_emlrtRSI;
        if (lastc != 0) {
          h_st.site = &kf_emlrtRSI;
          memset(&work[0], 0, (uint32_T)lastc * sizeof(real_T));
          istart = alpha1_tmp;
          ia = (in + 6 * lastv) + 1;
          for (i = ix0; i <= ia; i += 6) {
            scalarLB = i + lastc;
            h_st.site = &jf_emlrtRSI;
            for (b_ia = i; b_ia < scalarLB; b_ia++) {
              knt = b_ia - i;
              work[knt] += T[b_ia - 1] * T[istart];
            }
            istart++;
          }
        }
        f_st.site = &hf_emlrtRSI;
        g_st.site = &lf_emlrtRSI;
        h_st.site = &mf_emlrtRSI;
        i_st.site = &nf_emlrtRSI;
        if (!(-tau[b_i] == 0.0)) {
          istart = in;
          j_st.site = &of_emlrtRSI;
          for (i = 0; i <= lastv; i++) {
            lambda = T[alpha1_tmp + i];
            if (lambda != 0.0) {
              lambda *= -tau[b_i];
              ia = istart + 1;
              scalarLB = lastc + istart;
              j_st.site = &pf_emlrtRSI;
              if ((istart + 1 <= scalarLB) && (scalarLB > 2147483646)) {
                k_st.site = &k_emlrtRSI;
                check_forloop_overflow_error(&k_st);
              }
              knt = ((((scalarLB - istart) / 2) << 1) + istart) + 1;
              ix0 = knt - 2;
              for (b_ia = ia; b_ia <= ix0; b_ia += 2) {
                __m128d r1;
                r = _mm_loadu_pd(&work[(b_ia - istart) - 1]);
                r1 = _mm_loadu_pd(&T[b_ia - 1]);
                _mm_storeu_pd(
                    &T[b_ia - 1],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(lambda))));
              }
              for (b_ia = knt; b_ia <= scalarLB; b_ia++) {
                T[b_ia - 1] += work[(b_ia - istart) - 1] * lambda;
              }
            }
            istart += 6;
          }
        }
      }
      e_st.site = &ff_emlrtRSI;
      b_xzlarf(&e_st, 5 - b_i, 5 - b_i, alpha1_tmp + 1, tau[b_i], T,
               (b_i + in) + 2, work);
      T[alpha1_tmp] = alpha1;
    }
    c_st.site = &af_emlrtRSI;
    istart = b_xhseqr(&c_st, T);
    if ((istart != 0) && (!emlrtSetWarningFlag(&b_st))) {
      c_st.site = &bf_emlrtRSI;
      b_warning(&c_st);
    }
  }
  istart = 1;
  do {
    exitg1 = 0;
    if (istart <= 6) {
      boolean_T guard1;
      guard1 = false;
      if (istart != 6) {
        lambda = T[istart + 6 * (istart - 1)];
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
}

/* End of code generation (eigSkewHermitianStandard.c) */
