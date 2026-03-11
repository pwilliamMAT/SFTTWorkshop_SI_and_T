/*
 * xzgehrd.c
 *
 * Code generation for function 'xzgehrd'
 *
 */

/* Include files */
#include "xzgehrd.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fe_emlrtRSI = {
    46,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo ge_emlrtRSI = {
    50,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo he_emlrtRSI = {
    58,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo ie_emlrtRSI = {
    84,       /* lineNo */
    "xzlarf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

static emlrtRSInfo je_emlrtRSI = {
    91,       /* lineNo */
    "xzlarf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

static emlrtRSInfo le_emlrtRSI = {
    58,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemv.m" /* pathName */
};

/* Function Definitions */
void xzgehrd(const emlrtStack *sp, real_T a[36], real_T tau[5])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T work[6];
  int32_T b_i;
  int32_T i;
  int32_T ia;
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
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }
  for (b_i = 0; b_i < 5; b_i++) {
    __m128d r;
    real_T alpha1;
    real_T xnorm;
    int32_T alpha1_tmp;
    int32_T b_vectorUB;
    int32_T c_i;
    int32_T in;
    int32_T knt;
    int32_T lastc;
    int32_T lastv;
    int32_T vectorUB;
    int32_T work_tmp;
    in = (b_i + 1) * 6;
    alpha1_tmp = (b_i + 6 * b_i) + 1;
    alpha1 = a[alpha1_tmp];
    c_i = b_i + 3;
    lastc = muIntScalarMin_sint32(c_i, 6) + b_i * 6;
    st.site = &fe_emlrtRSI;
    tau[b_i] = 0.0;
    b_st.site = &ld_emlrtRSI;
    xnorm = xnrm2(&b_st, 4 - b_i, a, lastc);
    if (xnorm != 0.0) {
      real_T beta1;
      beta1 = muDoubleScalarHypot(a[alpha1_tmp], xnorm);
      if (alpha1 >= 0.0) {
        beta1 = -beta1;
      }
      if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        work_tmp = (lastc - b_i) + 3;
        do {
          knt++;
          b_st.site = &md_emlrtRSI;
          c_st.site = &td_emlrtRSI;
          d_st.site = &ud_emlrtRSI;
          c_i = ((work_tmp - lastc) + 1) / 2 * 2 + lastc;
          vectorUB = c_i - 2;
          for (i = lastc; i <= vectorUB; i += 2) {
            r = _mm_loadu_pd(&a[i - 1]);
            r = _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r);
            _mm_storeu_pd(&a[i - 1], r);
          }
          for (i = c_i; i <= work_tmp; i++) {
            a[i - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          alpha1 *= 9.9792015476736E+291;
        } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                 (knt < 20));
        b_st.site = &nd_emlrtRSI;
        xnorm = xnrm2(&b_st, 4 - b_i, a, lastc);
        beta1 = muDoubleScalarHypot(alpha1, xnorm);
        if (alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        tau[b_i] = (beta1 - alpha1) / beta1;
        xnorm = 1.0 / (alpha1 - beta1);
        b_st.site = &od_emlrtRSI;
        c_st.site = &td_emlrtRSI;
        d_st.site = &ud_emlrtRSI;
        c_i = ((work_tmp - lastc) + 1) / 2 * 2 + lastc;
        b_vectorUB = c_i - 2;
        for (i = lastc; i <= b_vectorUB; i += 2) {
          r = _mm_loadu_pd(&a[i - 1]);
          r = _mm_mul_pd(_mm_set1_pd(xnorm), r);
          _mm_storeu_pd(&a[i - 1], r);
        }
        for (i = c_i; i <= work_tmp; i++) {
          a[i - 1] *= xnorm;
        }
        b_st.site = &pd_emlrtRSI;
        for (i = 0; i < knt; i++) {
          beta1 *= 1.0020841800044864E-292;
        }
        alpha1 = beta1;
      } else {
        tau[b_i] = (beta1 - alpha1) / beta1;
        xnorm = 1.0 / (alpha1 - beta1);
        b_st.site = &qd_emlrtRSI;
        c_st.site = &td_emlrtRSI;
        c_i = (lastc - b_i) + 3;
        d_st.site = &ud_emlrtRSI;
        vectorUB = ((c_i - lastc) + 1) / 2 * 2 + lastc;
        b_vectorUB = vectorUB - 2;
        for (i = lastc; i <= b_vectorUB; i += 2) {
          r = _mm_loadu_pd(&a[i - 1]);
          r = _mm_mul_pd(_mm_set1_pd(xnorm), r);
          _mm_storeu_pd(&a[i - 1], r);
        }
        for (i = vectorUB; i <= c_i; i++) {
          a[i - 1] *= xnorm;
        }
        alpha1 = beta1;
      }
    }
    a[alpha1_tmp] = 1.0;
    knt = in + 1;
    st.site = &ge_emlrtRSI;
    if (tau[b_i] != 0.0) {
      boolean_T exitg2;
      lastv = 4 - b_i;
      c_i = (alpha1_tmp - b_i) + 4;
      while ((lastv + 1 > 0) && (a[c_i] == 0.0)) {
        lastv--;
        c_i--;
      }
      lastc = 6;
      exitg2 = false;
      while ((!exitg2) && (lastc > 0)) {
        int32_T exitg1;
        c_i = in + lastc;
        vectorUB = c_i;
        do {
          exitg1 = 0;
          if (vectorUB <= c_i + lastv * 6) {
            if (a[vectorUB - 1] != 0.0) {
              exitg1 = 1;
            } else {
              vectorUB += 6;
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
      b_st.site = &ie_emlrtRSI;
      c_st.site = &ke_emlrtRSI;
      if (lastc != 0) {
        d_st.site = &me_emlrtRSI;
        memset(&work[0], 0, (uint32_T)lastc * sizeof(real_T));
        c_i = alpha1_tmp;
        vectorUB = (in + 6 * lastv) + 1;
        for (i = knt; i <= vectorUB; i += 6) {
          b_vectorUB = i + lastc;
          d_st.site = &le_emlrtRSI;
          for (ia = i; ia < b_vectorUB; ia++) {
            work_tmp = ia - i;
            work[work_tmp] += a[ia - 1] * a[c_i];
          }
          c_i++;
        }
      }
      b_st.site = &je_emlrtRSI;
      c_st.site = &ne_emlrtRSI;
      d_st.site = &oe_emlrtRSI;
      e_st.site = &pe_emlrtRSI;
      if (!(-tau[b_i] == 0.0)) {
        c_i = in;
        f_st.site = &qe_emlrtRSI;
        for (i = 0; i <= lastv; i++) {
          xnorm = a[alpha1_tmp + i];
          if (xnorm != 0.0) {
            xnorm *= -tau[b_i];
            vectorUB = c_i + 1;
            b_vectorUB = lastc + c_i;
            f_st.site = &re_emlrtRSI;
            if ((c_i + 1 <= b_vectorUB) && (b_vectorUB > 2147483646)) {
              g_st.site = &tb_emlrtRSI;
              check_forloop_overflow_error(&g_st);
            }
            work_tmp = ((b_vectorUB - vectorUB) + 1) / 2 * 2 + vectorUB;
            knt = work_tmp - 2;
            for (ia = vectorUB; ia <= knt; ia += 2) {
              __m128d r1;
              r = _mm_loadu_pd(&work[(ia - c_i) - 1]);
              r = _mm_mul_pd(r, _mm_set1_pd(xnorm));
              r1 = _mm_loadu_pd(&a[ia - 1]);
              r = _mm_add_pd(r1, r);
              _mm_storeu_pd(&a[ia - 1], r);
            }
            for (ia = work_tmp; ia <= b_vectorUB; ia++) {
              a[ia - 1] += work[(ia - c_i) - 1] * xnorm;
            }
          }
          c_i += 6;
        }
      }
    }
    st.site = &he_emlrtRSI;
    xzlarf(&st, 5 - b_i, 5 - b_i, alpha1_tmp + 1, tau[b_i], a, (b_i + in) + 2,
           work);
    a[alpha1_tmp] = alpha1;
  }
}

/* End of code generation (xzgehrd.c) */
