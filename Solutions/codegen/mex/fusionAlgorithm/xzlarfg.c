/*
 * xzlarfg.c
 *
 * Code generation for function 'xzlarfg'
 *
 */

/* Include files */
#include "xzlarfg.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
real_T xzlarfg(const emlrtStack *sp, int32_T n, real_T *alpha1, real_T x[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T tau;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  tau = 0.0;
  if (n > 0) {
    real_T xnorm;
    st.site = &kd_emlrtRSI;
    xnorm = b_xnrm2(n - 1, x);
    if (xnorm != 0.0) {
      real_T beta1;
      beta1 = muDoubleScalarHypot(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        beta1 = -beta1;
      }
      if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
        __m128d r;
        int32_T b_vectorUB;
        int32_T knt;
        int32_T scalarLB;
        int32_T vectorUB;
        boolean_T overflow;
        knt = 0;
        overflow = (n > 2147483646);
        scalarLB = (((n - 1) / 2) << 1) + 2;
        vectorUB = scalarLB - 2;
        do {
          knt++;
          st.site = &ld_emlrtRSI;
          b_st.site = &sd_emlrtRSI;
          c_st.site = &td_emlrtRSI;
          if (overflow) {
            d_st.site = &sb_emlrtRSI;
            check_forloop_overflow_error(&d_st);
          }
          for (k = 2; k <= vectorUB; k += 2) {
            r = _mm_loadu_pd(&x[k - 1]);
            _mm_storeu_pd(&x[k - 1],
                          _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
          }
          for (k = scalarLB; k <= n; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                 (knt < 20));
        st.site = &md_emlrtRSI;
        xnorm = b_xnrm2(n - 1, x);
        beta1 = muDoubleScalarHypot(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        st.site = &nd_emlrtRSI;
        b_st.site = &sd_emlrtRSI;
        c_st.site = &td_emlrtRSI;
        if (n > 2147483646) {
          d_st.site = &sb_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        b_vectorUB = scalarLB - 2;
        for (k = 2; k <= b_vectorUB; k += 2) {
          r = _mm_loadu_pd(&x[k - 1]);
          _mm_storeu_pd(&x[k - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (k = scalarLB; k <= n; k++) {
          x[k - 1] *= xnorm;
        }
        st.site = &od_emlrtRSI;
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        *alpha1 = beta1;
      } else {
        int32_T b_vectorUB;
        int32_T vectorUB;
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        st.site = &pd_emlrtRSI;
        b_st.site = &sd_emlrtRSI;
        c_st.site = &td_emlrtRSI;
        if (n > 2147483646) {
          d_st.site = &sb_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        vectorUB = (((n - 1) / 2) << 1) + 2;
        b_vectorUB = vectorUB - 2;
        for (k = 2; k <= b_vectorUB; k += 2) {
          __m128d r;
          r = _mm_loadu_pd(&x[k - 1]);
          _mm_storeu_pd(&x[k - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (k = vectorUB; k <= n; k++) {
          x[k - 1] *= xnorm;
        }
        *alpha1 = beta1;
      }
    }
  }
  return tau;
}

/* End of code generation (xzlarfg.c) */
