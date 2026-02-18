/*
 * eigSkewHermitianStandard.c
 *
 * Code generation for function 'eigSkewHermitianStandard'
 *
 */

/* Include files */
#include "eigSkewHermitianStandard.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "xdlanv2.h"
#include "xrot.h"
#include "xzgehrd.h"
#include "xzlarfg.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo wd_emlrtRSI = {
    10,                         /* lineNo */
    "eigSkewHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigSkew"
    "HermitianStandard.m" /* pathName */
};

static emlrtRSInfo xd_emlrtRSI = {
    19,                             /* lineNo */
    "eigRealSkewSymmetricStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigReal"
    "SkewSymmetricStandard.m" /* pathName */
};

static emlrtRSInfo ae_emlrtRSI = {
    52,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo be_emlrtRSI = {
    54,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo we_emlrtRSI = {
    32,       /* lineNo */
    "xhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xhseqr.m" /* pathName */
};

static emlrtRSInfo xe_emlrtRSI = {
    22,        /* lineNo */
    "xdhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdhseqr.m" /* pathName */
};

/* Function Definitions */
void eigSkewHermitianStandard(const emlrtStack *sp, const real_T A[36],
                              creal_T V[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T T[36];
  real_T unusedExpr[5];
  real_T v[3];
  real_T h12;
  real_T h21;
  real_T h22;
  real_T lambda;
  real_T rt1i;
  real_T rt2i;
  real_T rt2r;
  real_T tr;
  int32_T b_i;
  int32_T istart;
  int32_T k;
  boolean_T converged;
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
  st.site = &wd_emlrtRSI;
  b_st.site = &xd_emlrtRSI;
  c_st.site = &yd_emlrtRSI;
  d_st.site = &dd_emlrtRSI;
  e_st.site = &ed_emlrtRSI;
  converged = true;
  for (k = 0; k < 36; k++) {
    if (converged) {
      lambda = A[k];
      if (muDoubleScalarIsInf(lambda) || muDoubleScalarIsNaN(lambda)) {
        converged = false;
      }
    } else {
      converged = false;
    }
  }
  if (!converged) {
    for (k = 0; k < 36; k++) {
      T[k] = rtNaN;
    }
    istart = 2;
    for (k = 0; k < 5; k++) {
      if (istart <= 6) {
        memset(&T[(k * 6 + istart) + -1], 0,
               (uint32_T)(-istart + 7) * sizeof(real_T));
      }
      istart++;
    }
  } else {
    int32_T i;
    int32_T info;
    int32_T kdefl;
    boolean_T exitg1;
    c_st.site = &ae_emlrtRSI;
    memcpy(&T[0], &A[0], 36U * sizeof(real_T));
    d_st.site = &de_emlrtRSI;
    xzgehrd(&d_st, T, unusedExpr);
    c_st.site = &be_emlrtRSI;
    d_st.site = &we_emlrtRSI;
    e_st.site = &xe_emlrtRSI;
    info = -1;
    T[2] = 0.0;
    T[3] = 0.0;
    T[9] = 0.0;
    T[10] = 0.0;
    T[16] = 0.0;
    T[17] = 0.0;
    T[23] = 0.0;
    kdefl = 0;
    i = 5;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1)) {
      int32_T its;
      int32_T l;
      int32_T nr;
      int32_T tst_tmp_tmp;
      boolean_T exitg2;
      l = 1;
      converged = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its < 301)) {
        real_T s;
        int32_T b_k;
        boolean_T exitg3;
        b_k = i;
        exitg3 = false;
        while ((!exitg3) && (b_k + 1 > l)) {
          istart = b_k + 6 * (b_k - 1);
          h22 = muDoubleScalarAbs(T[istart]);
          if (h22 <= 6.0125050800269183E-292) {
            exitg3 = true;
          } else {
            tst_tmp_tmp = b_k + 6 * b_k;
            h12 = T[tst_tmp_tmp];
            rt2i = muDoubleScalarAbs(h12);
            h21 = T[istart - 1];
            lambda = muDoubleScalarAbs(h21) + rt2i;
            if (lambda == 0.0) {
              if (b_k - 1 >= 1) {
                lambda = muDoubleScalarAbs(T[(b_k + 6 * (b_k - 2)) - 1]);
              }
              if (b_k + 2 <= 6) {
                lambda += muDoubleScalarAbs(T[tst_tmp_tmp + 1]);
              }
            }
            if (h22 <= 2.2204460492503131E-16 * lambda) {
              rt2r = muDoubleScalarAbs(T[tst_tmp_tmp - 1]);
              lambda = muDoubleScalarAbs(h21 - h12);
              h12 = muDoubleScalarMax(rt2i, lambda);
              lambda = muDoubleScalarMin(rt2i, lambda);
              s = h12 + lambda;
              if (muDoubleScalarMin(h22, rt2r) *
                      (muDoubleScalarMax(h22, rt2r) / s) <=
                  muDoubleScalarMax(6.0125050800269183E-292,
                                    2.2204460492503131E-16 *
                                        (lambda * (h12 / s)))) {
                exitg3 = true;
              } else {
                b_k--;
              }
            } else {
              b_k--;
            }
          }
        }
        l = b_k + 1;
        if (b_k + 1 > 1) {
          T[b_k + 6 * (b_k - 1)] = 0.0;
        }
        if (b_k + 1 >= i) {
          converged = true;
          exitg2 = true;
        } else {
          __m128d r;
          real_T rt1r;
          int32_T m;
          kdefl++;
          if (kdefl - kdefl / 20 * 20 == 0) {
            s = muDoubleScalarAbs(T[i + 6 * (i - 1)]) +
                muDoubleScalarAbs(T[(i + 6 * (i - 2)) - 1]);
            lambda = 0.75 * s + T[i + 6 * i];
            h12 = -0.4375 * s;
            h21 = s;
            h22 = lambda;
          } else if (kdefl - kdefl / 10 * 10 == 0) {
            istart = b_k + 6 * b_k;
            s = muDoubleScalarAbs(T[istart + 1]) +
                muDoubleScalarAbs(T[(b_k + 6 * (b_k + 1)) + 2]);
            lambda = 0.75 * s + T[istart];
            h12 = -0.4375 * s;
            h21 = s;
            h22 = lambda;
          } else {
            istart = i + 6 * (i - 1);
            lambda = T[istart - 1];
            h21 = T[istart];
            istart = i + 6 * i;
            h12 = T[istart - 1];
            h22 = T[istart];
          }
          s = ((muDoubleScalarAbs(lambda) + muDoubleScalarAbs(h12)) +
               muDoubleScalarAbs(h21)) +
              muDoubleScalarAbs(h22);
          if (s == 0.0) {
            rt1r = 0.0;
            rt1i = 0.0;
            rt2r = 0.0;
            rt2i = 0.0;
          } else {
            lambda /= s;
            h21 /= s;
            h12 /= s;
            h22 /= s;
            tr = (lambda + h22) / 2.0;
            lambda = (lambda - tr) * (h22 - tr) - h12 * h21;
            h12 = muDoubleScalarSqrt(muDoubleScalarAbs(lambda));
            if (lambda >= 0.0) {
              rt1r = tr * s;
              rt2r = rt1r;
              rt1i = h12 * s;
              rt2i = -rt1i;
            } else {
              rt1r = tr + h12;
              rt2r = tr - h12;
              if (muDoubleScalarAbs(rt1r - h22) <=
                  muDoubleScalarAbs(rt2r - h22)) {
                rt1r *= s;
                rt2r = rt1r;
              } else {
                rt2r *= s;
                rt1r = rt2r;
              }
              rt1i = 0.0;
              rt2i = 0.0;
            }
          }
          m = i - 1;
          exitg3 = false;
          while ((!exitg3) && (m >= b_k + 1)) {
            istart = m + 6 * (m - 1);
            lambda = T[istart];
            tr = T[istart - 1];
            h12 = tr - rt2r;
            s = (muDoubleScalarAbs(h12) + muDoubleScalarAbs(rt2i)) +
                muDoubleScalarAbs(lambda);
            h21 = lambda / s;
            istart = m + 6 * m;
            v[0] = (h21 * T[istart - 1] + h12 * (h12 / s)) - rt1i * (rt2i / s);
            lambda = T[istart];
            v[1] = h21 * (((tr + lambda) - rt1r) - rt2r);
            v[2] = h21 * T[istart + 1];
            s = (muDoubleScalarAbs(v[0]) + muDoubleScalarAbs(v[1])) +
                muDoubleScalarAbs(v[2]);
            r = _mm_loadu_pd(&v[0]);
            _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
            v[2] /= s;
            if (m == b_k + 1) {
              exitg3 = true;
            } else {
              istart = m + 6 * (m - 2);
              if (muDoubleScalarAbs(T[istart - 1]) *
                      (muDoubleScalarAbs(v[1]) + muDoubleScalarAbs(v[2])) <=
                  2.2204460492503131E-16 * muDoubleScalarAbs(v[0]) *
                      ((muDoubleScalarAbs(T[istart - 2]) +
                        muDoubleScalarAbs(tr)) +
                       muDoubleScalarAbs(lambda))) {
                exitg3 = true;
              } else {
                m--;
              }
            }
          }
          for (b_i = m; b_i <= i; b_i++) {
            istart = (i - b_i) + 2;
            nr = muIntScalarMin_sint32(3, istart);
            if (b_i > m) {
              istart = ((b_i - 2) * 6 + b_i) - 1;
              for (k = 0; k < nr; k++) {
                v[k] = T[istart + k];
              }
            }
            lambda = v[0];
            f_st.site = &cf_emlrtRSI;
            rt1r = xzlarfg(&f_st, nr, &lambda, v);
            if (b_i > m) {
              istart = b_i + 6 * (b_i - 2);
              T[istart - 1] = lambda;
              T[istart] = 0.0;
              if (b_i < i) {
                T[istart + 1] = 0.0;
              }
            } else if (m > b_k + 1) {
              istart = (b_i + 6 * (b_i - 2)) - 1;
              T[istart] *= 1.0 - rt1r;
            }
            rt1i = v[1];
            rt2i = rt1r * v[1];
            if (nr == 3) {
              int32_T b_scalarLB;
              int32_T c_i;
              tr = v[2];
              h22 = rt1r * v[2];
              for (k = b_i; k < 7; k++) {
                istart = b_i + 6 * (k - 1);
                lambda = T[istart - 1];
                h12 = T[istart];
                h21 = T[istart + 1];
                rt2r = (lambda + rt1i * h12) + tr * h21;
                lambda -= rt2r * rt1r;
                T[istart - 1] = lambda;
                h12 -= rt2r * rt2i;
                T[istart] = h12;
                h21 -= rt2r * h22;
                T[istart + 1] = h21;
              }
              istart = b_i + 3;
              nr = i + 1;
              c_i = muIntScalarMin_sint32(istart, nr);
              b_scalarLB = (c_i / 2) << 1;
              istart = b_scalarLB - 2;
              for (k = 0; k <= istart; k += 2) {
                __m128d r1;
                __m128d r2;
                __m128d r3;
                int32_T scalarLB;
                tst_tmp_tmp = k + 6 * b_i;
                r = _mm_loadu_pd(&T[tst_tmp_tmp]);
                nr = k + 6 * (b_i + 1);
                r1 = _mm_loadu_pd(&T[nr]);
                scalarLB = k + 6 * (b_i - 1);
                r2 = _mm_loadu_pd(&T[scalarLB]);
                r3 =
                    _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(rt1i), r)),
                               _mm_mul_pd(_mm_set1_pd(tr), r1));
                _mm_storeu_pd(
                    &T[scalarLB],
                    _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(rt1r))));
                _mm_storeu_pd(&T[tst_tmp_tmp],
                              _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt2i))));
                _mm_storeu_pd(&T[nr],
                              _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(h22))));
              }
              for (k = b_scalarLB; k < c_i; k++) {
                istart = k + 6 * (b_i - 1);
                lambda = T[istart];
                tst_tmp_tmp = k + 6 * b_i;
                h12 = T[tst_tmp_tmp];
                nr = k + 6 * (b_i + 1);
                h21 = T[nr];
                rt2r = (lambda + rt1i * h12) + tr * h21;
                lambda -= rt2r * rt1r;
                T[istart] = lambda;
                h12 -= rt2r * rt2i;
                T[tst_tmp_tmp] = h12;
                h21 -= rt2r * h22;
                T[nr] = h21;
              }
            } else if (nr == 2) {
              int32_T scalarLB;
              for (k = b_i; k < 7; k++) {
                istart = b_i + 6 * (k - 1);
                lambda = T[istart - 1];
                h12 = T[istart];
                rt2r = lambda + rt1i * h12;
                lambda -= rt2r * rt1r;
                T[istart - 1] = lambda;
                h12 -= rt2r * rt2i;
                T[istart] = h12;
              }
              scalarLB = ((i + 1) / 2) << 1;
              istart = scalarLB - 2;
              for (k = 0; k <= istart; k += 2) {
                __m128d r1;
                __m128d r2;
                tst_tmp_tmp = k + 6 * b_i;
                r = _mm_loadu_pd(&T[tst_tmp_tmp]);
                nr = k + 6 * (b_i - 1);
                r1 = _mm_loadu_pd(&T[nr]);
                r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(rt1i), r));
                _mm_storeu_pd(
                    &T[nr], _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(rt1r))));
                _mm_storeu_pd(&T[tst_tmp_tmp],
                              _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt2i))));
              }
              for (k = scalarLB; k <= i; k++) {
                istart = k + 6 * (b_i - 1);
                lambda = T[istart];
                nr = k + 6 * b_i;
                h12 = T[nr];
                rt2r = lambda + rt1i * h12;
                lambda -= rt2r * rt1r;
                T[istart] = lambda;
                h12 -= rt2r * rt2i;
                T[nr] = h12;
              }
            }
          }
          its++;
        }
      }
      if (!converged) {
        info = i;
        exitg1 = true;
      } else {
        if ((l != i + 1) && (l == i)) {
          istart = i + 6 * i;
          lambda = T[istart - 1];
          nr = 6 * (i - 1);
          tst_tmp_tmp = i + nr;
          h12 = T[tst_tmp_tmp];
          h21 = T[istart];
          xdlanv2(&T[tst_tmp_tmp - 1], &lambda, &h12, &h21, &rt2i, &tr, &rt2r,
                  &h22, &rt1i);
          T[istart - 1] = lambda;
          T[tst_tmp_tmp] = h12;
          T[istart] = h21;
          if (i + 1 < 6) {
            istart = (i + 1) * 6 + i;
            f_st.site = &gf_emlrtRSI;
            xrot(&f_st, 5 - i, T, istart, istart + 1, h22, rt1i);
          }
          f_st.site = &hf_emlrtRSI;
          b_xrot(&f_st, i - 1, T, nr + 1, i * 6 + 1, h22, rt1i);
        }
        kdefl = 0;
        i = l - 2;
      }
    }
    for (k = 0; k < 4; k++) {
      for (b_i = k + 3; b_i < 7; b_i++) {
        T[(b_i + 6 * k) - 1] = 0.0;
      }
    }
    if ((info + 1 != 0) && (!emlrtSetWarningFlag(&b_st))) {
      c_st.site = &ce_emlrtRSI;
      b_warning(&c_st);
    }
  }
  istart = 1;
  int32_T exitg4;
  do {
    exitg4 = 0;
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
      exitg4 = 1;
    }
  } while (exitg4 == 0);
}

/* End of code generation (eigSkewHermitianStandard.c) */
