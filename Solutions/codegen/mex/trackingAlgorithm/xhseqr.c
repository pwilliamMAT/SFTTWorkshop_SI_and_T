/*
 * xhseqr.c
 *
 * Code generation for function 'xhseqr'
 *
 */

/* Include files */
#include "xhseqr.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo lf_emlrtRSI = {
    32,       /* lineNo */
    "xhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xhseqr.m" /* pathName */
};

static emlrtRSInfo mf_emlrtRSI = {
    22,        /* lineNo */
    "xdhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdhseqr.m" /* pathName */
};

/* Function Definitions */
int32_T b_xhseqr(const emlrtStack *sp, real_T h[36])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v[3];
  real_T h11;
  real_T h12;
  real_T h21;
  real_T h21s;
  real_T h22;
  real_T rt1r;
  real_T rt2r;
  real_T t3;
  int32_T b_i;
  int32_T i;
  int32_T info;
  int32_T j;
  int32_T kdefl;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &lf_emlrtRSI;
  b_st.site = &mf_emlrtRSI;
  info = 0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[9] = 0.0;
  h[10] = 0.0;
  h[16] = 0.0;
  h[17] = 0.0;
  h[23] = 0.0;
  kdefl = 0;
  i = 5;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int32_T its;
    int32_T ix_tmp;
    int32_T iy;
    int32_T l;
    int32_T nr;
    int32_T s_tmp;
    int32_T scalarLB;
    boolean_T converged;
    boolean_T exitg2;
    l = 1;
    converged = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      real_T s;
      int32_T k;
      boolean_T exitg3;
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > l)) {
        s_tmp = k + 6 * (k - 1);
        h21s = muDoubleScalarAbs(h[s_tmp]);
        if (h21s <= 6.0125050800269183E-292) {
          exitg3 = true;
        } else {
          ix_tmp = k + 6 * k;
          h21 = muDoubleScalarAbs(h[ix_tmp]);
          h11 = muDoubleScalarAbs(h[s_tmp - 1]) + h21;
          if (h11 == 0.0) {
            if (k - 1 >= 1) {
              h11 = muDoubleScalarAbs(h[(k + 6 * (k - 2)) - 1]);
            }
            if (k + 2 <= 6) {
              h11 += muDoubleScalarAbs(h[ix_tmp + 1]);
            }
          }
          if (h21s <= 2.2204460492503131E-16 * h11) {
            h11 = muDoubleScalarAbs(h[ix_tmp - 1]);
            h12 = muDoubleScalarAbs(h[s_tmp - 1] - h[ix_tmp]);
            t3 = muDoubleScalarMax(h21, h12);
            h12 = muDoubleScalarMin(h21, h12);
            s = t3 + h12;
            if (muDoubleScalarMin(h21s, h11) *
                    (muDoubleScalarMax(h21s, h11) / s) <=
                muDoubleScalarMax(6.0125050800269183E-292,
                                  2.2204460492503131E-16 * (h12 * (t3 / s)))) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }
      l = k + 1;
      if (k + 1 > 1) {
        h[k + 6 * (k - 1)] = 0.0;
      }
      if (k + 1 >= i) {
        converged = true;
        exitg2 = true;
      } else {
        __m128d r;
        int32_T m;
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          s = muDoubleScalarAbs(h[i + 6 * (i - 1)]) +
              muDoubleScalarAbs(h[(i + 6 * (i - 2)) - 1]);
          h11 = 0.75 * s + h[i + 6 * i];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = h11;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          s_tmp = k + 6 * k;
          s = muDoubleScalarAbs(h[s_tmp + 1]) +
              muDoubleScalarAbs(h[(k + 6 * (k + 1)) + 2]);
          h11 = 0.75 * s + h[s_tmp];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = h11;
        } else {
          ix_tmp = i + 6 * (i - 1);
          h11 = h[ix_tmp - 1];
          h21 = h[ix_tmp];
          s_tmp = i + 6 * i;
          h12 = h[s_tmp - 1];
          h22 = h[s_tmp];
        }
        s = ((muDoubleScalarAbs(h11) + muDoubleScalarAbs(h12)) +
             muDoubleScalarAbs(h21)) +
            muDoubleScalarAbs(h22);
        if (s == 0.0) {
          rt1r = 0.0;
          h11 = 0.0;
          rt2r = 0.0;
          h12 = 0.0;
        } else {
          h11 /= s;
          h21 /= s;
          h12 /= s;
          h22 /= s;
          t3 = (h11 + h22) / 2.0;
          h11 = (h11 - t3) * (h22 - t3) - h12 * h21;
          h12 = muDoubleScalarSqrt(muDoubleScalarAbs(h11));
          if (h11 >= 0.0) {
            rt1r = t3 * s;
            rt2r = rt1r;
            h11 = h12 * s;
            h12 = -h11;
          } else {
            rt1r = t3 + h12;
            rt2r = t3 - h12;
            if (muDoubleScalarAbs(rt1r - h22) <=
                muDoubleScalarAbs(rt2r - h22)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            h11 = 0.0;
            h12 = 0.0;
          }
        }
        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          s_tmp = m + 6 * (m - 1);
          h21 = h[s_tmp - 1];
          t3 = h21 - rt2r;
          s = (muDoubleScalarAbs(t3) + muDoubleScalarAbs(h12)) +
              muDoubleScalarAbs(h[s_tmp]);
          h21s = h[s_tmp] / s;
          ix_tmp = m + 6 * m;
          v[0] = (h21s * h[ix_tmp - 1] + t3 * (t3 / s)) - h11 * (h12 / s);
          v[1] = h21s * (((h21 + h[ix_tmp]) - rt1r) - rt2r);
          v[2] = h21s * h[ix_tmp + 1];
          s = (muDoubleScalarAbs(v[0]) + muDoubleScalarAbs(v[1])) +
              muDoubleScalarAbs(v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
          v[2] /= s;
          if (m == k + 1) {
            exitg3 = true;
          } else {
            nr = m + 6 * (m - 2);
            if (muDoubleScalarAbs(h[nr - 1]) *
                    (muDoubleScalarAbs(v[1]) + muDoubleScalarAbs(v[2])) <=
                2.2204460492503131E-16 * muDoubleScalarAbs(v[0]) *
                    ((muDoubleScalarAbs(h[nr - 2]) +
                      muDoubleScalarAbs(h[s_tmp - 1])) +
                     muDoubleScalarAbs(h[ix_tmp]))) {
              exitg3 = true;
            } else {
              m--;
            }
          }
        }
        for (b_i = m; b_i <= i; b_i++) {
          s_tmp = (i - b_i) + 2;
          nr = muIntScalarMin_sint32(3, s_tmp);
          if (b_i > m) {
            ix_tmp = ((b_i - 2) * 6 + b_i) - 1;
            for (j = 0; j < nr; j++) {
              v[j] = h[ix_tmp + j];
            }
          }
          h11 = v[0];
          c_st.site = &sf_emlrtRSI;
          s = b_xzlarfg(&c_st, nr, &h11, v);
          if (b_i > m) {
            ix_tmp = b_i + 6 * (b_i - 2);
            h[ix_tmp - 1] = h11;
            h[ix_tmp] = 0.0;
            if (b_i < i) {
              h[ix_tmp + 1] = 0.0;
            }
          } else if (m > k + 1) {
            ix_tmp = (b_i + 6 * (b_i - 2)) - 1;
            h[ix_tmp] *= 1.0 - s;
          }
          h22 = v[1];
          rt2r = s * v[1];
          if (nr == 3) {
            int32_T b_scalarLB;
            h21s = v[2];
            t3 = s * v[2];
            for (j = b_i; j < 7; j++) {
              ix_tmp = b_i + 6 * (j - 1);
              h11 = h[ix_tmp - 1];
              h12 = h[ix_tmp];
              h21 = h[ix_tmp + 1];
              rt1r = (h11 + h22 * h12) + h21s * h21;
              h11 -= rt1r * s;
              h[ix_tmp - 1] = h11;
              h12 -= rt1r * rt2r;
              h[ix_tmp] = h12;
              h21 -= rt1r * t3;
              h[ix_tmp + 1] = h21;
            }
            ix_tmp = b_i + 3;
            nr = i + 1;
            scalarLB = muIntScalarMin_sint32(ix_tmp, nr);
            b_scalarLB = (scalarLB / 2) << 1;
            ix_tmp = b_scalarLB - 2;
            for (j = 0; j <= ix_tmp; j += 2) {
              __m128d r1;
              __m128d r2;
              __m128d r3;
              iy = j + 6 * b_i;
              r = _mm_loadu_pd(&h[iy]);
              s_tmp = j + 6 * (b_i + 1);
              r1 = _mm_loadu_pd(&h[s_tmp]);
              nr = j + 6 * (b_i - 1);
              r2 = _mm_loadu_pd(&h[nr]);
              r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(h22), r)),
                              _mm_mul_pd(_mm_set1_pd(h21s), r1));
              _mm_storeu_pd(&h[nr],
                            _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[iy],
                            _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt2r))));
              _mm_storeu_pd(&h[s_tmp],
                            _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(t3))));
            }
            for (j = b_scalarLB; j < scalarLB; j++) {
              ix_tmp = j + 6 * (b_i - 1);
              h11 = h[ix_tmp];
              iy = j + 6 * b_i;
              h12 = h[iy];
              s_tmp = j + 6 * (b_i + 1);
              h21 = h[s_tmp];
              rt1r = (h11 + h22 * h12) + h21s * h21;
              h11 -= rt1r * s;
              h[ix_tmp] = h11;
              h12 -= rt1r * rt2r;
              h[iy] = h12;
              h21 -= rt1r * t3;
              h[s_tmp] = h21;
            }
          } else if (nr == 2) {
            for (j = b_i; j < 7; j++) {
              ix_tmp = b_i + 6 * (j - 1);
              h11 = h[ix_tmp - 1];
              h12 = h[ix_tmp];
              rt1r = h11 + h22 * h12;
              h11 -= rt1r * s;
              h[ix_tmp - 1] = h11;
              h12 -= rt1r * rt2r;
              h[ix_tmp] = h12;
            }
            scalarLB = ((i + 1) / 2) << 1;
            ix_tmp = scalarLB - 2;
            for (j = 0; j <= ix_tmp; j += 2) {
              __m128d r1;
              __m128d r2;
              iy = j + 6 * b_i;
              r = _mm_loadu_pd(&h[iy]);
              s_tmp = j + 6 * (b_i - 1);
              r1 = _mm_loadu_pd(&h[s_tmp]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(h22), r));
              _mm_storeu_pd(&h[s_tmp],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[iy],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt2r))));
            }
            for (j = scalarLB; j <= i; j++) {
              ix_tmp = j + 6 * (b_i - 1);
              h11 = h[ix_tmp];
              nr = j + 6 * b_i;
              h12 = h[nr];
              rt1r = h11 + h22 * h12;
              h11 -= rt1r * s;
              h[ix_tmp] = h11;
              h12 -= rt1r * rt2r;
              h[nr] = h12;
            }
          }
        }
        its++;
      }
    }
    if (!converged) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((l != i + 1) && (l == i)) {
        s_tmp = i + 6 * i;
        h11 = h[s_tmp - 1];
        scalarLB = 6 * (i - 1);
        ix_tmp = i + scalarLB;
        h12 = h[ix_tmp];
        h21 = h[s_tmp];
        xdlanv2(&h[ix_tmp - 1], &h11, &h12, &h21, &t3, &h21s, &h22, &rt2r,
                &rt1r);
        h[s_tmp - 1] = h11;
        h[ix_tmp] = h12;
        h[s_tmp] = h21;
        if (i + 1 < 6) {
          s_tmp = 4 - i;
          ix_tmp = (i + 1) * 6 + i;
          for (j = 0; j <= s_tmp; j++) {
            nr = ix_tmp + j * 6;
            h11 = h[nr];
            h12 = h[nr - 1];
            h[nr] = rt2r * h11 - rt1r * h12;
            h[nr - 1] = rt2r * h12 + rt1r * h11;
          }
        }
        if (i - 1 >= 1) {
          iy = i * 6;
          s_tmp = (uint8_T)(i - 1);
          for (j = 0; j < s_tmp; j++) {
            ix_tmp = iy + j;
            h11 = h[ix_tmp];
            nr = scalarLB + j;
            h12 = h[nr];
            h[ix_tmp] = rt2r * h11 - rt1r * h12;
            h[nr] = rt2r * h12 + rt1r * h11;
          }
        }
      }
      kdefl = 0;
      i = l - 2;
    }
  }
  for (j = 0; j < 4; j++) {
    for (b_i = j + 3; b_i < 7; b_i++) {
      h[(b_i + 6 * j) - 1] = 0.0;
    }
  }
  return info;
}

int32_T c_xhseqr(const emlrtStack *sp, real_T h[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v[3];
  real_T h11;
  real_T h12;
  real_T h21;
  real_T h21s;
  real_T h22;
  real_T rt1r;
  real_T rt2r;
  real_T t3;
  int32_T b_i;
  int32_T i;
  int32_T info;
  int32_T j;
  int32_T kdefl;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &lf_emlrtRSI;
  b_st.site = &mf_emlrtRSI;
  info = 0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  kdefl = 0;
  i = 3;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int32_T b_temp_tmp_tmp;
    int32_T c_i;
    int32_T its;
    int32_T l;
    int32_T nr;
    int32_T scalarLB;
    int32_T temp_tmp_tmp;
    boolean_T converged;
    boolean_T exitg2;
    l = 1;
    converged = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      real_T s;
      int32_T k;
      boolean_T exitg3;
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > l)) {
        temp_tmp_tmp = k + ((k - 1) << 2);
        h21s = muDoubleScalarAbs(h[temp_tmp_tmp]);
        if (h21s <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          nr = k + (k << 2);
          h21 = muDoubleScalarAbs(h[nr]);
          h11 = muDoubleScalarAbs(h[temp_tmp_tmp - 1]) + h21;
          if (h11 == 0.0) {
            if (k - 1 >= 1) {
              h11 = muDoubleScalarAbs(h[(k + ((k - 2) << 2)) - 1]);
            }
            if (k + 2 <= 4) {
              h11 += muDoubleScalarAbs(h[nr + 1]);
            }
          }
          if (h21s <= 2.2204460492503131E-16 * h11) {
            h11 = muDoubleScalarAbs(h[nr - 1]);
            h12 = muDoubleScalarAbs(h[temp_tmp_tmp - 1] - h[nr]);
            t3 = muDoubleScalarMax(h21, h12);
            h12 = muDoubleScalarMin(h21, h12);
            s = t3 + h12;
            if (muDoubleScalarMin(h21s, h11) *
                    (muDoubleScalarMax(h21s, h11) / s) <=
                muDoubleScalarMax(4.0083367200179456E-292,
                                  2.2204460492503131E-16 * (h12 * (t3 / s)))) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }
      l = k + 1;
      if (k + 1 > 1) {
        h[k + ((k - 1) << 2)] = 0.0;
      }
      if (k + 1 >= i) {
        converged = true;
        exitg2 = true;
      } else {
        __m128d r;
        int32_T m;
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          s = muDoubleScalarAbs(h[i + ((i - 1) << 2)]) +
              muDoubleScalarAbs(h[(i + ((i - 2) << 2)) - 1]);
          h11 = 0.75 * s + h[i + (i << 2)];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = h11;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          temp_tmp_tmp = k + (k << 2);
          s = muDoubleScalarAbs(h[temp_tmp_tmp + 1]) +
              muDoubleScalarAbs(h[(k + ((k + 1) << 2)) + 2]);
          h11 = 0.75 * s + h[temp_tmp_tmp];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = h11;
        } else {
          nr = i + ((i - 1) << 2);
          h11 = h[nr - 1];
          h21 = h[nr];
          temp_tmp_tmp = i + (i << 2);
          h12 = h[temp_tmp_tmp - 1];
          h22 = h[temp_tmp_tmp];
        }
        s = ((muDoubleScalarAbs(h11) + muDoubleScalarAbs(h12)) +
             muDoubleScalarAbs(h21)) +
            muDoubleScalarAbs(h22);
        if (s == 0.0) {
          rt1r = 0.0;
          h11 = 0.0;
          rt2r = 0.0;
          h12 = 0.0;
        } else {
          h11 /= s;
          h21 /= s;
          h12 /= s;
          h22 /= s;
          t3 = (h11 + h22) / 2.0;
          h11 = (h11 - t3) * (h22 - t3) - h12 * h21;
          h12 = muDoubleScalarSqrt(muDoubleScalarAbs(h11));
          if (h11 >= 0.0) {
            rt1r = t3 * s;
            rt2r = rt1r;
            h11 = h12 * s;
            h12 = -h11;
          } else {
            rt1r = t3 + h12;
            rt2r = t3 - h12;
            if (muDoubleScalarAbs(rt1r - h22) <=
                muDoubleScalarAbs(rt2r - h22)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            h11 = 0.0;
            h12 = 0.0;
          }
        }
        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          temp_tmp_tmp = m + ((m - 1) << 2);
          h21 = h[temp_tmp_tmp - 1];
          t3 = h21 - rt2r;
          s = (muDoubleScalarAbs(t3) + muDoubleScalarAbs(h12)) +
              muDoubleScalarAbs(h[temp_tmp_tmp]);
          h21s = h[temp_tmp_tmp] / s;
          nr = m + (m << 2);
          v[0] = (h21s * h[nr - 1] + t3 * (t3 / s)) - h11 * (h12 / s);
          v[1] = h21s * (((h21 + h[nr]) - rt1r) - rt2r);
          v[2] = h21s * h[nr + 1];
          s = (muDoubleScalarAbs(v[0]) + muDoubleScalarAbs(v[1])) +
              muDoubleScalarAbs(v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
          v[2] /= s;
          if ((m == k + 1) ||
              (muDoubleScalarAbs(h[m - 1]) *
                   (muDoubleScalarAbs(v[1]) + muDoubleScalarAbs(v[2])) <=
               2.2204460492503131E-16 * muDoubleScalarAbs(v[0]) *
                   ((muDoubleScalarAbs(h[0]) +
                     muDoubleScalarAbs(h[temp_tmp_tmp - 1])) +
                    muDoubleScalarAbs(h[nr])))) {
            exitg3 = true;
          } else {
            m--;
          }
        }
        for (b_i = m; b_i <= i; b_i++) {
          temp_tmp_tmp = (i - b_i) + 2;
          nr = muIntScalarMin_sint32(3, temp_tmp_tmp);
          if (b_i > m) {
            temp_tmp_tmp = (((b_i - 2) << 2) + b_i) - 1;
            for (j = 0; j < nr; j++) {
              v[j] = h[temp_tmp_tmp + j];
            }
          }
          h11 = v[0];
          c_st.site = &sf_emlrtRSI;
          s = b_xzlarfg(&c_st, nr, &h11, v);
          if (b_i > m) {
            temp_tmp_tmp = b_i + ((b_i - 2) << 2);
            h[temp_tmp_tmp - 1] = h11;
            h[temp_tmp_tmp] = 0.0;
            if (b_i < i) {
              h[b_i + 1] = 0.0;
            }
          } else if (m > k + 1) {
            h[b_i - 1] *= 1.0 - s;
          }
          h22 = v[1];
          rt2r = s * v[1];
          if (nr == 3) {
            int32_T b_scalarLB;
            h21s = v[2];
            t3 = s * v[2];
            for (j = b_i; j < 5; j++) {
              temp_tmp_tmp = b_i + ((j - 1) << 2);
              h11 = h[temp_tmp_tmp - 1];
              h12 = h[temp_tmp_tmp];
              h21 = h[temp_tmp_tmp + 1];
              rt1r = (h11 + h22 * h12) + h21s * h21;
              h11 -= rt1r * s;
              h[temp_tmp_tmp - 1] = h11;
              h12 -= rt1r * rt2r;
              h[temp_tmp_tmp] = h12;
              h21 -= rt1r * t3;
              h[temp_tmp_tmp + 1] = h21;
            }
            temp_tmp_tmp = b_i + 3;
            nr = i + 1;
            c_i = muIntScalarMin_sint32(temp_tmp_tmp, nr);
            b_scalarLB = (c_i / 2) << 1;
            temp_tmp_tmp = b_scalarLB - 2;
            for (j = 0; j <= temp_tmp_tmp; j += 2) {
              __m128d r1;
              __m128d r2;
              __m128d r3;
              b_temp_tmp_tmp = j + (b_i << 2);
              r = _mm_loadu_pd(&h[b_temp_tmp_tmp]);
              nr = j + ((b_i + 1) << 2);
              r1 = _mm_loadu_pd(&h[nr]);
              scalarLB = j + ((b_i - 1) << 2);
              r2 = _mm_loadu_pd(&h[scalarLB]);
              r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(h22), r)),
                              _mm_mul_pd(_mm_set1_pd(h21s), r1));
              _mm_storeu_pd(&h[scalarLB],
                            _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[b_temp_tmp_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt2r))));
              _mm_storeu_pd(&h[nr],
                            _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(t3))));
            }
            for (j = b_scalarLB; j < c_i; j++) {
              temp_tmp_tmp = j + ((b_i - 1) << 2);
              h11 = h[temp_tmp_tmp];
              b_temp_tmp_tmp = j + (b_i << 2);
              h12 = h[b_temp_tmp_tmp];
              nr = j + ((b_i + 1) << 2);
              h21 = h[nr];
              rt1r = (h11 + h22 * h12) + h21s * h21;
              h11 -= rt1r * s;
              h[temp_tmp_tmp] = h11;
              h12 -= rt1r * rt2r;
              h[b_temp_tmp_tmp] = h12;
              h21 -= rt1r * t3;
              h[nr] = h21;
            }
          } else if (nr == 2) {
            for (j = b_i; j < 5; j++) {
              temp_tmp_tmp = b_i + ((j - 1) << 2);
              h11 = h[temp_tmp_tmp - 1];
              h12 = h[temp_tmp_tmp];
              rt1r = h11 + h22 * h12;
              h11 -= rt1r * s;
              h[temp_tmp_tmp - 1] = h11;
              h12 -= rt1r * rt2r;
              h[temp_tmp_tmp] = h12;
            }
            scalarLB = ((i + 1) / 2) << 1;
            temp_tmp_tmp = scalarLB - 2;
            for (j = 0; j <= temp_tmp_tmp; j += 2) {
              __m128d r1;
              __m128d r2;
              b_temp_tmp_tmp = j + (b_i << 2);
              r = _mm_loadu_pd(&h[b_temp_tmp_tmp]);
              nr = j + ((b_i - 1) << 2);
              r1 = _mm_loadu_pd(&h[nr]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(h22), r));
              _mm_storeu_pd(&h[nr],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[b_temp_tmp_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt2r))));
            }
            for (j = scalarLB; j <= i; j++) {
              temp_tmp_tmp = j + ((b_i - 1) << 2);
              h11 = h[temp_tmp_tmp];
              b_temp_tmp_tmp = j + (b_i << 2);
              h12 = h[b_temp_tmp_tmp];
              rt1r = h11 + h22 * h12;
              h11 -= rt1r * s;
              h[temp_tmp_tmp] = h11;
              h12 -= rt1r * rt2r;
              h[b_temp_tmp_tmp] = h12;
            }
          }
        }
        its++;
      }
    }
    if (!converged) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((l != i + 1) && (l == i)) {
        scalarLB = i << 2;
        temp_tmp_tmp = i + scalarLB;
        h11 = h[temp_tmp_tmp - 1];
        c_i = (i - 1) << 2;
        nr = i + c_i;
        h12 = h[nr];
        h21 = h[temp_tmp_tmp];
        xdlanv2(&h[nr - 1], &h11, &h12, &h21, &t3, &h21s, &h22, &rt2r, &rt1r);
        h[temp_tmp_tmp - 1] = h11;
        h[nr] = h12;
        h[temp_tmp_tmp] = h21;
        if (i + 1 < 4) {
          temp_tmp_tmp = 2 - i;
          nr = ((i + 1) << 2) + i;
          for (j = 0; j <= temp_tmp_tmp; j++) {
            b_temp_tmp_tmp = nr + (j << 2);
            h11 = h[b_temp_tmp_tmp];
            h12 = h[b_temp_tmp_tmp - 1];
            h[b_temp_tmp_tmp] = rt2r * h11 - rt1r * h12;
            h[b_temp_tmp_tmp - 1] = rt2r * h12 + rt1r * h11;
          }
        }
        if (i - 1 >= 1) {
          for (j = 0; j <= i - 2; j++) {
            temp_tmp_tmp = scalarLB + j;
            h11 = h[temp_tmp_tmp];
            nr = c_i + j;
            h12 = h[nr];
            h[temp_tmp_tmp] = rt2r * h11 - rt1r * h12;
            h[nr] = rt2r * h12 + rt1r * h11;
          }
        }
      }
      kdefl = 0;
      i = l - 2;
    }
  }
  for (j = 0; j < 2; j++) {
    for (b_i = j + 3; b_i < 5; b_i++) {
      h[(b_i + (j << 2)) - 1] = 0.0;
    }
  }
  return info;
}

int32_T xhseqr(const emlrtStack *sp, real_T h[9])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v[3];
  real_T d_sum;
  real_T h12;
  real_T h21;
  real_T h22;
  real_T rt1r;
  real_T rt2r;
  real_T s;
  real_T temp;
  int32_T b_k;
  int32_T c_k;
  int32_T i;
  int32_T info;
  int32_T kdefl;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &lf_emlrtRSI;
  b_st.site = &mf_emlrtRSI;
  info = 0;
  h[2] = 0.0;
  kdefl = 0;
  i = 2;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int32_T its;
    int32_T ix_tmp;
    int32_T iy;
    int32_T k;
    int32_T scalarLB;
    int32_T temp_tmp_tmp;
    boolean_T converged;
    boolean_T exitg2;
    k = -1;
    converged = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      boolean_T exitg3;
      k = i - 1;
      exitg3 = false;
      while ((!exitg3) && (k + 2 > 1)) {
        iy = k + 3 * k;
        rt1r = muDoubleScalarAbs(h[iy + 1]);
        if (rt1r <= 3.0062525400134592E-292) {
          exitg3 = true;
        } else {
          ix_tmp = 3 * (k + 1);
          temp_tmp_tmp = k + ix_tmp;
          h12 = muDoubleScalarAbs(h[temp_tmp_tmp + 1]);
          temp = muDoubleScalarAbs(h[iy]) + h12;
          if (temp == 0.0) {
            if (k >= 1) {
              temp = muDoubleScalarAbs(h[k]);
            }
            if (k + 3 <= 3) {
              temp += muDoubleScalarAbs(h[ix_tmp + 2]);
            }
          }
          if (rt1r <= 2.2204460492503131E-16 * temp) {
            h21 = muDoubleScalarAbs(h[temp_tmp_tmp]);
            temp = muDoubleScalarAbs(h[iy] - h[temp_tmp_tmp + 1]);
            rt2r = muDoubleScalarMax(h12, temp);
            temp = muDoubleScalarMin(h12, temp);
            s = rt2r + temp;
            if (muDoubleScalarMin(rt1r, h21) *
                    (muDoubleScalarMax(rt1r, h21) / s) <=
                muDoubleScalarMax(3.0062525400134592E-292,
                                  2.2204460492503131E-16 *
                                      (temp * (rt2r / s)))) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }
      if (k + 2 > 1) {
        h[(k + 3 * k) + 1] = 0.0;
      }
      if (k + 2 >= i) {
        converged = true;
        exitg2 = true;
      } else {
        __m128d r;
        int32_T m;
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          s = muDoubleScalarAbs(h[i + 3 * (i - 1)]) +
              muDoubleScalarAbs(h[i - 1]);
          temp = 0.75 * s + h[i + 3 * i];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = temp;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          s = muDoubleScalarAbs(h[1]) + muDoubleScalarAbs(h[5]);
          temp = 0.75 * s + h[0];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = temp;
        } else {
          iy = i + 3 * (i - 1);
          temp = h[iy - 1];
          h21 = h[iy];
          iy = i + 3 * i;
          h12 = h[iy - 1];
          h22 = h[iy];
        }
        s = ((muDoubleScalarAbs(temp) + muDoubleScalarAbs(h12)) +
             muDoubleScalarAbs(h21)) +
            muDoubleScalarAbs(h22);
        if (s == 0.0) {
          rt1r = 0.0;
          temp = 0.0;
          rt2r = 0.0;
          h12 = 0.0;
        } else {
          temp /= s;
          h21 /= s;
          h12 /= s;
          h22 /= s;
          d_sum = (temp + h22) / 2.0;
          temp = (temp - d_sum) * (h22 - d_sum) - h12 * h21;
          h12 = muDoubleScalarSqrt(muDoubleScalarAbs(temp));
          if (temp >= 0.0) {
            rt1r = d_sum * s;
            rt2r = rt1r;
            temp = h12 * s;
            h12 = -temp;
          } else {
            rt1r = d_sum + h12;
            rt2r = d_sum - h12;
            if (muDoubleScalarAbs(rt1r - h22) <=
                muDoubleScalarAbs(rt2r - h22)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            temp = 0.0;
            h12 = 0.0;
          }
        }
        m = i - 1;
        if (i - 1 >= 1) {
          s = (muDoubleScalarAbs(h[0] - rt2r) + muDoubleScalarAbs(h12)) +
              muDoubleScalarAbs(h[1]);
          h21 = h[1] / s;
          v[0] = (h21 * h[3] + (h[0] - rt1r) * ((h[0] - rt2r) / s)) -
                 temp * (h12 / s);
          v[1] = h21 * (((h[0] + h[4]) - rt1r) - rt2r);
          v[2] = h21 * h[5];
          s = (muDoubleScalarAbs(v[0]) + muDoubleScalarAbs(v[1])) +
              muDoubleScalarAbs(v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
          v[2] /= s;
        }
        for (c_k = m; c_k <= i; c_k++) {
          iy = (i - c_k) + 2;
          iy = muIntScalarMin_sint32(3, iy);
          if (c_k > i - 1) {
            ix_tmp = ((c_k - 2) * 3 + c_k) - 1;
            for (b_k = 0; b_k < iy; b_k++) {
              v[b_k] = h[ix_tmp + b_k];
            }
          }
          temp = v[0];
          c_st.site = &sf_emlrtRSI;
          h22 = b_xzlarfg(&c_st, iy, &temp, v);
          if (c_k > i - 1) {
            h[c_k - 1] = temp;
            h[c_k] = 0.0;
            if (c_k < i) {
              /* Check node always fails. would cause program termination and
               * was eliminated */
            }
          }
          rt2r = v[1];
          rt1r = h22 * v[1];
          if (iy == 3) {
            int32_T b_scalarLB;
            h12 = v[2];
            h21 = h22 * v[2];
            for (b_k = c_k; b_k < 4; b_k++) {
              iy = 3 * (b_k - 1);
              ix_tmp = c_k + iy;
              temp = h[ix_tmp - 1];
              d_sum = (temp + rt2r * h[ix_tmp]) + h12 * h[iy + 2];
              h[ix_tmp - 1] = temp - d_sum * h22;
              h[ix_tmp] -= d_sum * rt1r;
              h[iy + 2] -= d_sum * h21;
            }
            iy = c_k + 3;
            ix_tmp = i + 1;
            scalarLB = muIntScalarMin_sint32(iy, ix_tmp);
            b_scalarLB = (scalarLB / 2) << 1;
            iy = b_scalarLB - 2;
            for (b_k = 0; b_k <= iy; b_k += 2) {
              __m128d r1;
              __m128d r2;
              ix_tmp = b_k + 3 * c_k;
              r = _mm_loadu_pd(&h[ix_tmp]);
              temp_tmp_tmp = b_k + 3 * (c_k - 1);
              r1 = _mm_loadu_pd(&h[temp_tmp_tmp]);
              r2 = _mm_loadu_pd(&h[b_k + 6]);
              r1 = _mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(rt2r), r)),
                              _mm_mul_pd(_mm_set1_pd(h12), r2));
              r = _mm_loadu_pd(&h[temp_tmp_tmp]);
              _mm_storeu_pd(&h[temp_tmp_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r1, _mm_set1_pd(h22))));
              r = _mm_loadu_pd(&h[ix_tmp]);
              _mm_storeu_pd(&h[ix_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r1, _mm_set1_pd(rt1r))));
              r = _mm_loadu_pd(&h[b_k + 6]);
              _mm_storeu_pd(&h[b_k + 6],
                            _mm_sub_pd(r, _mm_mul_pd(r1, _mm_set1_pd(h21))));
            }
            for (b_k = b_scalarLB; b_k < scalarLB; b_k++) {
              iy = b_k + 3 * (c_k - 1);
              temp = h[iy];
              ix_tmp = b_k + 3 * c_k;
              d_sum = (temp + rt2r * h[ix_tmp]) + h12 * h[b_k + 6];
              h[iy] = temp - d_sum * h22;
              h[ix_tmp] -= d_sum * rt1r;
              h[b_k + 6] -= d_sum * h21;
            }
          } else if (iy == 2) {
            for (b_k = c_k; b_k < 4; b_k++) {
              iy = c_k + 3 * (b_k - 1);
              temp = h[iy - 1];
              h12 = h[iy];
              d_sum = temp + rt2r * h12;
              temp -= d_sum * h22;
              h[iy - 1] = temp;
              h12 -= d_sum * rt1r;
              h[iy] = h12;
            }
            scalarLB = ((i + 1) / 2) << 1;
            iy = scalarLB - 2;
            for (b_k = 0; b_k <= iy; b_k += 2) {
              __m128d r1;
              __m128d r2;
              ix_tmp = b_k + 3 * c_k;
              r = _mm_loadu_pd(&h[ix_tmp]);
              temp_tmp_tmp = b_k + 3 * (c_k - 1);
              r1 = _mm_loadu_pd(&h[temp_tmp_tmp]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(rt2r), r));
              _mm_storeu_pd(&h[temp_tmp_tmp],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(h22))));
              _mm_storeu_pd(&h[ix_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt1r))));
            }
            for (b_k = scalarLB; b_k <= i; b_k++) {
              iy = b_k + 3 * (c_k - 1);
              temp = h[iy];
              ix_tmp = b_k + 3 * c_k;
              h12 = h[ix_tmp];
              d_sum = temp + rt2r * h12;
              temp -= d_sum * h22;
              h[iy] = temp;
              h12 -= d_sum * rt1r;
              h[ix_tmp] = h12;
            }
          }
        }
        its++;
      }
    }
    if (!converged) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((k + 2 != i + 1) && (k + 2 == i)) {
        iy = i + 3 * i;
        temp = h[iy - 1];
        scalarLB = 3 * (i - 1);
        ix_tmp = i + scalarLB;
        h12 = h[ix_tmp];
        h21 = h[iy];
        xdlanv2(&h[ix_tmp - 1], &temp, &h12, &h21, &rt2r, &rt1r, &d_sum, &h22,
                &s);
        h[iy - 1] = temp;
        h[ix_tmp] = h12;
        h[iy] = h21;
        if (i + 1 < 3) {
          iy = 1 - i;
          ix_tmp = (i + 1) * 3 + i;
          for (b_k = 0; b_k <= iy; b_k++) {
            temp_tmp_tmp = ix_tmp + b_k * 3;
            temp = h[temp_tmp_tmp];
            h12 = h[temp_tmp_tmp - 1];
            h[temp_tmp_tmp] = h22 * temp - s * h12;
            h[temp_tmp_tmp - 1] = h22 * h12 + s * temp;
          }
        }
        if (i - 1 >= 1) {
          iy = i * 3;
          temp = h22 * h[scalarLB] + s * h[iy];
          h[iy] = h22 * h[iy] - s * h[scalarLB];
          h[scalarLB] = temp;
        }
      }
      kdefl = 0;
      i = k;
    }
  }
  h[2] = 0.0;
  return info;
}

/* End of code generation (xhseqr.c) */
