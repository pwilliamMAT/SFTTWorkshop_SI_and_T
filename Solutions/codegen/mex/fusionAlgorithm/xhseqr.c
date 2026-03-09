/*
 * xhseqr.c
 *
 * Code generation for function 'xhseqr'
 *
 */

/* Include files */
#include "xhseqr.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xrot.h"
#include "xzlarfg.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo bv_emlrtRSI = {
    21,       /* lineNo */
    "xhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xhseqr.m" /* pathName */
};

static emlrtRSInfo cv_emlrtRSI = {
    16,        /* lineNo */
    "xdhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdhseqr.m" /* pathName */
};

/* Function Definitions */
int32_T xhseqr(const emlrtStack *sp, real_T h[36], real_T z[36])
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
  st.site = &bv_emlrtRSI;
  b_st.site = &cv_emlrtRSI;
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
    int32_T c_i;
    int32_T its;
    int32_T iy;
    int32_T l;
    int32_T nr;
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
        iy = k + 6 * (k - 1);
        h21s = muDoubleScalarAbs(h[iy]);
        if (h21s <= 6.0125050800269183E-292) {
          exitg3 = true;
        } else {
          nr = k + 6 * k;
          h21 = muDoubleScalarAbs(h[nr]);
          h11 = muDoubleScalarAbs(h[iy - 1]) + h21;
          if (h11 == 0.0) {
            if (k - 1 >= 1) {
              h11 = muDoubleScalarAbs(h[(k + 6 * (k - 2)) - 1]);
            }
            if (k + 2 <= 6) {
              h11 += muDoubleScalarAbs(h[nr + 1]);
            }
          }
          if (h21s <= 2.2204460492503131E-16 * h11) {
            h11 = muDoubleScalarAbs(h[nr - 1]);
            h12 = muDoubleScalarAbs(h[iy - 1] - h[nr]);
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
          iy = k + 6 * k;
          s = muDoubleScalarAbs(h[iy + 1]) +
              muDoubleScalarAbs(h[(k + 6 * (k + 1)) + 2]);
          h11 = 0.75 * s + h[iy];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = h11;
        } else {
          iy = i + 6 * (i - 1);
          h11 = h[iy - 1];
          h21 = h[iy];
          iy = i + 6 * i;
          h12 = h[iy - 1];
          h22 = h[iy];
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
          iy = m + 6 * (m - 1);
          h21 = h[iy - 1];
          t3 = h21 - rt2r;
          s = (muDoubleScalarAbs(t3) + muDoubleScalarAbs(h12)) +
              muDoubleScalarAbs(h[iy]);
          h21s = h[iy] / s;
          nr = m + 6 * m;
          v[0] = (h21s * h[nr - 1] + t3 * (t3 / s)) - h11 * (h12 / s);
          v[1] = h21s * (((h21 + h[nr]) - rt1r) - rt2r);
          v[2] = h21s * h[nr + 1];
          s = (muDoubleScalarAbs(v[0]) + muDoubleScalarAbs(v[1])) +
              muDoubleScalarAbs(v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
          v[2] /= s;
          if (m == k + 1) {
            exitg3 = true;
          } else {
            temp_tmp_tmp = m + 6 * (m - 2);
            if (muDoubleScalarAbs(h[temp_tmp_tmp - 1]) *
                    (muDoubleScalarAbs(v[1]) + muDoubleScalarAbs(v[2])) <=
                2.2204460492503131E-16 * muDoubleScalarAbs(v[0]) *
                    ((muDoubleScalarAbs(h[temp_tmp_tmp - 2]) +
                      muDoubleScalarAbs(h[iy - 1])) +
                     muDoubleScalarAbs(h[nr]))) {
              exitg3 = true;
            } else {
              m--;
            }
          }
        }
        for (b_i = m; b_i <= i; b_i++) {
          iy = (i - b_i) + 2;
          nr = muIntScalarMin_sint32(3, iy);
          if (b_i > m) {
            iy = ((b_i - 2) * 6 + b_i) - 1;
            for (j = 0; j < nr; j++) {
              v[j] = h[iy + j];
            }
          }
          h11 = v[0];
          c_st.site = &cf_emlrtRSI;
          s = xzlarfg(&c_st, nr, &h11, v);
          if (b_i > m) {
            iy = b_i + 6 * (b_i - 2);
            h[iy - 1] = h11;
            h[iy] = 0.0;
            if (b_i < i) {
              h[iy + 1] = 0.0;
            }
          } else if (m > k + 1) {
            iy = (b_i + 6 * (b_i - 2)) - 1;
            h[iy] *= 1.0 - s;
          }
          h22 = v[1];
          rt2r = s * v[1];
          if (nr == 3) {
            __m128d r1;
            __m128d r2;
            __m128d r3;
            int32_T b_scalarLB;
            int32_T scalarLB;
            h21s = v[2];
            t3 = s * v[2];
            for (j = b_i; j < 7; j++) {
              iy = b_i + 6 * (j - 1);
              h11 = h[iy - 1];
              h12 = h[iy];
              h21 = h[iy + 1];
              rt1r = (h11 + h22 * h12) + h21s * h21;
              h11 -= rt1r * s;
              h[iy - 1] = h11;
              h12 -= rt1r * rt2r;
              h[iy] = h12;
              h21 -= rt1r * t3;
              h[iy + 1] = h21;
            }
            iy = b_i + 3;
            nr = i + 1;
            scalarLB = muIntScalarMin_sint32(iy, nr);
            b_scalarLB = (scalarLB / 2) << 1;
            iy = b_scalarLB - 2;
            for (j = 0; j <= iy; j += 2) {
              temp_tmp_tmp = j + 6 * b_i;
              r = _mm_loadu_pd(&h[temp_tmp_tmp]);
              c_i = j + 6 * (b_i + 1);
              r1 = _mm_loadu_pd(&h[c_i]);
              nr = j + 6 * (b_i - 1);
              r2 = _mm_loadu_pd(&h[nr]);
              r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(h22), r)),
                              _mm_mul_pd(_mm_set1_pd(h21s), r1));
              _mm_storeu_pd(&h[nr],
                            _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[temp_tmp_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt2r))));
              _mm_storeu_pd(&h[c_i],
                            _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(t3))));
            }
            for (j = b_scalarLB; j < scalarLB; j++) {
              iy = j + 6 * (b_i - 1);
              h11 = h[iy];
              temp_tmp_tmp = j + 6 * b_i;
              h12 = h[temp_tmp_tmp];
              c_i = j + 6 * (b_i + 1);
              h21 = h[c_i];
              rt1r = (h11 + h22 * h12) + h21s * h21;
              h11 -= rt1r * s;
              h[iy] = h11;
              h12 -= rt1r * rt2r;
              h[temp_tmp_tmp] = h12;
              h21 -= rt1r * t3;
              h[c_i] = h21;
            }
            __m128d r4;
            __m128d r5;
            __m128d r6;
            __m128d r7;
            __m128d r8;
            r = _mm_loadu_pd(&z[6 * b_i]);
            nr = 6 * (b_i + 1);
            r1 = _mm_loadu_pd(&z[nr]);
            temp_tmp_tmp = 6 * (b_i - 1);
            r2 = _mm_loadu_pd(&z[temp_tmp_tmp]);
            r3 = _mm_set1_pd(v[1]);
            r4 = _mm_set1_pd(v[2]);
            r5 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r3, r)),
                            _mm_mul_pd(r4, r1));
            r6 = _mm_set1_pd(s);
            _mm_storeu_pd(&z[temp_tmp_tmp], _mm_sub_pd(r2, _mm_mul_pd(r5, r6)));
            r7 = _mm_set1_pd(rt2r);
            _mm_storeu_pd(&z[6 * b_i], _mm_sub_pd(r, _mm_mul_pd(r5, r7)));
            r8 = _mm_set1_pd(t3);
            _mm_storeu_pd(&z[nr], _mm_sub_pd(r1, _mm_mul_pd(r5, r8)));
            iy = 6 * b_i + 2;
            r = _mm_loadu_pd(&z[iy]);
            r1 = _mm_loadu_pd(&z[nr + 2]);
            r2 = _mm_loadu_pd(&z[temp_tmp_tmp + 2]);
            r5 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r3, r)),
                            _mm_mul_pd(r4, r1));
            _mm_storeu_pd(&z[temp_tmp_tmp + 2],
                          _mm_sub_pd(r2, _mm_mul_pd(r5, r6)));
            _mm_storeu_pd(&z[iy], _mm_sub_pd(r, _mm_mul_pd(r5, r7)));
            _mm_storeu_pd(&z[nr + 2], _mm_sub_pd(r1, _mm_mul_pd(r5, r8)));
            iy = 6 * b_i + 4;
            r = _mm_loadu_pd(&z[iy]);
            r1 = _mm_loadu_pd(&z[nr + 4]);
            r2 = _mm_loadu_pd(&z[temp_tmp_tmp + 4]);
            r5 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r3, r)),
                            _mm_mul_pd(r4, r1));
            _mm_storeu_pd(&z[temp_tmp_tmp + 4],
                          _mm_sub_pd(r2, _mm_mul_pd(r5, r6)));
            _mm_storeu_pd(&z[iy], _mm_sub_pd(r, _mm_mul_pd(r5, r7)));
            _mm_storeu_pd(&z[nr + 4], _mm_sub_pd(r1, _mm_mul_pd(r5, r8)));
          } else if (nr == 2) {
            __m128d r1;
            __m128d r2;
            int32_T scalarLB;
            for (j = b_i; j < 7; j++) {
              iy = b_i + 6 * (j - 1);
              h11 = h[iy - 1];
              h12 = h[iy];
              rt1r = h11 + h22 * h12;
              h11 -= rt1r * s;
              h[iy - 1] = h11;
              h12 -= rt1r * rt2r;
              h[iy] = h12;
            }
            scalarLB = ((i + 1) / 2) << 1;
            iy = scalarLB - 2;
            for (j = 0; j <= iy; j += 2) {
              temp_tmp_tmp = j + 6 * b_i;
              r = _mm_loadu_pd(&h[temp_tmp_tmp]);
              c_i = j + 6 * (b_i - 1);
              r1 = _mm_loadu_pd(&h[c_i]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(h22), r));
              _mm_storeu_pd(&h[c_i],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[temp_tmp_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt2r))));
            }
            for (j = scalarLB; j <= i; j++) {
              iy = j + 6 * (b_i - 1);
              h11 = h[iy];
              nr = j + 6 * b_i;
              h12 = h[nr];
              rt1r = h11 + h22 * h12;
              h11 -= rt1r * s;
              h[iy] = h11;
              h12 -= rt1r * rt2r;
              h[nr] = h12;
            }
            __m128d r3;
            __m128d r4;
            __m128d r5;
            r = _mm_loadu_pd(&z[6 * b_i]);
            nr = 6 * (b_i - 1);
            r1 = _mm_loadu_pd(&z[nr]);
            r2 = _mm_set1_pd(v[1]);
            r3 = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            r4 = _mm_set1_pd(s);
            _mm_storeu_pd(&z[nr], _mm_sub_pd(r1, _mm_mul_pd(r3, r4)));
            r5 = _mm_set1_pd(rt2r);
            _mm_storeu_pd(&z[6 * b_i], _mm_sub_pd(r, _mm_mul_pd(r3, r5)));
            iy = 6 * b_i + 2;
            r = _mm_loadu_pd(&z[iy]);
            r1 = _mm_loadu_pd(&z[nr + 2]);
            r3 = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            _mm_storeu_pd(&z[nr + 2], _mm_sub_pd(r1, _mm_mul_pd(r3, r4)));
            _mm_storeu_pd(&z[iy], _mm_sub_pd(r, _mm_mul_pd(r3, r5)));
            iy = 6 * b_i + 4;
            r = _mm_loadu_pd(&z[iy]);
            r1 = _mm_loadu_pd(&z[nr + 4]);
            r3 = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            _mm_storeu_pd(&z[nr + 4], _mm_sub_pd(r1, _mm_mul_pd(r3, r4)));
            _mm_storeu_pd(&z[iy], _mm_sub_pd(r, _mm_mul_pd(r3, r5)));
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
        iy = i + 6 * i;
        h11 = h[iy - 1];
        c_i = 6 * (i - 1);
        nr = i + c_i;
        h12 = h[nr];
        h21 = h[iy];
        xdlanv2(&h[nr - 1], &h11, &h12, &h21, &t3, &h21s, &h22, &rt2r, &rt1r);
        h[iy - 1] = h11;
        h[nr] = h12;
        h[iy] = h21;
        if (i + 1 < 6) {
          iy = (i + 1) * 6 + i;
          c_st.site = &gf_emlrtRSI;
          xrot(&c_st, 5 - i, h, iy, iy + 1, rt2r, rt1r);
        }
        c_st.site = &hf_emlrtRSI;
        b_xrot(&c_st, i - 1, h, c_i + 1, i * 6 + 1, rt2r, rt1r);
        iy = i * 6;
        for (j = 0; j < 6; j++) {
          nr = iy + j;
          h11 = z[nr];
          temp_tmp_tmp = c_i + j;
          h12 = z[temp_tmp_tmp];
          z[nr] = rt2r * h11 - rt1r * h12;
          z[temp_tmp_tmp] = rt2r * h12 + rt1r * h11;
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

/* End of code generation (xhseqr.c) */
