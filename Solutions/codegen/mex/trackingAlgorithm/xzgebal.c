/*
 * xzgebal.c
 *
 * Code generation for function 'xzgebal'
 *
 */

/* Include files */
#include "xzgebal.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "xnrm2.h"
#include "xswap.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo ch_emlrtRSI = {
    39,        /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo dh_emlrtRSI = {
    55,        /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo eh_emlrtRSI = {
    74,        /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo fh_emlrtRSI = {
    76,        /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo gh_emlrtRSI = {
    92,        /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo hh_emlrtRSI = {
    95,        /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo ih_emlrtRSI = {
    119,       /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo jh_emlrtRSI = {
    120,       /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo kh_emlrtRSI = {
    122,       /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo lh_emlrtRSI = {
    124,       /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

static emlrtRSInfo mh_emlrtRSI = {
    170,       /* lineNo */
    "xzgebal", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgebal.m" /* pathName
                                                                     */
};

/* Function Definitions */
int32_T xzgebal(const emlrtStack *sp, real_T A[9], int32_T *ihi,
                real_T scale[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T temp;
  int32_T exitg5;
  int32_T ilo;
  int32_T ix;
  int32_T ix0_tmp;
  int32_T iy;
  int32_T k;
  int32_T kend;
  int32_T l;
  int32_T temp_tmp;
  boolean_T notdone;
  boolean_T skipThisRow;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  scale[0] = 1.0;
  scale[1] = 1.0;
  scale[2] = 1.0;
  ilo = 1;
  l = 3;
  notdone = true;
  do {
    exitg5 = 0;
    if (notdone) {
      int32_T exitg4;
      notdone = false;
      iy = l;
      do {
        exitg4 = 0;
        if (iy > 0) {
          boolean_T exitg6;
          skipThisRow = false;
          st.site = &ch_emlrtRSI;
          kend = 0;
          exitg6 = false;
          while ((!exitg6) && (kend <= l - 1)) {
            if ((kend + 1 == iy) || (!(A[(iy + 3 * kend) - 1] != 0.0))) {
              kend++;
            } else {
              skipThisRow = true;
              exitg6 = true;
            }
          }
          if (skipThisRow) {
            iy--;
          } else {
            scale[l - 1] = iy;
            if (iy != l) {
              st.site = &dh_emlrtRSI;
              xswap(&st, l, A, (iy - 1) * 3 + 1, (l - 1) * 3 + 1);
              temp = A[iy - 1];
              A[iy - 1] = A[l - 1];
              A[l - 1] = temp;
              temp = A[iy + 2];
              A[iy + 2] = A[l + 2];
              A[l + 2] = temp;
              temp = A[iy + 5];
              A[iy + 5] = A[l + 5];
              A[l + 5] = temp;
            }
            exitg4 = 1;
          }
        } else {
          exitg4 = 2;
        }
      } while (exitg4 == 0);
      if (exitg4 == 1) {
        if (l == 1) {
          ilo = 1;
          *ihi = 1;
          exitg5 = 1;
        } else {
          l--;
          notdone = true;
        }
      }
    } else {
      notdone = true;
      while (notdone) {
        boolean_T exitg6;
        notdone = false;
        st.site = &eh_emlrtRSI;
        kend = ilo;
        exitg6 = false;
        while ((!exitg6) && (kend <= l)) {
          boolean_T exitg7;
          skipThisRow = false;
          st.site = &fh_emlrtRSI;
          iy = ilo;
          exitg7 = false;
          while ((!exitg7) && (iy <= l)) {
            if ((iy == kend) || (!(A[(iy + 3 * (kend - 1)) - 1] != 0.0))) {
              iy++;
            } else {
              skipThisRow = true;
              exitg7 = true;
            }
          }
          if (skipThisRow) {
            kend++;
          } else {
            scale[ilo - 1] = kend;
            if (kend != ilo) {
              iy = (ilo - 1) * 3;
              st.site = &gh_emlrtRSI;
              xswap(&st, l, A, (kend - 1) * 3 + 1, iy + 1);
              ix = (iy + kend) - 1;
              iy = (iy + ilo) - 1;
              st.site = &hh_emlrtRSI;
              b_st.site = &nh_emlrtRSI;
              c_st.site = &oh_emlrtRSI;
              if (4 - ilo > 2147483646) {
                d_st.site = &k_emlrtRSI;
                check_forloop_overflow_error(&d_st);
              }
              kend = (uint8_T)(4 - ilo);
              for (k = 0; k < kend; k++) {
                temp_tmp = ix + k * 3;
                temp = A[temp_tmp];
                ix0_tmp = iy + k * 3;
                A[temp_tmp] = A[ix0_tmp];
                A[ix0_tmp] = temp;
              }
            }
            ilo++;
            notdone = true;
            exitg6 = true;
          }
        }
      }
      *ihi = l;
      skipThisRow = false;
      exitg5 = 2;
    }
  } while (exitg5 == 0);
  if (exitg5 != 1) {
    boolean_T exitg3;
    exitg3 = false;
    while ((!exitg3) && (!skipThisRow)) {
      int32_T exitg2;
      skipThisRow = true;
      st.site = &ih_emlrtRSI;
      ix = ilo - 1;
      do {
        exitg2 = 0;
        if (ix + 1 <= l) {
          real_T b_s;
          real_T c;
          real_T ca;
          real_T r;
          real_T s;
          iy = (l - ilo) + 1;
          st.site = &jh_emlrtRSI;
          c = b_xnrm2(&st, iy, A, ix * 3 + ilo);
          temp_tmp = (ilo - 1) * 3 + ix;
          ix0_tmp = temp_tmp + 1;
          r = 0.0;
          if (iy >= 1) {
            if (iy == 1) {
              r = muDoubleScalarAbs(A[temp_tmp]);
            } else {
              temp = 3.3121686421112381E-170;
              kend = (temp_tmp + (iy - 1) * 3) + 1;
              for (k = ix0_tmp; k <= kend; k += 3) {
                s = muDoubleScalarAbs(A[k - 1]);
                if (s > temp) {
                  b_s = temp / s;
                  r = r * b_s * b_s + 1.0;
                  temp = s;
                } else {
                  b_s = s / temp;
                  r += b_s * b_s;
                }
              }
              r = temp * muDoubleScalarSqrt(r);
              if (muDoubleScalarIsNaN(r)) {
                iy = temp_tmp + 1;
                int32_T exitg8;
                do {
                  exitg8 = 0;
                  if (iy <= kend) {
                    if (muDoubleScalarIsNaN(A[iy - 1])) {
                      exitg8 = 1;
                    } else {
                      iy += 3;
                    }
                  } else {
                    r = rtInf;
                    exitg8 = 1;
                  }
                } while (exitg8 == 0);
              }
            }
          }
          st.site = &kh_emlrtRSI;
          kend = ix * 3;
          b_st.site = &ph_emlrtRSI;
          iy = 1;
          if (l > 1) {
            temp = muDoubleScalarAbs(A[kend]);
            c_st.site = &qh_emlrtRSI;
            for (k = 2; k <= l; k++) {
              s = muDoubleScalarAbs(A[(kend + k) - 1]);
              if (s > temp) {
                iy = k;
                temp = s;
              }
            }
          }
          ca = muDoubleScalarAbs(A[(iy + 3 * ix) - 1]);
          st.site = &lh_emlrtRSI;
          kend = 4 - ilo;
          b_st.site = &ph_emlrtRSI;
          if (4 - ilo < 1) {
            iy = 0;
          } else {
            iy = 1;
            if (4 - ilo > 1) {
              temp = muDoubleScalarAbs(A[temp_tmp]);
              c_st.site = &qh_emlrtRSI;
              if (4 - ilo > 2147483646) {
                d_st.site = &k_emlrtRSI;
                check_forloop_overflow_error(&d_st);
              }
              for (k = 2; k <= kend; k++) {
                s = muDoubleScalarAbs(A[temp_tmp + (k - 1) * 3]);
                if (s > temp) {
                  iy = k;
                  temp = s;
                }
              }
            }
          }
          temp = muDoubleScalarAbs(A[ix + 3 * ((iy + ilo) - 2)]);
          if ((c == 0.0) || (r == 0.0)) {
            ix++;
          } else {
            real_T f;
            int32_T exitg1;
            s = r / 2.0;
            f = 1.0;
            b_s = c + r;
            do {
              exitg1 = 0;
              if ((c < s) &&
                  (muDoubleScalarMax(f, muDoubleScalarMax(c, ca)) <
                   4.9896007738368E+291) &&
                  (muDoubleScalarMin(r, muDoubleScalarMin(s, temp)) >
                   2.0041683600089728E-292)) {
                if (muDoubleScalarIsNaN(((((c + f) + ca) + r) + s) + temp)) {
                  exitg1 = 1;
                } else {
                  f *= 2.0;
                  c *= 2.0;
                  ca *= 2.0;
                  r /= 2.0;
                  s /= 2.0;
                  temp /= 2.0;
                }
              } else {
                s = c / 2.0;
                while ((s >= r) &&
                       (muDoubleScalarMax(r, temp) < 4.9896007738368E+291) &&
                       (muDoubleScalarMin(muDoubleScalarMin(f, c),
                                          muDoubleScalarMin(s, ca)) >
                        2.0041683600089728E-292)) {
                  f /= 2.0;
                  c /= 2.0;
                  s /= 2.0;
                  ca /= 2.0;
                  r *= 2.0;
                  temp *= 2.0;
                }
                if ((!(c + r >= 0.95 * b_s)) &&
                    ((!(f < 1.0)) || (!(scale[ix] < 1.0)) ||
                     (!(f * scale[ix] <= 1.0020841800044864E-292))) &&
                    ((!(f > 1.0)) || (!(scale[ix] > 1.0)) ||
                     (!(scale[ix] >= 9.9792015476736E+291 / f)))) {
                  temp = 1.0 / f;
                  scale[ix] *= f;
                  iy = (temp_tmp + 3 * (3 - ilo)) + 1;
                  for (k = ix0_tmp; k <= iy; k += 3) {
                    A[k - 1] *= temp;
                  }
                  st.site = &mh_emlrtRSI;
                  kend = ix * 3 + 1;
                  b_st.site = &ld_emlrtRSI;
                  iy = (kend + l) - 1;
                  c_st.site = &md_emlrtRSI;
                  if ((kend <= iy) && (iy > 2147483646)) {
                    d_st.site = &k_emlrtRSI;
                    check_forloop_overflow_error(&d_st);
                  }
                  temp_tmp = ((iy - kend) + 1) / 2 * 2 + kend;
                  ix0_tmp = temp_tmp - 2;
                  for (k = kend; k <= ix0_tmp; k += 2) {
                    __m128d b_r;
                    b_r = _mm_loadu_pd(&A[k - 1]);
                    b_r = _mm_mul_pd(_mm_set1_pd(f), b_r);
                    _mm_storeu_pd(&A[k - 1], b_r);
                  }
                  for (k = temp_tmp; k <= iy; k++) {
                    A[k - 1] *= f;
                  }
                  skipThisRow = false;
                }
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = 2;
            } else {
              ix++;
            }
          }
        } else {
          exitg2 = 1;
        }
      } while (exitg2 == 0);
      if (exitg2 != 1) {
        exitg3 = true;
      }
    }
  }
  return ilo;
}

/* End of code generation (xzgebal.c) */
