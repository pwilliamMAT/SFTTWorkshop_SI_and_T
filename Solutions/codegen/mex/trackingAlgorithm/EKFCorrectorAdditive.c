/*
 * EKFCorrectorAdditive.c
 *
 * Code generation for function 'EKFCorrectorAdditive'
 *
 */

/* Include files */
#include "EKFCorrectorAdditive.h"
#include "rt_nonfinite.h"
#include "stateToMeasurementJacobian.h"
#include "stateToMeasurementWrapped.h"
#include "trackingAlgorithm_data.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ny_emlrtRSI = {
    163,                                        /* lineNo */
    "EKFCorrectorAdditive/measurementMatrices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrectorAddi"
    "tive.m" /* pathName */
};

static emlrtRSInfo qy_emlrtRSI = {
    121,                                                        /* lineNo */
    "EKFCorrectorAdditive/getMeasurementJacobianAndCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrectorAddi"
    "tive.m" /* pathName */
};

static emlrtRSInfo ry_emlrtRSI = {
    126,                                                        /* lineNo */
    "EKFCorrectorAdditive/getMeasurementJacobianAndCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrectorAddi"
    "tive.m" /* pathName */
};

static emlrtRSInfo sy_emlrtRSI = {
    137,                                                        /* lineNo */
    "EKFCorrectorAdditive/getMeasurementJacobianAndCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrectorAddi"
    "tive.m" /* pathName */
};

/* Function Definitions */
void c_EKFCorrectorAdditive_getMeasu(const emlrtStack *sp, const real_T Rs[16],
                                     const real_T x[6], const real_T S[36],
                                     const real_T varargin_2_OriginPosition[9],
                                     const real_T varargin_2_OriginVelocity[9],
                                     const real_T varargin_2_Orientation[27],
                                     real_T zEstimated[4], real_T Pxy[24],
                                     real_T Sy[16], real_T dHdx[24])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  real_T M[40];
  real_T b_S[36];
  real_T y_tmp[36];
  real_T Pxy_tmp[24];
  real_T b_y_tmp[24];
  real_T R[16];
  real_T tau[4];
  real_T work[4];
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T ii;
  int32_T itau;
  int32_T k;
  int32_T knt;
  int32_T lastc;
  int32_T lastv;
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
  st.site = &qy_emlrtRSI;
  b_st.site = &ny_emlrtRSI;
  stateToMeasurementJacobian(&b_st, x, varargin_2_OriginPosition,
                             varargin_2_OriginVelocity, varargin_2_Orientation,
                             dHdx);
  st.site = &ry_emlrtRSI;
  stateToMeasurementWrapped(&st, x, varargin_2_OriginPosition,
                            varargin_2_OriginVelocity, varargin_2_Orientation,
                            zEstimated);
  for (k = 0; k < 6; k++) {
    for (i = 0; i < 6; i++) {
      y_tmp[i + 6 * k] = S[k + 6 * i];
    }
  }
  for (k = 0; k < 4; k++) {
    for (i = 0; i < 6; i++) {
      Pxy_tmp[i + 6 * k] = dHdx[k + (i << 2)];
    }
  }
  memset(&b_S[0], 0, 36U * sizeof(real_T));
  for (k = 0; k < 6; k++) {
    b_i = 6 * k + 2;
    lastc = 6 * k + 4;
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&b_S[6 * k]);
      r1 = _mm_set1_pd(y_tmp[i + 6 * k]);
      _mm_storeu_pd(&b_S[6 * k],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&S[6 * i]), r1)));
      r = _mm_loadu_pd(&b_S[b_i]);
      _mm_storeu_pd(&b_S[b_i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&S[6 * i + 2]), r1)));
      r = _mm_loadu_pd(&b_S[lastc]);
      _mm_storeu_pd(&b_S[lastc],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&S[6 * i + 4]), r1)));
    }
  }
  memset(&Pxy[0], 0, 24U * sizeof(real_T));
  for (k = 0; k < 4; k++) {
    b_i = 6 * k + 2;
    lastc = 6 * k + 4;
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&b_S[6 * i]);
      r1 = _mm_loadu_pd(&Pxy[6 * k]);
      r2 = _mm_set1_pd(Pxy_tmp[i + 6 * k]);
      _mm_storeu_pd(&Pxy[6 * k], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&b_S[6 * i + 2]);
      r1 = _mm_loadu_pd(&Pxy[b_i]);
      _mm_storeu_pd(&Pxy[b_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&b_S[6 * i + 4]);
      r1 = _mm_loadu_pd(&Pxy[lastc]);
      _mm_storeu_pd(&Pxy[lastc], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  st.site = &sy_emlrtRSI;
  memset(&b_y_tmp[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    b_i = 6 * i + 2;
    lastc = 6 * i + 4;
    for (k = 0; k < 6; k++) {
      r = _mm_loadu_pd(&y_tmp[6 * k]);
      r1 = _mm_loadu_pd(&b_y_tmp[6 * i]);
      r2 = _mm_set1_pd(Pxy_tmp[k + 6 * i]);
      _mm_storeu_pd(&b_y_tmp[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&y_tmp[6 * k + 2]);
      r1 = _mm_loadu_pd(&b_y_tmp[b_i]);
      _mm_storeu_pd(&b_y_tmp[b_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&y_tmp[6 * k + 4]);
      r1 = _mm_loadu_pd(&b_y_tmp[lastc]);
      _mm_storeu_pd(&b_y_tmp[lastc], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
    for (k = 0; k < 6; k++) {
      M[k + 10 * i] = b_y_tmp[k + 6 * i];
    }
    M[10 * i + 6] = Rs[i];
    M[10 * i + 7] = Rs[i + 4];
    M[10 * i + 8] = Rs[i + 8];
    M[10 * i + 9] = Rs[i + 12];
  }
  b_st.site = &xu_emlrtRSI;
  c_st.site = &yu_emlrtRSI;
  d_st.site = &av_emlrtRSI;
  e_st.site = &bv_emlrtRSI;
  f_st.site = &ev_emlrtRSI;
  g_st.site = &fv_emlrtRSI;
  work[0] = 0.0;
  work[1] = 0.0;
  work[2] = 0.0;
  work[3] = 0.0;
  for (c_i = 0; c_i < 4; c_i++) {
    real_T atmp;
    real_T xnorm;
    ii = c_i * 10 + c_i;
    atmp = M[ii];
    h_st.site = &hv_emlrtRSI;
    itau = ii + 2;
    tau[c_i] = 0.0;
    i_st.site = &sc_emlrtRSI;
    xnorm = h_xnrm2(&i_st, 9 - c_i, M, ii + 2);
    if (xnorm != 0.0) {
      real_T beta1;
      real_T d;
      d = M[ii];
      beta1 = muDoubleScalarHypot(d, xnorm);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        lastc = (ii - c_i) + 10;
        lastv = (((((lastc - ii) - 1) / 2) << 1) + ii) + 2;
        b_i = lastv - 2;
        do {
          knt++;
          for (k = itau; k <= b_i; k += 2) {
            r = _mm_loadu_pd(&M[k - 1]);
            _mm_storeu_pd(&M[k - 1],
                          _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
          }
          for (k = lastv; k <= lastc; k++) {
            M[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                 (knt < 20));
        i_st.site = &uc_emlrtRSI;
        xnorm = h_xnrm2(&i_st, 9 - c_i, M, ii + 2);
        beta1 = muDoubleScalarHypot(atmp, xnorm);
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        tau[c_i] = (beta1 - atmp) / beta1;
        xnorm = 1.0 / (atmp - beta1);
        b_i = lastv - 2;
        for (i = itau; i <= b_i; i += 2) {
          r = _mm_loadu_pd(&M[i - 1]);
          _mm_storeu_pd(&M[i - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (i = lastv; i <= lastc; i++) {
          M[i - 1] *= xnorm;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        tau[c_i] = (beta1 - d) / beta1;
        xnorm = 1.0 / (d - beta1);
        b_i = (ii - c_i) + 10;
        lastc = (((((b_i - ii) - 1) / 2) << 1) + ii) + 2;
        lastv = lastc - 2;
        for (i = itau; i <= lastv; i += 2) {
          r = _mm_loadu_pd(&M[i - 1]);
          _mm_storeu_pd(&M[i - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (i = lastc; i <= b_i; i++) {
          M[i - 1] *= xnorm;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (c_i + 1 < 4) {
      M[ii] = 1.0;
      h_st.site = &gv_emlrtRSI;
      if (tau[c_i] != 0.0) {
        lastv = 10 - c_i;
        b_i = (ii - c_i) + 9;
        while ((lastv > 0) && (M[b_i] == 0.0)) {
          lastv--;
          b_i--;
        }
        i_st.site = &gf_emlrtRSI;
        lastc = b_ilazlc(&i_st, lastv, 3 - c_i, M, ii + 11);
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        i_st.site = &hf_emlrtRSI;
        c_xgemv(&i_st, lastv, lastc, M, ii + 11, M, ii + 1, work);
        i_st.site = &if_emlrtRSI;
        c_xgerc(&i_st, lastv, lastc, -tau[c_i], ii + 1, work, M, ii + 11);
      }
      M[ii] = atmp;
    }
  }
  for (i = 0; i < 4; i++) {
    e_st.site = &cv_emlrtRSI;
    for (k = 0; k <= i; k++) {
      R[k + (i << 2)] = M[k + 10 * i];
    }
    b_i = i + 2;
    if (b_i <= 4) {
      memset(&R[(i * 4 + b_i) + -1], 0, (uint32_T)(-b_i + 5) * sizeof(real_T));
    }
  }
  e_st.site = &dv_emlrtRSI;
  f_st.site = &iv_emlrtRSI;
  g_st.site = &jv_emlrtRSI;
  itau = 3;
  work[0] = 0.0;
  work[1] = 0.0;
  work[2] = 0.0;
  work[3] = 0.0;
  for (i = 3; i >= 0; i--) {
    ii = i + i * 10;
    if (i + 1 < 4) {
      M[ii] = 1.0;
      h_st.site = &mv_emlrtRSI;
      if (tau[itau] != 0.0) {
        lastc = 10 - i;
        b_i = (ii - i) + 9;
        while ((lastc > 0) && (M[b_i] == 0.0)) {
          lastc--;
          b_i--;
        }
        i_st.site = &gf_emlrtRSI;
        b_i = b_ilazlc(&i_st, lastc, 3 - i, M, ii + 11);
      } else {
        lastc = 0;
        b_i = 0;
      }
      if (lastc > 0) {
        i_st.site = &hf_emlrtRSI;
        c_xgemv(&i_st, lastc, b_i, M, ii + 11, M, ii + 1, work);
        i_st.site = &if_emlrtRSI;
        c_xgerc(&i_st, lastc, b_i, -tau[itau], ii + 1, work, M, ii + 11);
      }
    }
    b_i = ii + 2;
    h_st.site = &lv_emlrtRSI;
    lastc = (ii - i) + 10;
    lastv = (((((lastc - ii) - 1) / 2) << 1) + ii) + 2;
    knt = lastv - 2;
    for (k = b_i; k <= knt; k += 2) {
      r = _mm_loadu_pd(&M[k - 1]);
      _mm_storeu_pd(&M[k - 1], _mm_mul_pd(_mm_set1_pd(-tau[itau]), r));
    }
    for (k = lastv; k <= lastc; k++) {
      M[k - 1] *= -tau[itau];
    }
    M[ii] = 1.0 - tau[itau];
    h_st.site = &kv_emlrtRSI;
    for (k = 0; k < i; k++) {
      M[(ii - k) - 1] = 0.0;
    }
    itau = i - 1;
  }
  for (i = 0; i < 4; i++) {
    b_i = i << 2;
    Sy[b_i] = R[i];
    Sy[b_i + 1] = R[i + 4];
    Sy[b_i + 2] = R[i + 8];
    Sy[b_i + 3] = R[i + 12];
  }
}

/* End of code generation (EKFCorrectorAdditive.c) */
