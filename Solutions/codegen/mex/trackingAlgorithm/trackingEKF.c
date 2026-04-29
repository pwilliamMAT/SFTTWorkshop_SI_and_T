/*
 * trackingEKF.c
 *
 * Code generation for function 'trackingEKF'
 *
 */

/* Include files */
#include "trackingEKF.h"
#include "EKFCorrectorAdditive.h"
#include "ExtendedKalmanFilter.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "trisolve.h"
#include "warning.h"
#include "wrapResidual.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fx_emlrtRSI =
    {
        349,                    /* lineNo */
        "trackingEKF/residual", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo gx_emlrtRSI = {
    720,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo hx_emlrtRSI = {
    730,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ix_emlrtRSI = {
    735,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo jx_emlrtRSI = {
    733,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo kx_emlrtRSI = {
    14,                         /* lineNo */
    "validateInputSizeAndType", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\validateInputSiz"
    "eAndType.m" /* pathName */
};

static emlrtRSInfo oy_emlrtRSI = {
    128,                     /* lineNo */
    "EKFCorrector/residual", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo py_emlrtRSI = {
    129,                     /* lineNo */
    "EKFCorrector/residual", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo blb_emlrtRSI =
    {
        188,                   /* lineNo */
        "trackingEKF/correct", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo clb_emlrtRSI =
    {
        191,                   /* lineNo */
        "trackingEKF/correct", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo dlb_emlrtRSI = {
    660,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo elb_emlrtRSI = {
    670,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo flb_emlrtRSI = {
    676,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo glb_emlrtRSI = {
    673,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo hlb_emlrtRSI = {
    93,                     /* lineNo */
    "EKFCorrector/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo ilb_emlrtRSI = {
    94,                     /* lineNo */
    "EKFCorrector/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo jlb_emlrtRSI = {
    101,                    /* lineNo */
    "EKFCorrector/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo klb_emlrtRSI = {
    44,                                           /* lineNo */
    "EKFCorrector/correctStateAndSqrtCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo llb_emlrtRSI = {
    58,                                           /* lineNo */
    "EKFCorrector/correctStateAndSqrtCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrector.m" /* pathName */
};

static emlrtRSInfo mlb_emlrtRSI = {
    95,         /* lineNo */
    "linsolve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\linsolve.m" /* pathName
                                                                            */
};

static emlrtRSInfo nlb_emlrtRSI = {
    406,       /* lineNo */
    "solveLT", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\linsolve.m" /* pathName
                                                                            */
};

static emlrtRSInfo olb_emlrtRSI = {
    102,        /* lineNo */
    "linsolve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\linsolve.m" /* pathName
                                                                            */
};

static emlrtRSInfo plb_emlrtRSI = {
    444,       /* lineNo */
    "solveUT", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\linsolve.m" /* pathName
                                                                            */
};

static emlrtRSInfo qlb_emlrtRSI = {
    448,       /* lineNo */
    "solveUT", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\linsolve.m" /* pathName
                                                                            */
};

/* Function Definitions */
void trackingEKF_correct(const emlrtStack *sp, trackingEKF *filter,
                         const real_T varargin_1[4],
                         const real_T varargin_3_OriginPosition[9],
                         const real_T varargin_3_OriginVelocity[9],
                         const real_T varargin_3_Orientation[27])
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
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack l_st;
  emlrtStack st;
  real_T M[60];
  real_T b_A[36];
  real_T b_filter[36];
  real_T B[24];
  real_T K[24];
  real_T dHdx[24];
  real_T A[16];
  real_T Sy[16];
  real_T tau[6];
  real_T work[6];
  real_T residue[4];
  real_T atmp;
  real_T beta1;
  real_T d;
  real_T xnorm;
  int32_T b_i;
  int32_T c_i;
  int32_T d_i;
  int32_T i;
  int32_T iaii;
  int32_T knt;
  int32_T lastc;
  int32_T lastv;
  boolean_T exitg1;
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
  l_st.prev = &k_st;
  l_st.tls = k_st.tls;
  st.site = &blb_emlrtRSI;
  b_st.site = &dlb_emlrtRSI;
  c_st.site = &kx_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 4)) {
    if ((!muDoubleScalarIsInf(varargin_1[i])) &&
        (!muDoubleScalarIsNaN(varargin_1[i]))) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:ExtendedKalmanFilter:expectedFinite", 3, 4, 1, "z");
  }
  b_st.site = &elb_emlrtRSI;
  c_ExtendedKalmanFilter_validate(&b_st, filter, varargin_3_OriginPosition,
                                  varargin_3_OriginVelocity,
                                  varargin_3_Orientation);
  b_st.site = &flb_emlrtRSI;
  b_st.site = &glb_emlrtRSI;
  c_st.site = &hlb_emlrtRSI;
  c_EKFCorrectorAdditive_getMeasu(
      &c_st, filter->pSqrtMeasurementNoise, filter->pState,
      filter->pSqrtStateCovariance, varargin_3_OriginPosition,
      varargin_3_OriginVelocity, varargin_3_Orientation, residue, K, Sy, dHdx);
  r = _mm_loadu_pd(&residue[0]);
  _mm_storeu_pd(&residue[0], _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]), r));
  r = _mm_loadu_pd(&residue[2]);
  _mm_storeu_pd(&residue[2], _mm_sub_pd(_mm_loadu_pd(&varargin_1[2]), r));
  c_st.site = &ilb_emlrtRSI;
  wrapResidual(&c_st, residue);
  c_st.site = &jlb_emlrtRSI;
  d_st.site = &klb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    i = b_i << 2;
    B[i] = K[b_i];
    B[i + 1] = K[b_i + 6];
    B[i + 2] = K[b_i + 12];
    B[i + 3] = K[b_i + 18];
  }
  e_st.site = &mlb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    i = b_i << 2;
    K[i] = B[i];
    K[i + 1] = B[i + 1];
    K[i + 2] = B[i + 2];
    K[i + 3] = B[i + 3];
  }
  trisolve(Sy, K);
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 4)) {
    xnorm = Sy[i + (i << 2)];
    if ((xnorm == 0.0) ||
        (muDoubleScalarIsInf(xnorm) || muDoubleScalarIsNaN(xnorm))) {
      f_st.site = &nlb_emlrtRSI;
      if (!emlrtSetWarningFlag(&f_st)) {
        g_st.site = &lab_emlrtRSI;
        c_warning(&g_st);
      }
      exitg1 = true;
    } else {
      i++;
    }
  }
  d_st.site = &klb_emlrtRSI;
  for (b_i = 0; b_i < 4; b_i++) {
    i = b_i << 2;
    A[i] = Sy[b_i];
    A[i + 1] = Sy[b_i + 4];
    A[i + 2] = Sy[b_i + 8];
    A[i + 3] = Sy[b_i + 12];
  }
  e_st.site = &olb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    i = b_i << 2;
    B[i] = K[i];
    B[i + 1] = K[i + 1];
    B[i + 2] = K[i + 2];
    B[i + 3] = K[i + 3];
  }
  f_st.site = &plb_emlrtRSI;
  b_trisolve(A, B);
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 4)) {
    xnorm = A[i + (i << 2)];
    if ((xnorm == 0.0) ||
        (muDoubleScalarIsInf(xnorm) || muDoubleScalarIsNaN(xnorm))) {
      f_st.site = &qlb_emlrtRSI;
      if (!emlrtSetWarningFlag(&f_st)) {
        g_st.site = &lab_emlrtRSI;
        c_warning(&g_st);
      }
      exitg1 = true;
    } else {
      i++;
    }
  }
  for (b_i = 0; b_i < 4; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      K[c_i + 6 * b_i] = B[b_i + (c_i << 2)];
    }
  }
  for (b_i = 0; b_i <= 22; b_i += 2) {
    r = _mm_loadu_pd(&K[b_i]);
    _mm_storeu_pd(&B[b_i], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  }
  memset(&b_A[0], 0, 36U * sizeof(real_T));
  for (b_i = 0; b_i < 6; b_i++) {
    i = 6 * b_i + 2;
    lastc = 6 * b_i + 4;
    for (d_i = 0; d_i < 4; d_i++) {
      r = _mm_loadu_pd(&B[6 * d_i]);
      r1 = _mm_loadu_pd(&b_A[6 * b_i]);
      r2 = _mm_set1_pd(dHdx[d_i + (b_i << 2)]);
      _mm_storeu_pd(&b_A[6 * b_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&B[6 * d_i + 2]);
      r1 = _mm_loadu_pd(&b_A[i]);
      _mm_storeu_pd(&b_A[i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&B[6 * d_i + 4]);
      r1 = _mm_loadu_pd(&b_A[lastc]);
      _mm_storeu_pd(&b_A[lastc], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    i = b_i + 6 * b_i;
    b_A[i]++;
  }
  d_st.site = &llb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      xnorm = 0.0;
      for (d_i = 0; d_i < 6; d_i++) {
        xnorm +=
            filter->pSqrtStateCovariance[d_i + 6 * c_i] * b_A[b_i + 6 * d_i];
      }
      b_filter[c_i + 6 * b_i] = xnorm;
    }
  }
  memset(&B[0], 0, 24U * sizeof(real_T));
  for (b_i = 0; b_i < 4; b_i++) {
    i = 6 * b_i + 2;
    lastc = 6 * b_i + 4;
    for (c_i = 0; c_i < 4; c_i++) {
      r = _mm_loadu_pd(&K[6 * c_i]);
      r1 = _mm_loadu_pd(&B[6 * b_i]);
      r2 = _mm_set1_pd(filter->pSqrtMeasurementNoise[c_i + (b_i << 2)]);
      _mm_storeu_pd(&B[6 * b_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&K[6 * c_i + 2]);
      r1 = _mm_loadu_pd(&B[i]);
      _mm_storeu_pd(&B[i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&K[6 * c_i + 4]);
      r1 = _mm_loadu_pd(&B[lastc]);
      _mm_storeu_pd(&B[lastc], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      M[c_i + 10 * b_i] = b_filter[c_i + 6 * b_i];
    }
    M[10 * b_i + 6] = B[b_i];
    M[10 * b_i + 7] = B[b_i + 6];
    M[10 * b_i + 8] = B[b_i + 12];
    M[10 * b_i + 9] = B[b_i + 18];
  }
  e_st.site = &xu_emlrtRSI;
  f_st.site = &yu_emlrtRSI;
  g_st.site = &av_emlrtRSI;
  h_st.site = &bv_emlrtRSI;
  i_st.site = &ev_emlrtRSI;
  j_st.site = &fv_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    work[b_i] = 0.0;
  }
  for (d_i = 0; d_i < 6; d_i++) {
    int32_T ii;
    ii = d_i * 10 + d_i;
    atmp = M[ii];
    k_st.site = &hv_emlrtRSI;
    iaii = ii + 2;
    tau[d_i] = 0.0;
    l_st.site = &sc_emlrtRSI;
    xnorm = i_xnrm2(&l_st, 9 - d_i, M, ii + 2);
    if (xnorm != 0.0) {
      d = M[ii];
      beta1 = muDoubleScalarHypot(d, xnorm);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        lastc = (ii - d_i) + 10;
        lastv = (((((lastc - ii) - 1) / 2) << 1) + ii) + 2;
        i = lastv - 2;
        do {
          knt++;
          for (b_i = iaii; b_i <= i; b_i += 2) {
            r = _mm_loadu_pd(&M[b_i - 1]);
            _mm_storeu_pd(&M[b_i - 1],
                          _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
          }
          for (b_i = lastv; b_i <= lastc; b_i++) {
            M[b_i - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                 (knt < 20));
        l_st.site = &uc_emlrtRSI;
        xnorm = i_xnrm2(&l_st, 9 - d_i, M, ii + 2);
        beta1 = muDoubleScalarHypot(atmp, xnorm);
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        tau[d_i] = (beta1 - atmp) / beta1;
        xnorm = 1.0 / (atmp - beta1);
        i = lastv - 2;
        for (b_i = iaii; b_i <= i; b_i += 2) {
          r = _mm_loadu_pd(&M[b_i - 1]);
          _mm_storeu_pd(&M[b_i - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (b_i = lastv; b_i <= lastc; b_i++) {
          M[b_i - 1] *= xnorm;
        }
        for (b_i = 0; b_i < knt; b_i++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        tau[d_i] = (beta1 - d) / beta1;
        xnorm = 1.0 / (d - beta1);
        i = (ii - d_i) + 10;
        lastc = (((((i - ii) - 1) / 2) << 1) + ii) + 2;
        lastv = lastc - 2;
        for (b_i = iaii; b_i <= lastv; b_i += 2) {
          r = _mm_loadu_pd(&M[b_i - 1]);
          _mm_storeu_pd(&M[b_i - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (b_i = lastc; b_i <= i; b_i++) {
          M[b_i - 1] *= xnorm;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (d_i + 1 < 6) {
      M[ii] = 1.0;
      k_st.site = &gv_emlrtRSI;
      if (tau[d_i] != 0.0) {
        lastv = 10 - d_i;
        i = (ii - d_i) + 9;
        while ((lastv > 0) && (M[i] == 0.0)) {
          lastv--;
          i--;
        }
        l_st.site = &gf_emlrtRSI;
        lastc = c_ilazlc(&l_st, lastv, 5 - d_i, M, ii + 11);
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        l_st.site = &hf_emlrtRSI;
        d_xgemv(&l_st, lastv, lastc, M, ii + 11, M, ii + 1, work);
        l_st.site = &if_emlrtRSI;
        d_xgerc(&l_st, lastv, lastc, -tau[d_i], ii + 1, work, M, ii + 11);
      }
      M[ii] = atmp;
    }
  }
  for (c_i = 0; c_i < 6; c_i++) {
    h_st.site = &cv_emlrtRSI;
    for (b_i = 0; b_i <= c_i; b_i++) {
      b_A[b_i + 6 * c_i] = M[b_i + 10 * c_i];
    }
    i = c_i + 2;
    if (i <= 6) {
      memset(&b_A[(c_i * 6 + i) + -1], 0, (uint32_T)(-i + 7) * sizeof(real_T));
    }
  }
  h_st.site = &dv_emlrtRSI;
  i_st.site = &iv_emlrtRSI;
  j_st.site = &jv_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    work[b_i] = 0.0;
  }
  for (c_i = 5; c_i >= 0; c_i--) {
    iaii = c_i + c_i * 10;
    if (c_i + 1 < 6) {
      M[iaii] = 1.0;
      k_st.site = &mv_emlrtRSI;
      if (tau[c_i] != 0.0) {
        lastc = 10 - c_i;
        i = (iaii - c_i) + 9;
        while ((lastc > 0) && (M[i] == 0.0)) {
          lastc--;
          i--;
        }
        l_st.site = &gf_emlrtRSI;
        i = c_ilazlc(&l_st, lastc, 5 - c_i, M, iaii + 11);
      } else {
        lastc = 0;
        i = 0;
      }
      if (lastc > 0) {
        l_st.site = &hf_emlrtRSI;
        d_xgemv(&l_st, lastc, i, M, iaii + 11, M, iaii + 1, work);
        l_st.site = &if_emlrtRSI;
        d_xgerc(&l_st, lastc, i, -tau[c_i], iaii + 1, work, M, iaii + 11);
      }
    }
    i = iaii + 2;
    k_st.site = &lv_emlrtRSI;
    lastc = (iaii - c_i) + 10;
    lastv = (((((lastc - iaii) - 1) / 2) << 1) + iaii) + 2;
    knt = lastv - 2;
    for (b_i = i; b_i <= knt; b_i += 2) {
      r = _mm_loadu_pd(&M[b_i - 1]);
      _mm_storeu_pd(&M[b_i - 1], _mm_mul_pd(_mm_set1_pd(-tau[c_i]), r));
    }
    for (b_i = lastv; b_i <= lastc; b_i++) {
      M[b_i - 1] *= -tau[c_i];
    }
    M[iaii] = 1.0 - tau[c_i];
    k_st.site = &kv_emlrtRSI;
    for (b_i = 0; b_i < c_i; b_i++) {
      M[(iaii - b_i) - 1] = 0.0;
    }
  }
  xnorm = residue[0];
  d = residue[1];
  atmp = residue[2];
  beta1 = residue[3];
  for (b_i = 0; b_i <= 4; b_i += 2) {
    r = _mm_loadu_pd(&K[b_i]);
    r1 = _mm_mul_pd(r, _mm_set1_pd(xnorm));
    r = _mm_loadu_pd(&K[b_i + 6]);
    r = _mm_mul_pd(r, _mm_set1_pd(d));
    r1 = _mm_add_pd(r1, r);
    r = _mm_loadu_pd(&K[b_i + 12]);
    r = _mm_mul_pd(r, _mm_set1_pd(atmp));
    r1 = _mm_add_pd(r1, r);
    r = _mm_loadu_pd(&K[b_i + 18]);
    r = _mm_mul_pd(r, _mm_set1_pd(beta1));
    r = _mm_add_pd(r1, r);
    r1 = _mm_loadu_pd(&filter->pState[b_i]);
    r = _mm_add_pd(r1, r);
    _mm_storeu_pd(&filter->pState[b_i], r);
  }
  b_st.site = &glb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      filter->pSqrtStateCovariance[c_i + 6 * b_i] = b_A[b_i + 6 * c_i];
    }
  }
  filter->pIsSetStateCovariance = true;
  filter->pSqrtStateCovarianceScalar = -1.0;
  st.site = &clb_emlrtRSI;
  if (!filter->pIsInitialized) {
    filter->pIsDistributionsSetup = true;
  }
}

void trackingEKF_residual(const emlrtStack *sp, trackingEKF *EKF,
                          const real_T z[4],
                          const real_T c_measurementParams_f2_OriginPo[9],
                          const real_T c_measurementParams_f2_OriginVe[9],
                          const real_T c_measurementParams_f2_Orientat[27],
                          real_T res[4], real_T S[16])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T a__1[24];
  real_T a__2[24];
  real_T Sy[16];
  int32_T i;
  int32_T i1;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &fx_emlrtRSI;
  b_st.site = &gx_emlrtRSI;
  c_st.site = &kx_emlrtRSI;
  d_st.site = &vd_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if ((!muDoubleScalarIsInf(z[k])) && (!muDoubleScalarIsNaN(z[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:residual:expectedFinite", 3, 4, 1, "z");
  }
  b_st.site = &hx_emlrtRSI;
  c_ExtendedKalmanFilter_validate(&b_st, EKF, c_measurementParams_f2_OriginPo,
                                  c_measurementParams_f2_OriginVe,
                                  c_measurementParams_f2_Orientat);
  b_st.site = &ix_emlrtRSI;
  b_st.site = &jx_emlrtRSI;
  c_st.site = &oy_emlrtRSI;
  c_EKFCorrectorAdditive_getMeasu(
      &c_st, EKF->pSqrtMeasurementNoise, EKF->pState, EKF->pSqrtStateCovariance,
      c_measurementParams_f2_OriginPo, c_measurementParams_f2_OriginVe,
      c_measurementParams_f2_Orientat, res, a__1, Sy, a__2);
  r = _mm_loadu_pd(&res[0]);
  _mm_storeu_pd(&res[0], _mm_sub_pd(_mm_loadu_pd(&z[0]), r));
  r = _mm_loadu_pd(&res[2]);
  _mm_storeu_pd(&res[2], _mm_sub_pd(_mm_loadu_pd(&z[2]), r));
  c_st.site = &py_emlrtRSI;
  wrapResidual(&c_st, res);
  memset(&S[0], 0, sizeof(real_T) << 4);
  for (i = 0; i < 4; i++) {
    k = i << 2;
    for (i1 = 0; i1 < 4; i1++) {
      __m128d r1;
      __m128d r2;
      int32_T i2;
      i2 = i1 << 2;
      r = _mm_loadu_pd(&Sy[i2]);
      r1 = _mm_loadu_pd(&S[k]);
      r2 = _mm_set1_pd(Sy[i + i2]);
      _mm_storeu_pd(&S[k], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&Sy[i2 + 2]);
      r1 = _mm_loadu_pd(&S[k + 2]);
      _mm_storeu_pd(&S[k + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
}

/* End of code generation (trackingEKF.c) */
