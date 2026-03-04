/*
 * trackingEKF.c
 *
 * Code generation for function 'trackingEKF'
 *
 */

/* Include files */
#include "trackingEKF.h"
#include "ConstantVelocityModel.h"
#include "EKFCorrectorAdditive.h"
#include "ExtendedKalmanFilter.h"
#include "mrdivide_helper.h"
#include "predictStateNonAdditive.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "trisolve.h"
#include "warning.h"
#include "wrapResidual.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "xzgetrf.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo pc_emlrtRSI = {
    142,                                                      /* lineNo */
    "trackingEKF/trackingEKF",                                /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo qc_emlrtRSI = {
    143,                                                      /* lineNo */
    "trackingEKF/trackingEKF",                                /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo rc_emlrtRSI = {
    135,                                                      /* lineNo */
    "trackingEKF/trackingEKF",                                /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo sc_emlrtRSI = {
    144,                                                      /* lineNo */
    "trackingEKF/trackingEKF",                                /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo yc_emlrtRSI = {
    535,                                         /* lineNo */
    "ExtendedKalmanFilter/ExtendedKalmanFilter", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo gk_emlrtRSI = {
    1639,                                                          /* lineNo */
    "ExtendedKalmanFilter/stateCovarianceScalarExpandIfNecessary", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo hu_emlrtRSI = {
    168,                                                      /* lineNo */
    "trackingEKF/predict",                                    /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo iu_emlrtRSI = {
    171,                                                      /* lineNo */
    "trackingEKF/predict",                                    /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo ju_emlrtRSI = {
    154,                                          /* lineNo */
    "AbstractSmoother/setupInitialDistributions", /* fcnName */
    "/MATLAB/toolbox/shared/smoothers/+matlabshared/+smoothers/+internal/"
    "AbstractSmoother.m" /* pathName */
};

static emlrtRSInfo ku_emlrtRSI = {
    42,                                          /* lineNo */
    "LinearizedSmoother/ensureMethodDefinition", /* fcnName */
    "/MATLAB/toolbox/shared/smoothers/+matlabshared/+smoothers/+internal/"
    "LinearizedSmoother.m" /* pathName */
};

static emlrtRSInfo lu_emlrtRSI = {
    31,                                               /* lineNo */
    "LinearizedSmoother/ensureLastJacobianIsDefined", /* fcnName */
    "/MATLAB/toolbox/shared/smoothers/+matlabshared/+smoothers/+internal/"
    "LinearizedSmoother.m" /* pathName */
};

static emlrtRSInfo mu_emlrtRSI = {
    220,                                      /* lineNo */
    "GaussianSmoother/getStateAndCovariance", /* fcnName */
    "/MATLAB/toolbox/shared/smoothers/+matlabshared/+smoothers/+internal/"
    "GaussianSmoother.m" /* pathName */
};

static emlrtRSInfo nu_emlrtRSI = {
    611,                            /* lineNo */
    "ExtendedKalmanFilter/predict", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo ou_emlrtRSI = {
    614,                            /* lineNo */
    "ExtendedKalmanFilter/predict", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo pu_emlrtRSI = {
    1593,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo qu_emlrtRSI = {
    1594,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo ru_emlrtRSI = {
    1618,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo su_emlrtRSI = {
    1628,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo uu_emlrtRSI = {
    1920,                                               /* lineNo */
    "ExtendedKalmanFilter/ensureProcessNoiseIsDefined", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo vu_emlrtRSI = {
    1652,                                                       /* lineNo */
    "ExtendedKalmanFilter/processNoiseScalarExpandIfNecessary", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo wu_emlrtRSI = {
    13,                                                   /* lineNo */
    "EKFPredictorNonAdditive/validateStateTransitionFcn", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFPredictorNonAdditive.m" /* pathName */
};

static emlrtRSInfo dv_emlrtRSI = {
    1941,                                                      /* lineNo */
    "ExtendedKalmanFilter/validateStateTransitionJacobianFcn", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo ev_emlrtRSI = {
    1943,                                                      /* lineNo */
    "ExtendedKalmanFilter/validateStateTransitionJacobianFcn", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo fv_emlrtRSI = {
    18,                                                           /* lineNo */
    "EKFPredictorNonAdditive/validateStateTransitionJacobianFcn", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFPredictorNonAdditive.m" /* pathName */
};

static emlrtRSInfo gv_emlrtRSI = {
    13,                                /* lineNo */
    "predictStateNonAdditiveJacobian", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "predictStateNonAdditiveJacobian.m" /* pathName */
};

static emlrtRSInfo lv_emlrtRSI = {
    64,                                /* lineNo */
    "EKFPredictorNonAdditive/predict", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFPredictorNonAdditive.m" /* pathName */
};

static emlrtRSInfo mv_emlrtRSI = {
    66,                                /* lineNo */
    "EKFPredictorNonAdditive/predict", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFPredictorNonAdditive.m" /* pathName */
};

static emlrtRSInfo nv_emlrtRSI = {
    67,                                /* lineNo */
    "EKFPredictorNonAdditive/predict", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFPredictorNonAdditive.m" /* pathName */
};

static emlrtRSInfo ov_emlrtRSI = {
    102,                                          /* lineNo */
    "EKFPredictorNonAdditive/predictionMatrices", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFPredictorNonAdditive.m" /* pathName */
};

static emlrtRSInfo uy_emlrtRSI = {
    349,                                                      /* lineNo */
    "trackingEKF/residual",                                   /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo vy_emlrtRSI = {
    720,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo wy_emlrtRSI = {
    730,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo xy_emlrtRSI = {
    735,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo yy_emlrtRSI = {
    733,                             /* lineNo */
    "ExtendedKalmanFilter/residual", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo aab_emlrtRSI = {
    14,                         /* lineNo */
    "validateInputSizeAndType", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/validateInputSizeAndType.m" /* pathName */
};

static emlrtRSInfo ebb_emlrtRSI = {
    128,                     /* lineNo */
    "EKFCorrector/residual", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo fbb_emlrtRSI = {
    129,                     /* lineNo */
    "EKFCorrector/residual", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo mcb_emlrtRSI = {
    395,                                                      /* lineNo */
    "trackingEKF/likelihood",                                 /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo ncb_emlrtRSI = {
    396,                                                      /* lineNo */
    "trackingEKF/likelihood",                                 /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo ocb_emlrtRSI = {
    19,                 /* lineNo */
    "KalmanLikelihood", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/KalmanLikelihood.m" /* pathName */
};

static emlrtRSInfo pcb_emlrtRSI = {
    21,                 /* lineNo */
    "KalmanLikelihood", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/KalmanLikelihood.m" /* pathName */
};

static emlrtRSInfo qcb_emlrtRSI = {
    21,                                           /* lineNo */
    "det",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/det.m" /* pathName */
};

static emlrtRSInfo kmb_emlrtRSI = {
    188,                                                      /* lineNo */
    "trackingEKF/correct",                                    /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo lmb_emlrtRSI = {
    191,                                                      /* lineNo */
    "trackingEKF/correct",                                    /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/trackingEKF.m" /* pathName */
};

static emlrtRSInfo mmb_emlrtRSI = {
    660,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo nmb_emlrtRSI = {
    670,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo omb_emlrtRSI = {
    676,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo pmb_emlrtRSI = {
    673,                            /* lineNo */
    "ExtendedKalmanFilter/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

static emlrtRSInfo qmb_emlrtRSI = {
    93,                     /* lineNo */
    "EKFCorrector/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo rmb_emlrtRSI = {
    94,                     /* lineNo */
    "EKFCorrector/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo smb_emlrtRSI = {
    101,                    /* lineNo */
    "EKFCorrector/correct", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo tmb_emlrtRSI = {
    44,                                           /* lineNo */
    "EKFCorrector/correctStateAndSqrtCovariance", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo umb_emlrtRSI = {
    58,                                           /* lineNo */
    "EKFCorrector/correctStateAndSqrtCovariance", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/EKFCorrector.m" /* pathName */
};

static emlrtRSInfo vmb_emlrtRSI = {
    95,                                                /* lineNo */
    "linsolve",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/linsolve.m" /* pathName */
};

static emlrtRSInfo wmb_emlrtRSI = {
    406,                                               /* lineNo */
    "solveLT",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/linsolve.m" /* pathName */
};

static emlrtRSInfo xmb_emlrtRSI = {
    102,                                               /* lineNo */
    "linsolve",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/linsolve.m" /* pathName */
};

static emlrtRSInfo ymb_emlrtRSI = {
    444,                                               /* lineNo */
    "solveUT",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/linsolve.m" /* pathName */
};

static emlrtRSInfo anb_emlrtRSI = {
    448,                                               /* lineNo */
    "solveUT",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/linsolve.m" /* pathName */
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
  st.site = &kmb_emlrtRSI;
  b_st.site = &mmb_emlrtRSI;
  c_st.site = &aab_emlrtRSI;
  d_st.site = &ge_emlrtRSI;
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
  b_st.site = &nmb_emlrtRSI;
  c_ExtendedKalmanFilter_validate(&b_st, filter, varargin_3_OriginPosition,
                                  varargin_3_OriginVelocity,
                                  varargin_3_Orientation);
  b_st.site = &omb_emlrtRSI;
  b_st.site = &pmb_emlrtRSI;
  c_st.site = &qmb_emlrtRSI;
  c_EKFCorrectorAdditive_getMeasu(
      &c_st, filter->pSqrtMeasurementNoise, filter->pState,
      filter->pSqrtStateCovariance, varargin_3_OriginPosition,
      varargin_3_OriginVelocity, varargin_3_Orientation, residue, K, Sy, dHdx);
  r = _mm_loadu_pd(&residue[0]);
  _mm_storeu_pd(&residue[0], _mm_sub_pd(_mm_loadu_pd(&varargin_1[0]), r));
  r = _mm_loadu_pd(&residue[2]);
  _mm_storeu_pd(&residue[2], _mm_sub_pd(_mm_loadu_pd(&varargin_1[2]), r));
  c_st.site = &rmb_emlrtRSI;
  wrapResidual(&c_st, residue);
  c_st.site = &smb_emlrtRSI;
  d_st.site = &tmb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    i = b_i << 2;
    B[i] = K[b_i];
    B[i + 1] = K[b_i + 6];
    B[i + 2] = K[b_i + 12];
    B[i + 3] = K[b_i + 18];
  }
  e_st.site = &vmb_emlrtRSI;
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
      f_st.site = &wmb_emlrtRSI;
      if (!emlrtSetWarningFlag(&f_st)) {
        g_st.site = &bcb_emlrtRSI;
        c_warning(&g_st);
      }
      exitg1 = true;
    } else {
      i++;
    }
  }
  d_st.site = &tmb_emlrtRSI;
  for (b_i = 0; b_i < 4; b_i++) {
    i = b_i << 2;
    A[i] = Sy[b_i];
    A[i + 1] = Sy[b_i + 4];
    A[i + 2] = Sy[b_i + 8];
    A[i + 3] = Sy[b_i + 12];
  }
  e_st.site = &xmb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    i = b_i << 2;
    B[i] = K[i];
    B[i + 1] = K[i + 1];
    B[i + 2] = K[i + 2];
    B[i + 3] = K[i + 3];
  }
  f_st.site = &ymb_emlrtRSI;
  b_trisolve(A, B);
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 4)) {
    xnorm = A[i + (i << 2)];
    if ((xnorm == 0.0) ||
        (muDoubleScalarIsInf(xnorm) || muDoubleScalarIsNaN(xnorm))) {
      f_st.site = &anb_emlrtRSI;
      if (!emlrtSetWarningFlag(&f_st)) {
        g_st.site = &bcb_emlrtRSI;
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
  d_st.site = &umb_emlrtRSI;
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
  e_st.site = &pv_emlrtRSI;
  f_st.site = &qv_emlrtRSI;
  g_st.site = &rv_emlrtRSI;
  h_st.site = &sv_emlrtRSI;
  i_st.site = &vv_emlrtRSI;
  j_st.site = &wv_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    work[b_i] = 0.0;
  }
  for (d_i = 0; d_i < 6; d_i++) {
    int32_T ii;
    ii = d_i * 10 + d_i;
    atmp = M[ii];
    k_st.site = &yv_emlrtRSI;
    iaii = ii + 2;
    tau[d_i] = 0.0;
    l_st.site = &cd_emlrtRSI;
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
        l_st.site = &ed_emlrtRSI;
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
      k_st.site = &xv_emlrtRSI;
      if (tau[d_i] != 0.0) {
        lastv = 10 - d_i;
        i = (ii - d_i) + 9;
        while ((lastv > 0) && (M[i] == 0.0)) {
          lastv--;
          i--;
        }
        l_st.site = &qf_emlrtRSI;
        lastc = c_ilazlc(&l_st, lastv, 5 - d_i, M, ii + 11);
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        l_st.site = &rf_emlrtRSI;
        d_xgemv(&l_st, lastv, lastc, M, ii + 11, M, ii + 1, work);
        l_st.site = &sf_emlrtRSI;
        d_xgerc(&l_st, lastv, lastc, -tau[d_i], ii + 1, work, M, ii + 11);
      }
      M[ii] = atmp;
    }
  }
  for (c_i = 0; c_i < 6; c_i++) {
    h_st.site = &tv_emlrtRSI;
    for (b_i = 0; b_i <= c_i; b_i++) {
      b_A[b_i + 6 * c_i] = M[b_i + 10 * c_i];
    }
    i = c_i + 2;
    if (i <= 6) {
      memset(&b_A[(c_i * 6 + i) + -1], 0, (uint32_T)(-i + 7) * sizeof(real_T));
    }
  }
  h_st.site = &uv_emlrtRSI;
  i_st.site = &aw_emlrtRSI;
  j_st.site = &bw_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    work[b_i] = 0.0;
  }
  for (c_i = 5; c_i >= 0; c_i--) {
    iaii = c_i + c_i * 10;
    if (c_i + 1 < 6) {
      M[iaii] = 1.0;
      k_st.site = &ew_emlrtRSI;
      if (tau[c_i] != 0.0) {
        lastc = 10 - c_i;
        i = (iaii - c_i) + 9;
        while ((lastc > 0) && (M[i] == 0.0)) {
          lastc--;
          i--;
        }
        l_st.site = &qf_emlrtRSI;
        i = c_ilazlc(&l_st, lastc, 5 - c_i, M, iaii + 11);
      } else {
        lastc = 0;
        i = 0;
      }
      if (lastc > 0) {
        l_st.site = &rf_emlrtRSI;
        d_xgemv(&l_st, lastc, i, M, iaii + 11, M, iaii + 1, work);
        l_st.site = &sf_emlrtRSI;
        d_xgerc(&l_st, lastc, i, -tau[c_i], iaii + 1, work, M, iaii + 11);
      }
    }
    i = iaii + 2;
    k_st.site = &dw_emlrtRSI;
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
    k_st.site = &cw_emlrtRSI;
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
  b_st.site = &pmb_emlrtRSI;
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      filter->pSqrtStateCovariance[c_i + 6 * b_i] = b_A[b_i + 6 * c_i];
    }
  }
  filter->pIsSetStateCovariance = true;
  filter->pSqrtStateCovarianceScalar = -1.0;
  st.site = &lmb_emlrtRSI;
  if (!filter->pIsInitialized) {
    filter->pIsDistributionsSetup = true;
  }
}

real_T trackingEKF_likelihood(const emlrtStack *sp, trackingEKF *EKF,
                              const real_T z[4],
                              const real_T varargin_1_f2_OriginPosition[9],
                              const real_T varargin_1_f2_OriginVelocity[9],
                              const real_T varargin_1_f2_Orientation[27])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T S[16];
  real_T Y[4];
  real_T zres[4];
  real_T l;
  int32_T ipiv[4];
  boolean_T isodd;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &mcb_emlrtRSI;
  trackingEKF_residual(&st, EKF, z, varargin_1_f2_OriginPosition,
                       varargin_1_f2_OriginVelocity, varargin_1_f2_Orientation,
                       zres, S);
  st.site = &ncb_emlrtRSI;
  b_st.site = &ocb_emlrtRSI;
  Y[0] = zres[0];
  Y[1] = zres[1];
  Y[2] = zres[2];
  Y[3] = zres[3];
  c_st.site = &mbb_emlrtRSI;
  mrdiv(&c_st, Y, S);
  b_st.site = &pcb_emlrtRSI;
  c_st.site = &qcb_emlrtRSI;
  d_st.site = &tbb_emlrtRSI;
  xzgetrf(&d_st, S, ipiv);
  isodd = (ipiv[0] > 1);
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }
  l = S[0] * S[5] * S[10] * S[15];
  if (ipiv[2] > 3) {
    isodd = !isodd;
  }
  if (isodd) {
    l = -l;
  }
  b_st.site = &pcb_emlrtRSI;
  if (l < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  return muDoubleScalarMax(
      muDoubleScalarExp(-(((Y[0] * zres[0] + Y[1] * zres[1]) + Y[2] * zres[2]) +
                          Y[3] * zres[3]) /
                        2.0) /
          39.478417604357432 / muDoubleScalarSqrt(l),
      2.2250738585072014E-308);
}

void trackingEKF_predict(const emlrtStack *sp, trackingEKF *filter,
                         real_T varargin_1)
{
  static const int8_T b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  __m128d r;
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
  real_T M[54];
  real_T b_filter[36];
  real_T B[18];
  real_T dv1[18];
  real_T dv[6];
  real_T tau[6];
  real_T work[6];
  real_T xnorm;
  int32_T b_i;
  int32_T c_i;
  int32_T d_i;
  int32_T i;
  int32_T ii;
  int32_T itau;
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
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  st.site = &hu_emlrtRSI;
  b_st.site = &ju_emlrtRSI;
  c_st.site = &ku_emlrtRSI;
  if (!filter->IsLastJacobianInitialized) {
    d_st.site = &lu_emlrtRSI;
    e_st.site = &mu_emlrtRSI;
    filter->IsLastJacobianInitialized = true;
  }
  if (!filter->pIsDistributionsSetup) {
    filter->pIsDistributionsSetup = true;
  }
  st.site = &iu_emlrtRSI;
  b_st.site = &nu_emlrtRSI;
  c_st.site = &pu_emlrtRSI;
  d_st.site = &tu_emlrtRSI;
  if ((!filter->pIsSetStateCovariance) ||
      (filter->pSqrtStateCovarianceScalar != -1.0)) {
    xnorm = filter->pSqrtStateCovarianceScalar;
    e_st.site = &gk_emlrtRSI;
    for (i = 0; i < 36; i++) {
      filter->pSqrtStateCovariance[i] = xnorm * (real_T)iv[i];
    }
  }
  c_st.site = &qu_emlrtRSI;
  d_st.site = &uu_emlrtRSI;
  if ((!filter->pIsSetProcessNoise) ||
      (filter->pSqrtProcessNoiseScalar != -1.0)) {
    xnorm = filter->pSqrtProcessNoiseScalar;
    e_st.site = &vu_emlrtRSI;
    for (i = 0; i < 9; i++) {
      filter->pSqrtProcessNoise[i] = xnorm * (real_T)b[i];
    }
    filter->pIsSetProcessNoise = true;
    filter->pSqrtProcessNoiseScalar = -1.0;
  }
  if (filter->pIsFirstCallPredict) {
    if (!filter->pIsValidStateTransitionFcn) {
      c_st.site = &ru_emlrtRSI;
      for (i = 0; i < 6; i++) {
        dv[i] = filter->pState[i];
      }
      d_st.site = &wu_emlrtRSI;
      predictStateNonAdditive(&d_st, dv, varargin_1);
      filter->pIsValidStateTransitionFcn = true;
    }
    c_st.site = &su_emlrtRSI;
    d_st.site = &dv_emlrtRSI;
    e_st.site = &fv_emlrtRSI;
    f_st.site = &gv_emlrtRSI;
    c_ConstantVelocityModel_predict(&f_st, filter->pState, varargin_1,
                                    filter->pJacobian, B);
    d_st.site = &ev_emlrtRSI;
    filter->pIsFirstCallPredict = false;
  }
  b_st.site = &ou_emlrtRSI;
  c_st.site = &lv_emlrtRSI;
  d_st.site = &ov_emlrtRSI;
  e_st.site = &gv_emlrtRSI;
  c_ConstantVelocityModel_predict(&e_st, filter->pState, varargin_1,
                                  filter->pJacobian, B);
  for (i = 0; i < 6; i++) {
    dv[i] = filter->pState[i];
  }
  c_st.site = &mv_emlrtRSI;
  predictStateNonAdditive(&c_st, dv, varargin_1);
  c_st.site = &nv_emlrtRSI;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      xnorm = 0.0;
      for (c_i = 0; c_i < 6; c_i++) {
        xnorm += filter->pSqrtStateCovariance[c_i + 6 * b_i] *
                 filter->pJacobian[i + 6 * c_i];
      }
      b_filter[b_i + 6 * i] = xnorm;
    }
  }
  memset(&dv1[0], 0, 18U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    d_i = 6 * i + 2;
    lastc = 6 * i + 4;
    for (c_i = 0; c_i < 3; c_i++) {
      __m128d r1;
      __m128d r2;
      r = _mm_loadu_pd(&B[6 * c_i]);
      r1 = _mm_loadu_pd(&dv1[6 * i]);
      r2 = _mm_set1_pd(filter->pSqrtProcessNoise[c_i + 3 * i]);
      _mm_storeu_pd(&dv1[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&B[6 * c_i + 2]);
      r1 = _mm_loadu_pd(&dv1[d_i]);
      _mm_storeu_pd(&dv1[d_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&B[6 * c_i + 4]);
      r1 = _mm_loadu_pd(&dv1[lastc]);
      _mm_storeu_pd(&dv1[lastc], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      M[b_i + 9 * i] = b_filter[b_i + 6 * i];
    }
    M[9 * i + 6] = dv1[i];
    M[9 * i + 7] = dv1[i + 6];
    M[9 * i + 8] = dv1[i + 12];
  }
  d_st.site = &pv_emlrtRSI;
  e_st.site = &qv_emlrtRSI;
  f_st.site = &rv_emlrtRSI;
  g_st.site = &sv_emlrtRSI;
  h_st.site = &vv_emlrtRSI;
  i_st.site = &wv_emlrtRSI;
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }
  for (b_i = 0; b_i < 6; b_i++) {
    real_T atmp;
    ii = b_i * 9 + b_i;
    atmp = M[ii];
    j_st.site = &yv_emlrtRSI;
    itau = ii + 2;
    tau[b_i] = 0.0;
    k_st.site = &cd_emlrtRSI;
    xnorm = e_xnrm2(&k_st, 8 - b_i, M, ii + 2);
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
        lastc = (ii - b_i) + 9;
        lastv = (((((lastc - ii) - 1) / 2) << 1) + ii) + 2;
        d_i = lastv - 2;
        do {
          knt++;
          for (i = itau; i <= d_i; i += 2) {
            r = _mm_loadu_pd(&M[i - 1]);
            _mm_storeu_pd(&M[i - 1],
                          _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
          }
          for (i = lastv; i <= lastc; i++) {
            M[i - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                 (knt < 20));
        k_st.site = &ed_emlrtRSI;
        xnorm = e_xnrm2(&k_st, 8 - b_i, M, ii + 2);
        beta1 = muDoubleScalarHypot(atmp, xnorm);
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        tau[b_i] = (beta1 - atmp) / beta1;
        xnorm = 1.0 / (atmp - beta1);
        d_i = lastv - 2;
        for (i = itau; i <= d_i; i += 2) {
          r = _mm_loadu_pd(&M[i - 1]);
          _mm_storeu_pd(&M[i - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (i = lastv; i <= lastc; i++) {
          M[i - 1] *= xnorm;
        }
        for (i = 0; i < knt; i++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        tau[b_i] = (beta1 - d) / beta1;
        xnorm = 1.0 / (d - beta1);
        d_i = (ii - b_i) + 9;
        lastc = (((((d_i - ii) - 1) / 2) << 1) + ii) + 2;
        lastv = lastc - 2;
        for (i = itau; i <= lastv; i += 2) {
          r = _mm_loadu_pd(&M[i - 1]);
          _mm_storeu_pd(&M[i - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (i = lastc; i <= d_i; i++) {
          M[i - 1] *= xnorm;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (b_i + 1 < 6) {
      M[ii] = 1.0;
      j_st.site = &xv_emlrtRSI;
      if (tau[b_i] != 0.0) {
        lastv = 9 - b_i;
        d_i = (ii - b_i) + 8;
        while ((lastv > 0) && (M[d_i] == 0.0)) {
          lastv--;
          d_i--;
        }
        k_st.site = &qf_emlrtRSI;
        lastc = ilazlc(&k_st, lastv, 5 - b_i, M, ii + 10);
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        k_st.site = &rf_emlrtRSI;
        b_xgemv(&k_st, lastv, lastc, M, ii + 10, M, ii + 1, work);
        k_st.site = &sf_emlrtRSI;
        b_xgerc(&k_st, lastv, lastc, -tau[b_i], ii + 1, work, M, ii + 10);
      }
      M[ii] = atmp;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    g_st.site = &tv_emlrtRSI;
    for (i = 0; i <= b_i; i++) {
      b_filter[i + 6 * b_i] = M[i + 9 * b_i];
    }
    d_i = b_i + 2;
    if (d_i <= 6) {
      memset(&b_filter[(b_i * 6 + d_i) + -1], 0,
             (uint32_T)(-d_i + 7) * sizeof(real_T));
    }
  }
  g_st.site = &uv_emlrtRSI;
  h_st.site = &aw_emlrtRSI;
  i_st.site = &bw_emlrtRSI;
  itau = 5;
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }
  for (c_i = 5; c_i >= 0; c_i--) {
    ii = c_i + c_i * 9;
    if (c_i + 1 < 6) {
      M[ii] = 1.0;
      j_st.site = &ew_emlrtRSI;
      if (tau[itau] != 0.0) {
        lastc = 9 - c_i;
        d_i = (ii - c_i) + 8;
        while ((lastc > 0) && (M[d_i] == 0.0)) {
          lastc--;
          d_i--;
        }
        k_st.site = &qf_emlrtRSI;
        d_i = ilazlc(&k_st, lastc, 5 - c_i, M, ii + 10);
      } else {
        lastc = 0;
        d_i = 0;
      }
      if (lastc > 0) {
        k_st.site = &rf_emlrtRSI;
        b_xgemv(&k_st, lastc, d_i, M, ii + 10, M, ii + 1, work);
        k_st.site = &sf_emlrtRSI;
        b_xgerc(&k_st, lastc, d_i, -tau[itau], ii + 1, work, M, ii + 10);
      }
    }
    d_i = ii + 2;
    j_st.site = &dw_emlrtRSI;
    lastc = (ii - c_i) + 9;
    lastv = (((((lastc - ii) - 1) / 2) << 1) + ii) + 2;
    knt = lastv - 2;
    for (i = d_i; i <= knt; i += 2) {
      r = _mm_loadu_pd(&M[i - 1]);
      _mm_storeu_pd(&M[i - 1], _mm_mul_pd(_mm_set1_pd(-tau[itau]), r));
    }
    for (i = lastv; i <= lastc; i++) {
      M[i - 1] *= -tau[itau];
    }
    M[ii] = 1.0 - tau[itau];
    j_st.site = &cw_emlrtRSI;
    for (i = 0; i < c_i; i++) {
      M[(ii - i) - 1] = 0.0;
    }
    itau = c_i - 1;
  }
  for (i = 0; i < 6; i++) {
    filter->pState[i] = dv[i];
  }
  b_st.site = &ou_emlrtRSI;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      filter->pSqrtStateCovariance[b_i + 6 * i] = b_filter[i + 6 * b_i];
    }
  }
  filter->pIsSetStateCovariance = true;
  filter->pSqrtStateCovarianceScalar = -1.0;
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
  st.site = &uy_emlrtRSI;
  b_st.site = &vy_emlrtRSI;
  c_st.site = &aab_emlrtRSI;
  d_st.site = &ge_emlrtRSI;
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
  b_st.site = &wy_emlrtRSI;
  c_ExtendedKalmanFilter_validate(&b_st, EKF, c_measurementParams_f2_OriginPo,
                                  c_measurementParams_f2_OriginVe,
                                  c_measurementParams_f2_Orientat);
  b_st.site = &xy_emlrtRSI;
  b_st.site = &yy_emlrtRSI;
  c_st.site = &ebb_emlrtRSI;
  c_EKFCorrectorAdditive_getMeasu(
      &c_st, EKF->pSqrtMeasurementNoise, EKF->pState, EKF->pSqrtStateCovariance,
      c_measurementParams_f2_OriginPo, c_measurementParams_f2_OriginVe,
      c_measurementParams_f2_Orientat, res, a__1, Sy, a__2);
  r = _mm_loadu_pd(&res[0]);
  _mm_storeu_pd(&res[0], _mm_sub_pd(_mm_loadu_pd(&z[0]), r));
  r = _mm_loadu_pd(&res[2]);
  _mm_storeu_pd(&res[2], _mm_sub_pd(_mm_loadu_pd(&z[2]), r));
  c_st.site = &fbb_emlrtRSI;
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

trackingEKF *trackingEKF_trackingEKF(const emlrtStack *sp, trackingEKF *EKF,
                                     const real_T varargin_22[9])
{
  emlrtStack b_st;
  emlrtStack st;
  trackingEKF *b_EKF;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_EKF = EKF;
  st.site = &pc_emlrtRSI;
  b_EKF->pIsFirstCallPredict = true;
  b_EKF->pIsFirstCallCorrect = true;
  for (i = 0; i < 6; i++) {
    b_EKF->pState[i] = 0.0;
  }
  b_EKF->pSqrtStateCovarianceScalar = 1.0;
  for (i = 0; i < 36; i++) {
    b_EKF->pSqrtStateCovariance[i] = iv[i];
  }
  b_EKF->pIsSetStateCovariance = true;
  b_EKF->pSqrtStateCovarianceScalar = -1.0;
  b_EKF->pIsValidStateTransitionFcn = false;
  b_EKF->pIsValidMeasurementFcn = false;
  b_EKF->pIsValidMeasurementFcn = false;
  b_EKF->pIsValidStateTransitionFcn = false;
  b_EKF->pSqrtProcessNoiseScalar = 1.0;
  b_st.site = &yc_emlrtRSI;
  c_ExtendedKalmanFilter_set_Proc(&b_st, b_EKF, varargin_22);
  for (i = 0; i < 16; i++) {
    b_EKF->pSqrtMeasurementNoise[i] = iv1[i];
  }
  b_EKF->pSqrtMeasurementNoiseScalar = -1.0;
  st.site = &qc_emlrtRSI;
  st.site = &rc_emlrtRSI;
  st.site = &rc_emlrtRSI;
  st.site = &rc_emlrtRSI;
  st.site = &rc_emlrtRSI;
  st.site = &sc_emlrtRSI;
  b_EKF->IsLastJacobianInitialized = false;
  b_EKF->pIsDistributionsSetup = false;
  b_EKF->pIsInitialized = false;
  return b_EKF;
}

/* End of code generation (trackingEKF.c) */
