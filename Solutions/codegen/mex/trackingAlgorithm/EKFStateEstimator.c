/*
 * EKFStateEstimator.c
 *
 * Code generation for function 'EKFStateEstimator'
 *
 */

/* Include files */
#include "EKFStateEstimator.h"
#include "ConstantVelocityModel.h"
#include "ExtendedKalmanFilter.h"
#include "eml_int_forloop_overflow_check.h"
#include "predictStateNonAdditive.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo uj_emlrtRSI = {
    1639,                                                          /* lineNo */
    "ExtendedKalmanFilter/stateCovarianceScalarExpandIfNecessary", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ft_emlrtRSI = {
    114,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo gt_emlrtRSI = {
    115,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ht_emlrtRSI = {
    119,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo it_emlrtRSI = {
    122,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo jt_emlrtRSI = {
    125,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo kt_emlrtRSI = {
    126,                         /* lineNo */
    "EKFStateEstimator/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo pt_emlrtRSI =
    {
        168,                   /* lineNo */
        "trackingEKF/predict", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo qt_emlrtRSI =
    {
        171,                   /* lineNo */
        "trackingEKF/predict", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo rt_emlrtRSI = {
    154,                                          /* lineNo */
    "AbstractSmoother/setupInitialDistributions", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\smoothers\\+matlabshared\\+"
    "smoothers\\+internal\\AbstractSmoother.m" /* pathName */
};

static emlrtRSInfo st_emlrtRSI = {
    42,                                          /* lineNo */
    "LinearizedSmoother/ensureMethodDefinition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\smoothers\\+matlabshared\\+"
    "smoothers\\+internal\\LinearizedSmoother.m" /* pathName */
};

static emlrtRSInfo tt_emlrtRSI = {
    31,                                               /* lineNo */
    "LinearizedSmoother/ensureLastJacobianIsDefined", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\smoothers\\+matlabshared\\+"
    "smoothers\\+internal\\LinearizedSmoother.m" /* pathName */
};

static emlrtRSInfo ut_emlrtRSI = {
    220,                                      /* lineNo */
    "GaussianSmoother/getStateAndCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\smoothers\\+matlabshared\\+"
    "smoothers\\+internal\\GaussianSmoother.m" /* pathName */
};

static emlrtRSInfo vt_emlrtRSI = {
    611,                            /* lineNo */
    "ExtendedKalmanFilter/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo wt_emlrtRSI = {
    614,                            /* lineNo */
    "ExtendedKalmanFilter/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo xt_emlrtRSI = {
    1593,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo yt_emlrtRSI = {
    1594,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo au_emlrtRSI = {
    1618,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo bu_emlrtRSI = {
    1628,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureFilterPredictionReadiness", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo du_emlrtRSI = {
    1920,                                               /* lineNo */
    "ExtendedKalmanFilter/ensureProcessNoiseIsDefined", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo eu_emlrtRSI = {
    1652,                                                       /* lineNo */
    "ExtendedKalmanFilter/processNoiseScalarExpandIfNecessary", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo fu_emlrtRSI = {
    13,                                                   /* lineNo */
    "EKFPredictorNonAdditive/validateStateTransitionFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFPredictorNonA"
    "dditive.m" /* pathName */
};

static emlrtRSInfo lu_emlrtRSI = {
    1941,                                                      /* lineNo */
    "ExtendedKalmanFilter/validateStateTransitionJacobianFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo mu_emlrtRSI = {
    1943,                                                      /* lineNo */
    "ExtendedKalmanFilter/validateStateTransitionJacobianFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo nu_emlrtRSI = {
    18,                                                           /* lineNo */
    "EKFPredictorNonAdditive/validateStateTransitionJacobianFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFPredictorNonA"
    "dditive.m" /* pathName */
};

static emlrtRSInfo ou_emlrtRSI = {
    13,                                /* lineNo */
    "predictStateNonAdditiveJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\predictStateNonAdditiveJ"
    "acobian.m" /* pathName */
};

static emlrtRSInfo tu_emlrtRSI = {
    64,                                /* lineNo */
    "EKFPredictorNonAdditive/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFPredictorNonA"
    "dditive.m" /* pathName */
};

static emlrtRSInfo uu_emlrtRSI = {
    66,                                /* lineNo */
    "EKFPredictorNonAdditive/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFPredictorNonA"
    "dditive.m" /* pathName */
};

static emlrtRSInfo vu_emlrtRSI = {
    67,                                /* lineNo */
    "EKFPredictorNonAdditive/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFPredictorNonA"
    "dditive.m" /* pathName */
};

static emlrtRSInfo wu_emlrtRSI = {
    102,                                          /* lineNo */
    "EKFPredictorNonAdditive/predictionMatrices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFPredictorNonA"
    "dditive.m" /* pathName */
};

static emlrtRSInfo rv_emlrtRSI = {
    91,                                  /* lineNo */
    "EKFStateEstimator/gateProbability", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo sv_emlrtRSI = {
    12,                /* lineNo */
    "gateProbability", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gateProbability.m" /* pathName */
};

static emlrtRSInfo
    tv_emlrtRSI =
        {
            43,         /* lineNo */
            "gammainc", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\gammain"
            "c.m" /* pathName */
};

static emlrtRSInfo
    uv_emlrtRSI =
        {
            92,                /* lineNo */
            "scalar_gammainc", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\gammain"
            "c.m" /* pathName */
};

static emlrtRSInfo vv_emlrtRSI = {
    374,            /* lineNo */
    "eml_gammainc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\private\\eml_"
    "gammainc.m" /* pathName */
};

static emlrtRSInfo xv_emlrtRSI = {
    294,            /* lineNo */
    "eml_gammainc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\private\\eml_"
    "gammainc.m" /* pathName */
};

static emlrtRSInfo yv_emlrtRSI = {
    198,            /* lineNo */
    "eml_gammainc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\private\\eml_"
    "gammainc.m" /* pathName */
};

static emlrtRSInfo hw_emlrtRSI = {
    80,                                       /* lineNo */
    "EKFStateEstimator/detectionProbability", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo iw_emlrtRSI = {
    61,                                        /* lineNo */
    "CompositeFieldOfViewModel/detectability", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "detectability\\CompositeFieldOfViewModel.m" /* pathName */
};

static emlrtRSInfo jw_emlrtRSI = {
    62,                                        /* lineNo */
    "CompositeFieldOfViewModel/detectability", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "detectability\\CompositeFieldOfViewModel.m" /* pathName */
};

static emlrtRTEInfo v_emlrtRTEI = {
    356,            /* lineNo */
    13,             /* colNo */
    "eml_gammainc", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\private\\eml_"
    "gammainc.m" /* pName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    276,            /* lineNo */
    13,             /* colNo */
    "eml_gammainc", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\private\\eml_"
    "gammainc.m" /* pName */
};

static emlrtBCInfo bd_emlrtBCI = {
    0,                                         /* iFirst */
    107,                                       /* iLast */
    62,                                        /* lineNo */
    55,                                        /* colNo */
    "",                                        /* aName */
    "CompositeFieldOfViewModel/detectability", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "detectability\\CompositeFieldOfViewModel.m", /* pName */
    0                                             /* checkKind */
};

/* Function Definitions */
void EKFStateEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *estimator_TargetSpecifications,
    trackingEKF *estimator_TrackingFilter, struct_T *pdf, real_T dT)
{
  static const int8_T b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
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
  real_T M[54];
  real_T b_estimator_TrackingFilter[36];
  real_T B[18];
  real_T dv[18];
  real_T tau[6];
  real_T work[6];
  real_T x[6];
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
  l_st.prev = &k_st;
  l_st.tls = k_st.tls;
  st.site = &ft_emlrtRSI;
  ExtendedKalmanFilter_set_State(&st, estimator_TrackingFilter, pdf->State);
  st.site = &gt_emlrtRSI;
  c_ExtendedKalmanFilter_set_Stat(&st, estimator_TrackingFilter,
                                  pdf->StateCovariance);
  st.site = &ht_emlrtRSI;
  c_ExtendedKalmanFilter_set_Proc(
      &st, estimator_TrackingFilter,
      estimator_TargetSpecifications->StateTransitionModel
          .PropAccelerationVariance);
  st.site = &it_emlrtRSI;
  b_st.site = &pt_emlrtRSI;
  c_st.site = &rt_emlrtRSI;
  d_st.site = &st_emlrtRSI;
  if (!estimator_TrackingFilter->IsLastJacobianInitialized) {
    e_st.site = &tt_emlrtRSI;
    f_st.site = &ut_emlrtRSI;
    estimator_TrackingFilter->IsLastJacobianInitialized = true;
  }
  if (!estimator_TrackingFilter->pIsDistributionsSetup) {
    estimator_TrackingFilter->pIsDistributionsSetup = true;
  }
  b_st.site = &qt_emlrtRSI;
  c_st.site = &vt_emlrtRSI;
  d_st.site = &xt_emlrtRSI;
  e_st.site = &cu_emlrtRSI;
  if ((!estimator_TrackingFilter->pIsSetStateCovariance) ||
      (estimator_TrackingFilter->pSqrtStateCovarianceScalar != -1.0)) {
    xnorm = estimator_TrackingFilter->pSqrtStateCovarianceScalar;
    f_st.site = &uj_emlrtRSI;
    for (i = 0; i < 36; i++) {
      estimator_TrackingFilter->pSqrtStateCovariance[i] = xnorm * (real_T)iv[i];
    }
  }
  d_st.site = &yt_emlrtRSI;
  e_st.site = &du_emlrtRSI;
  if ((!estimator_TrackingFilter->pIsSetProcessNoise) ||
      (estimator_TrackingFilter->pSqrtProcessNoiseScalar != -1.0)) {
    xnorm = estimator_TrackingFilter->pSqrtProcessNoiseScalar;
    f_st.site = &eu_emlrtRSI;
    for (i = 0; i < 9; i++) {
      estimator_TrackingFilter->pSqrtProcessNoise[i] = xnorm * (real_T)b[i];
    }
    estimator_TrackingFilter->pIsSetProcessNoise = true;
    estimator_TrackingFilter->pSqrtProcessNoiseScalar = -1.0;
  }
  if (estimator_TrackingFilter->pIsFirstCallPredict) {
    if (!estimator_TrackingFilter->pIsValidStateTransitionFcn) {
      d_st.site = &au_emlrtRSI;
      for (i = 0; i < 6; i++) {
        x[i] = estimator_TrackingFilter->pState[i];
      }
      e_st.site = &fu_emlrtRSI;
      predictStateNonAdditive(&e_st, x, dT);
      estimator_TrackingFilter->pIsValidStateTransitionFcn = true;
    }
    d_st.site = &bu_emlrtRSI;
    e_st.site = &lu_emlrtRSI;
    for (i = 0; i < 6; i++) {
      x[i] = estimator_TrackingFilter->pState[i];
    }
    f_st.site = &nu_emlrtRSI;
    g_st.site = &ou_emlrtRSI;
    c_ConstantVelocityModel_predict(&g_st, x, dT,
                                    estimator_TrackingFilter->pJacobian, B);
    e_st.site = &mu_emlrtRSI;
    estimator_TrackingFilter->pIsFirstCallPredict = false;
  }
  c_st.site = &wt_emlrtRSI;
  for (i = 0; i < 6; i++) {
    x[i] = estimator_TrackingFilter->pState[i];
  }
  d_st.site = &tu_emlrtRSI;
  e_st.site = &wu_emlrtRSI;
  f_st.site = &ou_emlrtRSI;
  c_ConstantVelocityModel_predict(&f_st, x, dT,
                                  estimator_TrackingFilter->pJacobian, B);
  d_st.site = &uu_emlrtRSI;
  predictStateNonAdditive(&d_st, x, dT);
  d_st.site = &vu_emlrtRSI;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      xnorm = 0.0;
      for (c_i = 0; c_i < 6; c_i++) {
        xnorm += estimator_TrackingFilter->pSqrtStateCovariance[c_i + 6 * b_i] *
                 estimator_TrackingFilter->pJacobian[i + 6 * c_i];
      }
      b_estimator_TrackingFilter[b_i + 6 * i] = xnorm;
    }
  }
  memset(&dv[0], 0, 18U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    d_i = 6 * i + 2;
    lastc = 6 * i + 4;
    for (c_i = 0; c_i < 3; c_i++) {
      r = _mm_loadu_pd(&B[6 * c_i]);
      r1 = _mm_loadu_pd(&dv[6 * i]);
      r2 =
          _mm_set1_pd(estimator_TrackingFilter->pSqrtProcessNoise[c_i + 3 * i]);
      _mm_storeu_pd(&dv[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&B[6 * c_i + 2]);
      r1 = _mm_loadu_pd(&dv[d_i]);
      _mm_storeu_pd(&dv[d_i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&B[6 * c_i + 4]);
      r1 = _mm_loadu_pd(&dv[lastc]);
      _mm_storeu_pd(&dv[lastc], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      M[b_i + 9 * i] = b_estimator_TrackingFilter[b_i + 6 * i];
    }
    M[9 * i + 6] = dv[i];
    M[9 * i + 7] = dv[i + 6];
    M[9 * i + 8] = dv[i + 12];
  }
  e_st.site = &xu_emlrtRSI;
  f_st.site = &yu_emlrtRSI;
  g_st.site = &av_emlrtRSI;
  h_st.site = &bv_emlrtRSI;
  i_st.site = &ev_emlrtRSI;
  j_st.site = &fv_emlrtRSI;
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }
  for (b_i = 0; b_i < 6; b_i++) {
    real_T atmp;
    ii = b_i * 9 + b_i;
    atmp = M[ii];
    k_st.site = &hv_emlrtRSI;
    itau = ii + 2;
    tau[b_i] = 0.0;
    l_st.site = &sc_emlrtRSI;
    xnorm = e_xnrm2(&l_st, 8 - b_i, M, ii + 2);
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
        l_st.site = &uc_emlrtRSI;
        xnorm = e_xnrm2(&l_st, 8 - b_i, M, ii + 2);
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
      k_st.site = &gv_emlrtRSI;
      if (tau[b_i] != 0.0) {
        lastv = 9 - b_i;
        d_i = (ii - b_i) + 8;
        while ((lastv > 0) && (M[d_i] == 0.0)) {
          lastv--;
          d_i--;
        }
        l_st.site = &gf_emlrtRSI;
        lastc = ilazlc(&l_st, lastv, 5 - b_i, M, ii + 10);
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        l_st.site = &hf_emlrtRSI;
        b_xgemv(&l_st, lastv, lastc, M, ii + 10, M, ii + 1, work);
        l_st.site = &if_emlrtRSI;
        b_xgerc(&l_st, lastv, lastc, -tau[b_i], ii + 1, work, M, ii + 10);
      }
      M[ii] = atmp;
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    h_st.site = &cv_emlrtRSI;
    for (i = 0; i <= b_i; i++) {
      estimator_TrackingFilter->pSqrtStateCovariance[i + 6 * b_i] =
          M[i + 9 * b_i];
    }
    d_i = b_i + 2;
    if (d_i <= 6) {
      memset(
          &estimator_TrackingFilter->pSqrtStateCovariance[(b_i * 6 + d_i) + -1],
          0, (uint32_T)(-d_i + 7) * sizeof(real_T));
    }
  }
  h_st.site = &dv_emlrtRSI;
  i_st.site = &iv_emlrtRSI;
  j_st.site = &jv_emlrtRSI;
  itau = 5;
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }
  for (c_i = 5; c_i >= 0; c_i--) {
    ii = c_i + c_i * 9;
    if (c_i + 1 < 6) {
      M[ii] = 1.0;
      k_st.site = &mv_emlrtRSI;
      if (tau[itau] != 0.0) {
        lastc = 9 - c_i;
        d_i = (ii - c_i) + 8;
        while ((lastc > 0) && (M[d_i] == 0.0)) {
          lastc--;
          d_i--;
        }
        l_st.site = &gf_emlrtRSI;
        d_i = ilazlc(&l_st, lastc, 5 - c_i, M, ii + 10);
      } else {
        lastc = 0;
        d_i = 0;
      }
      if (lastc > 0) {
        l_st.site = &hf_emlrtRSI;
        b_xgemv(&l_st, lastc, d_i, M, ii + 10, M, ii + 1, work);
        l_st.site = &if_emlrtRSI;
        b_xgerc(&l_st, lastc, d_i, -tau[itau], ii + 1, work, M, ii + 10);
      }
    }
    d_i = ii + 2;
    k_st.site = &lv_emlrtRSI;
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
    k_st.site = &kv_emlrtRSI;
    for (i = 0; i < c_i; i++) {
      M[(ii - i) - 1] = 0.0;
    }
    itau = c_i - 1;
  }
  for (i = 0; i < 6; i++) {
    estimator_TrackingFilter->pState[i] = x[i];
  }
  c_st.site = &wt_emlrtRSI;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      b_estimator_TrackingFilter[b_i + 6 * i] =
          estimator_TrackingFilter->pSqrtStateCovariance[i + 6 * b_i];
    }
  }
  memcpy(&estimator_TrackingFilter->pSqrtStateCovariance[0],
         &b_estimator_TrackingFilter[0], 36U * sizeof(real_T));
  estimator_TrackingFilter->pIsSetStateCovariance = true;
  estimator_TrackingFilter->pSqrtStateCovarianceScalar = -1.0;
  st.site = &jt_emlrtRSI;
  for (i = 0; i < 6; i++) {
    pdf->State[i] = estimator_TrackingFilter->pState[i];
  }
  st.site = &kt_emlrtRSI;
  b_st.site = &sj_emlrtRSI;
  if ((!estimator_TrackingFilter->pIsSetStateCovariance) ||
      (estimator_TrackingFilter->pSqrtStateCovarianceScalar != -1.0)) {
    xnorm = estimator_TrackingFilter->pSqrtStateCovarianceScalar;
    c_st.site = &uj_emlrtRSI;
    for (i = 0; i < 36; i++) {
      estimator_TrackingFilter->pSqrtStateCovariance[i] = xnorm * (real_T)iv[i];
    }
    estimator_TrackingFilter->pIsSetStateCovariance = true;
    estimator_TrackingFilter->pSqrtStateCovarianceScalar = -1.0;
  }
  b_st.site = &tj_emlrtRSI;
  for (c_i = 0; c_i < 6; c_i++) {
    for (i = 0; i < 6; i++) {
      pdf->StateCovariance[i + 6 * c_i] = 0.0;
    }
    d_i = 6 * c_i + 2;
    lastc = 6 * c_i + 4;
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&estimator_TrackingFilter->pSqrtStateCovariance[6 * i]);
      r1 = _mm_loadu_pd(&pdf->StateCovariance[6 * c_i]);
      r2 = _mm_set1_pd(
          estimator_TrackingFilter->pSqrtStateCovariance[c_i + 6 * i]);
      _mm_storeu_pd(&pdf->StateCovariance[6 * c_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(
          &estimator_TrackingFilter->pSqrtStateCovariance[6 * i + 2]);
      r1 = _mm_loadu_pd(&pdf->StateCovariance[d_i]);
      _mm_storeu_pd(&pdf->StateCovariance[d_i],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(
          &estimator_TrackingFilter->pSqrtStateCovariance[6 * i + 4]);
      r1 = _mm_loadu_pd(&pdf->StateCovariance[lastc]);
      _mm_storeu_pd(&pdf->StateCovariance[lastc],
                    _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
}

real_T c_EKFStateEstimator_detectionPr(
    const emlrtStack *sp,
    const c_fusion_tracker_sensorspecs_Ae *estimator_SensorSpecifications,
    const real_T pdf_State[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_xP[3];
  real_T c_xP[3];
  real_T xP[3];
  real_T Pd;
  real_T Pmiss;
  real_T xE_idx_0;
  real_T xE_idx_1;
  real_T xE_idx_2;
  real_T xE_idx_3;
  real_T xE_idx_4;
  real_T xE_idx_5;
  int32_T b;
  int32_T b_i;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  xE_idx_0 = pdf_State[0];
  xE_idx_3 = pdf_State[1];
  xE_idx_1 = pdf_State[2];
  xE_idx_4 = pdf_State[3];
  xE_idx_2 = pdf_State[4];
  xE_idx_5 = pdf_State[5];
  st.site = &hw_emlrtRSI;
  Pmiss = 1.0;
  b = estimator_SensorSpecifications->DetectabilityModel.NumModels;
  b_st.site = &iw_emlrtRSI;
  if (estimator_SensorSpecifications->DetectabilityModel.NumModels >
      2147483646) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (i = 0; i < b; i++) {
    __m128d r;
    real_T azimuth;
    real_T d;
    real_T d1;
    real_T d2;
    real_T d3;
    real_T d4;
    real_T d5;
    real_T d6;
    real_T d7;
    real_T elevation;
    real_T rr;
    real_T xP_tmp;
    b_st.site = &jw_emlrtRSI;
    if (i > 107) {
      emlrtDynamicBoundsCheckR2012b(i, 0, 107, &bd_emlrtBCI, &b_st);
    }
    xP[0] = xE_idx_0;
    b_xP[0] = xE_idx_3;
    xP[1] = xE_idx_1;
    b_xP[1] = xE_idx_4;
    xP[2] = xE_idx_2;
    b_xP[2] = xE_idx_5;
    for (b_i = 0; b_i < 3; b_i++) {
      int32_T c_i;
      int32_T i1;
      r = _mm_loadu_pd(&xP[0]);
      c_i = 3 * (2 - b_i);
      _mm_storeu_pd(
          &c_xP[0],
          _mm_sub_pd(r, _mm_loadu_pd(
                            &estimator_SensorSpecifications->DetectabilityModel
                                 .FieldsOfView[i]
                                 .OriginPosition[c_i])));
      c_xP[2] = xP[2] - estimator_SensorSpecifications->DetectabilityModel
                            .FieldsOfView[i]
                            .OriginPosition[c_i + 2];
      memset(&xP[0], 0, 3U * sizeof(real_T));
      r = _mm_loadu_pd(&xP[0]);
      i1 = 9 * (2 - b_i);
      _mm_storeu_pd(
          &xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &estimator_SensorSpecifications
                                            ->DetectabilityModel.FieldsOfView[i]
                                            .Orientation[i1]),
                                   _mm_set1_pd(c_xP[0]))));
      Pd = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
               .Orientation[i1 + 2];
      xP[2] += Pd * c_xP[0];
      c_xP[0] = b_xP[0] - estimator_SensorSpecifications->DetectabilityModel
                              .FieldsOfView[i]
                              .OriginVelocity[c_i];
      r = _mm_loadu_pd(&xP[0]);
      _mm_storeu_pd(
          &xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &estimator_SensorSpecifications
                                            ->DetectabilityModel.FieldsOfView[i]
                                            .Orientation[i1 + 3]),
                                   _mm_set1_pd(c_xP[1]))));
      rr = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
               .Orientation[i1 + 5];
      xP[2] += rr * c_xP[1];
      c_xP[1] = b_xP[1] - estimator_SensorSpecifications->DetectabilityModel
                              .FieldsOfView[i]
                              .OriginVelocity[c_i + 1];
      r = _mm_loadu_pd(&xP[0]);
      _mm_storeu_pd(
          &xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &estimator_SensorSpecifications
                                            ->DetectabilityModel.FieldsOfView[i]
                                            .Orientation[i1 + 6]),
                                   _mm_set1_pd(c_xP[2]))));
      xP_tmp =
          estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
              .Orientation[i1 + 8];
      xP[2] += xP_tmp * c_xP[2];
      c_xP[2] = b_xP[2] - estimator_SensorSpecifications->DetectabilityModel
                              .FieldsOfView[i]
                              .OriginVelocity[c_i + 2];
      memset(&b_xP[0], 0, 3U * sizeof(real_T));
      r = _mm_loadu_pd(&b_xP[0]);
      _mm_storeu_pd(
          &b_xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &estimator_SensorSpecifications
                                            ->DetectabilityModel.FieldsOfView[i]
                                            .Orientation[i1]),
                                   _mm_set1_pd(c_xP[0]))));
      b_xP[2] += Pd * c_xP[0];
      r = _mm_loadu_pd(&b_xP[0]);
      _mm_storeu_pd(
          &b_xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &estimator_SensorSpecifications
                                            ->DetectabilityModel.FieldsOfView[i]
                                            .Orientation[i1 + 3]),
                                   _mm_set1_pd(c_xP[1]))));
      b_xP[2] += rr * c_xP[1];
      r = _mm_loadu_pd(&b_xP[0]);
      _mm_storeu_pd(
          &b_xP[0],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(
                                       &estimator_SensorSpecifications
                                            ->DetectabilityModel.FieldsOfView[i]
                                            .Orientation[i1 + 6]),
                                   _mm_set1_pd(c_xP[2]))));
      b_xP[2] += xP_tmp * c_xP[2];
    }
    azimuth = 57.295779513082323 * muDoubleScalarAtan2(xP[1], xP[0]);
    Pd = xP[0] * xP[0] + xP[1] * xP[1];
    elevation =
        57.295779513082323 * muDoubleScalarAtan2(xP[2], muDoubleScalarSqrt(Pd));
    Pd = muDoubleScalarSqrt(Pd + xP[2] * xP[2]);
    r = _mm_loadu_pd(&xP[0]);
    _mm_storeu_pd(&xP[0], _mm_div_pd(r, _mm_set1_pd(Pd)));
    xP[2] /= Pd;
    rr = (b_xP[0] * xP[0] + b_xP[1] * xP[1]) + b_xP[2] * xP[2];
    xP_tmp = 0.0;
    d = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
            .AzimuthLimits[0];
    d1 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .AzimuthLimits[1];
    d2 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .ElevationLimits[0];
    d3 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .ElevationLimits[1];
    d4 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .RangeLimits[0];
    d5 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .RangeLimits[1];
    d6 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .RangeRateLimits[0];
    d7 = estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
             .RangeRateLimits[1];
    if ((azimuth >= d) && (azimuth <= d1) &&
        ((elevation >= d2) && (elevation <= d3)) &&
        ((Pd >= d4) && (Pd <= d5)) && ((rr >= d6) && (rr <= d7))) {
      xP_tmp =
          estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
              .DetectionProbability;
    }
    if ((!(azimuth >= d)) || (!(azimuth <= d1)) ||
        ((!(elevation >= d2)) || (!(elevation <= d3))) ||
        ((!(Pd >= d4)) || (!(Pd <= d5))) || ((!(rr >= d6)) || (!(rr <= d7)))) {
      xP_tmp =
          estimator_SensorSpecifications->DetectabilityModel.FieldsOfView[i]
              .MinDetectionProbability;
    }
    Pmiss *= 1.0 - xP_tmp;
  }
  return 1.0 - Pmiss;
}

real_T c_EKFStateEstimator_gateProbabi(const emlrtStack *sp, real_T gateSize)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T Pg;
  real_T x;
  int32_T b_i;
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
  st.site = &rv_emlrtRSI;
  b_st.site = &id_emlrtRSI;
  c_st.site = &jd_emlrtRSI;
  st.site = &rv_emlrtRSI;
  b_st.site = &sv_emlrtRSI;
  b_st.site = &sv_emlrtRSI;
  x = muDoubleScalarSqrt(gateSize * gateSize) / 2.0;
  c_st.site = &tv_emlrtRSI;
  d_st.site = &uv_emlrtRSI;
  if (!(x > 0.0)) {
    if (x == 0.0) {
      Pg = 0.0;
    } else {
      Pg = rtNaN;
    }
  } else if (muDoubleScalarIsInf(x)) {
    Pg = 1.0;
  } else {
    real_T a2;
    real_T a3;
    real_T b_t;
    real_T t;
    real_T xD0;
    int32_T exitg1;
    if (muDoubleScalarAbs(2.0 - x) > 0.1 * (x + 2.0)) {
      if (2.2250738585072014E-308 * x > 2.0) {
        xD0 = x;
      } else if ((x < 1.0) && (1.7976931348623157E+308 * x < 2.0)) {
        xD0 = (1.3862943611198906 - 2.0 * muDoubleScalarLog(x)) - 2.0;
      } else {
        xD0 = (2.0 * muDoubleScalarLog(2.0 / x) + x) - 2.0;
      }
    } else {
      t = x / 2.0;
      Pg = (1.0 - t) / (t + 1.0);
      a2 = Pg * Pg;
      xD0 = (2.0 - x) * Pg;
      a3 = xD0;
      Pg = 2.0 * (2.0 * Pg);
      b_t = 3.0;
      do {
        exitg1 = 0;
        Pg *= a2;
        xD0 += Pg / b_t;
        if (xD0 == a3) {
          exitg1 = 1;
        } else {
          a3 = xD0;
          b_t += 2.0;
        }
      } while (exitg1 == 0);
    }
    if (x > 1.0E+6) {
      real_T sqrtx;
      real_T tsq;
      sqrtx = muDoubleScalarSqrt(x);
      t = muDoubleScalarAbs((2.0 - x) - 1.0) / sqrtx;
      tsq = t * t;
      if (t < 15.0) {
        Pg = 0.70710678118654746 * t;
        b_t = 3.97886080735226 / (Pg + 3.97886080735226);
        b_t = 0.5 *
              ((((((((((((((((((((((0.0012710976495261409 * (b_t - 0.5) +
                                    0.00011931402283834095) *
                                       (b_t - 0.5) -
                                   0.0039638509736051354) *
                                      (b_t - 0.5) -
                                  0.00087077963531729586) *
                                     (b_t - 0.5) +
                                 0.0077367252831352668) *
                                    (b_t - 0.5) +
                                0.0038333512626488732) *
                                   (b_t - 0.5) -
                               0.012722381378212275) *
                                  (b_t - 0.5) -
                              0.013382364453346007) *
                                 (b_t - 0.5) +
                             0.016131532973325226) *
                                (b_t - 0.5) +
                            0.039097684558848406) *
                               (b_t - 0.5) +
                           0.0024936720005350331) *
                              (b_t - 0.5) -
                          0.0838864557023002) *
                             (b_t - 0.5) -
                         0.11946395996432542) *
                            (b_t - 0.5) +
                        0.016620792496936737) *
                           (b_t - 0.5) +
                       0.35752427444953105) *
                          (b_t - 0.5) +
                      0.80527640875291062) *
                         (b_t - 0.5) +
                     1.1890298290927332) *
                        (b_t - 0.5) +
                    1.3704021768233816) *
                       (b_t - 0.5) +
                   1.313146538310231) *
                      (b_t - 0.5) +
                  1.0792551515585667) *
                     (b_t - 0.5) +
                 0.77436819911953858) *
                    (b_t - 0.5) +
                0.49016508058531844) *
                   (b_t - 0.5) +
               0.27537474159737679) *
              b_t * muDoubleScalarExp(-Pg * Pg) * 2.5066282746310002 *
              muDoubleScalarExp(0.5 * tsq);
        a2 = (b_t * ((tsq - 3.0) * t) - (tsq - 4.0)) / 6.0;
        a3 = (b_t * ((tsq * tsq - 9.0) * tsq + 6.0) -
              ((tsq - 1.0) * tsq - 6.0) * t) /
             72.0;
        Pg = 5.0 * tsq;
        Pg =
            (b_t * (((((Pg + 45.0) * tsq - 81.0) * tsq - 315.0) * tsq + 270.0) *
                    t) -
             ((((Pg + 40.0) * tsq - 111.0) * tsq - 174.0) * tsq + 192.0)) /
            6480.0;
      } else {
        b_t = (((3.0 - 15.0 / tsq) / tsq - 1.0) / tsq + 1.0) / t;
        a2 = (((25.0 - 210.0 / tsq) / tsq - 4.0) / tsq + 1.0) / tsq;
        a3 = (((130.0 - 1750.0 / tsq) / tsq - 11.0) / tsq + 1.0) / (tsq * t);
        Pg = (((546.0 - 11368.0 / tsq) / tsq - 26.0) / tsq + 1.0) / (tsq * tsq);
      }
      Pg = 2.0 * (((b_t / sqrtx + a2 / x) + a3 / (x * sqrtx)) + Pg / (x * x));
      if (-1.3068528194400546 - xD0 < 709.782712893384) {
        Pg *= muDoubleScalarExp(-1.3068528194400546 - xD0);
      } else {
        e_st.site = &yv_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &e_st, &x_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarExp((-1.3068528194400546 - xD0) +
                               muDoubleScalarLog(Pg));
      }
      Pg = 1.0 - Pg;
    } else if (x < 2.0) {
      int32_T i;
      b_t = 1.0;
      if (x > 4.4408920985006262E-16) {
        Pg = x / 2.0;
        b_t = 2.0;
        do {
          exitg1 = 0;
          Pg = x * Pg / ((b_t - 1.0) + 2.0);
          if (Pg < 2.2204460492503131E-16) {
            exitg1 = 1;
          } else {
            b_t++;
          }
        } while (exitg1 == 0);
      }
      Pg = 0.0;
      i = (int32_T) - ((-1.0 - (b_t - 1.0)) + 1.0);
      emlrtForLoopVectorCheckR2021a(b_t - 1.0, -1.0, 1.0, mxDOUBLE_CLASS,
                                    (int32_T) - ((-1.0 - (b_t - 1.0)) + 1.0),
                                    &w_emlrtRTEI, &d_st);
      for (b_i = 0; b_i < i; b_i++) {
        Pg = x * (Pg + 1.0) / (((b_t - 1.0) - (real_T)b_i) + 2.0);
      }
      Pg++;
      if (-1.3068528194400546 - xD0 < 709.782712893384) {
        Pg *= muDoubleScalarExp(-1.3068528194400546 - xD0);
      } else {
        e_st.site = &xv_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &e_st, &x_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarExp((-1.3068528194400546 - xD0) +
                               muDoubleScalarLog(Pg));
      }
      if (Pg > 1.0) {
        Pg = 1.0;
      }
    } else {
      int32_T i;
      Pg = 1.0;
      b_t = 1.0;
      do {
        exitg1 = 0;
        i = (int32_T)muDoubleScalarFloor(x + 2.0);
        if ((int32_T)b_t <= i) {
          Pg = (2.0 - b_t) * Pg / x;
          if (muDoubleScalarAbs(Pg) < 2.2204460492503131E-16) {
            exitg1 = 1;
          } else {
            b_t++;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if ((int32_T)b_t <= i) {
        Pg = 1.0;
      } else {
        Pg = 1.0;
        b_t = 2.0;
      }
      i = (int32_T)b_t;
      emlrtForLoopVectorCheckR2021a(b_t - 1.0, -1.0, 1.0, mxDOUBLE_CLASS,
                                    (int32_T)b_t - 1, &v_emlrtRTEI, &d_st);
      for (b_i = 0; b_i <= i - 2; b_i++) {
        Pg = (2.0 - ((b_t - 1.0) - (real_T)b_i)) * Pg / x + 1.0;
      }
      Pg = Pg * 2.0 / x;
      if (-1.3068528194400546 - xD0 < 709.782712893384) {
        Pg *= muDoubleScalarExp(-1.3068528194400546 - xD0);
      } else {
        e_st.site = &vv_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &e_st, &x_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarExp((-1.3068528194400546 - xD0) +
                               muDoubleScalarLog(Pg));
      }
      if (Pg > 1.0) {
        Pg = 1.0;
      }
      Pg = 1.0 - Pg;
    }
  }
  return Pg;
}

/* End of code generation (EKFStateEstimator.c) */
