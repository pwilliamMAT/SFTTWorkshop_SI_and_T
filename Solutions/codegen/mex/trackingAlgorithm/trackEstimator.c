/*
 * trackEstimator.c
 *
 * Code generation for function 'trackEstimator'
 *
 */

/* Include files */
#include "trackEstimator.h"
#include "ExtendedKalmanFilter.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo w_emlrtRSI = {
    43,                      /* lineNo */
    "defaultStateEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    25,                                    /* lineNo */
    "EKFStateEstimator/EKFStateEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    13,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    24,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    25,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    22,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    23,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    26,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI = {
    56,                            /* lineNo */
    "IPDAEstimator/IPDAEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    55,                            /* lineNo */
    "IPDAEstimator/IPDAEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo jb_emlrtRSI = {
    59,                            /* lineNo */
    "IPDAEstimator/IPDAEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo kb_emlrtRSI = {
    217,           /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    228,                /* lineNo */
    "parseCoderInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo mb_emlrtRSI = {
    25,                         /* lineNo */
    "constantPreservingStruct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\constantPreservingStruct.m" /* pathName */
};

static emlrtRSInfo rb_emlrtRSI = {
    40,                          /* lineNo */
    "stickyStruct/dotReference", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\stickyStruct.m" /* pathName */
};

static emlrtRSInfo sb_emlrtRSI = {
    46,                              /* lineNo */
    "TrackEstimator/TrackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo tb_emlrtRSI = {
    49,                              /* lineNo */
    "TrackEstimator/TrackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo ub_emlrtRSI = {
    151,           /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo vb_emlrtRSI = {
    162,                /* lineNo */
    "parseCoderInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo xb_emlrtRSI = {
    32,                                       /* lineNo */
    "IPDAEstimator/set.TargetSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ac_emlrtRSI = {
    41,                                       /* lineNo */
    "IPDAEstimator/set.SensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo bc_emlrtRSI = {
    56,                     /* lineNo */
    "TrackEstimator/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo cc_emlrtRSI = {
    71,                    /* lineNo */
    "IPDAEstimator/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    72,                    /* lineNo */
    "IPDAEstimator/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI = {
    67,                    /* lineNo */
    "IPDAEstimator/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo fc_emlrtRSI = {
    58,                        /* lineNo */
    "EKFStateEstimator/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo gc_emlrtRSI =
    {
        142,                       /* lineNo */
        "trackingEKF/trackingEKF", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI =
    {
        143,                       /* lineNo */
        "trackingEKF/trackingEKF", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI =
    {
        135,                       /* lineNo */
        "trackingEKF/trackingEKF", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI =
    {
        144,                       /* lineNo */
        "trackingEKF/trackingEKF", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackingE"
        "KF.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI = {
    535,                                         /* lineNo */
    "ExtendedKalmanFilter/ExtendedKalmanFilter", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

/* Function Definitions */
trackingEKF *trackEstimator(
    const emlrtStack *sp, const c_fusion_tracker_targetspecs_Pa *tgtSpecs,
    const c_fusion_tracker_sensorspecs_Ae *sensorSpecs, trackingEKF *iobj_0,
    c_fusion_tracker_targetspecs_Pa *c_trkEstimator_StateEstimator_S,
    c_fusion_tracker_sensorspecs_Ae *d_trkEstimator_StateEstimator_S,
    c_fusion_tracker_sensorspecs_Ae *c_trkEstimator_StateEstimator_E)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  trackingEKF *e_trkEstimator_StateEstimator_S;
  int32_T i;
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
  st.site = &bb_emlrtRSI;
  b_st.site = &w_emlrtRSI;
  c_st.site = &x_emlrtRSI;
  st.site = &eb_emlrtRSI;
  b_st.site = &hb_emlrtRSI;
  c_st.site = &kb_emlrtRSI;
  d_st.site = &lb_emlrtRSI;
  e_st.site = &mb_emlrtRSI;
  b_st.site = &ib_emlrtRSI;
  b_st.site = &jb_emlrtRSI;
  c_st.site = &rb_emlrtRSI;
  st.site = &fb_emlrtRSI;
  b_st.site = &sb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  e_st.site = &mb_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &rb_emlrtRSI;
  st.site = &cb_emlrtRSI;
  b_st.site = &wb_emlrtRSI;
  c_st.site = &xb_emlrtRSI;
  st.site = &db_emlrtRSI;
  b_st.site = &yb_emlrtRSI;
  c_st.site = &ac_emlrtRSI;
  st.site = &gb_emlrtRSI;
  b_st.site = &bc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  d_st.site = &fc_emlrtRSI;
  e_st.site = &gc_emlrtRSI;
  iobj_0->pIsFirstCallPredict = true;
  iobj_0->pIsFirstCallCorrect = true;
  for (i = 0; i < 6; i++) {
    iobj_0->pState[i] = 0.0;
  }
  iobj_0->pSqrtStateCovarianceScalar = 1.0;
  for (i = 0; i < 36; i++) {
    iobj_0->pSqrtStateCovariance[i] = iv[i];
  }
  iobj_0->pIsSetStateCovariance = true;
  iobj_0->pSqrtStateCovarianceScalar = -1.0;
  iobj_0->pIsValidStateTransitionFcn = false;
  iobj_0->pIsValidMeasurementFcn = false;
  iobj_0->pIsValidMeasurementFcn = false;
  iobj_0->pIsValidStateTransitionFcn = false;
  iobj_0->pSqrtProcessNoiseScalar = 1.0;
  f_st.site = &pc_emlrtRSI;
  c_ExtendedKalmanFilter_set_Proc(
      &f_st, iobj_0, tgtSpecs->StateTransitionModel.PropAccelerationVariance);
  for (i = 0; i < 16; i++) {
    iobj_0->pSqrtMeasurementNoise[i] = iv1[i];
  }
  iobj_0->pSqrtMeasurementNoiseScalar = -1.0;
  e_st.site = &hc_emlrtRSI;
  e_st.site = &ic_emlrtRSI;
  e_st.site = &ic_emlrtRSI;
  e_st.site = &ic_emlrtRSI;
  e_st.site = &ic_emlrtRSI;
  e_st.site = &jc_emlrtRSI;
  iobj_0->IsLastJacobianInitialized = false;
  iobj_0->pIsDistributionsSetup = false;
  iobj_0->pIsInitialized = false;
  c_st.site = &cc_emlrtRSI;
  c_st.site = &dc_emlrtRSI;
  *c_trkEstimator_StateEstimator_S = *tgtSpecs;
  *d_trkEstimator_StateEstimator_S = *sensorSpecs;
  e_trkEstimator_StateEstimator_S = iobj_0;
  *c_trkEstimator_StateEstimator_E = *sensorSpecs;
  return e_trkEstimator_StateEstimator_S;
}

/* End of code generation (trackEstimator.c) */
