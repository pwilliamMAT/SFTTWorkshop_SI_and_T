/*
 * trackingAlgorithm.c
 *
 * Code generation for function 'trackingAlgorithm'
 *
 */

/* Include files */
#include "trackingAlgorithm.h"
#include "JIPDATracker.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "omp.h"
#include <string.h>

/* Variable Definitions */
static fusion_tracker_JIPDATracker tracker;

static boolean_T tracker_not_empty;

static emlrtRSInfo emlrtRSI = {
    12,                  /* lineNo */
    "trackingAlgorithm", /* fcnName */
    "/MATLAB "
    "Drive/Repositories/SFTTWorkshop_SI_and_T-3/HelperFunctions/"
    "trackingAlgorithm.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    17,                  /* lineNo */
    "trackingAlgorithm", /* fcnName */
    "/MATLAB "
    "Drive/Repositories/SFTTWorkshop_SI_and_T-3/HelperFunctions/"
    "trackingAlgorithm.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    11,                  /* lineNo */
    "trackingAlgorithm", /* fcnName */
    "/MATLAB "
    "Drive/Repositories/SFTTWorkshop_SI_and_T-3/HelperFunctions/"
    "trackingAlgorithm.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    34,                         /* lineNo */
    "multiSensorTargetTracker", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/multiSensorTargetTracker.m" /* pathName
                                                                     */
};

static emlrtRSInfo e_emlrtRSI = {
    50,                         /* lineNo */
    "multiSensorTargetTracker", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/multiSensorTargetTracker.m" /* pathName
                                                                     */
};

static emlrtRSInfo f_emlrtRSI = {
    51,                         /* lineNo */
    "multiSensorTargetTracker", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/multiSensorTargetTracker.m" /* pathName
                                                                     */
};

static emlrtRSInfo g_emlrtRSI = {
    103,                         /* lineNo */
    "JIPDATracker/JIPDATracker", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo h_emlrtRSI = {
    1,               /* lineNo */
    "System/System", /* fcnName */
    "/MATLAB/toolbox/shared/system/coder/+matlab/+system/+coder/System.p" /* pathName
                                                                           */
};

static emlrtRSInfo j_emlrtRSI =
    {
        1,                           /* lineNo */
        "SystemCore/parenReference", /* fcnName */
        "/MATLAB/toolbox/shared/system/coder/+matlab/+system/+coder/"
        "SystemCore.p" /* pathName */
};

static emlrtRSInfo m_emlrtRSI =
    {
        1,                 /* lineNo */
        "SystemCore/step", /* fcnName */
        "/MATLAB/toolbox/shared/system/coder/+matlab/+system/+coder/"
        "SystemCore.p" /* pathName */
};

static emlrtRSInfo yb_emlrtRSI = {
    23,                                        /* lineNo */
    "TrackEstimator/set.TargetSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo nl_emlrtRSI = {
    246,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo ol_emlrtRSI = {
    248,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo pl_emlrtRSI = {
    252,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo ql_emlrtRSI = {
    254,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo rl_emlrtRSI = {
    258,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo sl_emlrtRSI = {
    260,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo tl_emlrtRSI = {
    264,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo ul_emlrtRSI = {
    266,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo vl_emlrtRSI = {
    270,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo wl_emlrtRSI = {
    272,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo xl_emlrtRSI = {
    276,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo yl_emlrtRSI = {
    278,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo lm_emlrtRSI = {
    302,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo mm_emlrtRSI = {
    305,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo nm_emlrtRSI = {
    308,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo om_emlrtRSI = {
    311,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo pm_emlrtRSI = {
    312,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo qm_emlrtRSI = {
    89,                                               /* lineNo */
    "JIPDATrackInitiator/updateTargetSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo rm_emlrtRSI = {
    99,                                              /* lineNo */
    "JIPDATrackAssigner/updateTargetSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo sm_emlrtRSI = {
    75,                                             /* lineNo */
    "JIPDATrackUpdater/updateTargetSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo tm_emlrtRSI = {
    58,                                                /* lineNo */
    "JIPDATrackMaintainer/updateTargetSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pathName */
};

static emlrtRSInfo um_emlrtRSI = {
    53,                                                /* lineNo */
    "ObjectTrackOutputter/updateTargetSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo vm_emlrtRSI = {
    317,                                        /* lineNo */
    "JIPDATracker/updateConfirmationThreshold", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo wm_emlrtRSI = {
    322,                                    /* lineNo */
    "JIPDATracker/updateDeletionThreshold", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRTEInfo emlrtRTEI =
    {
        1,                 /* lineNo */
        1,                 /* colNo */
        "SystemCore/step", /* fName */
        "/MATLAB/toolbox/shared/system/coder/+matlab/+system/+coder/"
        "SystemCore.p" /* pName */
};

static emlrtRTEInfo nc_emlrtRTEI = {
    5,                   /* lineNo */
    12,                  /* colNo */
    "trackingAlgorithm", /* fName */
    "/MATLAB "
    "Drive/Repositories/SFTTWorkshop_SI_and_T-3/HelperFunctions/"
    "trackingAlgorithm.m" /* pName */
};

/* Function Definitions */
emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

void trackingAlgorithm(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                       const struct0_T dets[1], const cell_4 *targetSpec,
                       const c_fusion_tracker_sensorspecs_Ae *sensorSpec,
                       emxArray_struct1_T *tracks)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  int32_T i;
  boolean_T sensorSpecChange;
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
  /*  trackingAlgorithm Summary of this function goes here */
  /*  Define the tracker as a persistent variable */
  /*  Initialize the tracker on first call using isempty */
  if (!tracker_not_empty) {
    /*  Instantiate tracker */
    st.site = &c_emlrtRSI;
    b_st.site = &d_emlrtRSI;
    tracker.MaxMahalanobisDistance = 5.0;
    tracker.DeletionExistenceProbability = 0.1;
    tracker.MaxNumEvents = rtInf;
    c_st.site = &g_emlrtRSI;
    d_st.site = &h_emlrtRSI;
    e_st.site = &i_emlrtRSI;
    d_st.site = &h_emlrtRSI;
    e_st.site = &j_emlrtRSI;
    tracker.isInitialized = 0;
    e_st.site = &j_emlrtRSI;
    for (i = 0; i < 7; i++) {
      tracker.tunablePropertyChanged[i] = false;
    }
    b_st.site = &e_emlrtRSI;
    c_st.site = &i_emlrtRSI;
    sensorSpecChange = (tracker.isInitialized == 1);
    if (sensorSpecChange) {
      tracker.TunablePropsChanged = true;
      tracker.tunablePropertyChanged[0] = true;
    }
    b_st.site = &e_emlrtRSI;
    tracker.TargetSpecifications = *targetSpec;
    b_st.site = &f_emlrtRSI;
    c_st.site = &l_emlrtRSI;
    sensorSpecChange = (tracker.isInitialized == 1);
    if (sensorSpecChange) {
      tracker.TunablePropsChanged = true;
      tracker.tunablePropertyChanged[1] = true;
    }
    tracker.SensorSpecifications[0] = *sensorSpec;
    tracker_not_empty = true;
    st.site = &emlrtRSI;
    b_st.site = &l_emlrtRSI;
    sensorSpecChange = (tracker.isInitialized == 1);
    if (sensorSpecChange) {
      tracker.TunablePropsChanged = true;
      tracker.tunablePropertyChanged[3] = true;
    }
    st.site = &emlrtRSI;
    tracker.c_ConfirmationExistenceProbabil = 0.95;
    /*  Increased from default because clutter density is high */
  }
  /*  Update the tracker every step using current detections and time stamp */
  st.site = &b_emlrtRSI;
  b_st.site = &j_emlrtRSI;
  if (tracker.isInitialized == 2) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &emlrtRTEI, "MATLAB:system:methodCalledWhenReleasedCodegen",
        "MATLAB:system:methodCalledWhenReleasedCodegen", 3, 4, 4, "step");
  }
  if (tracker.isInitialized != 1) {
    c_st.site = &m_emlrtRSI;
    d_st.site = &j_emlrtRSI;
    if (tracker.isInitialized != 0) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &emlrtRTEI,
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen",
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen", 3, 4, 5,
          "setup");
    }
    tracker.isInitialized = 1;
    e_st.site = &m_emlrtRSI;
    JIPDATracker_setupImpl(SD, &e_st, &tracker);
    tracker.TunablePropsChanged = false;
    d_st.site = &j_emlrtRSI;
    tracker.TrackListManager.InternalTrackList->size[0] = 0;
    tracker.LastTrackID = 0U;
  }
  c_st.site = &j_emlrtRSI;
  if (tracker.TunablePropsChanged) {
    real_T b_val;
    tracker.TunablePropsChanged = false;
    d_st.site = &m_emlrtRSI;
    e_st.site = &nl_emlrtRSI;
    sensorSpecChange = tracker.tunablePropertyChanged[1];
    if (sensorSpecChange) {
      e_st.site = &ol_emlrtRSI;
      c_JIPDATracker_updateSensorSpec(SD, &e_st, &tracker);
    }
    e_st.site = &pl_emlrtRSI;
    sensorSpecChange = tracker.tunablePropertyChanged[0];
    if (sensorSpecChange) {
      cell_4 val;
      e_st.site = &ql_emlrtRSI;
      f_st.site = &lm_emlrtRSI;
      SD->f14.e_obj = tracker.Initiator[0];
      val = tracker.TargetSpecifications;
      SD->f14.e_obj.TargetSpecifications = val;
      g_st.site = &qm_emlrtRSI;
      h_st.site = &yb_emlrtRSI;
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[0] =
          val.f1.StateTransitionModel.PropVelocityMean[0];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[1] =
          val.f1.StateTransitionModel.PropVelocityMean[1];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[2] =
          val.f1.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.e_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f1.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.e_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f1.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.SurvivalModel = val.f1.SurvivalModel;
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[0] = val.f1.IsLockedDataType[0];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[1] = val.f1.IsLockedDataType[1];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[0] =
          val.f2.StateTransitionModel.PropVelocityMean[0];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[1] =
          val.f2.StateTransitionModel.PropVelocityMean[1];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[2] =
          val.f2.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.e_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f2.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.e_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f2.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.SurvivalModel = val.f2.SurvivalModel;
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[0] = val.f2.IsLockedDataType[0];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[1] = val.f2.IsLockedDataType[1];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[0] =
          val.f3.StateTransitionModel.PropVelocityMean[0];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[1] =
          val.f3.StateTransitionModel.PropVelocityMean[1];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[2] =
          val.f3.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.e_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f3.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.e_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f3.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.SurvivalModel = val.f3.SurvivalModel;
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[0] = val.f3.IsLockedDataType[0];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[1] = val.f3.IsLockedDataType[1];
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TargetSpecifications[0] = val.f1;
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TargetSpecifications[0] = val.f2;
      SD->f14.e_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TargetSpecifications[0] = val.f3;
      tracker.Initiator[0] = SD->f14.e_obj;
      f_st.site = &mm_emlrtRSI;
      SD->f14.b_obj = tracker.Assigner[0];
      val = tracker.TargetSpecifications;
      SD->f14.b_obj.TargetSpecifications = val;
      g_st.site = &rm_emlrtRSI;
      h_st.site = &yb_emlrtRSI;
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[0] =
          val.f1.StateTransitionModel.PropVelocityMean[0];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[1] =
          val.f1.StateTransitionModel.PropVelocityMean[1];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[2] =
          val.f1.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.b_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f1.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.b_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f1.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.SurvivalModel = val.f1.SurvivalModel;
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[0] = val.f1.IsLockedDataType[0];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[1] = val.f1.IsLockedDataType[1];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[0] =
          val.f2.StateTransitionModel.PropVelocityMean[0];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[1] =
          val.f2.StateTransitionModel.PropVelocityMean[1];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[2] =
          val.f2.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.b_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f2.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.b_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f2.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.SurvivalModel = val.f2.SurvivalModel;
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[0] = val.f2.IsLockedDataType[0];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[1] = val.f2.IsLockedDataType[1];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[0] =
          val.f3.StateTransitionModel.PropVelocityMean[0];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[1] =
          val.f3.StateTransitionModel.PropVelocityMean[1];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[2] =
          val.f3.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.b_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f3.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.b_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f3.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.SurvivalModel = val.f3.SurvivalModel;
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[0] = val.f3.IsLockedDataType[0];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[1] = val.f3.IsLockedDataType[1];
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TargetSpecifications[0] = val.f1;
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TargetSpecifications[0] = val.f2;
      SD->f14.b_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TargetSpecifications[0] = val.f3;
      tracker.Assigner[0] = SD->f14.b_obj;
      f_st.site = &nm_emlrtRSI;
      SD->f14.d_obj = tracker.Updater[0];
      val = tracker.TargetSpecifications;
      SD->f14.d_obj.TargetSpecifications = val;
      g_st.site = &sm_emlrtRSI;
      h_st.site = &yb_emlrtRSI;
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[0] =
          val.f1.StateTransitionModel.PropVelocityMean[0];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[1] =
          val.f1.StateTransitionModel.PropVelocityMean[1];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[2] =
          val.f1.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.d_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f1.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.d_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f1.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.SurvivalModel = val.f1.SurvivalModel;
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[0] = val.f1.IsLockedDataType[0];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[1] = val.f1.IsLockedDataType[1];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[0] =
          val.f2.StateTransitionModel.PropVelocityMean[0];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[1] =
          val.f2.StateTransitionModel.PropVelocityMean[1];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[2] =
          val.f2.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.d_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f2.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.d_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f2.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.SurvivalModel = val.f2.SurvivalModel;
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[0] = val.f2.IsLockedDataType[0];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[1] = val.f2.IsLockedDataType[1];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[0] =
          val.f3.StateTransitionModel.PropVelocityMean[0];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[1] =
          val.f3.StateTransitionModel.PropVelocityMean[1];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[2] =
          val.f3.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.d_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f3.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.d_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f3.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.SurvivalModel = val.f3.SurvivalModel;
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[0] = val.f3.IsLockedDataType[0];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[1] = val.f3.IsLockedDataType[1];
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TargetSpecifications[0] = val.f1;
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TargetSpecifications[0] = val.f2;
      SD->f14.d_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TargetSpecifications[0] = val.f3;
      tracker.Updater[0] = SD->f14.d_obj;
      tracker.TrackListManager.TargetSpecifications =
          tracker.TargetSpecifications;
      f_st.site = &om_emlrtRSI;
      SD->f14.c_obj = tracker.TrackMaintenance;
      val = tracker.TargetSpecifications;
      SD->f14.c_obj.TargetSpecifications = val;
      g_st.site = &tm_emlrtRSI;
      h_st.site = &yb_emlrtRSI;
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[0] =
          val.f1.StateTransitionModel.PropVelocityMean[0];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[1] =
          val.f1.StateTransitionModel.PropVelocityMean[1];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[2] =
          val.f1.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.c_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f1.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.c_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f1.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.SurvivalModel = val.f1.SurvivalModel;
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[0] = val.f1.IsLockedDataType[0];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[1] = val.f1.IsLockedDataType[1];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[0] =
          val.f2.StateTransitionModel.PropVelocityMean[0];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[1] =
          val.f2.StateTransitionModel.PropVelocityMean[1];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[2] =
          val.f2.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.c_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f2.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.c_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f2.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.SurvivalModel = val.f2.SurvivalModel;
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[0] = val.f2.IsLockedDataType[0];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[1] = val.f2.IsLockedDataType[1];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[0] =
          val.f3.StateTransitionModel.PropVelocityMean[0];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[1] =
          val.f3.StateTransitionModel.PropVelocityMean[1];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[2] =
          val.f3.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.c_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f3.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.c_obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f3.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.SurvivalModel = val.f3.SurvivalModel;
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[0] = val.f3.IsLockedDataType[0];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[1] = val.f3.IsLockedDataType[1];
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TargetSpecifications[0] = val.f1;
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TargetSpecifications[0] = val.f2;
      SD->f14.c_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TargetSpecifications[0] = val.f3;
      tracker.TrackMaintenance = SD->f14.c_obj;
      f_st.site = &pm_emlrtRSI;
      SD->f14.obj = tracker.Outputter;
      val = tracker.TargetSpecifications;
      SD->f14.obj.TargetSpecifications = val;
      g_st.site = &um_emlrtRSI;
      h_st.site = &yb_emlrtRSI;
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[0] =
          val.f1.StateTransitionModel.PropVelocityMean[0];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[1] =
          val.f1.StateTransitionModel.PropVelocityMean[1];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.StateTransitionModel.PropVelocityMean[2] =
          val.f1.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f1.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f1.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f1.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.SurvivalModel = val.f1.SurvivalModel;
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[0] = val.f1.IsLockedDataType[0];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f1.IsLockedDataType[1] = val.f1.IsLockedDataType[1];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[0] =
          val.f2.StateTransitionModel.PropVelocityMean[0];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[1] =
          val.f2.StateTransitionModel.PropVelocityMean[1];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.StateTransitionModel.PropVelocityMean[2] =
          val.f2.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f2.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f2.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f2.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.SurvivalModel = val.f2.SurvivalModel;
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[0] = val.f2.IsLockedDataType[0];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f2.IsLockedDataType[1] = val.f2.IsLockedDataType[1];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[0] =
          val.f3.StateTransitionModel.PropVelocityMean[0];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[1] =
          val.f3.StateTransitionModel.PropVelocityMean[1];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.StateTransitionModel.PropVelocityMean[2] =
          val.f3.StateTransitionModel.PropVelocityMean[2];
      memcpy(&SD->f14.obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropVelocityVariance[0],
             &val.f3.StateTransitionModel.PropVelocityVariance[0],
             9U * sizeof(real_T));
      memcpy(&SD->f14.obj.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications.f3.StateTransitionModel
                  .PropAccelerationVariance[0],
             &val.f3.StateTransitionModel.PropAccelerationVariance[0],
             9U * sizeof(real_T));
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.SurvivalModel = val.f3.SurvivalModel;
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[0] = val.f3.IsLockedDataType[0];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.TargetSpecifications
          .f3.IsLockedDataType[1] = val.f3.IsLockedDataType[1];
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TargetSpecifications[0] = val.f1;
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TargetSpecifications[0] = val.f2;
      SD->f14.obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TargetSpecifications[0] = val.f3;
      tracker.Outputter = SD->f14.obj;
    }
    e_st.site = &rl_emlrtRSI;
    sensorSpecChange = tracker.tunablePropertyChanged[3];
    if (sensorSpecChange) {
      e_st.site = &sl_emlrtRSI;
      f_st.site = &vm_emlrtRSI;
      SD->f14.c_obj = tracker.TrackMaintenance;
      b_val = tracker.c_ConfirmationExistenceProbabil;
      g_st.site = &gl_emlrtRSI;
      h_st.site = &hl_emlrtRSI;
      if (!(b_val > 0.0)) {
        emlrtErrorWithMessageIdR2018a(&h_st, &b_emlrtRTEI,
                                      "MATLAB:validators:mustBePositive",
                                      "MATLAB:validators:mustBePositive", 0);
      }
      h_st.site = &hl_emlrtRSI;
      i_st.site = &il_emlrtRSI;
      if (!(b_val < 1.0)) {
        emlrtErrorWithMessageIdR2018a(
            &i_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
            "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
      }
      SD->f14.c_obj.ConfirmationThreshold = b_val;
      tracker.TrackMaintenance = SD->f14.c_obj;
    }
    e_st.site = &tl_emlrtRSI;
    sensorSpecChange = tracker.tunablePropertyChanged[4];
    if (sensorSpecChange) {
      e_st.site = &ul_emlrtRSI;
      f_st.site = &wm_emlrtRSI;
      SD->f14.c_obj = tracker.TrackMaintenance;
      b_val = tracker.DeletionExistenceProbability;
      g_st.site = &jl_emlrtRSI;
      h_st.site = &kl_emlrtRSI;
      if (!(b_val > 0.0)) {
        emlrtErrorWithMessageIdR2018a(&h_st, &b_emlrtRTEI,
                                      "MATLAB:validators:mustBePositive",
                                      "MATLAB:validators:mustBePositive", 0);
      }
      h_st.site = &kl_emlrtRSI;
      SD->f14.c_obj.DeletionThreshold = b_val;
      tracker.TrackMaintenance = SD->f14.c_obj;
    }
    e_st.site = &vl_emlrtRSI;
    sensorSpecChange = tracker.tunablePropertyChanged[2];
    if (sensorSpecChange) {
      e_st.site = &wl_emlrtRSI;
      tracker.Assigner[0].AssignmentThreshold = tracker.MaxMahalanobisDistance;
      tracker.Updater[0].AssignmentThreshold = tracker.MaxMahalanobisDistance;
    }
    e_st.site = &xl_emlrtRSI;
    sensorSpecChange = tracker.tunablePropertyChanged[5];
    if (sensorSpecChange) {
      e_st.site = &yl_emlrtRSI;
      tracker.Assigner[0].MaxNumEvents = tracker.MaxNumEvents;
    }
    d_st.site = &m_emlrtRSI;
    for (i = 0; i < 7; i++) {
      tracker.tunablePropertyChanged[i] = false;
    }
  }
  c_st.site = &m_emlrtRSI;
  JIPDATracker_stepImpl(
      SD, &c_st, &tracker, dets[0].LookTime.data, dets[0].LookTime.size,
      dets[0].LookAzimuth.data, dets[0].LookAzimuth.size,
      dets[0].LookElevation.data, dets[0].LookElevation.size,
      dets[0].DetectionTime.data, dets[0].DetectionTime.size,
      dets[0].Azimuth.data, dets[0].Azimuth.size, dets[0].Elevation.data,
      dets[0].Elevation.size, dets[0].Range.data, dets[0].Range.size,
      dets[0].RangeRate.data, dets[0].RangeRate.size,
      dets[0].AzimuthAccuracy.data, dets[0].AzimuthAccuracy.size,
      dets[0].ElevationAccuracy.data, dets[0].ElevationAccuracy.size,
      dets[0].RangeAccuracy.data, dets[0].RangeAccuracy.size,
      dets[0].RangeRateAccuracy.data, dets[0].RangeRateAccuracy.size, tracks);
}

void trackingAlgorithm_emx_free(const emlrtStack *sp)
{
  d_emxFreeStruct_fusion_tracker_(sp, &tracker);
}

void trackingAlgorithm_emx_init(const emlrtStack *sp)
{
  d_emxInitStruct_fusion_tracker_(sp, &tracker, &nc_emlrtRTEI);
}

void trackingAlgorithm_init(void)
{
  tracker_not_empty = false;
}

/* End of code generation (trackingAlgorithm.c) */
