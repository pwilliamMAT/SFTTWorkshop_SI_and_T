/*
 * ObjectTrackOutputter.c
 *
 * Code generation for function 'ObjectTrackOutputter'
 *
 */

/* Include files */
#include "ObjectTrackOutputter.h"
#include "TrackEstimator1.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "rt_nonfinite.h"
#include "trackEstimator.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ak_emlrtRSI = {
    36,                           /* lineNo */
    "ObjectTrackOutputter/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo bk_emlrtRSI = {
    39,                           /* lineNo */
    "ObjectTrackOutputter/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo ck_emlrtRSI = {
    33,                           /* lineNo */
    "ObjectTrackOutputter/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo dk_emlrtRSI =
    {
        255,                       /* lineNo */
        "objectTrack/objectTrack", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ek_emlrtRSI =
    {
        524,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo fk_emlrtRSI =
    {
        488,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo gk_emlrtRSI =
    {
        487,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo hk_emlrtRSI =
    {
        485,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ik_emlrtRSI =
    {
        277,                          /* lineNo */
        "objectTrack/set.UpdateTime", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo jk_emlrtRSI =
    {
        287,                     /* lineNo */
        "objectTrack/set.State", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo kk_emlrtRSI =
    {
        293,                               /* lineNo */
        "objectTrack/set.StateCovariance", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo lk_emlrtRSI =
    {
        295,                               /* lineNo */
        "objectTrack/set.StateCovariance", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ok_emlrtRSI =
    {
        324,                               /* lineNo */
        "objectTrack/set.TrackLogicState", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

/* Function Definitions */
void ObjectTrackOutputter_setup(const emlrtStack *sp,
                                i_fusion_tracker_internal_compo *outputter,
                                trackingEKF *iobj_0)
{
  c_fusion_tracker_sensorspecs_Ae b_outputter;
  c_fusion_tracker_sensorspecs_Ae c_estimator_StateEstimator_Exis;
  c_fusion_tracker_sensorspecs_Ae d_estimator_StateEstimator_Stat;
  c_fusion_tracker_targetspecs_Pa c_estimator_StateEstimator_Stat;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T c_sampleTrackFromList_StateCova[36];
  real_T sampleTrackFromList_State[6];
  real_T d_estimator_StateEstimator_Exis;
  real_T e_estimator_StateEstimator_Exis;
  real_T e_expl_temp;
  int32_T i;
  int32_T k;
  uint32_T b_expl_temp;
  uint32_T expl_temp;
  boolean_T c_expl_temp;
  boolean_T d_expl_temp;
  boolean_T exitg1;
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
  b_outputter = outputter->SensorSpecifications[0];
  st.site = &ck_emlrtRSI;
  iobj_0 = trackEstimator(
      &st, &outputter->TargetSpecifications[0], &b_outputter, iobj_0,
      &c_estimator_StateEstimator_Stat, &d_estimator_StateEstimator_Stat,
      &c_estimator_StateEstimator_Exis);
  st.site = &ak_emlrtRSI;
  c_TrackEstimator_sampleDistribu(
      iobj_0, &expl_temp, &b_expl_temp, &c_expl_temp, &d_expl_temp,
      sampleTrackFromList_State, c_sampleTrackFromList_StateCova, &e_expl_temp);
  st.site = &bk_emlrtRSI;
  b_st.site = &dk_emlrtRSI;
  c_st.site = &hk_emlrtRSI;
  d_st.site = &ik_emlrtRSI;
  c_st.site = &gk_emlrtRSI;
  d_st.site = &jk_emlrtRSI;
  e_st.site = &vd_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(sampleTrackFromList_State[k])) &&
        (!muDoubleScalarIsNaN(sampleTrackFromList_State[k]))) {
      k++;
    } else {
      c_expl_temp = false;
      exitg1 = true;
    }
  }
  if (!c_expl_temp) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 5, "State");
  }
  c_st.site = &fk_emlrtRSI;
  d_st.site = &kk_emlrtRSI;
  e_st.site = &vd_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 36)) {
    if ((!muDoubleScalarIsInf(c_sampleTrackFromList_StateCova[k])) &&
        (!muDoubleScalarIsNaN(c_sampleTrackFromList_StateCova[k]))) {
      k++;
    } else {
      c_expl_temp = false;
      exitg1 = true;
    }
  }
  if (!c_expl_temp) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 15, "StateCovariance");
  }
  d_st.site = &lk_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&d_st, c_sampleTrackFromList_StateCova);
  c_st.site = &ek_emlrtRSI;
  d_st.site = &ok_emlrtRSI;
  outputter->SampleTrack.TrackID = 0U;
  outputter->SampleTrack.BranchID = 0U;
  outputter->SampleTrack.SourceIndex = 1U;
  outputter->SampleTrack.Age = 0U;
  outputter->SampleTrack.ObjectClassID = 0.0;
  outputter->SampleTrack.ObjectClassProbabilities = 1.0;
  outputter->SampleTrack.IsConfirmed = false;
  outputter->SampleTrack.IsCoasted = false;
  outputter->SampleTrack.IsSelfReported = true;
  for (i = 0; i < 6; i++) {
    outputter->SampleTrack.pState[i] = sampleTrackFromList_State[i];
  }
  for (i = 0; i < 36; i++) {
    outputter->SampleTrack.pStateCovariance[i] =
        c_sampleTrackFromList_StateCova[i];
  }
  outputter->SampleTrack.pUpdateTime = 0.0;
  outputter->SampleTrack.pTrackLogicState = 0.0;
  outputter->Estimator.StateEstimator.StateEstimator.TargetSpecifications[0] =
      c_estimator_StateEstimator_Stat;
  outputter->Estimator.StateEstimator.StateEstimator.SensorSpecifications[0] =
      d_estimator_StateEstimator_Stat;
  outputter->Estimator.StateEstimator.StateEstimator.TrackingFilter = iobj_0;
  outputter->Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] = c_estimator_StateEstimator_Exis;
  outputter->Estimator.StateEstimator.ExistenceEstimator.DetectionProbability =
      d_estimator_StateEstimator_Exis;
  outputter->Estimator.StateEstimator.ExistenceEstimator.SurvivalProbability =
      e_estimator_StateEstimator_Exis;
}

/* End of code generation (ObjectTrackOutputter.c) */
