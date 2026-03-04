/*
 * ObjectTrackOutputter.c
 *
 * Code generation for function 'ObjectTrackOutputter'
 *
 */

/* Include files */
#include "ObjectTrackOutputter.h"
#include "IPDAEstimator.h"
#include "TrackEstimator1.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "trackEstimator.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo lk_emlrtRSI = {
    36,                           /* lineNo */
    "ObjectTrackOutputter/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo mk_emlrtRSI = {
    39,                           /* lineNo */
    "ObjectTrackOutputter/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo nk_emlrtRSI = {
    33,                           /* lineNo */
    "ObjectTrackOutputter/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo ok_emlrtRSI = {
    255,                                                      /* lineNo */
    "objectTrack/objectTrack",                                /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo pk_emlrtRSI = {
    524,                                                      /* lineNo */
    "objectTrack/setProperties",                              /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo qk_emlrtRSI = {
    492,                                                      /* lineNo */
    "objectTrack/setProperties",                              /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo rk_emlrtRSI = {
    488,                                                      /* lineNo */
    "objectTrack/setProperties",                              /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo sk_emlrtRSI = {
    487,                                                      /* lineNo */
    "objectTrack/setProperties",                              /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo tk_emlrtRSI = {
    485,                                                      /* lineNo */
    "objectTrack/setProperties",                              /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo uk_emlrtRSI = {
    277,                                                      /* lineNo */
    "objectTrack/set.UpdateTime",                             /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo vk_emlrtRSI = {
    287,                                                      /* lineNo */
    "objectTrack/set.State",                                  /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo wk_emlrtRSI = {
    293,                                                      /* lineNo */
    "objectTrack/set.StateCovariance",                        /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo xk_emlrtRSI = {
    295,                                                      /* lineNo */
    "objectTrack/set.StateCovariance",                        /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo bl_emlrtRSI = {
    355,                                                      /* lineNo */
    "objectTrack/set.ObjectClassProbabilities",               /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo cl_emlrtRSI = {
    358,                                                      /* lineNo */
    "objectTrack/set.ObjectClassProbabilities",               /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRSInfo dl_emlrtRSI = {
    324,                                                      /* lineNo */
    "objectTrack/set.TrackLogicState",                        /* fcnName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pathName */
};

static emlrtRTEInfo ec_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+valattr/validatele.m" /* pName */
};

static emlrtRTEInfo fc_emlrtRTEI = {
    359,                                                      /* lineNo */
    17,                                                       /* colNo */
    "objectTrack/set.ObjectClassProbabilities",               /* fName */
    "/MATLAB/toolbox/shared/tracking/fusionlib/objectTrack.m" /* pName */
};

/* Function Definitions */
void ObjectTrackOutputter_setup(trackingAlgorithmStackData *SD,
                                const emlrtStack *sp,
                                i_fusion_tracker_internal_compo *outputter,
                                trackingEKF *iobj_0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  struct_T sampleTrackFromList_Hypothesis[3];
  real_T statePdf_StateCovariance[36];
  real_T statePdf_State[6];
  real_T c_statePdf_ObjectClassProbabili[3];
  real_T sampleTrackFromList_LogWeights[3];
  real_T c_sampleTrackFromList_Existence;
  real_T g_expl_temp;
  int32_T i;
  int32_T k;
  int32_T statePdf_ObjectClassID;
  uint32_T b_expl_temp;
  uint32_T expl_temp;
  char_T f_expl_temp[10];
  boolean_T e_expl_temp[3];
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
  SD->u3.f10.outputter = outputter->SensorSpecifications[0];
  st.site = &nk_emlrtRSI;
  trackEstimator(SD, &st, &outputter->TargetSpecifications.f1,
                 &outputter->TargetSpecifications.f2,
                 &outputter->TargetSpecifications.f3, &SD->u3.f10.outputter,
                 &iobj_0[0], &SD->u3.f10.estimator);
  st.site = &lk_emlrtRSI;
  c_TrackEstimator_sampleDistribu(
      SD->u3.f10.estimator.StateEstimator.StateEstimator.Estimators.f1
          .TrackingFilter,
      SD->u3.f10.estimator.StateEstimator.StateEstimator.Estimators.f2
          .TrackingFilter,
      SD->u3.f10.estimator.StateEstimator.StateEstimator.Estimators.f3
          .TrackingFilter,
      &expl_temp, &b_expl_temp, &c_expl_temp, &d_expl_temp,
      sampleTrackFromList_Hypothesis, sampleTrackFromList_LogWeights,
      e_expl_temp, &c_sampleTrackFromList_Existence);
  statePdf_ObjectClassID = IPDAEstimator_toObjectTrack(
      sampleTrackFromList_Hypothesis, sampleTrackFromList_LogWeights,
      c_sampleTrackFromList_Existence, statePdf_State, statePdf_StateCovariance,
      c_statePdf_ObjectClassProbabili, f_expl_temp, &g_expl_temp);
  st.site = &mk_emlrtRSI;
  b_st.site = &ok_emlrtRSI;
  c_st.site = &tk_emlrtRSI;
  d_st.site = &uk_emlrtRSI;
  c_st.site = &sk_emlrtRSI;
  d_st.site = &vk_emlrtRSI;
  e_st.site = &ge_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(statePdf_State[k])) &&
        (!muDoubleScalarIsNaN(statePdf_State[k]))) {
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
  c_st.site = &rk_emlrtRSI;
  d_st.site = &wk_emlrtRSI;
  e_st.site = &ge_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 36)) {
    if ((!muDoubleScalarIsInf(statePdf_StateCovariance[k])) &&
        (!muDoubleScalarIsNaN(statePdf_StateCovariance[k]))) {
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
  d_st.site = &xk_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&d_st, statePdf_StateCovariance);
  c_st.site = &qk_emlrtRSI;
  d_st.site = &bl_emlrtRSI;
  e_st.site = &ge_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if ((!muDoubleScalarIsInf(c_statePdf_ObjectClassProbabili[k])) &&
        (!muDoubleScalarIsNaN(c_statePdf_ObjectClassProbabili[k]))) {
      k++;
    } else {
      c_expl_temp = false;
      exitg1 = true;
    }
  }
  if (!c_expl_temp) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  e_st.site = &ge_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!(c_statePdf_ObjectClassProbabili[k] < 0.0)) {
      k++;
    } else {
      c_expl_temp = false;
      exitg1 = true;
    }
  }
  if (!c_expl_temp) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &nb_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  e_st.site = &ge_emlrtRSI;
  c_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (c_statePdf_ObjectClassProbabili[k] <= 1.0) {
      k++;
    } else {
      c_expl_temp = false;
      exitg1 = true;
    }
  }
  if (!c_expl_temp) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &ec_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
        "MATLAB:objectTrack:notLessEqual", 9, 4, 24, "ObjectClassProbabilities",
        4, 2, "<=", 4, 1, "1");
  }
  d_st.site = &cl_emlrtRSI;
  if (muDoubleScalarAbs(d_sumColumnB(c_statePdf_ObjectClassProbabili) - 1.0) >
      1.4901161193847656E-8) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &fc_emlrtRTEI,
        "shared_tracking:objectTrack:invalidClassProbabilityVector",
        "shared_tracking:objectTrack:invalidClassProbabilityVector", 3, 4, 24,
        "ObjectClassProbabilities");
  }
  c_st.site = &pk_emlrtRSI;
  d_st.site = &dl_emlrtRSI;
  outputter->SampleTrack.TrackID = 0U;
  outputter->SampleTrack.BranchID = 0U;
  outputter->SampleTrack.SourceIndex = 1U;
  outputter->SampleTrack.Age = 0U;
  outputter->SampleTrack.ObjectClassID = statePdf_ObjectClassID;
  outputter->SampleTrack.ObjectClassProbabilities[0] =
      c_statePdf_ObjectClassProbabili[0];
  outputter->SampleTrack.ObjectClassProbabilities[1] =
      c_statePdf_ObjectClassProbabili[1];
  outputter->SampleTrack.ObjectClassProbabilities[2] =
      c_statePdf_ObjectClassProbabili[2];
  outputter->SampleTrack.IsConfirmed = false;
  outputter->SampleTrack.IsCoasted = false;
  outputter->SampleTrack.IsSelfReported = true;
  for (i = 0; i < 6; i++) {
    outputter->SampleTrack.pState[i] = statePdf_State[i];
  }
  for (i = 0; i < 36; i++) {
    outputter->SampleTrack.pStateCovariance[i] = statePdf_StateCovariance[i];
  }
  outputter->SampleTrack.pUpdateTime = 0.0;
  outputter->SampleTrack.pTrackLogicState = 0.0;
  outputter->Estimator = SD->u3.f10.estimator;
}

/* End of code generation (ObjectTrackOutputter.c) */
