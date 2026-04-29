/*
 * IPDAEstimator.c
 *
 * Code generation for function 'IPDAEstimator'
 *
 */

/* Include files */
#include "IPDAEstimator.h"
#include "AerospaceMonostaticRadar.h"
#include "EKFStateEstimator.h"
#include "ExtendedKalmanFilter.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "trackingEKF.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo gkb_emlrtRSI = {
    170,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo hkb_emlrtRSI = {
    168,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ikb_emlrtRSI = {
    163,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo jkb_emlrtRSI = {
    157,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo kkb_emlrtRSI = {
    155,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo lkb_emlrtRSI = {
    153,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo mkb_emlrtRSI = {
    145,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo nkb_emlrtRSI = {
    144,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo okb_emlrtRSI = {
    139,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo pkb_emlrtRSI = {
    135,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo qkb_emlrtRSI = {
    128,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo rkb_emlrtRSI = {
    120,                               /* lineNo */
    "IPDAEstimator/correctUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo skb_emlrtRSI = {
    121,                               /* lineNo */
    "IPDAEstimator/correctUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo tkb_emlrtRSI = {
    114,                     /* lineNo */
    "IPDAEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ukb_emlrtRSI = {
    161,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo vkb_emlrtRSI = {
    162,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo wkb_emlrtRSI = {
    167,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo xkb_emlrtRSI = {
    170,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ykb_emlrtRSI = {
    173,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo alb_emlrtRSI = {
    174,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo ulb_emlrtRSI = {
    82,                         /* lineNo */
    "ExistenceEstimator/merge", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\ExistenceEstimator."
    "m" /* pathName */
};

static emlrtRSInfo vlb_emlrtRSI = {
    220,                       /* lineNo */
    "EKFStateEstimator/merge", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo wlb_emlrtRSI = {
    221,                       /* lineNo */
    "EKFStateEstimator/merge", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtBCInfo xj_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    217,                       /* lineNo */
    69,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo yj_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    212,                       /* lineNo */
    69,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo ak_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    212,                       /* lineNo */
    77,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo bk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    211,                       /* lineNo */
    49,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo ck_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    211,                       /* lineNo */
    57,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtBCInfo dk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    135,                         /* lineNo */
    46,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo ek_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    139,                         /* lineNo */
    71,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo fk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    155,                         /* lineNo */
    79,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo gk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    157,                         /* lineNo */
    86,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo hk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    146,                         /* lineNo */
    13,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo ik_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    152,                         /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo jk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    153,                         /* lineNo */
    65,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo kk_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    80,                         /* lineNo */
    79,                         /* colNo */
    "",                         /* aName */
    "ExistenceEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\ExistenceEstimator."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo lk_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    80,                         /* lineNo */
    87,                         /* colNo */
    "",                         /* aName */
    "ExistenceEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\ExistenceEstimator."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo mk_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    159,                         /* lineNo */
    56,                          /* colNo */
    "",                          /* aName */
    "IPDAEstimator/correctJPDA", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo nk_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    216,                       /* lineNo */
    21,                        /* colNo */
    "",                        /* aName */
    "EKFStateEstimator/merge", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m", /* pName */
    0                                             /* checkKind */
};

static emlrtRTEInfo ml_emlrtRTEI = {
    145,             /* lineNo */
    13,              /* colNo */
    "IPDAEstimator", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pName */
};

static emlrtRTEInfo nl_emlrtRTEI = {
    146,             /* lineNo */
    13,              /* colNo */
    "IPDAEstimator", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pName */
};

static emlrtRTEInfo ol_emlrtRTEI = {
    125,             /* lineNo */
    24,              /* colNo */
    "IPDAEstimator", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pName */
};

/* Function Declarations */
static void IPDAEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Targ,
    trackingEKF *c_estimator_StateEstimator_Trac, struct_T *pdf, real_T dT);

/* Function Definitions */
static void IPDAEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Targ,
    trackingEKF *c_estimator_StateEstimator_Trac, struct_T *pdf, real_T dT)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
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
  if (dT > 0.0) {
    real_T val;
    st.site = &et_emlrtRSI;
    EKFStateEstimator_predict(&st, c_estimator_StateEstimator_Targ,
                              c_estimator_StateEstimator_Trac, pdf, dT);
    st.site = &dt_emlrtRSI;
    b_st.site = &nv_emlrtRSI;
    c_st.site = &ov_emlrtRSI;
    d_st.site = &id_emlrtRSI;
    e_st.site = &jd_emlrtRSI;
    if ((c_estimator_StateEstimator_Targ->SurvivalModel.SurvivalRate < 0.0) &&
        (muDoubleScalarFloor(dT) != dT)) {
      emlrtErrorWithMessageIdR2018a(&e_st, &dc_emlrtRTEI,
                                    "Coder:toolbox:power_domainError",
                                    "Coder:toolbox:power_domainError", 0);
    }
    val = muDoubleScalarPower(
        c_estimator_StateEstimator_Targ->SurvivalModel.SurvivalRate, dT);
    st.site = &dt_emlrtRSI;
    b_st.site = &pv_emlrtRSI;
    c_st.site = &qv_emlrtRSI;
    if (!(val >= 0.0)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &u_emlrtRTEI,
                                    "MATLAB:validators:mustBeNonnegative",
                                    "MATLAB:validators:mustBeNonnegative", 0);
    }
    c_st.site = &qv_emlrtRSI;
    d_st.site = &tk_emlrtRSI;
    if (!(val < 1.0)) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
          "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
    }
    pdf->ExistenceProbability *= val;
  }
}

void IPDAEstimator_correctJPDA(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Targ,
    c_fusion_tracker_sensorspecs_Ae *c_estimator_StateEstimator_Sens,
    trackingEKF *c_estimator_StateEstimator_Trac,
    c_fusion_tracker_sensorspecs_Ae *c_estimator_ExistenceEstimator_,
    struct_T *pdf, const real_T measurements_data[],
    const int32_T measurements_size[2], const real_T dTs_data[],
    const int32_T dTs_size[2], const b_emxArray_struct_T *modelData,
    const real_T assignmentProbs_data[], int32_T assignmentProbs_size,
    real_T gateSize)
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  const b_struct_T *modelData_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_real_T *weights;
  emxArray_struct_T *hypothesis;
  struct_T predPdf;
  struct_T *hypothesis_data;
  real_T b_e[36];
  real_T R[16];
  real_T e[6];
  real_T Pg;
  real_T a;
  real_T *weights_data;
  int32_T b_i;
  int32_T b_idx;
  int32_T c_i;
  int32_T i;
  int32_T idx;
  int32_T ii;
  int32_T k;
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
  modelData_data = modelData->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (modelData->size[0] == 1) {
    st.site = &qkb_emlrtRSI;
    b_st.site = &yp_emlrtRSI;
    c_st.site = &bq_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &c_st, c_estimator_StateEstimator_Sens, modelData_data[0].LookTime.data,
        modelData_data[0].LookTime.size, modelData_data[0].LookAzimuth.data,
        modelData_data[0].LookAzimuth.size,
        modelData_data[0].LookElevation.data,
        modelData_data[0].LookElevation.size,
        modelData_data[0].DetectionTime.data,
        modelData_data[0].DetectionTime.size,
        modelData_data[0].AzimuthNoise.data,
        modelData_data[0].AzimuthNoise.size,
        modelData_data[0].ElevationNoise.data,
        modelData_data[0].ElevationNoise.size,
        modelData_data[0].RangeNoise.data, modelData_data[0].RangeNoise.size,
        modelData_data[0].RangeRateNoise.data,
        modelData_data[0].RangeRateNoise.size);
    c_st.site = &bq_emlrtRSI;
    b_st.site = &aq_emlrtRSI;
    c_st.site = &ns_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &c_st, c_estimator_ExistenceEstimator_, modelData_data[0].LookTime.data,
        modelData_data[0].LookTime.size, modelData_data[0].LookAzimuth.data,
        modelData_data[0].LookAzimuth.size,
        modelData_data[0].LookElevation.data,
        modelData_data[0].LookElevation.size,
        modelData_data[0].DetectionTime.data,
        modelData_data[0].DetectionTime.size,
        modelData_data[0].AzimuthNoise.data,
        modelData_data[0].AzimuthNoise.size,
        modelData_data[0].ElevationNoise.data,
        modelData_data[0].ElevationNoise.size,
        modelData_data[0].RangeNoise.data, modelData_data[0].RangeNoise.size,
        modelData_data[0].RangeRateNoise.data,
        modelData_data[0].RangeRateNoise.size);
  }
  if (modelData->size[0] != 1) {
    int32_T b_iv[2];
    int8_T ii_data;
    if (assignmentProbs_size - 1 < 1) {
      idx = 0;
    } else {
      if ((assignmentProbs_size - 1 < 1) ||
          (assignmentProbs_size - 1 > assignmentProbs_size)) {
        emlrtDynamicBoundsCheckR2012b(assignmentProbs_size - 1, 1,
                                      assignmentProbs_size, &dk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      idx = assignmentProbs_size - 1;
    }
    b_iv[0] = 1;
    b_iv[1] = idx;
    st.site = &pkb_emlrtRSI;
    b_indexShapeCheck(&st, assignmentProbs_size, b_iv);
    st.site = &pkb_emlrtRSI;
    b_st.site = &wbb_emlrtRSI;
    k = (idx >= 1);
    if (k > idx) {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    c_st.site = &xbb_emlrtRSI;
    b_idx = 0;
    d_st.site = &ybb_emlrtRSI;
    ii = 0;
    exitg1 = false;
    while ((!exitg1) && (ii <= idx - 1)) {
      if (assignmentProbs_data[ii] != 0.0) {
        b_idx = 1;
        ii_data = (int8_T)(ii + 1);
        exitg1 = true;
      } else {
        ii++;
      }
    }
    if (b_idx > k) {
      emlrtErrorWithMessageIdR2018a(&c_st, &eb_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (k == 1) {
      if (b_idx == 0) {
        k = 0;
      }
    } else {
      k = (b_idx >= 1);
      b_iv[0] = 1;
      b_iv[1] = k;
      d_st.site = &igb_emlrtRSI;
      b_indexShapeCheck(&d_st, 0, b_iv);
    }
    if (k == 0) {
      ii_data = 1;
    }
    st.site = &okb_emlrtRSI;
    if (ii_data > modelData->size[0]) {
      emlrtDynamicBoundsCheckR2012b(ii_data, 1, modelData->size[0],
                                    &ek_emlrtBCI, &st);
    }
    b_st.site = &yp_emlrtRSI;
    c_st.site = &bq_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &c_st, c_estimator_StateEstimator_Sens,
        modelData_data[ii_data - 1].LookTime.data,
        modelData_data[ii_data - 1].LookTime.size,
        modelData_data[ii_data - 1].LookAzimuth.data,
        modelData_data[ii_data - 1].LookAzimuth.size,
        modelData_data[ii_data - 1].LookElevation.data,
        modelData_data[ii_data - 1].LookElevation.size,
        modelData_data[ii_data - 1].DetectionTime.data,
        modelData_data[ii_data - 1].DetectionTime.size,
        modelData_data[ii_data - 1].AzimuthNoise.data,
        modelData_data[ii_data - 1].AzimuthNoise.size,
        modelData_data[ii_data - 1].ElevationNoise.data,
        modelData_data[ii_data - 1].ElevationNoise.size,
        modelData_data[ii_data - 1].RangeNoise.data,
        modelData_data[ii_data - 1].RangeNoise.size,
        modelData_data[ii_data - 1].RangeRateNoise.data,
        modelData_data[ii_data - 1].RangeRateNoise.size);
    c_st.site = &bq_emlrtRSI;
    b_st.site = &aq_emlrtRSI;
    c_st.site = &ns_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &c_st, c_estimator_ExistenceEstimator_,
        modelData_data[ii_data - 1].LookTime.data,
        modelData_data[ii_data - 1].LookTime.size,
        modelData_data[ii_data - 1].LookAzimuth.data,
        modelData_data[ii_data - 1].LookAzimuth.size,
        modelData_data[ii_data - 1].LookElevation.data,
        modelData_data[ii_data - 1].LookElevation.size,
        modelData_data[ii_data - 1].DetectionTime.data,
        modelData_data[ii_data - 1].DetectionTime.size,
        modelData_data[ii_data - 1].AzimuthNoise.data,
        modelData_data[ii_data - 1].AzimuthNoise.size,
        modelData_data[ii_data - 1].ElevationNoise.data,
        modelData_data[ii_data - 1].ElevationNoise.size,
        modelData_data[ii_data - 1].RangeNoise.data,
        modelData_data[ii_data - 1].RangeNoise.size,
        modelData_data[ii_data - 1].RangeRateNoise.data,
        modelData_data[ii_data - 1].RangeRateNoise.size);
  }
  st.site = &nkb_emlrtRSI;
  b_st.site = &bn_emlrtRSI;
  c_st.site = &cn_emlrtRSI;
  d_st.site = &dn_emlrtRSI;
  if (dTs_size[1] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &xb_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  b_idx = dTs_size[1];
  if (dTs_size[1] <= 2) {
    if (dTs_size[1] == 1) {
      Pg = dTs_data[0];
    } else if ((dTs_data[0] < dTs_data[1]) ||
               (muDoubleScalarIsNaN(dTs_data[0]) &&
                (!muDoubleScalarIsNaN(dTs_data[1])))) {
      Pg = dTs_data[1];
    } else {
      Pg = dTs_data[0];
    }
  } else {
    if (!muDoubleScalarIsNaN(dTs_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= b_idx)) {
        if (!muDoubleScalarIsNaN(dTs_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      Pg = dTs_data[0];
    } else {
      Pg = dTs_data[idx - 1];
      idx++;
      for (i = idx; i <= b_idx; i++) {
        a = dTs_data[i - 1];
        if (Pg < a) {
          Pg = a;
        }
      }
    }
  }
  predPdf = *pdf;
  st.site = &nkb_emlrtRSI;
  IPDAEstimator_predict(&st, c_estimator_StateEstimator_Targ,
                        c_estimator_StateEstimator_Trac, &predPdf, Pg);
  st.site = &mkb_emlrtRSI;
  b_st.site = &rkb_emlrtRSI;
  Pg = c_EKFStateEstimator_gateProbabi(&b_st, gateSize);
  b_st.site = &skb_emlrtRSI;
  c_st.site = &skb_emlrtRSI;
  Pg *= c_EKFStateEstimator_detectionPr(&c_st, c_estimator_StateEstimator_Sens,
                                        predPdf.State);
  c_st.site = &nw_emlrtRSI;
  d_st.site = &ow_emlrtRSI;
  if (!(Pg >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &u_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  d_st.site = &ow_emlrtRSI;
  e_st.site = &tk_emlrtRSI;
  if (!(Pg < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  predPdf.ExistenceProbability = (1.0 - Pg) * predPdf.ExistenceProbability /
                                 (1.0 - Pg * predPdf.ExistenceProbability);
  emxInit_struct_T(sp, &hypothesis, &ml_emlrtRTEI, true);
  idx = hypothesis->size[0];
  hypothesis->size[0] = 1;
  emxEnsureCapacity_struct_T(sp, hypothesis, idx, &ml_emlrtRTEI);
  hypothesis_data = hypothesis->data;
  hypothesis_data[0] = predPdf;
  emxInit_real_T(sp, &weights, 1, &nl_emlrtRTEI, true);
  idx = weights->size[0];
  weights->size[0] = 1;
  emxEnsureCapacity_real_T(sp, weights, idx, &nl_emlrtRTEI);
  weights_data = weights->data;
  if (assignmentProbs_size < 1) {
    emlrtDynamicBoundsCheckR2012b(assignmentProbs_size, 1, assignmentProbs_size,
                                  &hk_emlrtBCI, (emlrtConstCTX)sp);
  }
  weights_data[0] = assignmentProbs_data[assignmentProbs_size - 1];
  for (b_i = 0; b_i <= assignmentProbs_size - 2; b_i++) {
    if (b_i + 1 > assignmentProbs_size) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, assignmentProbs_size,
                                    &ik_emlrtBCI, (emlrtConstCTX)sp);
    }
    Pg = assignmentProbs_data[b_i];
    if (Pg > 0.0) {
      predPdf = *pdf;
      if (b_i + 1 > b_idx) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, b_idx, &jk_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      st.site = &lkb_emlrtRSI;
      IPDAEstimator_predict(&st, c_estimator_StateEstimator_Targ,
                            c_estimator_StateEstimator_Trac, &predPdf,
                            dTs_data[b_i]);
      if (modelData->size[0] != 1) {
        st.site = &kkb_emlrtRSI;
        if (b_i + 1 > modelData->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, modelData->size[0],
                                        &fk_emlrtBCI, &st);
        }
        b_st.site = &yp_emlrtRSI;
        c_st.site = &bq_emlrtRSI;
        c_AerospaceMonostaticRadar_upda(
            &c_st, c_estimator_StateEstimator_Sens,
            modelData_data[b_i].LookTime.data,
            modelData_data[b_i].LookTime.size,
            modelData_data[b_i].LookAzimuth.data,
            modelData_data[b_i].LookAzimuth.size,
            modelData_data[b_i].LookElevation.data,
            modelData_data[b_i].LookElevation.size,
            modelData_data[b_i].DetectionTime.data,
            modelData_data[b_i].DetectionTime.size,
            modelData_data[b_i].AzimuthNoise.data,
            modelData_data[b_i].AzimuthNoise.size,
            modelData_data[b_i].ElevationNoise.data,
            modelData_data[b_i].ElevationNoise.size,
            modelData_data[b_i].RangeNoise.data,
            modelData_data[b_i].RangeNoise.size,
            modelData_data[b_i].RangeRateNoise.data,
            modelData_data[b_i].RangeRateNoise.size);
        c_st.site = &bq_emlrtRSI;
        b_st.site = &aq_emlrtRSI;
        c_st.site = &ns_emlrtRSI;
        c_AerospaceMonostaticRadar_upda(
            &c_st, c_estimator_ExistenceEstimator_,
            modelData_data[b_i].LookTime.data,
            modelData_data[b_i].LookTime.size,
            modelData_data[b_i].LookAzimuth.data,
            modelData_data[b_i].LookAzimuth.size,
            modelData_data[b_i].LookElevation.data,
            modelData_data[b_i].LookElevation.size,
            modelData_data[b_i].DetectionTime.data,
            modelData_data[b_i].DetectionTime.size,
            modelData_data[b_i].AzimuthNoise.data,
            modelData_data[b_i].AzimuthNoise.size,
            modelData_data[b_i].ElevationNoise.data,
            modelData_data[b_i].ElevationNoise.size,
            modelData_data[b_i].RangeNoise.data,
            modelData_data[b_i].RangeNoise.size,
            modelData_data[b_i].RangeRateNoise.data,
            modelData_data[b_i].RangeRateNoise.size);
      }
      st.site = &jkb_emlrtRSI;
      if (b_i + 1 > measurements_size[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, measurements_size[1],
                                      &gk_emlrtBCI, &st);
      }
      b_st.site = &tkb_emlrtRSI;
      c_st.site = &ukb_emlrtRSI;
      ExtendedKalmanFilter_set_State(&c_st, c_estimator_StateEstimator_Trac,
                                     predPdf.State);
      c_st.site = &vkb_emlrtRSI;
      c_ExtendedKalmanFilter_set_Stat(&c_st, c_estimator_StateEstimator_Trac,
                                      predPdf.StateCovariance);
      memset(&R[0], 0, 16U * sizeof(real_T));
      R[0] = c_estimator_StateEstimator_Sens->MeasurementModel.AzimuthVariance;
      R[5] =
          c_estimator_StateEstimator_Sens->MeasurementModel.ElevationVariance;
      R[10] = c_estimator_StateEstimator_Sens->MeasurementModel.RangeVariance;
      R[15] =
          c_estimator_StateEstimator_Sens->MeasurementModel.RangeRateVariance;
      c_st.site = &wkb_emlrtRSI;
      c_ExtendedKalmanFilter_set_Meas(&c_st, c_estimator_StateEstimator_Trac,
                                      R);
      c_st.site = &xkb_emlrtRSI;
      trackingEKF_correct(
          &c_st, c_estimator_StateEstimator_Trac, &measurements_data[4 * b_i],
          c_estimator_StateEstimator_Sens->MeasurementModel.OriginPosition,
          c_estimator_StateEstimator_Sens->MeasurementModel.OriginVelocity,
          c_estimator_StateEstimator_Sens->MeasurementModel.Orientation);
      c_st.site = &ykb_emlrtRSI;
      for (i = 0; i < 6; i++) {
        predPdf.State[i] = c_estimator_StateEstimator_Trac->pState[i];
      }
      c_st.site = &alb_emlrtRSI;
      d_st.site = &sj_emlrtRSI;
      if ((!c_estimator_StateEstimator_Trac->pIsSetStateCovariance) ||
          (c_estimator_StateEstimator_Trac->pSqrtStateCovarianceScalar !=
           -1.0)) {
        a = c_estimator_StateEstimator_Trac->pSqrtStateCovarianceScalar;
        for (i = 0; i < 36; i++) {
          c_estimator_StateEstimator_Trac->pSqrtStateCovariance[i] =
              a * (real_T)iv[i];
        }
        c_estimator_StateEstimator_Trac->pIsSetStateCovariance = true;
        c_estimator_StateEstimator_Trac->pSqrtStateCovarianceScalar = -1.0;
      }
      d_st.site = &tj_emlrtRSI;
      for (c_i = 0; c_i < 6; c_i++) {
        for (i = 0; i < 6; i++) {
          predPdf.StateCovariance[i + 6 * c_i] = 0.0;
        }
        idx = 6 * c_i + 2;
        k = 6 * c_i + 4;
        for (i = 0; i < 6; i++) {
          r = _mm_loadu_pd(
              &c_estimator_StateEstimator_Trac->pSqrtStateCovariance[6 * i]);
          r1 = _mm_loadu_pd(&predPdf.StateCovariance[6 * c_i]);
          r2 = _mm_set1_pd(c_estimator_StateEstimator_Trac
                               ->pSqrtStateCovariance[c_i + 6 * i]);
          _mm_storeu_pd(&predPdf.StateCovariance[6 * c_i],
                        _mm_add_pd(r1, _mm_mul_pd(r, r2)));
          r = _mm_loadu_pd(&c_estimator_StateEstimator_Trac
                                ->pSqrtStateCovariance[6 * i + 2]);
          r1 = _mm_loadu_pd(&predPdf.StateCovariance[idx]);
          _mm_storeu_pd(&predPdf.StateCovariance[idx],
                        _mm_add_pd(r1, _mm_mul_pd(r, r2)));
          r = _mm_loadu_pd(&c_estimator_StateEstimator_Trac
                                ->pSqrtStateCovariance[6 * i + 4]);
          r1 = _mm_loadu_pd(&predPdf.StateCovariance[k]);
          _mm_storeu_pd(&predPdf.StateCovariance[k],
                        _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        }
      }
      predPdf.ExistenceProbability = 1.0;
      idx = hypothesis->size[0];
      k = hypothesis->size[0];
      hypothesis->size[0]++;
      emxEnsureCapacity_struct_T(sp, hypothesis, k, &ol_emlrtRTEI);
      hypothesis_data = hypothesis->data;
      hypothesis_data[idx] = predPdf;
      idx = weights->size[0];
      k = weights->size[0];
      weights->size[0]++;
      emxEnsureCapacity_real_T(sp, weights, k, &ol_emlrtRTEI);
      weights_data = weights->data;
      if (b_i + 1 > assignmentProbs_size) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, assignmentProbs_size,
                                      &mk_emlrtBCI, (emlrtConstCTX)sp);
      }
      weights_data[idx] = Pg;
    }
  }
  st.site = &ikb_emlrtRSI;
  Pg = weights_data[0] * hypothesis_data[0].ExistenceProbability;
  pdf->ExistenceProbability = Pg;
  ii = hypothesis->size[0];
  for (i = 0; i <= ii - 2; i++) {
    if (i + 2 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, weights->size[0], &kk_emlrtBCI,
                                    &st);
    }
    if (i + 2 > ii) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, ii, &lk_emlrtBCI, &st);
    }
    pdf->ExistenceProbability +=
        weights_data[i + 1] * hypothesis_data[i + 1].ExistenceProbability;
  }
  b_st.site = &ulb_emlrtRSI;
  pdf->ExistenceProbability /= c_sum(&b_st, weights);
  weights_data[0] = Pg;
  st.site = &hkb_emlrtRSI;
  Pg = c_sum(&st, weights);
  idx = weights->size[0];
  k = (weights->size[0] / 2) << 1;
  b_idx = k - 2;
  for (i = 0; i <= b_idx; i += 2) {
    r = _mm_loadu_pd(&weights_data[i]);
    _mm_storeu_pd(&weights_data[i], _mm_div_pd(r, _mm_set1_pd(Pg)));
  }
  for (i = k; i < idx; i++) {
    weights_data[i] /= Pg;
  }
  st.site = &gkb_emlrtRSI;
  r = _mm_loadu_pd(&hypothesis_data[0].State[0]);
  r1 = _mm_set1_pd(weights_data[0]);
  _mm_storeu_pd(&pdf->State[0], _mm_mul_pd(r1, r));
  r = _mm_loadu_pd(&hypothesis_data[0].State[2]);
  _mm_storeu_pd(&pdf->State[2], _mm_mul_pd(r1, r));
  r = _mm_loadu_pd(&hypothesis_data[0].State[4]);
  _mm_storeu_pd(&pdf->State[4], _mm_mul_pd(r1, r));
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&hypothesis_data[0].StateCovariance[i]);
    _mm_storeu_pd(&pdf->StateCovariance[i], _mm_mul_pd(r1, r));
  }
  for (i = 0; i <= ii - 2; i++) {
    if (i + 2 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, weights->size[0], &bk_emlrtBCI,
                                    &st);
    }
    if (i + 2 > ii) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, ii, &ck_emlrtBCI, &st);
    }
    r = _mm_loadu_pd(&hypothesis_data[i + 1].State[0]);
    r1 = _mm_loadu_pd(&pdf->State[0]);
    Pg = weights_data[i + 1];
    r2 = _mm_set1_pd(Pg);
    _mm_storeu_pd(&pdf->State[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
    r = _mm_loadu_pd(&hypothesis_data[i + 1].State[2]);
    r1 = _mm_loadu_pd(&pdf->State[2]);
    _mm_storeu_pd(&pdf->State[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
    r = _mm_loadu_pd(&hypothesis_data[i + 1].State[4]);
    r1 = _mm_loadu_pd(&pdf->State[4]);
    _mm_storeu_pd(&pdf->State[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
    if (i + 2 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, weights->size[0], &yj_emlrtBCI,
                                    &st);
    }
    if (i + 2 > ii) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, ii, &ak_emlrtBCI, &st);
    }
    for (c_i = 0; c_i < 36; c_i++) {
      pdf->StateCovariance[c_i] +=
          Pg * hypothesis_data[i + 1].StateCovariance[c_i];
    }
  }
  for (c_i = 0; c_i < ii; c_i++) {
    if (c_i + 1 > ii) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, ii, &nk_emlrtBCI, &st);
    }
    r = _mm_loadu_pd(&hypothesis_data[c_i].State[0]);
    r1 = _mm_loadu_pd(&pdf->State[0]);
    _mm_storeu_pd(&e[0], _mm_sub_pd(r, r1));
    if (c_i + 1 > ii) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, ii, &nk_emlrtBCI, &st);
    }
    r = _mm_loadu_pd(&hypothesis_data[c_i].State[2]);
    r1 = _mm_loadu_pd(&pdf->State[2]);
    _mm_storeu_pd(&e[2], _mm_sub_pd(r, r1));
    if (c_i + 1 > ii) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, ii, &nk_emlrtBCI, &st);
    }
    r = _mm_loadu_pd(&hypothesis_data[c_i].State[4]);
    r1 = _mm_loadu_pd(&pdf->State[4]);
    _mm_storeu_pd(&e[4], _mm_sub_pd(r, r1));
    if (c_i + 1 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, weights->size[0], &xj_emlrtBCI,
                                    &st);
    }
    for (i = 0; i < 6; i++) {
      r = _mm_loadu_pd(&e[0]);
      r1 = _mm_set1_pd(e[i]);
      _mm_storeu_pd(&b_e[6 * i], _mm_mul_pd(r, r1));
      r = _mm_loadu_pd(&e[2]);
      _mm_storeu_pd(&b_e[6 * i + 2], _mm_mul_pd(r, r1));
      r = _mm_loadu_pd(&e[4]);
      _mm_storeu_pd(&b_e[6 * i + 4], _mm_mul_pd(r, r1));
    }
    for (i = 0; i < 36; i++) {
      pdf->StateCovariance[i] += weights_data[c_i] * b_e[i];
    }
  }
  emxFree_struct_T(&st, &hypothesis);
  b_st.site = &vlb_emlrtRSI;
  Pg = c_sum(&b_st, weights);
  r = _mm_loadu_pd(&pdf->State[0]);
  r1 = _mm_set1_pd(Pg);
  _mm_storeu_pd(&pdf->State[0], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&pdf->State[2]);
  _mm_storeu_pd(&pdf->State[2], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&pdf->State[4]);
  _mm_storeu_pd(&pdf->State[4], _mm_div_pd(r, r1));
  b_st.site = &wlb_emlrtRSI;
  Pg = c_sum(&b_st, weights);
  emxFree_real_T(&st, &weights);
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&pdf->StateCovariance[i]);
    r = _mm_div_pd(r, _mm_set1_pd(Pg));
    _mm_storeu_pd(&pdf->StateCovariance[i], r);
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (IPDAEstimator.c) */
