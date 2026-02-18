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
#include "MultiModalEstimator.h"
#include "find.h"
#include "indexShapeCheck.h"
#include "logsumexp.h"
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
static emlrtRSInfo hlb_emlrtRSI = {
    170,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ilb_emlrtRSI = {
    168,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo jlb_emlrtRSI = {
    163,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo klb_emlrtRSI = {
    157,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo llb_emlrtRSI = {
    155,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo mlb_emlrtRSI = {
    153,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo nlb_emlrtRSI = {
    145,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo olb_emlrtRSI = {
    144,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo plb_emlrtRSI = {
    139,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo qlb_emlrtRSI = {
    135,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo rlb_emlrtRSI = {
    128,                         /* lineNo */
    "IPDAEstimator/correctJPDA", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo slb_emlrtRSI = {
    120,                               /* lineNo */
    "IPDAEstimator/correctUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo tlb_emlrtRSI = {
    121,                               /* lineNo */
    "IPDAEstimator/correctUnassigned", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo ulb_emlrtRSI = {
    114,                     /* lineNo */
    "IPDAEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo vlb_emlrtRSI = {
    193,                           /* lineNo */
    "MultiModalEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo wlb_emlrtRSI = {
    194,                           /* lineNo */
    "MultiModalEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo xlb_emlrtRSI = {
    195,                           /* lineNo */
    "MultiModalEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo ylb_emlrtRSI = {
    202,                           /* lineNo */
    "MultiModalEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo amb_emlrtRSI = {
    203,                           /* lineNo */
    "MultiModalEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo bmb_emlrtRSI = {
    206,                           /* lineNo */
    "MultiModalEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\MultiModalEstimator"
    ".m" /* pathName */
};

static emlrtRSInfo cmb_emlrtRSI = {
    161,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo dmb_emlrtRSI = {
    162,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo emb_emlrtRSI = {
    167,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo fmb_emlrtRSI = {
    170,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo gmb_emlrtRSI = {
    173,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo hmb_emlrtRSI = {
    174,                         /* lineNo */
    "EKFStateEstimator/correct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo cnb_emlrtRSI = {
    82,                         /* lineNo */
    "ExistenceEstimator/merge", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\ExistenceEstimator."
    "m" /* pathName */
};

static emlrtBCInfo yj_emlrtBCI = {
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

static emlrtBCInfo ak_emlrtBCI = {
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

static emlrtBCInfo bk_emlrtBCI = {
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

static emlrtBCInfo ck_emlrtBCI = {
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

static emlrtBCInfo dk_emlrtBCI = {
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

static emlrtBCInfo ek_emlrtBCI = {
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

static emlrtBCInfo fk_emlrtBCI = {
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

static emlrtBCInfo gk_emlrtBCI = {
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

static emlrtBCInfo hk_emlrtBCI = {
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

static emlrtBCInfo ik_emlrtBCI = {
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

static emlrtRTEInfo ll_emlrtRTEI = {
    145,             /* lineNo */
    13,              /* colNo */
    "IPDAEstimator", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pName */
};

static emlrtRTEInfo ml_emlrtRTEI = {
    146,             /* lineNo */
    13,              /* colNo */
    "IPDAEstimator", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pName */
};

static emlrtRTEInfo nl_emlrtRTEI = {
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
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Esti,
    trackingEKF *d_estimator_StateEstimator_Esti,
    const c_fusion_tracker_targetspecs_Ge *e_estimator_StateEstimator_Esti,
    trackingEKF *f_estimator_StateEstimator_Esti,
    const c_fusion_tracker_targetspecs_He *g_estimator_StateEstimator_Esti,
    trackingEKF *h_estimator_StateEstimator_Esti, b_struct_T *pdf, real_T dT);

static void
c_IPDAEstimator_updateEstimator(trackingAlgorithmStackData *SD,
                                const emlrtStack *sp,
                                h_fusion_tracker_internal_estim *estimator,
                                const real_T modelData_LookTime_data[],
                                const int32_T modelData_LookTime_size[2],
                                const real_T modelData_LookAzimuth_data[],
                                const int32_T modelData_LookAzimuth_size[2],
                                const real_T modelData_LookElevation_data[],
                                const int32_T modelData_LookElevation_size[2],
                                const real_T modelData_DetectionTime_data[],
                                const int32_T modelData_DetectionTime_size[2],
                                const real_T modelData_AzimuthNoise_data[],
                                const int32_T modelData_AzimuthNoise_size[2],
                                const real_T modelData_ElevationNoise_data[],
                                const int32_T modelData_ElevationNoise_size[2],
                                const real_T modelData_RangeNoise_data[],
                                const int32_T modelData_RangeNoise_size[2],
                                const real_T modelData_RangeRateNoise_data[],
                                const int32_T modelData_RangeRateNoise_size[2]);

/* Function Definitions */
static void IPDAEstimator_predict(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Esti,
    trackingEKF *d_estimator_StateEstimator_Esti,
    const c_fusion_tracker_targetspecs_Ge *e_estimator_StateEstimator_Esti,
    trackingEKF *f_estimator_StateEstimator_Esti,
    const c_fusion_tracker_targetspecs_He *g_estimator_StateEstimator_Esti,
    trackingEKF *h_estimator_StateEstimator_Esti, b_struct_T *pdf, real_T dT)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  if (dT > 0.0) {
    real_T val;
    st.site = &st_emlrtRSI;
    MultiModalEstimator_predict(
        &st, c_estimator_StateEstimator_Esti, d_estimator_StateEstimator_Esti,
        e_estimator_StateEstimator_Esti, f_estimator_StateEstimator_Esti,
        g_estimator_StateEstimator_Esti, h_estimator_StateEstimator_Esti, pdf,
        dT);
    st.site = &rt_emlrtRSI;
    b_st.site = &rt_emlrtRSI;
    val = c_MultiModalEstimator_survivalP(
        &b_st, c_estimator_StateEstimator_Esti, e_estimator_StateEstimator_Esti,
        g_estimator_StateEstimator_Esti, pdf->LogWeights, pdf->IsValid, dT);
    b_st.site = &xw_emlrtRSI;
    c_st.site = &yw_emlrtRSI;
    if (!(val >= 0.0)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &u_emlrtRTEI,
                                    "MATLAB:validators:mustBeNonnegative",
                                    "MATLAB:validators:mustBeNonnegative", 0);
    }
    c_st.site = &yw_emlrtRSI;
    d_st.site = &hl_emlrtRSI;
    if (!(val < 1.0)) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
          "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
    }
    pdf->ExistenceProbability *= val;
  }
}

static void
c_IPDAEstimator_updateEstimator(trackingAlgorithmStackData *SD,
                                const emlrtStack *sp,
                                h_fusion_tracker_internal_estim *estimator,
                                const real_T modelData_LookTime_data[],
                                const int32_T modelData_LookTime_size[2],
                                const real_T modelData_LookAzimuth_data[],
                                const int32_T modelData_LookAzimuth_size[2],
                                const real_T modelData_LookElevation_data[],
                                const int32_T modelData_LookElevation_size[2],
                                const real_T modelData_DetectionTime_data[],
                                const int32_T modelData_DetectionTime_size[2],
                                const real_T modelData_AzimuthNoise_data[],
                                const int32_T modelData_AzimuthNoise_size[2],
                                const real_T modelData_ElevationNoise_data[],
                                const int32_T modelData_ElevationNoise_size[2],
                                const real_T modelData_RangeNoise_data[],
                                const int32_T modelData_RangeNoise_size[2],
                                const real_T modelData_RangeRateNoise_data[],
                                const int32_T modelData_RangeRateNoise_size[2])
{
  c_fusion_tracker_sensorspecs_Ae val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &mq_emlrtRSI;
  SD->u1.f1.estimator = estimator->StateEstimator;
  b_st.site = &oq_emlrtRSI;
  SD->u1.f1.b_estimator = estimator->StateEstimator.Estimators.f1;
  val = estimator->StateEstimator.Estimators.f1.SensorSpecifications[0];
  c_st.site = &pq_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &c_st, &val, modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  SD->u1.f1.b_estimator.SensorSpecifications[0] = val;
  SD->u1.f1.estimator.Estimators.f1 = SD->u1.f1.b_estimator;
  b_st.site = &oq_emlrtRSI;
  SD->u1.f1.c_estimator = estimator->StateEstimator.Estimators.f2;
  val = estimator->StateEstimator.Estimators.f2.SensorSpecifications[0];
  c_st.site = &pq_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &c_st, &val, modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  SD->u1.f1.c_estimator.SensorSpecifications[0] = val;
  SD->u1.f1.estimator.Estimators.f2 = SD->u1.f1.c_estimator;
  b_st.site = &oq_emlrtRSI;
  SD->u1.f1.d_estimator = estimator->StateEstimator.Estimators.f3;
  val = estimator->StateEstimator.Estimators.f3.SensorSpecifications[0];
  c_st.site = &pq_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &c_st, &val, modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  SD->u1.f1.d_estimator.SensorSpecifications[0] = val;
  SD->u1.f1.estimator.Estimators.f3 = SD->u1.f1.d_estimator;
  estimator->StateEstimator = SD->u1.f1.estimator;
  st.site = &nq_emlrtRSI;
  SD->u1.f1.e_estimator = estimator->ExistenceEstimator;
  SD->u1.f1.e_estimator.SensorSpecifications[0] =
      estimator->ExistenceEstimator.SensorSpecifications[0];
  b_st.site = &ct_emlrtRSI;
  c_AerospaceMonostaticRadar_upda(
      &b_st, &SD->u1.f1.e_estimator.SensorSpecifications[0],
      modelData_LookTime_data, modelData_LookTime_size,
      modelData_LookAzimuth_data, modelData_LookAzimuth_size,
      modelData_LookElevation_data, modelData_LookElevation_size,
      modelData_DetectionTime_data, modelData_DetectionTime_size,
      modelData_AzimuthNoise_data, modelData_AzimuthNoise_size,
      modelData_ElevationNoise_data, modelData_ElevationNoise_size,
      modelData_RangeNoise_data, modelData_RangeNoise_size,
      modelData_RangeRateNoise_data, modelData_RangeRateNoise_size);
  estimator->ExistenceEstimator = SD->u1.f1.e_estimator;
}

void IPDAEstimator_correctJPDA(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    h_fusion_tracker_internal_estim *estimator, b_struct_T *pdf,
    const real_T measurements_data[], const int32_T measurements_size[2],
    const real_T dTs_data[], const int32_T dTs_size[2],
    const b_emxArray_struct_T *modelData, const real_T assignmentProbs_data[],
    int32_T assignmentProbs_size, real_T gateSize)
{
  __m128d r1;
  b_struct_T predPdf;
  b_struct_T *hypothesis_data;
  const c_struct_T *modelData_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_real_T *weights;
  emxArray_struct_T *hypothesis;
  real_T b_assignmentProbs_data[50];
  real_T b_a[36];
  real_T R[16];
  real_T w[3];
  real_T Pg;
  real_T a;
  real_T *weights_data;
  int32_T w_size[2];
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T idx;
  int32_T ii_data;
  int32_T last;
  int8_T tmp_data[3];
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
    SD->u2.f6.r = *estimator;
    st.site = &rlb_emlrtRSI;
    c_IPDAEstimator_updateEstimator(
        SD, &st, &SD->u2.f6.r, modelData_data[0].LookTime.data,
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
    *estimator = SD->u2.f6.r;
  }
  if (modelData->size[0] != 1) {
    if (assignmentProbs_size - 1 < 1) {
      idx = 0;
    } else {
      if ((assignmentProbs_size - 1 < 1) ||
          (assignmentProbs_size - 1 > assignmentProbs_size)) {
        emlrtDynamicBoundsCheckR2012b(assignmentProbs_size - 1, 1,
                                      assignmentProbs_size, &yj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      idx = assignmentProbs_size - 1;
    }
    w_size[0] = 1;
    w_size[1] = idx;
    st.site = &qlb_emlrtRSI;
    b_indexShapeCheck(&st, assignmentProbs_size, w_size);
    st.site = &qlb_emlrtRSI;
    if (idx - 1 >= 0) {
      memcpy(&b_assignmentProbs_data[0], &assignmentProbs_data[0],
             (uint32_T)idx * sizeof(real_T));
    }
    b_st.site = &ldb_emlrtRSI;
    idx = d_eml_find(&b_st, b_assignmentProbs_data, idx, (int32_T *)&ii_data);
    if (idx == 0) {
      ii_data = 1;
    }
    SD->u2.f6.r = *estimator;
    if ((ii_data < 1) || (ii_data > modelData->size[0])) {
      emlrtDynamicBoundsCheckR2012b(ii_data, 1, modelData->size[0],
                                    &bk_emlrtBCI, (emlrtConstCTX)sp);
    }
    st.site = &plb_emlrtRSI;
    c_IPDAEstimator_updateEstimator(
        SD, &st, &SD->u2.f6.r, modelData_data[ii_data - 1].LookTime.data,
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
    *estimator = SD->u2.f6.r;
  }
  st.site = &olb_emlrtRSI;
  b_st.site = &on_emlrtRSI;
  c_st.site = &pn_emlrtRSI;
  d_st.site = &qn_emlrtRSI;
  if (dTs_size[1] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &x_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  last = dTs_size[1];
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
      boolean_T exitg1;
      idx = 0;
      ii_data = 2;
      exitg1 = false;
      while ((!exitg1) && (ii_data <= last)) {
        if (!muDoubleScalarIsNaN(dTs_data[ii_data - 1])) {
          idx = ii_data;
          exitg1 = true;
        } else {
          ii_data++;
        }
      }
    }
    if (idx == 0) {
      Pg = dTs_data[0];
    } else {
      Pg = dTs_data[idx - 1];
      ii_data = idx + 1;
      for (i = ii_data; i <= last; i++) {
        a = dTs_data[i - 1];
        if (Pg < a) {
          Pg = a;
        }
      }
    }
  }
  predPdf = *pdf;
  st.site = &olb_emlrtRSI;
  IPDAEstimator_predict(
      &st, &estimator->StateEstimator.Estimators.f1.TargetSpecifications[0],
      estimator->StateEstimator.Estimators.f1.TrackingFilter,
      &estimator->StateEstimator.Estimators.f2.TargetSpecifications[0],
      estimator->StateEstimator.Estimators.f2.TrackingFilter,
      &estimator->StateEstimator.Estimators.f3.TargetSpecifications[0],
      estimator->StateEstimator.Estimators.f3.TrackingFilter, &predPdf, Pg);
  st.site = &nlb_emlrtRSI;
  b_st.site = &slb_emlrtRSI;
  Pg = c_MultiModalEstimator_gateProba(&b_st, gateSize);
  b_st.site = &tlb_emlrtRSI;
  c_st.site = &tlb_emlrtRSI;
  Pg *= c_MultiModalEstimator_detection(
      &c_st, &estimator->StateEstimator.Estimators.f1.SensorSpecifications[0],
      &estimator->StateEstimator.Estimators.f2.SensorSpecifications[0],
      &estimator->StateEstimator.Estimators.f3.SensorSpecifications[0],
      predPdf.Hypothesis, predPdf.LogWeights, predPdf.IsValid);
  c_st.site = &ay_emlrtRSI;
  d_st.site = &by_emlrtRSI;
  if (!(Pg >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &u_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  d_st.site = &by_emlrtRSI;
  e_st.site = &hl_emlrtRSI;
  if (!(Pg < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  predPdf.ExistenceProbability = (1.0 - Pg) * predPdf.ExistenceProbability /
                                 (1.0 - Pg * predPdf.ExistenceProbability);
  emxInit_struct_T(sp, &hypothesis, &ll_emlrtRTEI, true);
  idx = hypothesis->size[0];
  hypothesis->size[0] = 1;
  emxEnsureCapacity_struct_T(sp, hypothesis, idx, &ll_emlrtRTEI);
  hypothesis_data = hypothesis->data;
  hypothesis_data[0] = predPdf;
  emxInit_real_T(sp, &weights, 1, &ml_emlrtRTEI, true);
  idx = weights->size[0];
  weights->size[0] = 1;
  emxEnsureCapacity_real_T(sp, weights, idx, &ml_emlrtRTEI);
  weights_data = weights->data;
  if (assignmentProbs_size < 1) {
    emlrtDynamicBoundsCheckR2012b(assignmentProbs_size, 1, assignmentProbs_size,
                                  &ck_emlrtBCI, (emlrtConstCTX)sp);
  }
  weights_data[0] = assignmentProbs_data[assignmentProbs_size - 1];
  for (b_i = 0; b_i <= assignmentProbs_size - 2; b_i++) {
    real_T d;
    if (b_i + 1 > assignmentProbs_size) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, assignmentProbs_size,
                                    &dk_emlrtBCI, (emlrtConstCTX)sp);
    }
    d = assignmentProbs_data[b_i];
    if (d > 0.0) {
      __m128d r2;
      __m128d r3;
      real_T dv[36];
      real_T w_data[3];
      boolean_T b;
      boolean_T b1;
      boolean_T b2;
      predPdf = *pdf;
      if (b_i + 1 > last) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, last, &ek_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      st.site = &mlb_emlrtRSI;
      IPDAEstimator_predict(
          &st, &estimator->StateEstimator.Estimators.f1.TargetSpecifications[0],
          estimator->StateEstimator.Estimators.f1.TrackingFilter,
          &estimator->StateEstimator.Estimators.f2.TargetSpecifications[0],
          estimator->StateEstimator.Estimators.f2.TrackingFilter,
          &estimator->StateEstimator.Estimators.f3.TargetSpecifications[0],
          estimator->StateEstimator.Estimators.f3.TrackingFilter, &predPdf,
          dTs_data[b_i]);
      if (modelData->size[0] != 1) {
        SD->u2.f6.r = *estimator;
        if (b_i + 1 > modelData->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, modelData->size[0],
                                        &hk_emlrtBCI, (emlrtConstCTX)sp);
        }
        st.site = &llb_emlrtRSI;
        c_IPDAEstimator_updateEstimator(
            SD, &st, &SD->u2.f6.r, modelData_data[b_i].LookTime.data,
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
        *estimator = SD->u2.f6.r;
      }
      st.site = &klb_emlrtRSI;
      if (b_i + 1 > measurements_size[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, measurements_size[1],
                                      &ak_emlrtBCI, &st);
      }
      b_st.site = &ulb_emlrtRSI;
      if (predPdf.IsValid[0]) {
        c_st.site = &vlb_emlrtRSI;
        Pg = EKFStateEstimator_likelihood(
            &c_st,
            &estimator->StateEstimator.Estimators.f1.SensorSpecifications[0],
            estimator->StateEstimator.Estimators.f1.TrackingFilter,
            predPdf.Hypothesis[0].State, predPdf.Hypothesis[0].StateCovariance,
            &measurements_data[4 * b_i]);
        c_st.site = &wlb_emlrtRSI;
        d_st.site = &cmb_emlrtRSI;
        ExtendedKalmanFilter_set_State(
            &d_st, estimator->StateEstimator.Estimators.f1.TrackingFilter,
            predPdf.Hypothesis[0].State);
        d_st.site = &dmb_emlrtRSI;
        c_ExtendedKalmanFilter_set_Stat(
            &d_st, estimator->StateEstimator.Estimators.f1.TrackingFilter,
            predPdf.Hypothesis[0].StateCovariance);
        memset(&R[0], 0, 16U * sizeof(real_T));
        R[0] = estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                   .MeasurementModel.AzimuthVariance;
        R[5] = estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                   .MeasurementModel.ElevationVariance;
        R[10] = estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                    .MeasurementModel.RangeVariance;
        R[15] = estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                    .MeasurementModel.RangeRateVariance;
        d_st.site = &emb_emlrtRSI;
        c_ExtendedKalmanFilter_set_Meas(
            &d_st, estimator->StateEstimator.Estimators.f1.TrackingFilter, R);
        d_st.site = &fmb_emlrtRSI;
        trackingEKF_correct(
            &d_st, estimator->StateEstimator.Estimators.f1.TrackingFilter,
            &measurements_data[4 * b_i],
            estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                .MeasurementModel.OriginPosition,
            estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                .MeasurementModel.OriginVelocity,
            estimator->StateEstimator.Estimators.f1.SensorSpecifications[0]
                .MeasurementModel.Orientation);
        d_st.site = &gmb_emlrtRSI;
        for (i = 0; i < 6; i++) {
          predPdf.Hypothesis[0].State[i] =
              estimator->StateEstimator.Estimators.f1.TrackingFilter->pState[i];
        }
        d_st.site = &hmb_emlrtRSI;
        if ((!estimator->StateEstimator.Estimators.f1.TrackingFilter
                  ->pIsSetStateCovariance) ||
            (estimator->StateEstimator.Estimators.f1.TrackingFilter
                 ->pSqrtStateCovarianceScalar != -1.0)) {
          a = estimator->StateEstimator.Estimators.f1.TrackingFilter
                  ->pSqrtStateCovarianceScalar;
          for (i = 0; i < 36; i++) {
            estimator->StateEstimator.Estimators.f1.TrackingFilter
                ->pSqrtStateCovariance[i] = a * (real_T)iv[i];
          }
          estimator->StateEstimator.Estimators.f1.TrackingFilter
              ->pIsSetStateCovariance = true;
          estimator->StateEstimator.Estimators.f1.TrackingFilter
              ->pSqrtStateCovarianceScalar = -1.0;
        }
        for (i = 0; i < 36; i++) {
          dv[i] = estimator->StateEstimator.Estimators.f1.TrackingFilter
                      ->pSqrtStateCovariance[i];
        }
        for (i = 0; i < 36; i++) {
          b_a[i] = estimator->StateEstimator.Estimators.f1.TrackingFilter
                       ->pSqrtStateCovariance[i];
        }
        for (c_i = 0; c_i < 6; c_i++) {
          for (i = 0; i < 6; i++) {
            predPdf.Hypothesis[0].StateCovariance[i + 6 * c_i] = 0.0;
          }
          ii_data = 6 * c_i + 2;
          idx = 6 * c_i + 4;
          for (i = 0; i < 6; i++) {
            r1 = _mm_loadu_pd(&b_a[6 * i]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[0].StateCovariance[6 * c_i]);
            r3 = _mm_set1_pd(dv[c_i + 6 * i]);
            _mm_storeu_pd(&predPdf.Hypothesis[0].StateCovariance[6 * c_i],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
            r1 = _mm_loadu_pd(&b_a[6 * i + 2]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[0].StateCovariance[ii_data]);
            _mm_storeu_pd(&predPdf.Hypothesis[0].StateCovariance[ii_data],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
            r1 = _mm_loadu_pd(&b_a[6 * i + 4]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[0].StateCovariance[idx]);
            _mm_storeu_pd(&predPdf.Hypothesis[0].StateCovariance[idx],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
          }
        }
        c_st.site = &xlb_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarLog(Pg);
        w[0] = predPdf.LogWeights[0] + Pg;
      } else {
        w[0] = -1.7976931348623157E+308;
      }
      if (predPdf.IsValid[1]) {
        c_st.site = &vlb_emlrtRSI;
        Pg = EKFStateEstimator_likelihood(
            &c_st,
            &estimator->StateEstimator.Estimators.f2.SensorSpecifications[0],
            estimator->StateEstimator.Estimators.f2.TrackingFilter,
            predPdf.Hypothesis[1].State, predPdf.Hypothesis[1].StateCovariance,
            &measurements_data[4 * b_i]);
        c_st.site = &wlb_emlrtRSI;
        d_st.site = &cmb_emlrtRSI;
        ExtendedKalmanFilter_set_State(
            &d_st, estimator->StateEstimator.Estimators.f2.TrackingFilter,
            predPdf.Hypothesis[1].State);
        d_st.site = &dmb_emlrtRSI;
        c_ExtendedKalmanFilter_set_Stat(
            &d_st, estimator->StateEstimator.Estimators.f2.TrackingFilter,
            predPdf.Hypothesis[1].StateCovariance);
        memset(&R[0], 0, 16U * sizeof(real_T));
        R[0] = estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                   .MeasurementModel.AzimuthVariance;
        R[5] = estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                   .MeasurementModel.ElevationVariance;
        R[10] = estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                    .MeasurementModel.RangeVariance;
        R[15] = estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                    .MeasurementModel.RangeRateVariance;
        d_st.site = &emb_emlrtRSI;
        c_ExtendedKalmanFilter_set_Meas(
            &d_st, estimator->StateEstimator.Estimators.f2.TrackingFilter, R);
        d_st.site = &fmb_emlrtRSI;
        trackingEKF_correct(
            &d_st, estimator->StateEstimator.Estimators.f2.TrackingFilter,
            &measurements_data[4 * b_i],
            estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                .MeasurementModel.OriginPosition,
            estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                .MeasurementModel.OriginVelocity,
            estimator->StateEstimator.Estimators.f2.SensorSpecifications[0]
                .MeasurementModel.Orientation);
        d_st.site = &gmb_emlrtRSI;
        for (i = 0; i < 6; i++) {
          predPdf.Hypothesis[1].State[i] =
              estimator->StateEstimator.Estimators.f2.TrackingFilter->pState[i];
        }
        d_st.site = &hmb_emlrtRSI;
        if ((!estimator->StateEstimator.Estimators.f2.TrackingFilter
                  ->pIsSetStateCovariance) ||
            (estimator->StateEstimator.Estimators.f2.TrackingFilter
                 ->pSqrtStateCovarianceScalar != -1.0)) {
          a = estimator->StateEstimator.Estimators.f2.TrackingFilter
                  ->pSqrtStateCovarianceScalar;
          for (i = 0; i < 36; i++) {
            estimator->StateEstimator.Estimators.f2.TrackingFilter
                ->pSqrtStateCovariance[i] = a * (real_T)iv[i];
          }
          estimator->StateEstimator.Estimators.f2.TrackingFilter
              ->pIsSetStateCovariance = true;
          estimator->StateEstimator.Estimators.f2.TrackingFilter
              ->pSqrtStateCovarianceScalar = -1.0;
        }
        for (i = 0; i < 36; i++) {
          dv[i] = estimator->StateEstimator.Estimators.f2.TrackingFilter
                      ->pSqrtStateCovariance[i];
        }
        for (i = 0; i < 36; i++) {
          b_a[i] = estimator->StateEstimator.Estimators.f2.TrackingFilter
                       ->pSqrtStateCovariance[i];
        }
        for (c_i = 0; c_i < 6; c_i++) {
          for (i = 0; i < 6; i++) {
            predPdf.Hypothesis[1].StateCovariance[i + 6 * c_i] = 0.0;
          }
          ii_data = 6 * c_i + 2;
          idx = 6 * c_i + 4;
          for (i = 0; i < 6; i++) {
            r1 = _mm_loadu_pd(&b_a[6 * i]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[1].StateCovariance[6 * c_i]);
            r3 = _mm_set1_pd(dv[c_i + 6 * i]);
            _mm_storeu_pd(&predPdf.Hypothesis[1].StateCovariance[6 * c_i],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
            r1 = _mm_loadu_pd(&b_a[6 * i + 2]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[1].StateCovariance[ii_data]);
            _mm_storeu_pd(&predPdf.Hypothesis[1].StateCovariance[ii_data],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
            r1 = _mm_loadu_pd(&b_a[6 * i + 4]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[1].StateCovariance[idx]);
            _mm_storeu_pd(&predPdf.Hypothesis[1].StateCovariance[idx],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
          }
        }
        c_st.site = &xlb_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarLog(Pg);
        w[1] = predPdf.LogWeights[1] + Pg;
      } else {
        w[1] = -1.7976931348623157E+308;
      }
      if (predPdf.IsValid[2]) {
        c_st.site = &vlb_emlrtRSI;
        Pg = EKFStateEstimator_likelihood(
            &c_st,
            &estimator->StateEstimator.Estimators.f3.SensorSpecifications[0],
            estimator->StateEstimator.Estimators.f3.TrackingFilter,
            predPdf.Hypothesis[2].State, predPdf.Hypothesis[2].StateCovariance,
            &measurements_data[4 * b_i]);
        c_st.site = &wlb_emlrtRSI;
        d_st.site = &cmb_emlrtRSI;
        ExtendedKalmanFilter_set_State(
            &d_st, estimator->StateEstimator.Estimators.f3.TrackingFilter,
            predPdf.Hypothesis[2].State);
        d_st.site = &dmb_emlrtRSI;
        c_ExtendedKalmanFilter_set_Stat(
            &d_st, estimator->StateEstimator.Estimators.f3.TrackingFilter,
            predPdf.Hypothesis[2].StateCovariance);
        memset(&R[0], 0, 16U * sizeof(real_T));
        R[0] = estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                   .MeasurementModel.AzimuthVariance;
        R[5] = estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                   .MeasurementModel.ElevationVariance;
        R[10] = estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                    .MeasurementModel.RangeVariance;
        R[15] = estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                    .MeasurementModel.RangeRateVariance;
        d_st.site = &emb_emlrtRSI;
        c_ExtendedKalmanFilter_set_Meas(
            &d_st, estimator->StateEstimator.Estimators.f3.TrackingFilter, R);
        d_st.site = &fmb_emlrtRSI;
        trackingEKF_correct(
            &d_st, estimator->StateEstimator.Estimators.f3.TrackingFilter,
            &measurements_data[4 * b_i],
            estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                .MeasurementModel.OriginPosition,
            estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                .MeasurementModel.OriginVelocity,
            estimator->StateEstimator.Estimators.f3.SensorSpecifications[0]
                .MeasurementModel.Orientation);
        d_st.site = &gmb_emlrtRSI;
        for (i = 0; i < 6; i++) {
          predPdf.Hypothesis[2].State[i] =
              estimator->StateEstimator.Estimators.f3.TrackingFilter->pState[i];
        }
        d_st.site = &hmb_emlrtRSI;
        if ((!estimator->StateEstimator.Estimators.f3.TrackingFilter
                  ->pIsSetStateCovariance) ||
            (estimator->StateEstimator.Estimators.f3.TrackingFilter
                 ->pSqrtStateCovarianceScalar != -1.0)) {
          a = estimator->StateEstimator.Estimators.f3.TrackingFilter
                  ->pSqrtStateCovarianceScalar;
          for (i = 0; i < 36; i++) {
            estimator->StateEstimator.Estimators.f3.TrackingFilter
                ->pSqrtStateCovariance[i] = a * (real_T)iv[i];
          }
          estimator->StateEstimator.Estimators.f3.TrackingFilter
              ->pIsSetStateCovariance = true;
          estimator->StateEstimator.Estimators.f3.TrackingFilter
              ->pSqrtStateCovarianceScalar = -1.0;
        }
        for (i = 0; i < 36; i++) {
          dv[i] = estimator->StateEstimator.Estimators.f3.TrackingFilter
                      ->pSqrtStateCovariance[i];
        }
        for (i = 0; i < 36; i++) {
          b_a[i] = estimator->StateEstimator.Estimators.f3.TrackingFilter
                       ->pSqrtStateCovariance[i];
        }
        for (c_i = 0; c_i < 6; c_i++) {
          for (i = 0; i < 6; i++) {
            predPdf.Hypothesis[2].StateCovariance[i + 6 * c_i] = 0.0;
          }
          ii_data = 6 * c_i + 2;
          idx = 6 * c_i + 4;
          for (i = 0; i < 6; i++) {
            r1 = _mm_loadu_pd(&b_a[6 * i]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[2].StateCovariance[6 * c_i]);
            r3 = _mm_set1_pd(dv[c_i + 6 * i]);
            _mm_storeu_pd(&predPdf.Hypothesis[2].StateCovariance[6 * c_i],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
            r1 = _mm_loadu_pd(&b_a[6 * i + 2]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[2].StateCovariance[ii_data]);
            _mm_storeu_pd(&predPdf.Hypothesis[2].StateCovariance[ii_data],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
            r1 = _mm_loadu_pd(&b_a[6 * i + 4]);
            r2 = _mm_loadu_pd(&predPdf.Hypothesis[2].StateCovariance[idx]);
            _mm_storeu_pd(&predPdf.Hypothesis[2].StateCovariance[idx],
                          _mm_add_pd(r2, _mm_mul_pd(r1, r3)));
          }
        }
        c_st.site = &xlb_emlrtRSI;
        if (Pg < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        Pg = muDoubleScalarLog(Pg);
        w[2] = predPdf.LogWeights[2] + Pg;
      } else {
        w[2] = -1.7976931348623157E+308;
      }
      c_st.site = &ylb_emlrtRSI;
      Pg = c_logsumexp(&c_st, w);
      r1 = _mm_loadu_pd(&w[0]);
      _mm_storeu_pd(&w[0], _mm_sub_pd(r1, _mm_set1_pd(Pg)));
      w[2] -= Pg;
      c_st.site = &amb_emlrtRSI;
      if (estimator->StateEstimator.DeletionThreshold < 0.0) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &v_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
            "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
      }
      Pg = muDoubleScalarLog(estimator->StateEstimator.DeletionThreshold);
      idx = 0;
      b = (predPdf.IsValid[0] && (w[0] > Pg));
      if (b) {
        idx = 1;
      }
      b1 = (predPdf.IsValid[1] && (w[1] > Pg));
      if (b1) {
        idx++;
      }
      b2 = (predPdf.IsValid[2] && (w[2] > Pg));
      if (b2) {
        idx++;
      }
      ii_data = 0;
      if (b) {
        tmp_data[0] = 0;
        ii_data = 1;
      }
      if (b1) {
        tmp_data[ii_data] = 1;
        ii_data++;
      }
      if (b2) {
        tmp_data[ii_data] = 2;
      }
      w_size[0] = 1;
      w_size[1] = idx;
      for (i = 0; i < idx; i++) {
        w_data[i] = w[tmp_data[i]];
      }
      c_st.site = &bmb_emlrtRSI;
      a = logsumexp(&c_st, w_data, w_size);
      for (i = 0; i < idx; i++) {
        int8_T i1;
        i1 = tmp_data[i];
        predPdf.LogWeights[i1] = w[i1] - a;
      }
      if ((!predPdf.IsValid[0]) || (!(w[0] > Pg))) {
        predPdf.LogWeights[0] = -1.7976931348623157E+308;
      }
      if ((!predPdf.IsValid[1]) || (!(w[1] > Pg))) {
        predPdf.LogWeights[1] = -1.7976931348623157E+308;
      }
      if ((!predPdf.IsValid[2]) || (!(w[2] > Pg))) {
        predPdf.LogWeights[2] = -1.7976931348623157E+308;
      }
      predPdf.ExistenceProbability = 1.0;
      ii_data = hypothesis->size[0];
      idx = hypothesis->size[0];
      hypothesis->size[0]++;
      emxEnsureCapacity_struct_T(sp, hypothesis, idx, &nl_emlrtRTEI);
      hypothesis_data = hypothesis->data;
      hypothesis_data[ii_data] = predPdf;
      ii_data = weights->size[0];
      idx = weights->size[0];
      weights->size[0]++;
      emxEnsureCapacity_real_T(sp, weights, idx, &nl_emlrtRTEI);
      weights_data = weights->data;
      if (b_i + 1 > assignmentProbs_size) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, assignmentProbs_size,
                                      &ik_emlrtBCI, (emlrtConstCTX)sp);
      }
      weights_data[ii_data] = d;
    }
  }
  st.site = &jlb_emlrtRSI;
  Pg = weights_data[0] * hypothesis_data[0].ExistenceProbability;
  pdf->ExistenceProbability = Pg;
  idx = hypothesis->size[0];
  for (i = 0; i <= idx - 2; i++) {
    if (i + 2 > weights->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, weights->size[0], &fk_emlrtBCI,
                                    &st);
    }
    if (i + 2 > idx) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, idx, &gk_emlrtBCI, &st);
    }
    pdf->ExistenceProbability +=
        weights_data[i + 1] * hypothesis_data[i + 1].ExistenceProbability;
  }
  b_st.site = &cnb_emlrtRSI;
  pdf->ExistenceProbability /= c_sum(&b_st, weights);
  weights_data[0] = Pg;
  st.site = &ilb_emlrtRSI;
  Pg = c_sum(&st, weights);
  ii_data = weights->size[0];
  idx = (weights->size[0] / 2) << 1;
  last = idx - 2;
  for (i = 0; i <= last; i += 2) {
    r1 = _mm_loadu_pd(&weights_data[i]);
    _mm_storeu_pd(&weights_data[i], _mm_div_pd(r1, _mm_set1_pd(Pg)));
  }
  for (i = idx; i < ii_data; i++) {
    weights_data[i] /= Pg;
  }
  st.site = &hlb_emlrtRSI;
  MultiModalEstimator_merge(&st, estimator->StateEstimator.DeletionThreshold,
                            hypothesis, weights, pdf);
  emxFree_real_T(sp, &weights);
  emxFree_struct_T(sp, &hypothesis);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

int32_T IPDAEstimator_toObjectTrack(const struct_T pdf_Hypothesis[3],
                                    const real_T pdf_LogWeights[3],
                                    real_T pdf_ExistenceProbability,
                                    real_T objectTrackPdf_State[6],
                                    real_T objectTrackPdf_StateCovariance[36],
                                    real_T c_objectTrackPdf_ObjectClassPro[3],
                                    char_T objectTrackPdf_TrackLogic[10],
                                    real_T *objectTrackPdf_TrackLogicState)
{
  int32_T i;
  int32_T k;
  int32_T objectTrackPdf_ObjectClassID;
  for (i = 0; i < 10; i++) {
    objectTrackPdf_TrackLogic[i] = cv1[i];
  }
  if (!muDoubleScalarIsNaN(pdf_LogWeights[0])) {
    objectTrackPdf_ObjectClassID = 1;
  } else {
    boolean_T exitg1;
    objectTrackPdf_ObjectClassID = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(pdf_LogWeights[k - 1])) {
        objectTrackPdf_ObjectClassID = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (objectTrackPdf_ObjectClassID == 0) {
    objectTrackPdf_ObjectClassID = 1;
  } else {
    real_T ex;
    ex = pdf_LogWeights[objectTrackPdf_ObjectClassID - 1];
    k = objectTrackPdf_ObjectClassID + 1;
    for (i = k; i < 4; i++) {
      real_T d;
      d = pdf_LogWeights[i - 1];
      if (ex < d) {
        ex = d;
        objectTrackPdf_ObjectClassID = i;
      }
    }
  }
  for (i = 0; i < 6; i++) {
    objectTrackPdf_State[i] = pdf_Hypothesis[2].State[i];
  }
  memcpy(&objectTrackPdf_StateCovariance[0],
         &pdf_Hypothesis[2].StateCovariance[0], 36U * sizeof(real_T));
  if (objectTrackPdf_ObjectClassID == 1) {
    for (i = 0; i < 6; i++) {
      objectTrackPdf_State[i] = pdf_Hypothesis[0].State[i];
    }
    memcpy(&objectTrackPdf_StateCovariance[0],
           &pdf_Hypothesis[0].StateCovariance[0], 36U * sizeof(real_T));
  }
  if (objectTrackPdf_ObjectClassID == 2) {
    for (i = 0; i < 6; i++) {
      objectTrackPdf_State[i] = pdf_Hypothesis[1].State[i];
    }
    memcpy(&objectTrackPdf_StateCovariance[0],
           &pdf_Hypothesis[1].StateCovariance[0], 36U * sizeof(real_T));
  }
  if (objectTrackPdf_ObjectClassID == 3) {
    for (i = 0; i < 6; i++) {
      objectTrackPdf_State[i] = pdf_Hypothesis[2].State[i];
    }
    memcpy(&objectTrackPdf_StateCovariance[0],
           &pdf_Hypothesis[2].StateCovariance[0], 36U * sizeof(real_T));
  }
  c_objectTrackPdf_ObjectClassPro[0] = muDoubleScalarExp(pdf_LogWeights[0]);
  c_objectTrackPdf_ObjectClassPro[1] = muDoubleScalarExp(pdf_LogWeights[1]);
  c_objectTrackPdf_ObjectClassPro[2] = muDoubleScalarExp(pdf_LogWeights[2]);
  *objectTrackPdf_TrackLogicState = pdf_ExistenceProbability;
  return objectTrackPdf_ObjectClassID;
}

/* End of code generation (IPDAEstimator.c) */
