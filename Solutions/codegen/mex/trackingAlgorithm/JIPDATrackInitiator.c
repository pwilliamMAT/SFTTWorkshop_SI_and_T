/*
 * JIPDATrackInitiator.c
 *
 * Code generation for function 'JIPDATrackInitiator'
 *
 */

/* Include files */
#include "JIPDATrackInitiator.h"
#include "EKFStateEstimator.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "stateToMeasurementJacobian.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_mexutil.h"
#include "trackingAlgorithm_types.h"
#include "warning.h"
#include "xzgetrf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fmb_emlrtRSI = {
    137,               /* lineNo */
    "initializeTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo gmb_emlrtRSI = {
    70,                          /* lineNo */
    "TrackEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo hmb_emlrtRSI = {
    83,                         /* lineNo */
    "IPDAEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo imb_emlrtRSI = {
    84,                         /* lineNo */
    "IPDAEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo jmb_emlrtRSI = {
    85,                         /* lineNo */
    "IPDAEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo kmb_emlrtRSI = {
    101,                            /* lineNo */
    "EKFStateEstimator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\EKFStateEstimator.m" /* pathName */
};

static emlrtRSInfo lmb_emlrtRSI = {
    108,                           /* lineNo */
    "ConstantVelocityModel/prior", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "transition\\ConstantVelocityModel.m" /* pathName */
};

static emlrtRSInfo mmb_emlrtRSI = {
    31,    /* lineNo */
    "inv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m" /* pathName
                                                                       */
};

static emlrtRSInfo nmb_emlrtRSI = {
    42,          /* lineNo */
    "checkcond", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m" /* pathName
                                                                       */
};

static emlrtRSInfo omb_emlrtRSI = {
    46,          /* lineNo */
    "checkcond", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m" /* pathName
                                                                       */
};

static emlrtRSInfo pmb_emlrtRSI = {
    12,                   /* lineNo */
    "measurementToState", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\measurementToState.m" /* pathName */
};

static emlrtRSInfo qmb_emlrtRSI = {
    148,                                                         /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/inverseMeasurement", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo rmb_emlrtRSI = {
    16,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+stateInitiator\\GaussianStateIn"
    "itiator.m" /* pathName */
};

static emlrtRSInfo smb_emlrtRSI = {
    20,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+stateInitiator\\GaussianStateIn"
    "itiator.m" /* pathName */
};

static emlrtRSInfo tmb_emlrtRSI = {
    22,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+stateInitiator\\GaussianStateIn"
    "itiator.m" /* pathName */
};

static emlrtRSInfo umb_emlrtRSI = {
    24,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+stateInitiator\\GaussianStateIn"
    "itiator.m" /* pathName */
};

static emlrtRSInfo vmb_emlrtRSI = {
    31,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+stateInitiator\\GaussianStateIn"
    "itiator.m" /* pathName */
};

static emlrtRSInfo wmb_emlrtRSI = {
    35,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+stateInitiator\\GaussianStateIn"
    "itiator.m" /* pathName */
};

/* Function Declarations */
static void c_emlrt_marshallIn(const emlrtStack *sp,
                               const mxArray *a__output_of_sprintf_,
                               const char_T *identifier, char_T y[14]);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y[14]);

static void kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                char_T ret[14]);

/* Function Definitions */
static void c_emlrt_marshallIn(const emlrtStack *sp,
                               const mxArray *a__output_of_sprintf_,
                               const char_T *identifier, char_T y[14])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[14])
{
  kb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, char_T ret[14])
{
  static const int32_T dims[2] = {1, 14};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 14);
  emlrtDestroyArray(&src);
}

void initializeTrack(
    const emlrtStack *sp,
    const c_fusion_tracker_targetspecs_Pa *c_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *d_estimator_StateEstimator_Stat,
    const c_fusion_tracker_sensorspecs_Ae *c_estimator_StateEstimator_Exis,
    struct_T *track, const real_T z[4], real_T b_time)
{
  static const int32_T b_iv[2] = {1, 6};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  __m128d r;
  __m128d r1;
  __m128d r11;
  __m128d r12;
  __m128d r2;
  __m128d r3;
  __m128d r4;
  __m128d r5;
  __m128d r6;
  __m128d r7;
  __m128d r8;
  __m128d r9;
  __m128i r10;
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
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  real_T Istate[36];
  real_T Y[36];
  real_T b_I[36];
  real_T pMotion_Information[36];
  real_T H[24];
  real_T X[24];
  real_T R[16];
  real_T x[9];
  real_T y[9];
  real_T c_I[6];
  real_T pMotion_Mean[6];
  real_T xE[6];
  real_T xEModel[6];
  real_T d_y[3];
  real_T xP[3];
  real_T absx11;
  real_T absx21;
  real_T absx31;
  real_T xP_idx_0;
  real_T xP_idx_1;
  real_T xP_idx_2;
  int32_T ipiv[4];
  int32_T c_i;
  int32_T i;
  int32_T info;
  int32_T itmp;
  int32_T k;
  int32_T p1;
  int32_T p2;
  int32_T p3;
  int8_T In[36];
  int8_T tmp_data[6];
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
  st.site = &fmb_emlrtRSI;
  b_st.site = &gmb_emlrtRSI;
  c_st.site = &hmb_emlrtRSI;
  d_st.site = &kmb_emlrtRSI;
  e_st.site = &rmb_emlrtRSI;
  for (i = 0; i < 6; i++) {
    pMotion_Mean[i] = 0.0;
  }
  pMotion_Mean[1] =
      c_estimator_StateEstimator_Stat->StateTransitionModel.PropVelocityMean[0];
  pMotion_Mean[3] =
      c_estimator_StateEstimator_Stat->StateTransitionModel.PropVelocityMean[1];
  pMotion_Mean[5] =
      c_estimator_StateEstimator_Stat->StateTransitionModel.PropVelocityMean[2];
  memset(&pMotion_Information[0], 0, 36U * sizeof(real_T));
  f_st.site = &lmb_emlrtRSI;
  memcpy(&x[0],
         &c_estimator_StateEstimator_Stat->StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 =
      muDoubleScalarAbs(c_estimator_StateEstimator_Stat->StateTransitionModel
                            .PropVelocityVariance[0]);
  absx21 =
      muDoubleScalarAbs(c_estimator_StateEstimator_Stat->StateTransitionModel
                            .PropVelocityVariance[1]);
  absx31 =
      muDoubleScalarAbs(c_estimator_StateEstimator_Stat->StateTransitionModel
                            .PropVelocityVariance[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[1];
    x[1] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[0];
    x[3] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[4];
    x[4] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[3];
    x[6] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[7];
    x[7] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    x[0] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[2];
    x[2] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[0];
    x[3] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[5];
    x[5] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[3];
    x[6] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[8];
    x[8] = c_estimator_StateEstimator_Stat->StateTransitionModel
               .PropVelocityVariance[6];
  }
  x[1] /= x[0];
  x[2] /= x[0];
  x[4] -= x[1] * x[3];
  x[5] -= x[2] * x[3];
  x[7] -= x[1] * x[6];
  x[8] -= x[2] * x[6];
  if (muDoubleScalarAbs(x[5]) > muDoubleScalarAbs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = x[1];
    x[1] = x[2];
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }
  x[5] /= x[4];
  x[8] -= x[5] * x[7];
  absx11 = (x[1] * x[5] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  y[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  y[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  y[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
  g_st.site = &mmb_emlrtRSI;
  absx11 = b_norm(c_estimator_StateEstimator_Stat->StateTransitionModel
                      .PropVelocityVariance);
  absx21 = b_norm(y);
  absx31 = 1.0 / (absx11 * absx21);
  if ((absx11 == 0.0) || (absx21 == 0.0) || (absx31 == 0.0)) {
    if (!emlrtSetWarningFlag(&g_st)) {
      h_st.site = &nmb_emlrtRSI;
      c_warning(&h_st);
    }
  } else if ((muDoubleScalarIsNaN(absx31) ||
              (absx31 < 2.2204460492503131E-16)) &&
             (!emlrtSetWarningFlag(&g_st))) {
    char_T str[14];
    h_st.site = &omb_emlrtRSI;
    b_y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(&h_st, 6, m, &rfmt[0]);
    emlrtAssign(&b_y, m);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(absx31);
    emlrtAssign(&c_y, m);
    i_st.site = &hnb_emlrtRSI;
    c_emlrt_marshallIn(&i_st, b_sprintf(&i_st, b_y, c_y, &d_emlrtMCI),
                       "<output of sprintf>", str);
    h_st.site = &omb_emlrtRSI;
    e_warning(&h_st, str);
  }
  for (i = 0; i < 3; i++) {
    p1 = 6 * ((i << 1) + 1);
    pMotion_Information[p1 + 1] = y[3 * i];
    pMotion_Information[p1 + 3] = y[3 * i + 1];
    pMotion_Information[p1 + 5] = y[3 * i + 2];
  }
  real_T pz;
  e_st.site = &smb_emlrtRSI;
  f_st.site = &pmb_emlrtRSI;
  absx21 = 0.017453292519943295 * z[0];
  absx11 = 0.017453292519943295 * z[1];
  pz = z[2] * muDoubleScalarSin(absx11);
  absx11 = z[2] * muDoubleScalarCos(absx11);
  absx31 = absx11 * muDoubleScalarCos(absx21);
  absx21 = absx11 * muDoubleScalarSin(absx21);
  g_st.site = &qmb_emlrtRSI;
  h_st.site = &jd_emlrtRSI;
  g_st.site = &qmb_emlrtRSI;
  h_st.site = &jd_emlrtRSI;
  g_st.site = &qmb_emlrtRSI;
  h_st.site = &jd_emlrtRSI;
  g_st.site = &qmb_emlrtRSI;
  absx11 =
      z[3] / muDoubleScalarSqrt((absx31 * absx31 + absx21 * absx21) + pz * pz);
  xP_idx_0 = absx11 * absx31;
  xP_idx_1 = absx11 * absx21;
  xP_idx_2 = absx11 * pz;
  xP[0] = absx31;
  xP[1] = absx21;
  xP[2] = pz;
  for (i = 0; i < 3; i++) {
    for (k = 0; k < 3; k++) {
      p1 = k + 9 * i;
      y[3 * k] =
          d_estimator_StateEstimator_Stat->MeasurementModel.Orientation[p1];
      y[3 * k + 1] =
          d_estimator_StateEstimator_Stat->MeasurementModel.Orientation[p1 + 3];
      y[3 * k + 2] =
          d_estimator_StateEstimator_Stat->MeasurementModel.Orientation[p1 + 6];
    }
    absx11 = xP[0];
    absx21 = xP[1];
    absx31 = xP[2];
    r = _mm_loadu_pd(&y[0]);
    r1 = _mm_mul_pd(r, _mm_set1_pd(absx11));
    r = _mm_loadu_pd(&y[3]);
    r = _mm_mul_pd(r, _mm_set1_pd(absx21));
    r1 = _mm_add_pd(r1, r);
    r = _mm_loadu_pd(&y[6]);
    r = _mm_mul_pd(r, _mm_set1_pd(absx31));
    r = _mm_add_pd(r1, r);
    r1 = _mm_loadu_pd(&d_estimator_StateEstimator_Stat->MeasurementModel
                           .OriginPosition[3 * i]);
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&d_y[0], r);
    d_y[2] = ((y[2] * absx11 + y[5] * absx21) + y[8] * absx31) +
             d_estimator_StateEstimator_Stat->MeasurementModel
                 .OriginPosition[2 + 3 * i];
    r = _mm_loadu_pd(&d_y[0]);
    _mm_storeu_pd(&xP[0], r);
    r = _mm_loadu_pd(&y[0]);
    r1 = _mm_mul_pd(r, _mm_set1_pd(xP_idx_0));
    r = _mm_loadu_pd(&y[3]);
    r = _mm_mul_pd(r, _mm_set1_pd(xP_idx_1));
    r1 = _mm_add_pd(r1, r);
    r = _mm_loadu_pd(&y[6]);
    r = _mm_mul_pd(r, _mm_set1_pd(xP_idx_2));
    r = _mm_add_pd(r1, r);
    r1 = _mm_loadu_pd(&d_estimator_StateEstimator_Stat->MeasurementModel
                           .OriginVelocity[3 * i]);
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&d_y[0], r);
    xP[2] = d_y[2];
    d_y[2] = ((y[2] * xP_idx_0 + y[5] * xP_idx_1) + y[8] * xP_idx_2) +
             d_estimator_StateEstimator_Stat->MeasurementModel
                 .OriginVelocity[2 + 3 * i];
    xP_idx_0 = d_y[0];
    xP_idx_1 = d_y[1];
    xP_idx_2 = d_y[2];
  }
  xE[0] = xP[0];
  xE[1] = xP_idx_0;
  xE[2] = xP[1];
  xE[3] = xP_idx_1;
  xE[4] = xP[2];
  xE[5] = xP_idx_2;
  p2 = 0;
  p1 = 0;
  for (i = 0; i < 6; i++) {
    if (muDoubleScalarIsNaN(xE[i])) {
      p2++;
      tmp_data[p1] = (int8_T)i;
      p1++;
    }
  }
  for (i = 0; i < p2; i++) {
    int8_T b_i;
    b_i = tmp_data[i];
    xE[b_i] = pMotion_Mean[b_i];
  }
  e_st.site = &tmb_emlrtRSI;
  stateToMeasurementJacobian(
      &e_st, xE,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginPosition,
      d_estimator_StateEstimator_Stat->MeasurementModel.OriginVelocity,
      d_estimator_StateEstimator_Stat->MeasurementModel.Orientation, H);
  memset(&R[0], 0, 16U * sizeof(real_T));
  R[0] = d_estimator_StateEstimator_Stat->MeasurementModel.AzimuthVariance;
  R[5] = d_estimator_StateEstimator_Stat->MeasurementModel.ElevationVariance;
  R[10] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeVariance;
  R[15] = d_estimator_StateEstimator_Stat->MeasurementModel.RangeRateVariance;
  e_st.site = &umb_emlrtRSI;
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 6; k++) {
      X[k + 6 * i] = H[i + (k << 2)];
    }
  }
  f_st.site = &wy_emlrtRSI;
  g_st.site = &xy_emlrtRSI;
  h_st.site = &yy_emlrtRSI;
  i_st.site = &aab_emlrtRSI;
  j_st.site = &cab_emlrtRSI;
  k_st.site = &eab_emlrtRSI;
  info = xzgetrf(&k_st, R, ipiv);
  j_st.site = &dab_emlrtRSI;
  for (i = 0; i < 4; i++) {
    p1 = 6 * i - 1;
    p2 = i << 2;
    for (k = 0; k < i; k++) {
      p3 = 6 * k;
      absx11 = R[k + p2];
      if (absx11 != 0.0) {
        for (c_i = 0; c_i < 6; c_i++) {
          itmp = (c_i + p1) + 1;
          X[itmp] -= absx11 * X[c_i + p3];
        }
      }
    }
    r = _mm_loadu_pd(&X[p1 + 1]);
    r1 = _mm_set1_pd(1.0 / R[i + p2]);
    _mm_storeu_pd(&X[p1 + 1], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&X[p1 + 3]);
    _mm_storeu_pd(&X[p1 + 3], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&X[p1 + 5]);
    _mm_storeu_pd(&X[p1 + 5], _mm_mul_pd(r1, r));
  }
  for (i = 3; i >= 0; i--) {
    p1 = 6 * i - 1;
    p2 = (i << 2) - 1;
    p3 = i + 2;
    for (k = p3; k < 5; k++) {
      itmp = 6 * (k - 1);
      absx11 = R[k + p2];
      if (absx11 != 0.0) {
        for (c_i = 0; c_i < 6; c_i++) {
          int32_T X_tmp;
          X_tmp = (c_i + p1) + 1;
          X[X_tmp] -= absx11 * X[c_i + itmp];
        }
      }
    }
  }
  for (i = 2; i >= 0; i--) {
    p1 = ipiv[i];
    if (p1 != i + 1) {
      for (k = 0; k < 6; k++) {
        p2 = k + 6 * i;
        absx11 = X[p2];
        p3 = k + 6 * (p1 - 1);
        X[p2] = X[p3];
        X[p3] = absx11;
      }
    }
  }
  if (info > 0) {
    i_st.site = &bab_emlrtRSI;
    if (!emlrtSetWarningFlag(&i_st)) {
      j_st.site = &lab_emlrtRSI;
      c_warning(&j_st);
    }
  }
  memset(&b_I[0], 0, 36U * sizeof(real_T));
  r2 = _mm_loadu_pd(&pMotion_Information[0]);
  r3 = _mm_loadu_pd(&pMotion_Information[2]);
  r4 = _mm_loadu_pd(&pMotion_Information[4]);
  r5 = _mm_loadu_pd(&pMotion_Information[6]);
  r6 = _mm_loadu_pd(&pMotion_Information[8]);
  r7 = _mm_loadu_pd(&pMotion_Information[10]);
  r8 = _mm_loadu_pd(&pMotion_Information[12]);
  r9 = _mm_loadu_pd(&pMotion_Information[14]);
  r10 = _mm_set1_epi8(0);
  _mm_storeu_si128((__m128i *)&In[0], r10);
  _mm_storeu_si128((__m128i *)&In[16], r10);
  In[32] = 0;
  In[33] = 0;
  In[34] = 0;
  In[35] = 0;
  for (i = 0; i < 6; i++) {
    p1 = 6 * i + 2;
    p2 = 6 * i + 4;
    for (k = 0; k < 4; k++) {
      r = _mm_loadu_pd(&X[6 * k]);
      r1 = _mm_loadu_pd(&b_I[6 * i]);
      r12 = _mm_set1_pd(H[k + (i << 2)]);
      _mm_storeu_pd(&b_I[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
      r = _mm_loadu_pd(&X[6 * k + 2]);
      r1 = _mm_loadu_pd(&b_I[p1]);
      _mm_storeu_pd(&b_I[p1], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
      r = _mm_loadu_pd(&X[6 * k + 4]);
      r1 = _mm_loadu_pd(&b_I[p2]);
      _mm_storeu_pd(&b_I[p2], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    }
    In[i + 6 * i] = 1;
  }
  __m128d r13;
  __m128d r14;
  __m128d r15;
  __m128d r16;
  r = _mm_loadu_pd(&b_I[0]);
  r1 = _mm_loadu_pd(&b_I[2]);
  r11 = _mm_loadu_pd(&b_I[4]);
  r12 = _mm_loadu_pd(&b_I[6]);
  r13 = _mm_loadu_pd(&b_I[8]);
  r14 = _mm_loadu_pd(&b_I[10]);
  r15 = _mm_loadu_pd(&b_I[12]);
  r16 = _mm_loadu_pd(&b_I[14]);
  _mm_storeu_pd(&Istate[0], _mm_add_pd(r, r2));
  _mm_storeu_pd(&Istate[2], _mm_add_pd(r1, r3));
  _mm_storeu_pd(&Istate[4], _mm_add_pd(r11, r4));
  _mm_storeu_pd(&Istate[6], _mm_add_pd(r12, r5));
  _mm_storeu_pd(&Istate[8], _mm_add_pd(r13, r6));
  _mm_storeu_pd(&Istate[10], _mm_add_pd(r14, r7));
  _mm_storeu_pd(&Istate[12], _mm_add_pd(r15, r8));
  _mm_storeu_pd(&Istate[14], _mm_add_pd(r16, r9));
  r = _mm_loadu_pd(&b_I[16]);
  r2 = _mm_loadu_pd(&pMotion_Information[16]);
  r1 = _mm_loadu_pd(&b_I[18]);
  r3 = _mm_loadu_pd(&pMotion_Information[18]);
  r11 = _mm_loadu_pd(&b_I[20]);
  r4 = _mm_loadu_pd(&pMotion_Information[20]);
  r12 = _mm_loadu_pd(&b_I[22]);
  r5 = _mm_loadu_pd(&pMotion_Information[22]);
  r13 = _mm_loadu_pd(&b_I[24]);
  r6 = _mm_loadu_pd(&pMotion_Information[24]);
  r14 = _mm_loadu_pd(&b_I[26]);
  r7 = _mm_loadu_pd(&pMotion_Information[26]);
  r15 = _mm_loadu_pd(&b_I[28]);
  r8 = _mm_loadu_pd(&pMotion_Information[28]);
  r16 = _mm_loadu_pd(&b_I[30]);
  r9 = _mm_loadu_pd(&pMotion_Information[30]);
  _mm_storeu_pd(&Istate[16], _mm_add_pd(r, r2));
  _mm_storeu_pd(&Istate[18], _mm_add_pd(r1, r3));
  _mm_storeu_pd(&Istate[20], _mm_add_pd(r11, r4));
  _mm_storeu_pd(&Istate[22], _mm_add_pd(r12, r5));
  _mm_storeu_pd(&Istate[24], _mm_add_pd(r13, r6));
  _mm_storeu_pd(&Istate[26], _mm_add_pd(r14, r7));
  _mm_storeu_pd(&Istate[28], _mm_add_pd(r15, r8));
  _mm_storeu_pd(&Istate[30], _mm_add_pd(r16, r9));
  Istate[32] = b_I[32] + pMotion_Information[32];
  Istate[33] = b_I[33] + pMotion_Information[33];
  Istate[34] = b_I[34] + pMotion_Information[34];
  Istate[35] = b_I[35] + pMotion_Information[35];
  e_st.site = &vmb_emlrtRSI;
  for (i = 0; i < 36; i++) {
    Y[i] = In[i];
  }
  f_st.site = &wy_emlrtRSI;
  b_mrdiv(&f_st, Y, Istate);
  e_st.site = &wmb_emlrtRSI;
  for (i = 0; i < 36; i++) {
    track->StateCovariance[i] = In[i];
  }
  f_st.site = &wy_emlrtRSI;
  b_mrdiv(&f_st, track->StateCovariance, Istate);
  memset(&c_I[0], 0, 6U * sizeof(real_T));
  memset(&xEModel[0], 0, 6U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    r = _mm_loadu_pd(&b_I[6 * i]);
    r1 = _mm_loadu_pd(&c_I[0]);
    r12 = _mm_set1_pd(xE[i]);
    _mm_storeu_pd(&c_I[0], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&pMotion_Information[6 * i]);
    r1 = _mm_loadu_pd(&xEModel[0]);
    r11 = _mm_set1_pd(pMotion_Mean[i]);
    _mm_storeu_pd(&xEModel[0], _mm_add_pd(r1, _mm_mul_pd(r, r11)));
    p1 = 6 * i + 2;
    r = _mm_loadu_pd(&b_I[p1]);
    r1 = _mm_loadu_pd(&c_I[2]);
    _mm_storeu_pd(&c_I[2], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&pMotion_Information[p1]);
    r1 = _mm_loadu_pd(&xEModel[2]);
    _mm_storeu_pd(&xEModel[2], _mm_add_pd(r1, _mm_mul_pd(r, r11)));
    p1 = 6 * i + 4;
    r = _mm_loadu_pd(&b_I[p1]);
    r1 = _mm_loadu_pd(&c_I[4]);
    _mm_storeu_pd(&c_I[4], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&pMotion_Information[p1]);
    r1 = _mm_loadu_pd(&xEModel[4]);
    _mm_storeu_pd(&xEModel[4], _mm_add_pd(r1, _mm_mul_pd(r, r11)));
  }
  r = _mm_loadu_pd(&c_I[0]);
  r1 = _mm_loadu_pd(&xEModel[0]);
  _mm_storeu_pd(&c_I[0], _mm_add_pd(r, r1));
  r12 = _mm_set1_pd(0.0);
  _mm_storeu_pd(&track->State[0], r12);
  r = _mm_loadu_pd(&c_I[2]);
  r1 = _mm_loadu_pd(&xEModel[2]);
  _mm_storeu_pd(&c_I[2], _mm_add_pd(r, r1));
  _mm_storeu_pd(&track->State[2], r12);
  r = _mm_loadu_pd(&c_I[4]);
  r1 = _mm_loadu_pd(&xEModel[4]);
  _mm_storeu_pd(&c_I[4], _mm_add_pd(r, r1));
  _mm_storeu_pd(&track->State[4], r12);
  for (i = 0; i < 6; i++) {
    r = _mm_loadu_pd(&Y[6 * i]);
    r1 = _mm_loadu_pd(&track->State[0]);
    r12 = _mm_set1_pd(c_I[i]);
    _mm_storeu_pd(&track->State[0], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&Y[6 * i + 2]);
    r1 = _mm_loadu_pd(&track->State[2]);
    _mm_storeu_pd(&track->State[2], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&Y[6 * i + 4]);
    r1 = _mm_loadu_pd(&track->State[4]);
    _mm_storeu_pd(&track->State[4], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
  }
  c_st.site = &imb_emlrtRSI;
  absx11 = c_EKFStateEstimator_detectionPr(
      &c_st, d_estimator_StateEstimator_Stat, track->State);
  c_st.site = &jmb_emlrtRSI;
  d_st.site = &nw_emlrtRSI;
  e_st.site = &ow_emlrtRSI;
  if (!(absx11 >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &u_emlrtRTEI,
                                  "MATLAB:validators:mustBeNonnegative",
                                  "MATLAB:validators:mustBeNonnegative", 0);
  }
  e_st.site = &ow_emlrtRSI;
  f_st.site = &tk_emlrtRSI;
  if (!(absx11 < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &f_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  absx11 *= c_estimator_StateEstimator_Exis->BirthModel.BirthDensity;
  track->ExistenceProbability =
      absx11 /
      (absx11 + c_estimator_StateEstimator_Exis->ClutterModel.ClutterDensity);
  track->ExistenceProbability =
      muDoubleScalarMax(track->ExistenceProbability, 1.4901161193847656E-8);
  track->Time = b_time;
  track->Age = 1U;
  track->IsConfirmed = false;
  track->IsCoasted = false;
}

/* End of code generation (JIPDATrackInitiator.c) */
