/*
 * GaussianStateInitiator.c
 *
 * Code generation for function 'GaussianStateInitiator'
 *
 */

/* Include files */
#include "GaussianStateInitiator.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "stateToMeasurementJacobian.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_mexutil.h"
#include "warning.h"
#include "xzgetrf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fob_emlrtRSI = {
    108,                           /* lineNo */
    "ConstantVelocityModel/prior", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+transition/"
    "ConstantVelocityModel.m" /* pathName */
};

static emlrtRSInfo gob_emlrtRSI = {
    31,                                           /* lineNo */
    "inv",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName */
};

static emlrtRSInfo hob_emlrtRSI = {
    42,                                           /* lineNo */
    "checkcond",                                  /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName */
};

static emlrtRSInfo iob_emlrtRSI = {
    46,                                           /* lineNo */
    "checkcond",                                  /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/inv.m" /* pathName */
};

static emlrtRSInfo job_emlrtRSI = {
    12,                   /* lineNo */
    "measurementToState", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "measurementToState.m" /* pathName */
};

static emlrtRSInfo kob_emlrtRSI = {
    148,                                                         /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/inverseMeasurement", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+measurement/"
    "AzimuthElevationRangeAndRangeRateModel.m" /* pathName */
};

static emlrtRSInfo lob_emlrtRSI = {
    16,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
    "+stateInitiator/GaussianStateInitiator.m" /* pathName */
};

static emlrtRSInfo mob_emlrtRSI = {
    20,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
    "+stateInitiator/GaussianStateInitiator.m" /* pathName */
};

static emlrtRSInfo nob_emlrtRSI = {
    22,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
    "+stateInitiator/GaussianStateInitiator.m" /* pathName */
};

static emlrtRSInfo oob_emlrtRSI = {
    24,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
    "+stateInitiator/GaussianStateInitiator.m" /* pathName */
};

static emlrtRSInfo pob_emlrtRSI = {
    31,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
    "+stateInitiator/GaussianStateInitiator.m" /* pathName */
};

static emlrtRSInfo qob_emlrtRSI = {
    35,                                  /* lineNo */
    "GaussianStateInitiator/initialize", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
    "+stateInitiator/GaussianStateInitiator.m" /* pathName */
};

/* Function Declarations */
static void c_emlrt_marshallIn(const emlrtStack *sp,
                               const mxArray *a__output_of_sprintf_,
                               const char_T *identifier, char_T y[14]);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y[14]);

static void mb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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
  mb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void mb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, char_T ret[14])
{
  static const int32_T dims[2] = {1, 14};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 14);
  emlrtDestroyArray(&src);
}

void c_GaussianStateInitiator_initia(
    const emlrtStack *sp, const real_T motionModel_PropVelocityMean[3],
    const real_T c_motionModel_PropVelocityVaria[9],
    const real_T measModel_OriginPosition[9],
    const real_T measModel_OriginVelocity[9],
    const real_T measModel_Orientation[27], real_T measModel_AzimuthVariance,
    real_T measModel_ElevationVariance, real_T measModel_RangeVariance,
    real_T measModel_RangeRateVariance, const real_T measurement[4],
    real_T pdf_State[6], real_T pdf_StateCovariance[36])
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  st.site = &lob_emlrtRSI;
  for (i = 0; i < 6; i++) {
    pMotion_Mean[i] = 0.0;
  }
  pMotion_Mean[1] = motionModel_PropVelocityMean[0];
  pMotion_Mean[3] = motionModel_PropVelocityMean[1];
  pMotion_Mean[5] = motionModel_PropVelocityMean[2];
  memset(&pMotion_Information[0], 0, 36U * sizeof(real_T));
  b_st.site = &fob_emlrtRSI;
  memcpy(&x[0], &c_motionModel_PropVelocityVaria[0], 9U * sizeof(real_T));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = muDoubleScalarAbs(c_motionModel_PropVelocityVaria[0]);
  absx21 = muDoubleScalarAbs(c_motionModel_PropVelocityVaria[1]);
  absx31 = muDoubleScalarAbs(c_motionModel_PropVelocityVaria[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = c_motionModel_PropVelocityVaria[1];
    x[1] = c_motionModel_PropVelocityVaria[0];
    x[3] = c_motionModel_PropVelocityVaria[4];
    x[4] = c_motionModel_PropVelocityVaria[3];
    x[6] = c_motionModel_PropVelocityVaria[7];
    x[7] = c_motionModel_PropVelocityVaria[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    x[0] = c_motionModel_PropVelocityVaria[2];
    x[2] = c_motionModel_PropVelocityVaria[0];
    x[3] = c_motionModel_PropVelocityVaria[5];
    x[5] = c_motionModel_PropVelocityVaria[3];
    x[6] = c_motionModel_PropVelocityVaria[8];
    x[8] = c_motionModel_PropVelocityVaria[6];
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
  c_st.site = &gob_emlrtRSI;
  absx31 = 0.0;
  p1 = 0;
  exitg1 = false;
  while ((!exitg1) && (p1 < 3)) {
    absx11 = (muDoubleScalarAbs(c_motionModel_PropVelocityVaria[3 * p1]) +
              muDoubleScalarAbs(c_motionModel_PropVelocityVaria[3 * p1 + 1])) +
             muDoubleScalarAbs(c_motionModel_PropVelocityVaria[3 * p1 + 2]);
    if (muDoubleScalarIsNaN(absx11)) {
      absx31 = rtNaN;
      exitg1 = true;
    } else {
      if (absx11 > absx31) {
        absx31 = absx11;
      }
      p1++;
    }
  }
  absx21 = 0.0;
  p1 = 0;
  exitg1 = false;
  while ((!exitg1) && (p1 < 3)) {
    absx11 = (muDoubleScalarAbs(y[3 * p1]) + muDoubleScalarAbs(y[3 * p1 + 1])) +
             muDoubleScalarAbs(y[3 * p1 + 2]);
    if (muDoubleScalarIsNaN(absx11)) {
      absx21 = rtNaN;
      exitg1 = true;
    } else {
      if (absx11 > absx21) {
        absx21 = absx11;
      }
      p1++;
    }
  }
  absx11 = 1.0 / (absx31 * absx21);
  if ((absx31 == 0.0) || (absx21 == 0.0) || (absx11 == 0.0)) {
    if (!emlrtSetWarningFlag(&c_st)) {
      d_st.site = &hob_emlrtRSI;
      c_warning(&d_st);
    }
  } else if ((muDoubleScalarIsNaN(absx11) ||
              (absx11 < 2.2204460492503131E-16)) &&
             (!emlrtSetWarningFlag(&c_st))) {
    char_T str[14];
    d_st.site = &iob_emlrtRSI;
    b_y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(&d_st, 6, m, &rfmt[0]);
    emlrtAssign(&b_y, m);
    c_y = NULL;
    m = emlrtCreateDoubleScalar(absx11);
    emlrtAssign(&c_y, m);
    e_st.site = &bpb_emlrtRSI;
    c_emlrt_marshallIn(&e_st, b_sprintf(&e_st, b_y, c_y, &d_emlrtMCI),
                       "<output of sprintf>", str);
    d_st.site = &iob_emlrtRSI;
    e_warning(&d_st, str);
  }
  for (i = 0; i < 3; i++) {
    p1 = 6 * ((i << 1) + 1);
    pMotion_Information[p1 + 1] = y[3 * i];
    pMotion_Information[p1 + 3] = y[3 * i + 1];
    pMotion_Information[p1 + 5] = y[3 * i + 2];
  }
  real_T pz;
  st.site = &mob_emlrtRSI;
  b_st.site = &job_emlrtRSI;
  absx21 = 0.017453292519943295 * measurement[0];
  absx11 = 0.017453292519943295 * measurement[1];
  pz = measurement[2] * muDoubleScalarSin(absx11);
  absx11 = measurement[2] * muDoubleScalarCos(absx11);
  absx31 = absx11 * muDoubleScalarCos(absx21);
  absx21 = absx11 * muDoubleScalarSin(absx21);
  c_st.site = &kob_emlrtRSI;
  d_st.site = &td_emlrtRSI;
  c_st.site = &kob_emlrtRSI;
  d_st.site = &td_emlrtRSI;
  c_st.site = &kob_emlrtRSI;
  d_st.site = &td_emlrtRSI;
  c_st.site = &kob_emlrtRSI;
  absx11 = measurement[3] /
           muDoubleScalarSqrt((absx31 * absx31 + absx21 * absx21) + pz * pz);
  xP_idx_0 = absx11 * absx31;
  xP_idx_1 = absx11 * absx21;
  xP_idx_2 = absx11 * pz;
  xP[0] = absx31;
  xP[1] = absx21;
  xP[2] = pz;
  for (i = 0; i < 3; i++) {
    for (k = 0; k < 3; k++) {
      p1 = k + 9 * i;
      y[3 * k] = measModel_Orientation[p1];
      y[3 * k + 1] = measModel_Orientation[p1 + 3];
      y[3 * k + 2] = measModel_Orientation[p1 + 6];
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
    r1 = _mm_loadu_pd(&measModel_OriginPosition[3 * i]);
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&d_y[0], r);
    d_y[2] = ((y[2] * absx11 + y[5] * absx21) + y[8] * absx31) +
             measModel_OriginPosition[2 + 3 * i];
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
    r1 = _mm_loadu_pd(&measModel_OriginVelocity[3 * i]);
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&d_y[0], r);
    xP[2] = d_y[2];
    d_y[2] = ((y[2] * xP_idx_0 + y[5] * xP_idx_1) + y[8] * xP_idx_2) +
             measModel_OriginVelocity[2 + 3 * i];
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
  st.site = &nob_emlrtRSI;
  stateToMeasurementJacobian(&st, xE, measModel_OriginPosition,
                             measModel_OriginVelocity, measModel_Orientation,
                             H);
  memset(&R[0], 0, 16U * sizeof(real_T));
  R[0] = measModel_AzimuthVariance;
  R[5] = measModel_ElevationVariance;
  R[10] = measModel_RangeVariance;
  R[15] = measModel_RangeRateVariance;
  st.site = &oob_emlrtRSI;
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 6; k++) {
      X[k + 6 * i] = H[i + (k << 2)];
    }
  }
  b_st.site = &mbb_emlrtRSI;
  c_st.site = &nbb_emlrtRSI;
  d_st.site = &obb_emlrtRSI;
  e_st.site = &pbb_emlrtRSI;
  f_st.site = &rbb_emlrtRSI;
  g_st.site = &tbb_emlrtRSI;
  info = xzgetrf(&g_st, R, ipiv);
  f_st.site = &sbb_emlrtRSI;
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
    e_st.site = &qbb_emlrtRSI;
    if (!emlrtSetWarningFlag(&e_st)) {
      f_st.site = &bcb_emlrtRSI;
      c_warning(&f_st);
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
  st.site = &pob_emlrtRSI;
  for (i = 0; i < 36; i++) {
    Y[i] = In[i];
  }
  b_st.site = &mbb_emlrtRSI;
  b_mrdiv(&b_st, Y, Istate);
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
  memset(&pdf_State[0], 0, 6U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    absx11 = c_I[i] + xEModel[i];
    c_I[i] = absx11;
    r = _mm_loadu_pd(&Y[6 * i]);
    r1 = _mm_loadu_pd(&pdf_State[0]);
    r12 = _mm_set1_pd(absx11);
    _mm_storeu_pd(&pdf_State[0], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&Y[6 * i + 2]);
    r1 = _mm_loadu_pd(&pdf_State[2]);
    _mm_storeu_pd(&pdf_State[2], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
    r = _mm_loadu_pd(&Y[6 * i + 4]);
    r1 = _mm_loadu_pd(&pdf_State[4]);
    _mm_storeu_pd(&pdf_State[4], _mm_add_pd(r1, _mm_mul_pd(r, r12)));
  }
  st.site = &qob_emlrtRSI;
  for (i = 0; i < 36; i++) {
    pdf_StateCovariance[i] = In[i];
  }
  b_st.site = &mbb_emlrtRSI;
  b_mrdiv(&b_st, pdf_StateCovariance, Istate);
}

/* End of code generation (GaussianStateInitiator.c) */
