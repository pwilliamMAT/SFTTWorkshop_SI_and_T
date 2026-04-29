/*
 * ExtendedKalmanFilter.c
 *
 * Code generation for function 'ExtendedKalmanFilter'
 *
 */

/* Include files */
#include "ExtendedKalmanFilter.h"
#include "eig.h"
#include "eml_int_forloop_overflow_check.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "rt_nonfinite.h"
#include "stateToMeasurementJacobian.h"
#include "stateToMeasurementWrapped.h"
#include "svd.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_types.h"
#include "xpotrf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo qc_emlrtRSI = {
    1033,                                       /* lineNo */
    "ExtendedKalmanFilter/set.StateCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo rd_emlrtRSI = {
    1067,                                    /* lineNo */
    "ExtendedKalmanFilter/set.ProcessNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo sd_emlrtRSI = {
    1074,                                    /* lineNo */
    "ExtendedKalmanFilter/set.ProcessNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo td_emlrtRSI = {
    1084,                                    /* lineNo */
    "ExtendedKalmanFilter/set.ProcessNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ud_emlrtRSI = {
    1091,                                    /* lineNo */
    "ExtendedKalmanFilter/set.ProcessNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo nh_emlrtRSI = {
    15,        /* lineNo */
    "cholPSD", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\cholPSD.m" /* pathName */
};

static emlrtRSInfo oh_emlrtRSI = {
    18,        /* lineNo */
    "cholPSD", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\cholPSD.m" /* pathName */
};

static emlrtRSInfo ph_emlrtRSI = {
    20,        /* lineNo */
    "cholPSD", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\cholPSD.m" /* pathName */
};

static emlrtRSInfo qh_emlrtRSI = {
    15,     /* lineNo */
    "chol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo rh_emlrtRSI = {
    84,     /* lineNo */
    "chol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\chol.m" /* pathName
                                                                           */
};

static emlrtRSInfo sh_emlrtRSI = {
    100,    /* lineNo */
    "chol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\chol.m" /* pathName
                                                                           */
};

static emlrtRSInfo th_emlrtRSI = {
    101,    /* lineNo */
    "chol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\chol.m" /* pathName
                                                                           */
};

static emlrtRSInfo wh_emlrtRSI = {
    12,     /* lineNo */
    "chol", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo xh_emlrtRSI = {
    15,       /* lineNo */
    "svdPSD", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\svdPSD.m" /* pathName */
};

static emlrtRSInfo yh_emlrtRSI = {
    17,       /* lineNo */
    "svdPSD", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\svdPSD.m" /* pathName */
};

static emlrtRSInfo ai_emlrtRSI = {
    14,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo bi_emlrtRSI = {
    36,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo lt_emlrtRSI = {
    925,                              /* lineNo */
    "ExtendedKalmanFilter/set.State", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo mt_emlrtRSI = {
    1013,                                       /* lineNo */
    "ExtendedKalmanFilter/set.StateCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo nt_emlrtRSI = {
    1022,                                       /* lineNo */
    "ExtendedKalmanFilter/set.StateCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ot_emlrtRSI = {
    1026,                                       /* lineNo */
    "ExtendedKalmanFilter/set.StateCovariance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo xw_emlrtRSI = {
    1124,                                        /* lineNo */
    "ExtendedKalmanFilter/set.MeasurementNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo yw_emlrtRSI = {
    1135,                                        /* lineNo */
    "ExtendedKalmanFilter/set.MeasurementNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ax_emlrtRSI = {
    1146,                                        /* lineNo */
    "ExtendedKalmanFilter/set.MeasurementNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo bx_emlrtRSI = {
    1154,                                        /* lineNo */
    "ExtendedKalmanFilter/set.MeasurementNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo cx_emlrtRSI = {
    1155,                                        /* lineNo */
    "ExtendedKalmanFilter/set.MeasurementNoise", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo dx_emlrtRSI = {
    2071,                                                   /* lineNo */
    "ExtendedKalmanFilter/ensureMeasurementNoiseIsDefined", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ex_emlrtRSI = {
    1676,                                                           /* lineNo */
    "ExtendedKalmanFilter/measurementNoiseScalarExpandIfNecessary", /* fcnName
                                                                     */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo lx_emlrtRSI = {
    2020,                                                           /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementAndRelatedProperties", /* fcnName
                                                                     */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo mx_emlrtRSI = {
    2024,                                                           /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementAndRelatedProperties", /* fcnName
                                                                     */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo nx_emlrtRSI = {
    2030,                                                           /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementAndRelatedProperties", /* fcnName
                                                                     */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ox_emlrtRSI = {
    2036,                                                           /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementAndRelatedProperties", /* fcnName
                                                                     */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo px_emlrtRSI = {
    1972,                                          /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo qx_emlrtRSI = {
    14,                                            /* lineNo */
    "EKFCorrectorAdditive/validateMeasurementFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrectorAddi"
    "tive.m" /* pathName */
};

static emlrtRSInfo xx_emlrtRSI = {
    1998,                                                  /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementJacobianFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo yx_emlrtRSI = {
    2000,                                                  /* lineNo */
    "ExtendedKalmanFilter/validateMeasurementJacobianFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\ExtendedKalmanFi"
    "lter.m" /* pathName */
};

static emlrtRSInfo ay_emlrtRSI = {
    22,                                                    /* lineNo */
    "EKFCorrectorAdditive/validateMeasurementJacobianFcn", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\EKFCorrectorAddi"
    "tive.m" /* pathName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    109,    /* lineNo */
    27,     /* colNo */
    "chol", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\chol.m" /* pName
                                                                           */
};

/* Function Definitions */
void ExtendedKalmanFilter_set_State(const emlrtStack *sp, trackingEKF *obj,
                                    const real_T b_value[6])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &lt_emlrtRSI;
  b_st.site = &vd_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(b_value[k])) &&
        (!muDoubleScalarIsNaN(b_value[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:ExtendedKalmanFilter:expectedFinite", 3, 4, 5, "State");
  }
  for (i = 0; i < 6; i++) {
    obj->pState[i] = b_value[i];
  }
}

void c_ExtendedKalmanFilter_set_Meas(const emlrtStack *sp, trackingEKF *obj,
                                     real_T b_value[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T Ss[16];
  real_T V[16];
  int32_T i;
  int32_T j;
  int32_T jmax;
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
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  st.site = &xw_emlrtRSI;
  b_st.site = &vd_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    if ((!muDoubleScalarIsInf(b_value[k])) &&
        (!muDoubleScalarIsNaN(b_value[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:ExtendedKalmanFilter:expectedFinite", 3, 4, 16,
        "MeasurementNoise");
  }
  st.site = &yw_emlrtRSI;
  b_isSymmetricPositiveSemiDefini(&st, b_value);
  st.site = &ax_emlrtRSI;
  b_st.site = &nh_emlrtRSI;
  c_st.site = &qh_emlrtRSI;
  memcpy(&Ss[0], &b_value[0], 16U * sizeof(real_T));
  d_st.site = &rh_emlrtRSI;
  k = c_xpotrf(&d_st, Ss);
  if (k == 0) {
    jmax = 3;
  } else {
    jmax = k - 2;
  }
  d_st.site = &sh_emlrtRSI;
  if (jmax > 2147483646) {
    e_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }
  for (j = 0; j < jmax; j++) {
    d_st.site = &th_emlrtRSI;
    if ((j + 2 <= jmax + 1) && (jmax + 1 > 2147483646)) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
  }
  if (k == 0) {
    int32_T b;
    b_st.site = &oh_emlrtRSI;
    c_st.site = &wh_emlrtRSI;
    memcpy(&Ss[0], &b_value[0], 16U * sizeof(real_T));
    d_st.site = &rh_emlrtRSI;
    k = c_xpotrf(&d_st, Ss);
    if (k == 0) {
      jmax = 4;
    } else {
      jmax = k - 1;
    }
    b = jmax - 2;
    d_st.site = &sh_emlrtRSI;
    if (jmax - 1 > 2147483646) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    for (j = 0; j <= b; j++) {
      int32_T a;
      a = j + 2;
      d_st.site = &th_emlrtRSI;
      if ((j + 2 <= jmax) && (jmax > 2147483646)) {
        e_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&e_st);
      }
      if (a <= jmax) {
        memset(&Ss[(j * 4 + a) + -1], 0,
               (uint32_T)((jmax - a) + 1) * sizeof(real_T));
      }
    }
    if (k != 0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &e_emlrtRTEI, "MATLAB:posdef",
                                    "MATLAB:posdef", 0);
    }
    for (j = 0; j < 4; j++) {
      k = j << 2;
      b_value[k] = Ss[j];
      b_value[k + 1] = Ss[j + 4];
      b_value[k + 2] = Ss[j + 8];
      b_value[k + 3] = Ss[j + 12];
    }
  } else {
    __m128d r;
    real_T s[4];
    b_st.site = &ph_emlrtRSI;
    c_st.site = &xh_emlrtRSI;
    d_st.site = &ai_emlrtRSI;
    p = true;
    for (j = 0; j < 16; j++) {
      if (p) {
        real_T d;
        d = b_value[j];
        if (muDoubleScalarIsInf(d) || muDoubleScalarIsNaN(d)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (p) {
      d_st.site = &bi_emlrtRSI;
      c_svd(&d_st, b_value, Ss, s, V);
    } else {
      s[0] = rtNaN;
      s[1] = rtNaN;
      s[2] = rtNaN;
      s[3] = rtNaN;
      for (j = 0; j < 16; j++) {
        V[j] = rtNaN;
      }
    }
    memset(&Ss[0], 0, 16U * sizeof(real_T));
    Ss[0] = s[0];
    Ss[5] = s[1];
    Ss[10] = s[2];
    Ss[15] = s[3];
    c_st.site = &yh_emlrtRSI;
    p = false;
    for (j = 0; j < 16; j++) {
      if (p || (Ss[j] < 0.0)) {
        p = true;
      }
    }
    if (p) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    for (j = 0; j <= 14; j += 2) {
      r = _mm_loadu_pd(&Ss[j]);
      _mm_storeu_pd(&Ss[j], _mm_sqrt_pd(r));
    }
    memset(&b_value[0], 0, sizeof(real_T) << 4);
    for (j = 0; j < 4; j++) {
      k = j << 2;
      for (i = 0; i < 4; i++) {
        __m128d r1;
        __m128d r2;
        jmax = i << 2;
        r = _mm_loadu_pd(&V[jmax]);
        r1 = _mm_loadu_pd(&b_value[k]);
        r2 = _mm_set1_pd(Ss[i + k]);
        _mm_storeu_pd(&b_value[k], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&V[jmax + 2]);
        r1 = _mm_loadu_pd(&b_value[k + 2]);
        _mm_storeu_pd(&b_value[k + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
  }
  obj->pSqrtMeasurementNoiseScalar = -1.0;
  st.site = &bx_emlrtRSI;
  b_st.site = &dx_emlrtRSI;
  if (obj->pSqrtMeasurementNoiseScalar > 0.0) {
    c_st.site = &ex_emlrtRSI;
    obj->pSqrtMeasurementNoiseScalar = -1.0;
  }
  st.site = &cx_emlrtRSI;
  memcpy(&obj->pSqrtMeasurementNoise[0], &b_value[0], 16U * sizeof(real_T));
}

void c_ExtendedKalmanFilter_set_Proc(const emlrtStack *sp, trackingEKF *obj,
                                     const real_T b_value[9])
{
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T Ss[9];
  real_T b_x[9];
  real_T s[3];
  real_T dv[2];
  real_T dv1[2];
  real_T absx;
  real_T x;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T jmax;
  boolean_T x_data[6];
  boolean_T y[3];
  boolean_T b_y;
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
  st.site = &rd_emlrtRSI;
  b_st.site = &vd_emlrtRSI;
  p = true;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx < 9)) {
    if ((!muDoubleScalarIsInf(b_value[idx])) &&
        (!muDoubleScalarIsNaN(b_value[idx]))) {
      idx++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:ExtendedKalmanFilter:expectedFinite", 3, 4, 12, "ProcessNoise");
  }
  st.site = &sd_emlrtRSI;
  s[0] = muDoubleScalarAbs(b_value[0]);
  s[1] = muDoubleScalarAbs(b_value[4]);
  s[2] = muDoubleScalarAbs(b_value[8]);
  if (!muDoubleScalarIsNaN(s[0])) {
    idx = 1;
  } else {
    idx = 0;
    jmax = 2;
    exitg1 = false;
    while ((!exitg1) && (jmax < 4)) {
      if (!muDoubleScalarIsNaN(s[jmax - 1])) {
        idx = jmax;
        exitg1 = true;
      } else {
        jmax++;
      }
    }
  }
  if (idx == 0) {
    absx = s[0];
  } else {
    absx = s[idx - 1];
    idx++;
    for (i = idx; i < 4; i++) {
      x = s[i - 1];
      if (absx < x) {
        absx = x;
      }
    }
  }
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    absx = rtNaN;
  } else if (absx < 4.4501477170144028E-308) {
    absx = 4.94065645841247E-324;
  } else {
    frexp(absx, &i1);
    absx = ldexp(1.0, i1 - 53);
  }
  absx *= 100.0;
  for (i = 0; i < 3; i++) {
    Ss[3 * i] = b_value[i];
    Ss[3 * i + 1] = b_value[i + 3];
    Ss[3 * i + 2] = b_value[i + 6];
  }
  r = _mm_loadu_pd(&Ss[0]);
  _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[0]), r));
  dv1[0] = muDoubleScalarAbs(dv[0]);
  dv1[1] = muDoubleScalarAbs(dv[1]);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&b_x[0], r);
  r = _mm_loadu_pd(&Ss[2]);
  _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[2]), r));
  dv1[0] = muDoubleScalarAbs(dv[0]);
  dv1[1] = muDoubleScalarAbs(dv[1]);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&b_x[2], r);
  r = _mm_loadu_pd(&Ss[4]);
  _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[4]), r));
  dv1[0] = muDoubleScalarAbs(dv[0]);
  dv1[1] = muDoubleScalarAbs(dv[1]);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&b_x[4], r);
  r = _mm_loadu_pd(&Ss[6]);
  _mm_storeu_pd(&dv[0], _mm_sub_pd(_mm_loadu_pd(&b_value[6]), r));
  dv1[0] = muDoubleScalarAbs(dv[0]);
  dv1[1] = muDoubleScalarAbs(dv[1]);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&b_x[6], r);
  b_x[8] = muDoubleScalarAbs(b_value[8] - Ss[8]);
  b_st.site = &wd_emlrtRSI;
  x = muDoubleScalarSqrt(absx);
  b_st.site = &wd_emlrtRSI;
  c_st.site = &yd_emlrtRSI;
  y[0] = true;
  y[1] = true;
  y[2] = true;
  idx = 3;
  for (i = 0; i < 3; i++) {
    jmax = idx;
    i1 = idx - 2;
    idx += 3;
    d_st.site = &ae_emlrtRSI;
    if ((i1 <= jmax) && (jmax > 2147483646)) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    exitg1 = false;
    while ((!exitg1) && (i1 <= jmax)) {
      if (!(b_x[i1 - 1] < x)) {
        y[i] = false;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
  p = true;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= 2)) {
    if (!y[idx]) {
      p = false;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  creal_T d[3];
  r = _mm_loadu_pd(&Ss[0]);
  r1 = _mm_set1_pd(2.0);
  _mm_storeu_pd(&Ss[0],
                _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[0]), r), r1));
  r = _mm_loadu_pd(&Ss[2]);
  _mm_storeu_pd(&Ss[2],
                _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[2]), r), r1));
  r = _mm_loadu_pd(&Ss[4]);
  _mm_storeu_pd(&Ss[4],
                _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[4]), r), r1));
  r = _mm_loadu_pd(&Ss[6]);
  _mm_storeu_pd(&Ss[6],
                _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&b_value[6]), r), r1));
  Ss[8] = (b_value[8] + Ss[8]) / 2.0;
  b_st.site = &xd_emlrtRSI;
  eig(&b_st, Ss, d);
  x_data[0] = (d[0].re < -absx);
  x_data[1] = (d[1].re < -absx);
  x_data[2] = (d[2].re < -absx);
  b_y = false;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= 2)) {
    if (x_data[idx]) {
      b_y = true;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  if (b_y || (!p)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &g_emlrtRTEI,
        "shared_tracking:KalmanFilter:invalidCovarianceValues",
        "shared_tracking:KalmanFilter:invalidCovarianceValues", 3, 4, 12,
        "ProcessNoise");
  }
  st.site = &td_emlrtRSI;
  b_st.site = &nh_emlrtRSI;
  c_st.site = &qh_emlrtRSI;
  memcpy(&Ss[0], &b_value[0], 9U * sizeof(real_T));
  d_st.site = &rh_emlrtRSI;
  idx = xpotrf(&d_st, Ss);
  if (idx == 0) {
    jmax = 2;
  } else {
    jmax = idx - 2;
  }
  d_st.site = &sh_emlrtRSI;
  if (jmax > 2147483646) {
    e_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }
  for (i = 0; i < jmax; i++) {
    d_st.site = &th_emlrtRSI;
    if ((i + 2 <= jmax + 1) && (jmax + 1 > 2147483646)) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
  }
  if (idx == 0) {
    b_st.site = &oh_emlrtRSI;
    c_st.site = &wh_emlrtRSI;
    memcpy(&Ss[0], &b_value[0], 9U * sizeof(real_T));
    d_st.site = &rh_emlrtRSI;
    idx = xpotrf(&d_st, Ss);
    if (idx == 0) {
      jmax = 3;
    } else {
      jmax = idx - 1;
    }
    i1 = jmax - 2;
    d_st.site = &sh_emlrtRSI;
    if (jmax - 1 > 2147483646) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    for (i = 0; i <= i1; i++) {
      int32_T a;
      a = i + 2;
      d_st.site = &th_emlrtRSI;
      if ((i + 2 <= jmax) && (jmax > 2147483646)) {
        e_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&e_st);
      }
      if (a <= jmax) {
        memset(&Ss[(i * 3 + a) + -1], 0,
               (uint32_T)((jmax - a) + 1) * sizeof(real_T));
      }
    }
    if (idx != 0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &e_emlrtRTEI, "MATLAB:posdef",
                                    "MATLAB:posdef", 0);
    }
    for (i = 0; i < 3; i++) {
      obj->pSqrtProcessNoise[3 * i] = Ss[i];
      obj->pSqrtProcessNoise[3 * i + 1] = Ss[i + 3];
      obj->pSqrtProcessNoise[3 * i + 2] = Ss[i + 6];
    }
  } else {
    b_st.site = &ph_emlrtRSI;
    c_st.site = &xh_emlrtRSI;
    d_st.site = &ai_emlrtRSI;
    p = true;
    for (i = 0; i < 9; i++) {
      if (p) {
        absx = b_value[i];
        if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (p) {
      d_st.site = &bi_emlrtRSI;
      svd(&d_st, b_value, Ss, s, b_x);
    } else {
      s[0] = rtNaN;
      s[1] = rtNaN;
      s[2] = rtNaN;
      for (i = 0; i < 9; i++) {
        b_x[i] = rtNaN;
      }
    }
    memset(&Ss[0], 0, 9U * sizeof(real_T));
    Ss[0] = s[0];
    Ss[4] = s[1];
    Ss[8] = s[2];
    c_st.site = &yh_emlrtRSI;
    p = false;
    for (i = 0; i < 9; i++) {
      if (p || (Ss[i] < 0.0)) {
        p = true;
      }
    }
    if (p) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    r = _mm_loadu_pd(&Ss[0]);
    _mm_storeu_pd(&Ss[0], _mm_sqrt_pd(r));
    r = _mm_loadu_pd(&Ss[2]);
    _mm_storeu_pd(&Ss[2], _mm_sqrt_pd(r));
    r = _mm_loadu_pd(&Ss[4]);
    _mm_storeu_pd(&Ss[4], _mm_sqrt_pd(r));
    r = _mm_loadu_pd(&Ss[6]);
    _mm_storeu_pd(&Ss[6], _mm_sqrt_pd(r));
    Ss[8] = muDoubleScalarSqrt(Ss[8]);
    for (i = 0; i < 3; i++) {
      obj->pSqrtProcessNoise[3 * i] = 0.0;
      idx = 3 * i + 1;
      obj->pSqrtProcessNoise[idx] = 0.0;
      jmax = 3 * i + 2;
      obj->pSqrtProcessNoise[jmax] = 0.0;
      absx = Ss[3 * i];
      r = _mm_loadu_pd(&b_x[0]);
      r1 = _mm_loadu_pd(&obj->pSqrtProcessNoise[3 * i]);
      _mm_storeu_pd(&obj->pSqrtProcessNoise[3 * i],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(absx))));
      obj->pSqrtProcessNoise[jmax] += b_x[2] * absx;
      absx = Ss[idx];
      r = _mm_loadu_pd(&b_x[3]);
      r1 = _mm_loadu_pd(&obj->pSqrtProcessNoise[3 * i]);
      _mm_storeu_pd(&obj->pSqrtProcessNoise[3 * i],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(absx))));
      obj->pSqrtProcessNoise[jmax] += b_x[5] * absx;
      absx = Ss[jmax];
      r = _mm_loadu_pd(&b_x[6]);
      r1 = _mm_loadu_pd(&obj->pSqrtProcessNoise[3 * i]);
      _mm_storeu_pd(&obj->pSqrtProcessNoise[3 * i],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(absx))));
      obj->pSqrtProcessNoise[jmax] += b_x[8] * absx;
    }
  }
  st.site = &ud_emlrtRSI;
  obj->pIsSetProcessNoise = true;
  obj->pSqrtProcessNoiseScalar = -1.0;
}

void c_ExtendedKalmanFilter_set_Stat(const emlrtStack *sp, trackingEKF *obj,
                                     const real_T b_value[36])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T Ss[36];
  real_T V[36];
  int32_T b_i;
  int32_T i;
  int32_T jmax;
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
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  st.site = &mt_emlrtRSI;
  b_st.site = &vd_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 36)) {
    if ((!muDoubleScalarIsInf(b_value[k])) &&
        (!muDoubleScalarIsNaN(b_value[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &f_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:ExtendedKalmanFilter:expectedFinite", 3, 4, 15,
        "StateCovariance");
  }
  st.site = &nt_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&st, b_value);
  st.site = &ot_emlrtRSI;
  b_st.site = &nh_emlrtRSI;
  c_st.site = &qh_emlrtRSI;
  memcpy(&Ss[0], &b_value[0], 36U * sizeof(real_T));
  d_st.site = &rh_emlrtRSI;
  k = b_xpotrf(&d_st, Ss);
  if (k == 0) {
    jmax = 5;
  } else {
    jmax = k - 2;
  }
  d_st.site = &sh_emlrtRSI;
  if (jmax > 2147483646) {
    e_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }
  for (i = 0; i < jmax; i++) {
    d_st.site = &th_emlrtRSI;
    if ((i + 2 <= jmax + 1) && (jmax + 1 > 2147483646)) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
  }
  if (k == 0) {
    int32_T b;
    b_st.site = &oh_emlrtRSI;
    c_st.site = &wh_emlrtRSI;
    memcpy(&Ss[0], &b_value[0], 36U * sizeof(real_T));
    d_st.site = &rh_emlrtRSI;
    k = b_xpotrf(&d_st, Ss);
    if (k == 0) {
      jmax = 6;
    } else {
      jmax = k - 1;
    }
    b = jmax - 2;
    d_st.site = &sh_emlrtRSI;
    if (jmax - 1 > 2147483646) {
      e_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    for (i = 0; i <= b; i++) {
      int32_T a;
      a = i + 2;
      d_st.site = &th_emlrtRSI;
      if ((i + 2 <= jmax) && (jmax > 2147483646)) {
        e_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&e_st);
      }
      if (a <= jmax) {
        memset(&Ss[(i * 6 + a) + -1], 0,
               (uint32_T)((jmax - a) + 1) * sizeof(real_T));
      }
    }
    if (k != 0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &e_emlrtRTEI, "MATLAB:posdef",
                                    "MATLAB:posdef", 0);
    }
    for (i = 0; i < 6; i++) {
      for (b_i = 0; b_i < 6; b_i++) {
        obj->pSqrtStateCovariance[b_i + 6 * i] = Ss[i + 6 * b_i];
      }
    }
  } else {
    __m128d r;
    real_T s[6];
    b_st.site = &ph_emlrtRSI;
    c_st.site = &xh_emlrtRSI;
    d_st.site = &ai_emlrtRSI;
    p = true;
    for (i = 0; i < 36; i++) {
      if (p) {
        real_T d;
        d = b_value[i];
        if (muDoubleScalarIsInf(d) || muDoubleScalarIsNaN(d)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (p) {
      d_st.site = &bi_emlrtRSI;
      b_svd(&d_st, b_value, Ss, s, V);
    } else {
      for (i = 0; i < 6; i++) {
        s[i] = rtNaN;
      }
      for (i = 0; i < 36; i++) {
        V[i] = rtNaN;
      }
    }
    memset(&Ss[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      Ss[i + 6 * i] = s[i];
    }
    c_st.site = &yh_emlrtRSI;
    p = false;
    for (i = 0; i < 36; i++) {
      if (p || (Ss[i] < 0.0)) {
        p = true;
      }
    }
    if (p) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
    }
    for (i = 0; i <= 34; i += 2) {
      r = _mm_loadu_pd(&Ss[i]);
      _mm_storeu_pd(&Ss[i], _mm_sqrt_pd(r));
    }
    for (b_i = 0; b_i < 6; b_i++) {
      for (i = 0; i < 6; i++) {
        obj->pSqrtStateCovariance[i + 6 * b_i] = 0.0;
      }
      k = 6 * b_i + 2;
      jmax = 6 * b_i + 4;
      for (i = 0; i < 6; i++) {
        __m128d r1;
        __m128d r2;
        r = _mm_loadu_pd(&V[6 * i]);
        r1 = _mm_loadu_pd(&obj->pSqrtStateCovariance[6 * b_i]);
        r2 = _mm_set1_pd(Ss[i + 6 * b_i]);
        _mm_storeu_pd(&obj->pSqrtStateCovariance[6 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&V[6 * i + 2]);
        r1 = _mm_loadu_pd(&obj->pSqrtStateCovariance[k]);
        _mm_storeu_pd(&obj->pSqrtStateCovariance[k],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&V[6 * i + 4]);
        r1 = _mm_loadu_pd(&obj->pSqrtStateCovariance[jmax]);
        _mm_storeu_pd(&obj->pSqrtStateCovariance[jmax],
                      _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
  }
  st.site = &qc_emlrtRSI;
  obj->pIsSetStateCovariance = true;
  obj->pSqrtStateCovarianceScalar = -1.0;
}

void c_ExtendedKalmanFilter_validate(const emlrtStack *sp, trackingEKF *obj,
                                     const real_T varargin_2_OriginPosition[9],
                                     const real_T varargin_2_OriginVelocity[9],
                                     const real_T varargin_2_Orientation[27])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_unusedExpr[24];
  real_T unusedExpr[4];
  real_T a;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &lx_emlrtRSI;
  b_st.site = &cu_emlrtRSI;
  if ((!obj->pIsSetStateCovariance) ||
      (obj->pSqrtStateCovarianceScalar != -1.0)) {
    a = obj->pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      obj->pSqrtStateCovariance[i] = a * (real_T)iv[i];
    }
    obj->pIsSetStateCovariance = true;
    obj->pSqrtStateCovarianceScalar = -1.0;
  }
  if (obj->pIsFirstCallCorrect) {
    st.site = &mx_emlrtRSI;
    if (!obj->pIsValidMeasurementFcn) {
      b_st.site = &px_emlrtRSI;
      c_st.site = &qx_emlrtRSI;
      stateToMeasurementWrapped(&c_st, obj->pState, varargin_2_OriginPosition,
                                varargin_2_OriginVelocity,
                                varargin_2_Orientation, unusedExpr);
      obj->pIsValidMeasurementFcn = true;
    }
    st.site = &nx_emlrtRSI;
    b_st.site = &xx_emlrtRSI;
    c_st.site = &ay_emlrtRSI;
    stateToMeasurementJacobian(&c_st, obj->pState, varargin_2_OriginPosition,
                               varargin_2_OriginVelocity,
                               varargin_2_Orientation, b_unusedExpr);
    b_st.site = &yx_emlrtRSI;
    obj->pIsFirstCallCorrect = false;
  }
  st.site = &ox_emlrtRSI;
  b_st.site = &dx_emlrtRSI;
  if (obj->pSqrtMeasurementNoiseScalar > 0.0) {
    a = obj->pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 16; i++) {
      obj->pSqrtMeasurementNoise[i] = a * (real_T)iv1[i];
    }
    obj->pSqrtMeasurementNoiseScalar = -1.0;
  }
}

/* End of code generation (ExtendedKalmanFilter.c) */
