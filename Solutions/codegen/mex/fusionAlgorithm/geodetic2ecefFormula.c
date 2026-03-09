/*
 * geodetic2ecefFormula.c
 *
 * Code generation for function 'geodetic2ecefFormula'
 *
 */

/* Include files */
#include "geodetic2ecefFormula.h"
#include "cosd.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo vh_emlrtRSI = {
    25,                     /* lineNo */
    "geodetic2ecefFormula", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\geodesy\\private\\geodetic2ecefFor"
    "mula.m" /* pathName */
};

static emlrtRSInfo wh_emlrtRSI = {
    59,                     /* lineNo */
    "geodetic2cylindrical", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\geodesy\\private\\geodetic2ecefFor"
    "mula.m" /* pathName */
};

static emlrtRSInfo xh_emlrtRSI = {
    78,                     /* lineNo */
    "geodetic2cylindrical", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\mapgeodesy\\+map\\+geodesy\\+"
    "internal\\geodetic2cylindrical.m" /* pathName */
};

static emlrtRTEInfo ec_emlrtRTEI = {
    13,     /* lineNo */
    9,      /* colNo */
    "sqrt", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m" /* pName
                                                                       */
};

/* Function Definitions */
real_T geodetic2ecefFormula(const emlrtStack *sp, real_T *y, real_T *z)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T N;
  real_T rho;
  real_T sinphi;
  real_T x;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &vh_emlrtRSI;
  b_st.site = &wh_emlrtRSI;
  sinphi = 42.39423231362;
  b_sind(&sinphi);
  c_st.site = &xh_emlrtRSI;
  x = 1.0 - 0.0066943799901413165 * (sinphi * sinphi);
  if (x < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &ec_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  x = muDoubleScalarSqrt(x);
  N = 6.378137E+6 / x;
  x = 42.39423231362;
  b_cosd(&x);
  rho = N * x;
  *z = N * 0.99330562000985867 * sinphi;
  x = -70.95934958874;
  b_cosd(&x);
  x *= rho;
  N = -70.95934958874;
  b_sind(&N);
  *y = rho * N;
  return x;
}

/* End of code generation (geodetic2ecefFormula.c) */
