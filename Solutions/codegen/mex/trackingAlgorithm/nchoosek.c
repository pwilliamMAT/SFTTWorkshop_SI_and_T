/*
 * nchoosek.c
 *
 * Code generation for function 'nchoosek'
 *
 */

/* Include files */
#include "nchoosek.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_mexutil.h"
#include "warning.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo
    feb_emlrtRSI =
        {
            56,         /* lineNo */
            "nchoosek", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pathName */
};

static emlrtRSInfo
    geb_emlrtRSI =
        {
            141,   /* lineNo */
            "nCk", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pathName */
};

static emlrtRSInfo
    heb_emlrtRSI =
        {
            142,   /* lineNo */
            "nCk", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pathName */
};

static emlrtRSInfo
    ieb_emlrtRSI =
        {
            129,   /* lineNo */
            "nCk", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pathName */
};

static emlrtRSInfo
    jeb_emlrtRSI =
        {
            153,        /* lineNo */
            "nCkInt64", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pathName */
};

static emlrtRTEInfo
    sb_emlrtRTEI =
        {
            38,         /* lineNo */
            26,         /* colNo */
            "nchoosek", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pName */
};

static emlrtRTEInfo
    tb_emlrtRTEI =
        {
            17,         /* lineNo */
            23,         /* colNo */
            "nchoosek", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pName */
};

static emlrtRTEInfo
    ql_emlrtRTEI =
        {
            155,        /* lineNo */
            13,         /* colNo */
            "nCkInt64", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\specfun\\nchoose"
            "k.m" /* pName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y[23]);

static void emlrt_marshallIn(const emlrtStack *sp,
                             const mxArray *a__output_of_sprintf_,
                             const char_T *identifier, char_T y[23]);

static void lb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                char_T ret[23]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[23])
{
  lb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp,
                             const mxArray *a__output_of_sprintf_,
                             const char_T *identifier, char_T y[23])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

static void lb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, char_T ret[23])
{
  static const int32_T dims[2] = {1, 23};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 23);
  emlrtDestroyArray(&src);
}

real_T nchoosek(const emlrtStack *sp, real_T x, real_T k)
{
  static const int32_T b_iv[2] = {1, 7};
  static const int32_T b_iv1[2] = {1, 7};
  static const char_T rfmt[7] = {'%', '2', '3', '.', '1', '5', 'e'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *m;
  real_T r;
  real_T y;
  uint64_T u;
  int32_T j;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  if ((!(k >= 0.0)) || (!(k == muDoubleScalarFloor(k)))) {
    emlrtErrorWithMessageIdR2018a(sp, &tb_emlrtRTEI,
                                  "MATLAB:nchoosek:InvalidArg2",
                                  "MATLAB:nchoosek:InvalidArg2", 0);
  }
  if (!(k <= x)) {
    emlrtErrorWithMessageIdR2018a(sp, &sb_emlrtRTEI,
                                  "MATLAB:nchoosek:KOutOfRange",
                                  "MATLAB:nchoosek:KOutOfRange", 0);
  }
  r = k;
  if (k > x - k) {
    r = x - k;
  }
  if (r == 0.0) {
    y = 1.0;
  } else if (r == 1.0) {
    y = x;
  } else {
    st.site = &feb_emlrtRSI;
    if (muDoubleScalarIsInf(x) ||
        (muDoubleScalarIsInf(r) || muDoubleScalarIsNaN(r))) {
      y = rtNaN;
    } else if (r > 1000.0) {
      y = rtInf;
    } else {
      real_T maxRelErr;
      int32_T i;
      maxRelErr = 0.0;
      y = x;
      i = (int32_T)(r - 1.0);
      for (j = 0; j < i; j++) {
        y *= ((x - ((real_T)j + 2.0)) + 1.0) / ((real_T)j + 2.0);
        if (!(y < 1.125899906842624E+15)) {
          maxRelErr += 4.4408920985006262E-16;
        }
        y = muDoubleScalarRound(y);
      }
      if ((maxRelErr != 0.0) && (y <= 3.6893488147419103E+19)) {
        uint64_T b_i;
        uint64_T b_k;
        uint64_T n;
        uint64_T yint;
        boolean_T exitg1;
        b_st.site = &ieb_emlrtRSI;
        y = muDoubleScalarRound(x);
        if (y < 1.8446744073709552E+19) {
          n = (uint64_T)y;
        } else {
          n = MAX_uint64_T;
        }
        y = muDoubleScalarRound(r);
        if (y < 1.8446744073709552E+19) {
          if (y >= 0.0) {
            b_k = (uint64_T)y;
          } else {
            b_k = 0ULL;
          }
        } else {
          b_k = 0ULL;
        }
        yint = 1ULL;
        c_st.site = &jeb_emlrtRSI;
        if (b_k > 18446744073709551614ULL) {
          d_st.site = &k_emlrtRSI;
          b_check_forloop_overflow_error(&d_st);
        }
        b_i = 1ULL;
        exitg1 = false;
        while ((!exitg1) && (b_i <= b_k)) {
          uint64_T q_tmp;
          q_tmp = yint / b_i;
          if (n == 0ULL) {
            emlrtDivisionByZeroErrorR2012b(&ql_emlrtRTEI, &b_st);
          } else {
            u = MAX_uint64_T / n;
          }
          if (q_tmp >= u) {
            yint = MAX_uint64_T;
            exitg1 = true;
          } else {
            yint = q_tmp * n + (yint - q_tmp * b_i) * n / b_i;
            n--;
            b_i++;
          }
        }
        y = (real_T)yint;
        maxRelErr = 0.0;
      }
      if (y > 9.007199254740992E+15) {
        maxRelErr = muDoubleScalarMax(maxRelErr, 2.2204460492503131E-16);
      }
      if ((maxRelErr != 0.0) &&
          ((!muDoubleScalarIsInf(y)) && (!muDoubleScalarIsNaN(y))) &&
          (!emlrtSetWarningFlag(&st))) {
        char_T b_str[23];
        char_T str[23];
        b_st.site = &heb_emlrtRSI;
        b_y = NULL;
        m = emlrtCreateCharArray(2, &b_iv[0]);
        emlrtInitCharArrayR2013a(&b_st, 7, m, &rfmt[0]);
        emlrtAssign(&b_y, m);
        c_y = NULL;
        m = emlrtCreateDoubleScalar(maxRelErr);
        emlrtAssign(&c_y, m);
        c_st.site = &yob_emlrtRSI;
        emlrt_marshallIn(&c_st, b_sprintf(&c_st, b_y, c_y, &d_emlrtMCI),
                         "<output of sprintf>", str);
        b_st.site = &heb_emlrtRSI;
        d_y = NULL;
        m = emlrtCreateCharArray(2, &b_iv1[0]);
        emlrtInitCharArrayR2013a(&b_st, 7, m, &rfmt[0]);
        emlrtAssign(&d_y, m);
        e_y = NULL;
        m = emlrtCreateDoubleScalar(muDoubleScalarCeil(maxRelErr * y));
        emlrtAssign(&e_y, m);
        c_st.site = &yob_emlrtRSI;
        emlrt_marshallIn(&c_st, b_sprintf(&c_st, d_y, e_y, &d_emlrtMCI),
                         "<output of sprintf>", b_str);
        b_st.site = &geb_emlrtRSI;
        d_warning(&b_st, str, b_str);
      }
    }
  }
  return y;
}

/* End of code generation (nchoosek.c) */
