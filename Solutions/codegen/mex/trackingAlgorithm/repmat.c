/*
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtMCInfo c_emlrtMCI = {
    53,       /* lineNo */
    5,        /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRTEInfo m_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtRTEInfo qe_emlrtRTEI = {
    67,       /* lineNo */
    9,        /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRSInfo xob_emlrtRSI = {
    53,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

/* Function Declarations */
static void b_error(const emlrtStack *sp, const mxArray *m,
                    emlrtMCInfo *location);

/* Function Definitions */
static void b_error(const emlrtStack *sp, const mxArray *m,
                    emlrtMCInfo *location)
{
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 1, &m, "error", true,
                        location);
}

void b_repmat(const emlrtStack *sp, int32_T varargin_2, emxArray_int32_T *b)
{
  int32_T i;
  int32_T i1;
  int32_T *b_data;
  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = varargin_2;
  emxEnsureCapacity_int32_T(sp, b, i, &qe_emlrtRTEI);
  b_data = b->data;
  for (i1 = 0; i1 < varargin_2; i1++) {
    b_data[i1] = 1;
  }
}

void c_repmat(const emlrtStack *sp, int32_T varargin_1, emxArray_int32_T *b)
{
  int32_T i;
  int32_T i1;
  int32_T *b_data;
  if (varargin_1 < 0) {
    emlrtNonNegativeCheckR2012b(varargin_1, &g_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = b->size[0];
  b->size[0] = varargin_1;
  emxEnsureCapacity_int32_T(sp, b, i, &qe_emlrtRTEI);
  b_data = b->data;
  for (i1 = 0; i1 < varargin_1; i1++) {
    b_data[i1] = 1;
  }
}

void repmat(const emlrtStack *sp, const real_T a_data[],
            const int32_T a_size[2], real_T varargin_2, real_T b_data[],
            int32_T b_size[2])
{
  static const int32_T b_iv[2] = {1, 15};
  static const char_T u[15] = {'M', 'A', 'T', 'L', 'A', 'B', ':', 'p',
                               'm', 'a', 'x', 's', 'i', 'z', 'e'};
  emlrtStack st;
  const mxArray *m;
  const mxArray *y;
  int32_T i;
  int32_T jcol;
  int32_T jtilecol;
  int32_T outsize_idx_1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &gq_emlrtRSI;
  if ((varargin_2 != varargin_2) || muDoubleScalarIsInf(varargin_2)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &m_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  outsize_idx_1 = a_size[1] * (int32_T)varargin_2;
  if (!(outsize_idx_1 == (real_T)a_size[1] * (real_T)(int32_T)varargin_2)) {
    y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 15, m, &u[0]);
    emlrtAssign(&y, m);
    st.site = &xob_emlrtRSI;
    b_error(&st, y, &c_emlrtMCI);
  }
  b_size[0] = 1;
  b_size[1] = outsize_idx_1;
  outsize_idx_1 = a_size[1];
  i = (int32_T)varargin_2;
  for (jtilecol = 0; jtilecol < i; jtilecol++) {
    int32_T ibtile;
    ibtile = jtilecol * outsize_idx_1;
    for (jcol = 0; jcol < outsize_idx_1; jcol++) {
      b_data[ibtile + jcol] = a_data[jcol];
    }
  }
}

/* End of code generation (repmat.c) */
