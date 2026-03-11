/*
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "assertValidSizeArg.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo dj_emlrtRSI = {
    153,      /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRTEInfo td_emlrtRTEI = {
    106,      /* lineNo */
    25,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

/* Function Definitions */
void repmat(const emlrtStack *sp, const objectTrack *a,
            const real_T varargin_1[2], emxArray_objectTrack *b)
{
  emlrtStack b_st;
  emlrtStack st;
  objectTrack *b_data;
  int32_T i;
  int32_T i1;
  int32_T jtilecol;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &cj_emlrtRSI;
  assertValidSizeArg(&st, varargin_1);
  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  i1 = (int32_T)varargin_1[1];
  b->size[1] = (int32_T)varargin_1[1];
  emxEnsureCapacity_objectTrack(sp, b, i, &td_emlrtRTEI);
  b_data = b->data;
  if ((int32_T)varargin_1[1] != 0) {
    st.site = &dj_emlrtRSI;
    if ((int32_T)varargin_1[1] > 2147483646) {
      b_st.site = &tb_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (jtilecol = 0; jtilecol < i1; jtilecol++) {
      emxCopyStruct_objectTrack(sp, &b_data[jtilecol], a, &sc_emlrtRTEI);
    }
  }
}

/* End of code generation (repmat.c) */
