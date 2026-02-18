/*
 * AbstractProblemSolutionPair.c
 *
 * Code generation for function 'AbstractProblemSolutionPair'
 *
 */

/* Include files */
#include "AbstractProblemSolutionPair.h"
#include "colon.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo bib_emlrtRSI = {
    50,                                           /* lineNo */
    "AbstractProblemSolutionPair/get.Assignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo cib_emlrtRSI = {
    51,                                           /* lineNo */
    "AbstractProblemSolutionPair/get.Assignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRTEInfo dh_emlrtRTEI = {
    50,                            /* lineNo */
    13,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo eh_emlrtRTEI = {
    51,                            /* lineNo */
    13,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo fh_emlrtRTEI = {
    50,                            /* lineNo */
    28,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

/* Function Definitions */
void c_AbstractProblemSolutionPair_g(const emlrtStack *sp,
                                     const emxArray_real_T *obj_RowSoln,
                                     const int32_T obj_CostSize[2],
                                     emxArray_real_T *val)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_int32_T *rowIdx;
  emxArray_int32_T *y;
  const real_T *obj_RowSoln_data;
  real_T *val_data;
  int32_T b_loop_ub;
  int32_T i;
  int32_T loop_ub;
  int32_T *rowIdx_data;
  int32_T *y_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  obj_RowSoln_data = obj_RowSoln->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_int32_T(sp, &y, 2, &fh_emlrtRTEI);
  st.site = &bib_emlrtRSI;
  b_st.site = &hdb_emlrtRSI;
  c_st.site = &idb_emlrtRSI;
  eml_integer_colon_dispatcher(&c_st, obj_CostSize[0] + obj_CostSize[1], y);
  y_data = y->data;
  emxInit_int32_T(sp, &rowIdx, 1, &dh_emlrtRTEI);
  loop_ub = y->size[1];
  b_loop_ub = rowIdx->size[0];
  rowIdx->size[0] = y->size[1];
  emxEnsureCapacity_int32_T(sp, rowIdx, b_loop_ub, &dh_emlrtRTEI);
  rowIdx_data = rowIdx->data;
  for (i = 0; i < loop_ub; i++) {
    rowIdx_data[i] = y_data[i];
  }
  st.site = &cib_emlrtRSI;
  b_st.site = &yp_emlrtRSI;
  c_st.site = &aq_emlrtRSI;
  if (obj_RowSoln->size[0] != rowIdx->size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &l_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  b_loop_ub = val->size[0] * val->size[1];
  val->size[0] = y->size[1];
  emxFree_int32_T(&b_st, &y);
  val->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, val, b_loop_ub, &eh_emlrtRTEI);
  val_data = val->data;
  for (i = 0; i < loop_ub; i++) {
    val_data[i] = rowIdx_data[i];
  }
  emxFree_int32_T(&b_st, &rowIdx);
  b_loop_ub = obj_RowSoln->size[0];
  for (i = 0; i < b_loop_ub; i++) {
    val_data[i + val->size[0]] = obj_RowSoln_data[i];
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (AbstractProblemSolutionPair.c) */
