/*
 * minPriorityQueue.c
 *
 * Code generation for function 'minPriorityQueue'
 *
 */

/* Include files */
#include "minPriorityQueue.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo bo_emlrtRSI = {
    86,                        /* lineNo */
    "minPriorityQueue/percUp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtRSInfo co_emlrtRSI = {
    88,                        /* lineNo */
    "minPriorityQueue/percUp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m" /* pathName */
};

static emlrtBCInfo wi_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    87,                        /* lineNo */
    50,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/percUp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo xi_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    87,                        /* lineNo */
    26,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/percUp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo yi_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    88,                        /* lineNo */
    83,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/percUp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo aj_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    88,                        /* lineNo */
    42,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/percUp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo bj_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    88,                        /* lineNo */
    17,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/percUp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo cj_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    88,                        /* lineNo */
    74,                        /* colNo */
    "",                        /* aName */
    "minPriorityQueue/percUp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

/* Function Definitions */
void minPriorityQueue_percUp(const emlrtStack *sp,
                             c_matlab_internal_coder_minPrio *obj, int32_T i,
                             const emxArray_real_T *dist)
{
  emlrtStack st;
  const real_T *dist_data;
  int32_T iparent;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  dist_data = dist->data;
  iparent = i / 2;
  exitg1 = false;
  while ((!exitg1) && (iparent > 0)) {
    real_T d;
    real_T d1;
    int32_T b_obj;
    int32_T c_obj;
    int32_T obj_idx_1;
    boolean_T tf;
    st.site = &bo_emlrtRSI;
    b_obj = obj->heap->size[0];
    if ((i < 1) || (i > b_obj)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, b_obj, &kc_emlrtBCI, &st);
    }
    if (iparent > b_obj) {
      emlrtDynamicBoundsCheckR2012b(iparent, 1, b_obj, &jc_emlrtBCI, &st);
    }
    obj_idx_1 = obj->heap->data[i - 1];
    if ((obj_idx_1 < 1) || (obj_idx_1 > dist->size[0])) {
      emlrtDynamicBoundsCheckR2012b(obj->heap->data[i - 1], 1, dist->size[0],
                                    &qc_emlrtBCI, &st);
    }
    c_obj = obj->heap->data[iparent - 1];
    if ((c_obj < 1) || (c_obj > dist->size[0])) {
      emlrtDynamicBoundsCheckR2012b(obj->heap->data[iparent - 1], 1,
                                    dist->size[0], &rc_emlrtBCI, &st);
    }
    d = dist_data[obj_idx_1 - 1];
    d1 = dist_data[c_obj - 1];
    if (d < d1) {
      tf = true;
    } else {
      if (obj_idx_1 > dist->size[0]) {
        emlrtDynamicBoundsCheckR2012b(obj->heap->data[i - 1], 1, dist->size[0],
                                      &sc_emlrtBCI, &st);
      }
      if (c_obj > dist->size[0]) {
        emlrtDynamicBoundsCheckR2012b(obj->heap->data[iparent - 1], 1,
                                      dist->size[0], &tc_emlrtBCI, &st);
      }
      if ((d == d1) && (obj_idx_1 <= c_obj)) {
        tf = true;
      } else {
        tf = false;
      }
    }
    if (tf) {
      if (iparent > b_obj) {
        emlrtDynamicBoundsCheckR2012b(iparent, 1, b_obj, &wi_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i > b_obj) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_obj, &wi_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i > b_obj) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_obj, &xi_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (iparent > b_obj) {
        emlrtDynamicBoundsCheckR2012b(iparent, 1, b_obj, &xi_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      obj_idx_1 = obj->heap->data[i - 1];
      obj->heap->data[i - 1] = obj->heap->data[iparent - 1];
      obj->heap->data[iparent - 1] = obj_idx_1;
      st.site = &co_emlrtRSI;
      b_indexShapeCheck(&st, obj->heap->size[0]);
      if (iparent > b_obj) {
        emlrtDynamicBoundsCheckR2012b(iparent, 1, b_obj, &yi_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (i > b_obj) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_obj, &yi_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      c_obj = obj->indexToHeap->size[0];
      if (i > b_obj) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_obj, &aj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (iparent > b_obj) {
        emlrtDynamicBoundsCheckR2012b(iparent, 1, b_obj, &aj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if ((obj->heap->data[iparent - 1] < 1) ||
          (obj->heap->data[iparent - 1] > c_obj)) {
        emlrtDynamicBoundsCheckR2012b(obj->heap->data[iparent - 1], 1, c_obj,
                                      &cj_emlrtBCI, (emlrtConstCTX)sp);
      }
      if ((obj->heap->data[i - 1] < 1) || (obj->heap->data[i - 1] > c_obj)) {
        emlrtDynamicBoundsCheckR2012b(obj->heap->data[i - 1], 1, c_obj,
                                      &cj_emlrtBCI, (emlrtConstCTX)sp);
      }
      obj_idx_1 = obj->indexToHeap->data[obj->heap->data[i - 1] - 1];
      if ((obj->heap->data[i - 1] < 1) || (obj->heap->data[i - 1] > c_obj)) {
        emlrtDynamicBoundsCheckR2012b(obj->heap->data[i - 1], 1, c_obj,
                                      &bj_emlrtBCI, (emlrtConstCTX)sp);
      }
      obj->indexToHeap->data[obj->heap->data[i - 1] - 1] =
          obj->indexToHeap->data[obj->heap->data[iparent - 1] - 1];
      if ((obj->heap->data[iparent - 1] < 1) ||
          (obj->heap->data[iparent - 1] > c_obj)) {
        emlrtDynamicBoundsCheckR2012b(obj->heap->data[iparent - 1], 1, c_obj,
                                      &bj_emlrtBCI, (emlrtConstCTX)sp);
      }
      obj->indexToHeap->data[obj->heap->data[iparent - 1] - 1] = obj_idx_1;
      i = iparent;
      iparent = (int32_T)((uint32_T)iparent >> 1);
    } else {
      exitg1 = true;
    }
  }
}

/* End of code generation (minPriorityQueue.c) */
