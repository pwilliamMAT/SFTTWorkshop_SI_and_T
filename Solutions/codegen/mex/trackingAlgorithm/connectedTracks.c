/*
 * connectedTracks.c
 *
 * Code generation for function 'connectedTracks'
 *
 */

/* Include files */
#include "connectedTracks.h"
#include "colon.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo rcb_emlrtRSI = {
    26,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo scb_emlrtRSI = {
    27,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo tcb_emlrtRSI = {
    29,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo ucb_emlrtRSI = {
    30,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo vcb_emlrtRSI = {
    41,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo wcb_emlrtRSI = {
    42,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo xcb_emlrtRSI = {
    45,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo ycb_emlrtRSI = {
    60,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo adb_emlrtRSI = {
    62,                /* lineNo */
    "connectedTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pathName */
};

static emlrtRSInfo bdb_emlrtRSI = {
    13,               /* lineNo */
    "nullAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtRSInfo cdb_emlrtRSI = {
    17,               /* lineNo */
    "nullAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtRSInfo ddb_emlrtRSI = {
    152,                      /* lineNo */
    "onearg_null_assignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtBCInfo dd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    36,                /* lineNo */
    18,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtRTEInfo eb_emlrtRTEI = {
    61,                /* lineNo */
    1,                 /* colNo */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pName */
};

static emlrtECInfo m_emlrtECI = {
    -1,                /* nDims */
    62,                /* lineNo */
    1,                 /* colNo */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pName */
};

static emlrtRTEInfo gb_emlrtRTEI = {
    85,                /* lineNo */
    27,                /* colNo */
    "validate_inputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pName */
};

static emlrtBCInfo ed_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    31,                /* lineNo */
    18,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo fd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    34,                /* lineNo */
    19,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo gd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    39,                /* lineNo */
    35,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo hd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    62,                /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo id_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    43,                /* lineNo */
    22,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo jd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    43,                /* lineNo */
    26,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo kd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    43,                /* lineNo */
    51,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo ld_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    44,                /* lineNo */
    31,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo md_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    46,                /* lineNo */
    30,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo nd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    46,                /* lineNo */
    34,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo od_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    46,                /* lineNo */
    56,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo pd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    47,                /* lineNo */
    39,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtBCInfo qd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    49,                /* lineNo */
    38,                /* colNo */
    "",                /* aName */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m", /* pName */
    0                              /* checkKind */
};

static emlrtRTEInfo ne_emlrtRTEI = {
    27,                /* lineNo */
    1,                 /* colNo */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pName */
};

static emlrtRTEInfo oe_emlrtRTEI = {
    29,                /* lineNo */
    1,                 /* colNo */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pName */
};

static emlrtRTEInfo pe_emlrtRTEI = {
    41,                /* lineNo */
    13,                /* colNo */
    "connectedTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\connectedTracks.m" /* pName */
};

/* Function Definitions */
int32_T connectedTracks(const emlrtStack *sp, const emxArray_boolean_T *A,
                        int32_T clustRows_data[], int32_T clustRows_size[2],
                        emxArray_int32_T *clustCols)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  emxArray_int32_T *colStack;
  int32_T colStackSize;
  int32_T currentCol;
  int32_T k;
  int32_T n;
  int32_T numClusters;
  int32_T nz;
  int32_T row;
  int32_T track;
  int32_T vlen;
  int32_T *clustCols_data;
  int32_T *colStack_data;
  int8_T tmp_data[51];
  boolean_T unassignedDetCluster_data[51];
  const boolean_T *A_data;
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
  A_data = A->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  nz = A->size[0];
  n = A->size[1];
  st.site = &rcb_emlrtRSI;
  b_st.site = &gq_emlrtRSI;
  clustRows_size[0] = 1;
  clustRows_size[1] = A->size[0];
  if (nz - 1 >= 0) {
    memset(&clustRows_data[0], 0, (uint32_T)nz * sizeof(int32_T));
  }
  st.site = &scb_emlrtRSI;
  b_st.site = &gq_emlrtRSI;
  vlen = clustCols->size[0] * clustCols->size[1];
  clustCols->size[0] = 1;
  clustCols->size[1] = A->size[1];
  emxEnsureCapacity_int32_T(&st, clustCols, vlen, &ne_emlrtRTEI);
  clustCols_data = clustCols->data;
  for (k = 0; k < n; k++) {
    clustCols_data[k] = 0;
  }
  numClusters = 0;
  st.site = &tcb_emlrtRSI;
  b_st.site = &gq_emlrtRSI;
  emxInit_int32_T(&st, &colStack, 2, &oe_emlrtRTEI);
  vlen = colStack->size[0] * colStack->size[1];
  colStack->size[0] = 1;
  colStack->size[1] = A->size[1];
  emxEnsureCapacity_int32_T(&st, colStack, vlen, &oe_emlrtRTEI);
  colStack_data = colStack->data;
  for (k = 0; k < n; k++) {
    colStack_data[k] = 1;
  }
  st.site = &ucb_emlrtRSI;
  if (A->size[1] > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (track = 0; track < n; track++) {
    if (track + 1 > clustCols->size[1]) {
      emlrtDynamicBoundsCheckR2012b(track + 1, 1, clustCols->size[1],
                                    &ed_emlrtBCI, (emlrtConstCTX)sp);
    }
    if (clustCols_data[track] == 0) {
      numClusters++;
      if (track + 1 > clustCols->size[1]) {
        emlrtDynamicBoundsCheckR2012b(track + 1, 1, clustCols->size[1],
                                      &fd_emlrtBCI, (emlrtConstCTX)sp);
      }
      clustCols_data[track] = numClusters;
      colStackSize = 1;
      if (colStack->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, colStack->size[1], &dd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      colStack_data[0] = track + 1;
      while (colStackSize > 0) {
        vlen = colStack->size[1];
        if (colStackSize > colStack->size[1]) {
          emlrtDynamicBoundsCheckR2012b(colStackSize, 1, colStack->size[1],
                                        &gd_emlrtBCI, (emlrtConstCTX)sp);
        }
        currentCol = colStack_data[colStackSize - 1];
        colStackSize--;
        st.site = &vcb_emlrtRSI;
        b_st.site = &bdb_emlrtRSI;
        if (colStack->size[1] < 1) {
          emlrtErrorWithMessageIdR2018a(&b_st, &gb_emlrtRTEI,
                                        "MATLAB:subsdeldimmismatch",
                                        "MATLAB:subsdeldimmismatch", 0);
        }
        b_st.site = &cdb_emlrtRSI;
        c_st.site = &ddb_emlrtRSI;
        for (k = vlen; k < vlen; k++) {
          colStack_data[k - 1] = colStack_data[k];
        }
        vlen = colStack->size[0] * colStack->size[1];
        colStack->size[1]--;
        emxEnsureCapacity_int32_T(&b_st, colStack, vlen, &pe_emlrtRTEI);
        colStack_data = colStack->data;
        st.site = &wcb_emlrtRSI;
        for (row = 0; row < nz; row++) {
          if (row + 1 > nz) {
            emlrtDynamicBoundsCheckR2012b(row + 1, 1, nz, &id_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if (currentCol > n) {
            emlrtDynamicBoundsCheckR2012b(currentCol, 1, n, &jd_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if (A_data[row + A->size[0] * (currentCol - 1)]) {
            vlen = clustRows_size[1];
            if (row + 1 > vlen) {
              emlrtDynamicBoundsCheckR2012b(row + 1, 1, vlen, &kd_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            if (clustRows_data[row] == 0) {
              if (row + 1 > vlen) {
                emlrtDynamicBoundsCheckR2012b(row + 1, 1, vlen, &ld_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              clustRows_data[row] = numClusters;
              st.site = &xcb_emlrtRSI;
              if (n > 2147483646) {
                b_st.site = &k_emlrtRSI;
                check_forloop_overflow_error(&b_st);
              }
              for (k = 0; k < n; k++) {
                if (row + 1 > nz) {
                  emlrtDynamicBoundsCheckR2012b(row + 1, 1, nz, &md_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                if (k + 1 > n) {
                  emlrtDynamicBoundsCheckR2012b(k + 1, 1, n, &nd_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                if (A_data[row + A->size[0] * k]) {
                  if (k + 1 > clustCols->size[1]) {
                    emlrtDynamicBoundsCheckR2012b(k + 1, 1, clustCols->size[1],
                                                  &od_emlrtBCI,
                                                  (emlrtConstCTX)sp);
                  }
                  if (clustCols_data[k] == 0) {
                    if (k + 1 > clustCols->size[1]) {
                      emlrtDynamicBoundsCheckR2012b(
                          k + 1, 1, clustCols->size[1], &pd_emlrtBCI,
                          (emlrtConstCTX)sp);
                    }
                    clustCols_data[k] = numClusters;
                    colStackSize++;
                    if (colStackSize > colStack->size[1]) {
                      emlrtDynamicBoundsCheckR2012b(
                          colStackSize, 1, colStack->size[1], &qd_emlrtBCI,
                          (emlrtConstCTX)sp);
                    }
                    colStack_data[colStackSize - 1] = k + 1;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  currentCol = clustRows_size[1];
  for (k = 0; k < currentCol; k++) {
    unassignedDetCluster_data[k] = (clustRows_data[k] == 0);
  }
  st.site = &ycb_emlrtRSI;
  b_st.site = &edb_emlrtRSI;
  c_st.site = &mw_emlrtRSI;
  vlen = clustRows_size[1];
  if (clustRows_size[1] == 0) {
    nz = 0;
  } else {
    d_st.site = &fdb_emlrtRSI;
    nz = unassignedDetCluster_data[0];
    e_st.site = &gdb_emlrtRSI;
    if (clustRows_size[1] > 2147483646) {
      f_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }
    for (k = 2; k <= vlen; k++) {
      nz += unassignedDetCluster_data[k - 1];
    }
  }
  if (nz > clustRows_size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &eb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  colStackSize = 0;
  vlen = 0;
  for (k = 0; k < currentCol; k++) {
    if (unassignedDetCluster_data[k]) {
      colStackSize++;
      tmp_data[vlen] = (int8_T)k;
      vlen++;
    }
  }
  st.site = &adb_emlrtRSI;
  b_st.site = &hdb_emlrtRSI;
  c_st.site = &idb_emlrtRSI;
  eml_integer_colon_dispatcher(&c_st, nz, colStack);
  colStack_data = colStack->data;
  currentCol = colStack->size[1];
  if (colStackSize != colStack->size[1]) {
    emlrtSubAssignSizeCheck1dR2017a(colStackSize, colStack->size[1],
                                    &m_emlrtECI, (emlrtConstCTX)sp);
  }
  for (k = 0; k < currentCol; k++) {
    int8_T i;
    i = tmp_data[k];
    vlen = clustRows_size[1];
    if (i > vlen - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, vlen - 1, &hd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    clustRows_data[i] = colStack_data[k] + numClusters;
  }
  emxFree_int32_T(sp, &colStack);
  numClusters += nz;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return numClusters;
}

/* End of code generation (connectedTracks.c) */
