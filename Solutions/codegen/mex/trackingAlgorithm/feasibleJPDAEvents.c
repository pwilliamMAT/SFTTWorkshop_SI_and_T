/*
 * feasibleJPDAEvents.c
 *
 * Code generation for function 'feasibleJPDAEvents'
 *
 */

/* Include files */
#include "feasibleJPDAEvents.h"
#include "any.h"
#include "eml_int_forloop_overflow_check.h"
#include "find.h"
#include "numPotentialFeasibleEvents.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo tp_emlrtRSI = {
    78,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRSInfo up_emlrtRSI = {
    80,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRSInfo cjb_emlrtRSI = {
    48,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo djb_emlrtRSI = {
    50,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo ejb_emlrtRSI = {
    51,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo fjb_emlrtRSI = {
    60,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo gjb_emlrtRSI = {
    61,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo hjb_emlrtRSI = {
    63,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo ijb_emlrtRSI = {
    86,                   /* lineNo */
    "feasibleJPDAEvents", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pathName */
};

static emlrtRSInfo kjb_emlrtRSI = {
    85,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRSInfo ljb_emlrtRSI = {
    39,                                   /* lineNo */
    "StrictSingleCoderUtilities/IntFind", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\+codegen\\"
    "StrictSingleCoderUtilities.m" /* pathName */
};

static emlrtBCInfo wf_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    115,                  /* lineNo */
    20,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo xf_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    115,                  /* lineNo */
    18,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo yf_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    61,                   /* lineNo */
    100,                  /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo ag_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    80,                   /* lineNo */
    18,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo bg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    80,                   /* lineNo */
    20,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo cg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    108,                  /* lineNo */
    15,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo dg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    111,                  /* lineNo */
    11,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo eg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    89,                   /* lineNo */
    19,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo fg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    90,                   /* lineNo */
    19,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo gg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    97,                   /* lineNo */
    25,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo hg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    97,                   /* lineNo */
    27,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo ig_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    97,                   /* lineNo */
    34,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo jg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    98,                   /* lineNo */
    25,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo kg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    98,                   /* lineNo */
    32,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo lg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    98,                   /* lineNo */
    41,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo mg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    67,                   /* lineNo */
    31,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo ng_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    67,                   /* lineNo */
    17,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo og_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    67,                   /* lineNo */
    19,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo pg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    65,                   /* lineNo */
    17,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtBCInfo qg_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    65,                   /* lineNo */
    19,                   /* colNo */
    "",                   /* aName */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m", /* pName */
    0                                              /* checkKind */
};

static emlrtRTEInfo me_emlrtRTEI = {
    73,       /* lineNo */
    28,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRTEInfo fi_emlrtRTEI = {
    50,                   /* lineNo */
    1,                    /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

static emlrtRTEInfo gi_emlrtRTEI = {
    58,                   /* lineNo */
    1,                    /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

static emlrtRTEInfo hi_emlrtRTEI = {
    61,                   /* lineNo */
    83,                   /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

static emlrtRTEInfo ii_emlrtRTEI = {
    115,                  /* lineNo */
    1,                    /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

static emlrtRTEInfo ji_emlrtRTEI = {
    61,                   /* lineNo */
    5,                    /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

static emlrtRTEInfo ki_emlrtRTEI = {
    63,                   /* lineNo */
    20,                   /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

static emlrtRTEInfo li_emlrtRTEI = {
    51,                   /* lineNo */
    1,                    /* colNo */
    "feasibleJPDAEvents", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\feasibleJPDAEvents.m" /* pName */
};

/* Function Definitions */
void feasibleJPDAEvents(const emlrtStack *sp,
                        const emxArray_boolean_T *validationMatrix,
                        emxArray_boolean_T *outFJE)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T c_X_data;
  emxArray_boolean_T *FJE;
  emxArray_boolean_T *b_validationMatrix;
  emxArray_boolean_T *omega;
  emxArray_int32_T *b_ii;
  emxArray_int32_T *ii;
  emxArray_uint32_T *Xjs;
  int32_T X_size;
  int32_T X_tmp;
  int32_T iacol;
  int32_T ibmat;
  int32_T ibtile;
  int32_T jcol;
  int32_T jtilecol;
  int32_T k;
  int32_T loop_ub;
  int32_T nPotentials;
  int32_T numMeas;
  int32_T numTrks;
  int32_T *b_ii_data;
  int32_T *ii_data;
  uint32_T X_data[104];
  uint32_T L;
  uint32_T Xj;
  uint32_T j;
  uint32_T *Xjs_data;
  boolean_T b_X_data[52];
  const boolean_T *validationMatrix_data;
  boolean_T *FJE_data;
  boolean_T *omega_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  validationMatrix_data = validationMatrix->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  numMeas = validationMatrix->size[0];
  numTrks = validationMatrix->size[1] - 1;
  st.site = &cjb_emlrtRSI;
  nPotentials = numPotentialFeasibleEvents(&st, validationMatrix,
                                           validationMatrix->size[0],
                                           validationMatrix->size[1] - 1);
  st.site = &djb_emlrtRSI;
  b_st.site = &jjb_emlrtRSI;
  emxInit_boolean_T(&b_st, &omega, 2, &fi_emlrtRTEI, true);
  ibtile = omega->size[0] * omega->size[1];
  omega->size[0] = validationMatrix->size[0];
  loop_ub = validationMatrix->size[1];
  omega->size[1] = validationMatrix->size[1];
  emxEnsureCapacity_boolean_T(&b_st, omega, ibtile, &fi_emlrtRTEI);
  omega_data = omega->data;
  for (jcol = 0; jcol < numMeas; jcol++) {
    omega_data[jcol] = true;
  }
  for (jcol = 0; jcol < numTrks; jcol++) {
    for (jtilecol = 0; jtilecol < numMeas; jtilecol++) {
      omega_data[jtilecol + omega->size[0] * (jcol + 1)] = false;
    }
  }
  st.site = &ejb_emlrtRSI;
  b_st.site = &sp_emlrtRSI;
  if (nPotentials < 0) {
    emlrtNonNegativeCheckR2012b(nPotentials, &g_emlrtDCI, &st);
  }
  emxInit_boolean_T(&st, &FJE, 3, &li_emlrtRTEI, true);
  ibtile = FJE->size[0] * FJE->size[1] * FJE->size[2];
  FJE->size[0] = validationMatrix->size[0];
  FJE->size[1] = validationMatrix->size[1];
  FJE->size[2] = nPotentials;
  emxEnsureCapacity_boolean_T(&st, FJE, ibtile, &me_emlrtRTEI);
  FJE_data = FJE->data;
  b_st.site = &tp_emlrtRSI;
  if (nPotentials > 2147483646) {
    c_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (jtilecol = 0; jtilecol < nPotentials; jtilecol++) {
    ibtile = jtilecol * (numMeas * loop_ub) - 1;
    b_st.site = &up_emlrtRSI;
    if (loop_ub > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (jcol = 0; jcol < loop_ub; jcol++) {
      iacol = jcol * numMeas;
      ibmat = ibtile + iacol;
      b_st.site = &kjb_emlrtRSI;
      for (k = 0; k < numMeas; k++) {
        FJE_data[(ibmat + k) + 1] = omega_data[iacol + k];
      }
    }
  }
  emxFree_boolean_T(&st, &omega);
  Xj = 0U;
  X_tmp = muIntScalarMin_sint32(numTrks, numMeas) + 1;
  ibtile = X_tmp << 1;
  if (ibtile - 1 >= 0) {
    memset(&X_data[0], 0, (uint32_T)ibtile * sizeof(uint32_T));
  }
  emxInit_uint32_T(sp, &Xjs, 2, &gi_emlrtRTEI, true);
  ibtile = Xjs->size[0] * Xjs->size[1];
  Xjs->size[0] = validationMatrix->size[0];
  Xjs->size[1] = validationMatrix->size[1];
  emxEnsureCapacity_uint32_T(sp, Xjs, ibtile, &gi_emlrtRTEI);
  Xjs_data = Xjs->data;
  ibtile = validationMatrix->size[0] * validationMatrix->size[1];
  for (jcol = 0; jcol < ibtile; jcol++) {
    Xjs_data[jcol] = 0U;
  }
  st.site = &fjb_emlrtRSI;
  emxInit_int32_T(sp, &ii, 2, &ke_emlrtRTEI);
  emxInit_int32_T(sp, &b_ii, 2, &ke_emlrtRTEI);
  emxInit_boolean_T(sp, &b_validationMatrix, 2, &hi_emlrtRTEI, true);
  for (k = 0; k < numMeas; k++) {
    st.site = &gjb_emlrtRSI;
    if (k + 1 > numMeas) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, numMeas, &yf_emlrtBCI, &st);
    }
    b_st.site = &ljb_emlrtRSI;
    ibtile = b_validationMatrix->size[0] * b_validationMatrix->size[1];
    b_validationMatrix->size[0] = 1;
    b_validationMatrix->size[1] = loop_ub;
    emxEnsureCapacity_boolean_T(&b_st, b_validationMatrix, ibtile,
                                &hi_emlrtRTEI);
    omega_data = b_validationMatrix->data;
    for (jcol = 0; jcol < loop_ub; jcol++) {
      omega_data[jcol] =
          validationMatrix_data[k + validationMatrix->size[0] * jcol];
    }
    c_st.site = &wbb_emlrtRSI;
    eml_find(&c_st, b_validationMatrix, b_ii);
    ibtile = b_ii->size[0] * b_ii->size[1];
    b_ii->size[0] = 1;
    emxEnsureCapacity_int32_T(sp, b_ii, ibtile, &ji_emlrtRTEI);
    ii_data = b_ii->data;
    ibtile = b_ii->size[1] - 1;
    iacol = (b_ii->size[1] / 4) << 2;
    ibmat = iacol - 4;
    for (jtilecol = 0; jtilecol <= ibmat; jtilecol += 4) {
      __m128i r;
      r = _mm_loadu_si128((const __m128i *)&ii_data[jtilecol]);
      _mm_storeu_si128((__m128i *)&ii_data[jtilecol],
                       _mm_sub_epi32(r, _mm_set1_epi32(1)));
    }
    for (jtilecol = iacol; jtilecol <= ibtile; jtilecol++) {
      ii_data[jtilecol]--;
    }
    iacol = b_ii->size[1];
    for (jcol = 0; jcol <= numTrks; jcol++) {
      st.site = &hjb_emlrtRSI;
      ibtile = b_validationMatrix->size[0] * b_validationMatrix->size[1];
      b_validationMatrix->size[0] = 1;
      b_validationMatrix->size[1] = iacol;
      emxEnsureCapacity_boolean_T(&st, b_validationMatrix, ibtile,
                                  &ki_emlrtRTEI);
      omega_data = b_validationMatrix->data;
      for (jtilecol = 0; jtilecol < iacol; jtilecol++) {
        omega_data[jtilecol] = (ii_data[jtilecol] > jcol);
      }
      b_st.site = &wbb_emlrtRSI;
      eml_find(&b_st, b_validationMatrix, ii);
      b_ii_data = ii->data;
      if (ii->size[1] == 0) {
        if (k + 1 > Xjs->size[0]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, Xjs->size[0], &pg_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        if (jcol + 1 > Xjs->size[1]) {
          emlrtDynamicBoundsCheckR2012b(jcol + 1, 1, Xjs->size[1], &qg_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xjs_data[k + Xjs->size[0] * jcol] = 0U;
      } else {
        if ((b_ii_data[0] < 1) || (b_ii_data[0] > iacol)) {
          emlrtDynamicBoundsCheckR2012b(b_ii_data[0], 1, iacol, &mg_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        if (k + 1 > Xjs->size[0]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, Xjs->size[0], &ng_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        if (jcol + 1 > Xjs->size[1]) {
          emlrtDynamicBoundsCheckR2012b(jcol + 1, 1, Xjs->size[1], &og_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        ibtile = ii_data[b_ii_data[0] - 1];
        if (ibtile < 0) {
          ibtile = 0;
        }
        Xjs_data[k + Xjs->size[0] * jcol] = (uint32_T)ibtile;
      }
    }
  }
  emxFree_boolean_T(sp, &b_validationMatrix);
  emxFree_int32_T(sp, &b_ii);
  emxFree_int32_T(sp, &ii);
  L = 0U;
  j = 1U;
  nPotentials = 1;
  while (j <= (uint32_T)numMeas) {
    boolean_T b;
    while ((L < (uint32_T)(X_tmp - 1)) && (j <= (uint32_T)numMeas)) {
      uint32_T qY;
      if ((j < 1U) || ((int32_T)j > Xjs->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)j, 1, Xjs->size[0], &ag_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      qY = Xj + 1U;
      if (Xj + 1U < Xj) {
        qY = MAX_uint32_T;
      }
      if (((int32_T)qY < 1) || ((int32_T)qY > Xjs->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)qY, 1, Xjs->size[1],
                                      &bg_emlrtBCI, (emlrtConstCTX)sp);
      }
      Xj = Xjs_data[((int32_T)j + Xjs->size[0] * ((int32_T)qY - 1)) - 1];
      if (Xj == 0U) {
        Xj = 0U;
        j++;
      } else {
        X_size = X_tmp;
        for (jcol = 0; jcol < X_tmp; jcol++) {
          b_X_data[jcol] = (X_data[jcol + X_tmp] == Xj);
        }
        c_X_data.data = &b_X_data[0];
        c_X_data.size = &X_size;
        c_X_data.allocatedSize = 52;
        c_X_data.numDimensions = 1;
        c_X_data.canFreeData = false;
        st.site = &ijb_emlrtRSI;
        if (!any(&st, &c_X_data)) {
          L++;
          b = (((int32_T)L < 1) || ((int32_T)L > X_tmp));
          if (b) {
            emlrtDynamicBoundsCheckR2012b((int32_T)L, 1, X_tmp, &eg_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          X_data[(int32_T)L - 1] = j;
          if (b) {
            emlrtDynamicBoundsCheckR2012b((int32_T)L, 1, X_tmp, &fg_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          X_data[((int32_T)L + X_tmp) - 1] = Xj;
          j++;
          nPotentials++;
          ibtile = (int32_T)L;
          for (jcol = 0; jcol < ibtile; jcol++) {
            if (((int32_T)((uint32_T)jcol + 1U) < 1) ||
                ((int32_T)((uint32_T)jcol + 1U) > X_tmp)) {
              emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)jcol + 1U), 1,
                                            X_tmp, &hg_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            iacol = (int32_T)X_data[jcol];
            if ((iacol < 1) || (iacol > FJE->size[0])) {
              emlrtDynamicBoundsCheckR2012b(iacol, 1, FJE->size[0],
                                            &gg_emlrtBCI, (emlrtConstCTX)sp);
            }
            if ((nPotentials < 1) || (nPotentials > FJE->size[2])) {
              emlrtDynamicBoundsCheckR2012b(nPotentials, 1, FJE->size[2],
                                            &ig_emlrtBCI, (emlrtConstCTX)sp);
            }
            FJE_data[(iacol + FJE->size[0] * FJE->size[1] * (nPotentials - 1)) -
                     1] = false;
            if (iacol > FJE->size[0]) {
              emlrtDynamicBoundsCheckR2012b(iacol, 1, FJE->size[0],
                                            &jg_emlrtBCI, (emlrtConstCTX)sp);
            }
            Xj = X_data[jcol + X_tmp];
            qY = Xj + 1U;
            if (Xj + 1U < Xj) {
              qY = MAX_uint32_T;
            }
            if (((int32_T)qY < 1) || ((int32_T)qY > FJE->size[1])) {
              emlrtDynamicBoundsCheckR2012b((int32_T)qY, 1, FJE->size[1],
                                            &kg_emlrtBCI, (emlrtConstCTX)sp);
            }
            if (nPotentials > FJE->size[2]) {
              emlrtDynamicBoundsCheckR2012b(nPotentials, 1, FJE->size[2],
                                            &lg_emlrtBCI, (emlrtConstCTX)sp);
            }
            FJE_data[((iacol + FJE->size[0] * ((int32_T)qY - 1)) +
                      FJE->size[0] * FJE->size[1] * (nPotentials - 1)) -
                     1] = true;
          }
          Xj = 0U;
        }
      }
    }
    if (L >= 1U) {
      b = (((int32_T)L < 1) || ((int32_T)L > X_tmp));
      if (b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)L, 1, X_tmp, &cg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      j = X_data[(int32_T)L - 1];
      ibtile = ((int32_T)L + X_tmp) - 1;
      Xj = X_data[ibtile];
      X_data[(int32_T)L - 1] = 0U;
      if (b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)L, 1, X_tmp, &dg_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[ibtile] = 0U;
      L--;
    }
  }
  emxFree_uint32_T(sp, &Xjs);
  if (nPotentials < 1) {
    nPotentials = 0;
  } else {
    if (FJE->size[2] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, FJE->size[2], &xf_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (nPotentials > FJE->size[2]) {
      emlrtDynamicBoundsCheckR2012b(nPotentials, 1, FJE->size[2], &wf_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
  }
  iacol = FJE->size[0];
  ibtile = outFJE->size[0] * outFJE->size[1] * outFJE->size[2];
  outFJE->size[0] = FJE->size[0];
  ibmat = FJE->size[1];
  outFJE->size[1] = FJE->size[1];
  outFJE->size[2] = nPotentials;
  emxEnsureCapacity_boolean_T(sp, outFJE, ibtile, &ii_emlrtRTEI);
  omega_data = outFJE->data;
  for (jtilecol = 0; jtilecol < nPotentials; jtilecol++) {
    for (jcol = 0; jcol < ibmat; jcol++) {
      for (k = 0; k < iacol; k++) {
        omega_data[(k + outFJE->size[0] * jcol) +
                   outFJE->size[0] * outFJE->size[1] * jtilecol] =
            FJE_data[(k + FJE->size[0] * jcol) +
                     FJE->size[0] * FJE->size[1] * jtilecol];
      }
    }
  }
  emxFree_boolean_T(sp, &FJE);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (feasibleJPDAEvents.c) */
