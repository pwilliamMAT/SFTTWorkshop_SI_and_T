/*
 * logsumexp.c
 *
 * Code generation for function 'logsumexp'
 *
 */

/* Include files */
#include "logsumexp.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo en_emlrtRSI = {
    255,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo fn_emlrtRSI = {
    966,                    /* lineNo */
    "maxRealVectorOmitNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo rib_emlrtRSI = {
    12,          /* lineNo */
    "logsumexp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\logsumexp.m" /* pathName */
};

static emlrtRSInfo sib_emlrtRSI = {
    17,          /* lineNo */
    "logsumexp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\logsumexp.m" /* pathName */
};

static emlrtRTEInfo ei_emlrtRTEI = {
    16,          /* lineNo */
    5,           /* colNo */
    "logsumexp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\logsumexp.m" /* pName */
};

/* Function Definitions */
real_T logsumexp(const emlrtStack *sp, const emxArray_real_T *x)
{
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  emxArray_real_T *xdiff;
  const real_T *x_data;
  real_T b_x;
  real_T s;
  real_T *xdiff_data;
  int32_T b_k;
  int32_T idx;
  int32_T k;
  int32_T last;
  int32_T logsumexp_numThreads;
  int32_T nx;
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &rib_emlrtRSI;
  b_st.site = &bn_emlrtRSI;
  c_st.site = &cn_emlrtRSI;
  d_st.site = &dn_emlrtRSI;
  if (x->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &xb_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  e_st.site = &en_emlrtRSI;
  f_st.site = &fn_emlrtRSI;
  last = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      s = x_data[0];
    } else if ((x_data[0] < x_data[1]) || (muDoubleScalarIsNaN(x_data[0]) &&
                                           (!muDoubleScalarIsNaN(x_data[1])))) {
      s = x_data[1];
    } else {
      s = x_data[0];
    }
  } else {
    g_st.site = &hn_emlrtRSI;
    if (!muDoubleScalarIsNaN(x_data[0])) {
      idx = 1;
    } else {
      boolean_T exitg1;
      idx = 0;
      h_st.site = &in_emlrtRSI;
      if (x->size[0] > 2147483646) {
        i_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      nx = 2;
      exitg1 = false;
      while ((!exitg1) && (nx <= last)) {
        if (!muDoubleScalarIsNaN(x_data[nx - 1])) {
          idx = nx;
          exitg1 = true;
        } else {
          nx++;
        }
      }
    }
    if (idx == 0) {
      s = x_data[0];
    } else {
      g_st.site = &gn_emlrtRSI;
      s = x_data[idx - 1];
      nx = idx + 1;
      h_st.site = &jn_emlrtRSI;
      if ((idx + 1 <= x->size[0]) && (x->size[0] > 2147483646)) {
        i_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (k = nx; k <= last; k++) {
        b_x = x_data[k - 1];
        if (s < b_x) {
          s = b_x;
        }
      }
    }
  }
  if (!muDoubleScalarIsInf(s)) {
    emxInit_real_T(sp, &xdiff, 1, &ei_emlrtRTEI, true);
    nx = xdiff->size[0];
    xdiff->size[0] = x->size[0];
    emxEnsureCapacity_real_T(sp, xdiff, nx, &ei_emlrtRTEI);
    xdiff_data = xdiff->data;
    nx = (x->size[0] / 2) << 1;
    idx = nx - 2;
    for (k = 0; k <= idx; k += 2) {
      _mm_storeu_pd(&xdiff_data[k],
                    _mm_sub_pd(_mm_loadu_pd(&x_data[k]), _mm_set1_pd(s)));
    }
    for (k = nx; k < last; k++) {
      xdiff_data[k] = x_data[k] - s;
    }
    st.site = &sib_emlrtRSI;
    b_st.site = &tib_emlrtRSI;
    nx = xdiff->size[0];
    c_st.site = &xr_emlrtRSI;
    if (xdiff->size[0] > 2147483646) {
      d_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
    if (xdiff->size[0] < 800) {
      for (b_k = 0; b_k < nx; b_k++) {
        xdiff_data[b_k] = muDoubleScalarExp(xdiff_data[b_k]);
      }
    } else {
      emlrtEnterParallelRegion(&b_st, omp_in_parallel());
      emlrtPushJmpBuf(&b_st, &emlrtJBStack);
      logsumexp_numThreads =
          emlrtAllocRegionTLSs(b_st.tls, omp_in_parallel(),
                               omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(logsumexp_numThreads)

      for (b_k = 0; b_k < nx; b_k++) {
        xdiff_data[b_k] = muDoubleScalarExp(xdiff_data[b_k]);
      }
      emlrtPopJmpBuf(&b_st, &emlrtJBStack);
      emlrtExitParallelRegion(&b_st, omp_in_parallel());
    }
    st.site = &sib_emlrtRSI;
    b_st.site = &sib_emlrtRSI;
    b_x = c_sum(&b_st, xdiff);
    emxFree_real_T(&st, &xdiff);
    if (b_x < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &st, &x_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
          "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
    }
    b_x = muDoubleScalarLog(b_x);
    s += b_x;
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return s;
}

/* End of code generation (logsumexp.c) */
