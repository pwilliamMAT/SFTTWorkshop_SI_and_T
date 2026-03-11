/*
 * constvel.c
 *
 * Code generation for function 'constvel'
 *
 */

/* Include files */
#include "constvel.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "validateattributes.h"
#include "mwmathutil.h"
#include "omp.h"

/* Function Definitions */
void binary_expand_op(const emlrtStack *sp, emxArray_real_T *in1,
                      const emxArray_real_T *in2, int32_T in3)
{
  jmp_buf *volatile emlrtJBStack;
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  real_T *b_in2_data;
  real_T *in1_data;
  int32_T binary_expand_op_numThreads;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in2, 2, &qj_emlrtRTEI);
  stride_0_1 = b_in2->size[0] * b_in2->size[1];
  b_in2->size[0] = 1;
  if (in1->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in1->size[1];
  }
  b_in2->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in2, stride_0_1, &qj_emlrtRTEI);
  b_in2_data = b_in2->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in1->size[1] != 1);
  if (loop_ub < 800) {
    for (i = 0; i < loop_ub; i++) {
      b_in2_data[i] =
          in2_data[in3 + 6 * (i * stride_0_1)] + in1_data[i * stride_1_1];
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    binary_expand_op_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(binary_expand_op_numThreads)

    for (i = 0; i < loop_ub; i++) {
      b_in2_data[i] =
          in2_data[in3 + 6 * (i * stride_0_1)] + in1_data[i * stride_1_1];
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  stride_0_1 = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, stride_0_1, &qj_emlrtRTEI);
  in1_data = in1->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    in1_data[i1] = b_in2_data[i1];
  }
  emxFree_real_T(sp, &b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void binary_expand_op_1(const emlrtStack *sp, emxArray_real_T *in1,
                        const emxArray_real_T *in2, int32_T in3)
{
  jmp_buf *volatile emlrtJBStack;
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  real_T *b_in2_data;
  real_T *in1_data;
  int32_T binary_expand_op_1_numThreads;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in2, 2, &pj_emlrtRTEI);
  stride_0_1 = b_in2->size[0] * b_in2->size[1];
  b_in2->size[0] = 1;
  if (in1->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in1->size[1];
  }
  b_in2->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in2, stride_0_1, &pj_emlrtRTEI);
  b_in2_data = b_in2->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in1->size[1] != 1);
  if (loop_ub < 800) {
    for (i = 0; i < loop_ub; i++) {
      b_in2_data[i] =
          in2_data[(in3 + 6 * (i * stride_0_1)) - 1] + in1_data[i * stride_1_1];
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    binary_expand_op_1_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(binary_expand_op_1_numThreads)

    for (i = 0; i < loop_ub; i++) {
      b_in2_data[i] =
          in2_data[(in3 + 6 * (i * stride_0_1)) - 1] + in1_data[i * stride_1_1];
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  stride_0_1 = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, stride_0_1, &pj_emlrtRTEI);
  in1_data = in1->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    in1_data[i1] = b_in2_data[i1];
  }
  emxFree_real_T(sp, &b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void constvel(const emlrtStack *sp, real_T state[6], const real_T varargin_1[3],
              real_T varargin_2)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T d;
  real_T d1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &bh_emlrtRSI;
  d_validateattributes(&st, state);
  st.site = &eh_emlrtRSI;
  e_validateattributes(&st, varargin_1);
  st.site = &fh_emlrtRSI;
  b_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_2) || muDoubleScalarIsNaN(varargin_2)) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:constvel:expectedFinite", 3, 4, 19, "input number 3, dt,");
  }
  st.site = &gh_emlrtRSI;
  d = state[1];
  d1 = 0.5 * (varargin_2 * varargin_2);
  state[0] = (state[0] + d * varargin_2) + d1 * varargin_1[0];
  d += varargin_1[0] * varargin_2;
  state[1] = d;
  st.site = &gh_emlrtRSI;
  d = state[3];
  state[2] = (state[2] + d * varargin_2) + d1 * varargin_1[1];
  d += varargin_1[1] * varargin_2;
  state[3] = d;
  st.site = &gh_emlrtRSI;
  d = state[5];
  state[4] = (state[4] + d * varargin_2) + d1 * varargin_1[2];
  d += varargin_1[2] * varargin_2;
  state[5] = d;
}

void plus(const emlrtStack *sp, emxArray_real_T *in1,
          const emxArray_real_T *in2)
{
  jmp_buf *volatile emlrtJBStack;
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T plus_numThreads;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &pj_emlrtRTEI);
  stride_0_1 = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 1;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }
  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, stride_0_1, &pj_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  if (loop_ub < 800) {
    for (i = 0; i < loop_ub; i++) {
      b_in1_data[i] = in1_data[i * stride_0_1] + in2_data[i * stride_1_1];
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    plus_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(plus_numThreads)

    for (i = 0; i < loop_ub; i++) {
      b_in1_data[i] = in1_data[i * stride_0_1] + in2_data[i * stride_1_1];
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  stride_0_1 = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, stride_0_1, &pj_emlrtRTEI);
  in1_data = in1->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    in1_data[i1] = b_in1_data[i1];
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (constvel.c) */
