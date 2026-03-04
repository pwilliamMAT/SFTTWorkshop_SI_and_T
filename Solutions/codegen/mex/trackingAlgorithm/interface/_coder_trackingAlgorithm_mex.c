/*
 * _coder_trackingAlgorithm_mex.c
 *
 * Code generation for function '_coder_trackingAlgorithm_mex'
 *
 */

/* Include files */
#include "_coder_trackingAlgorithm_mex.h"
#include "_coder_trackingAlgorithm_api.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_initialize.h"
#include "trackingAlgorithm_terminate.h"
#include "trackingAlgorithm_types.h"
#include "omp.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  trackingAlgorithmStackData *c_trackingAlgorithmStackDataGlo = NULL;
  c_trackingAlgorithmStackDataGlo = (trackingAlgorithmStackData *)emlrtMxCalloc(
      (size_t)1, (size_t)1U * sizeof(trackingAlgorithmStackData));
  mexAtExit(&trackingAlgorithm_atexit);
  emlrtLoadMATLABLibrary("sys/os/glnxa64/libiomp5.so");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&trackingAlgorithm_nestLockGlobal);
  trackingAlgorithm_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    trackingAlgorithm_mexFunction(c_trackingAlgorithmStackDataGlo, nlhs, plhs,
                                  nrhs, prhs);
    trackingAlgorithm_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&trackingAlgorithm_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&trackingAlgorithm_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
  emlrtMxFree(c_trackingAlgorithmStackDataGlo);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "UTF-8", true);
  return emlrtRootTLSGlobal;
}

void trackingAlgorithm_mexFunction(trackingAlgorithmStackData *SD, int32_T nlhs,
                                   mxArray *plhs[1], int32_T nrhs,
                                   const mxArray *prhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        17, "trackingAlgorithm");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 17,
                        "trackingAlgorithm");
  }
  /* Call the function. */
  trackingAlgorithm_api(SD, prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_trackingAlgorithm_mex.c) */
