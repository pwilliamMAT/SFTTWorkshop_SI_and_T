/*
 * _coder_fusionAlgorithm_mex.c
 *
 * Code generation for function '_coder_fusionAlgorithm_mex'
 *
 */

/* Include files */
#include "_coder_fusionAlgorithm_mex.h"
#include "_coder_fusionAlgorithm_api.h"
#include "fusionAlgorithm.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_initialize.h"
#include "fusionAlgorithm_terminate.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Definitions */
void fusionAlgorithm_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                                 const mxArray *prhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        15, "fusionAlgorithm");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 15,
                        "fusionAlgorithm");
  }
  /* Call the function. */
  fusionAlgorithm_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&fusionAlgorithm_atexit);
  emlrtLoadMATLABLibrary("sys/os/glnxa64/libiomp5.so");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&fusionAlgorithm_nestLockGlobal);
  fusionAlgorithm_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    fusionAlgorithm_mexFunction(nlhs, plhs, nrhs, prhs);
    fusionAlgorithm_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&fusionAlgorithm_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&fusionAlgorithm_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "UTF-8", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_fusionAlgorithm_mex.c) */
