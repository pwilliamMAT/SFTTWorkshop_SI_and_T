/*
 * fusionAlgorithm_terminate.c
 *
 * Code generation for function 'fusionAlgorithm_terminate'
 *
 */

/* Include files */
#include "fusionAlgorithm_terminate.h"
#include "_coder_fusionAlgorithm_mex.h"
#include "fusionAlgorithm.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Declarations */
static void c_fusionAlgorithm_emx_free_dtor(const emlrtStack *sp,
                                            const void *r);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void c_fusionAlgorithm_emx_free_dtor(const emlrtStack *sp, const void *r)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = NULL;
  fusionAlgorithm_emx_free(&st);
}

static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void fusionAlgorithm_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025b\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&fusionAlgorithm_nestLockGlobal);
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtPushHeapReferenceStackR2021a(&st, true, NULL,
                                      (void *)&c_fusionAlgorithm_emx_free_dtor,
                                      NULL, NULL, NULL);
    fusionAlgorithm_delete();
    fusionAlgorithm_emx_free(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&fusionAlgorithm_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&fusionAlgorithm_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

void fusionAlgorithm_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (fusionAlgorithm_terminate.c) */
