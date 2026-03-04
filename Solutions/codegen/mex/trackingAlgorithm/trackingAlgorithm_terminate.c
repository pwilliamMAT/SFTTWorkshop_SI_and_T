/*
 * trackingAlgorithm_terminate.c
 *
 * Code generation for function 'trackingAlgorithm_terminate'
 *
 */

/* Include files */
#include "trackingAlgorithm_terminate.h"
#include "_coder_trackingAlgorithm_mex.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm.h"
#include "trackingAlgorithm_data.h"
#include "omp.h"

/* Function Declarations */
static void c_trackingAlgorithm_emx_free_dt(const emlrtStack *sp,
                                            const void *r);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void c_trackingAlgorithm_emx_free_dt(const emlrtStack *sp, const void *r)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = NULL;
  trackingAlgorithm_emx_free(&st);
}

static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void trackingAlgorithm_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  emlrtLoadMATLABLibrary("sys/os/glnxa64/libiomp5.so");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&trackingAlgorithm_nestLockGlobal);
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtPushHeapReferenceStackR2021a(&st, true, NULL,
                                      (void *)&c_trackingAlgorithm_emx_free_dt,
                                      NULL, NULL, NULL);
    trackingAlgorithm_emx_free(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&trackingAlgorithm_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&trackingAlgorithm_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

void trackingAlgorithm_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (trackingAlgorithm_terminate.c) */
