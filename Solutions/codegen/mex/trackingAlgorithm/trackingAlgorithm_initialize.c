/*
 * trackingAlgorithm_initialize.c
 *
 * Code generation for function 'trackingAlgorithm_initialize'
 *
 */

/* Include files */
#include "trackingAlgorithm_initialize.h"
#include "_coder_trackingAlgorithm_mex.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm.h"
#include "trackingAlgorithm_data.h"

/* Function Declarations */
static void trackingAlgorithm_once(const emlrtStack *sp);

/* Function Definitions */
static void trackingAlgorithm_once(const emlrtStack *sp)
{
  emlrtStack st;
  mex_InitInfAndNan();
  st.prev = sp;
  st.tls = sp->tls;
  st.site = NULL;
  trackingAlgorithm_emx_init(&st);
  trackingAlgorithm_init();
}

void trackingAlgorithm_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtLicenseCheckR2022a(&st, "EMLRT:runTime:MexFunctionNeedsLicense",
                          "sensor_fusion_and_tracking", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    trackingAlgorithm_once(&st);
  }
}

/* End of code generation (trackingAlgorithm_initialize.c) */
