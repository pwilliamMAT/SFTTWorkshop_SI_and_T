/*
 * fusionAlgorithm_initialize.c
 *
 * Code generation for function 'fusionAlgorithm_initialize'
 *
 */

/* Include files */
#include "fusionAlgorithm_initialize.h"
#include "_coder_fusionAlgorithm_mex.h"
#include "fusionAlgorithm.h"
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void fusionAlgorithm_once(const emlrtStack *sp);

/* Function Definitions */
static void fusionAlgorithm_once(const emlrtStack *sp)
{
  emlrtStack st;
  mex_InitInfAndNan();
  st.prev = sp;
  st.tls = sp->tls;
  st.site = NULL;
  fusionAlgorithm_emx_init(&st);
  fusionAlgorithm_new();
  fusionAlgorithm_init();
}

void fusionAlgorithm_initialize(void)
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
  emlrtLicenseCheckR2022a(&st, "EMLRT:runTime:MexFunctionNeedsLicense",
                          "map_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    fusionAlgorithm_once(&st);
  }
}

/* End of code generation (fusionAlgorithm_initialize.c) */
