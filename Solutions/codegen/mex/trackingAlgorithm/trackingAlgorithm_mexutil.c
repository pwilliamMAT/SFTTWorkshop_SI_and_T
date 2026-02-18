/*
 * trackingAlgorithm_mexutil.c
 *
 * Code generation for function 'trackingAlgorithm_mexutil'
 *
 */

/* Include files */
#include "trackingAlgorithm_mexutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *m,
                         const mxArray *m1, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m2;
  pArrays[0] = m;
  pArrays[1] = m1;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m2, 2, &pArrays[0],
                               "sprintf", true, location);
}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  if (((numerator < 0) != (denominator < 0)) &&
      (numerator % denominator != 0)) {
    quotient = -1;
  } else {
    quotient = 0;
  }
  quotient += numerator / denominator;
  return quotient;
}

/* End of code generation (trackingAlgorithm_mexutil.c) */
