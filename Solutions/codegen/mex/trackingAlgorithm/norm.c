/*
 * norm.c
 *
 * Code generation for function 'norm'
 *
 */

/* Include files */
#include "norm.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
real_T b_norm(const real_T x[9])
{
  real_T y;
  int32_T j;
  boolean_T exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 3)) {
    real_T s;
    s = (muDoubleScalarAbs(x[3 * j]) + muDoubleScalarAbs(x[3 * j + 1])) +
        muDoubleScalarAbs(x[3 * j + 2]);
    if (muDoubleScalarIsNaN(s)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }
      j++;
    }
  }
  return y;
}

/* End of code generation (norm.c) */
