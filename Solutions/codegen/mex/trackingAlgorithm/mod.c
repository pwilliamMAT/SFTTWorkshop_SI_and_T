/*
 * mod.c
 *
 * Code generation for function 'mod'
 *
 */

/* Include files */
#include "mod.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
real_T b_mod(real_T x, real_T y)
{
  real_T r;
  if (y == 0.0) {
    r = x;
    if (x == 0.0) {
      r = y;
    }
  } else if (muDoubleScalarIsNaN(x) || muDoubleScalarIsNaN(y) ||
             muDoubleScalarIsInf(x)) {
    r = rtNaN;
  } else if (muDoubleScalarIsInf(y)) {
    if (y > 0.0) {
      if (x > 0.0) {
        r = x;
      } else if (x < 0.0) {
        r = y;
      } else {
        r = 0.0;
      }
    } else if (x > 0.0) {
      r = y;
    } else if (x < 0.0) {
      r = x;
    } else {
      r = -0.0;
    }
  } else {
    if (y > muDoubleScalarFloor(y)) {
      r = muDoubleScalarAbs(x / y);
      if (muDoubleScalarAbs(r - muDoubleScalarFloor(r + 0.5)) >
          2.2204460492503131E-16 * r) {
        r = muDoubleScalarRem(x, y);
      } else {
        r = 0.0;
      }
    } else {
      r = muDoubleScalarRem(x, y);
    }
    if (r == 0.0) {
      r = y * 0.0;
    } else if (r < 0.0) {
      if (y > 0.0) {
        r += y;
      }
    } else if ((r > 0.0) && (y < 0.0)) {
      r += y;
    }
  }
  return r;
}

/* End of code generation (mod.c) */
