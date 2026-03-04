/*
 * trisolve.c
 *
 * Code generation for function 'trisolve'
 *
 */

/* Include files */
#include "trisolve.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void b_trisolve(const real_T A[16], real_T B[24])
{
  int32_T b_i;
  int32_T j;
  int32_T k;
  for (j = 0; j < 6; j++) {
    int32_T jBcol;
    jBcol = j << 2;
    for (k = 3; k >= 0; k--) {
      real_T d;
      int32_T i;
      int32_T kAcol;
      kAcol = k << 2;
      i = k + jBcol;
      d = B[i];
      if (d != 0.0) {
        B[i] = d / A[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          int32_T i1;
          i1 = b_i + jBcol;
          B[i1] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

void trisolve(const real_T A[16], real_T B[24])
{
  int32_T b_i;
  int32_T j;
  int32_T k;
  for (j = 0; j < 6; j++) {
    int32_T jBcol;
    jBcol = (j << 2) - 1;
    for (k = 0; k < 4; k++) {
      real_T d;
      int32_T i;
      int32_T kAcol;
      kAcol = (k << 2) - 1;
      i = (k + jBcol) + 1;
      d = B[i];
      if (d != 0.0) {
        int32_T i1;
        B[i] = d / A[(k + kAcol) + 1];
        i1 = k + 2;
        for (b_i = i1; b_i < 5; b_i++) {
          int32_T i2;
          i2 = b_i + jBcol;
          B[i2] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

/* End of code generation (trisolve.c) */
