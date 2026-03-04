/*
 * rotmat.c
 *
 * Code generation for function 'rotmat'
 *
 */

/* Include files */
#include "rotmat.h"
#include "rt_nonfinite.h"

/* Function Definitions */
int32_T binary_expand_op(real_T in1_data[], const real_T in2_data[],
                         const int32_T *in2_size, const real_T in3_data[],
                         const int32_T *in3_size)
{
  int32_T i;
  int32_T in1_size;
  int32_T stride_0_0;
  int32_T stride_1_0;
  if (*in3_size == 1) {
    in1_size = *in2_size;
  } else {
    in1_size = *in3_size;
  }
  stride_0_0 = (*in2_size != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = in2_data[i * stride_0_0] * in3_data[i * stride_1_0] * 2.0;
  }
  return in1_size;
}

/* End of code generation (rotmat.c) */
