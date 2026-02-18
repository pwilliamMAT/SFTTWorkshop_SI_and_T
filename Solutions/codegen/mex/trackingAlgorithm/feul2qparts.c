/*
 * feul2qparts.c
 *
 * Code generation for function 'feul2qparts'
 *
 */

/* Include files */
#include "feul2qparts.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
int32_T b_minus(real_T in1_data[], const real_T in2_data[],
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
    in1_data[i] = in2_data[i * stride_0_0] - in3_data[i * stride_1_0];
  }
  return in1_size;
}

int32_T b_plus(real_T in1_data[], const real_T in2_data[],
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
    in1_data[i] = in2_data[i * stride_0_0] + in3_data[i * stride_1_0];
  }
  return in1_size;
}

int32_T b_times(real_T in1_data[], const real_T in2_data[],
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
    in1_data[i] = in2_data[i * stride_0_0] * in3_data[i * stride_1_0];
  }
  return in1_size;
}

void times(real_T in1_data[], int32_T *in1_size, const real_T in2_data[],
           const int32_T *in2_size)
{
  real_T b_in1_data[100];
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  if (*in2_size == 1) {
    loop_ub = *in1_size;
  } else {
    loop_ub = *in2_size;
  }
  stride_0_0 = (*in1_size != 1);
  stride_1_0 = (*in2_size != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] * in2_data[i * stride_1_0];
  }
  *in1_size = loop_ub;
  if (loop_ub - 1 >= 0) {
    memcpy(&in1_data[0], &b_in1_data[0], (uint32_T)loop_ub * sizeof(real_T));
  }
}

/* End of code generation (feul2qparts.c) */
