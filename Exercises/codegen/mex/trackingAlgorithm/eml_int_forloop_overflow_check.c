/*
 * eml_int_forloop_overflow_check.c
 *
 * Code generation for function 'eml_int_forloop_overflow_check'
 *
 */

/* Include files */
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRTEInfo dc_emlrtRTEI = {
    87,                             /* lineNo */
    33,                             /* colNo */
    "check_forloop_overflow_error", /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m" /* pName
                                                                           */
};

/* Function Definitions */
void b_check_forloop_overflow_error(const emlrtStack *sp)
{
  emlrtErrorWithMessageIdR2018a(
      sp, &dc_emlrtRTEI, "Coder:toolbox:int_forloop_overflow",
      "Coder:toolbox:int_forloop_overflow", 3, 4, 6, "uint64");
}

void check_forloop_overflow_error(const emlrtStack *sp)
{
  emlrtErrorWithMessageIdR2018a(
      sp, &dc_emlrtRTEI, "Coder:toolbox:int_forloop_overflow",
      "Coder:toolbox:int_forloop_overflow", 3, 4, 5, "int32");
}

/* End of code generation (eml_int_forloop_overflow_check.c) */
