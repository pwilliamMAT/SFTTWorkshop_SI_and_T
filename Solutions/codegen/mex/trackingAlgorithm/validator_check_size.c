/*
 * validator_check_size.c
 *
 * Code generation for function 'validator_check_size'
 *
 */

/* Include files */
#include "validator_check_size.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo ar_emlrtRSI = {
    45,                     /* lineNo */
    "validator_check_size", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\validator_"
    "check_size.m" /* pathName */
};

static emlrtRTEInfo n_emlrtRTEI = {
    167,               /* lineNo */
    31,                /* colNo */
    "expandOrReshape", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\validator_"
    "check_size.m" /* pName */
};

/* Function Definitions */
real_T validator_check_size(const emlrtStack *sp, const real_T in_data[],
                            const int32_T in_size[2])
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ar_emlrtRSI;
  if (in_size[1] != 1) {
    emlrtErrorWithMessageIdR2018a(
        &st, &n_emlrtRTEI, "Coder:builtins:ValidatorSizeMismatch",
        "Coder:builtins:ValidatorSizeMismatch", 6, 6, 0.0, 12, 1, 6, 2.0);
  }
  return in_data[0];
}

/* End of code generation (validator_check_size.c) */
