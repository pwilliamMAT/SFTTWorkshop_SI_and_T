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
static emlrtRSInfo pr_emlrtRSI = {
    45,                     /* lineNo */
    "validator_check_size", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/validator_check_size.m" /* pathName
                                                                       */
};

static emlrtRTEInfo o_emlrtRTEI = {
    167,               /* lineNo */
    31,                /* colNo */
    "expandOrReshape", /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/validator_check_size.m" /* pName
                                                                       */
};

/* Function Definitions */
real_T validator_check_size(const emlrtStack *sp, const real_T in_data[],
                            const int32_T in_size[2])
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &pr_emlrtRSI;
  if (in_size[1] != 1) {
    emlrtErrorWithMessageIdR2018a(
        &st, &o_emlrtRTEI, "Coder:builtins:ValidatorSizeMismatch",
        "Coder:builtins:ValidatorSizeMismatch", 6, 6, 0.0, 12, 1, 6, 2.0);
  }
  return in_data[0];
}

/* End of code generation (validator_check_size.c) */
