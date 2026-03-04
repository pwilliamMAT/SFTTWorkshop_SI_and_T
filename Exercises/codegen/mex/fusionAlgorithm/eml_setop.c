/*
 * eml_setop.c
 *
 * Code generation for function 'eml_setop'
 *
 */

/* Include files */
#include "eml_setop.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "issorted.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo pp_emlrtRSI = {
    241,                                                     /* lineNo */
    "do_vectors",                                            /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pathName */
};

static emlrtRSInfo qp_emlrtRSI = {
    244,                                                     /* lineNo */
    "do_vectors",                                            /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pathName */
};

static emlrtRTEInfo jb_emlrtRTEI = {
    242,                                                     /* lineNo */
    13,                                                      /* colNo */
    "do_vectors",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = {
    245,                                                     /* lineNo */
    13,                                                      /* colNo */
    "do_vectors",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = {
    409,                                                     /* lineNo */
    5,                                                       /* colNo */
    "do_vectors",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = {
    420,                                                     /* lineNo */
    9,                                                       /* colNo */
    "do_vectors",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = {
    447,                                                     /* lineNo */
    5,                                                       /* colNo */
    "do_vectors",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo ff_emlrtRTEI = {
    223,                                                     /* lineNo */
    24,                                                      /* colNo */
    "eml_setop",                                             /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo gf_emlrtRTEI = {
    224,                                                     /* lineNo */
    25,                                                      /* colNo */
    "eml_setop",                                             /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo hf_emlrtRTEI = {
    225,                                                     /* lineNo */
    25,                                                      /* colNo */
    "eml_setop",                                             /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo if_emlrtRTEI = {
    415,                                                     /* lineNo */
    9,                                                       /* colNo */
    "eml_setop",                                             /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo jf_emlrtRTEI = {
    426,                                                     /* lineNo */
    13,                                                      /* colNo */
    "eml_setop",                                             /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

static emlrtRTEInfo kf_emlrtRTEI = {
    451,                                                     /* lineNo */
    9,                                                       /* colNo */
    "eml_setop",                                             /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/private/eml_setop.m" /* pName */
};

/* Function Definitions */
void do_vectors(const emlrtStack *sp, const emxArray_uint32_T *a,
                const emxArray_uint32_T *b, emxArray_uint32_T *c,
                emxArray_int32_T *ia, emxArray_int32_T *ib)
{
  emlrtStack st;
  int32_T iafirst;
  int32_T ialast;
  int32_T ibfirst;
  int32_T iblast;
  int32_T na;
  int32_T nb;
  int32_T nc;
  int32_T ncmax;
  int32_T *ia_data;
  int32_T *ib_data;
  const uint32_T *a_data;
  const uint32_T *b_data;
  uint32_T *c_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_data = b->data;
  a_data = a->data;
  na = a->size[0];
  nb = b->size[0];
  ncmax = muIntScalarMin_sint32(na, nb);
  iafirst = c->size[0];
  c->size[0] = ncmax;
  emxEnsureCapacity_uint32_T(sp, c, iafirst, &ff_emlrtRTEI);
  c_data = c->data;
  iafirst = ia->size[0];
  ia->size[0] = ncmax;
  emxEnsureCapacity_int32_T(sp, ia, iafirst, &gf_emlrtRTEI);
  ia_data = ia->data;
  iafirst = ib->size[0];
  ib->size[0] = ncmax;
  emxEnsureCapacity_int32_T(sp, ib, iafirst, &hf_emlrtRTEI);
  ib_data = ib->data;
  st.site = &pp_emlrtRSI;
  if (!issorted(&st, a)) {
    emlrtErrorWithMessageIdR2018a(sp, &jb_emlrtRTEI,
                                  "Coder:toolbox:eml_setop_unsortedA",
                                  "Coder:toolbox:eml_setop_unsortedA", 0);
  }
  st.site = &qp_emlrtRSI;
  if (!issorted(&st, b)) {
    emlrtErrorWithMessageIdR2018a(sp, &kb_emlrtRTEI,
                                  "Coder:toolbox:eml_setop_unsortedB",
                                  "Coder:toolbox:eml_setop_unsortedB", 0);
  }
  nc = 0;
  iafirst = 0;
  ialast = 1;
  ibfirst = 0;
  iblast = 1;
  while ((ialast <= na) && (iblast <= nb)) {
    int32_T b_ialast;
    int32_T b_iblast;
    uint32_T ak;
    uint32_T bk;
    b_ialast = ialast;
    ak = a_data[ialast - 1];
    while ((b_ialast < a->size[0]) && (a_data[b_ialast] == ak)) {
      b_ialast++;
    }
    ialast = b_ialast;
    b_iblast = iblast;
    bk = b_data[iblast - 1];
    while ((b_iblast < b->size[0]) && (b_data[b_iblast] == bk)) {
      b_iblast++;
    }
    iblast = b_iblast;
    if (ak == bk) {
      nc++;
      c_data[nc - 1] = ak;
      ia_data[nc - 1] = iafirst + 1;
      ib_data[nc - 1] = ibfirst + 1;
      ialast = b_ialast + 1;
      iafirst = b_ialast;
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      ialast = b_ialast + 1;
      iafirst = b_ialast;
    } else {
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    }
  }
  if (ncmax > 0) {
    if (nc > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &lb_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (nc < 1) {
      ialast = 0;
    } else {
      ialast = nc;
    }
    iafirst = ia->size[0];
    ia->size[0] = ialast;
    emxEnsureCapacity_int32_T(sp, ia, iafirst, &if_emlrtRTEI);
    if (nc > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &mb_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    iafirst = ib->size[0];
    ib->size[0] = ialast;
    emxEnsureCapacity_int32_T(sp, ib, iafirst, &jf_emlrtRTEI);
    if (nc > ncmax) {
      emlrtErrorWithMessageIdR2018a(sp, &nb_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    iafirst = c->size[0];
    c->size[0] = ialast;
    emxEnsureCapacity_uint32_T(sp, c, iafirst, &kf_emlrtRTEI);
  }
}

/* End of code generation (eml_setop.c) */
