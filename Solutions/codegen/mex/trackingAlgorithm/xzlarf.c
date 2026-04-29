/*
 * xzlarf.c
 *
 * Code generation for function 'xzlarf'
 *
 */

/* Include files */
#include "xzlarf.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_mexutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo jf_emlrtRSI = {
    103,      /* lineNo */
    "ilazlc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

/* Function Definitions */
int32_T b_ilazlc(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[40],
                 int32_T ia0)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T j;
  boolean_T exitg2;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  j = n;
  exitg2 = false;
  while ((!exitg2) && (j > 0)) {
    int32_T colbottom;
    int32_T coltop;
    int32_T exitg1;
    coltop = ia0 + (j - 1) * 10;
    colbottom = (coltop + m) - 1;
    st.site = &jf_emlrtRSI;
    if ((coltop <= colbottom) && (colbottom > 2147483646)) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    do {
      exitg1 = 0;
      if (coltop <= colbottom) {
        if (A[coltop - 1] != 0.0) {
          exitg1 = 1;
        } else {
          coltop++;
        }
      } else {
        j--;
        exitg1 = 2;
      }
    } while (exitg1 == 0);
    if (exitg1 == 1) {
      exitg2 = true;
    }
  }
  return j;
}

void b_xzlarf(const emlrtStack *sp, int32_T m, int32_T n, int32_T iv0,
              real_T tau, real_T C[36], int32_T ic0, real_T work[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  int32_T colbottom;
  int32_T i;
  int32_T ia;
  int32_T iac;
  int32_T lastc;
  int32_T lastv;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m - 1;
    i = iv0 + m;
    while ((lastv + 1 > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    st.site = &gf_emlrtRSI;
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int32_T exitg1;
      i = ic0 + (lastc - 1) * 6;
      colbottom = i + lastv;
      b_st.site = &jf_emlrtRSI;
      if ((i <= colbottom) && (colbottom > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      do {
        exitg1 = 0;
        if (i <= colbottom) {
          if (C[i - 1] != 0.0) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = -1;
    lastc = 0;
  }
  if (lastv + 1 > 0) {
    real_T c;
    st.site = &hf_emlrtRSI;
    b_st.site = &xe_emlrtRSI;
    if (lastc != 0) {
      c_st.site = &af_emlrtRSI;
      if (lastc > 2147483646) {
        d_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      i = (uint8_T)lastc;
      if (i - 1 >= 0) {
        memset(&work[0], 0, (uint32_T)i * sizeof(real_T));
      }
      colbottom = ic0 + 6 * (lastc - 1);
      for (iac = ic0; iac <= colbottom; iac += 6) {
        c = 0.0;
        i = iac + lastv;
        c_st.site = &kf_emlrtRSI;
        if ((iac <= i) && (i > 2147483646)) {
          d_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }
        i = div_nde_s32_floor(iac - ic0, 6);
        work[i] += c;
      }
    }
    st.site = &if_emlrtRSI;
    b_st.site = &bf_emlrtRSI;
    c_st.site = &cf_emlrtRSI;
    d_st.site = &df_emlrtRSI;
    if (!(-tau == 0.0)) {
      int32_T jA;
      jA = ic0;
      e_st.site = &ef_emlrtRSI;
      i = (uint8_T)lastc;
      for (ia = 0; ia < i; ia++) {
        c = work[ia];
        if (c != 0.0) {
          c *= -tau;
          colbottom = lastv + jA;
          e_st.site = &ff_emlrtRSI;
          if ((jA <= colbottom) && (colbottom > 2147483646)) {
            f_st.site = &k_emlrtRSI;
            check_forloop_overflow_error(&f_st);
          }
          for (iac = jA; iac <= colbottom; iac++) {
            C[iac - 1] += C[((iv0 + iac) - jA) - 1] * c;
          }
        }
        jA += 6;
      }
    }
  }
}

int32_T c_ilazlc(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[60],
                 int32_T ia0)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T j;
  boolean_T exitg2;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  j = n;
  exitg2 = false;
  while ((!exitg2) && (j > 0)) {
    int32_T colbottom;
    int32_T coltop;
    int32_T exitg1;
    coltop = ia0 + (j - 1) * 10;
    colbottom = (coltop + m) - 1;
    st.site = &jf_emlrtRSI;
    if ((coltop <= colbottom) && (colbottom > 2147483646)) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    do {
      exitg1 = 0;
      if (coltop <= colbottom) {
        if (A[coltop - 1] != 0.0) {
          exitg1 = 1;
        } else {
          coltop++;
        }
      } else {
        j--;
        exitg1 = 2;
      }
    } while (exitg1 == 0);
    if (exitg1 == 1) {
      exitg2 = true;
    }
  }
  return j;
}

void c_xzlarf(const emlrtStack *sp, int32_T m, int32_T n, int32_T iv0,
              real_T tau, real_T C[16], int32_T ic0, real_T work[4])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  int32_T colbottom;
  int32_T i;
  int32_T ia;
  int32_T iac;
  int32_T lastc;
  int32_T lastv;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m - 1;
    i = iv0 + m;
    while ((lastv + 1 > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    st.site = &gf_emlrtRSI;
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int32_T exitg1;
      i = ic0 + ((lastc - 1) << 2);
      colbottom = i + lastv;
      b_st.site = &jf_emlrtRSI;
      if ((i <= colbottom) && (colbottom > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      do {
        exitg1 = 0;
        if (i <= colbottom) {
          if (C[i - 1] != 0.0) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = -1;
    lastc = 0;
  }
  if (lastv + 1 > 0) {
    real_T c;
    st.site = &hf_emlrtRSI;
    b_st.site = &xe_emlrtRSI;
    if (lastc != 0) {
      c_st.site = &af_emlrtRSI;
      if (lastc > 2147483646) {
        d_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      i = (uint8_T)lastc;
      if (i - 1 >= 0) {
        memset(&work[0], 0, (uint32_T)i * sizeof(real_T));
      }
      colbottom = ic0 + ((lastc - 1) << 2);
      for (iac = ic0; iac <= colbottom; iac += 4) {
        c = 0.0;
        i = iac + lastv;
        c_st.site = &kf_emlrtRSI;
        if ((iac <= i) && (i > 2147483646)) {
          d_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }
        i = (iac - ic0) >> 2;
        work[i] += c;
      }
    }
    st.site = &if_emlrtRSI;
    b_st.site = &bf_emlrtRSI;
    c_st.site = &cf_emlrtRSI;
    d_st.site = &df_emlrtRSI;
    if (!(-tau == 0.0)) {
      int32_T jA;
      jA = ic0;
      e_st.site = &ef_emlrtRSI;
      i = (uint8_T)lastc;
      for (ia = 0; ia < i; ia++) {
        c = work[ia];
        if (c != 0.0) {
          c *= -tau;
          colbottom = lastv + jA;
          e_st.site = &ff_emlrtRSI;
          if ((jA <= colbottom) && (colbottom > 2147483646)) {
            f_st.site = &k_emlrtRSI;
            check_forloop_overflow_error(&f_st);
          }
          for (iac = jA; iac <= colbottom; iac++) {
            C[iac - 1] += C[((iv0 + iac) - jA) - 1] * c;
          }
        }
        jA += 4;
      }
    }
  }
}

int32_T ilazlc(const emlrtStack *sp, int32_T m, int32_T n, const real_T A[54],
               int32_T ia0)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T j;
  boolean_T exitg2;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  j = n;
  exitg2 = false;
  while ((!exitg2) && (j > 0)) {
    int32_T colbottom;
    int32_T coltop;
    int32_T exitg1;
    coltop = ia0 + (j - 1) * 9;
    colbottom = (coltop + m) - 1;
    st.site = &jf_emlrtRSI;
    if ((coltop <= colbottom) && (colbottom > 2147483646)) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    do {
      exitg1 = 0;
      if (coltop <= colbottom) {
        if (A[coltop - 1] != 0.0) {
          exitg1 = 1;
        } else {
          coltop++;
        }
      } else {
        j--;
        exitg1 = 2;
      }
    } while (exitg1 == 0);
    if (exitg1 == 1) {
      exitg2 = true;
    }
  }
  return j;
}

void xzlarf(const emlrtStack *sp, int32_T m, int32_T n, int32_T iv0, real_T tau,
            real_T C[9], int32_T ic0, real_T work[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  int32_T colbottom;
  int32_T i;
  int32_T ia;
  int32_T iac;
  int32_T lastc;
  int32_T lastv;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m - 1;
    i = iv0 + m;
    while ((lastv + 1 > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    st.site = &gf_emlrtRSI;
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int32_T exitg1;
      i = ic0 + (lastc - 1) * 3;
      colbottom = i + lastv;
      b_st.site = &jf_emlrtRSI;
      if ((i <= colbottom) && (colbottom > 2147483646)) {
        c_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      do {
        exitg1 = 0;
        if (i <= colbottom) {
          if (C[i - 1] != 0.0) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = -1;
    lastc = 0;
  }
  if (lastv + 1 > 0) {
    real_T c;
    st.site = &hf_emlrtRSI;
    b_st.site = &xe_emlrtRSI;
    if (lastc != 0) {
      c_st.site = &af_emlrtRSI;
      if (lastc > 2147483646) {
        d_st.site = &k_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      i = (uint8_T)lastc;
      if (i - 1 >= 0) {
        memset(&work[0], 0, (uint32_T)i * sizeof(real_T));
      }
      colbottom = ic0 + 3 * (lastc - 1);
      for (iac = ic0; iac <= colbottom; iac += 3) {
        c = 0.0;
        i = iac + lastv;
        c_st.site = &kf_emlrtRSI;
        if ((iac <= i) && (i > 2147483646)) {
          d_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }
        i = div_nde_s32_floor(iac - ic0, 3);
        work[i] += c;
      }
    }
    st.site = &if_emlrtRSI;
    b_st.site = &bf_emlrtRSI;
    c_st.site = &cf_emlrtRSI;
    d_st.site = &df_emlrtRSI;
    if (!(-tau == 0.0)) {
      int32_T jA;
      jA = ic0;
      e_st.site = &ef_emlrtRSI;
      i = (uint8_T)lastc;
      for (ia = 0; ia < i; ia++) {
        c = work[ia];
        if (c != 0.0) {
          c *= -tau;
          colbottom = lastv + jA;
          e_st.site = &ff_emlrtRSI;
          if ((jA <= colbottom) && (colbottom > 2147483646)) {
            f_st.site = &k_emlrtRSI;
            check_forloop_overflow_error(&f_st);
          }
          for (iac = jA; iac <= colbottom; iac++) {
            C[iac - 1] += C[((iv0 + iac) - jA) - 1] * c;
          }
        }
        jA += 3;
      }
    }
  }
}

/* End of code generation (xzlarf.c) */
