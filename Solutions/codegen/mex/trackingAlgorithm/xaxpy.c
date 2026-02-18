/*
 * xaxpy.c
 *
 * Code generation for function 'xaxpy'
 *
 */

/* Include files */
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

/* Function Definitions */
void b_xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[36], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    for (k = 0; k < n; k++) {
      int32_T i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

void c_xaxpy(int32_T n, real_T a, const real_T x[36], int32_T ix0, real_T y[6],
             int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d r;
      int32_T i;
      i = (iy0 + k) - 1;
      r = _mm_loadu_pd(&y[i]);
      _mm_storeu_pd(&y[i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(a),
                                             _mm_loadu_pd(&x[(ix0 + k) - 1]))));
    }
    for (k = scalarLB; k < n; k++) {
      vectorUB = (iy0 + k) - 1;
      y[vectorUB] += a * x[(ix0 + k) - 1];
    }
  }
}

void d_xaxpy(int32_T n, real_T a, const real_T x[6], int32_T ix0, real_T y[36],
             int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = n / 2 * 2;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d r;
      __m128d r1;
      int32_T i;
      i = (iy0 + k) - 1;
      r = _mm_loadu_pd(&x[(ix0 + k) - 1]);
      r = _mm_mul_pd(_mm_set1_pd(a), r);
      r1 = _mm_loadu_pd(&y[i]);
      r = _mm_add_pd(r1, r);
      _mm_storeu_pd(&y[i], r);
    }
    for (k = scalarLB; k < n; k++) {
      vectorUB = (iy0 + k) - 1;
      y[vectorUB] += a * x[(ix0 + k) - 1];
    }
  }
}

void e_xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[16], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    for (k = 0; k < n; k++) {
      int32_T i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

void f_xaxpy(int32_T n, real_T a, const real_T x[16], int32_T ix0, real_T y[4],
             int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d r;
      int32_T i;
      i = (iy0 + k) - 1;
      r = _mm_loadu_pd(&y[i]);
      _mm_storeu_pd(&y[i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(a),
                                             _mm_loadu_pd(&x[(ix0 + k) - 1]))));
    }
    for (k = scalarLB; k < n; k++) {
      vectorUB = (iy0 + k) - 1;
      y[vectorUB] += a * x[(ix0 + k) - 1];
    }
  }
}

void g_xaxpy(int32_T n, real_T a, const real_T x[4], int32_T ix0, real_T y[16],
             int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = n / 2 * 2;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d r;
      __m128d r1;
      int32_T i;
      i = (iy0 + k) - 1;
      r = _mm_loadu_pd(&x[(ix0 + k) - 1]);
      r = _mm_mul_pd(_mm_set1_pd(a), r);
      r1 = _mm_loadu_pd(&y[i]);
      r = _mm_add_pd(r1, r);
      _mm_storeu_pd(&y[i], r);
    }
    for (k = scalarLB; k < n; k++) {
      vectorUB = (iy0 + k) - 1;
      y[vectorUB] += a * x[(ix0 + k) - 1];
    }
  }
}

void h_xaxpy(real_T a, const real_T x[9], int32_T ix0, real_T y[3])
{
  if (!(a == 0.0)) {
    __m128d r;
    r = _mm_loadu_pd(&y[1]);
    _mm_storeu_pd(&y[1], _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(a),
                                                  _mm_loadu_pd(&x[ix0 - 1]))));
  }
}

void i_xaxpy(real_T a, const real_T x[3], real_T y[9], int32_T iy0)
{
  if (!(a == 0.0)) {
    __m128d r;
    __m128d r1;
    int32_T i;
    i = iy0 - 1;
    r = _mm_loadu_pd(&x[1]);
    r = _mm_mul_pd(_mm_set1_pd(a), r);
    r1 = _mm_loadu_pd(&y[i]);
    r = _mm_add_pd(r1, r);
    _mm_storeu_pd(&y[i], r);
  }
}

void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    for (k = 0; k < n; k++) {
      int32_T i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

/* End of code generation (xaxpy.c) */
