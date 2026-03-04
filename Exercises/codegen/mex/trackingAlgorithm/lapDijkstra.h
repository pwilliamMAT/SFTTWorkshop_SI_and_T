/*
 * lapDijkstra.h
 *
 * Code generation for function 'lapDijkstra'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "trackingAlgorithm_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void lapDijkstra(const emlrtStack *sp, const emxArray_real_T *costMatrix,
                 emxArray_real_T *rowSoln, emxArray_real_T *colSoln,
                 emxArray_real_T *colRedux);

/* End of code generation (lapDijkstra.h) */
