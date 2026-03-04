/*
 * AssignerGNN.h
 *
 * Code generation for function 'AssignerGNN'
 *
 */

#pragma once

/* Include files */
#include "fusionAlgorithm_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void AssignerGNN_stepImpl(const emlrtStack *sp,
                          c_matlabshared_tracking_interna *obj,
                          const emxArray_real_T *costMatrix,
                          emxArray_uint32_T *assignments,
                          emxArray_uint32_T *unassignedRows,
                          emxArray_uint32_T *unassignedColumns);

/* End of code generation (AssignerGNN.h) */
