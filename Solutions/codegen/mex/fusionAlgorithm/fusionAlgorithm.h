/*
 * fusionAlgorithm.h
 *
 * Code generation for function 'fusionAlgorithm'
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
emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

void fusionAlgorithm(const emlrtStack *sp, const emxArray_struct0_T *tracks,
                     real_T b_time, struct2_T fusedTracks_data[],
                     int32_T fusedTracks_size[1]);

void fusionAlgorithm_delete(void);

void fusionAlgorithm_emx_free(const emlrtStack *sp);

void fusionAlgorithm_emx_init(const emlrtStack *sp);

void fusionAlgorithm_init(void);

void fusionAlgorithm_new(void);

/* End of code generation (fusionAlgorithm.h) */
