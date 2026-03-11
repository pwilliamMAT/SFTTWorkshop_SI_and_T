/*
 * Fuserxcov.h
 *
 * Code generation for function 'Fuserxcov'
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
void Fuserxcov_fuse(const emlrtStack *sp, const fusion_internal_Fuserxcov *obj,
                    objectTrack *centralTrack,
                    const emxArray_objectTrack *sourceTracks,
                    const emxArray_real_T *inAssigned);

/* End of code generation (Fuserxcov.h) */
