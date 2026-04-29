/*
 * JIPDATrackAssigner.c
 *
 * Code generation for function 'JIPDATrackAssigner'
 *
 */

/* Include files */
#include "JIPDATrackAssigner.h"
#include "AerospaceMonostaticRadar.h"
#include "TrackEstimator1.h"
#include "computeLikelihoodByIndex.h"
#include "connectedTracks.h"
#include "eml_int_forloop_overflow_check.h"
#include "find.h"
#include "jpda.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo to_emlrtRSI = {
    93,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo uo_emlrtRSI = {
    90,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo vo_emlrtRSI = {
    81,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo wo_emlrtRSI = {
    72,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo xo_emlrtRSI = {
    62,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo yo_emlrtRSI = {
    61,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo ap_emlrtRSI = {
    60,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo bp_emlrtRSI = {
    52,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo cp_emlrtRSI = {
    46,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo dp_emlrtRSI = {
    111,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo ep_emlrtRSI = {
    112,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo fp_emlrtRSI = {
    114,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo gp_emlrtRSI = {
    119,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo hp_emlrtRSI = {
    121,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo vp_emlrtRSI = {
    132,                                        /* lineNo */
    "JIPDATrackAssigner/singleModelLikelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo wp_emlrtRSI = {
    143,                                        /* lineNo */
    "JIPDATrackAssigner/singleModelLikelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo os_emlrtRSI = {
    9,                          /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m" /* pathName */
};

static emlrtRSInfo ps_emlrtRSI = {
    16,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m" /* pathName */
};

static emlrtRSInfo qs_emlrtRSI = {
    19,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m" /* pathName */
};

static emlrtRSInfo rs_emlrtRSI = {
    23,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m" /* pathName */
};

static emlrtRSInfo ss_emlrtRSI = {
    25,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m" /* pathName */
};

static emlrtRSInfo ys_emlrtRSI = {
    27,                                        /* lineNo */
    "TrackEstimator/get.SensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+estimators\\TrackEstimator.m" /* pathName */
};

static emlrtRSInfo abb_emlrtRSI = {
    160,                                                    /* lineNo */
    "JIPDATrackAssigner/measurementVaryingModelLikelihood", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo bbb_emlrtRSI = {
    12,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m" /* pathName */
};

static emlrtECInfo b_emlrtECI = {
    -1,                          /* nDims */
    82,                          /* lineNo */
    21,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    -1,                          /* nDims */
    90,                          /* lineNo */
    13,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtBCInfo hb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    90,                          /* lineNo */
    60,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtECInfo d_emlrtECI = {
    -1,                          /* nDims */
    76,                          /* lineNo */
    21,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtECInfo e_emlrtECI = {
    -1,                          /* nDims */
    75,                          /* lineNo */
    21,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtBCInfo ib_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    76,                          /* lineNo */
    62,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    75,                          /* lineNo */
    76,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    12,                         /* lineNo */
    59,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    17,                         /* lineNo */
    42,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo mb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    19,                         /* lineNo */
    71,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    23,                         /* lineNo */
    54,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    154,                                                    /* lineNo */
    37,                                                     /* colNo */
    "JIPDATrackAssigner/measurementVaryingModelLikelihood", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    1    /* checkKind */
};

static emlrtBCInfo ob_emlrtBCI = {
    -1,                                                     /* iFirst */
    -1,                                                     /* iLast */
    160,                                                    /* lineNo */
    28,                                                     /* colNo */
    "",                                                     /* aName */
    "JIPDATrackAssigner/measurementVaryingModelLikelihood", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    19,                         /* lineNo */
    55,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    140,                                        /* lineNo */
    37,                                         /* colNo */
    "JIPDATrackAssigner/singleModelLikelihood", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    1    /* checkKind */
};

static emlrtBCInfo qb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    23,                         /* lineNo */
    39,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo rb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    23,                         /* lineNo */
    71,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo sb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    25,                         /* lineNo */
    49,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo tb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    25,                         /* lineNo */
    81,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\computeLikelihoodByIndex"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    143,                                        /* lineNo */
    28,                                         /* colNo */
    "",                                         /* aName */
    "JIPDATrackAssigner/singleModelLikelihood", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    88,                          /* lineNo */
    13,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    89,                          /* lineNo */
    13,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo xb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    64,                          /* lineNo */
    38,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    69,                          /* lineNo */
    46,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo ac_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    69,                          /* lineNo */
    71,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo bc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    66,                          /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo cc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    75,                          /* lineNo */
    32,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    75,                          /* lineNo */
    48,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo ec_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    82,                          /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtRTEInfo sd_emlrtRTEI = {
    46,                   /* lineNo */
    13,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo td_emlrtRTEI = {
    49,                   /* lineNo */
    32,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo ud_emlrtRTEI = {
    55,                   /* lineNo */
    13,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo vd_emlrtRTEI = {
    57,                   /* lineNo */
    13,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo wd_emlrtRTEI = {
    61,                   /* lineNo */
    42,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo xd_emlrtRTEI = {
    90,                   /* lineNo */
    47,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo yd_emlrtRTEI = {
    61,                   /* lineNo */
    17,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo ae_emlrtRTEI = {
    90,                   /* lineNo */
    39,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo be_emlrtRTEI = {
    90,                   /* lineNo */
    33,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo ce_emlrtRTEI = {
    64,                   /* lineNo */
    38,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo de_emlrtRTEI = {
    69,                   /* lineNo */
    71,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo ee_emlrtRTEI = {
    69,                   /* lineNo */
    21,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo fe_emlrtRTEI = {
    75,                   /* lineNo */
    48,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo ge_emlrtRTEI = {
    75,                   /* lineNo */
    69,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo he_emlrtRTEI = {
    81,                   /* lineNo */
    45,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo ie_emlrtRTEI = {
    72,                   /* lineNo */
    21,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

static emlrtRTEInfo je_emlrtRTEI = {
    44,                   /* lineNo */
    73,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pName */
};

/* Function Definitions */
void JIPDATrackAssigner_assign(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T assigner_AssignmentThreshold, real_T c_assigner_InitializationThresh,
    real_T assigner_MaxNumEvents,
    const c_fusion_tracker_targetspecs_Pa *c_assigner_Estimator_StateEstim,
    const c_fusion_tracker_sensorspecs_Ae *d_assigner_Estimator_StateEstim,
    trackingEKF *e_assigner_Estimator_StateEstim,
    const c_fusion_tracker_sensorspecs_Ae *f_assigner_Estimator_StateEstim,
    const emxArray_struct_T *trackList, const struct0_T *sensorData,
    emxArray_real_T *assignment, emxArray_boolean_T *unassignedTracks,
    struct0_T *unassignedSensorData)
{
  jmp_buf *volatile emlrtJBStack;
  b_emxArray_struct_T *modelData;
  b_struct_T *modelData_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_boolean_T c_clustDets_data;
  emxArray_boolean_T *b_likelihoodMatrix;
  emxArray_int32_T *clustTracks;
  emxArray_int32_T *ii;
  emxArray_int32_T *r;
  emxArray_real_T *b_jpda;
  emxArray_real_T *c_jpda;
  emxArray_real_T *d_jpda;
  emxArray_real_T *lhood;
  emxArray_real_T *likelihoodMatrix;
  emxArray_real_T *thisClusterTracks;
  const struct_T *trackList_data;
  struct_T b_trackList_data;
  real_T time_data[5000];
  real_T z_data[200];
  real_T b_time_data[100];
  real_T detAssignmentProb_data[51];
  real_T measIndex;
  real_T varargin_2;
  real_T *assignment_data;
  real_T *jpda_data;
  real_T *lhood_data;
  real_T *likelihoodMatrix_data;
  real_T *thisClusterTracks_data;
  int32_T clustDets_data[51];
  int32_T b_iv[4];
  int32_T b_clustDets_size[2];
  int32_T b_time_size[2];
  int32_T clustDets_size[2];
  int32_T time_size[2];
  int32_T z_size[2];
  int32_T JIPDATrackAssigner_assign_numThreads;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T c_loop_ub;
  int32_T d_i;
  int32_T d_loop_ub;
  int32_T e_loop_ub;
  int32_T end;
  int32_T f_loop_ub;
  int32_T g_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T nClusters;
  int32_T tmp_size;
  int32_T *clustTracks_data;
  int32_T *ii_data;
  int32_T *r4;
  int8_T c_tmp_data[50];
  int8_T i2;
  boolean_T b_clustDets_data[51];
  boolean_T unassignedDets_data[50];
  boolean_T b;
  boolean_T *b_clustTracks_data;
  boolean_T *unassignedTracks_data;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  trackList_data = trackList->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &cp_emlrtRSI;
  emxInit_struct_T1(&st, &modelData, &je_emlrtRTEI);
  b_st.site = &dp_emlrtRSI;
  c_AerospaceMonostaticRadar_pars(
      &b_st, sensorData->LookTime.data, sensorData->LookTime.size,
      sensorData->LookAzimuth.data, sensorData->LookAzimuth.size,
      sensorData->LookElevation.data, sensorData->LookElevation.size,
      sensorData->DetectionTime.data, sensorData->DetectionTime.size,
      sensorData->Azimuth.data, sensorData->Azimuth.size,
      sensorData->Elevation.data, sensorData->Elevation.size,
      sensorData->Range.data, sensorData->Range.size,
      sensorData->RangeRate.data, sensorData->RangeRate.size,
      sensorData->AzimuthAccuracy.data, sensorData->AzimuthAccuracy.size,
      sensorData->ElevationAccuracy.data, sensorData->ElevationAccuracy.size,
      sensorData->RangeAccuracy.data, sensorData->RangeAccuracy.size,
      sensorData->RangeRateAccuracy.data, sensorData->RangeRateAccuracy.size,
      z_data, z_size, modelData);
  modelData_data = modelData->data;
  b_st.site = &ep_emlrtRSI;
  c_AerospaceMonostaticRadar_time(
      sensorData->LookTime.data, sensorData->LookTime.size,
      sensorData->DetectionTime.data, sensorData->DetectionTime.size, time_data,
      time_size);
  if ((time_size[1] == 1) && (z_size[1] > 0)) {
    b_time_size[0] = 1;
    b_time_size[1] = 1;
    loop_ub = time_size[0] - 1;
    if (loop_ub >= 0) {
      memcpy(&b_time_data[0], &time_data[0],
             (uint32_T)(loop_ub + 1) * sizeof(real_T));
    }
    b_st.site = &fp_emlrtRSI;
    repmat(&b_st, b_time_data, b_time_size, z_size[1], time_data, time_size);
  }
  emxInit_real_T(&st, &likelihoodMatrix, 2, &sd_emlrtRTEI, true);
  if (modelData->size[0] == 1) {
    b_st.site = &gp_emlrtRSI;
    c_st.site = &vp_emlrtRSI;
    d_st.site = &xp_emlrtRSI;
    e_st.site = &yp_emlrtRSI;
    SD->u1.f1.val = *d_assigner_Estimator_StateEstim;
    f_st.site = &bq_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &f_st, &SD->u1.f1.val, modelData_data[0].LookTime.data,
        modelData_data[0].LookTime.size, modelData_data[0].LookAzimuth.data,
        modelData_data[0].LookAzimuth.size,
        modelData_data[0].LookElevation.data,
        modelData_data[0].LookElevation.size,
        modelData_data[0].DetectionTime.data,
        modelData_data[0].DetectionTime.size,
        modelData_data[0].AzimuthNoise.data,
        modelData_data[0].AzimuthNoise.size,
        modelData_data[0].ElevationNoise.data,
        modelData_data[0].ElevationNoise.size,
        modelData_data[0].RangeNoise.data, modelData_data[0].RangeNoise.size,
        modelData_data[0].RangeRateNoise.data,
        modelData_data[0].RangeRateNoise.size);
    f_st.site = &bq_emlrtRSI;
    e_st.site = &aq_emlrtRSI;
    SD->u1.f1.estimator_SensorSpecifications = *f_assigner_Estimator_StateEstim;
    f_st.site = &ns_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &f_st, &SD->u1.f1.estimator_SensorSpecifications,
        modelData_data[0].LookTime.data, modelData_data[0].LookTime.size,
        modelData_data[0].LookAzimuth.data, modelData_data[0].LookAzimuth.size,
        modelData_data[0].LookElevation.data,
        modelData_data[0].LookElevation.size,
        modelData_data[0].DetectionTime.data,
        modelData_data[0].DetectionTime.size,
        modelData_data[0].AzimuthNoise.data,
        modelData_data[0].AzimuthNoise.size,
        modelData_data[0].ElevationNoise.data,
        modelData_data[0].ElevationNoise.size,
        modelData_data[0].RangeNoise.data, modelData_data[0].RangeNoise.size,
        modelData_data[0].RangeRateNoise.data,
        modelData_data[0].RangeRateNoise.size);
    if ((real_T)trackList->size[0] + 1.0 != trackList->size[0] + 1) {
      emlrtIntegerCheckR2012b((real_T)trackList->size[0] + 1.0, &c_emlrtDCI,
                              &b_st);
    }
    b_loop_ub = (z_size[1] + 1) * (trackList->size[0] + 1);
    loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
    likelihoodMatrix->size[0] = z_size[1] + 1;
    likelihoodMatrix->size[1] = trackList->size[0] + 1;
    emxEnsureCapacity_real_T(&b_st, likelihoodMatrix, loop_ub, &sd_emlrtRTEI);
    likelihoodMatrix_data = likelihoodMatrix->data;
    for (b_i = 0; b_i < b_loop_ub; b_i++) {
      real_T trkIndex;
      c_st.site = &wp_emlrtRSI;
      d_st.site = &os_emlrtRSI;
      trkIndex =
          parseIndex(&d_st, trackList, z_size, (real_T)b_i + 1.0, &measIndex);
      if ((trkIndex == 0.0) && (measIndex > 0.0)) {
        d_st.site = &ps_emlrtRSI;
        e_st.site = &ys_emlrtRSI;
        if (((int32_T)measIndex < 1) || ((int32_T)measIndex > z_size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, z_size[1],
                                        &lb_emlrtBCI, &c_st);
        }
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
            ((int32_T)((uint32_T)b_i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                        loop_ub, &ub_emlrtBCI, &c_st);
        }
        likelihoodMatrix_data[b_i] = SD->u1.f1.val.ClutterModel.ClutterDensity;
      } else if ((measIndex == 0.0) && (trkIndex > 0.0)) {
        if (time_size[1] < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, time_size[1], &mb_emlrtBCI,
                                        &c_st);
        }
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)trkIndex, 1, trackList->size[0], &pb_emlrtBCI, &c_st);
        }
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
            ((int32_T)((uint32_T)b_i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                        loop_ub, &ub_emlrtBCI, &c_st);
        }
        b_trackList_data = trackList_data[(int32_T)trkIndex - 1];
        d_st.site = &qs_emlrtRSI;
        likelihoodMatrix_data[b_i] = c_TrackEstimator_likelihoodUnas(
            &d_st, c_assigner_Estimator_StateEstim, &SD->u1.f1.val,
            e_assigner_Estimator_StateEstim, &b_trackList_data, time_data[0],
            assigner_AssignmentThreshold);
      } else if ((measIndex == 0.0) && (trkIndex == 0.0)) {
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
            ((int32_T)((uint32_T)b_i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                        loop_ub, &ub_emlrtBCI, &c_st);
        }
        likelihoodMatrix_data[b_i] = 1.0;
      } else {
        real_T d;
        real_T d_tmp;
        if (((int32_T)measIndex < 1) || ((int32_T)measIndex > z_size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, z_size[1],
                                        &nb_emlrtBCI, &c_st);
        }
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)trkIndex, 1, trackList->size[0], &qb_emlrtBCI, &c_st);
        }
        b = (((int32_T)measIndex < 1) || ((int32_T)measIndex > time_size[1]));
        if (b) {
          emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, time_size[1],
                                        &rb_emlrtBCI, &c_st);
        }
        d_tmp = time_data[(int32_T)measIndex - 1];
        loop_ub = 4 * ((int32_T)measIndex - 1);
        d_st.site = &rs_emlrtRSI;
        d = TrackEstimator_distance(
            &d_st, c_assigner_Estimator_StateEstim, &SD->u1.f1.val,
            e_assigner_Estimator_StateEstim,
            &trackList_data[(int32_T)trkIndex - 1], &z_data[loop_ub], d_tmp);
        if (d < assigner_AssignmentThreshold) {
          if (((int32_T)trkIndex < 1) ||
              ((int32_T)trkIndex > trackList->size[0])) {
            emlrtDynamicBoundsCheckR2012b(
                (int32_T)trkIndex, 1, trackList->size[0], &sb_emlrtBCI, &c_st);
          }
          if (b) {
            emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, time_size[1],
                                          &tb_emlrtBCI, &c_st);
          }
          d_loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
          if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
              ((int32_T)((uint32_T)b_i + 1U) > d_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                          d_loop_ub, &ub_emlrtBCI, &c_st);
          }
          b_trackList_data = trackList_data[(int32_T)trkIndex - 1];
          d_st.site = &ss_emlrtRSI;
          likelihoodMatrix_data[b_i] = TrackEstimator_likelihood(
              &d_st, c_assigner_Estimator_StateEstim, &SD->u1.f1.val,
              e_assigner_Estimator_StateEstim, &b_trackList_data,
              &z_data[loop_ub], d_tmp);
        } else {
          loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
          if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
              ((int32_T)((uint32_T)b_i + 1U) > loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                          loop_ub, &ub_emlrtBCI, &c_st);
          }
          likelihoodMatrix_data[b_i] = 0.0;
        }
      }
    }
  } else {
    b_st.site = &hp_emlrtRSI;
    if ((real_T)trackList->size[0] + 1.0 != trackList->size[0] + 1) {
      emlrtIntegerCheckR2012b((real_T)trackList->size[0] + 1.0, &b_emlrtDCI,
                              &b_st);
    }
    b_loop_ub = (z_size[1] + 1) * (trackList->size[0] + 1);
    loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
    likelihoodMatrix->size[0] = z_size[1] + 1;
    likelihoodMatrix->size[1] = trackList->size[0] + 1;
    emxEnsureCapacity_real_T(&b_st, likelihoodMatrix, loop_ub, &sd_emlrtRTEI);
    likelihoodMatrix_data = likelihoodMatrix->data;
    for (i = 0; i < b_loop_ub; i++) {
      real_T trkIndex;
      c_st.site = &abb_emlrtRSI;
      SD->u1.f1.val = *d_assigner_Estimator_StateEstim;
      d_st.site = &os_emlrtRSI;
      trkIndex =
          parseIndex(&d_st, trackList, z_size, (real_T)i + 1.0, &measIndex);
      if (measIndex > 0.0) {
        d_st.site = &bbb_emlrtRSI;
        if (((int32_T)measIndex < 1) ||
            ((int32_T)measIndex > modelData->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)measIndex, 1, modelData->size[0], &kb_emlrtBCI, &d_st);
        }
        e_st.site = &xp_emlrtRSI;
        f_st.site = &yp_emlrtRSI;
        SD->u1.f1.val = *d_assigner_Estimator_StateEstim;
        g_st.site = &bq_emlrtRSI;
        c_AerospaceMonostaticRadar_upda(
            &g_st, &SD->u1.f1.val,
            modelData_data[(int32_T)measIndex - 1].LookTime.data,
            modelData_data[(int32_T)measIndex - 1].LookTime.size,
            modelData_data[(int32_T)measIndex - 1].LookAzimuth.data,
            modelData_data[(int32_T)measIndex - 1].LookAzimuth.size,
            modelData_data[(int32_T)measIndex - 1].LookElevation.data,
            modelData_data[(int32_T)measIndex - 1].LookElevation.size,
            modelData_data[(int32_T)measIndex - 1].DetectionTime.data,
            modelData_data[(int32_T)measIndex - 1].DetectionTime.size,
            modelData_data[(int32_T)measIndex - 1].AzimuthNoise.data,
            modelData_data[(int32_T)measIndex - 1].AzimuthNoise.size,
            modelData_data[(int32_T)measIndex - 1].ElevationNoise.data,
            modelData_data[(int32_T)measIndex - 1].ElevationNoise.size,
            modelData_data[(int32_T)measIndex - 1].RangeNoise.data,
            modelData_data[(int32_T)measIndex - 1].RangeNoise.size,
            modelData_data[(int32_T)measIndex - 1].RangeRateNoise.data,
            modelData_data[(int32_T)measIndex - 1].RangeRateNoise.size);
        f_st.site = &aq_emlrtRSI;
        SD->u1.f1.estimator_SensorSpecifications =
            *f_assigner_Estimator_StateEstim;
        g_st.site = &ns_emlrtRSI;
        c_AerospaceMonostaticRadar_upda(
            &g_st, &SD->u1.f1.estimator_SensorSpecifications,
            modelData_data[(int32_T)measIndex - 1].LookTime.data,
            modelData_data[(int32_T)measIndex - 1].LookTime.size,
            modelData_data[(int32_T)measIndex - 1].LookAzimuth.data,
            modelData_data[(int32_T)measIndex - 1].LookAzimuth.size,
            modelData_data[(int32_T)measIndex - 1].LookElevation.data,
            modelData_data[(int32_T)measIndex - 1].LookElevation.size,
            modelData_data[(int32_T)measIndex - 1].DetectionTime.data,
            modelData_data[(int32_T)measIndex - 1].DetectionTime.size,
            modelData_data[(int32_T)measIndex - 1].AzimuthNoise.data,
            modelData_data[(int32_T)measIndex - 1].AzimuthNoise.size,
            modelData_data[(int32_T)measIndex - 1].ElevationNoise.data,
            modelData_data[(int32_T)measIndex - 1].ElevationNoise.size,
            modelData_data[(int32_T)measIndex - 1].RangeNoise.data,
            modelData_data[(int32_T)measIndex - 1].RangeNoise.size,
            modelData_data[(int32_T)measIndex - 1].RangeRateNoise.data,
            modelData_data[(int32_T)measIndex - 1].RangeRateNoise.size);
      }
      if ((trkIndex == 0.0) && (measIndex > 0.0)) {
        d_st.site = &ps_emlrtRSI;
        e_st.site = &ys_emlrtRSI;
        if (((int32_T)measIndex < 1) || ((int32_T)measIndex > z_size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, z_size[1],
                                        &lb_emlrtBCI, &c_st);
        }
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)i + 1U) < 1) ||
            ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, loop_ub,
                                        &ob_emlrtBCI, &c_st);
        }
        likelihoodMatrix_data[i] = SD->u1.f1.val.ClutterModel.ClutterDensity;
      } else if ((measIndex == 0.0) && (trkIndex > 0.0)) {
        if (time_size[1] < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, time_size[1], &mb_emlrtBCI,
                                        &c_st);
        }
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)trkIndex, 1, trackList->size[0], &pb_emlrtBCI, &c_st);
        }
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)i + 1U) < 1) ||
            ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, loop_ub,
                                        &ob_emlrtBCI, &c_st);
        }
        b_trackList_data = trackList_data[(int32_T)trkIndex - 1];
        d_st.site = &qs_emlrtRSI;
        likelihoodMatrix_data[i] = c_TrackEstimator_likelihoodUnas(
            &d_st, c_assigner_Estimator_StateEstim, &SD->u1.f1.val,
            e_assigner_Estimator_StateEstim, &b_trackList_data, time_data[0],
            assigner_AssignmentThreshold);
      } else if ((measIndex == 0.0) && (trkIndex == 0.0)) {
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)i + 1U) < 1) ||
            ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, loop_ub,
                                        &ob_emlrtBCI, &c_st);
        }
        likelihoodMatrix_data[i] = 1.0;
      } else {
        real_T d;
        real_T d_tmp;
        if (((int32_T)measIndex < 1) || ((int32_T)measIndex > z_size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, z_size[1],
                                        &nb_emlrtBCI, &c_st);
        }
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)trkIndex, 1, trackList->size[0], &qb_emlrtBCI, &c_st);
        }
        b = (((int32_T)measIndex < 1) || ((int32_T)measIndex > time_size[1]));
        if (b) {
          emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, time_size[1],
                                        &rb_emlrtBCI, &c_st);
        }
        d_tmp = time_data[(int32_T)measIndex - 1];
        loop_ub = 4 * ((int32_T)measIndex - 1);
        d_st.site = &rs_emlrtRSI;
        d = TrackEstimator_distance(
            &d_st, c_assigner_Estimator_StateEstim, &SD->u1.f1.val,
            e_assigner_Estimator_StateEstim,
            &trackList_data[(int32_T)trkIndex - 1], &z_data[loop_ub], d_tmp);
        if (d < assigner_AssignmentThreshold) {
          if (((int32_T)trkIndex < 1) ||
              ((int32_T)trkIndex > trackList->size[0])) {
            emlrtDynamicBoundsCheckR2012b(
                (int32_T)trkIndex, 1, trackList->size[0], &sb_emlrtBCI, &c_st);
          }
          if (b) {
            emlrtDynamicBoundsCheckR2012b((int32_T)measIndex, 1, time_size[1],
                                          &tb_emlrtBCI, &c_st);
          }
          d_loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
          if (((int32_T)((uint32_T)i + 1U) < 1) ||
              ((int32_T)((uint32_T)i + 1U) > d_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1,
                                          d_loop_ub, &ob_emlrtBCI, &c_st);
          }
          b_trackList_data = trackList_data[(int32_T)trkIndex - 1];
          d_st.site = &ss_emlrtRSI;
          likelihoodMatrix_data[i] = TrackEstimator_likelihood(
              &d_st, c_assigner_Estimator_StateEstim, &SD->u1.f1.val,
              e_assigner_Estimator_StateEstim, &b_trackList_data,
              &z_data[loop_ub], d_tmp);
        } else {
          loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
          if (((int32_T)((uint32_T)i + 1U) < 1) ||
              ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1,
                                          loop_ub, &ob_emlrtBCI, &c_st);
          }
          likelihoodMatrix_data[i] = 0.0;
        }
      }
    }
  }
  emxFree_struct_T1(&st, &modelData);
  if (likelihoodMatrix->size[0] < 2) {
    c_loop_ub = 0;
    loop_ub = 0;
  } else {
    c_loop_ub = 1;
    loop_ub = likelihoodMatrix->size[0];
  }
  if (likelihoodMatrix->size[1] < 2) {
    tmp_size = 0;
    d_loop_ub = 0;
  } else {
    tmp_size = 1;
    d_loop_ub = likelihoodMatrix->size[1];
  }
  emxInit_boolean_T(sp, &b_likelihoodMatrix, 2, &td_emlrtRTEI, true);
  b_loop_ub = loop_ub - c_loop_ub;
  loop_ub = b_likelihoodMatrix->size[0] * b_likelihoodMatrix->size[1];
  b_likelihoodMatrix->size[0] = b_loop_ub;
  d_loop_ub -= tmp_size;
  b_likelihoodMatrix->size[1] = d_loop_ub;
  emxEnsureCapacity_boolean_T(sp, b_likelihoodMatrix, loop_ub, &td_emlrtRTEI);
  unassignedTracks_data = b_likelihoodMatrix->data;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    for (i = 0; i < b_loop_ub; i++) {
      unassignedTracks_data[i + b_likelihoodMatrix->size[0] * b_i] =
          (likelihoodMatrix_data[(c_loop_ub + i) + likelihoodMatrix->size[0] *
                                                       (tmp_size + b_i)] > 0.0);
    }
  }
  emxInit_int32_T(sp, &clustTracks, 2, &je_emlrtRTEI);
  st.site = &bp_emlrtRSI;
  nClusters = connectedTracks(&st, b_likelihoodMatrix, clustDets_data,
                              clustDets_size, clustTracks);
  clustTracks_data = clustTracks->data;
  emxFree_boolean_T(sp, &b_likelihoodMatrix);
  c_i = likelihoodMatrix->size[1] - 1;
  loop_ub = unassignedTracks->size[0];
  unassignedTracks->size[0] = likelihoodMatrix->size[1] - 1;
  emxEnsureCapacity_boolean_T(sp, unassignedTracks, loop_ub, &ud_emlrtRTEI);
  unassignedTracks_data = unassignedTracks->data;
  e_loop_ub = likelihoodMatrix->size[1];
  for (b_i = 0; b_i <= e_loop_ub - 2; b_i++) {
    unassignedTracks_data[b_i] = false;
  }
  end = likelihoodMatrix->size[0] - 1;
  f_loop_ub = likelihoodMatrix->size[0];
  if (f_loop_ub - 2 >= 0) {
    memset(&unassignedDets_data[0], 0,
           (uint32_T)(f_loop_ub - 1) * sizeof(boolean_T));
  }
  loop_ub = assignment->size[0] * assignment->size[1];
  assignment->size[0] = likelihoodMatrix->size[0];
  assignment->size[1] = likelihoodMatrix->size[1] - 1;
  emxEnsureCapacity_real_T(sp, assignment, loop_ub, &vd_emlrtRTEI);
  assignment_data = assignment->data;
  g_loop_ub = likelihoodMatrix->size[0] * (likelihoodMatrix->size[1] - 1);
  for (b_i = 0; b_i < g_loop_ub; b_i++) {
    assignment_data[b_i] = 0.0;
  }
  st.site = &ap_emlrtRSI;
  if (nClusters > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  emxInit_real_T(sp, &thisClusterTracks, 2, &yd_emlrtRTEI, true);
  emxInit_real_T(sp, &lhood, 2, &ee_emlrtRTEI, true);
  emxInit_real_T(sp, &b_jpda, 2, &ie_emlrtRTEI, true);
  emxInit_int32_T(sp, &ii, 2, &ke_emlrtRTEI);
  emxInit_boolean_T(sp, &unassignedTracks, 2, &wd_emlrtRTEI, true);
  emxInit_int32_T(sp, &r, 1, &de_emlrtRTEI);
  emxInit_real_T(sp, &c_jpda, 2, &ge_emlrtRTEI, true);
  emxInit_real_T(sp, &d_jpda, 2, &he_emlrtRTEI, true);
  for (d_i = 0; d_i < nClusters; d_i++) {
    int32_T h_loop_ub;
    int32_T i_loop_ub;
    st.site = &yo_emlrtRSI;
    loop_ub = unassignedTracks->size[0] * unassignedTracks->size[1];
    unassignedTracks->size[0] = 1;
    d_loop_ub = clustTracks->size[1];
    unassignedTracks->size[1] = clustTracks->size[1];
    emxEnsureCapacity_boolean_T(&st, unassignedTracks, loop_ub, &wd_emlrtRTEI);
    b_clustTracks_data = unassignedTracks->data;
    for (b_i = 0; b_i < d_loop_ub; b_i++) {
      b_clustTracks_data[b_i] = (clustTracks_data[b_i] == d_i + 1);
    }
    b_st.site = &wbb_emlrtRSI;
    eml_find(&b_st, unassignedTracks, ii);
    ii_data = ii->data;
    loop_ub = thisClusterTracks->size[0] * thisClusterTracks->size[1];
    thisClusterTracks->size[0] = 1;
    h_loop_ub = ii->size[1];
    thisClusterTracks->size[1] = ii->size[1];
    emxEnsureCapacity_real_T(&st, thisClusterTracks, loop_ub, &yd_emlrtRTEI);
    thisClusterTracks_data = thisClusterTracks->data;
    for (b_i = 0; b_i < h_loop_ub; b_i++) {
      thisClusterTracks_data[b_i] = ii_data[b_i];
    }
    st.site = &xo_emlrtRSI;
    b_clustDets_size[0] = 1;
    loop_ub = clustDets_size[1];
    b_clustDets_size[1] = clustDets_size[1];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      b_clustDets_data[b_i] = (clustDets_data[b_i] == d_i + 1);
    }
    c_clustDets_data.data = &b_clustDets_data[0];
    c_clustDets_data.size = &b_clustDets_size[0];
    c_clustDets_data.allocatedSize = 51;
    c_clustDets_data.numDimensions = 2;
    c_clustDets_data.canFreeData = false;
    b_st.site = &wbb_emlrtRSI;
    eml_find(&b_st, &c_clustDets_data, ii);
    ii_data = ii->data;
    loop_ub = 0;
    i_loop_ub = ii->size[1];
    for (b_i = 0; b_i < i_loop_ub; b_i++) {
      loop_ub++;
    }
    if (loop_ub == 0) {
      loop_ub = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = h_loop_ub;
      emxEnsureCapacity_int32_T(sp, ii, loop_ub, &ce_emlrtRTEI);
      ii_data = ii->data;
      for (b_i = 0; b_i < h_loop_ub; b_i++) {
        loop_ub = (int32_T)thisClusterTracks_data[b_i];
        if ((loop_ub < 1) || (loop_ub > c_i)) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 1, c_i, &xb_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        ii_data[b_i] = loop_ub;
      }
      loop_ub = ii->size[1];
      for (b_i = 0; b_i < loop_ub; b_i++) {
        unassignedTracks_data[ii_data[b_i] - 1] = true;
      }
    } else {
      loop_ub = 0;
      for (b_i = 0; b_i < h_loop_ub; b_i++) {
        loop_ub++;
      }
      if (loop_ub == 0) {
        int8_T b_tmp_data[51];
        d_loop_ub = ii->size[1];
        for (b_i = 0; b_i < i_loop_ub; b_i++) {
          if (((int8_T)ii_data[b_i] < 1) || ((int8_T)ii_data[b_i] > end)) {
            emlrtDynamicBoundsCheckR2012b((int8_T)ii_data[b_i], 1, end,
                                          &bc_emlrtBCI, (emlrtConstCTX)sp);
          }
          b_tmp_data[b_i] = (int8_T)ii_data[b_i];
        }
        for (b_i = 0; b_i < d_loop_ub; b_i++) {
          unassignedDets_data[b_tmp_data[b_i] - 1] = true;
        }
      } else {
        int32_T tmp_data[52];
        int32_T j_loop_ub;
        int8_T b_tmp_data[51];
        boolean_T d_tmp_data[51];
        b_loop_ub = ii->size[1] + 1;
        tmp_data[0] = 0;
        d_loop_ub = (ii->size[1] / 4) << 2;
        loop_ub = d_loop_ub - 4;
        for (b_i = 0; b_i <= loop_ub; b_i += 4) {
          __m128i r2;
          __m128i r3;
          r2 = _mm_loadu_si128((const __m128i *)&ii_data[b_i]);
          r3 = _mm_set1_epi32(1);
          _mm_storeu_si128((__m128i *)&b_iv[0], _mm_add_epi32(r2, r3));
          if ((b_iv[0] < 1) || (b_iv[0] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[0], 1, f_loop_ub, &yb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if ((b_iv[1] < 1) || (b_iv[1] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[1], 1, f_loop_ub, &yb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if ((b_iv[2] < 1) || (b_iv[2] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[2], 1, f_loop_ub, &yb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if ((b_iv[3] < 1) || (b_iv[3] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[3], 1, f_loop_ub, &yb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          r2 = _mm_loadu_si128((const __m128i *)&b_iv[0]);
          _mm_storeu_si128((__m128i *)&tmp_data[b_i + 1],
                           _mm_sub_epi32(r2, r3));
        }
        for (b_i = d_loop_ub; b_i < i_loop_ub; b_i++) {
          loop_ub = ii_data[b_i] + 1;
          if ((loop_ub < 1) || (loop_ub > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, f_loop_ub, &yb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          tmp_data[b_i + 1] = loop_ub - 1;
        }
        loop_ub = r->size[0];
        r->size[0] = thisClusterTracks->size[1] + 1;
        emxEnsureCapacity_int32_T(sp, r, loop_ub, &de_emlrtRTEI);
        r4 = r->data;
        if (e_loop_ub < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, e_loop_ub, &ac_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        r4[0] = 0;
        for (b_i = 0; b_i < h_loop_ub; b_i++) {
          loop_ub = (int32_T)thisClusterTracks_data[b_i] + 1;
          if ((loop_ub < 1) || (loop_ub > e_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, e_loop_ub, &ac_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          r4[b_i + 1] = loop_ub - 1;
        }
        loop_ub = lhood->size[0] * lhood->size[1];
        lhood->size[0] = b_loop_ub;
        d_loop_ub = r->size[0];
        lhood->size[1] = r->size[0];
        emxEnsureCapacity_real_T(sp, lhood, loop_ub, &ee_emlrtRTEI);
        lhood_data = lhood->data;
        for (b_i = 0; b_i < d_loop_ub; b_i++) {
          for (i = 0; i < b_loop_ub; i++) {
            lhood_data[i + lhood->size[0] * b_i] =
                likelihoodMatrix_data[tmp_data[i] +
                                      likelihoodMatrix->size[0] * r4[b_i]];
          }
        }
        st.site = &wo_emlrtRSI;
        jpda(&st, lhood, assigner_MaxNumEvents, b_jpda);
        jpda_data = b_jpda->data;
        j_loop_ub = b_jpda->size[0];
        c_loop_ub = b_jpda->size[0] - 1;
        b = ((b_jpda->size[0] - 1 < 1) ||
             (b_jpda->size[0] - 1 > b_jpda->size[0]));
        if (b) {
          emlrtDynamicBoundsCheckR2012b(b_jpda->size[0] - 1, 1, b_jpda->size[0],
                                        &jb_emlrtBCI, (emlrtConstCTX)sp);
        }
        tmp_size = ii->size[1];
        for (b_i = 0; b_i < i_loop_ub; b_i++) {
          if (((int8_T)ii_data[b_i] < 1) ||
              ((int8_T)ii_data[b_i] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int8_T)ii_data[b_i], 1, f_loop_ub,
                                          &cc_emlrtBCI, (emlrtConstCTX)sp);
          }
          b_tmp_data[b_i] = (int8_T)((int8_T)ii_data[b_i] - 1);
        }
        loop_ub = r->size[0];
        r->size[0] = h_loop_ub;
        emxEnsureCapacity_int32_T(sp, r, loop_ub, &fe_emlrtRTEI);
        r4 = r->data;
        for (b_i = 0; b_i < h_loop_ub; b_i++) {
          loop_ub = (int32_T)thisClusterTracks_data[b_i];
          if ((loop_ub < 1) || (loop_ub > c_i)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, c_i, &dc_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          r4[b_i] = loop_ub - 1;
        }
        z_size[0] = tmp_size;
        d_loop_ub = r->size[0];
        z_size[1] = r->size[0];
        time_size[0] = b_jpda->size[0] - 1;
        b_loop_ub = b_jpda->size[1];
        time_size[1] = b_jpda->size[1];
        emlrtSubAssignSizeCheckR2012b(&z_size[0], 2, &time_size[0], 2,
                                      &e_emlrtECI, (emlrtCTX)sp);
        loop_ub = c_jpda->size[0] * c_jpda->size[1];
        c_jpda->size[0] = b_jpda->size[0] - 1;
        c_jpda->size[1] = b_jpda->size[1];
        emxEnsureCapacity_real_T(sp, c_jpda, loop_ub, &ge_emlrtRTEI);
        lhood_data = c_jpda->data;
        for (b_i = 0; b_i < b_loop_ub; b_i++) {
          for (i = 0; i <= j_loop_ub - 2; i++) {
            lhood_data[i + c_jpda->size[0] * b_i] =
                jpda_data[i + b_jpda->size[0] * b_i];
          }
        }
        for (b_i = 0; b_i < d_loop_ub; b_i++) {
          for (i = 0; i < tmp_size; i++) {
            assignment_data[b_tmp_data[i] + assignment->size[0] * r4[b_i]] =
                lhood_data[i + tmp_size * b_i];
          }
        }
        if (b_jpda->size[0] < 1) {
          emlrtDynamicBoundsCheckR2012b(b_jpda->size[0], 1, b_jpda->size[0],
                                        &ib_emlrtBCI, (emlrtConstCTX)sp);
        }
        z_size[0] = 1;
        z_size[1] = r->size[0];
        time_size[0] = 1;
        time_size[1] = b_jpda->size[1];
        emlrtSubAssignSizeCheckR2012b(&z_size[0], 2, &time_size[0], 2,
                                      &d_emlrtECI, (emlrtCTX)sp);
        for (b_i = 0; b_i < d_loop_ub; b_i++) {
          assignment_data[end + assignment->size[0] * r4[b_i]] =
              jpda_data[c_loop_ub + b_jpda->size[0] * b_i];
        }
        loop_ub = d_jpda->size[0] * d_jpda->size[1];
        d_jpda->size[0] = b_jpda->size[0] - 1;
        d_jpda->size[1] = b_jpda->size[1];
        emxEnsureCapacity_real_T(sp, d_jpda, loop_ub, &he_emlrtRTEI);
        lhood_data = d_jpda->data;
        for (b_i = 0; b_i < b_loop_ub; b_i++) {
          for (i = 0; i <= j_loop_ub - 2; i++) {
            lhood_data[i + d_jpda->size[0] * b_i] =
                jpda_data[i + b_jpda->size[0] * b_i];
          }
        }
        st.site = &vo_emlrtRSI;
        loop_ub = b_sum(d_jpda, detAssignmentProb_data);
        d_loop_ub = ii->size[1];
        for (b_i = 0; b_i < i_loop_ub; b_i++) {
          if (((int8_T)ii_data[b_i] < 1) || ((int8_T)ii_data[b_i] > end)) {
            emlrtDynamicBoundsCheckR2012b((int8_T)ii_data[b_i], 1, end,
                                          &ec_emlrtBCI, (emlrtConstCTX)sp);
          }
          b_tmp_data[b_i] = (int8_T)ii_data[b_i];
        }
        for (b_i = 0; b_i < loop_ub; b_i++) {
          d_tmp_data[b_i] =
              (detAssignmentProb_data[b_i] < c_assigner_InitializationThresh);
        }
        if (loop_ub != d_loop_ub) {
          emlrtSubAssignSizeCheck1dR2017a(d_loop_ub, loop_ub, &b_emlrtECI,
                                          (emlrtConstCTX)sp);
        }
        for (b_i = 0; b_i < d_loop_ub; b_i++) {
          unassignedDets_data[b_tmp_data[b_i] - 1] = d_tmp_data[b_i];
        }
      }
    }
  }
  emxFree_real_T(sp, &d_jpda);
  emxFree_real_T(sp, &c_jpda);
  emxFree_int32_T(sp, &r);
  emxFree_boolean_T(sp, &unassignedTracks);
  emxFree_int32_T(sp, &ii);
  emxFree_int32_T(sp, &clustTracks);
  emxFree_real_T(sp, &lhood);
  emxFree_real_T(sp, &likelihoodMatrix);
  for (b_i = 0; b_i < g_loop_ub; b_i++) {
    loop_ub = assignment->size[0] * assignment->size[1];
    if (assignment_data[b_i] > 1.0) {
      if (b_i > loop_ub - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, loop_ub - 1, &vb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      assignment_data[b_i] = 1.0;
    }
  }
  loop_ub = assignment->size[0] * assignment->size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    d_loop_ub = assignment->size[0] * assignment->size[1];
    if (assignment_data[b_i] < 0.0) {
      if (b_i > d_loop_ub - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, d_loop_ub - 1, &wb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      assignment_data[b_i] = 0.0;
    }
  }
  if (assignment->size[0] - 1 < 1) {
    d_loop_ub = 0;
  } else {
    if ((assignment->size[0] - 1 < 1) ||
        (assignment->size[0] - 1 > assignment->size[0])) {
      emlrtDynamicBoundsCheckR2012b(assignment->size[0] - 1, 1,
                                    assignment->size[0], &hb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    d_loop_ub = assignment->size[0] - 1;
  }
  loop_ub = b_jpda->size[0] * b_jpda->size[1];
  b_jpda->size[0] = d_loop_ub;
  c_loop_ub = assignment->size[1];
  b_jpda->size[1] = assignment->size[1];
  emxEnsureCapacity_real_T(sp, b_jpda, loop_ub, &xd_emlrtRTEI);
  jpda_data = b_jpda->data;
  for (b_i = 0; b_i < c_loop_ub; b_i++) {
    for (i = 0; i < d_loop_ub; i++) {
      jpda_data[i + b_jpda->size[0] * b_i] =
          assignment_data[i + assignment->size[0] * b_i];
    }
  }
  st.site = &uo_emlrtRSI;
  sum(&st, b_jpda, thisClusterTracks);
  emxFree_real_T(sp, &b_jpda);
  loop_ub = thisClusterTracks->size[0] * thisClusterTracks->size[1];
  thisClusterTracks->size[0] = 1;
  emxEnsureCapacity_real_T(sp, thisClusterTracks, loop_ub, &ae_emlrtRTEI);
  thisClusterTracks_data = thisClusterTracks->data;
  loop_ub = thisClusterTracks->size[1] - 1;
  d_loop_ub = (thisClusterTracks->size[1] / 2) << 1;
  b_loop_ub = d_loop_ub - 2;
  for (b_i = 0; b_i <= b_loop_ub; b_i += 2) {
    __m128d r1;
    r1 = _mm_loadu_pd(&thisClusterTracks_data[b_i]);
    _mm_storeu_pd(&thisClusterTracks_data[b_i],
                  _mm_sub_pd(_mm_set1_pd(1.0), r1));
  }
  for (b_i = d_loop_ub; b_i <= loop_ub; b_i++) {
    thisClusterTracks_data[b_i] = 1.0 - thisClusterTracks_data[b_i];
  }
  loop_ub = thisClusterTracks->size[0] * thisClusterTracks->size[1];
  thisClusterTracks->size[0] = 1;
  emxEnsureCapacity_real_T(sp, thisClusterTracks, loop_ub, &be_emlrtRTEI);
  thisClusterTracks_data = thisClusterTracks->data;
  loop_ub = thisClusterTracks->size[1] - 1;
  d_loop_ub = thisClusterTracks->size[1] - 1;
  if (thisClusterTracks->size[1] < 800) {
    for (i1 = 0; i1 <= loop_ub; i1++) {
      measIndex = thisClusterTracks_data[i1];
      thisClusterTracks_data[i1] = muDoubleScalarMax(0.0, measIndex);
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    JIPDATrackAssigner_assign_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(                                          \
        JIPDATrackAssigner_assign_numThreads) private(varargin_2)

    for (i1 = 0; i1 <= d_loop_ub; i1++) {
      varargin_2 = thisClusterTracks_data[i1];
      thisClusterTracks_data[i1] = muDoubleScalarMax(0.0, varargin_2);
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  z_size[0] = 1;
  z_size[1] = c_loop_ub;
  emlrtSubAssignSizeCheckR2012b(&z_size[0], 2, &thisClusterTracks->size[0], 2,
                                &c_emlrtECI, (emlrtCTX)sp);
  for (b_i = 0; b_i < c_loop_ub; b_i++) {
    assignment_data[(assignment->size[0] + assignment->size[0] * b_i) - 1] =
        thisClusterTracks_data[b_i];
  }
  emxFree_real_T(sp, &thisClusterTracks);
  st.site = &to_emlrtRSI;
  *unassignedSensorData = *sensorData;
  d_loop_ub = 0;
  loop_ub = 0;
  for (b_i = 0; b_i < end; b_i++) {
    if (unassignedDets_data[b_i]) {
      d_loop_ub++;
      c_tmp_data[loop_ub] = (int8_T)b_i;
      loop_ub++;
    }
  }
  unassignedSensorData->DetectionTime.size[0] = 1;
  unassignedSensorData->DetectionTime.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->DetectionTime.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->DetectionTime.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->DetectionTime.data[b_i] =
        sensorData->DetectionTime.data[i2];
  }
  unassignedSensorData->Azimuth.size[0] = 1;
  unassignedSensorData->Azimuth.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->Azimuth.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->Azimuth.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->Azimuth.data[b_i] = sensorData->Azimuth.data[i2];
  }
  unassignedSensorData->Elevation.size[0] = 1;
  unassignedSensorData->Elevation.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->Elevation.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->Elevation.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->Elevation.data[b_i] = sensorData->Elevation.data[i2];
  }
  unassignedSensorData->Range.size[0] = 1;
  unassignedSensorData->Range.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->Range.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->Range.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->Range.data[b_i] = sensorData->Range.data[i2];
  }
  unassignedSensorData->RangeRate.size[0] = 1;
  unassignedSensorData->RangeRate.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->RangeRate.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->RangeRate.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->RangeRate.data[b_i] = sensorData->RangeRate.data[i2];
  }
  unassignedSensorData->AzimuthAccuracy.size[0] = 1;
  unassignedSensorData->AzimuthAccuracy.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->AzimuthAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->AzimuthAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->AzimuthAccuracy.data[b_i] =
        sensorData->AzimuthAccuracy.data[i2];
  }
  unassignedSensorData->ElevationAccuracy.size[0] = 1;
  unassignedSensorData->ElevationAccuracy.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->ElevationAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->ElevationAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->ElevationAccuracy.data[b_i] =
        sensorData->ElevationAccuracy.data[i2];
  }
  unassignedSensorData->RangeAccuracy.size[0] = 1;
  unassignedSensorData->RangeAccuracy.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->RangeAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->RangeAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->RangeAccuracy.data[b_i] =
        sensorData->RangeAccuracy.data[i2];
  }
  unassignedSensorData->RangeRateAccuracy.size[0] = 1;
  unassignedSensorData->RangeRateAccuracy.size[1] = d_loop_ub;
  for (b_i = 0; b_i < d_loop_ub; b_i++) {
    i2 = c_tmp_data[b_i];
    if (i2 > sensorData->RangeRateAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->RangeRateAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->RangeRateAccuracy.data[b_i] =
        sensorData->RangeRateAccuracy.data[i2];
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (JIPDATrackAssigner.c) */
