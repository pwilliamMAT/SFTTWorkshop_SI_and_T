/*
 * FuserManager.c
 *
 * Code generation for function 'FuserManager'
 *
 */

/* Include files */
#include "FuserManager.h"
#include "constvel.h"
#include "eml_int_forloop_overflow_check.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_internal_types.h"
#include "fusionAlgorithm_types.h"
#include "gaussEKFilter.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "mrdivide_helper.h"
#include "objectTrack.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "validateattributes.h"
#include "xzgetrf.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bc_emlrtRSI = {
    445,                      /* lineNo */
    "FuserManager/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo cc_emlrtRSI = {
    446,                      /* lineNo */
    "FuserManager/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    451,                      /* lineNo */
    "FuserManager/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI = {
    460,                      /* lineNo */
    "FuserManager/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo fc_emlrtRSI = {
    467,                      /* lineNo */
    "FuserManager/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo gc_emlrtRSI = {
    475,                      /* lineNo */
    "FuserManager/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI = {
    530,                       /* lineNo */
    "FuserManager/parseTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo jh_emlrtRSI =
    {
        484,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo pi_emlrtRSI = {
    650,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo qi_emlrtRSI = {
    653,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ri_emlrtRSI = {
    667,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo si_emlrtRSI = {
    673,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ti_emlrtRSI = {
    675,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ui_emlrtRSI = {
    676,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo vi_emlrtRSI = {
    679,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo wi_emlrtRSI = {
    680,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo xi_emlrtRSI = {
    683,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo yi_emlrtRSI = {
    688,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo aj_emlrtRSI = {
    689,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo bj_emlrtRSI = {
    700,                          /* lineNo */
    "FuserManager/getLiveTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo hj_emlrtRSI =
    {
        137,   /* lineNo */
        "cat", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\cat.m" /* pathName
                                                                          */
};

static emlrtRTEInfo u_emlrtRTEI = {
    543,                          /* lineNo */
    85,                           /* colNo */
    "FuserManager/getConfigByID", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo v_emlrtRTEI = {
    553,                          /* lineNo */
    17,                           /* colNo */
    "FuserManager/getConfigByID", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo eb_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtBCInfo wh_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    685,                     /* lineNo */
    75,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo xh_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    684,                     /* lineNo */
    57,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo yh_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    714,                               /* lineNo */
    35,                                /* colNo */
    "",                                /* aName */
    "FuserManager/collectTrackStates", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo ai_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    713,                               /* lineNo */
    33,                                /* colNo */
    "",                                /* aName */
    "FuserManager/collectTrackStates", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo bi_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    44,           /* lineNo */
    53,           /* colNo */
    "",           /* aName */
    "calcOneRow", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ci_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    44,           /* lineNo */
    29,           /* colNo */
    "",           /* aName */
    "calcOneRow", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo di_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    43,           /* lineNo */
    14,           /* colNo */
    "",           /* aName */
    "calcOneRow", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo g_emlrtECI = {
    -1,              /* nDims */
    33,              /* lineNo */
    5,               /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtBCInfo ei_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    714,                               /* lineNo */
    47,                                /* colNo */
    "",                                /* aName */
    "FuserManager/collectTrackStates", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo fi_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    713,                               /* lineNo */
    45,                                /* colNo */
    "",                                /* aName */
    "FuserManager/collectTrackStates", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtRTEInfo hc_emlrtRTEI = {
    702,                          /* lineNo */
    21,                           /* colNo */
    "FuserManager/getLiveTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtBCInfo gi_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    678,                     /* lineNo */
    43,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo hi_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    693,                     /* lineNo */
    49,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtECInfo h_emlrtECI = {
    -1,                      /* nDims */
    690,                     /* lineNo */
    17,                      /* colNo */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtBCInfo ii_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    690,                     /* lineNo */
    30,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo ji_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    680,                     /* lineNo */
    51,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo ki_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    679,                     /* lineNo */
    40,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtRTEInfo ic_emlrtRTEI = {
    677,                     /* lineNo */
    25,                      /* colNo */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtBCInfo li_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    660,                     /* lineNo */
    38,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtDCInfo rc_emlrtDCI = {
    639,                     /* lineNo */
    30,                      /* colNo */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    1                           /* checkKind */
};

static emlrtDCInfo sc_emlrtDCI = {
    639,                     /* lineNo */
    30,                      /* colNo */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    4                           /* checkKind */
};

static emlrtDCInfo tc_emlrtDCI = {
    701,                          /* lineNo */
    36,                           /* colNo */
    "FuserManager/getLiveTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    1                           /* checkKind */
};

static emlrtBCInfo mi_emlrtBCI = {
    0,                            /* iFirst */
    99,                           /* iLast */
    703,                          /* lineNo */
    49,                           /* colNo */
    "",                           /* aName */
    "FuserManager/getLiveTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo ni_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    703,                          /* lineNo */
    28,                           /* colNo */
    "",                           /* aName */
    "FuserManager/getLiveTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo oi_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    704,                          /* lineNo */
    30,                           /* colNo */
    "",                           /* aName */
    "FuserManager/getLiveTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo pi_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    696,                     /* lineNo */
    13,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo qi_emlrtBCI = {
    -1,                      /* iFirst */
    -1,                      /* iLast */
    693,                     /* lineNo */
    21,                      /* colNo */
    "",                      /* aName */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo ri_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    44,           /* lineNo */
    12,           /* colNo */
    "",           /* aName */
    "calcOneRow", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo rc_emlrtRTEI = {
    475,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = {
    492,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo uc_emlrtRTEI = {
    495,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo vc_emlrtRTEI = {
    496,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo wc_emlrtRTEI = {
    497,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = {
    492,            /* lineNo */
    31,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo qi_emlrtRTEI = {
    639,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ri_emlrtRTEI = {
    700,            /* lineNo */
    34,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo si_emlrtRTEI = {
    700,            /* lineNo */
    33,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ti_emlrtRTEI = {
    650,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ui_emlrtRTEI = {
    703,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo vi_emlrtRTEI = {
    653,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo wi_emlrtRTEI = {
    656,            /* lineNo */
    27,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo xi_emlrtRTEI = {
    639,            /* lineNo */
    26,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo yi_emlrtRTEI = {
    667,            /* lineNo */
    21,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo bj_emlrtRTEI = {
    687,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo cj_emlrtRTEI =
    {
        107,   /* lineNo */
        28,    /* colNo */
        "cat", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\cat.m" /* pName
                                                                          */
};

static emlrtRTEInfo dj_emlrtRTEI = {
    689,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo fj_emlrtRTEI = {
    33,              /* lineNo */
    21,              /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtRTEInfo gj_emlrtRTEI = {
    693,            /* lineNo */
    32,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo hj_emlrtRTEI = {
    675,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ij_emlrtRTEI = {
    676,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo jj_emlrtRTEI = {
    688,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo kj_emlrtRTEI = {
    629,            /* lineNo */
    31,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo lj_emlrtRTEI = {
    33,              /* lineNo */
    5,               /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtRTEInfo mj_emlrtRTEI = {
    38,              /* lineNo */
    1,               /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtRTEInfo nj_emlrtRTEI = {
    39,              /* lineNo */
    1,               /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

/* Function Definitions */
void FuserManager_distance(const emlrtStack *sp, trackFuser *obj,
                           const emxArray_struct0_T *localTracks,
                           emxArray_real_T *costMatrix)
{
  __m128d r2;
  __m128d r3;
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  emxArray_int32_T *r1;
  emxArray_objectTrack *centralTracks;
  emxArray_real_T *S;
  emxArray_real_T *allCovars;
  emxArray_real_T *allStates;
  emxArray_real_T *centralCovars;
  emxArray_real_T *centralStates;
  emxArray_real_T *e;
  emxArray_real_T *oneColumnCost;
  emxArray_real_T *r;
  emxArray_real_T *transformedCovars;
  emxArray_real_T *transformedStates;
  emxArray_uint32_T *trackClasses;
  fuserSourceConfiguration *sourceConfig;
  objectTrack b_obj;
  objectTrack c_obj;
  objectTrack *centralTracks_data;
  const struct0_T *localTracks_data;
  real_T x[36];
  real_T Y[6];
  real_T dv[2];
  real_T b_Y;
  real_T n;
  real_T numCentralTracks;
  real_T y;
  real_T *S_data;
  real_T *centralCovars_data;
  real_T *costMatrix_data;
  real_T *e_data;
  real_T *transformedCovars_data;
  real_T *transformedStates_data;
  int32_T ipiv[6];
  int32_T b_iv[2];
  int32_T FuserManager_distance_numThreads;
  int32_T b_i;
  int32_T b_k;
  int32_T b_loop_ub;
  int32_T bcoef;
  int32_T c_i;
  int32_T c_loop_ub;
  int32_T d_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T iacol;
  int32_T ibmat;
  int32_T ibtile;
  int32_T k;
  int32_T loop_ub;
  int32_T *r4;
  uint32_T u;
  uint32_T *trackClasses_data;
  boolean_T b;
  boolean_T b1;
  boolean_T b_overflow;
  boolean_T emlrtHadParallelError = false;
  boolean_T isodd;
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  d_st.prev = &b_st;
  d_st.tls = b_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  localTracks_data = localTracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  numCentralTracks = obj->pNumLiveTracks;
  if (!(numCentralTracks >= 0.0)) {
    emlrtNonNegativeCheckR2012b(numCentralTracks, &sc_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  n = (int32_T)muDoubleScalarFloor(numCentralTracks);
  if (numCentralTracks != n) {
    emlrtIntegerCheckR2012b(numCentralTracks, &rc_emlrtDCI, (emlrtConstCTX)sp);
  }
  iacol = (int32_T)numCentralTracks;
  b_iv[0] = (int32_T)numCentralTracks;
  i = localTracks->size[0];
  b_iv[1] = localTracks->size[0];
  loop_ub = (int32_T)numCentralTracks;
  ibtile = costMatrix->size[0] * costMatrix->size[1];
  costMatrix->size[0] = (int32_T)numCentralTracks;
  costMatrix->size[1] = localTracks->size[0];
  emxEnsureCapacity_real_T(sp, costMatrix, ibtile, &qi_emlrtRTEI);
  costMatrix_data = costMatrix->data;
  ibtile = (int32_T)numCentralTracks * localTracks->size[0];
  for (b_i = 0; b_i < ibtile; b_i++) {
    costMatrix_data[b_i] = rtInf;
  }
  emxInit_real_T(sp, &transformedStates, 2, &hj_emlrtRTEI);
  emxInit_real_T(sp, &transformedCovars, 3, &ij_emlrtRTEI);
  emxInit_real_T(sp, &allStates, 2, &bj_emlrtRTEI);
  emxInit_real_T(sp, &allCovars, 3, &jj_emlrtRTEI);
  emxInit_real_T(sp, &oneColumnCost, 2, &dj_emlrtRTEI);
  emxInit_objectTrack(sp, &centralTracks, &kj_emlrtRTEI);
  emxInit_uint32_T(sp, &trackClasses, 2, &kj_emlrtRTEI);
  emxInit_real_T(sp, &centralStates, 2, &kj_emlrtRTEI);
  emxInit_real_T(sp, &centralCovars, 3, &kj_emlrtRTEI);
  emxInitStruct_objectTrack(sp, &b_obj, &wi_emlrtRTEI, true);
  emxInit_real_T(sp, &r, 2, &lj_emlrtRTEI);
  emxInit_real_T(sp, &e, 2, &mj_emlrtRTEI);
  emxInit_real_T(sp, &S, 3, &nj_emlrtRTEI);
  emxInit_int32_T(sp, &r1, 2, &gj_emlrtRTEI);
  emxInitMatrix_objectTrack1(sp, &c_obj, &si_emlrtRTEI);
  if ((localTracks->size[0] != 0) && (!(obj->pNumLiveTracks == 0.0))) {
    st.site = &pi_emlrtRSI;
    emxCopyStruct_objectTrack(&st, &b_obj, &obj->pTracksList[0], &ri_emlrtRTEI);
    emxCopyStruct_objectTrack(&st, &c_obj, &b_obj, &si_emlrtRTEI);
    dv[0] = 1.0;
    dv[1] = numCentralTracks;
    b_st.site = &bj_emlrtRSI;
    repmat(&b_st, &c_obj, dv, centralTracks);
    centralTracks_data = centralTracks->data;
    ibtile = trackClasses->size[0] * trackClasses->size[1];
    trackClasses->size[0] = 1;
    emxEnsureCapacity_uint32_T(&st, trackClasses, ibtile, &ti_emlrtRTEI);
    if (numCentralTracks != n) {
      emlrtIntegerCheckR2012b(numCentralTracks, &tc_emlrtDCI, &st);
    }
    ibtile = trackClasses->size[0] * trackClasses->size[1];
    trackClasses->size[1] = (int32_T)numCentralTracks;
    emxEnsureCapacity_uint32_T(&st, trackClasses, ibtile, &ti_emlrtRTEI);
    trackClasses_data = trackClasses->data;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, numCentralTracks, mxDOUBLE_CLASS,
                                  (int32_T)numCentralTracks, &hc_emlrtRTEI,
                                  &st);
    for (b_i = 0; b_i < iacol; b_i++) {
      if (b_i > 99) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, 99, &mi_emlrtBCI, &st);
      }
      if (b_i > centralTracks->size[1] - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, centralTracks->size[1] - 1,
                                      &ni_emlrtBCI, &st);
      }
      emxCopyStruct_objectTrack(&st, &centralTracks_data[b_i],
                                &obj->pTracksList[b_i], &ui_emlrtRTEI);
      if (b_i + 1 > trackClasses->size[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trackClasses->size[1],
                                      &oi_emlrtBCI, &st);
      }
      n = muDoubleScalarRound(obj->pTracksList[b_i].ObjectClassID);
      if (n < 4.294967296E+9) {
        if (n >= 0.0) {
          u = (uint32_T)n;
        } else {
          u = 0U;
        }
      } else if (n >= 4.294967296E+9) {
        u = MAX_uint32_T;
      } else {
        u = 0U;
      }
      trackClasses_data[b_i] = u;
    }
    st.site = &qi_emlrtRSI;
    ibtile = centralStates->size[0] * centralStates->size[1];
    centralStates->size[0] = 6;
    i1 = centralTracks->size[1];
    centralStates->size[1] = centralTracks->size[1];
    emxEnsureCapacity_real_T(&st, centralStates, ibtile, &vi_emlrtRTEI);
    transformedStates_data = centralStates->data;
    ibtile = centralCovars->size[0] * centralCovars->size[1] *
             centralCovars->size[2];
    centralCovars->size[0] = 6;
    centralCovars->size[1] = 6;
    centralCovars->size[2] = centralTracks->size[1];
    emxEnsureCapacity_real_T(&st, centralCovars, ibtile, &vi_emlrtRTEI);
    centralCovars_data = centralCovars->data;
    ibtile = centralTracks->size[1];
    b_loop_ub = centralTracks->size[1] * 36;
    if (b_loop_ub < 800) {
      for (c_i = 0; c_i < i1; c_i++) {
        if (c_i > centralTracks->size[1] - 1) {
          emlrtDynamicBoundsCheckR2012b(c_i, 0, centralTracks->size[1] - 1,
                                        &fi_emlrtBCI, &st);
        }
        if (c_i + 1 > i1) {
          emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, i1, &ai_emlrtBCI, &st);
        }
        for (i2 = 0; i2 < 6; i2++) {
          transformedStates_data[i2 + 6 * c_i] =
              centralTracks_data[c_i].pState[i2];
        }
        if (c_i > centralTracks->size[1] - 1) {
          emlrtDynamicBoundsCheckR2012b(c_i, 0, centralTracks->size[1] - 1,
                                        &ei_emlrtBCI, &st);
        }
        if (c_i + 1 > i1) {
          emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, i1, &yh_emlrtBCI, &st);
        }
        for (i2 = 0; i2 < 6; i2++) {
          for (i3 = 0; i3 < 6; i3++) {
            ibtile = i3 + 6 * i2;
            centralCovars_data[ibtile + 36 * c_i] =
                centralTracks_data[c_i].pStateCovariance[ibtile];
          }
        }
      }
    } else {
      emlrtEnterParallelRegion(&st, omp_in_parallel());
      emlrtPushJmpBuf(&st, &emlrtJBStack);
      FuserManager_distance_numThreads =
          emlrtAllocRegionTLSs(st.tls, omp_in_parallel(), omp_get_max_threads(),
                               omp_get_num_procs());
#pragma omp parallel num_threads(FuserManager_distance_numThreads) private(    \
        i2, i3, c_st, emlrtJBEnviron, i4)                                      \
    firstprivate(st, emlrtHadParallelError)
      {
        if (setjmp(emlrtJBEnviron) == 0) {
          c_st.prev = &st;
          c_st.tls = emlrtAllocTLS(&st, omp_get_thread_num());
          c_st.site = NULL;
          emlrtSetJmpBuf(&c_st, &emlrtJBEnviron);
        } else {
          emlrtHadParallelError = true;
        }
#pragma omp for nowait
        for (c_i = 0; c_i < ibtile; c_i++) {
          if (emlrtHadParallelError) {
            continue;
          }
          if (setjmp(emlrtJBEnviron) == 0) {
            if (c_i > centralTracks->size[1] - 1) {
              emlrtDynamicBoundsCheckR2012b(c_i, 0, centralTracks->size[1] - 1,
                                            &fi_emlrtBCI, &c_st);
            }
            if (c_i + 1 > centralStates->size[1]) {
              emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, centralStates->size[1],
                                            &ai_emlrtBCI, &c_st);
            }
            for (i2 = 0; i2 < 6; i2++) {
              transformedStates_data[i2 + 6 * c_i] =
                  centralTracks_data[c_i].pState[i2];
            }
            if (c_i > centralTracks->size[1] - 1) {
              emlrtDynamicBoundsCheckR2012b(c_i, 0, centralTracks->size[1] - 1,
                                            &ei_emlrtBCI, &c_st);
            }
            if (c_i + 1 > centralStates->size[1]) {
              emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, centralStates->size[1],
                                            &yh_emlrtBCI, &c_st);
            }
            for (i2 = 0; i2 < 6; i2++) {
              for (i3 = 0; i3 < 6; i3++) {
                i4 = i3 + 6 * i2;
                centralCovars_data[i4 + 36 * c_i] =
                    centralTracks_data[c_i].pStateCovariance[i4];
              }
            }
          } else {
            emlrtHadParallelError = true;
          }
        }
      }
      emlrtPopJmpBuf(&st, &emlrtJBStack);
      emlrtExitParallelRegion(&st, omp_in_parallel());
    }
    emxCopyStruct_objectTrack(sp, &b_obj, &obj->pTracksList[0], &wi_emlrtRTEI);
    iacol = costMatrix->size[0] * costMatrix->size[1];
    costMatrix->size[0] = b_iv[0];
    costMatrix->size[1] = b_iv[1];
    emxEnsureCapacity_real_T(sp, costMatrix, iacol, &xi_emlrtRTEI);
    costMatrix_data = costMatrix->data;
    overflow = ((int32_T)numCentralTracks > 2147483646);
    b = muDoubleScalarIsInf(numCentralTracks);
    if ((!b) && (!(numCentralTracks > 2.147483647E+9))) {
      b1 = true;
    } else {
      b1 = false;
    }
    if (numCentralTracks <= 0.0) {
      n = 0.0;
    } else {
      n = numCentralTracks;
    }
    b_overflow = ((int32_T)numCentralTracks > 2147483646);
    for (d_i = 0; d_i < i; d_i++) {
      if (d_i + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, i, &li_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      ibtile = S->size[0] * S->size[1] * S->size[2];
      S->size[0] = 6;
      S->size[1] = 6;
      S->size[2] = i1;
      emxEnsureCapacity_real_T(sp, S, ibtile, &yi_emlrtRTEI);
      S_data = S->data;
      for (b_i = 0; b_i < b_loop_ub; b_i++) {
        S_data[b_i] = centralCovars_data[b_i];
      }
      st.site = &ri_emlrtRSI;
      gaussEKFilter_predict(
          &st, centralStates, S, obj->ProcessNoise,
          localTracks_data[d_i].UpdateTime - b_obj.pUpdateTime, e);
      e_data = e->data;
      S_data = S->data;
      st.site = &si_emlrtRSI;
      b_st.site = &vg_emlrtRSI;
      sourceConfig = FuserManager_getConfigByID(
          &b_st, obj, localTracks_data[d_i].SourceIndex);
      st.site = &ti_emlrtRSI;
      b_st.site = &cj_emlrtRSI;
      if (b || (numCentralTracks > 2.147483647E+9)) {
        emlrtErrorWithMessageIdR2018a(&b_st, &eb_emlrtRTEI,
                                      "Coder:MATLAB:NonIntegerInput",
                                      "Coder:MATLAB:NonIntegerInput", 4, 12,
                                      MIN_int32_T, 12, MAX_int32_T);
      }
      ibtile = transformedStates->size[0] * transformedStates->size[1];
      transformedStates->size[0] = 6;
      transformedStates->size[1] = loop_ub;
      emxEnsureCapacity_real_T(&st, transformedStates, ibtile, &aj_emlrtRTEI);
      transformedStates_data = transformedStates->data;
      b_st.site = &fj_emlrtRSI;
      if (overflow) {
        d_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      for (b_i = 0; b_i < loop_ub; b_i++) {
        ibtile = b_i * 6;
        for (k = 0; k < 6; k++) {
          transformedStates_data[ibtile + k] = localTracks_data[d_i].State[k];
        }
      }
      st.site = &ui_emlrtRSI;
      b_st.site = &cj_emlrtRSI;
      if (!b1) {
        emlrtErrorWithMessageIdR2018a(&b_st, &eb_emlrtRTEI,
                                      "Coder:MATLAB:NonIntegerInput",
                                      "Coder:MATLAB:NonIntegerInput", 4, 12,
                                      MIN_int32_T, 12, MAX_int32_T);
      }
      if (!(n <= 2.147483647E+9)) {
        emlrtErrorWithMessageIdR2018a(&b_st, &db_emlrtRTEI,
                                      "Coder:MATLAB:pmaxsize",
                                      "Coder:MATLAB:pmaxsize", 0);
      }
      ibtile = transformedCovars->size[0] * transformedCovars->size[1] *
               transformedCovars->size[2];
      transformedCovars->size[0] = 6;
      transformedCovars->size[1] = 6;
      transformedCovars->size[2] = loop_ub;
      emxEnsureCapacity_real_T(&st, transformedCovars, ibtile, &aj_emlrtRTEI);
      transformedCovars_data = transformedCovars->data;
      b_st.site = &fj_emlrtRSI;
      if (b_overflow) {
        d_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      for (b_i = 0; b_i < loop_ub; b_i++) {
        ibtile = b_i * 36 - 1;
        for (k = 0; k < 6; k++) {
          iacol = k * 6;
          ibmat = ibtile + k * 6;
          for (b_k = 0; b_k < 6; b_k++) {
            transformedCovars_data[(ibmat + b_k) + 1] =
                localTracks_data[d_i].StateCovariance[iacol + b_k];
          }
        }
      }
      emlrtForLoopVectorCheckR2021a(1.0, 1.0, numCentralTracks, mxDOUBLE_CLASS,
                                    (int32_T)numCentralTracks, &ic_emlrtRTEI,
                                    (emlrtConstCTX)sp);
      for (k = 0; k < loop_ub; k++) {
        if (k > centralTracks->size[1] - 1) {
          emlrtDynamicBoundsCheckR2012b(k, 0, centralTracks->size[1] - 1,
                                        &gi_emlrtBCI, (emlrtConstCTX)sp);
        }
        st.site = &vi_emlrtRSI;
        if (k + 1 > e->size[1]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, e->size[1], &ki_emlrtBCI,
                                        &st);
        }
        b_st.site = &sc_emlrtRSI;
        validateattributes(&b_st, &e_data[6 * k]);
        st.site = &wi_emlrtRSI;
        if (k + 1 > S->size[2]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, S->size[2], &ji_emlrtBCI,
                                        &st);
        }
        b_st.site = &tc_emlrtRSI;
        b_validateattributes(&b_st, &S_data[36 * k]);
        b_st.site = &uc_emlrtRSI;
        isSymmetricPositiveSemiDefinite(&b_st, &S_data[36 * k]);
        st.site = &xi_emlrtRSI;
        if (!sourceConfig->pIsTransformToLocalValid) {
          b_st.site = &gj_emlrtRSI;
          sourceConfig->pIsTransformToLocalValid = true;
        }
        if (k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, loop_ub, &xh_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        for (b_i = 0; b_i < 6; b_i++) {
          iacol = b_i + 6 * k;
          transformedStates_data[iacol] = e_data[iacol];
        }
        if (k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, loop_ub, &wh_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        for (b_i = 0; b_i < 36; b_i++) {
          iacol = b_i + k * 36;
          transformedCovars_data[iacol] = S_data[iacol];
        }
      }
      ibtile = allStates->size[0] * allStates->size[1];
      allStates->size[0] = 6;
      c_loop_ub = transformedStates->size[1] + 1;
      allStates->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(sp, allStates, ibtile, &bj_emlrtRTEI);
      S_data = allStates->data;
      for (b_i = 0; b_i < 6; b_i++) {
        S_data[b_i] = localTracks_data[d_i].State[b_i];
      }
      for (b_i = 0; b_i < loop_ub; b_i++) {
        for (k = 0; k < 6; k++) {
          S_data[k + 6 * (b_i + 1)] = transformedStates_data[k + 6 * b_i];
        }
      }
      st.site = &yi_emlrtRSI;
      ibtile = allCovars->size[0] * allCovars->size[1] * allCovars->size[2];
      allCovars->size[0] = 6;
      allCovars->size[1] = 6;
      allCovars->size[2] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&st, allCovars, ibtile, &cj_emlrtRTEI);
      transformedStates_data = allCovars->data;
      for (b_i = 0; b_i < 36; b_i++) {
        transformedStates_data[b_i] =
            localTracks_data[d_i].StateCovariance[b_i];
      }
      iacol = 36 * transformedCovars->size[2];
      b_st.site = &hj_emlrtRSI;
      if (iacol > 2147483646) {
        d_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      for (b_i = 0; b_i < iacol; b_i++) {
        transformedStates_data[b_i + 36] = transformedCovars_data[b_i];
      }
      st.site = &aj_emlrtRSI;
      ibtile = oneColumnCost->size[0] * oneColumnCost->size[1];
      oneColumnCost->size[0] = 1;
      oneColumnCost->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&st, oneColumnCost, ibtile, &dj_emlrtRTEI);
      transformedCovars_data = oneColumnCost->data;
      for (b_i = 0; b_i < c_loop_ub; b_i++) {
        transformedCovars_data[b_i] = 0.0;
      }
      b_st.site = &ij_emlrtRSI;
      ibtile = e->size[0] * e->size[1];
      e->size[0] = 6;
      e->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&b_st, e, ibtile, &ej_emlrtRTEI);
      e_data = e->data;
      bcoef = (allStates->size[1] != 1);
      for (b_i = 0; b_i < c_loop_ub; b_i++) {
        ibtile = bcoef * b_i;
        r2 = _mm_loadu_pd(&S_data[0]);
        r3 = _mm_loadu_pd(&S_data[6 * ibtile]);
        _mm_storeu_pd(&e_data[6 * b_i], _mm_sub_pd(r2, r3));
        r2 = _mm_loadu_pd(&S_data[2]);
        r3 = _mm_loadu_pd(&S_data[6 * ibtile + 2]);
        _mm_storeu_pd(&e_data[6 * b_i + 2], _mm_sub_pd(r2, r3));
        r2 = _mm_loadu_pd(&S_data[4]);
        r3 = _mm_loadu_pd(&S_data[6 * ibtile + 4]);
        _mm_storeu_pd(&e_data[6 * b_i + 4], _mm_sub_pd(r2, r3));
      }
      ibtile = S->size[0] * S->size[1] * S->size[2];
      S->size[0] = 6;
      S->size[1] = 6;
      S->size[2] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&b_st, S, ibtile, &ej_emlrtRTEI);
      S_data = S->data;
      for (b_i = 0; b_i < c_loop_ub; b_i++) {
        ibtile = bcoef * b_i;
        for (k = 0; k < 6; k++) {
          r2 = _mm_loadu_pd(&transformedStates_data[6 * k]);
          iacol = 6 * k + 36 * ibtile;
          r3 = _mm_loadu_pd(&transformedStates_data[iacol]);
          ibmat = 6 * k + 36 * b_i;
          _mm_storeu_pd(&S_data[ibmat], _mm_add_pd(r2, r3));
          r2 = _mm_loadu_pd(&transformedStates_data[6 * k + 2]);
          r3 = _mm_loadu_pd(&transformedStates_data[iacol + 2]);
          _mm_storeu_pd(&S_data[ibmat + 2], _mm_add_pd(r2, r3));
          r2 = _mm_loadu_pd(&transformedStates_data[6 * k + 4]);
          r3 = _mm_loadu_pd(&transformedStates_data[iacol + 4]);
          _mm_storeu_pd(&S_data[ibmat + 4], _mm_add_pd(r2, r3));
        }
      }
      ibtile = r->size[0] * r->size[1];
      r->size[0] = 1;
      r->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&b_st, r, ibtile, &fj_emlrtRTEI);
      transformedStates_data = r->data;
      for (k = 0; k < c_loop_ub; k++) {
        if (k + 1 > e->size[1]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, e->size[1], &di_emlrtBCI,
                                        &b_st);
        }
        d_st.site = &jj_emlrtRSI;
        if (k + 1 > S->size[2]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, S->size[2], &ci_emlrtBCI,
                                        &d_st);
        }
        for (b_i = 0; b_i < 6; b_i++) {
          Y[b_i] = e_data[b_i + 6 * k];
        }
        e_st.site = &kj_emlrtRSI;
        mrdiv(&e_st, Y, &S_data[36 * k]);
        d_st.site = &jj_emlrtRSI;
        if (k + 1 > S->size[2]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, S->size[2], &bi_emlrtBCI,
                                        &d_st);
        }
        e_st.site = &ak_emlrtRSI;
        for (b_i = 0; b_i < 36; b_i++) {
          x[b_i] = S_data[b_i + k * 36];
        }
        f_st.site = &rj_emlrtRSI;
        xzgetrf(&f_st, x, ipiv);
        y = x[0];
        isodd = false;
        for (b_i = 0; b_i < 5; b_i++) {
          y *= x[(b_i + 6 * (b_i + 1)) + 1];
          if (ipiv[b_i] > b_i + 1) {
            isodd = !isodd;
          }
        }
        if (isodd) {
          y = -y;
        }
        d_st.site = &jj_emlrtRSI;
        if (y < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &d_st, &vb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
              "Coder:toolbox:ElFunDomainError", 3, 4, 3, "log");
        }
        b_Y = 0.0;
        for (b_i = 0; b_i < 6; b_i++) {
          b_Y += Y[b_i] * e_data[b_i + 6 * k];
        }
        if (k + 1 > r->size[1]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, r->size[1], &ri_emlrtBCI,
                                        &b_st);
        }
        transformedStates_data[k] = b_Y + muDoubleScalarLog(y);
      }
      b_iv[0] = 1;
      b_iv[1] = transformedStates->size[1] + 1;
      emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &r->size[0], 2, &g_emlrtECI,
                                    &st);
      for (b_i = 0; b_i < c_loop_ub; b_i++) {
        transformedCovars_data[b_i] = transformedStates_data[b_i];
      }
      if (oneColumnCost->size[1] < 2) {
        iacol = 0;
        ibtile = 0;
      } else {
        iacol = 1;
        ibtile = transformedStates->size[1] + 1;
      }
      if (d_i + 1 > costMatrix->size[1]) {
        emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, costMatrix->size[1],
                                      &ii_emlrtBCI, (emlrtConstCTX)sp);
      }
      ibmat = costMatrix->size[0];
      b_iv[0] = 1;
      b_iv[1] = ibtile - iacol;
      emlrtSubAssignSizeCheckR2012b(&costMatrix->size[0], 1, &b_iv[0], 2,
                                    &h_emlrtECI, (emlrtCTX)sp);
      for (b_i = 0; b_i < ibmat; b_i++) {
        costMatrix_data[b_i + costMatrix->size[0] * d_i] =
            transformedCovars_data[iacol + b_i];
      }
      if (localTracks_data[d_i].ObjectClassID != 0.0) {
        ibmat = trackClasses->size[1];
        iacol = 0;
        for (b_i = 0; b_i < ibmat; b_i++) {
          u = trackClasses_data[b_i];
          if ((u != localTracks_data[d_i].ObjectClassID) && (u != 0U)) {
            iacol++;
          }
        }
        ibtile = r1->size[0] * r1->size[1];
        r1->size[0] = 1;
        r1->size[1] = iacol;
        emxEnsureCapacity_int32_T(sp, r1, ibtile, &gj_emlrtRTEI);
        r4 = r1->data;
        ibtile = 0;
        for (b_i = 0; b_i < ibmat; b_i++) {
          u = trackClasses_data[b_i];
          if ((u != localTracks_data[d_i].ObjectClassID) && (u != 0U)) {
            r4[ibtile] = b_i;
            ibtile++;
          }
        }
        if (d_i + 1 > costMatrix->size[1]) {
          emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, costMatrix->size[1],
                                        &hi_emlrtBCI, (emlrtConstCTX)sp);
        }
        ibtile = r1->size[1];
        for (b_i = 0; b_i < ibtile; b_i++) {
          if ((r4[b_i] < 0) || (r4[b_i] > costMatrix->size[0] - 1)) {
            emlrtDynamicBoundsCheckR2012b(r4[b_i], 0, costMatrix->size[0] - 1,
                                          &qi_emlrtBCI, (emlrtConstCTX)sp);
          }
          costMatrix_data[r4[b_i] + costMatrix->size[0] * d_i] = rtInf;
        }
      }
    }
    ibtile = costMatrix->size[0] * costMatrix->size[1];
    for (b_i = 0; b_i < ibtile; b_i++) {
      iacol = costMatrix->size[0] * costMatrix->size[1];
      if (costMatrix_data[b_i] > 100.0) {
        if (b_i > iacol - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, iacol - 1, &pi_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        costMatrix_data[b_i] = rtInf;
      }
    }
  }
  emxFreeMatrix_objectTrack1(sp, &c_obj);
  emxFree_int32_T(sp, &r1);
  emxFree_real_T(sp, &S);
  emxFree_real_T(sp, &e);
  emxFree_real_T(sp, &r);
  emxFreeStruct_objectTrack(sp, &b_obj);
  emxFree_real_T(sp, &centralCovars);
  emxFree_real_T(sp, &centralStates);
  emxFree_uint32_T(sp, &trackClasses);
  emxFree_objectTrack(sp, &centralTracks);
  emxFree_real_T(sp, &oneColumnCost);
  emxFree_real_T(sp, &allCovars);
  emxFree_real_T(sp, &allStates);
  emxFree_real_T(sp, &transformedCovars);
  emxFree_real_T(sp, &transformedStates);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

fuserSourceConfiguration *FuserManager_getConfigByID(const emlrtStack *sp,
                                                     trackFuser *obj,
                                                     uint32_T configID)
{
  fuserSourceConfiguration *config;
  int32_T k;
  boolean_T inKnownIDs[2];
  boolean_T exitg1;
  boolean_T y;
  inKnownIDs[0] = (configID == obj->pSourceConfigIDs[0]);
  inKnownIDs[1] = (configID == obj->pSourceConfigIDs[1]);
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (inKnownIDs[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!y) {
    emlrtErrorWithMessageIdR2018a(sp, &u_emlrtRTEI,
                                  "fusion:trackFuser:UnknownConfig",
                                  "fusion:trackFuser:UnknownConfig", 6, 4, 11,
                                  "SourceIndex", 4, 20, "SourceConfigurations");
  }
  k = 0;
  int32_T exitg2;
  do {
    exitg2 = 0;
    if (k < 2) {
      if (inKnownIDs[k]) {
        config = obj->pSourceConfigurations[k];
        exitg2 = 1;
      } else {
        k++;
      }
    } else {
      emlrtErrorWithMessageIdR2018a(
          sp, &v_emlrtRTEI, "fusion:trackFuser:UnknownConfig",
          "fusion:trackFuser:UnknownConfig", 6, 4, 11, "SourceIndex", 4, 20,
          "SourceConfigurations");
    }
  } while (exitg2 == 0);
  return config;
}

void FuserManager_setupImpl(const emlrtStack *sp, trackFuser *obj,
                            const emxArray_struct0_T *tracks)
{
  static const real_T stateCov[36] = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  b_objectTrack expl_temp;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  fuserSourceConfiguration *sourceConfig;
  objectTrack b[100];
  objectTrack b_expl_temp;
  const struct0_T *tracks_data;
  real_T imvec[6];
  real_T dv[3];
  real_T varargin_1[3];
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  tracks_data = tracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &bc_emlrtRSI;
  b_st.site = &hc_emlrtRSI;
  objectTrack_objectTrack(
      &b_st, tracks_data[0].SourceIndex, tracks_data[0].UpdateTime,
      tracks_data[0].State, tracks_data[0].StateCovariance,
      tracks_data[0].ObjectClassID,
      tracks_data[0].ObjectClassProbabilities.data,
      tracks_data[0].ObjectClassProbabilities.size, tracks_data[0].TrackLogic,
      tracks_data[0].TrackLogicState.data, tracks_data[0].TrackLogicState.size,
      tracks_data[0].IsConfirmed, tracks_data[0].IsCoasted,
      tracks_data[0].IsSelfReported, tracks_data[0].ObjectAttributes,
      &expl_temp);
  st.site = &cc_emlrtRSI;
  b_st.site = &vg_emlrtRSI;
  sourceConfig = FuserManager_getConfigByID(&b_st, obj, expl_temp.SourceIndex);
  st.site = &dc_emlrtRSI;
  if (!sourceConfig->pIsTransformToCentralValid) {
    b_st.site = &wg_emlrtRSI;
    sourceConfig->pIsTransformToCentralValid = true;
  }
  st.site = &ec_emlrtRSI;
  b_st.site = &xg_emlrtRSI;
  c_st.site = &bh_emlrtRSI;
  d_validateattributes(&c_st, expl_temp.pState);
  varargin_1[0] = 0.0;
  varargin_1[1] = 0.0;
  varargin_1[2] = 0.0;
  b_st.site = &yg_emlrtRSI;
  c_st.site = &ch_emlrtRSI;
  d_st.site = &bh_emlrtRSI;
  d_validateattributes(&d_st, expl_temp.pState);
  dv[0] = 0.0;
  dv[1] = 0.0;
  dv[2] = 0.0;
  d_st.site = &eh_emlrtRSI;
  e_validateattributes(&d_st, dv);
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      imvec[b_i] = expl_temp.pState[b_i];
    }
    real_T d;
    d = expl_temp.pState[i];
    imvec[i] =
        d + muDoubleScalarMax(1.4901161193847656E-8,
                              1.4901161193847656E-8 * muDoubleScalarAbs(d));
    c_st.site = &dh_emlrtRSI;
    constvel(&c_st, imvec, varargin_1, 1.0);
  }
  b_st.site = &ah_emlrtRSI;
  c_st.site = &ch_emlrtRSI;
  d_st.site = &bh_emlrtRSI;
  d_validateattributes(&d_st, expl_temp.pState);
  d_st.site = &eh_emlrtRSI;
  e_validateattributes(&d_st, varargin_1);
  for (i = 0; i < 3; i++) {
    varargin_1[0] = 0.0;
    varargin_1[1] = 0.0;
    varargin_1[2] = 0.0;
    varargin_1[i] = 1.4901161193847656E-8;
    for (b_i = 0; b_i < 6; b_i++) {
      imvec[b_i] = expl_temp.pState[b_i];
    }
    c_st.site = &dh_emlrtRSI;
    constvel(&c_st, imvec, varargin_1, 1.0);
  }
  st.site = &fc_emlrtRSI;
  b_st.site = &hh_emlrtRSI;
  sourceConfig = FuserManager_getConfigByID(&b_st, obj, expl_temp.SourceIndex);
  b_st.site = &ih_emlrtRSI;
  if (!sourceConfig->pIsTransformToCentralValid) {
    c_st.site = &wg_emlrtRSI;
    sourceConfig->pIsTransformToCentralValid = true;
  }
  st.site = &gc_emlrtRSI;
  b_st.site = &ic_emlrtRSI;
  c_st.site = &jh_emlrtRSI;
  c_st.site = &jc_emlrtRSI;
  c_st.site = &kc_emlrtRSI;
  for (i = 0; i < 6; i++) {
    imvec[i] = 0.0;
  }
  d_st.site = &sc_emlrtRSI;
  validateattributes(&d_st, imvec);
  c_st.site = &lc_emlrtRSI;
  d_st.site = &tc_emlrtRSI;
  b_validateattributes(&d_st, stateCov);
  d_st.site = &uc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&d_st, stateCov);
  c_st.site = &nc_emlrtRSI;
  emxInitStruct_objectTrack(&b_st, &b_expl_temp, &sc_emlrtRTEI, true);
  c_i = b_expl_temp.pTrackLogicState->size[0] *
        b_expl_temp.pTrackLogicState->size[1];
  b_expl_temp.pTrackLogicState->size[0] = 1;
  b_expl_temp.pTrackLogicState->size[1] = 3;
  emxEnsureCapacity_boolean_T(&b_st, b_expl_temp.pTrackLogicState, c_i,
                              &rc_emlrtRTEI);
  b_expl_temp.pTrackLogicState->data[0] = false;
  b_expl_temp.pTrackLogicState->data[1] = false;
  b_expl_temp.pTrackLogicState->data[2] = false;
  emxInitMatrix_objectTrack(sp, b, &xc_emlrtRTEI, true);
  b_expl_temp.TrackID = MAX_uint32_T;
  b_expl_temp.BranchID = MAX_uint32_T;
  b_expl_temp.SourceIndex = 3U;
  b_expl_temp.Age = MAX_uint32_T;
  b_expl_temp.ObjectClassID = 0.0;
  b_expl_temp.ObjectClassProbabilities = 1.0;
  b_expl_temp.IsConfirmed = false;
  b_expl_temp.IsCoasted = false;
  b_expl_temp.IsSelfReported = false;
  b_expl_temp.ObjectAttributes.Category = expl_temp.ObjectAttributes.Category;
  b_expl_temp.pUpdateTime = 0.0;
  for (i = 0; i < 8; i++) {
    b_expl_temp.ObjectAttributes.Callsign[i] =
        expl_temp.ObjectAttributes.Callsign[i];
  }
  for (i = 0; i < 6; i++) {
    b_expl_temp.pState[i] = 0.0;
  }
  memcpy(&b_expl_temp.pStateCovariance[0], &stateCov[0], 36U * sizeof(real_T));
  for (i = 0; i < 100; i++) {
    emxCopyStruct_objectTrack(sp, &b[i], &b_expl_temp, &sc_emlrtRTEI);
  }
  emxFreeStruct_objectTrack(sp, &b_expl_temp);
  for (i = 0; i < 100; i++) {
    emxCopyStruct_objectTrack(sp, &obj->pTracksList[i], &b[i], &tc_emlrtRTEI);
  }
  emxFreeMatrix_objectTrack(sp, b);
  c_i = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = 1;
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, obj->pUsedConfigIDs, c_i, &uc_emlrtRTEI);
  obj->pUsedConfigIDs->data[0] = false;
  obj->pUsedConfigIDs->data[1] = false;
  c_i = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = 2;
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, obj->pUsedConfigIDs, c_i, &vc_emlrtRTEI);
  obj->pUsedConfigIDs->data[0] = false;
  obj->pUsedConfigIDs->data[1] = false;
  obj->pUsedConfigIDs->data[2] = false;
  obj->pUsedConfigIDs->data[3] = false;
  c_i = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = 1;
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, obj->pUsedConfigIDs, c_i, &wc_emlrtRTEI);
  obj->pUsedConfigIDs->data[0] = false;
  obj->pUsedConfigIDs->data[1] = false;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (FuserManager.c) */
