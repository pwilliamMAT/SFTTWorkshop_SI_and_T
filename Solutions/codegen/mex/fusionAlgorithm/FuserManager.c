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
#include "fusionAlgorithm.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
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
    530,                       /* lineNo */
    "FuserManager/parseTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI =
    {
        255,                       /* lineNo */
        "objectTrack/objectTrack", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI =
    {
        485,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI =
    {
        487,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI =
    {
        488,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo lc_emlrtRSI =
    {
        492,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo mc_emlrtRSI =
    {
        493,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI =
    {
        519,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI =
    {
        521,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI =
    {
        524,                         /* lineNo */
        "objectTrack/setProperties", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo sg_emlrtRSI =
    {
        324,                               /* lineNo */
        "objectTrack/set.TrackLogicState", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo tg_emlrtRSI =
    {
        600,                                   /* lineNo */
        "objectTrack/validateTrackLogicState", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo wj_emlrtRSI = {
    650,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo xj_emlrtRSI = {
    653,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo yj_emlrtRSI = {
    667,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ak_emlrtRSI = {
    673,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo bk_emlrtRSI = {
    675,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ck_emlrtRSI = {
    676,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo dk_emlrtRSI = {
    679,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ek_emlrtRSI = {
    680,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo fk_emlrtRSI = {
    683,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo gk_emlrtRSI = {
    688,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo hk_emlrtRSI = {
    689,                     /* lineNo */
    "FuserManager/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ik_emlrtRSI = {
    700,                          /* lineNo */
    "FuserManager/getLiveTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo rk_emlrtRSI =
    {
        137,   /* lineNo */
        "cat", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\cat.m" /* pathName
                                                                          */
};

static emlrtRTEInfo j_emlrtRTEI = {
    22,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatele.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatescalar", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatescalar.m" /* pName */
};

static emlrtRTEInfo n_emlrtRTEI =
    {
        527,                         /* lineNo */
        17,                          /* colNo */
        "objectTrack/setProperties", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pName */
};

static emlrtRTEInfo r_emlrtRTEI = {
    543,                          /* lineNo */
    85,                           /* colNo */
    "FuserManager/getConfigByID", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    553,                          /* lineNo */
    17,                           /* colNo */
    "FuserManager/getConfigByID", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = {
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

static emlrtRTEInfo fc_emlrtRTEI = {
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

static emlrtBCInfo hi_emlrtBCI = {
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

static emlrtBCInfo ii_emlrtBCI = {
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

static emlrtBCInfo ji_emlrtBCI = {
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

static emlrtRTEInfo gc_emlrtRTEI = {
    677,                     /* lineNo */
    25,                      /* colNo */
    "FuserManager/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtBCInfo ki_emlrtBCI = {
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

static emlrtBCInfo li_emlrtBCI = {
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

static emlrtBCInfo mi_emlrtBCI = {
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

static emlrtBCInfo ni_emlrtBCI = {
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

static emlrtBCInfo oi_emlrtBCI = {
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

static emlrtBCInfo pi_emlrtBCI = {
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

static emlrtRTEInfo sc_emlrtRTEI = {
    495,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = {
    496,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo uc_emlrtRTEI = {
    497,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo vg_emlrtRTEI = {
    639,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo wg_emlrtRTEI = {
    650,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo xg_emlrtRTEI = {
    653,            /* lineNo */
    13,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo yg_emlrtRTEI = {
    639,            /* lineNo */
    26,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ah_emlrtRTEI = {
    667,            /* lineNo */
    21,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ch_emlrtRTEI = {
    687,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo dh_emlrtRTEI =
    {
        107,   /* lineNo */
        28,    /* colNo */
        "cat", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\cat.m" /* pName
                                                                          */
};

static emlrtRTEInfo eh_emlrtRTEI = {
    689,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo gh_emlrtRTEI = {
    33,              /* lineNo */
    21,              /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtRTEInfo hh_emlrtRTEI = {
    693,            /* lineNo */
    32,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo ih_emlrtRTEI = {
    675,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo jh_emlrtRTEI = {
    676,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo kh_emlrtRTEI = {
    688,            /* lineNo */
    17,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo lh_emlrtRTEI = {
    629,            /* lineNo */
    31,             /* colNo */
    "FuserManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtRTEInfo mh_emlrtRTEI = {
    33,              /* lineNo */
    5,               /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtRTEInfo nh_emlrtRTEI = {
    38,              /* lineNo */
    1,               /* colNo */
    "gaussNormDiff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pName */
};

static emlrtRTEInfo oh_emlrtRTEI = {
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
  c_objectTrack b_obj;
  c_objectTrack *centralTracks_data;
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
  objectTrack expl_temp;
  objectTrack unusedExpr;
  const struct0_T *localTracks_data;
  real_T track_pStateCovariance[36];
  real_T Y[6];
  real_T b_dv[2];
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
  uint32_T sourceConfig_tmp;
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
  emxEnsureCapacity_real_T(sp, costMatrix, ibtile, &vg_emlrtRTEI);
  costMatrix_data = costMatrix->data;
  ibtile = (int32_T)numCentralTracks * localTracks->size[0];
  for (b_i = 0; b_i < ibtile; b_i++) {
    costMatrix_data[b_i] = rtInf;
  }
  emxInit_real_T(sp, &transformedStates, 2, &ih_emlrtRTEI);
  emxInit_real_T(sp, &transformedCovars, 3, &jh_emlrtRTEI);
  emxInit_real_T(sp, &allStates, 2, &ch_emlrtRTEI);
  emxInit_real_T(sp, &allCovars, 3, &kh_emlrtRTEI);
  emxInit_real_T(sp, &oneColumnCost, 2, &eh_emlrtRTEI);
  emxInit_objectTrack(sp, &centralTracks, &lh_emlrtRTEI);
  emxInit_uint32_T(sp, &trackClasses, 2, &lh_emlrtRTEI);
  emxInit_real_T(sp, &centralStates, 2, &lh_emlrtRTEI);
  emxInit_real_T(sp, &centralCovars, 3, &lh_emlrtRTEI);
  emxInit_real_T(sp, &r, 2, &mh_emlrtRTEI);
  emxInit_real_T(sp, &e, 2, &nh_emlrtRTEI);
  emxInit_real_T(sp, &S, 3, &oh_emlrtRTEI);
  emxInit_int32_T(sp, &r1, 2, &hh_emlrtRTEI);
  if ((localTracks->size[0] != 0) && (!(obj->pNumLiveTracks == 0.0))) {
    st.site = &wj_emlrtRSI;
    b_obj = obj->pTracksList[0];
    b_dv[0] = 1.0;
    b_dv[1] = numCentralTracks;
    b_st.site = &ik_emlrtRSI;
    repmat(&b_st, &b_obj, b_dv, centralTracks);
    centralTracks_data = centralTracks->data;
    ibtile = trackClasses->size[0] * trackClasses->size[1];
    trackClasses->size[0] = 1;
    emxEnsureCapacity_uint32_T(&st, trackClasses, ibtile, &wg_emlrtRTEI);
    if (numCentralTracks != n) {
      emlrtIntegerCheckR2012b(numCentralTracks, &tc_emlrtDCI, &st);
    }
    ibtile = trackClasses->size[0] * trackClasses->size[1];
    trackClasses->size[1] = (int32_T)numCentralTracks;
    emxEnsureCapacity_uint32_T(&st, trackClasses, ibtile, &wg_emlrtRTEI);
    trackClasses_data = trackClasses->data;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, numCentralTracks, mxDOUBLE_CLASS,
                                  (int32_T)numCentralTracks, &fc_emlrtRTEI,
                                  &st);
    for (b_i = 0; b_i < iacol; b_i++) {
      if (b_i > 99) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, 99, &li_emlrtBCI, &st);
      }
      if (b_i > centralTracks->size[1] - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, centralTracks->size[1] - 1,
                                      &mi_emlrtBCI, &st);
      }
      centralTracks_data[b_i] = obj->pTracksList[b_i];
      if (b_i + 1 > trackClasses->size[1]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trackClasses->size[1],
                                      &ni_emlrtBCI, &st);
      }
      n = muDoubleScalarRound(obj->pTracksList[b_i].ObjectClassID);
      if (n < 4.294967296E+9) {
        if (n >= 0.0) {
          sourceConfig_tmp = (uint32_T)n;
        } else {
          sourceConfig_tmp = 0U;
        }
      } else if (n >= 4.294967296E+9) {
        sourceConfig_tmp = MAX_uint32_T;
      } else {
        sourceConfig_tmp = 0U;
      }
      trackClasses_data[b_i] = sourceConfig_tmp;
    }
    st.site = &xj_emlrtRSI;
    ibtile = centralStates->size[0] * centralStates->size[1];
    centralStates->size[0] = 6;
    i1 = centralTracks->size[1];
    centralStates->size[1] = centralTracks->size[1];
    emxEnsureCapacity_real_T(&st, centralStates, ibtile, &xg_emlrtRTEI);
    transformedStates_data = centralStates->data;
    ibtile = centralCovars->size[0] * centralCovars->size[1] *
             centralCovars->size[2];
    centralCovars->size[0] = 6;
    centralCovars->size[1] = 6;
    centralCovars->size[2] = centralTracks->size[1];
    emxEnsureCapacity_real_T(&st, centralCovars, ibtile, &xg_emlrtRTEI);
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
    b_obj = obj->pTracksList[0];
    iacol = costMatrix->size[0] * costMatrix->size[1];
    costMatrix->size[0] = b_iv[0];
    costMatrix->size[1] = b_iv[1];
    emxEnsureCapacity_real_T(sp, costMatrix, iacol, &yg_emlrtRTEI);
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
        emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, i, &ki_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      ibtile = S->size[0] * S->size[1] * S->size[2];
      S->size[0] = 6;
      S->size[1] = 6;
      S->size[2] = i1;
      emxEnsureCapacity_real_T(sp, S, ibtile, &ah_emlrtRTEI);
      S_data = S->data;
      for (b_i = 0; b_i < b_loop_ub; b_i++) {
        S_data[b_i] = centralCovars_data[b_i];
      }
      st.site = &yj_emlrtRSI;
      gaussEKFilter_predict(
          &st, centralStates, S, obj->ProcessNoise,
          localTracks_data[d_i].UpdateTime - b_obj.pUpdateTime, e);
      e_data = e->data;
      S_data = S->data;
      st.site = &ak_emlrtRSI;
      sourceConfig_tmp = localTracks_data[d_i].SourceIndex;
      b_st.site = &ug_emlrtRSI;
      sourceConfig = FuserManager_getConfigByID(&b_st, obj, sourceConfig_tmp);
      st.site = &bk_emlrtRSI;
      b_st.site = &jk_emlrtRSI;
      if (b || (numCentralTracks > 2.147483647E+9)) {
        emlrtErrorWithMessageIdR2018a(&b_st, &bb_emlrtRTEI,
                                      "Coder:MATLAB:NonIntegerInput",
                                      "Coder:MATLAB:NonIntegerInput", 4, 12,
                                      MIN_int32_T, 12, MAX_int32_T);
      }
      ibtile = transformedStates->size[0] * transformedStates->size[1];
      transformedStates->size[0] = 6;
      transformedStates->size[1] = loop_ub;
      emxEnsureCapacity_real_T(&st, transformedStates, ibtile, &bh_emlrtRTEI);
      transformedStates_data = transformedStates->data;
      b_st.site = &mk_emlrtRSI;
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
      st.site = &ck_emlrtRSI;
      b_st.site = &jk_emlrtRSI;
      if (!b1) {
        emlrtErrorWithMessageIdR2018a(&b_st, &bb_emlrtRTEI,
                                      "Coder:MATLAB:NonIntegerInput",
                                      "Coder:MATLAB:NonIntegerInput", 4, 12,
                                      MIN_int32_T, 12, MAX_int32_T);
      }
      if (!(n <= 2.147483647E+9)) {
        emlrtErrorWithMessageIdR2018a(&b_st, &ab_emlrtRTEI,
                                      "Coder:MATLAB:pmaxsize",
                                      "Coder:MATLAB:pmaxsize", 0);
      }
      ibtile = transformedCovars->size[0] * transformedCovars->size[1] *
               transformedCovars->size[2];
      transformedCovars->size[0] = 6;
      transformedCovars->size[1] = 6;
      transformedCovars->size[2] = loop_ub;
      emxEnsureCapacity_real_T(&st, transformedCovars, ibtile, &bh_emlrtRTEI);
      transformedCovars_data = transformedCovars->data;
      b_st.site = &mk_emlrtRSI;
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
                                    (int32_T)numCentralTracks, &gc_emlrtRTEI,
                                    (emlrtConstCTX)sp);
      for (k = 0; k < loop_ub; k++) {
        if (k > centralTracks->size[1] - 1) {
          emlrtDynamicBoundsCheckR2012b(k, 0, centralTracks->size[1] - 1,
                                        &pi_emlrtBCI, (emlrtConstCTX)sp);
        }
        st.site = &dk_emlrtRSI;
        if (k + 1 > e->size[1]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, e->size[1], &ji_emlrtBCI,
                                        &st);
        }
        b_st.site = &rc_emlrtRSI;
        validateattributes(&b_st, &e_data[6 * k]);
        for (b_i = 0; b_i < 6; b_i++) {
          Y[b_i] = e_data[b_i + 6 * k];
        }
        st.site = &ek_emlrtRSI;
        if (k + 1 > S->size[2]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, S->size[2], &ii_emlrtBCI,
                                        &st);
        }
        b_st.site = &sc_emlrtRSI;
        b_validateattributes(&b_st, &S_data[36 * k]);
        b_st.site = &tc_emlrtRSI;
        isSymmetricPositiveSemiDefinite(&b_st, &S_data[36 * k]);
        for (b_i = 0; b_i < 36; b_i++) {
          track_pStateCovariance[b_i] = S_data[b_i + k * 36];
        }
        st.site = &fk_emlrtRSI;
        if (!sourceConfig->pIsTransformToLocalValid) {
          b_st.site = &nk_emlrtRSI;
          d_st.site = &pk_emlrtRSI;
          central2local(
              &d_st, centralTracks_data[k].TrackID,
              centralTracks_data[k].BranchID, sourceConfig_tmp,
              centralTracks_data[k].Age, centralTracks_data[k].ObjectClassID,
              centralTracks_data[k].ObjectClassProbabilities,
              centralTracks_data[k].IsConfirmed,
              centralTracks_data[k].IsCoasted,
              centralTracks_data[k].IsSelfReported, Y, track_pStateCovariance,
              centralTracks_data[k].pUpdateTime, &expl_temp);
          d_st.site = &qk_emlrtRSI;
          b_local2central(
              &d_st, expl_temp.TrackID, expl_temp.BranchID,
              expl_temp.SourceIndex, expl_temp.Age, expl_temp.ObjectClassID,
              expl_temp.ObjectClassProbabilities, expl_temp.IsConfirmed,
              expl_temp.IsCoasted, expl_temp.IsSelfReported, expl_temp.pState,
              expl_temp.pStateCovariance, expl_temp.pUpdateTime, &unusedExpr);
          sourceConfig->pIsTransformToLocalValid = true;
        }
        b_st.site = &ok_emlrtRSI;
        central2local(
            &b_st, centralTracks_data[k].TrackID,
            centralTracks_data[k].BranchID, sourceConfig_tmp,
            centralTracks_data[k].Age, centralTracks_data[k].ObjectClassID,
            centralTracks_data[k].ObjectClassProbabilities,
            centralTracks_data[k].IsConfirmed, centralTracks_data[k].IsCoasted,
            centralTracks_data[k].IsSelfReported, Y, track_pStateCovariance,
            centralTracks_data[k].pUpdateTime, &expl_temp);
        if (k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, loop_ub, &xh_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        for (b_i = 0; b_i < 6; b_i++) {
          transformedStates_data[b_i + 6 * k] = expl_temp.pState[b_i];
        }
        if (k + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, loop_ub, &wh_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        for (b_i = 0; b_i < 36; b_i++) {
          transformedCovars_data[b_i + k * 36] =
              expl_temp.pStateCovariance[b_i];
        }
      }
      ibtile = allStates->size[0] * allStates->size[1];
      allStates->size[0] = 6;
      c_loop_ub = transformedStates->size[1] + 1;
      allStates->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(sp, allStates, ibtile, &ch_emlrtRTEI);
      S_data = allStates->data;
      for (b_i = 0; b_i < 6; b_i++) {
        S_data[b_i] = localTracks_data[d_i].State[b_i];
      }
      for (b_i = 0; b_i < loop_ub; b_i++) {
        for (k = 0; k < 6; k++) {
          S_data[k + 6 * (b_i + 1)] = transformedStates_data[k + 6 * b_i];
        }
      }
      st.site = &gk_emlrtRSI;
      ibtile = allCovars->size[0] * allCovars->size[1] * allCovars->size[2];
      allCovars->size[0] = 6;
      allCovars->size[1] = 6;
      allCovars->size[2] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&st, allCovars, ibtile, &dh_emlrtRTEI);
      transformedStates_data = allCovars->data;
      for (b_i = 0; b_i < 36; b_i++) {
        transformedStates_data[b_i] =
            localTracks_data[d_i].StateCovariance[b_i];
      }
      iacol = 36 * transformedCovars->size[2];
      b_st.site = &rk_emlrtRSI;
      if (iacol > 2147483646) {
        d_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      for (b_i = 0; b_i < iacol; b_i++) {
        transformedStates_data[b_i + 36] = transformedCovars_data[b_i];
      }
      st.site = &hk_emlrtRSI;
      ibtile = oneColumnCost->size[0] * oneColumnCost->size[1];
      oneColumnCost->size[0] = 1;
      oneColumnCost->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&st, oneColumnCost, ibtile, &eh_emlrtRTEI);
      transformedCovars_data = oneColumnCost->data;
      for (b_i = 0; b_i < c_loop_ub; b_i++) {
        transformedCovars_data[b_i] = 0.0;
      }
      b_st.site = &sk_emlrtRSI;
      ibtile = e->size[0] * e->size[1];
      e->size[0] = 6;
      e->size[1] = transformedStates->size[1] + 1;
      emxEnsureCapacity_real_T(&b_st, e, ibtile, &fh_emlrtRTEI);
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
      emxEnsureCapacity_real_T(&b_st, S, ibtile, &fh_emlrtRTEI);
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
      emxEnsureCapacity_real_T(&b_st, r, ibtile, &gh_emlrtRTEI);
      transformedStates_data = r->data;
      for (k = 0; k < c_loop_ub; k++) {
        if (k + 1 > e->size[1]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, e->size[1], &di_emlrtBCI,
                                        &b_st);
        }
        d_st.site = &tk_emlrtRSI;
        if (k + 1 > S->size[2]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, S->size[2], &ci_emlrtBCI,
                                        &d_st);
        }
        for (b_i = 0; b_i < 6; b_i++) {
          Y[b_i] = e_data[b_i + 6 * k];
        }
        e_st.site = &uk_emlrtRSI;
        mrdiv(&e_st, Y, &S_data[36 * k]);
        d_st.site = &tk_emlrtRSI;
        if (k + 1 > S->size[2]) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, S->size[2], &bi_emlrtBCI,
                                        &d_st);
        }
        e_st.site = &kl_emlrtRSI;
        for (b_i = 0; b_i < 36; b_i++) {
          track_pStateCovariance[b_i] = S_data[b_i + k * 36];
        }
        f_st.site = &cl_emlrtRSI;
        xzgetrf(&f_st, track_pStateCovariance, ipiv);
        y = track_pStateCovariance[0];
        isodd = false;
        for (b_i = 0; b_i < 5; b_i++) {
          y *= track_pStateCovariance[(b_i + 6 * (b_i + 1)) + 1];
          if (ipiv[b_i] > b_i + 1) {
            isodd = !isodd;
          }
        }
        if (isodd) {
          y = -y;
        }
        d_st.site = &tk_emlrtRSI;
        if (y < 0.0) {
          emlrtErrorWithMessageIdR2018a(
              &d_st, &sb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
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
                                      &hi_emlrtBCI, (emlrtConstCTX)sp);
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
          sourceConfig_tmp = trackClasses_data[b_i];
          if ((sourceConfig_tmp != localTracks_data[d_i].ObjectClassID) &&
              (sourceConfig_tmp != 0U)) {
            iacol++;
          }
        }
        ibtile = r1->size[0] * r1->size[1];
        r1->size[0] = 1;
        r1->size[1] = iacol;
        emxEnsureCapacity_int32_T(sp, r1, ibtile, &hh_emlrtRTEI);
        r4 = r1->data;
        ibtile = 0;
        for (b_i = 0; b_i < ibmat; b_i++) {
          sourceConfig_tmp = trackClasses_data[b_i];
          if ((sourceConfig_tmp != localTracks_data[d_i].ObjectClassID) &&
              (sourceConfig_tmp != 0U)) {
            r4[ibtile] = b_i;
            ibtile++;
          }
        }
        if (d_i + 1 > costMatrix->size[1]) {
          emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, costMatrix->size[1],
                                        &gi_emlrtBCI, (emlrtConstCTX)sp);
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
          emlrtDynamicBoundsCheckR2012b(b_i, 0, iacol - 1, &oi_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        costMatrix_data[b_i] = rtInf;
      }
    }
  }
  emxFree_int32_T(sp, &r1);
  emxFree_real_T(sp, &S);
  emxFree_real_T(sp, &e);
  emxFree_real_T(sp, &r);
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
    emlrtErrorWithMessageIdR2018a(sp, &r_emlrtRTEI,
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
          sp, &s_emlrtRTEI, "fusion:trackFuser:UnknownConfig",
          "fusion:trackFuser:UnknownConfig", 6, 4, 11, "SourceIndex", 4, 20,
          "SourceConfigurations");
    }
  } while (exitg2 == 0);
  return config;
}

void FuserManager_setupImpl(const emlrtStack *sp, trackFuser *obj,
                            const emxArray_struct0_T *tracks)
{
  static const char_T b_cv[10] = {'I', 'n', 't', 'e', 'g',
                                  'r', 'a', 't', 'e', 'd'};
  b_objectTrack expl_temp;
  b_objectTrack jw_emlrtRSI;
  b_objectTrack kw_emlrtRSI;
  b_objectTrack lw_emlrtRSI;
  b_objectTrack track;
  c_objectTrack b[100];
  c_objectTrack b_expl_temp;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  fuserSourceConfiguration *sourceConfig;
  const struct0_T *tracks_data;
  real_T imvec[6];
  real_T b_dv[3];
  real_T varargin_1[3];
  real_T a;
  real_T ct_pUpdateTime;
  int32_T b_i;
  int32_T i;
  int32_T kstr;
  uint32_T ct_Age;
  uint32_T ct_BranchID;
  uint32_T ct_SourceIndex;
  uint32_T ct_TrackID;
  boolean_T b_bool;
  boolean_T ct_IsCoasted;
  boolean_T ct_IsSelfReported;
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
  tracks_data = tracks->data;
  st.site = &bc_emlrtRSI;
  b_st.site = &gc_emlrtRSI;
  track.ObjectClassProbabilities.size[0] = 1;
  track.ObjectClassProbabilities.size[1] = 0;
  c_st.site = &hc_emlrtRSI;
  track.TrackID = tracks_data[0].TrackID;
  track.BranchID = tracks_data[0].BranchID;
  track.SourceIndex = tracks_data[0].SourceIndex;
  track.Age = tracks_data[0].Age;
  track.IsConfirmed = tracks_data[0].IsConfirmed;
  track.IsCoasted = tracks_data[0].IsCoasted;
  track.IsSelfReported = tracks_data[0].IsSelfReported;
  d_st.site = &ic_emlrtRSI;
  e_st.site = &qc_emlrtRSI;
  a = tracks_data[0].UpdateTime;
  f_st.site = &gb_emlrtRSI;
  if (a < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &f_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
  }
  f_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(a) || muDoubleScalarIsNaN(a)) {
    emlrtErrorWithMessageIdR2018a(
        &f_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
  }
  track.pUpdateTime = tracks_data[0].UpdateTime;
  d_st.site = &jc_emlrtRSI;
  e_st.site = &rc_emlrtRSI;
  validateattributes(&e_st, tracks_data[0].State);
  for (i = 0; i < 6; i++) {
    track.pState[i] = tracks_data[0].State[i];
  }
  d_st.site = &kc_emlrtRSI;
  e_st.site = &sc_emlrtRSI;
  b_validateattributes(&e_st, tracks_data[0].StateCovariance);
  e_st.site = &tc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&e_st, tracks_data[0].StateCovariance);
  for (i = 0; i < 36; i++) {
    track.pStateCovariance[i] = tracks_data[0].StateCovariance[i];
  }
  d_st.site = &lc_emlrtRSI;
  c_objectTrack_set_ObjectClassPr(&d_st, &track,
                                  tracks_data[0].ObjectClassProbabilities.data,
                                  tracks_data[0].ObjectClassProbabilities.size);
  d_st.site = &mc_emlrtRSI;
  e_st.site = &ig_emlrtRSI;
  c_validateattributes(&e_st, tracks_data[0].ObjectClassID);
  d_st.site = &nc_emlrtRSI;
  e_st.site = &jg_emlrtRSI;
  f_st.site = &kg_emlrtRSI;
  b_bool = false;
  kstr = 0;
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (kstr < 10) {
      uint8_T u;
      g_st.site = &lg_emlrtRSI;
      u = (uint8_T)tracks_data[0].TrackLogic[kstr];
      if (u > 127) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &i_emlrtRTEI, "Coder:toolbox:unsupportedString",
            "Coder:toolbox:unsupportedString", 2, 12, 127);
      }
      if (cv[u] != cv[(int32_T)b_cv[kstr]]) {
        exitg1 = 1;
      } else {
        kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  if (b_bool) {
    int32_T x_size_idx_1;
    boolean_T x_data[10];
    boolean_T exitg2;
    d_st.site = &oc_emlrtRSI;
    kstr = tracks_data[0].TrackLogicState.size[1];
    x_size_idx_1 = tracks_data[0].TrackLogicState.size[1];
    for (i = 0; i < kstr; i++) {
      x_data[i] = (tracks_data[0].TrackLogicState.data[i] == -1.0);
    }
    b_bool = (x_size_idx_1 != 0);
    if (b_bool) {
      e_st.site = &qg_emlrtRSI;
      f_st.site = &rg_emlrtRSI;
      kstr = 0;
      exitg2 = false;
      while ((!exitg2) && (kstr <= x_size_idx_1 - 1)) {
        if (!x_data[kstr]) {
          b_bool = false;
          exitg2 = true;
        } else {
          kstr++;
        }
      }
    }
    if (!b_bool) {
      d_st.site = &pc_emlrtRSI;
      e_st.site = &sg_emlrtRSI;
      f_st.site = &tg_emlrtRSI;
      g_st.site = &gb_emlrtRSI;
      b_bool = true;
      kstr = 0;
      exitg2 = false;
      while ((!exitg2) &&
             (kstr <= tracks_data[0].TrackLogicState.size[1] - 1)) {
        if ((!muDoubleScalarIsInf(tracks_data[0].TrackLogicState.data[kstr])) &&
            (!muDoubleScalarIsNaN(tracks_data[0].TrackLogicState.data[kstr]))) {
          kstr++;
        } else {
          b_bool = false;
          exitg2 = true;
        }
      }
      if (!b_bool) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &m_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedFinite",
            "MATLAB:objectTrack:expectedFinite", 3, 4, 15, "TrackLogicState");
      }
      g_st.site = &gb_emlrtRSI;
      b_bool = true;
      kstr = 0;
      exitg2 = false;
      while ((!exitg2) &&
             (kstr <= tracks_data[0].TrackLogicState.size[1] - 1)) {
        if (!(tracks_data[0].TrackLogicState.data[kstr] < 0.0)) {
          kstr++;
        } else {
          b_bool = false;
          exitg2 = true;
        }
      }
      if (!b_bool) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &l_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedNonnegative",
            "MATLAB:objectTrack:expectedNonnegative", 3, 4, 15,
            "TrackLogicState");
      }
      g_st.site = &gb_emlrtRSI;
      if (tracks_data[0].TrackLogicState.size[1] != 1) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &k_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedScalar",
            "MATLAB:objectTrack:expectedScalar", 3, 4, 15, "TrackLogicState");
      }
      g_st.site = &gb_emlrtRSI;
      b_bool = true;
      kstr = 0;
      exitg2 = false;
      while ((!exitg2) &&
             (kstr <= tracks_data[0].TrackLogicState.size[1] - 1)) {
        if (tracks_data[0].TrackLogicState.data[kstr] <= 1.0) {
          kstr++;
        } else {
          b_bool = false;
          exitg2 = true;
        }
      }
      if (!b_bool) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &j_emlrtRTEI, "MATLAB:validateattributes:expectedScalar",
            "MATLAB:objectTrack:notLessEqual", 9, 4, 15, "TrackLogicState", 4,
            2, "<=", 4, 1, "1");
      }
    }
  } else {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &n_emlrtRTEI,
        "shared_tracking:objectTrack:invalidTrackLogicType",
        "shared_tracking:objectTrack:invalidTrackLogicType", 3, 4, 10,
        "TrackLogic");
  }
  st.site = &cc_emlrtRSI;
  b_st.site = &ug_emlrtRSI;
  sourceConfig = FuserManager_getConfigByID(&b_st, obj, track.SourceIndex);
  st.site = &dc_emlrtRSI;
  if (!sourceConfig->pIsTransformToCentralValid) {
    b_st.site = &vg_emlrtRSI;
    c_st.site = &xg_emlrtRSI;
    /*  HELPER FUNCTIONS */
    /*  ---- Wrappers (homogeneous handles) ---- */
    /*  Pre-define output - force 'Integrated' for codegen consistency */
    ct_TrackID = b_objectTrack_objectTrack(
        &ct_BranchID, &ct_SourceIndex, &ct_Age, &a,
        expl_temp.ObjectClassProbabilities.data,
        expl_temp.ObjectClassProbabilities.size, &b_bool, &ct_IsCoasted,
        &ct_IsSelfReported, expl_temp.pState, expl_temp.pStateCovariance,
        &ct_pUpdateTime);
    if (track.SourceIndex == 1U) {
      /*  Radar */
      d_st.site = &ah_emlrtRSI;
      Ned2ecefTrack(&d_st, track.TrackID, track.BranchID, 1U, track.Age,
                    tracks_data[0].ObjectClassID,
                    track.ObjectClassProbabilities.data,
                    track.ObjectClassProbabilities.size, track.IsConfirmed,
                    track.IsCoasted, track.IsSelfReported, track.pState,
                    track.pStateCovariance, track.pUpdateTime, &expl_temp);
      ct_TrackID = expl_temp.TrackID;
      ct_BranchID = expl_temp.BranchID;
      ct_SourceIndex = expl_temp.SourceIndex;
      ct_Age = expl_temp.Age;
      a = expl_temp.ObjectClassID;
      b_bool = expl_temp.IsConfirmed;
      ct_IsCoasted = expl_temp.IsCoasted;
      ct_IsSelfReported = expl_temp.IsSelfReported;
      ct_pUpdateTime = expl_temp.pUpdateTime;
      /* radar2central(localTrack); %Ned2ecefTrack(localTrack); % UPDATED */
    } else if (track.SourceIndex == 2U) {
      /*  ADSB */
      d_st.site = &bh_emlrtRSI;
      Ecef2nedTrack(&d_st, track.TrackID, track.BranchID, 2U, track.Age,
                    tracks_data[0].ObjectClassID,
                    track.ObjectClassProbabilities.data,
                    track.ObjectClassProbabilities.size, track.IsConfirmed,
                    track.IsCoasted, track.IsSelfReported, track.pState,
                    track.pStateCovariance, track.pUpdateTime, &expl_temp);
      ct_TrackID = expl_temp.TrackID;
      ct_BranchID = expl_temp.BranchID;
      ct_SourceIndex = expl_temp.SourceIndex;
      ct_Age = expl_temp.Age;
      a = expl_temp.ObjectClassID;
      b_bool = expl_temp.IsConfirmed;
      ct_IsCoasted = expl_temp.IsCoasted;
      ct_IsSelfReported = expl_temp.IsSelfReported;
      ct_pUpdateTime = expl_temp.pUpdateTime;
      /* adsb2central(localTrack); % UPDATED */
    }
    c_st.site = &yg_emlrtRSI;
    /*  Pre-define output with longest TrackLogic to force codegen consistency
     */
    if (ct_SourceIndex == 1U) {
      /*  Radar */
      d_st.site = &fi_emlrtRSI;
      Ecef2nedTrack(&d_st, ct_TrackID, ct_BranchID, 1U, ct_Age, a,
                    expl_temp.ObjectClassProbabilities.data,
                    expl_temp.ObjectClassProbabilities.size, b_bool,
                    ct_IsCoasted, ct_IsSelfReported, expl_temp.pState,
                    expl_temp.pStateCovariance, ct_pUpdateTime, &jw_emlrtRSI);
      /*  central2radar(centralTrack); %Ecef2nedTrack(centralTrack); % UPDATED
       */
    } else if (ct_SourceIndex == 2U) {
      /*  ADSB */
      d_st.site = &gi_emlrtRSI;
      Ned2ecefTrack(&d_st, ct_TrackID, ct_BranchID, 2U, ct_Age, a,
                    expl_temp.ObjectClassProbabilities.data,
                    expl_temp.ObjectClassProbabilities.size, b_bool,
                    ct_IsCoasted, ct_IsSelfReported, expl_temp.pState,
                    expl_temp.pStateCovariance, ct_pUpdateTime, &jw_emlrtRSI);
      /* central2adsb(centralTrack); % UPDATED */
    }
    sourceConfig->pIsTransformToCentralValid = true;
  }
  b_st.site = &wg_emlrtRSI;
  /*  HELPER FUNCTIONS */
  /*  ---- Wrappers (homogeneous handles) ---- */
  /*  Pre-define output - force 'Integrated' for codegen consistency */
  b_objectTrack_objectTrack(&ct_BranchID, &ct_SourceIndex, &ct_Age, &a,
                            expl_temp.ObjectClassProbabilities.data,
                            expl_temp.ObjectClassProbabilities.size, &b_bool,
                            &ct_IsCoasted, &ct_IsSelfReported, expl_temp.pState,
                            expl_temp.pStateCovariance, &ct_pUpdateTime);
  if (track.SourceIndex == 1U) {
    /*  Radar */
    c_st.site = &ah_emlrtRSI;
    Ned2ecefTrack(&c_st, track.TrackID, track.BranchID, 1U, track.Age,
                  tracks_data[0].ObjectClassID,
                  track.ObjectClassProbabilities.data,
                  track.ObjectClassProbabilities.size, track.IsConfirmed,
                  track.IsCoasted, track.IsSelfReported, track.pState,
                  track.pStateCovariance, track.pUpdateTime, &expl_temp);
    /* radar2central(localTrack); %Ned2ecefTrack(localTrack); % UPDATED */
  } else if (track.SourceIndex == 2U) {
    /*  ADSB */
    c_st.site = &bh_emlrtRSI;
    Ecef2nedTrack(&c_st, track.TrackID, track.BranchID, 2U, track.Age,
                  tracks_data[0].ObjectClassID,
                  track.ObjectClassProbabilities.data,
                  track.ObjectClassProbabilities.size, track.IsConfirmed,
                  track.IsCoasted, track.IsSelfReported, track.pState,
                  track.pStateCovariance, track.pUpdateTime, &expl_temp);
    /* adsb2central(localTrack); % UPDATED */
  }
  st.site = &ec_emlrtRSI;
  b_st.site = &hi_emlrtRSI;
  c_st.site = &ki_emlrtRSI;
  d_validateattributes(&c_st, expl_temp.pState);
  varargin_1[0] = 0.0;
  varargin_1[1] = 0.0;
  varargin_1[2] = 0.0;
  b_st.site = &ii_emlrtRSI;
  c_st.site = &li_emlrtRSI;
  d_st.site = &ki_emlrtRSI;
  d_validateattributes(&d_st, expl_temp.pState);
  b_dv[0] = 0.0;
  b_dv[1] = 0.0;
  b_dv[2] = 0.0;
  d_st.site = &ni_emlrtRSI;
  e_validateattributes(&d_st, b_dv);
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      imvec[b_i] = expl_temp.pState[b_i];
    }
    a = expl_temp.pState[i];
    imvec[i] =
        a + muDoubleScalarMax(1.4901161193847656E-8,
                              1.4901161193847656E-8 * muDoubleScalarAbs(a));
    c_st.site = &mi_emlrtRSI;
    constvel(&c_st, imvec, varargin_1, 1.0);
  }
  b_st.site = &ji_emlrtRSI;
  c_st.site = &li_emlrtRSI;
  d_st.site = &ki_emlrtRSI;
  d_validateattributes(&d_st, expl_temp.pState);
  d_st.site = &ni_emlrtRSI;
  e_validateattributes(&d_st, varargin_1);
  for (i = 0; i < 3; i++) {
    varargin_1[0] = 0.0;
    varargin_1[1] = 0.0;
    varargin_1[2] = 0.0;
    varargin_1[i] = 1.4901161193847656E-8;
    for (b_i = 0; b_i < 6; b_i++) {
      imvec[b_i] = expl_temp.pState[b_i];
    }
    c_st.site = &mi_emlrtRSI;
    constvel(&c_st, imvec, varargin_1, 1.0);
  }
  st.site = &fc_emlrtRSI;
  b_st.site = &qi_emlrtRSI;
  sourceConfig = FuserManager_getConfigByID(&b_st, obj, track.SourceIndex);
  b_st.site = &ri_emlrtRSI;
  if (!sourceConfig->pIsTransformToCentralValid) {
    c_st.site = &vg_emlrtRSI;
    d_st.site = &xg_emlrtRSI;
    /*  HELPER FUNCTIONS */
    /*  ---- Wrappers (homogeneous handles) ---- */
    /*  Pre-define output - force 'Integrated' for codegen consistency */
    ct_TrackID = b_objectTrack_objectTrack(
        &ct_BranchID, &ct_SourceIndex, &ct_Age, &a,
        expl_temp.ObjectClassProbabilities.data,
        expl_temp.ObjectClassProbabilities.size, &b_bool, &ct_IsCoasted,
        &ct_IsSelfReported, expl_temp.pState, expl_temp.pStateCovariance,
        &ct_pUpdateTime);
    if (track.SourceIndex == 1U) {
      /*  Radar */
      e_st.site = &ah_emlrtRSI;
      Ned2ecefTrack(&e_st, track.TrackID, track.BranchID, 1U, track.Age,
                    tracks_data[0].ObjectClassID,
                    track.ObjectClassProbabilities.data,
                    track.ObjectClassProbabilities.size, track.IsConfirmed,
                    track.IsCoasted, track.IsSelfReported, track.pState,
                    track.pStateCovariance, track.pUpdateTime, &expl_temp);
      ct_TrackID = expl_temp.TrackID;
      ct_BranchID = expl_temp.BranchID;
      ct_SourceIndex = expl_temp.SourceIndex;
      ct_Age = expl_temp.Age;
      a = expl_temp.ObjectClassID;
      b_bool = expl_temp.IsConfirmed;
      ct_IsCoasted = expl_temp.IsCoasted;
      ct_IsSelfReported = expl_temp.IsSelfReported;
      ct_pUpdateTime = expl_temp.pUpdateTime;
      /* radar2central(localTrack); %Ned2ecefTrack(localTrack); % UPDATED */
    } else if (track.SourceIndex == 2U) {
      /*  ADSB */
      e_st.site = &bh_emlrtRSI;
      Ecef2nedTrack(&e_st, track.TrackID, track.BranchID, 2U, track.Age,
                    tracks_data[0].ObjectClassID,
                    track.ObjectClassProbabilities.data,
                    track.ObjectClassProbabilities.size, track.IsConfirmed,
                    track.IsCoasted, track.IsSelfReported, track.pState,
                    track.pStateCovariance, track.pUpdateTime, &expl_temp);
      ct_TrackID = expl_temp.TrackID;
      ct_BranchID = expl_temp.BranchID;
      ct_SourceIndex = expl_temp.SourceIndex;
      ct_Age = expl_temp.Age;
      a = expl_temp.ObjectClassID;
      b_bool = expl_temp.IsConfirmed;
      ct_IsCoasted = expl_temp.IsCoasted;
      ct_IsSelfReported = expl_temp.IsSelfReported;
      ct_pUpdateTime = expl_temp.pUpdateTime;
      /* adsb2central(localTrack); % UPDATED */
    }
    d_st.site = &yg_emlrtRSI;
    /*  Pre-define output with longest TrackLogic to force codegen consistency
     */
    if (ct_SourceIndex == 1U) {
      /*  Radar */
      e_st.site = &fi_emlrtRSI;
      Ecef2nedTrack(&e_st, ct_TrackID, ct_BranchID, 1U, ct_Age, a,
                    expl_temp.ObjectClassProbabilities.data,
                    expl_temp.ObjectClassProbabilities.size, b_bool,
                    ct_IsCoasted, ct_IsSelfReported, expl_temp.pState,
                    expl_temp.pStateCovariance, ct_pUpdateTime, &lw_emlrtRSI);
      /*  central2radar(centralTrack); %Ecef2nedTrack(centralTrack); % UPDATED
       */
    } else if (ct_SourceIndex == 2U) {
      /*  ADSB */
      e_st.site = &gi_emlrtRSI;
      Ned2ecefTrack(&e_st, ct_TrackID, ct_BranchID, 2U, ct_Age, a,
                    expl_temp.ObjectClassProbabilities.data,
                    expl_temp.ObjectClassProbabilities.size, b_bool,
                    ct_IsCoasted, ct_IsSelfReported, expl_temp.pState,
                    expl_temp.pStateCovariance, ct_pUpdateTime, &lw_emlrtRSI);
      /* central2adsb(centralTrack); % UPDATED */
    }
    sourceConfig->pIsTransformToCentralValid = true;
  }
  c_st.site = &wg_emlrtRSI;
  /*  HELPER FUNCTIONS */
  /*  ---- Wrappers (homogeneous handles) ---- */
  /*  Pre-define output - force 'Integrated' for codegen consistency */
  if (track.SourceIndex == 1U) {
    /*  Radar */
    d_st.site = &ah_emlrtRSI;
    Ned2ecefTrack(&d_st, track.TrackID, track.BranchID, 1U, track.Age,
                  tracks_data[0].ObjectClassID,
                  track.ObjectClassProbabilities.data,
                  track.ObjectClassProbabilities.size, track.IsConfirmed,
                  track.IsCoasted, track.IsSelfReported, track.pState,
                  track.pStateCovariance, track.pUpdateTime, &kw_emlrtRSI);
    /* radar2central(localTrack); %Ned2ecefTrack(localTrack); % UPDATED */
  } else if (track.SourceIndex == 2U) {
    /*  ADSB */
    d_st.site = &bh_emlrtRSI;
    Ecef2nedTrack(&d_st, track.TrackID, track.BranchID, 2U, track.Age,
                  tracks_data[0].ObjectClassID,
                  track.ObjectClassProbabilities.data,
                  track.ObjectClassProbabilities.size, track.IsConfirmed,
                  track.IsCoasted, track.IsSelfReported, track.pState,
                  track.pStateCovariance, track.pUpdateTime, &kw_emlrtRSI);
    /* adsb2central(localTrack); % UPDATED */
  }
  b_expl_temp.pTrackLogicState[0] = false;
  b_expl_temp.pTrackLogicState[1] = false;
  b_expl_temp.pTrackLogicState[2] = false;
  b_expl_temp.TrackID = MAX_uint32_T;
  b_expl_temp.BranchID = MAX_uint32_T;
  b_expl_temp.SourceIndex = 3U;
  b_expl_temp.Age = MAX_uint32_T;
  b_expl_temp.ObjectClassID = 0.0;
  b_expl_temp.ObjectClassProbabilities = 1.0;
  b_expl_temp.IsConfirmed = false;
  b_expl_temp.IsCoasted = false;
  b_expl_temp.IsSelfReported = false;
  b_expl_temp.pUpdateTime = 0.0;
  for (i = 0; i < 6; i++) {
    b_expl_temp.pState[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    b_expl_temp.pStateCovariance[i] = iv[i];
  }
  for (i = 0; i < 100; i++) {
    b[i] = b_expl_temp;
  }
  for (i = 0; i < 100; i++) {
    obj->pTracksList[i] = b[i];
  }
  kstr = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = 1;
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, obj->pUsedConfigIDs, kstr, &sc_emlrtRTEI);
  obj->pUsedConfigIDs->data[0] = false;
  obj->pUsedConfigIDs->data[1] = false;
  kstr = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = 2;
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, obj->pUsedConfigIDs, kstr, &tc_emlrtRTEI);
  obj->pUsedConfigIDs->data[0] = false;
  obj->pUsedConfigIDs->data[1] = false;
  obj->pUsedConfigIDs->data[2] = false;
  obj->pUsedConfigIDs->data[3] = false;
  kstr = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = 1;
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, obj->pUsedConfigIDs, kstr, &uc_emlrtRTEI);
  obj->pUsedConfigIDs->data[0] = false;
  obj->pUsedConfigIDs->data[1] = false;
}

/* End of code generation (FuserManager.c) */
