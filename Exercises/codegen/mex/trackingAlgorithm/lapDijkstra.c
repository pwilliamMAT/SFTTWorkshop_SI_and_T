/*
 * lapDijkstra.c
 *
 * Code generation for function 'lapDijkstra'
 *
 */

/* Include files */
#include "lapDijkstra.h"
#include "all.h"
#include "any.h"
#include "colon.h"
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "find.h"
#include "ifWhileCond.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"

/* Variable Definitions */
static emlrtRSInfo ygb_emlrtRSI = {
    226,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo ahb_emlrtRSI = {
    202,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo bhb_emlrtRSI = {
    195,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo chb_emlrtRSI = {
    193,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo dhb_emlrtRSI = {
    192,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo ehb_emlrtRSI = {
    146,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo fhb_emlrtRSI = {
    137,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo ghb_emlrtRSI = {
    118,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo hhb_emlrtRSI = {
    100,           /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo ihb_emlrtRSI = {
    90,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo jhb_emlrtRSI = {
    86,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo khb_emlrtRSI = {
    79,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo lhb_emlrtRSI = {
    76,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo mhb_emlrtRSI = {
    75,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo nhb_emlrtRSI = {
    73,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRSInfo ohb_emlrtRSI = {
    69,            /* lineNo */
    "lapDijkstra", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pathName */
};

static emlrtRTEInfo hc_emlrtRTEI = {
    74,            /* lineNo */
    1,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtECInfo hb_emlrtECI = {
    -1,            /* nDims */
    76,            /* lineNo */
    1,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo gh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    83,            /* lineNo */
    23,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtECInfo ib_emlrtECI = {
    2,             /* nDims */
    83,            /* lineNo */
    12,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo hh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    114,           /* lineNo */
    36,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ih_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    114,           /* lineNo */
    39,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtRTEInfo ic_emlrtRTEI = {
    117,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo jc_emlrtRTEI = {
    134,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo jh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    139,           /* lineNo */
    43,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo kh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    143,           /* lineNo */
    29,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo lh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    143,           /* lineNo */
    32,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtECInfo jb_emlrtECI = {
    -1,            /* nDims */
    143,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo kc_emlrtRTEI = {
    145,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo mh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    173,           /* lineNo */
    37,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo nh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    176,           /* lineNo */
    34,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo oh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    176,           /* lineNo */
    37,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ph_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    177,           /* lineNo */
    38,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtRTEInfo lc_emlrtRTEI = {
    201,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo qh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    205,           /* lineNo */
    29,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo rh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    205,           /* lineNo */
    32,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtECInfo kb_emlrtECI = {
    -1,            /* nDims */
    205,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo sh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    213,           /* lineNo */
    34,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo th_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    223,           /* lineNo */
    27,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtECInfo lb_emlrtECI = {
    -1,            /* nDims */
    223,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtECInfo mb_emlrtECI = {
    -1,            /* nDims */
    224,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtBCInfo uh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    138,           /* lineNo */
    21,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo vh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    81,            /* lineNo */
    24,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo wh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    170,           /* lineNo */
    34,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo xh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    214,           /* lineNo */
    43,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo yh_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    174,           /* lineNo */
    38,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ai_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    174,           /* lineNo */
    46,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo bi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    174,           /* lineNo */
    65,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ci_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    115,           /* lineNo */
    37,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo di_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    214,           /* lineNo */
    60,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ei_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    177,           /* lineNo */
    46,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo fi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    214,           /* lineNo */
    13,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo gi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    177,           /* lineNo */
    65,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo hi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    184,           /* lineNo */
    42,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ii_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    219,           /* lineNo */
    39,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ji_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    222,           /* lineNo */
    17,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ki_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    185,           /* lineNo */
    29,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo li_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    185,           /* lineNo */
    49,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo mi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    185,           /* lineNo */
    17,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ni_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    223,           /* lineNo */
    40,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo oi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    186,           /* lineNo */
    32,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo pi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    186,           /* lineNo */
    17,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo qi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    224,           /* lineNo */
    17,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ri_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    127,           /* lineNo */
    30,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo si_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    191,           /* lineNo */
    44,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ti_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    128,           /* lineNo */
    40,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo ui_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    193,           /* lineNo */
    40,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo vi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    129,           /* lineNo */
    43,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo wi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    135,           /* lineNo */
    34,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo xi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    205,           /* lineNo */
    41,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo yi_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    205,           /* lineNo */
    55,            /* colNo */
    "",            /* aName */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m", /* pName */
    0                /* checkKind */
};

static emlrtRTEInfo mj_emlrtRTEI = {
    68,            /* lineNo */
    1,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo nj_emlrtRTEI = {
    83,            /* lineNo */
    5,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo oj_emlrtRTEI = {
    16,                                            /* lineNo */
    13,                                            /* colNo */
    "isinf",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/isinf.m" /* pName */
};

static emlrtRTEInfo pj_emlrtRTEI = {
    90,            /* lineNo */
    9,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo qj_emlrtRTEI = {
    102,           /* lineNo */
    9,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo rj_emlrtRTEI = {
    115,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo sj_emlrtRTEI = {
    214,           /* lineNo */
    34,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo tj_emlrtRTEI = {
    219,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo uj_emlrtRTEI = {
    183,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo vj_emlrtRTEI = {
    220,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo wj_emlrtRTEI = {
    184,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo xj_emlrtRTEI = {
    223,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo yj_emlrtRTEI = {
    185,           /* lineNo */
    29,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo ak_emlrtRTEI = {
    127,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo bk_emlrtRTEI = {
    226,           /* lineNo */
    20,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo ck_emlrtRTEI = {
    128,           /* lineNo */
    40,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo dk_emlrtRTEI = {
    190,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo ek_emlrtRTEI = {
    129,           /* lineNo */
    43,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo fk_emlrtRTEI = {
    191,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo gk_emlrtRTEI = {
    193,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo hk_emlrtRTEI = {
    136,           /* lineNo */
    17,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo ik_emlrtRTEI = {
    202,           /* lineNo */
    33,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo jk_emlrtRTEI = {
    143,           /* lineNo */
    40,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo kk_emlrtRTEI = {
    139,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo lk_emlrtRTEI = {
    205,           /* lineNo */
    45,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo mk_emlrtRTEI = {
    138,           /* lineNo */
    45,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo nk_emlrtRTEI = {
    205,           /* lineNo */
    59,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo ok_emlrtRTEI = {
    205,           /* lineNo */
    40,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo pk_emlrtRTEI = {
    146,           /* lineNo */
    33,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo qk_emlrtRTEI = {
    100,           /* lineNo */
    9,             /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

static emlrtRTEInfo rk_emlrtRTEI = {
    143,           /* lineNo */
    21,            /* colNo */
    "lapDijkstra", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/+assignment/"
    "lapDijkstra.m" /* pName */
};

/* Function Definitions */
void lapDijkstra(const emlrtStack *sp, const emxArray_real_T *costMatrix,
                 emxArray_real_T *rowSoln, emxArray_real_T *colSoln,
                 emxArray_real_T *colRedux)
{
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  emxArray_boolean_T *isMin;
  emxArray_boolean_T *isUnassignedCol;
  emxArray_boolean_T *isnanRowSoln;
  emxArray_int32_T *b_ii;
  emxArray_int32_T *colList;
  emxArray_int32_T *endOfPath;
  emxArray_int32_T *ii;
  emxArray_int32_T *prevRow;
  emxArray_int32_T *r;
  emxArray_int32_T *r1;
  emxArray_int32_T *r2;
  emxArray_int32_T *r3;
  emxArray_int32_T *r4;
  emxArray_int32_T *r5;
  emxArray_int32_T *r6;
  emxArray_real_T *dist;
  emxArray_real_T *distColListK;
  const real_T *costMatrix_data;
  real_T *colRedux_data;
  real_T *colSoln_data;
  real_T *distColListK_data;
  real_T *dist_data;
  real_T *rowSoln_data;
  int32_T b_i;
  int32_T colMin;
  int32_T i;
  int32_T lapDijkstra_numThreads;
  int32_T nCol;
  int32_T nz;
  int32_T thisRow;
  int32_T *b_ii_data;
  int32_T *colList_data;
  int32_T *endOfPath_data;
  int32_T *ii_data;
  int32_T *prevRow_data;
  int32_T *r7;
  int32_T *r8;
  boolean_T *isMin_data;
  boolean_T *isnanRowSoln_data;
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  colRedux_data = colRedux->data;
  colSoln_data = colSoln->data;
  rowSoln_data = rowSoln->data;
  costMatrix_data = costMatrix->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  nCol = costMatrix->size[1];
  emxInit_boolean_T(sp, &isnanRowSoln, 1, &mj_emlrtRTEI, true);
  nz = rowSoln->size[0];
  colMin = isnanRowSoln->size[0];
  isnanRowSoln->size[0] = nz;
  emxEnsureCapacity_boolean_T(sp, isnanRowSoln, colMin, &mj_emlrtRTEI);
  isnanRowSoln_data = isnanRowSoln->data;
  if (nz < 1600) {
    for (i = 0; i < nz; i++) {
      isnanRowSoln_data[i] = muDoubleScalarIsNaN(rowSoln_data[i]);
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    lapDijkstra_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(lapDijkstra_numThreads)

    for (i = 0; i < nz; i++) {
      isnanRowSoln_data[i] = muDoubleScalarIsNaN(rowSoln_data[i]);
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  st.site = &ohb_emlrtRSI;
  if (any(&st, isnanRowSoln)) {
    int32_T b_nz;
    int32_T i1;
    st.site = &nhb_emlrtRSI;
    b_st.site = &gdb_emlrtRSI;
    c_st.site = &ow_emlrtRSI;
    b_nz = c_combineVectorElements(&c_st, isnanRowSoln);
    if (b_nz > costMatrix->size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &hc_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    emxInit_int32_T(sp, &ii, 1, &ie_emlrtRTEI);
    st.site = &mhb_emlrtRSI;
    c_repmat(&st, b_nz, ii);
    colMin = ii->size[0];
    st.site = &lhb_emlrtRSI;
    b_st.site = &ndb_emlrtRSI;
    b_eml_find(&b_st, isnanRowSoln, ii);
    ii_data = ii->data;
    i1 = ii->size[0];
    if (colMin != ii->size[0]) {
      emlrtSubAssignSizeCheck1dR2017a(colMin, ii->size[0], &hb_emlrtECI,
                                      (emlrtConstCTX)sp);
    }
    st.site = &khb_emlrtRSI;
    if (b_nz > 2147483646) {
      b_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    emxInit_real_T(sp, &dist, 2, &nj_emlrtRTEI, true);
    emxInit_int32_T(sp, &prevRow, 2, &pj_emlrtRTEI);
    emxInit_int32_T(sp, &colList, 2, &qk_emlrtRTEI);
    emxInit_int32_T(sp, &endOfPath, 2, &qj_emlrtRTEI);
    emxInit_real_T(sp, &distColListK, 2, &rj_emlrtRTEI, true);
    emxInit_boolean_T(sp, &isMin, 2, &ak_emlrtRTEI, true);
    emxInit_boolean_T(sp, &isUnassignedCol, 2, &hk_emlrtRTEI, true);
    emxInit_int32_T(sp, &r, 2, &rk_emlrtRTEI);
    emxInit_int32_T(sp, &r1, 2, &ck_emlrtRTEI);
    emxInit_int32_T(sp, &r2, 2, &ek_emlrtRTEI);
    emxInit_int32_T(sp, &r3, 2, &yj_emlrtRTEI);
    emxInit_int32_T(sp, &b_ii, 2, &ie_emlrtRTEI);
    emxInit_int32_T(sp, &r4, 2, &lk_emlrtRTEI);
    emxInit_int32_T(sp, &r5, 2, &nk_emlrtRTEI);
    emxInit_int32_T(sp, &r6, 2, &mk_emlrtRTEI);
    for (thisRow = 0; thisRow < b_nz; thisRow++) {
      int32_T b_colRedux;
      int32_T i2;
      int32_T rowFree;
      if ((thisRow + 1 < 1) || (thisRow + 1 > i1)) {
        emlrtDynamicBoundsCheckR2012b(thisRow + 1, 1, i1, &vh_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      rowFree = ii_data[thisRow];
      i2 = costMatrix->size[0];
      if ((ii_data[thisRow] < 1) || (ii_data[thisRow] > costMatrix->size[0])) {
        emlrtDynamicBoundsCheckR2012b(ii_data[thisRow], 1, costMatrix->size[0],
                                      &gh_emlrtBCI, (emlrtConstCTX)sp);
      }
      b_colRedux = colRedux->size[1];
      if ((nCol != b_colRedux) && ((nCol != 1) && (b_colRedux != 1))) {
        emlrtDimSizeImpxCheckR2021b(nCol, b_colRedux, &ib_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      nz = dist->size[0] * dist->size[1];
      dist->size[0] = 1;
      dist->size[1] = nCol;
      emxEnsureCapacity_real_T(sp, dist, nz, &nj_emlrtRTEI);
      dist_data = dist->data;
      for (b_i = 0; b_i < nCol; b_i++) {
        dist_data[b_i] =
            costMatrix_data[(rowFree + costMatrix->size[0] * b_i) - 1] -
            colRedux_data[b_i];
      }
      nz = isUnassignedCol->size[0] * isUnassignedCol->size[1];
      isUnassignedCol->size[0] = 1;
      isUnassignedCol->size[1] = nCol;
      emxEnsureCapacity_boolean_T(sp, isUnassignedCol, nz, &oj_emlrtRTEI);
      isnanRowSoln_data = isUnassignedCol->data;
      for (b_i = 0; b_i < nCol; b_i++) {
        isnanRowSoln_data[b_i] = muDoubleScalarIsInf(dist_data[b_i]);
      }
      st.site = &jhb_emlrtRSI;
      if (!all(&st, isUnassignedCol)) {
        real_T distMin;
        int32_T b_last;
        int32_T c_nz;
        int32_T last;
        int32_T low;
        int32_T up;
        boolean_T guard1;
        boolean_T unassignedFound;
        st.site = &ihb_emlrtRSI;
        b_repmat(&st, nCol, prevRow);
        nz = prevRow->size[0] * prevRow->size[1];
        prevRow->size[0] = 1;
        emxEnsureCapacity_int32_T(sp, prevRow, nz, &pj_emlrtRTEI);
        prevRow_data = prevRow->data;
        nz = prevRow->size[1] - 1;
        for (b_i = 0; b_i <= nz; b_i++) {
          prevRow_data[b_i] *= ii_data[thisRow];
        }
        low = 1;
        up = 1;
        st.site = &hhb_emlrtRSI;
        b_st.site = &jdb_emlrtRSI;
        c_st.site = &kdb_emlrtRSI;
        eml_integer_colon_dispatcher(&c_st, nCol, colList);
        colList_data = colList->data;
        last = 0;
        nz = endOfPath->size[0] * endOfPath->size[1];
        endOfPath->size[0] = 1;
        endOfPath->size[1] = 1;
        emxEnsureCapacity_int32_T(sp, endOfPath, nz, &qj_emlrtRTEI);
        endOfPath_data = endOfPath->data;
        endOfPath_data[0] = 0;
        unassignedFound = false;
        distMin = 0.0;
        guard1 = false;
        int32_T exitg1;
        boolean_T b_guard1;
        do {
          real_T costOld;
          int32_T b_loop_ub;
          int32_T c_loop_ub;
          int32_T loop_ub;
          exitg1 = 0;
          b_guard1 = false;
          if (low == up) {
            last = low - 1;
            if (up > nCol) {
              loop_ub = 0;
              colMin = 0;
            } else {
              if ((up < 1) || (up > colList->size[1])) {
                emlrtDynamicBoundsCheckR2012b(up, 1, colList->size[1],
                                              &hh_emlrtBCI, (emlrtConstCTX)sp);
              }
              loop_ub = up - 1;
              if ((nCol < 1) || (nCol > colList->size[1])) {
                emlrtDynamicBoundsCheckR2012b(nCol, 1, colList->size[1],
                                              &ih_emlrtBCI, (emlrtConstCTX)sp);
              }
              colMin = nCol;
            }
            nz = distColListK->size[0] * distColListK->size[1];
            distColListK->size[0] = 1;
            c_nz = colMin - loop_ub;
            distColListK->size[1] = c_nz;
            emxEnsureCapacity_real_T(sp, distColListK, nz, &rj_emlrtRTEI);
            distColListK_data = distColListK->data;
            for (b_i = 0; b_i < c_nz; b_i++) {
              nz = colList_data[loop_ub + b_i];
              if ((nz < 1) || (nz > dist->size[1])) {
                emlrtDynamicBoundsCheckR2012b(nz, 1, dist->size[1],
                                              &ci_emlrtBCI, (emlrtConstCTX)sp);
              }
              distColListK_data[b_i] = dist_data[nz - 1];
            }
            if (c_nz > nCol) {
              emlrtErrorWithMessageIdR2018a(
                  sp, &ic_emlrtRTEI, "Coder:builtins:AssertionFailed",
                  "Coder:builtins:AssertionFailed", 0);
            }
            if (c_nz < 1) {
              b_last = 0;
            } else {
              b_last = c_nz;
            }
            st.site = &ghb_emlrtRSI;
            b_st.site = &rhb_emlrtRSI;
            c_st.site = &shb_emlrtRSI;
            d_st.site = &thb_emlrtRSI;
            if (b_last < 1) {
              emlrtErrorWithMessageIdR2018a(
                  &d_st, &x_emlrtRTEI,
                  "Coder:toolbox:eml_min_or_max_varDimZero",
                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
            }
            e_st.site = &uhb_emlrtRSI;
            f_st.site = &vhb_emlrtRSI;
            if (b_last <= 2) {
              if (b_last == 1) {
                distMin = distColListK_data[0];
              } else if ((distColListK_data[0] > distColListK_data[1]) ||
                         (muDoubleScalarIsNaN(distColListK_data[0]) &&
                          (!muDoubleScalarIsNaN(distColListK_data[1])))) {
                distMin = distColListK_data[1];
              } else {
                distMin = distColListK_data[0];
              }
            } else {
              g_st.site = &vn_emlrtRSI;
              if (!muDoubleScalarIsNaN(distColListK_data[0])) {
                colMin = 1;
              } else {
                boolean_T exitg2;
                colMin = 0;
                h_st.site = &wn_emlrtRSI;
                if (b_last > 2147483646) {
                  i_st.site = &k_emlrtRSI;
                  check_forloop_overflow_error(&i_st);
                }
                nz = 2;
                exitg2 = false;
                while ((!exitg2) && (nz <= b_last)) {
                  if (!muDoubleScalarIsNaN(distColListK_data[nz - 1])) {
                    colMin = nz;
                    exitg2 = true;
                  } else {
                    nz++;
                  }
                }
              }
              if (colMin == 0) {
                distMin = distColListK_data[0];
              } else {
                g_st.site = &un_emlrtRSI;
                distMin = distColListK_data[colMin - 1];
                nz = colMin + 1;
                h_st.site = &xn_emlrtRSI;
                if ((colMin + 1 <= b_last) && (b_last > 2147483646)) {
                  i_st.site = &k_emlrtRSI;
                  check_forloop_overflow_error(&i_st);
                }
                for (b_i = nz; b_i <= b_last; b_i++) {
                  costOld = distColListK_data[b_i - 1];
                  if (distMin > costOld) {
                    distMin = costOld;
                  }
                }
              }
            }
            if (muDoubleScalarIsInf(distMin)) {
              exitg1 = 1;
            } else {
              colMin = isMin->size[0] * isMin->size[1];
              isMin->size[0] = 1;
              isMin->size[1] = c_nz;
              emxEnsureCapacity_boolean_T(sp, isMin, colMin, &ak_emlrtRTEI);
              isMin_data = isMin->data;
              for (b_i = 0; b_i < c_nz; b_i++) {
                nz = colList_data[loop_ub + b_i];
                if ((nz < 1) || (nz > dist->size[1])) {
                  emlrtDynamicBoundsCheckR2012b(
                      nz, 1, dist->size[1], &ri_emlrtBCI, (emlrtConstCTX)sp);
                }
                isMin_data[b_i] = (dist_data[nz - 1] == distMin);
              }
              b_last = isMin->size[1];
              colMin = 0;
              for (b_i = 0; b_i < b_last; b_i++) {
                if (isMin_data[b_i]) {
                  colMin++;
                }
              }
              nz = r1->size[0] * r1->size[1];
              r1->size[0] = 1;
              r1->size[1] = colMin;
              emxEnsureCapacity_int32_T(sp, r1, nz, &ck_emlrtRTEI);
              r8 = r1->data;
              nz = 0;
              for (b_i = 0; b_i < b_last; b_i++) {
                if (isMin_data[b_i]) {
                  r8[nz] = b_i;
                  nz++;
                }
              }
              b_loop_ub = r1->size[1];
              for (b_i = 0; b_i < b_loop_ub; b_i++) {
                if ((r8[b_i] < 0) || (r8[b_i] > c_nz - 1)) {
                  emlrtDynamicBoundsCheckR2012b(
                      r8[b_i], 0, c_nz - 1, &ti_emlrtBCI, (emlrtConstCTX)sp);
                }
              }
              colMin = 0;
              for (b_i = 0; b_i < b_last; b_i++) {
                if (!isMin_data[b_i]) {
                  colMin++;
                }
              }
              nz = r2->size[0] * r2->size[1];
              r2->size[0] = 1;
              r2->size[1] = colMin;
              emxEnsureCapacity_int32_T(sp, r2, nz, &ek_emlrtRTEI);
              b_ii_data = r2->data;
              nz = 0;
              for (b_i = 0; b_i < b_last; b_i++) {
                if (!isMin_data[b_i]) {
                  b_ii_data[nz] = b_i;
                  nz++;
                }
              }
              c_loop_ub = r2->size[1];
              for (b_i = 0; b_i < c_loop_ub; b_i++) {
                if ((b_ii_data[b_i] < 0) || (b_ii_data[b_i] > c_nz - 1)) {
                  emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 0, c_nz - 1,
                                                &vi_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              if (r1->size[1] > nCol) {
                emlrtErrorWithMessageIdR2018a(
                    sp, &jc_emlrtRTEI, "Coder:builtins:AssertionFailed",
                    "Coder:builtins:AssertionFailed", 0);
              }
              nz = colSoln->size[1];
              for (b_i = 0; b_i < b_loop_ub; b_i++) {
                colMin = colList_data[loop_ub + r8[b_i]];
                if ((colMin < 1) || (colMin > nz)) {
                  emlrtDynamicBoundsCheckR2012b(colMin, 1, nz, &wi_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              if (r1->size[1] < 1) {
                colMin = 0;
              } else {
                colMin = r1->size[1];
              }
              nz = isUnassignedCol->size[0] * isUnassignedCol->size[1];
              isUnassignedCol->size[0] = 1;
              isUnassignedCol->size[1] = colMin;
              emxEnsureCapacity_boolean_T(sp, isUnassignedCol, nz,
                                          &hk_emlrtRTEI);
              isnanRowSoln_data = isUnassignedCol->data;
              for (b_i = 0; b_i < colMin; b_i++) {
                isnanRowSoln_data[b_i] = muDoubleScalarIsNaN(
                    colSoln_data[colList_data[loop_ub + r8[b_i]] - 1]);
              }
              st.site = &fhb_emlrtRSI;
              if (b_any(&st, isUnassignedCol)) {
                b_last = isUnassignedCol->size[1];
                for (b_i = 0; b_i < b_last; b_i++) {
                  if (isnanRowSoln_data[b_i] && (b_i > r1->size[1] - 1)) {
                    emlrtDynamicBoundsCheckR2012b(b_i, 0, r1->size[1] - 1,
                                                  &uh_emlrtBCI,
                                                  (emlrtConstCTX)sp);
                  }
                }
                nz = 0;
                for (b_i = 0; b_i < b_last; b_i++) {
                  if (isnanRowSoln_data[b_i]) {
                    nz++;
                  }
                }
                if (nz < 1) {
                  emlrtDynamicBoundsCheckR2012b(1, 1, nz, &jh_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                colMin = endOfPath->size[0] * endOfPath->size[1];
                endOfPath->size[0] = 1;
                endOfPath->size[1] = 1;
                emxEnsureCapacity_int32_T(sp, endOfPath, colMin, &kk_emlrtRTEI);
                endOfPath_data = endOfPath->data;
                colMin = 0;
                for (b_i = 0; b_i < b_last; b_i++) {
                  if (isnanRowSoln_data[b_i]) {
                    colMin++;
                  }
                }
                nz = r6->size[0] * r6->size[1];
                r6->size[0] = 1;
                r6->size[1] = colMin;
                emxEnsureCapacity_int32_T(sp, r6, nz, &mk_emlrtRTEI);
                b_ii_data = r6->data;
                nz = 0;
                for (b_i = 0; b_i < b_last; b_i++) {
                  if (isnanRowSoln_data[b_i]) {
                    b_ii_data[nz] = b_i;
                    nz++;
                  }
                }
                endOfPath_data[0] = colList_data[loop_ub + r8[b_ii_data[0]]];
                unassignedFound = true;
              } else {
                if (up > nCol) {
                  b_last = 0;
                  colMin = 0;
                } else {
                  if (up > colList->size[1]) {
                    emlrtDynamicBoundsCheckR2012b(up, 1, colList->size[1],
                                                  &kh_emlrtBCI,
                                                  (emlrtConstCTX)sp);
                  }
                  b_last = up - 1;
                  if (nCol > colList->size[1]) {
                    emlrtDynamicBoundsCheckR2012b(nCol, 1, colList->size[1],
                                                  &lh_emlrtBCI,
                                                  (emlrtConstCTX)sp);
                  }
                  colMin = nCol;
                }
                nz = r->size[0] * r->size[1];
                r->size[0] = 1;
                c_nz = r1->size[1] + r2->size[1];
                r->size[1] = c_nz;
                emxEnsureCapacity_int32_T(sp, r, nz, &jk_emlrtRTEI);
                r7 = r->data;
                for (b_i = 0; b_i < b_loop_ub; b_i++) {
                  r7[b_i] = colList_data[loop_ub + r8[b_i]];
                }
                for (b_i = 0; b_i < c_loop_ub; b_i++) {
                  r7[b_i + r1->size[1]] =
                      colList_data[loop_ub + b_ii_data[b_i]];
                }
                nz = colMin - b_last;
                if (nz != c_nz) {
                  emlrtSubAssignSizeCheck1dR2017a(nz, c_nz, &jb_emlrtECI,
                                                  (emlrtConstCTX)sp);
                }
                for (b_i = 0; b_i < c_nz; b_i++) {
                  colList_data[b_last + b_i] = r7[b_i];
                }
                if (isMin->size[1] > nCol) {
                  emlrtErrorWithMessageIdR2018a(
                      sp, &kc_emlrtRTEI, "Coder:builtins:AssertionFailed",
                      "Coder:builtins:AssertionFailed", 0);
                }
                if (isMin->size[1] < 1) {
                  colMin = 0;
                } else {
                  colMin = isMin->size[1];
                }
                st.site = &ehb_emlrtRSI;
                b_st.site = &gdb_emlrtRSI;
                nz = isMin->size[0] * isMin->size[1];
                isMin->size[0] = 1;
                isMin->size[1] = colMin;
                emxEnsureCapacity_boolean_T(&b_st, isMin, nz, &pk_emlrtRTEI);
                c_st.site = &ow_emlrtRSI;
                nz = combineVectorElements(&c_st, isMin);
                up += nz;
              }
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }
          if (b_guard1) {
            if (!unassignedFound) {
              int32_T i3;
              int32_T loop_ub_tmp;
              if ((low < 1) || (low > colList->size[1])) {
                emlrtDynamicBoundsCheckR2012b(low, 1, colList->size[1],
                                              &wh_emlrtBCI, (emlrtConstCTX)sp);
              }
              colMin = colList_data[low - 1];
              low++;
              b_loop_ub = colSoln->size[1];
              if ((colMin < 1) || (colMin > b_loop_ub)) {
                emlrtDynamicBoundsCheckR2012b(colMin, 1, b_loop_ub,
                                              &mh_emlrtBCI, (emlrtConstCTX)sp);
              }
              c_loop_ub = (int32_T)colSoln_data[colMin - 1];
              if ((c_loop_ub < 1) || (c_loop_ub > i2)) {
                emlrtDynamicBoundsCheckR2012b(c_loop_ub, 1, i2, &yh_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              if (colMin > nCol) {
                emlrtDynamicBoundsCheckR2012b(colMin, 1, nCol, &ai_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              if (colMin > b_colRedux) {
                emlrtDynamicBoundsCheckR2012b(colMin, 1, b_colRedux,
                                              &bi_emlrtBCI, (emlrtConstCTX)sp);
              }
              costOld = costMatrix_data[(c_loop_ub +
                                         costMatrix->size[0] * (colMin - 1)) -
                                        1] -
                        colRedux_data[colMin - 1];
              if (up > nCol) {
                i3 = 0;
                nz = -1;
              } else {
                if ((up < 1) || (up > colList->size[1])) {
                  emlrtDynamicBoundsCheckR2012b(
                      up, 1, colList->size[1], &nh_emlrtBCI, (emlrtConstCTX)sp);
                }
                i3 = up - 1;
                if (nCol > colList->size[1]) {
                  emlrtDynamicBoundsCheckR2012b(nCol, 1, colList->size[1],
                                                &oh_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                nz = nCol - 1;
              }
              if (c_loop_ub > i2) {
                emlrtDynamicBoundsCheckR2012b(c_loop_ub, 1, i2, &ph_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              loop_ub_tmp = nz - i3;
              for (b_i = 0; b_i <= loop_ub_tmp; b_i++) {
                nz = colList_data[i3 + b_i];
                if ((nz < 1) || (nz > nCol)) {
                  emlrtDynamicBoundsCheckR2012b(nz, 1, nCol, &ei_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              for (b_i = 0; b_i <= loop_ub_tmp; b_i++) {
                nz = colList_data[i3 + b_i];
                if ((nz < 1) || (nz > b_colRedux)) {
                  emlrtDynamicBoundsCheckR2012b(nz, 1, b_colRedux, &gi_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              loop_ub = loop_ub_tmp + 1;
              nz = distColListK->size[0] * distColListK->size[1];
              distColListK->size[0] = 1;
              distColListK->size[1] = loop_ub_tmp + 1;
              emxEnsureCapacity_real_T(sp, distColListK, nz, &uj_emlrtRTEI);
              distColListK_data = distColListK->data;
              for (b_i = 0; b_i <= loop_ub_tmp; b_i++) {
                nz = colList_data[i3 + b_i] - 1;
                distColListK_data[b_i] =
                    ((costMatrix_data[(c_loop_ub + costMatrix->size[0] * nz) -
                                      1] -
                      colRedux_data[nz]) -
                     costOld) +
                    distMin;
              }
              for (b_i = 0; b_i <= loop_ub_tmp; b_i++) {
                nz = colList_data[i3 + b_i];
                if ((nz < 1) || (nz > dist->size[1])) {
                  emlrtDynamicBoundsCheckR2012b(
                      nz, 1, dist->size[1], &hi_emlrtBCI, (emlrtConstCTX)sp);
                }
              }
              nz = isMin->size[0] * isMin->size[1];
              isMin->size[0] = 1;
              isMin->size[1] = loop_ub_tmp + 1;
              emxEnsureCapacity_boolean_T(sp, isMin, nz, &wj_emlrtRTEI);
              isMin_data = isMin->data;
              for (b_i = 0; b_i < loop_ub; b_i++) {
                isMin_data[b_i] = (distColListK_data[b_i] <
                                   dist_data[colList_data[i3 + b_i] - 1]);
              }
              colMin = 0;
              for (b_i = 0; b_i < loop_ub; b_i++) {
                if (isMin_data[b_i]) {
                  colMin++;
                }
              }
              nz = r3->size[0] * r3->size[1];
              r3->size[0] = 1;
              r3->size[1] = colMin;
              emxEnsureCapacity_int32_T(sp, r3, nz, &yj_emlrtRTEI);
              b_ii_data = r3->data;
              nz = 0;
              for (b_i = 0; b_i < loop_ub; b_i++) {
                if (isMin_data[b_i]) {
                  b_ii_data[nz] = b_i;
                  nz++;
                }
              }
              b_last = r3->size[1];
              for (b_i = 0; b_i < b_last; b_i++) {
                if ((b_ii_data[b_i] < 0) || (b_ii_data[b_i] > loop_ub_tmp)) {
                  emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 0, loop_ub_tmp,
                                                &ki_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              for (b_i = 0; b_i < b_last; b_i++) {
                if ((b_ii_data[b_i] < 0) ||
                    (b_ii_data[b_i] > distColListK->size[1] - 1)) {
                  emlrtDynamicBoundsCheckR2012b(
                      b_ii_data[b_i], 0, distColListK->size[1] - 1,
                      &li_emlrtBCI, (emlrtConstCTX)sp);
                }
              }
              for (b_i = 0; b_i < b_last; b_i++) {
                nz = b_ii_data[b_i];
                colMin = colList_data[i3 + nz];
                if ((colMin < 1) || (colMin > dist->size[1])) {
                  emlrtDynamicBoundsCheckR2012b(colMin, 1, dist->size[1],
                                                &mi_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
                dist_data[colMin - 1] = distColListK_data[nz];
              }
              for (b_i = 0; b_i < b_last; b_i++) {
                if ((b_ii_data[b_i] < 0) || (b_ii_data[b_i] > loop_ub_tmp)) {
                  emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 0, loop_ub_tmp,
                                                &oi_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              for (b_i = 0; b_i < b_last; b_i++) {
                nz = colList_data[i3 + b_ii_data[b_i]];
                if ((nz < 1) || (nz > prevRow->size[1])) {
                  emlrtDynamicBoundsCheckR2012b(
                      nz, 1, prevRow->size[1], &pi_emlrtBCI, (emlrtConstCTX)sp);
                }
                prevRow_data[nz - 1] = c_loop_ub;
              }
              nz = isMin->size[0] * isMin->size[1];
              isMin->size[0] = 1;
              isMin->size[1] = loop_ub_tmp + 1;
              emxEnsureCapacity_boolean_T(sp, isMin, nz, &dk_emlrtRTEI);
              isMin_data = isMin->data;
              nz = distColListK->size[1] - 1;
              for (b_i = 0; b_i <= nz; b_i++) {
                isMin_data[b_i] =
                    ((distColListK_data[b_i] == distMin) && isMin_data[b_i]);
              }
              for (b_i = 0; b_i <= loop_ub_tmp; b_i++) {
                nz = colList_data[i3 + b_i];
                if ((nz < 1) || (nz > b_loop_ub)) {
                  emlrtDynamicBoundsCheckR2012b(nz, 1, b_loop_ub, &si_emlrtBCI,
                                                (emlrtConstCTX)sp);
                }
              }
              nz = isUnassignedCol->size[0] * isUnassignedCol->size[1];
              isUnassignedCol->size[0] = 1;
              isUnassignedCol->size[1] = loop_ub_tmp + 1;
              emxEnsureCapacity_boolean_T(sp, isUnassignedCol, nz,
                                          &fk_emlrtRTEI);
              isnanRowSoln_data = isUnassignedCol->data;
              for (b_i = 0; b_i <= loop_ub_tmp; b_i++) {
                isnanRowSoln_data[b_i] =
                    (muDoubleScalarIsNaN(
                         colSoln_data[colList_data[i3 + b_i] - 1]) &&
                     isMin_data[b_i]);
              }
              st.site = &dhb_emlrtRSI;
              if (b_any(&st, isUnassignedCol)) {
                st.site = &chb_emlrtRSI;
                b_st.site = &ndb_emlrtRSI;
                c_eml_find(&b_st, isUnassignedCol, b_ii);
                b_ii_data = b_ii->data;
                nz = endOfPath->size[0] * endOfPath->size[1];
                endOfPath->size[0] = 1;
                colMin = b_ii->size[1];
                endOfPath->size[1] = b_ii->size[1];
                emxEnsureCapacity_int32_T(sp, endOfPath, nz, &gk_emlrtRTEI);
                endOfPath_data = endOfPath->data;
                for (b_i = 0; b_i < colMin; b_i++) {
                  if ((b_ii_data[b_i] < 1) ||
                      (b_ii_data[b_i] > loop_ub_tmp + 1)) {
                    emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 1,
                                                  loop_ub_tmp + 1, &ui_emlrtBCI,
                                                  (emlrtConstCTX)sp);
                  }
                  endOfPath_data[b_i] = colList_data[(i3 + b_ii_data[b_i]) - 1];
                }
                guard1 = true;
                exitg1 = 1;
              } else {
                st.site = &bhb_emlrtRSI;
                if (b_any(&st, isMin)) {
                  if (isMin->size[1] > nCol) {
                    emlrtErrorWithMessageIdR2018a(
                        sp, &lc_emlrtRTEI, "Coder:builtins:AssertionFailed",
                        "Coder:builtins:AssertionFailed", 0);
                  }
                  if (isMin->size[1] < 1) {
                    colMin = 0;
                  } else {
                    colMin = loop_ub_tmp + 1;
                  }
                  st.site = &ahb_emlrtRSI;
                  b_st.site = &gdb_emlrtRSI;
                  nz = isUnassignedCol->size[0] * isUnassignedCol->size[1];
                  isUnassignedCol->size[0] = 1;
                  isUnassignedCol->size[1] = colMin;
                  emxEnsureCapacity_boolean_T(&b_st, isUnassignedCol, nz,
                                              &ik_emlrtRTEI);
                  isnanRowSoln_data = isUnassignedCol->data;
                  for (b_i = 0; b_i < colMin; b_i++) {
                    isnanRowSoln_data[b_i] = isMin_data[b_i];
                  }
                  c_st.site = &ow_emlrtRSI;
                  c_nz = combineVectorElements(&c_st, isUnassignedCol);
                  if (up > nCol) {
                    c_loop_ub = 0;
                    b_last = 0;
                  } else {
                    if (up > colList->size[1]) {
                      emlrtDynamicBoundsCheckR2012b(up, 1, colList->size[1],
                                                    &qh_emlrtBCI,
                                                    (emlrtConstCTX)sp);
                    }
                    c_loop_ub = up - 1;
                    if (nCol > colList->size[1]) {
                      emlrtDynamicBoundsCheckR2012b(nCol, 1, colList->size[1],
                                                    &rh_emlrtBCI,
                                                    (emlrtConstCTX)sp);
                    }
                    b_last = nCol;
                  }
                  colMin = 0;
                  for (b_i = 0; b_i < loop_ub; b_i++) {
                    if (isMin_data[b_i]) {
                      colMin++;
                    }
                  }
                  nz = r4->size[0] * r4->size[1];
                  r4->size[0] = 1;
                  r4->size[1] = colMin;
                  emxEnsureCapacity_int32_T(sp, r4, nz, &lk_emlrtRTEI);
                  b_ii_data = r4->data;
                  nz = 0;
                  colMin = 0;
                  for (b_i = 0; b_i < loop_ub; b_i++) {
                    if (isMin_data[b_i]) {
                      b_ii_data[nz] = b_i;
                      nz++;
                    } else {
                      colMin++;
                    }
                  }
                  nz = r5->size[0] * r5->size[1];
                  r5->size[0] = 1;
                  r5->size[1] = colMin;
                  emxEnsureCapacity_int32_T(sp, r5, nz, &nk_emlrtRTEI);
                  r8 = r5->data;
                  nz = 0;
                  for (b_i = 0; b_i < loop_ub; b_i++) {
                    if (!isMin_data[b_i]) {
                      r8[nz] = b_i;
                      nz++;
                    }
                  }
                  nz = r->size[0] * r->size[1];
                  r->size[0] = 1;
                  r->size[1] = r4->size[1] + r5->size[1];
                  emxEnsureCapacity_int32_T(sp, r, nz, &ok_emlrtRTEI);
                  r7 = r->data;
                  nz = r4->size[1];
                  for (b_i = 0; b_i < nz; b_i++) {
                    if ((b_ii_data[b_i] < 0) ||
                        (b_ii_data[b_i] > loop_ub_tmp)) {
                      emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 0,
                                                    loop_ub_tmp, &xi_emlrtBCI,
                                                    (emlrtConstCTX)sp);
                    }
                    r7[b_i] = colList_data[i3 + b_ii_data[b_i]];
                  }
                  nz = r5->size[1];
                  for (b_i = 0; b_i < nz; b_i++) {
                    if ((r8[b_i] < 0) || (r8[b_i] > loop_ub_tmp)) {
                      emlrtDynamicBoundsCheckR2012b(r8[b_i], 0, loop_ub_tmp,
                                                    &yi_emlrtBCI,
                                                    (emlrtConstCTX)sp);
                    }
                    r7[b_i + r4->size[1]] = colList_data[i3 + r8[b_i]];
                  }
                  nz = b_last - c_loop_ub;
                  colMin = r->size[1];
                  if (nz != r->size[1]) {
                    emlrtSubAssignSizeCheck1dR2017a(
                        nz, r->size[1], &kb_emlrtECI, (emlrtConstCTX)sp);
                  }
                  for (b_i = 0; b_i < colMin; b_i++) {
                    colList_data[c_loop_ub + b_i] = r7[b_i];
                  }
                  up += c_nz;
                }
                guard1 = false;
              }
            } else {
              guard1 = true;
              exitg1 = 1;
            }
          }
        } while (exitg1 == 0);
        if (guard1) {
          if (last < 1) {
            last = 0;
          } else if (last > colList->size[1]) {
            emlrtDynamicBoundsCheckR2012b(last, 1, colList->size[1],
                                          &sh_emlrtBCI, (emlrtConstCTX)sp);
          }
          for (b_i = 0; b_i < last; b_i++) {
            if ((colList_data[b_i] < 1) || (colList_data[b_i] > b_colRedux)) {
              emlrtDynamicBoundsCheckR2012b(colList_data[b_i], 1, b_colRedux,
                                            &xh_emlrtBCI, (emlrtConstCTX)sp);
            }
          }
          for (b_i = 0; b_i < last; b_i++) {
            if ((colList_data[b_i] < 1) ||
                (colList_data[b_i] > dist->size[1])) {
              emlrtDynamicBoundsCheckR2012b(colList_data[b_i], 1, dist->size[1],
                                            &di_emlrtBCI, (emlrtConstCTX)sp);
            }
          }
          nz = distColListK->size[0] * distColListK->size[1];
          distColListK->size[0] = 1;
          distColListK->size[1] = last;
          emxEnsureCapacity_real_T(sp, distColListK, nz, &sj_emlrtRTEI);
          distColListK_data = distColListK->data;
          for (b_i = 0; b_i < last; b_i++) {
            nz = colList_data[b_i];
            distColListK_data[b_i] =
                (colRedux_data[nz - 1] + dist_data[nz - 1]) - distMin;
          }
          nz = distColListK->size[1];
          for (b_i = 0; b_i < nz; b_i++) {
            if ((colList_data[b_i] < 1) || (colList_data[b_i] > b_colRedux)) {
              emlrtDynamicBoundsCheckR2012b(colList_data[b_i], 1, b_colRedux,
                                            &fi_emlrtBCI, (emlrtConstCTX)sp);
            }
            colRedux_data[colList_data[b_i] - 1] = distColListK_data[b_i];
          }
          c_nz = endOfPath->size[1];
          do {
            nz = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_last = endOfPath->size[1];
            b_ii->size[1] = endOfPath->size[1];
            emxEnsureCapacity_int32_T(sp, b_ii, nz, &tj_emlrtRTEI);
            b_ii_data = b_ii->data;
            for (b_i = 0; b_i < c_nz; b_i++) {
              if ((endOfPath_data[b_i] < 1) ||
                  (endOfPath_data[b_i] > prevRow->size[1])) {
                emlrtDynamicBoundsCheckR2012b(endOfPath_data[b_i], 1,
                                              prevRow->size[1], &ii_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              b_ii_data[b_i] = prevRow_data[endOfPath_data[b_i] - 1];
            }
            nz = colList->size[0] * colList->size[1];
            colList->size[0] = 1;
            colList->size[1] = endOfPath->size[1];
            emxEnsureCapacity_int32_T(sp, colList, nz, &vj_emlrtRTEI);
            colList_data = colList->data;
            for (b_i = 0; b_i < c_nz; b_i++) {
              colList_data[b_i] = endOfPath_data[b_i];
            }
            nz = colSoln->size[1];
            colMin = b_ii->size[1];
            for (b_i = 0; b_i < colMin; b_i++) {
              if ((endOfPath_data[b_i] < 1) || (endOfPath_data[b_i] > nz)) {
                emlrtDynamicBoundsCheckR2012b(endOfPath_data[b_i], 1, nz,
                                              &ji_emlrtBCI, (emlrtConstCTX)sp);
              }
              colSoln_data[endOfPath_data[b_i] - 1] = b_ii_data[b_i];
            }
            nz = r->size[0] * r->size[1];
            r->size[0] = 1;
            r->size[1] = b_ii->size[1];
            emxEnsureCapacity_int32_T(sp, r, nz, &xj_emlrtRTEI);
            r7 = r->data;
            nz = rowSoln->size[0];
            for (b_i = 0; b_i < colMin; b_i++) {
              if ((b_ii_data[b_i] < 1) || (b_ii_data[b_i] > nz)) {
                emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 1, nz,
                                              &ni_emlrtBCI, (emlrtConstCTX)sp);
              }
              r7[b_i] = (int32_T)rowSoln_data[b_ii_data[b_i] - 1];
            }
            if (r->size[1] != 1) {
              emlrtSubAssignSizeCheck1dR2017a(1, r->size[1], &lb_emlrtECI,
                                              (emlrtConstCTX)sp);
            }
            if (endOfPath->size[1] < 1) {
              emlrtDynamicBoundsCheckR2012b(1, 1, endOfPath->size[1],
                                            &th_emlrtBCI, (emlrtConstCTX)sp);
            }
            endOfPath_data[0] = r7[0];
            if (b_ii->size[1] != endOfPath->size[1]) {
              emlrtSubAssignSizeCheck1dR2017a(b_ii->size[1], endOfPath->size[1],
                                              &mb_emlrtECI, (emlrtConstCTX)sp);
            }
            for (b_i = 0; b_i < b_last; b_i++) {
              if ((b_ii_data[b_i] < 1) || (b_ii_data[b_i] > nz)) {
                emlrtDynamicBoundsCheckR2012b(b_ii_data[b_i], 1, nz,
                                              &qi_emlrtBCI, (emlrtConstCTX)sp);
              }
              rowSoln_data[b_ii_data[b_i] - 1] = colList_data[b_i];
            }
            nz = isUnassignedCol->size[0] * isUnassignedCol->size[1];
            isUnassignedCol->size[0] = 1;
            isUnassignedCol->size[1] = b_ii->size[1];
            emxEnsureCapacity_boolean_T(sp, isUnassignedCol, nz, &bk_emlrtRTEI);
            isnanRowSoln_data = isUnassignedCol->data;
            for (b_i = 0; b_i < colMin; b_i++) {
              isnanRowSoln_data[b_i] = (b_ii_data[b_i] == rowFree);
            }
            st.site = &ygb_emlrtRSI;
          } while (!ifWhileCond(&st, isUnassignedCol));
        }
      }
    }
    emxFree_int32_T(sp, &r6);
    emxFree_int32_T(sp, &r5);
    emxFree_int32_T(sp, &r4);
    emxFree_int32_T(sp, &b_ii);
    emxFree_int32_T(sp, &ii);
    emxFree_int32_T(sp, &r3);
    emxFree_int32_T(sp, &r2);
    emxFree_int32_T(sp, &r1);
    emxFree_int32_T(sp, &r);
    emxFree_boolean_T(sp, &isUnassignedCol);
    emxFree_boolean_T(sp, &isMin);
    emxFree_real_T(sp, &distColListK);
    emxFree_int32_T(sp, &endOfPath);
    emxFree_int32_T(sp, &colList);
    emxFree_int32_T(sp, &prevRow);
    emxFree_real_T(sp, &dist);
  }
  emxFree_boolean_T(sp, &isnanRowSoln);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (lapDijkstra.c) */
