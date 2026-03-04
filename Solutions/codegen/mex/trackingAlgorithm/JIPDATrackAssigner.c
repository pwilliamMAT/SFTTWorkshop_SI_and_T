/*
 * JIPDATrackAssigner.c
 *
 * Code generation for function 'JIPDATrackAssigner'
 *
 */

/* Include files */
#include "JIPDATrackAssigner.h"
#include "AerospaceMonostaticRadar.h"
#include "MultiModalEstimator.h"
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
static emlrtRSInfo ip_emlrtRSI = {
    93,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo jp_emlrtRSI = {
    90,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo kp_emlrtRSI = {
    81,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo lp_emlrtRSI = {
    72,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo mp_emlrtRSI = {
    62,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo np_emlrtRSI = {
    61,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo op_emlrtRSI = {
    60,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo pp_emlrtRSI = {
    52,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo qp_emlrtRSI = {
    46,                          /* lineNo */
    "JIPDATrackAssigner/assign", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo rp_emlrtRSI = {
    111,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo sp_emlrtRSI = {
    112,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo tp_emlrtRSI = {
    114,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo up_emlrtRSI = {
    119,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo vp_emlrtRSI = {
    121,                                          /* lineNo */
    "JIPDATrackAssigner/computeLikelihoodMatrix", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo kq_emlrtRSI = {
    132,                                        /* lineNo */
    "JIPDATrackAssigner/singleModelLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo lq_emlrtRSI = {
    143,                                        /* lineNo */
    "JIPDATrackAssigner/singleModelLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo et_emlrtRSI = {
    9,                          /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtRSInfo ft_emlrtRSI = {
    16,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtRSInfo gt_emlrtRSI = {
    19,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtRSInfo ht_emlrtRSI = {
    23,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtRSInfo it_emlrtRSI = {
    25,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtRSInfo pt_emlrtRSI = {
    27,                                        /* lineNo */
    "TrackEstimator/get.SensorSpecifications", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo qt_emlrtRSI = {
    92,                                    /* lineNo */
    "TrackEstimator/likelihoodUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo rt_emlrtRSI = {
    93,                                    /* lineNo */
    "TrackEstimator/likelihoodUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo qx_emlrtRSI = {
    107,                                  /* lineNo */
    "IPDAEstimator/likelihoodUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo rx_emlrtRSI = {
    108,                                  /* lineNo */
    "IPDAEstimator/likelihoodUnassigned", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

static emlrtRSInfo rcb_emlrtRSI = {
    160,                                                    /* lineNo */
    "JIPDATrackAssigner/measurementVaryingModelLikelihood", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo scb_emlrtRSI = {
    12,                         /* lineNo */
    "computeLikelihoodByIndex", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m" /* pathName */
};

static emlrtECInfo b_emlrtECI = {
    -1,                          /* nDims */
    82,                          /* lineNo */
    21,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    -1,                          /* nDims */
    90,                          /* lineNo */
    13,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtBCInfo hb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    90,                          /* lineNo */
    24,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    90,                          /* lineNo */
    60,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtECInfo d_emlrtECI = {
    -1,                          /* nDims */
    76,                          /* lineNo */
    21,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtECInfo e_emlrtECI = {
    -1,                          /* nDims */
    75,                          /* lineNo */
    21,                          /* colNo */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtBCInfo jb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    76,                          /* lineNo */
    62,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    75,                          /* lineNo */
    76,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    17,                         /* lineNo */
    42,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo mb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    19,                         /* lineNo */
    55,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    19,                         /* lineNo */
    71,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo ob_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    23,                         /* lineNo */
    54,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    88,                          /* lineNo */
    13,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo qb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    89,                          /* lineNo */
    13,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    140,                                        /* lineNo */
    37,                                         /* colNo */
    "JIPDATrackAssigner/singleModelLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    1                       /* checkKind */
};

static emlrtBCInfo rb_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    143,                                        /* lineNo */
    28,                                         /* colNo */
    "",                                         /* aName */
    "JIPDATrackAssigner/singleModelLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo sb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    23,                         /* lineNo */
    39,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo tb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    23,                         /* lineNo */
    71,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    64,                          /* lineNo */
    38,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    25,                         /* lineNo */
    49,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    25,                         /* lineNo */
    81,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo xb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    69,                          /* lineNo */
    46,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    69,                          /* lineNo */
    71,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo ac_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    66,                          /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo bc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    75,                          /* lineNo */
    32,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo cc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    75,                          /* lineNo */
    48,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    82,                          /* lineNo */
    36,                          /* colNo */
    "",                          /* aName */
    "JIPDATrackAssigner/assign", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtBCInfo bd_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    12,                         /* lineNo */
    59,                         /* colNo */
    "",                         /* aName */
    "computeLikelihoodByIndex", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+utils/"
    "computeLikelihoodByIndex.m", /* pName */
    0                             /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    154,                                                    /* lineNo */
    37,                                                     /* colNo */
    "JIPDATrackAssigner/measurementVaryingModelLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    1                       /* checkKind */
};

static emlrtBCInfo cd_emlrtBCI = {
    -1,                                                     /* iFirst */
    -1,                                                     /* iLast */
    160,                                                    /* lineNo */
    28,                                                     /* colNo */
    "",                                                     /* aName */
    "JIPDATrackAssigner/measurementVaryingModelLikelihood", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m", /* pName */
    0                       /* checkKind */
};

static emlrtRTEInfo qd_emlrtRTEI = {
    49,                   /* lineNo */
    32,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo rd_emlrtRTEI = {
    55,                   /* lineNo */
    13,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo sd_emlrtRTEI = {
    57,                   /* lineNo */
    13,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo td_emlrtRTEI = {
    61,                   /* lineNo */
    42,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo ud_emlrtRTEI = {
    46,                   /* lineNo */
    13,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo vd_emlrtRTEI = {
    61,                   /* lineNo */
    17,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo wd_emlrtRTEI = {
    90,                   /* lineNo */
    47,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo xd_emlrtRTEI = {
    90,                   /* lineNo */
    39,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo yd_emlrtRTEI = {
    90,                   /* lineNo */
    33,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo ae_emlrtRTEI = {
    64,                   /* lineNo */
    38,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo be_emlrtRTEI = {
    69,                   /* lineNo */
    71,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo ce_emlrtRTEI = {
    69,                   /* lineNo */
    21,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo de_emlrtRTEI = {
    75,                   /* lineNo */
    48,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo ee_emlrtRTEI = {
    75,                   /* lineNo */
    69,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo fe_emlrtRTEI = {
    81,                   /* lineNo */
    45,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo ge_emlrtRTEI = {
    72,                   /* lineNo */
    21,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo he_emlrtRTEI = {
    44,                   /* lineNo */
    73,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

static emlrtRTEInfo me_emlrtRTEI = {
    154,                  /* lineNo */
    26,                   /* colNo */
    "JIPDATrackAssigner", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pName */
};

/* Function Declarations */
static void c_JIPDATrackAssigner_measuremen(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T obj_AssignmentThreshold,
    const c_fusion_tracker_sensorspecs_Ae *c_obj_Estimator_StateEstimator_,
    c_fusion_tracker_internal_estim *d_obj_Estimator_StateEstimator_,
    d_fusion_tracker_internal_estim *e_obj_Estimator_StateEstimator_,
    e_fusion_tracker_internal_estim *f_obj_Estimator_StateEstimator_,
    const c_fusion_tracker_sensorspecs_Ae *g_obj_Estimator_StateEstimator_,
    const emxArray_struct_T *trackList, const real_T z_data[],
    const int32_T z_size[2], const real_T time_data[],
    const int32_T time_size[2], const b_emxArray_struct_T *modelData,
    emxArray_real_T *likelihood);

/* Function Definitions */
static void c_JIPDATrackAssigner_measuremen(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T obj_AssignmentThreshold,
    const c_fusion_tracker_sensorspecs_Ae *c_obj_Estimator_StateEstimator_,
    c_fusion_tracker_internal_estim *d_obj_Estimator_StateEstimator_,
    d_fusion_tracker_internal_estim *e_obj_Estimator_StateEstimator_,
    e_fusion_tracker_internal_estim *f_obj_Estimator_StateEstimator_,
    const c_fusion_tracker_sensorspecs_Ae *g_obj_Estimator_StateEstimator_,
    const emxArray_struct_T *trackList, const real_T z_data[],
    const int32_T z_size[2], const real_T time_data[],
    const int32_T time_size[2], const b_emxArray_struct_T *modelData,
    emxArray_real_T *likelihood)
{
  const b_struct_T *trackList_data;
  b_struct_T pdf;
  c_fusion_tracker_sensorspecs_Ae estimator_SensorSpecifications;
  const c_struct_T *modelData_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  trackingEKF *c_estimator_StateEstimator_Stat;
  trackingEKF *d_estimator_StateEstimator_Stat;
  trackingEKF *e_estimator_StateEstimator_Stat;
  real_T Pg;
  real_T *likelihood_data;
  int32_T b_i;
  int32_T d_tmp;
  int32_T i;
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
  modelData_data = modelData->data;
  trackList_data = trackList->data;
  if ((real_T)trackList->size[0] + 1.0 != trackList->size[0] + 1) {
    emlrtIntegerCheckR2012b((real_T)trackList->size[0] + 1.0, &c_emlrtDCI,
                            (emlrtConstCTX)sp);
  }
  i = (z_size[1] + 1) * (trackList->size[0] + 1);
  d_tmp = likelihood->size[0] * likelihood->size[1];
  likelihood->size[0] = z_size[1] + 1;
  likelihood->size[1] = trackList->size[0] + 1;
  emxEnsureCapacity_real_T(sp, likelihood, d_tmp, &me_emlrtRTEI);
  likelihood_data = likelihood->data;
  for (b_i = 0; b_i < i; b_i++) {
    real_T trkIndex;
    st.site = &rcb_emlrtRSI;
    SD->u1.f2.val = d_obj_Estimator_StateEstimator_->SensorSpecifications[0];
    c_estimator_StateEstimator_Stat =
        d_obj_Estimator_StateEstimator_->TrackingFilter;
    SD->u1.f2.b_val = e_obj_Estimator_StateEstimator_->SensorSpecifications[0];
    d_estimator_StateEstimator_Stat =
        e_obj_Estimator_StateEstimator_->TrackingFilter;
    SD->u1.f2.c_val = f_obj_Estimator_StateEstimator_->SensorSpecifications[0];
    e_estimator_StateEstimator_Stat =
        f_obj_Estimator_StateEstimator_->TrackingFilter;
    b_st.site = &et_emlrtRSI;
    trkIndex = parseIndex(&b_st, trackList, z_size, (real_T)b_i + 1.0, &Pg);
    if (Pg > 0.0) {
      b_st.site = &scb_emlrtRSI;
      if (((int32_T)Pg < 1) || ((int32_T)Pg > modelData->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, modelData->size[0],
                                      &bd_emlrtBCI, &b_st);
      }
      c_st.site = &mq_emlrtRSI;
      d_st.site = &nq_emlrtRSI;
      e_st.site = &pq_emlrtRSI;
      SD->u1.f2.val = d_obj_Estimator_StateEstimator_->SensorSpecifications[0];
      f_st.site = &qq_emlrtRSI;
      c_AerospaceMonostaticRadar_upda(
          &f_st, &SD->u1.f2.val, modelData_data[(int32_T)Pg - 1].LookTime.data,
          modelData_data[(int32_T)Pg - 1].LookTime.size,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.data,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.size,
          modelData_data[(int32_T)Pg - 1].LookElevation.data,
          modelData_data[(int32_T)Pg - 1].LookElevation.size,
          modelData_data[(int32_T)Pg - 1].DetectionTime.data,
          modelData_data[(int32_T)Pg - 1].DetectionTime.size,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.data,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.size,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.data,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.size);
      f_st.site = &qq_emlrtRSI;
      e_st.site = &pq_emlrtRSI;
      SD->u1.f2.b_val =
          e_obj_Estimator_StateEstimator_->SensorSpecifications[0];
      f_st.site = &qq_emlrtRSI;
      c_AerospaceMonostaticRadar_upda(
          &f_st, &SD->u1.f2.b_val,
          modelData_data[(int32_T)Pg - 1].LookTime.data,
          modelData_data[(int32_T)Pg - 1].LookTime.size,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.data,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.size,
          modelData_data[(int32_T)Pg - 1].LookElevation.data,
          modelData_data[(int32_T)Pg - 1].LookElevation.size,
          modelData_data[(int32_T)Pg - 1].DetectionTime.data,
          modelData_data[(int32_T)Pg - 1].DetectionTime.size,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.data,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.size,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.data,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.size);
      f_st.site = &qq_emlrtRSI;
      e_st.site = &pq_emlrtRSI;
      SD->u1.f2.c_val =
          f_obj_Estimator_StateEstimator_->SensorSpecifications[0];
      f_st.site = &qq_emlrtRSI;
      c_AerospaceMonostaticRadar_upda(
          &f_st, &SD->u1.f2.c_val,
          modelData_data[(int32_T)Pg - 1].LookTime.data,
          modelData_data[(int32_T)Pg - 1].LookTime.size,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.data,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.size,
          modelData_data[(int32_T)Pg - 1].LookElevation.data,
          modelData_data[(int32_T)Pg - 1].LookElevation.size,
          modelData_data[(int32_T)Pg - 1].DetectionTime.data,
          modelData_data[(int32_T)Pg - 1].DetectionTime.size,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.data,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.size,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.data,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.size);
      f_st.site = &qq_emlrtRSI;
      d_st.site = &oq_emlrtRSI;
      estimator_SensorSpecifications = *g_obj_Estimator_StateEstimator_;
      e_st.site = &dt_emlrtRSI;
      c_AerospaceMonostaticRadar_upda(
          &e_st, &estimator_SensorSpecifications,
          modelData_data[(int32_T)Pg - 1].LookTime.data,
          modelData_data[(int32_T)Pg - 1].LookTime.size,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.data,
          modelData_data[(int32_T)Pg - 1].LookAzimuth.size,
          modelData_data[(int32_T)Pg - 1].LookElevation.data,
          modelData_data[(int32_T)Pg - 1].LookElevation.size,
          modelData_data[(int32_T)Pg - 1].DetectionTime.data,
          modelData_data[(int32_T)Pg - 1].DetectionTime.size,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.data,
          modelData_data[(int32_T)Pg - 1].AzimuthNoise.size,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.data,
          modelData_data[(int32_T)Pg - 1].ElevationNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeNoise.size,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.data,
          modelData_data[(int32_T)Pg - 1].RangeRateNoise.size);
      c_estimator_StateEstimator_Stat =
          d_obj_Estimator_StateEstimator_->TrackingFilter;
      d_estimator_StateEstimator_Stat =
          e_obj_Estimator_StateEstimator_->TrackingFilter;
      e_estimator_StateEstimator_Stat =
          f_obj_Estimator_StateEstimator_->TrackingFilter;
    }
    if ((trkIndex == 0.0) && (Pg > 0.0)) {
      b_st.site = &ft_emlrtRSI;
      c_st.site = &pt_emlrtRSI;
      if (((int32_T)Pg < 1) || ((int32_T)Pg > z_size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, z_size[1], &lb_emlrtBCI,
                                      &st);
      }
      d_tmp = likelihood->size[0] * likelihood->size[1];
      if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
          ((int32_T)((uint32_T)b_i + 1U) > d_tmp)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, d_tmp,
                                      &cd_emlrtBCI, &st);
      }
      likelihood_data[b_i] =
          c_obj_Estimator_StateEstimator_->ClutterModel.ClutterDensity;
    } else if ((Pg == 0.0) && (trkIndex > 0.0)) {
      if (time_size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, time_size[1], &nb_emlrtBCI, &st);
      }
      b_st.site = &gt_emlrtRSI;
      if (((int32_T)trkIndex < 1) || ((int32_T)trkIndex > trackList->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)trkIndex, 1, trackList->size[0],
                                      &mb_emlrtBCI, &b_st);
      }
      pdf = trackList_data[(int32_T)trkIndex - 1];
      c_st.site = &qt_emlrtRSI;
      TrackEstimator_predict(
          &c_st, &d_obj_Estimator_StateEstimator_->TargetSpecifications[0],
          c_estimator_StateEstimator_Stat,
          &e_obj_Estimator_StateEstimator_->TargetSpecifications[0],
          d_estimator_StateEstimator_Stat,
          &f_obj_Estimator_StateEstimator_->TargetSpecifications[0],
          e_estimator_StateEstimator_Stat, &pdf, time_data[0]);
      c_st.site = &rt_emlrtRSI;
      d_st.site = &qx_emlrtRSI;
      Pg = c_MultiModalEstimator_gateProba(&d_st, obj_AssignmentThreshold);
      d_st.site = &rx_emlrtRSI;
      e_st.site = &rx_emlrtRSI;
      Pg *= c_MultiModalEstimator_detection(
          &e_st, &SD->u1.f2.val, &SD->u1.f2.b_val, &SD->u1.f2.c_val,
          pdf.Hypothesis, pdf.LogWeights, pdf.IsValid);
      e_st.site = &cy_emlrtRSI;
      f_st.site = &dy_emlrtRSI;
      if (!(Pg >= 0.0)) {
        emlrtErrorWithMessageIdR2018a(&f_st, &l_emlrtRTEI,
                                      "MATLAB:validators:mustBeNonnegative",
                                      "MATLAB:validators:mustBeNonnegative", 0);
      }
      f_st.site = &dy_emlrtRSI;
      g_st.site = &il_emlrtRSI;
      if (!(Pg < 1.0)) {
        emlrtErrorWithMessageIdR2018a(
            &g_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
            "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
      }
      d_tmp = likelihood->size[0] * likelihood->size[1];
      if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
          ((int32_T)((uint32_T)b_i + 1U) > d_tmp)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, d_tmp,
                                      &cd_emlrtBCI, &c_st);
      }
      likelihood_data[b_i] = 1.0 - Pg * pdf.ExistenceProbability;
    } else if ((Pg == 0.0) && (trkIndex == 0.0)) {
      d_tmp = likelihood->size[0] * likelihood->size[1];
      if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
          ((int32_T)((uint32_T)b_i + 1U) > d_tmp)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, d_tmp,
                                      &cd_emlrtBCI, &st);
      }
      likelihood_data[b_i] = 1.0;
    } else {
      real_T b_d_tmp;
      real_T d;
      boolean_T b;
      if (((int32_T)Pg < 1) || ((int32_T)Pg > z_size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, z_size[1], &ob_emlrtBCI,
                                      &st);
      }
      if (((int32_T)trkIndex < 1) || ((int32_T)trkIndex > trackList->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)trkIndex, 1, trackList->size[0],
                                      &sb_emlrtBCI, &st);
      }
      b = (((int32_T)Pg < 1) || ((int32_T)Pg > time_size[1]));
      if (b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, time_size[1],
                                      &tb_emlrtBCI, &st);
      }
      b_d_tmp = time_data[(int32_T)Pg - 1];
      d_tmp = 4 * ((int32_T)Pg - 1);
      pdf = trackList_data[(int32_T)trkIndex - 1];
      b_st.site = &ht_emlrtRSI;
      d = TrackEstimator_distance(
          &b_st, &d_obj_Estimator_StateEstimator_->TargetSpecifications[0],
          &SD->u1.f2.val, c_estimator_StateEstimator_Stat,
          &e_obj_Estimator_StateEstimator_->TargetSpecifications[0],
          &SD->u1.f2.b_val, d_estimator_StateEstimator_Stat,
          &f_obj_Estimator_StateEstimator_->TargetSpecifications[0],
          &SD->u1.f2.c_val, e_estimator_StateEstimator_Stat, &pdf,
          &z_data[d_tmp], b_d_tmp);
      if (d < obj_AssignmentThreshold) {
        int32_T i1;
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)trkIndex, 1,
                                        trackList->size[0], &vb_emlrtBCI, &st);
        }
        if (b) {
          emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, time_size[1],
                                        &wb_emlrtBCI, &st);
        }
        i1 = likelihood->size[0] * likelihood->size[1];
        if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
            ((int32_T)((uint32_T)b_i + 1U) > i1)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, i1,
                                        &cd_emlrtBCI, &st);
        }
        pdf = trackList_data[(int32_T)trkIndex - 1];
        b_st.site = &it_emlrtRSI;
        likelihood_data[b_i] = TrackEstimator_likelihood(
            &b_st, &d_obj_Estimator_StateEstimator_->TargetSpecifications[0],
            &SD->u1.f2.val, c_estimator_StateEstimator_Stat,
            &e_obj_Estimator_StateEstimator_->TargetSpecifications[0],
            &SD->u1.f2.b_val, d_estimator_StateEstimator_Stat,
            &f_obj_Estimator_StateEstimator_->TargetSpecifications[0],
            &SD->u1.f2.c_val, e_estimator_StateEstimator_Stat, &pdf,
            &z_data[d_tmp], b_d_tmp);
      } else {
        d_tmp = likelihood->size[0] * likelihood->size[1];
        if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
            ((int32_T)((uint32_T)b_i + 1U) > d_tmp)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, d_tmp,
                                        &cd_emlrtBCI, &st);
        }
        likelihood_data[b_i] = 0.0;
      }
    }
  }
}

void JIPDATrackAssigner_assign(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T assigner_AssignmentThreshold, real_T c_assigner_InitializationThresh,
    real_T assigner_MaxNumEvents,
    const c_fusion_tracker_sensorspecs_Ae *c_assigner_Estimator_StateEstim,
    c_fusion_tracker_internal_estim *d_assigner_Estimator_StateEstim,
    d_fusion_tracker_internal_estim *e_assigner_Estimator_StateEstim,
    e_fusion_tracker_internal_estim *f_assigner_Estimator_StateEstim,
    const c_fusion_tracker_sensorspecs_Ae *g_assigner_Estimator_StateEstim,
    const emxArray_struct_T *trackList, const struct0_T *sensorData,
    emxArray_real_T *assignment, emxArray_boolean_T *unassignedTracks,
    struct0_T *unassignedSensorData)
{
  jmp_buf *volatile emlrtJBStack;
  b_emxArray_struct_T *modelData;
  const b_struct_T *trackList_data;
  b_struct_T pdf;
  c_fusion_tracker_targetspecs_Ge b_estimator_TargetSpecification;
  c_fusion_tracker_targetspecs_He c_estimator_TargetSpecification;
  c_fusion_tracker_targetspecs_Pa estimator_TargetSpecifications;
  c_struct_T *modelData_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
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
  real_T time_data[5000];
  real_T z_data[200];
  real_T b_time_data[100];
  real_T detAssignmentProb_data[51];
  real_T Pg;
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
  int32_T d_loop_ub;
  int32_T e_loop_ub;
  int32_T end;
  int32_T f_loop_ub;
  int32_T g_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T k;
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  trackList_data = trackList->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &qp_emlrtRSI;
  emxInit_struct_T1(&st, &modelData, &he_emlrtRTEI);
  b_st.site = &rp_emlrtRSI;
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
  b_st.site = &sp_emlrtRSI;
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
    b_st.site = &tp_emlrtRSI;
    repmat(&b_st, b_time_data, b_time_size, z_size[1], time_data, time_size);
  }
  emxInit_real_T(&st, &likelihoodMatrix, 2, &ud_emlrtRTEI, true);
  if (modelData->size[0] == 1) {
    b_st.site = &up_emlrtRSI;
    c_st.site = &kq_emlrtRSI;
    d_st.site = &mq_emlrtRSI;
    e_st.site = &nq_emlrtRSI;
    f_st.site = &pq_emlrtRSI;
    estimator_TargetSpecifications =
        d_assigner_Estimator_StateEstim->TargetSpecifications[0];
    SD->u2.f7.val = d_assigner_Estimator_StateEstim->SensorSpecifications[0];
    g_st.site = &qq_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &g_st, &SD->u2.f7.val, modelData_data[0].LookTime.data,
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
    g_st.site = &qq_emlrtRSI;
    f_st.site = &pq_emlrtRSI;
    b_estimator_TargetSpecification =
        e_assigner_Estimator_StateEstim->TargetSpecifications[0];
    SD->u2.f7.b_val = e_assigner_Estimator_StateEstim->SensorSpecifications[0];
    g_st.site = &qq_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &g_st, &SD->u2.f7.b_val, modelData_data[0].LookTime.data,
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
    g_st.site = &qq_emlrtRSI;
    f_st.site = &pq_emlrtRSI;
    c_estimator_TargetSpecification =
        f_assigner_Estimator_StateEstim->TargetSpecifications[0];
    SD->u2.f7.c_val = f_assigner_Estimator_StateEstim->SensorSpecifications[0];
    g_st.site = &qq_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &g_st, &SD->u2.f7.c_val, modelData_data[0].LookTime.data,
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
    g_st.site = &qq_emlrtRSI;
    e_st.site = &oq_emlrtRSI;
    SD->u2.f7.estimator_SensorSpecifications = *g_assigner_Estimator_StateEstim;
    f_st.site = &dt_emlrtRSI;
    c_AerospaceMonostaticRadar_upda(
        &f_st, &SD->u2.f7.estimator_SensorSpecifications,
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
      emlrtIntegerCheckR2012b((real_T)trackList->size[0] + 1.0, &b_emlrtDCI,
                              &b_st);
    }
    d_loop_ub = (z_size[1] + 1) * (trackList->size[0] + 1);
    loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
    likelihoodMatrix->size[0] = z_size[1] + 1;
    likelihoodMatrix->size[1] = trackList->size[0] + 1;
    emxEnsureCapacity_real_T(&b_st, likelihoodMatrix, loop_ub, &ud_emlrtRTEI);
    likelihoodMatrix_data = likelihoodMatrix->data;
    for (i = 0; i < d_loop_ub; i++) {
      real_T trkIndex;
      c_st.site = &lq_emlrtRSI;
      d_st.site = &et_emlrtRSI;
      trkIndex = parseIndex(&d_st, trackList, z_size, (real_T)i + 1.0, &Pg);
      if ((trkIndex == 0.0) && (Pg > 0.0)) {
        d_st.site = &ft_emlrtRSI;
        e_st.site = &pt_emlrtRSI;
        if (((int32_T)Pg < 1) || ((int32_T)Pg > z_size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, z_size[1], &lb_emlrtBCI,
                                        &c_st);
        }
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)i + 1U) < 1) ||
            ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, loop_ub,
                                        &rb_emlrtBCI, &c_st);
        }
        likelihoodMatrix_data[i] =
            c_assigner_Estimator_StateEstim->ClutterModel.ClutterDensity;
      } else if ((Pg == 0.0) && (trkIndex > 0.0)) {
        if (time_size[1] < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, time_size[1], &nb_emlrtBCI,
                                        &c_st);
        }
        d_st.site = &gt_emlrtRSI;
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)trkIndex, 1, trackList->size[0], &mb_emlrtBCI, &d_st);
        }
        pdf = trackList_data[(int32_T)trkIndex - 1];
        e_st.site = &qt_emlrtRSI;
        TrackEstimator_predict(&e_st, &estimator_TargetSpecifications,
                               d_assigner_Estimator_StateEstim->TrackingFilter,
                               &b_estimator_TargetSpecification,
                               e_assigner_Estimator_StateEstim->TrackingFilter,
                               &c_estimator_TargetSpecification,
                               f_assigner_Estimator_StateEstim->TrackingFilter,
                               &pdf, time_data[0]);
        e_st.site = &rt_emlrtRSI;
        f_st.site = &qx_emlrtRSI;
        Pg = c_MultiModalEstimator_gateProba(&f_st,
                                             assigner_AssignmentThreshold);
        f_st.site = &rx_emlrtRSI;
        g_st.site = &rx_emlrtRSI;
        Pg *= c_MultiModalEstimator_detection(
            &g_st, &SD->u2.f7.val, &SD->u2.f7.b_val, &SD->u2.f7.c_val,
            pdf.Hypothesis, pdf.LogWeights, pdf.IsValid);
        g_st.site = &cy_emlrtRSI;
        h_st.site = &dy_emlrtRSI;
        if (!(Pg >= 0.0)) {
          emlrtErrorWithMessageIdR2018a(
              &h_st, &l_emlrtRTEI, "MATLAB:validators:mustBeNonnegative",
              "MATLAB:validators:mustBeNonnegative", 0);
        }
        h_st.site = &dy_emlrtRSI;
        i_st.site = &il_emlrtRSI;
        if (!(Pg < 1.0)) {
          emlrtErrorWithMessageIdR2018a(
              &i_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
              "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
        }
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)i + 1U) < 1) ||
            ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, loop_ub,
                                        &rb_emlrtBCI, &e_st);
        }
        likelihoodMatrix_data[i] = 1.0 - Pg * pdf.ExistenceProbability;
      } else if ((Pg == 0.0) && (trkIndex == 0.0)) {
        loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
        if (((int32_T)((uint32_T)i + 1U) < 1) ||
            ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, loop_ub,
                                        &rb_emlrtBCI, &c_st);
        }
        likelihoodMatrix_data[i] = 1.0;
      } else {
        real_T d;
        real_T d_tmp;
        if (((int32_T)Pg < 1) || ((int32_T)Pg > z_size[1])) {
          emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, z_size[1], &ob_emlrtBCI,
                                        &c_st);
        }
        if (((int32_T)trkIndex < 1) ||
            ((int32_T)trkIndex > trackList->size[0])) {
          emlrtDynamicBoundsCheckR2012b(
              (int32_T)trkIndex, 1, trackList->size[0], &sb_emlrtBCI, &c_st);
        }
        b = (((int32_T)Pg < 1) || ((int32_T)Pg > time_size[1]));
        if (b) {
          emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, time_size[1],
                                        &tb_emlrtBCI, &c_st);
        }
        d_tmp = time_data[(int32_T)Pg - 1];
        loop_ub = 4 * ((int32_T)Pg - 1);
        pdf = trackList_data[(int32_T)trkIndex - 1];
        d_st.site = &ht_emlrtRSI;
        d = TrackEstimator_distance(
            &d_st, &estimator_TargetSpecifications, &SD->u2.f7.val,
            d_assigner_Estimator_StateEstim->TrackingFilter,
            &b_estimator_TargetSpecification, &SD->u2.f7.b_val,
            e_assigner_Estimator_StateEstim->TrackingFilter,
            &c_estimator_TargetSpecification, &SD->u2.f7.c_val,
            f_assigner_Estimator_StateEstim->TrackingFilter, &pdf,
            &z_data[loop_ub], d_tmp);
        if (d < assigner_AssignmentThreshold) {
          if (((int32_T)trkIndex < 1) ||
              ((int32_T)trkIndex > trackList->size[0])) {
            emlrtDynamicBoundsCheckR2012b(
                (int32_T)trkIndex, 1, trackList->size[0], &vb_emlrtBCI, &c_st);
          }
          if (b) {
            emlrtDynamicBoundsCheckR2012b((int32_T)Pg, 1, time_size[1],
                                          &wb_emlrtBCI, &c_st);
          }
          c_loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
          if (((int32_T)((uint32_T)i + 1U) < 1) ||
              ((int32_T)((uint32_T)i + 1U) > c_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1,
                                          c_loop_ub, &rb_emlrtBCI, &c_st);
          }
          pdf = trackList_data[(int32_T)trkIndex - 1];
          d_st.site = &it_emlrtRSI;
          likelihoodMatrix_data[i] = TrackEstimator_likelihood(
              &d_st, &estimator_TargetSpecifications, &SD->u2.f7.val,
              d_assigner_Estimator_StateEstim->TrackingFilter,
              &b_estimator_TargetSpecification, &SD->u2.f7.b_val,
              e_assigner_Estimator_StateEstim->TrackingFilter,
              &c_estimator_TargetSpecification, &SD->u2.f7.c_val,
              f_assigner_Estimator_StateEstim->TrackingFilter, &pdf,
              &z_data[loop_ub], d_tmp);
        } else {
          loop_ub = likelihoodMatrix->size[0] * likelihoodMatrix->size[1];
          if (((int32_T)((uint32_T)i + 1U) < 1) ||
              ((int32_T)((uint32_T)i + 1U) > loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1,
                                          loop_ub, &rb_emlrtBCI, &c_st);
          }
          likelihoodMatrix_data[i] = 0.0;
        }
      }
    }
  } else {
    SD->u2.f7.assigner_Estimator_StateEstimat =
        *d_assigner_Estimator_StateEstim;
    SD->u2.f7.b_assigner_Estimator_StateEstim =
        *e_assigner_Estimator_StateEstim;
    SD->u2.f7.c_assigner_Estimator_StateEstim =
        *f_assigner_Estimator_StateEstim;
    b_st.site = &vp_emlrtRSI;
    c_JIPDATrackAssigner_measuremen(SD, &b_st, assigner_AssignmentThreshold,
                                    c_assigner_Estimator_StateEstim,
                                    &SD->u2.f7.assigner_Estimator_StateEstimat,
                                    &SD->u2.f7.b_assigner_Estimator_StateEstim,
                                    &SD->u2.f7.c_assigner_Estimator_StateEstim,
                                    g_assigner_Estimator_StateEstim, trackList,
                                    z_data, z_size, time_data, time_size,
                                    modelData, likelihoodMatrix);
    likelihoodMatrix_data = likelihoodMatrix->data;
  }
  emxFree_struct_T1(&st, &modelData);
  if (likelihoodMatrix->size[0] < 2) {
    b_loop_ub = 0;
    loop_ub = 0;
  } else {
    b_loop_ub = 1;
    loop_ub = likelihoodMatrix->size[0];
  }
  if (likelihoodMatrix->size[1] < 2) {
    tmp_size = 0;
    c_loop_ub = 0;
  } else {
    tmp_size = 1;
    c_loop_ub = likelihoodMatrix->size[1];
  }
  emxInit_boolean_T(sp, &b_likelihoodMatrix, 2, &qd_emlrtRTEI, true);
  d_loop_ub = loop_ub - b_loop_ub;
  loop_ub = b_likelihoodMatrix->size[0] * b_likelihoodMatrix->size[1];
  b_likelihoodMatrix->size[0] = d_loop_ub;
  c_loop_ub -= tmp_size;
  b_likelihoodMatrix->size[1] = c_loop_ub;
  emxEnsureCapacity_boolean_T(sp, b_likelihoodMatrix, loop_ub, &qd_emlrtRTEI);
  unassignedTracks_data = b_likelihoodMatrix->data;
  for (i = 0; i < c_loop_ub; i++) {
    for (k = 0; k < d_loop_ub; k++) {
      unassignedTracks_data[k + b_likelihoodMatrix->size[0] * i] =
          (likelihoodMatrix_data[(b_loop_ub + k) + likelihoodMatrix->size[0] *
                                                       (tmp_size + i)] > 0.0);
    }
  }
  emxInit_int32_T(sp, &clustTracks, 2, &he_emlrtRTEI);
  st.site = &pp_emlrtRSI;
  nClusters = connectedTracks(&st, b_likelihoodMatrix, clustDets_data,
                              clustDets_size, clustTracks);
  clustTracks_data = clustTracks->data;
  emxFree_boolean_T(sp, &b_likelihoodMatrix);
  b_i = likelihoodMatrix->size[1] - 1;
  loop_ub = unassignedTracks->size[0];
  unassignedTracks->size[0] = likelihoodMatrix->size[1] - 1;
  emxEnsureCapacity_boolean_T(sp, unassignedTracks, loop_ub, &rd_emlrtRTEI);
  unassignedTracks_data = unassignedTracks->data;
  e_loop_ub = likelihoodMatrix->size[1];
  for (i = 0; i <= e_loop_ub - 2; i++) {
    unassignedTracks_data[i] = false;
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
  emxEnsureCapacity_real_T(sp, assignment, loop_ub, &sd_emlrtRTEI);
  assignment_data = assignment->data;
  g_loop_ub = likelihoodMatrix->size[0] * (likelihoodMatrix->size[1] - 1);
  for (i = 0; i < g_loop_ub; i++) {
    assignment_data[i] = 0.0;
  }
  st.site = &op_emlrtRSI;
  if (nClusters > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  emxInit_real_T(sp, &thisClusterTracks, 2, &vd_emlrtRTEI, true);
  emxInit_real_T(sp, &lhood, 2, &ce_emlrtRTEI, true);
  emxInit_real_T(sp, &b_jpda, 2, &ge_emlrtRTEI, true);
  emxInit_int32_T(sp, &ii, 2, &ie_emlrtRTEI);
  emxInit_boolean_T(sp, &unassignedTracks, 2, &td_emlrtRTEI, true);
  emxInit_int32_T(sp, &r, 1, &be_emlrtRTEI);
  emxInit_real_T(sp, &c_jpda, 2, &ee_emlrtRTEI, true);
  emxInit_real_T(sp, &d_jpda, 2, &fe_emlrtRTEI, true);
  for (c_i = 0; c_i < nClusters; c_i++) {
    int32_T h_loop_ub;
    int32_T i_loop_ub;
    st.site = &np_emlrtRSI;
    loop_ub = unassignedTracks->size[0] * unassignedTracks->size[1];
    unassignedTracks->size[0] = 1;
    c_loop_ub = clustTracks->size[1];
    unassignedTracks->size[1] = clustTracks->size[1];
    emxEnsureCapacity_boolean_T(&st, unassignedTracks, loop_ub, &td_emlrtRTEI);
    b_clustTracks_data = unassignedTracks->data;
    for (k = 0; k < c_loop_ub; k++) {
      b_clustTracks_data[k] = (clustTracks_data[k] == c_i + 1);
    }
    b_st.site = &ndb_emlrtRSI;
    eml_find(&b_st, unassignedTracks, ii);
    ii_data = ii->data;
    loop_ub = thisClusterTracks->size[0] * thisClusterTracks->size[1];
    thisClusterTracks->size[0] = 1;
    h_loop_ub = ii->size[1];
    thisClusterTracks->size[1] = ii->size[1];
    emxEnsureCapacity_real_T(&st, thisClusterTracks, loop_ub, &vd_emlrtRTEI);
    thisClusterTracks_data = thisClusterTracks->data;
    for (k = 0; k < h_loop_ub; k++) {
      thisClusterTracks_data[k] = ii_data[k];
    }
    st.site = &mp_emlrtRSI;
    b_clustDets_size[0] = 1;
    loop_ub = clustDets_size[1];
    b_clustDets_size[1] = clustDets_size[1];
    for (k = 0; k < loop_ub; k++) {
      b_clustDets_data[k] = (clustDets_data[k] == c_i + 1);
    }
    c_clustDets_data.data = &b_clustDets_data[0];
    c_clustDets_data.size = &b_clustDets_size[0];
    c_clustDets_data.allocatedSize = 51;
    c_clustDets_data.numDimensions = 2;
    c_clustDets_data.canFreeData = false;
    b_st.site = &ndb_emlrtRSI;
    eml_find(&b_st, &c_clustDets_data, ii);
    ii_data = ii->data;
    loop_ub = 0;
    i_loop_ub = ii->size[1];
    for (k = 0; k < i_loop_ub; k++) {
      loop_ub++;
    }
    if (loop_ub == 0) {
      loop_ub = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = h_loop_ub;
      emxEnsureCapacity_int32_T(sp, ii, loop_ub, &ae_emlrtRTEI);
      ii_data = ii->data;
      for (k = 0; k < h_loop_ub; k++) {
        loop_ub = (int32_T)thisClusterTracks_data[k];
        if ((loop_ub < 1) || (loop_ub > b_i)) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 1, b_i, &ub_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        ii_data[k] = loop_ub;
      }
      loop_ub = ii->size[1];
      for (k = 0; k < loop_ub; k++) {
        unassignedTracks_data[ii_data[k] - 1] = true;
      }
    } else {
      loop_ub = 0;
      for (i = 0; i < h_loop_ub; i++) {
        loop_ub++;
      }
      if (loop_ub == 0) {
        int8_T b_tmp_data[51];
        c_loop_ub = ii->size[1];
        for (k = 0; k < i_loop_ub; k++) {
          if (((int8_T)ii_data[k] < 1) || ((int8_T)ii_data[k] > end)) {
            emlrtDynamicBoundsCheckR2012b((int8_T)ii_data[k], 1, end,
                                          &ac_emlrtBCI, (emlrtConstCTX)sp);
          }
          b_tmp_data[k] = (int8_T)ii_data[k];
        }
        for (k = 0; k < c_loop_ub; k++) {
          unassignedDets_data[b_tmp_data[k] - 1] = true;
        }
      } else {
        int32_T tmp_data[52];
        int32_T j_loop_ub;
        int8_T b_tmp_data[51];
        boolean_T d_tmp_data[51];
        d_loop_ub = ii->size[1] + 1;
        if (f_loop_ub < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, f_loop_ub, &xb_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        tmp_data[0] = 0;
        c_loop_ub = (ii->size[1] / 4) << 2;
        loop_ub = c_loop_ub - 4;
        for (i = 0; i <= loop_ub; i += 4) {
          __m128i r2;
          __m128i r3;
          r2 = _mm_loadu_si128((const __m128i *)&ii_data[i]);
          r3 = _mm_set1_epi32(1);
          _mm_storeu_si128((__m128i *)&b_iv[0], _mm_add_epi32(r2, r3));
          if ((b_iv[0] < 1) || (b_iv[0] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[0], 1, f_loop_ub, &xb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if ((b_iv[1] < 1) || (b_iv[1] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[1], 1, f_loop_ub, &xb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if ((b_iv[2] < 1) || (b_iv[2] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[2], 1, f_loop_ub, &xb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          if ((b_iv[3] < 1) || (b_iv[3] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(b_iv[3], 1, f_loop_ub, &xb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          r2 = _mm_loadu_si128((const __m128i *)&b_iv[0]);
          _mm_storeu_si128((__m128i *)&tmp_data[i + 1], _mm_sub_epi32(r2, r3));
        }
        for (i = c_loop_ub; i < i_loop_ub; i++) {
          loop_ub = ii_data[i] + 1;
          if ((loop_ub < 1) || (loop_ub > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, f_loop_ub, &xb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          tmp_data[i + 1] = loop_ub - 1;
        }
        loop_ub = r->size[0];
        r->size[0] = thisClusterTracks->size[1] + 1;
        emxEnsureCapacity_int32_T(sp, r, loop_ub, &be_emlrtRTEI);
        r4 = r->data;
        if (e_loop_ub < 1) {
          emlrtDynamicBoundsCheckR2012b(1, 1, e_loop_ub, &yb_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        r4[0] = 0;
        for (i = 0; i < h_loop_ub; i++) {
          loop_ub = (int32_T)thisClusterTracks_data[i] + 1;
          if ((loop_ub < 1) || (loop_ub > e_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, e_loop_ub, &yb_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          r4[i + 1] = loop_ub - 1;
        }
        loop_ub = lhood->size[0] * lhood->size[1];
        lhood->size[0] = d_loop_ub;
        c_loop_ub = r->size[0];
        lhood->size[1] = r->size[0];
        emxEnsureCapacity_real_T(sp, lhood, loop_ub, &ce_emlrtRTEI);
        lhood_data = lhood->data;
        for (i = 0; i < c_loop_ub; i++) {
          for (k = 0; k < d_loop_ub; k++) {
            lhood_data[k + lhood->size[0] * i] =
                likelihoodMatrix_data[tmp_data[k] +
                                      likelihoodMatrix->size[0] * r4[i]];
          }
        }
        st.site = &lp_emlrtRSI;
        jpda(&st, lhood, assigner_MaxNumEvents, b_jpda);
        jpda_data = b_jpda->data;
        j_loop_ub = b_jpda->size[0];
        b_loop_ub = b_jpda->size[0] - 1;
        b = ((b_jpda->size[0] - 1 < 1) ||
             (b_jpda->size[0] - 1 > b_jpda->size[0]));
        if (b) {
          emlrtDynamicBoundsCheckR2012b(b_jpda->size[0] - 1, 1, b_jpda->size[0],
                                        &kb_emlrtBCI, (emlrtConstCTX)sp);
        }
        tmp_size = ii->size[1];
        for (i = 0; i < i_loop_ub; i++) {
          if (((int8_T)ii_data[i] < 1) || ((int8_T)ii_data[i] > f_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int8_T)ii_data[i], 1, f_loop_ub,
                                          &bc_emlrtBCI, (emlrtConstCTX)sp);
          }
          b_tmp_data[i] = (int8_T)((int8_T)ii_data[i] - 1);
        }
        loop_ub = r->size[0];
        r->size[0] = h_loop_ub;
        emxEnsureCapacity_int32_T(sp, r, loop_ub, &de_emlrtRTEI);
        r4 = r->data;
        for (i = 0; i < h_loop_ub; i++) {
          loop_ub = (int32_T)thisClusterTracks_data[i];
          if ((loop_ub < 1) || (loop_ub > b_i)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, b_i, &cc_emlrtBCI,
                                          (emlrtConstCTX)sp);
          }
          r4[i] = loop_ub - 1;
        }
        z_size[0] = tmp_size;
        c_loop_ub = r->size[0];
        z_size[1] = r->size[0];
        time_size[0] = b_jpda->size[0] - 1;
        d_loop_ub = b_jpda->size[1];
        time_size[1] = b_jpda->size[1];
        emlrtSubAssignSizeCheckR2012b(&z_size[0], 2, &time_size[0], 2,
                                      &e_emlrtECI, (emlrtCTX)sp);
        loop_ub = c_jpda->size[0] * c_jpda->size[1];
        c_jpda->size[0] = b_jpda->size[0] - 1;
        c_jpda->size[1] = b_jpda->size[1];
        emxEnsureCapacity_real_T(sp, c_jpda, loop_ub, &ee_emlrtRTEI);
        lhood_data = c_jpda->data;
        for (i = 0; i < d_loop_ub; i++) {
          for (k = 0; k <= j_loop_ub - 2; k++) {
            lhood_data[k + c_jpda->size[0] * i] =
                jpda_data[k + b_jpda->size[0] * i];
          }
        }
        for (i = 0; i < c_loop_ub; i++) {
          for (k = 0; k < tmp_size; k++) {
            assignment_data[b_tmp_data[k] + assignment->size[0] * r4[i]] =
                lhood_data[k + tmp_size * i];
          }
        }
        if (b_jpda->size[0] < 1) {
          emlrtDynamicBoundsCheckR2012b(b_jpda->size[0], 1, b_jpda->size[0],
                                        &jb_emlrtBCI, (emlrtConstCTX)sp);
        }
        z_size[0] = 1;
        z_size[1] = r->size[0];
        time_size[0] = 1;
        time_size[1] = b_jpda->size[1];
        emlrtSubAssignSizeCheckR2012b(&z_size[0], 2, &time_size[0], 2,
                                      &d_emlrtECI, (emlrtCTX)sp);
        for (i = 0; i < c_loop_ub; i++) {
          assignment_data[end + assignment->size[0] * r4[i]] =
              jpda_data[b_loop_ub + b_jpda->size[0] * i];
        }
        loop_ub = d_jpda->size[0] * d_jpda->size[1];
        d_jpda->size[0] = b_jpda->size[0] - 1;
        d_jpda->size[1] = b_jpda->size[1];
        emxEnsureCapacity_real_T(sp, d_jpda, loop_ub, &fe_emlrtRTEI);
        lhood_data = d_jpda->data;
        for (i = 0; i < d_loop_ub; i++) {
          for (k = 0; k <= j_loop_ub - 2; k++) {
            lhood_data[k + d_jpda->size[0] * i] =
                jpda_data[k + b_jpda->size[0] * i];
          }
        }
        st.site = &kp_emlrtRSI;
        loop_ub = b_sum(d_jpda, detAssignmentProb_data);
        c_loop_ub = ii->size[1];
        for (i = 0; i < i_loop_ub; i++) {
          if (((int8_T)ii_data[i] < 1) || ((int8_T)ii_data[i] > end)) {
            emlrtDynamicBoundsCheckR2012b((int8_T)ii_data[i], 1, end,
                                          &dc_emlrtBCI, (emlrtConstCTX)sp);
          }
          b_tmp_data[i] = (int8_T)ii_data[i];
        }
        for (k = 0; k < loop_ub; k++) {
          d_tmp_data[k] =
              (detAssignmentProb_data[k] < c_assigner_InitializationThresh);
        }
        if (loop_ub != c_loop_ub) {
          emlrtSubAssignSizeCheck1dR2017a(c_loop_ub, loop_ub, &b_emlrtECI,
                                          (emlrtConstCTX)sp);
        }
        for (k = 0; k < c_loop_ub; k++) {
          unassignedDets_data[b_tmp_data[k] - 1] = d_tmp_data[k];
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
  for (i = 0; i < g_loop_ub; i++) {
    loop_ub = assignment->size[0] * assignment->size[1];
    if (assignment_data[i] > 1.0) {
      if (i > loop_ub - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, loop_ub - 1, &pb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      assignment_data[i] = 1.0;
    }
  }
  loop_ub = assignment->size[0] * assignment->size[1];
  for (i = 0; i < loop_ub; i++) {
    c_loop_ub = assignment->size[0] * assignment->size[1];
    if (assignment_data[i] < 0.0) {
      if (i > c_loop_ub - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, c_loop_ub - 1, &qb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      assignment_data[i] = 0.0;
    }
  }
  if (assignment->size[0] - 1 < 1) {
    c_loop_ub = 0;
  } else {
    if ((assignment->size[0] - 1 < 1) ||
        (assignment->size[0] - 1 > assignment->size[0])) {
      emlrtDynamicBoundsCheckR2012b(assignment->size[0] - 1, 1,
                                    assignment->size[0], &ib_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    c_loop_ub = assignment->size[0] - 1;
  }
  if (assignment->size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(assignment->size[0], 1, assignment->size[0],
                                  &hb_emlrtBCI, (emlrtConstCTX)sp);
  }
  loop_ub = b_jpda->size[0] * b_jpda->size[1];
  b_jpda->size[0] = c_loop_ub;
  b_loop_ub = assignment->size[1];
  b_jpda->size[1] = assignment->size[1];
  emxEnsureCapacity_real_T(sp, b_jpda, loop_ub, &wd_emlrtRTEI);
  jpda_data = b_jpda->data;
  for (i = 0; i < b_loop_ub; i++) {
    for (k = 0; k < c_loop_ub; k++) {
      jpda_data[k + b_jpda->size[0] * i] =
          assignment_data[k + assignment->size[0] * i];
    }
  }
  st.site = &jp_emlrtRSI;
  sum(&st, b_jpda, thisClusterTracks);
  emxFree_real_T(sp, &b_jpda);
  loop_ub = thisClusterTracks->size[0] * thisClusterTracks->size[1];
  thisClusterTracks->size[0] = 1;
  emxEnsureCapacity_real_T(sp, thisClusterTracks, loop_ub, &xd_emlrtRTEI);
  thisClusterTracks_data = thisClusterTracks->data;
  loop_ub = thisClusterTracks->size[1] - 1;
  c_loop_ub = (thisClusterTracks->size[1] / 2) << 1;
  d_loop_ub = c_loop_ub - 2;
  for (i = 0; i <= d_loop_ub; i += 2) {
    __m128d r1;
    r1 = _mm_loadu_pd(&thisClusterTracks_data[i]);
    _mm_storeu_pd(&thisClusterTracks_data[i], _mm_sub_pd(_mm_set1_pd(1.0), r1));
  }
  for (i = c_loop_ub; i <= loop_ub; i++) {
    thisClusterTracks_data[i] = 1.0 - thisClusterTracks_data[i];
  }
  loop_ub = thisClusterTracks->size[0] * thisClusterTracks->size[1];
  thisClusterTracks->size[0] = 1;
  emxEnsureCapacity_real_T(sp, thisClusterTracks, loop_ub, &yd_emlrtRTEI);
  thisClusterTracks_data = thisClusterTracks->data;
  loop_ub = thisClusterTracks->size[1] - 1;
  c_loop_ub = thisClusterTracks->size[1] - 1;
  if (thisClusterTracks->size[1] < 1600) {
    for (i1 = 0; i1 <= loop_ub; i1++) {
      Pg = thisClusterTracks_data[i1];
      thisClusterTracks_data[i1] = muDoubleScalarMax(0.0, Pg);
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    JIPDATrackAssigner_assign_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(                                          \
        JIPDATrackAssigner_assign_numThreads) private(varargin_2)

    for (i1 = 0; i1 <= c_loop_ub; i1++) {
      varargin_2 = thisClusterTracks_data[i1];
      thisClusterTracks_data[i1] = muDoubleScalarMax(0.0, varargin_2);
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  z_size[0] = 1;
  z_size[1] = b_loop_ub;
  emlrtSubAssignSizeCheckR2012b(&z_size[0], 2, &thisClusterTracks->size[0], 2,
                                &c_emlrtECI, (emlrtCTX)sp);
  for (i = 0; i < b_loop_ub; i++) {
    assignment_data[(assignment->size[0] + assignment->size[0] * i) - 1] =
        thisClusterTracks_data[i];
  }
  emxFree_real_T(sp, &thisClusterTracks);
  st.site = &ip_emlrtRSI;
  *unassignedSensorData = *sensorData;
  c_loop_ub = 0;
  loop_ub = 0;
  for (i = 0; i < end; i++) {
    if (unassignedDets_data[i]) {
      c_loop_ub++;
      c_tmp_data[loop_ub] = (int8_T)i;
      loop_ub++;
    }
  }
  unassignedSensorData->DetectionTime.size[0] = 1;
  unassignedSensorData->DetectionTime.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->DetectionTime.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->DetectionTime.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->DetectionTime.data[i] =
        sensorData->DetectionTime.data[i2];
  }
  unassignedSensorData->Azimuth.size[0] = 1;
  unassignedSensorData->Azimuth.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->Azimuth.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->Azimuth.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->Azimuth.data[i] = sensorData->Azimuth.data[i2];
  }
  unassignedSensorData->Elevation.size[0] = 1;
  unassignedSensorData->Elevation.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->Elevation.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->Elevation.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->Elevation.data[i] = sensorData->Elevation.data[i2];
  }
  unassignedSensorData->Range.size[0] = 1;
  unassignedSensorData->Range.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->Range.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->Range.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->Range.data[i] = sensorData->Range.data[i2];
  }
  unassignedSensorData->RangeRate.size[0] = 1;
  unassignedSensorData->RangeRate.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->RangeRate.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, sensorData->RangeRate.size[1] - 1,
                                    &y_emlrtBCI, &st);
    }
    unassignedSensorData->RangeRate.data[i] = sensorData->RangeRate.data[i2];
  }
  unassignedSensorData->AzimuthAccuracy.size[0] = 1;
  unassignedSensorData->AzimuthAccuracy.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->AzimuthAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->AzimuthAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->AzimuthAccuracy.data[i] =
        sensorData->AzimuthAccuracy.data[i2];
  }
  unassignedSensorData->ElevationAccuracy.size[0] = 1;
  unassignedSensorData->ElevationAccuracy.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->ElevationAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->ElevationAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->ElevationAccuracy.data[i] =
        sensorData->ElevationAccuracy.data[i2];
  }
  unassignedSensorData->RangeAccuracy.size[0] = 1;
  unassignedSensorData->RangeAccuracy.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->RangeAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->RangeAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->RangeAccuracy.data[i] =
        sensorData->RangeAccuracy.data[i2];
  }
  unassignedSensorData->RangeRateAccuracy.size[0] = 1;
  unassignedSensorData->RangeRateAccuracy.size[1] = c_loop_ub;
  for (i = 0; i < c_loop_ub; i++) {
    i2 = c_tmp_data[i];
    if (i2 > sensorData->RangeRateAccuracy.size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(
          i2, 0, sensorData->RangeRateAccuracy.size[1] - 1, &y_emlrtBCI, &st);
    }
    unassignedSensorData->RangeRateAccuracy.data[i] =
        sensorData->RangeRateAccuracy.data[i2];
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (JIPDATrackAssigner.c) */
