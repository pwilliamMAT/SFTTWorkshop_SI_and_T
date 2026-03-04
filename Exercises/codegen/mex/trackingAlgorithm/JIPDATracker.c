/*
 * JIPDATracker.c
 *
 * Code generation for function 'JIPDATracker'
 *
 */

/* Include files */
#include "JIPDATracker.h"
#include "AerospaceMonostaticRadar.h"
#include "IPDAEstimator.h"
#include "JIPDATrackAssigner.h"
#include "JIPDATrackInitiator.h"
#include "JIPDATrackUpdater.h"
#include "ObjectTrackOutputter.h"
#include "TrackEstimator1.h"
#include "TrackListManager.h"
#include "eml_int_forloop_overflow_check.h"
#include "ifWhileCond.h"
#include "indexShapeCheck.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "trackEstimator.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo n_emlrtRSI = {
    128,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo o_emlrtRSI = {
    131,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo p_emlrtRSI = {
    135,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo q_emlrtRSI = {
    140,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo r_emlrtRSI = {
    179,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo s_emlrtRSI = {
    149,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo t_emlrtRSI = {
    160,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo u_emlrtRSI = {
    170,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo v_emlrtRSI = {
    175,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo hk_emlrtRSI = {
    42,                                        /* lineNo */
    "SensorDataScheduler/SensorDataScheduler", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo ik_emlrtRSI = {
    38,                          /* lineNo */
    "JIPDATrackInitiator/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo jk_emlrtRSI = {
    37,                         /* lineNo */
    "JIPDATrackAssigner/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackAssigner.m" /* pathName */
};

static emlrtRSInfo kk_emlrtRSI = {
    36,                        /* lineNo */
    "JIPDATrackUpdater/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo fl_emlrtRSI = {
    29,                                          /* lineNo */
    "JIPDATrackMaintainer/JIPDATrackMaintainer", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pathName */
};

static emlrtRSInfo ml_emlrtRSI = {
    34,                           /* lineNo */
    "JIPDATrackMaintainer/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pathName */
};

static emlrtRSInfo xm_emlrtRSI = {
    189,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo ym_emlrtRSI = {
    197,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo an_emlrtRSI = {
    200,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo bn_emlrtRSI = {
    217,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo cn_emlrtRSI = {
    220,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo dn_emlrtRSI = {
    222,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo en_emlrtRSI = {
    225,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo fn_emlrtRSI = {
    228,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo gn_emlrtRSI = {
    234,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo hn_emlrtRSI = {
    237,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pathName
                                                                          */
};

static emlrtRSInfo in_emlrtRSI = {
    51,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo jn_emlrtRSI = {
    56,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo kn_emlrtRSI = {
    57,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo ln_emlrtRSI = {
    59,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo yn_emlrtRSI = {
    17,            /* lineNo */
    "uniquetolcg", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m" /* pathName
                                                                          */
};

static emlrtRSInfo bp_emlrtRSI = {
    42,                                             /* lineNo */
    "sort",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sort.m" /* pathName */
};

static emlrtRSInfo cp_emlrtRSI = {
    65,                         /* lineNo */
    "SensorDataScheduler/next", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo dp_emlrtRSI = {
    67,                         /* lineNo */
    "SensorDataScheduler/next", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pathName */
};

static emlrtRSInfo fp_emlrtRSI = {
    280,                                   /* lineNo */
    "AerospaceMonostaticRadar/selectTime", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+sensorspecs/"
    "AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo onb_emlrtRSI = {
    50,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo pnb_emlrtRSI = {
    51,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo qnb_emlrtRSI = {
    55,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo rnb_emlrtRSI = {
    61,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo snb_emlrtRSI = {
    67,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo tnb_emlrtRSI = {
    73,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo unb_emlrtRSI = {
    75,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo rob_emlrtRSI = {
    131,                             /* lineNo */
    "updateModelAndInitializeTrack", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo sob_emlrtRSI = {
    132,                             /* lineNo */
    "updateModelAndInitializeTrack", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m" /* pathName */
};

static emlrtRSInfo tob_emlrtRSI = {
    44,                            /* lineNo */
    "ObjectTrackOutputter/output", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo uob_emlrtRSI = {
    45,                            /* lineNo */
    "ObjectTrackOutputter/output", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo vob_emlrtRSI = {
    69,                                             /* lineNo */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pathName */
};

static emlrtRSInfo wob_emlrtRSI = {
    69,                                                            /* lineNo */
    "horzcatStructList",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/horzcatStructList.m" /* pathName
                                                                    */
};

static emlrtRSInfo xob_emlrtRSI = {
    25,                                                            /* lineNo */
    "horzcatStructList",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/horzcatStructList.m" /* pathName
                                                                    */
};

static emlrtBCInfo emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    35,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    44,                              /* lineNo */
    50,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    50,                              /* lineNo */
    67,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    73,                             /* lineNo */
    81,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    87,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    95,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    104,                            /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    118,                            /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    27,            /* lineNo */
    43,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    27,            /* lineNo */
    29,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtECInfo emlrtECI = {
    -1,            /* nDims */
    19,            /* lineNo */
    9,             /* colNo */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m" /* pName
                                                                          */
};

static emlrtBCInfo k_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    75,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    47,                            /* lineNo */
    13,                            /* colNo */
    "",                            /* aName */
    "ObjectTrackOutputter/output", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    19,            /* lineNo */
    19,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    35,            /* lineNo */
    21,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    36,            /* lineNo */
    17,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    33,            /* lineNo */
    16,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    28,            /* lineNo */
    37,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtDCInfo emlrtDCI = {
    65,                         /* lineNo */
    53,                         /* colNo */
    "SensorDataScheduler/next", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m", /* pName */
    1                        /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    65,                         /* lineNo */
    53,                         /* colNo */
    "",                         /* aName */
    "SensorDataScheduler/next", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    45,                              /* lineNo */
    74,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    50,                              /* lineNo */
    32,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    265,                                   /* lineNo */
    46,                                    /* colNo */
    "",                                    /* aName */
    "AerospaceMonostaticRadar/selectTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+sensorspecs/"
    "AerospaceMonostaticRadar.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    266,                                   /* lineNo */
    52,                                    /* colNo */
    "",                                    /* aName */
    "AerospaceMonostaticRadar/selectTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+sensorspecs/"
    "AerospaceMonostaticRadar.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    268,                                   /* lineNo */
    60,                                    /* colNo */
    "",                                    /* aName */
    "AerospaceMonostaticRadar/selectTime", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+sensorspecs/"
    "AerospaceMonostaticRadar.m", /* pName */
    0                             /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    52,                              /* lineNo */
    13,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    97,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    17,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    46,                            /* lineNo */
    13,                            /* colNo */
    "",                            /* aName */
    "ObjectTrackOutputter/output", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    73,                             /* lineNo */
    73,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    73,                             /* lineNo */
    90,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    77,                             /* lineNo */
    27,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    31,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackInitiator.m", /* pName */
    0                        /* checkKind */
};

static emlrtRTEInfo oc_emlrtRTEI = {
    125,            /* lineNo */
    18,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo pc_emlrtRTEI = {
    135,            /* lineNo */
    46,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo qc_emlrtRTEI = {
    294,            /* lineNo */
    63,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo rc_emlrtRTEI = {
    294,            /* lineNo */
    13,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo sc_emlrtRTEI = {
    57,                    /* lineNo */
    65,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = {
    57,                    /* lineNo */
    17,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pName */
};

static emlrtRTEInfo uc_emlrtRTEI = {
    23,            /* lineNo */
    9,             /* colNo */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m" /* pName
                                                                          */
};

static emlrtRTEInfo vc_emlrtRTEI = {
    187,            /* lineNo */
    30,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo wc_emlrtRTEI = {
    59,                    /* lineNo */
    13,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = {
    234,            /* lineNo */
    84,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo yc_emlrtRTEI = {
    31,                 /* lineNo */
    13,                 /* colNo */
    "TrackListManager", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "TrackListManager.m" /* pName */
};

static emlrtRTEInfo ad_emlrtRTEI = {
    48,                     /* lineNo */
    13,                     /* colNo */
    "JIPDATrackMaintainer", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pName */
};

static emlrtRTEInfo bd_emlrtRTEI = {
    52,                     /* lineNo */
    13,                     /* colNo */
    "JIPDATrackMaintainer", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pName */
};

static emlrtRTEInfo cd_emlrtRTEI = {
    234,            /* lineNo */
    13,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo dd_emlrtRTEI = {
    36,                 /* lineNo */
    13,                 /* colNo */
    "TrackListManager", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "TrackListManager.m" /* pName */
};

static emlrtRTEInfo ed_emlrtRTEI = {
    237,            /* lineNo */
    63,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo fd_emlrtRTEI = {
    44,                     /* lineNo */
    13,                     /* colNo */
    "ObjectTrackOutputter", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pName */
};

static emlrtRTEInfo gd_emlrtRTEI = {
    67,                                                            /* lineNo */
    10,                                                            /* colNo */
    "horzcatStructList",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/horzcatStructList.m" /* pName */
};

static emlrtRTEInfo hd_emlrtRTEI = {
    46,                     /* lineNo */
    13,                     /* colNo */
    "ObjectTrackOutputter", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pName */
};

static emlrtRTEInfo id_emlrtRTEI = {
    222,            /* lineNo */
    95,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo jd_emlrtRTEI = {
    225,            /* lineNo */
    88,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo kd_emlrtRTEI = {
    225,            /* lineNo */
    25,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo ld_emlrtRTEI = {
    228,            /* lineNo */
    115,            /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo md_emlrtRTEI = {
    228,            /* lineNo */
    26,             /* colNo */
    "JIPDATracker", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/JIPDATracker.m" /* pName
                                                                          */
};

static emlrtRTEInfo nd_emlrtRTEI = {
    52,                    /* lineNo */
    13,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "SensorDataScheduler.m" /* pName */
};

static emlrtRTEInfo od_emlrtRTEI = {
    45,                     /* lineNo */
    13,                     /* colNo */
    "ObjectTrackOutputter", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "ObjectTrackOutputter.m" /* pName */
};

static emlrtRTEInfo pd_emlrtRTEI = {
    1,             /* lineNo */
    16,            /* colNo */
    "uniquetolcg", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/uniquetolcg.m" /* pName
                                                                          */
};

/* Function Definitions */
void JIPDATracker_setupImpl(trackingAlgorithmStackData *SD,
                            const emlrtStack *sp,
                            fusion_tracker_JIPDATracker *b_tracker)
{
  c_fusion_tracker_targetspecs_Ge args_next_value_f2;
  c_fusion_tracker_targetspecs_He args_next_value_f3;
  c_fusion_tracker_targetspecs_Pa args_next_value_f1;
  cell_4 val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T d_args_next_value_f1_StateTrans[9];
  real_T e_args_next_value_f1_StateTrans[9];
  real_T f_args_next_value_f2_StateTrans[9];
  real_T f_args_next_value_f3_StateTrans[9];
  real_T g_args_next_value_f2_StateTrans[9];
  real_T g_args_next_value_f3_StateTrans[9];
  real_T args_next_value;
  real_T args_value;
  real_T c_args_next_value_f1_StateTrans;
  real_T c_args_next_value_f1_SurvivalMo;
  real_T c_args_next_value_f2_StateTrans;
  real_T c_args_next_value_f2_SurvivalMo;
  real_T c_args_next_value_f3_StateTrans;
  real_T d_args_next_value_f2_StateTrans;
  real_T d_args_next_value_f3_StateTrans;
  real_T e_args_next_value_f2_StateTrans;
  real_T e_args_next_value_f3_StateTrans;
  int32_T i;
  boolean_T c_args_next_value_f1_IsLockedDa;
  boolean_T c_args_next_value_f2_IsLockedDa;
  boolean_T d_args_next_value_f2_IsLockedDa;
  boolean_T flag;
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  SD->u4.f13.spec = b_tracker->SensorSpecifications[0];
  b_tracker->SensorSpecifications[0] = SD->u4.f13.spec;
  st.site = &n_emlrtRSI;
  b_st.site = &l_emlrtRSI;
  flag = (b_tracker->isInitialized == 1);
  if (flag) {
    b_tracker->TunablePropsChanged = true;
    b_tracker->tunablePropertyChanged[1] = true;
  }
  val = b_tracker->TargetSpecifications;
  val.f1 = b_tracker->TargetSpecifications.f1;
  st.site = &o_emlrtRSI;
  b_st.site = &i_emlrtRSI;
  flag = (b_tracker->isInitialized == 1);
  if (flag) {
    b_tracker->TunablePropsChanged = true;
    b_tracker->tunablePropertyChanged[0] = true;
  }
  st.site = &o_emlrtRSI;
  b_tracker->TargetSpecifications = val;
  val = b_tracker->TargetSpecifications;
  val.f2 = b_tracker->TargetSpecifications.f2;
  st.site = &o_emlrtRSI;
  b_st.site = &i_emlrtRSI;
  flag = (b_tracker->isInitialized == 1);
  if (flag) {
    b_tracker->TunablePropsChanged = true;
    b_tracker->tunablePropertyChanged[0] = true;
  }
  st.site = &o_emlrtRSI;
  b_tracker->TargetSpecifications = val;
  val = b_tracker->TargetSpecifications;
  val.f3 = b_tracker->TargetSpecifications.f3;
  st.site = &o_emlrtRSI;
  b_st.site = &i_emlrtRSI;
  flag = (b_tracker->isInitialized == 1);
  if (flag) {
    b_tracker->TunablePropsChanged = true;
    b_tracker->tunablePropertyChanged[0] = true;
  }
  st.site = &o_emlrtRSI;
  b_tracker->TargetSpecifications = val;
  c_emxInitStruct_fusion_tracker_(sp, &SD->u4.f13.obj, &pc_emlrtRTEI, true);
  SD->u4.f13.obj.TargetSpecifications = b_tracker->TargetSpecifications;
  SD->u4.f13.obj.SensorSpecifications[0] = b_tracker->SensorSpecifications[0];
  SD->u4.f13.obj.InternalTrackList->size[0] = 0;
  st.site = &p_emlrtRSI;
  TrackListManager_setup(SD, &st, &SD->u4.f13.obj);
  c_emxCopyStruct_fusion_tracker_(sp, &b_tracker->TrackListManager,
                                  &SD->u4.f13.obj, &oc_emlrtRTEI);
  c_emxFreeStruct_fusion_tracker_(sp, &SD->u4.f13.obj);
  st.site = &q_emlrtRSI;
  b_tracker->coder_buffer_pobj1.TimeTolerance = 1.0E-6;
  b_st.site = &hk_emlrtRSI;
  b_tracker->Scheduler = &b_tracker->coder_buffer_pobj1;
  SD->u4.f13.spec = b_tracker->SensorSpecifications[0];
  args_next_value_f1 = b_tracker->TargetSpecifications.f1;
  args_next_value_f2 = b_tracker->TargetSpecifications.f2;
  args_next_value_f3 = b_tracker->TargetSpecifications.f3;
  st.site = &s_emlrtRSI;
  b_st.site = &ik_emlrtRSI;
  trackEstimator(SD, &b_st, &args_next_value_f1, &args_next_value_f2,
                 &args_next_value_f3, &SD->u4.f13.spec,
                 &b_tracker->coder_buffer_pobj0[0], &SD->u4.f13.obj_Estimator);
  SD->u4.f13.initiators.TargetSpecifications.f1 = args_next_value_f1;
  SD->u4.f13.initiators.TargetSpecifications.f2 = args_next_value_f2;
  SD->u4.f13.initiators.TargetSpecifications.f3 = args_next_value_f3;
  SD->u4.f13.initiators.SensorSpecifications[0] = SD->u4.f13.spec;
  SD->u4.f13.initiators.Estimator = SD->u4.f13.obj_Estimator;
  b_tracker->Initiator[0] = SD->u4.f13.initiators;
  SD->u4.f13.spec = b_tracker->SensorSpecifications[0];
  args_next_value_f1 = b_tracker->TargetSpecifications.f1;
  args_next_value_f2 = b_tracker->TargetSpecifications.f2;
  args_next_value_f3 = b_tracker->TargetSpecifications.f3;
  args_next_value = b_tracker->MaxMahalanobisDistance;
  args_value = b_tracker->MaxNumEvents;
  st.site = &t_emlrtRSI;
  b_st.site = &jk_emlrtRSI;
  trackEstimator(SD, &b_st, &args_next_value_f1, &args_next_value_f2,
                 &args_next_value_f3, &SD->u4.f13.spec,
                 &b_tracker->coder_buffer_pobj0[3], &SD->u4.f13.obj_Estimator);
  SD->u4.f13.assigners.TargetSpecifications.f1 = args_next_value_f1;
  SD->u4.f13.assigners.TargetSpecifications.f2 = args_next_value_f2;
  SD->u4.f13.assigners.TargetSpecifications.f3 = args_next_value_f3;
  SD->u4.f13.assigners.SensorSpecifications[0] = SD->u4.f13.spec;
  SD->u4.f13.assigners.AssignmentThreshold = args_next_value;
  SD->u4.f13.assigners.InitializationThreshold = 0.0;
  SD->u4.f13.assigners.MaxNumEvents = args_value;
  SD->u4.f13.assigners.Estimator = SD->u4.f13.obj_Estimator;
  b_tracker->Assigner[0] = SD->u4.f13.assigners;
  SD->u4.f13.spec = b_tracker->SensorSpecifications[0];
  args_next_value_f1 = b_tracker->TargetSpecifications.f1;
  args_next_value_f2 = b_tracker->TargetSpecifications.f2;
  args_next_value_f3 = b_tracker->TargetSpecifications.f3;
  args_next_value = b_tracker->MaxMahalanobisDistance;
  st.site = &u_emlrtRSI;
  b_st.site = &kk_emlrtRSI;
  trackEstimator(SD, &b_st, &args_next_value_f1, &args_next_value_f2,
                 &args_next_value_f3, &SD->u4.f13.spec,
                 &b_tracker->coder_buffer_pobj0[6], &SD->u4.f13.obj_Estimator);
  SD->u4.f13.updaters.TargetSpecifications.f1 = args_next_value_f1;
  SD->u4.f13.updaters.TargetSpecifications.f2 = args_next_value_f2;
  SD->u4.f13.updaters.TargetSpecifications.f3 = args_next_value_f3;
  SD->u4.f13.updaters.SensorSpecifications[0] = SD->u4.f13.spec;
  SD->u4.f13.updaters.AssignmentThreshold = args_next_value;
  SD->u4.f13.updaters.Estimator = SD->u4.f13.obj_Estimator;
  b_tracker->Updater[0] = SD->u4.f13.updaters;
  args_next_value = b_tracker->TargetSpecifications.f1.StateTransitionModel
                        .PropVelocityMean[0];
  args_value = b_tracker->TargetSpecifications.f1.StateTransitionModel
                   .PropVelocityMean[1];
  c_args_next_value_f1_StateTrans =
      b_tracker->TargetSpecifications.f1.StateTransitionModel
          .PropVelocityMean[2];
  for (i = 0; i < 9; i++) {
    d_args_next_value_f1_StateTrans[i] =
        b_tracker->TargetSpecifications.f1.StateTransitionModel
            .PropVelocityVariance[i];
  }
  for (i = 0; i < 9; i++) {
    e_args_next_value_f1_StateTrans[i] =
        b_tracker->TargetSpecifications.f1.StateTransitionModel
            .PropAccelerationVariance[i];
  }
  c_args_next_value_f1_SurvivalMo =
      b_tracker->TargetSpecifications.f1.SurvivalModel.SurvivalRate;
  flag = b_tracker->TargetSpecifications.f1.IsLockedDataType[0];
  c_args_next_value_f1_IsLockedDa =
      b_tracker->TargetSpecifications.f1.IsLockedDataType[1];
  c_args_next_value_f2_StateTrans =
      b_tracker->TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[0];
  d_args_next_value_f2_StateTrans =
      b_tracker->TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[1];
  e_args_next_value_f2_StateTrans =
      b_tracker->TargetSpecifications.f2.StateTransitionModel
          .PropVelocityMean[2];
  for (i = 0; i < 9; i++) {
    f_args_next_value_f2_StateTrans[i] =
        b_tracker->TargetSpecifications.f2.StateTransitionModel
            .PropVelocityVariance[i];
  }
  for (i = 0; i < 9; i++) {
    g_args_next_value_f2_StateTrans[i] =
        b_tracker->TargetSpecifications.f2.StateTransitionModel
            .PropAccelerationVariance[i];
  }
  c_args_next_value_f2_SurvivalMo =
      b_tracker->TargetSpecifications.f2.SurvivalModel.SurvivalRate;
  c_args_next_value_f2_IsLockedDa =
      b_tracker->TargetSpecifications.f2.IsLockedDataType[0];
  d_args_next_value_f2_IsLockedDa =
      b_tracker->TargetSpecifications.f2.IsLockedDataType[1];
  c_args_next_value_f3_StateTrans =
      b_tracker->TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[0];
  d_args_next_value_f3_StateTrans =
      b_tracker->TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[1];
  e_args_next_value_f3_StateTrans =
      b_tracker->TargetSpecifications.f3.StateTransitionModel
          .PropVelocityMean[2];
  for (i = 0; i < 9; i++) {
    f_args_next_value_f3_StateTrans[i] =
        b_tracker->TargetSpecifications.f3.StateTransitionModel
            .PropVelocityVariance[i];
  }
  for (i = 0; i < 9; i++) {
    g_args_next_value_f3_StateTrans[i] =
        b_tracker->TargetSpecifications.f3.StateTransitionModel
            .PropAccelerationVariance[i];
  }
  real_T c_args_next_value_f3_SurvivalMo;
  boolean_T c_args_next_value_f3_IsLockedDa;
  boolean_T d_args_next_value_f3_IsLockedDa;
  c_args_next_value_f3_SurvivalMo =
      b_tracker->TargetSpecifications.f3.SurvivalModel.SurvivalRate;
  c_args_next_value_f3_IsLockedDa =
      b_tracker->TargetSpecifications.f3.IsLockedDataType[0];
  d_args_next_value_f3_IsLockedDa =
      b_tracker->TargetSpecifications.f3.IsLockedDataType[1];
  SD->u4.f13.spec = b_tracker->SensorSpecifications[0];
  SD->u4.f13.r.TargetSpecifications.f1.StateTransitionModel
      .PropVelocityMean[0] = args_next_value;
  SD->u4.f13.r.TargetSpecifications.f1.StateTransitionModel
      .PropVelocityMean[1] = args_value;
  SD->u4.f13.r.TargetSpecifications.f1.StateTransitionModel
      .PropVelocityMean[2] = c_args_next_value_f1_StateTrans;
  memcpy(&SD->u4.f13.r.TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         &d_args_next_value_f1_StateTrans[0], 9U * sizeof(real_T));
  memcpy(&SD->u4.f13.r.TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         &e_args_next_value_f1_StateTrans[0], 9U * sizeof(real_T));
  SD->u4.f13.r.TargetSpecifications.f1.SurvivalModel.SurvivalRate =
      c_args_next_value_f1_SurvivalMo;
  SD->u4.f13.r.TargetSpecifications.f1.IsLockedDataType[0] = flag;
  SD->u4.f13.r.TargetSpecifications.f1.IsLockedDataType[1] =
      c_args_next_value_f1_IsLockedDa;
  SD->u4.f13.r.TargetSpecifications.f2.StateTransitionModel
      .PropVelocityMean[0] = c_args_next_value_f2_StateTrans;
  SD->u4.f13.r.TargetSpecifications.f2.StateTransitionModel
      .PropVelocityMean[1] = d_args_next_value_f2_StateTrans;
  SD->u4.f13.r.TargetSpecifications.f2.StateTransitionModel
      .PropVelocityMean[2] = e_args_next_value_f2_StateTrans;
  memcpy(&SD->u4.f13.r.TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         &f_args_next_value_f2_StateTrans[0], 9U * sizeof(real_T));
  memcpy(&SD->u4.f13.r.TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         &g_args_next_value_f2_StateTrans[0], 9U * sizeof(real_T));
  SD->u4.f13.r.TargetSpecifications.f2.SurvivalModel.SurvivalRate =
      c_args_next_value_f2_SurvivalMo;
  SD->u4.f13.r.TargetSpecifications.f2.IsLockedDataType[0] =
      c_args_next_value_f2_IsLockedDa;
  SD->u4.f13.r.TargetSpecifications.f2.IsLockedDataType[1] =
      d_args_next_value_f2_IsLockedDa;
  SD->u4.f13.r.TargetSpecifications.f3.StateTransitionModel
      .PropVelocityMean[0] = c_args_next_value_f3_StateTrans;
  SD->u4.f13.r.TargetSpecifications.f3.StateTransitionModel
      .PropVelocityMean[1] = d_args_next_value_f3_StateTrans;
  SD->u4.f13.r.TargetSpecifications.f3.StateTransitionModel
      .PropVelocityMean[2] = e_args_next_value_f3_StateTrans;
  memcpy(&SD->u4.f13.r.TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         &f_args_next_value_f3_StateTrans[0], 9U * sizeof(real_T));
  memcpy(&SD->u4.f13.r.TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         &g_args_next_value_f3_StateTrans[0], 9U * sizeof(real_T));
  SD->u4.f13.r.TargetSpecifications.f3.SurvivalModel.SurvivalRate =
      c_args_next_value_f3_SurvivalMo;
  SD->u4.f13.r.TargetSpecifications.f3.IsLockedDataType[0] =
      c_args_next_value_f3_IsLockedDa;
  SD->u4.f13.r.TargetSpecifications.f3.IsLockedDataType[1] =
      d_args_next_value_f3_IsLockedDa;
  SD->u4.f13.r.SensorSpecifications[0] = SD->u4.f13.spec;
  st.site = &v_emlrtRSI;
  ObjectTrackOutputter_setup(SD, &st, &SD->u4.f13.r,
                             &b_tracker->coder_buffer_pobj0[9]);
  b_tracker->Outputter = SD->u4.f13.r;
  st.site = &r_emlrtRSI;
  val = b_tracker->TargetSpecifications;
  SD->u4.f13.spec = b_tracker->SensorSpecifications[0];
  args_next_value = b_tracker->c_ConfirmationExistenceProbabil;
  args_value = b_tracker->DeletionExistenceProbability;
  b_st.site = &fl_emlrtRSI;
  c_st.site = &gl_emlrtRSI;
  d_st.site = &hl_emlrtRSI;
  if (!(args_next_value > 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &b_emlrtRTEI,
                                  "MATLAB:validators:mustBePositive",
                                  "MATLAB:validators:mustBePositive", 0);
  }
  d_st.site = &hl_emlrtRSI;
  e_st.site = &il_emlrtRSI;
  if (!(args_next_value < 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
        "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
  }
  b_st.site = &fl_emlrtRSI;
  c_st.site = &jl_emlrtRSI;
  d_st.site = &kl_emlrtRSI;
  if (!(args_value > 0.0)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &b_emlrtRTEI,
                                  "MATLAB:validators:mustBePositive",
                                  "MATLAB:validators:mustBePositive", 0);
  }
  d_st.site = &kl_emlrtRSI;
  st.site = &r_emlrtRSI;
  b_st.site = &ml_emlrtRSI;
  trackEstimator(SD, &b_st, &val.f1, &val.f2, &val.f3, &SD->u4.f13.spec,
                 &b_tracker->coder_buffer_pobj0[12], &SD->u4.f13.obj_Estimator);
  b_tracker->TrackMaintenance.TargetSpecifications.f1.StateTransitionModel
      .PropVelocityMean[0] = val.f1.StateTransitionModel.PropVelocityMean[0];
  b_tracker->TrackMaintenance.TargetSpecifications.f1.StateTransitionModel
      .PropVelocityMean[1] = val.f1.StateTransitionModel.PropVelocityMean[1];
  b_tracker->TrackMaintenance.TargetSpecifications.f1.StateTransitionModel
      .PropVelocityMean[2] = val.f1.StateTransitionModel.PropVelocityMean[2];
  for (i = 0; i < 9; i++) {
    b_tracker->TrackMaintenance.TargetSpecifications.f1.StateTransitionModel
        .PropVelocityVariance[i] =
        val.f1.StateTransitionModel.PropVelocityVariance[i];
  }
  for (i = 0; i < 9; i++) {
    b_tracker->TrackMaintenance.TargetSpecifications.f1.StateTransitionModel
        .PropAccelerationVariance[i] =
        val.f1.StateTransitionModel.PropAccelerationVariance[i];
  }
  b_tracker->TrackMaintenance.TargetSpecifications.f1.SurvivalModel =
      val.f1.SurvivalModel;
  b_tracker->TrackMaintenance.TargetSpecifications.f1.IsLockedDataType[0] =
      val.f1.IsLockedDataType[0];
  b_tracker->TrackMaintenance.TargetSpecifications.f1.IsLockedDataType[1] =
      val.f1.IsLockedDataType[1];
  b_tracker->TrackMaintenance.TargetSpecifications.f2.StateTransitionModel
      .PropVelocityMean[0] = val.f2.StateTransitionModel.PropVelocityMean[0];
  b_tracker->TrackMaintenance.TargetSpecifications.f2.StateTransitionModel
      .PropVelocityMean[1] = val.f2.StateTransitionModel.PropVelocityMean[1];
  b_tracker->TrackMaintenance.TargetSpecifications.f2.StateTransitionModel
      .PropVelocityMean[2] = val.f2.StateTransitionModel.PropVelocityMean[2];
  for (i = 0; i < 9; i++) {
    b_tracker->TrackMaintenance.TargetSpecifications.f2.StateTransitionModel
        .PropVelocityVariance[i] =
        val.f2.StateTransitionModel.PropVelocityVariance[i];
  }
  for (i = 0; i < 9; i++) {
    b_tracker->TrackMaintenance.TargetSpecifications.f2.StateTransitionModel
        .PropAccelerationVariance[i] =
        val.f2.StateTransitionModel.PropAccelerationVariance[i];
  }
  b_tracker->TrackMaintenance.TargetSpecifications.f2.SurvivalModel =
      val.f2.SurvivalModel;
  b_tracker->TrackMaintenance.TargetSpecifications.f2.IsLockedDataType[0] =
      val.f2.IsLockedDataType[0];
  b_tracker->TrackMaintenance.TargetSpecifications.f2.IsLockedDataType[1] =
      val.f2.IsLockedDataType[1];
  b_tracker->TrackMaintenance.TargetSpecifications.f3.StateTransitionModel
      .PropVelocityMean[0] = val.f3.StateTransitionModel.PropVelocityMean[0];
  b_tracker->TrackMaintenance.TargetSpecifications.f3.StateTransitionModel
      .PropVelocityMean[1] = val.f3.StateTransitionModel.PropVelocityMean[1];
  b_tracker->TrackMaintenance.TargetSpecifications.f3.StateTransitionModel
      .PropVelocityMean[2] = val.f3.StateTransitionModel.PropVelocityMean[2];
  for (i = 0; i < 9; i++) {
    b_tracker->TrackMaintenance.TargetSpecifications.f3.StateTransitionModel
        .PropVelocityVariance[i] =
        val.f3.StateTransitionModel.PropVelocityVariance[i];
  }
  for (i = 0; i < 9; i++) {
    b_tracker->TrackMaintenance.TargetSpecifications.f3.StateTransitionModel
        .PropAccelerationVariance[i] =
        val.f3.StateTransitionModel.PropAccelerationVariance[i];
  }
  b_tracker->TrackMaintenance.TargetSpecifications.f3.SurvivalModel =
      val.f3.SurvivalModel;
  b_tracker->TrackMaintenance.TargetSpecifications.f3.IsLockedDataType[0] =
      val.f3.IsLockedDataType[0];
  b_tracker->TrackMaintenance.TargetSpecifications.f3.IsLockedDataType[1] =
      val.f3.IsLockedDataType[1];
  b_tracker->TrackMaintenance.SensorSpecifications[0] = SD->u4.f13.spec;
  b_tracker->TrackMaintenance.ConfirmationThreshold = args_next_value;
  b_tracker->TrackMaintenance.DeletionThreshold = args_value;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f1.StateTransitionModel =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f1.StateTransitionModel;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f1.SurvivalModel =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f1.SurvivalModel;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f1.IsLockedDataType[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f1.IsLockedDataType[0];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f1.IsLockedDataType[1] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f1.IsLockedDataType[1];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f2.StateTransitionModel =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f2.StateTransitionModel;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f2.SurvivalModel =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f2.SurvivalModel;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f2.IsLockedDataType[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f2.IsLockedDataType[0];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f2.IsLockedDataType[1] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f2.IsLockedDataType[1];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f3.StateTransitionModel =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f3.StateTransitionModel;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f3.SurvivalModel =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f3.SurvivalModel;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f3.IsLockedDataType[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f3.IsLockedDataType[0];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .TargetSpecifications.f3.IsLockedDataType[1] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator
          .TargetSpecifications.f3.IsLockedDataType[1];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .SensorSpecifications[0] = SD->u4.f13.obj_Estimator.StateEstimator
                                     .StateEstimator.SensorSpecifications[0];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator
      .DeletionThreshold =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.DeletionThreshold;
  SD->u4.f13.t103_f1.TargetSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TargetSpecifications[0];
  SD->u4.f13.t103_f1.SensorSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f1
          .SensorSpecifications[0];
  SD->u4.f13.t103_f1.TrackingFilter =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f1
          .TrackingFilter;
  SD->u4.f13.t103_f2.TargetSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TargetSpecifications[0];
  SD->u4.f13.t103_f2.SensorSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f2
          .SensorSpecifications[0];
  SD->u4.f13.t103_f2.TrackingFilter =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f2
          .TrackingFilter;
  SD->u4.f13.t103_f3.TargetSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TargetSpecifications[0];
  SD->u4.f13.t103_f3.SensorSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f3
          .SensorSpecifications[0];
  SD->u4.f13.t103_f3.TrackingFilter =
      SD->u4.f13.obj_Estimator.StateEstimator.StateEstimator.Estimators.f3
          .TrackingFilter;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator.Estimators
      .f1 = SD->u4.f13.t103_f1;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator.Estimators
      .f2 = SD->u4.f13.t103_f2;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.StateEstimator.Estimators
      .f3 = SD->u4.f13.t103_f3;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] =
      SD->u4.f13.obj_Estimator.StateEstimator.ExistenceEstimator
          .SensorSpecifications[0];
  b_tracker->TrackMaintenance.Estimator.StateEstimator.ExistenceEstimator
      .DetectionProbability = SD->u4.f13.obj_Estimator.StateEstimator
                                  .ExistenceEstimator.DetectionProbability;
  b_tracker->TrackMaintenance.Estimator.StateEstimator.ExistenceEstimator
      .SurvivalProbability = SD->u4.f13.obj_Estimator.StateEstimator
                                 .ExistenceEstimator.SurvivalProbability;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void JIPDATracker_stepImpl(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                           fusion_tracker_JIPDATracker *b_tracker,
                           const real_T varargin_1_LookTime_data[],
                           const int32_T varargin_1_LookTime_size[2],
                           const real_T varargin_1_LookAzimuth_data[],
                           const int32_T varargin_1_LookAzimuth_size[2],
                           const real_T varargin_1_LookElevation_data[],
                           const int32_T varargin_1_LookElevation_size[2],
                           const real_T varargin_1_DetectionTime_data[],
                           const int32_T varargin_1_DetectionTime_size[2],
                           const real_T varargin_1_Azimuth_data[],
                           const int32_T varargin_1_Azimuth_size[2],
                           const real_T varargin_1_Elevation_data[],
                           const int32_T varargin_1_Elevation_size[2],
                           const real_T varargin_1_Range_data[],
                           const int32_T varargin_1_Range_size[2],
                           const real_T varargin_1_RangeRate_data[],
                           const int32_T varargin_1_RangeRate_size[2],
                           const real_T varargin_1_AzimuthAccuracy_data[],
                           const int32_T varargin_1_AzimuthAccuracy_size[2],
                           const real_T c_varargin_1_ElevationAccuracy_[],
                           const int32_T d_varargin_1_ElevationAccuracy_[2],
                           const real_T varargin_1_RangeAccuracy_data[],
                           const int32_T varargin_1_RangeAccuracy_size[2],
                           const real_T c_varargin_1_RangeRateAccuracy_[],
                           const int32_T d_varargin_1_RangeRateAccuracy_[2],
                           emxArray_struct1_T *varargout_1)
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  b_emxArray_struct_T *modelData;
  b_struct_T a;
  b_struct_T *val_data;
  c_fusion_tracker_internal_compo *obj;
  c_fusion_tracker_targetspecs_Ge d_t44_Estimator_StateEstimator_;
  c_fusion_tracker_targetspecs_He e_t44_Estimator_StateEstimator_;
  c_fusion_tracker_targetspecs_Pa c_t44_Estimator_StateEstimator_;
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
  emxArray_boolean_T e_tmp_data;
  emxArray_boolean_T *a__1;
  emxArray_boolean_T *isConfirmed;
  emxArray_boolean_T *toDeleteTracks;
  emxArray_int32_T *iidx;
  emxArray_int8_T *ia;
  emxArray_real_T *assignment;
  emxArray_real_T *processTimestamps;
  emxArray_real_T *y;
  emxArray_real_T_1x100 dpb_emlrtRSI;
  emxArray_struct1_T *tracksObjectTrack;
  emxArray_struct_T *val;
  struct0_T multiSensorData;
  struct0_T unassignedSensorData_idx_0;
  struct1_T s1;
  struct1_T *tracksObjectTrack_data;
  struct1_T *varargout_1_data;
  trackingEKF *c_estimator_StateEstimator_Stat;
  trackingEKF *d_estimator_StateEstimator_Stat;
  trackingEKF *e_estimator_StateEstimator_Stat;
  real_T time_data[5000];
  real_T z_data[200];
  real_T b_time_data[100];
  real_T statePdf_StateCovariance[36];
  real_T statePdf_State[6];
  real_T c_statePdf_ObjectClassProbabili[3];
  real_T b_i;
  real_T timeInterval_idx_0;
  real_T timeInterval_idx_1;
  real_T *processTimestamps_data;
  real_T *y_data;
  int32_T tmp_data[101];
  int32_T b_time_size[2];
  int32_T time_size[2];
  int32_T tmp_size[2];
  int32_T z_size[2];
  int32_T JIPDATracker_stepImpl_numThreads;
  int32_T b_iidx;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T d_i;
  int32_T e_i;
  int32_T exitg1;
  int32_T i;
  int32_T ic_size;
  int32_T k;
  int32_T loop_ub;
  int32_T n;
  int32_T *iidx_data;
  uint32_T u;
  uint32_T u1;
  char_T expl_temp[10];
  int8_T b_tmp_data[100];
  int8_T c_tmp_data[50];
  int8_T i1;
  int8_T *ia_data;
  boolean_T d_tmp_data[100];
  boolean_T emlrtHadParallelError = false;
  boolean_T tf;
  boolean_T *toDeleteTracks_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  f_st.prev = &d_st;
  f_st.tls = d_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &xm_emlrtRSI;
  obj = b_tracker->Scheduler;
  b_st.site = &in_emlrtRSI;
  c_AerospaceMonostaticRadar_time(
      varargin_1_LookTime_data, varargin_1_LookTime_size,
      varargin_1_DetectionTime_data, varargin_1_DetectionTime_size,
      dpb_emlrtRSI.data, dpb_emlrtRSI.size);
  b_st.site = &jn_emlrtRSI;
  c_AerospaceMonostaticRadar_time(
      varargin_1_LookTime_data, varargin_1_LookTime_size,
      varargin_1_DetectionTime_data, varargin_1_DetectionTime_size, time_data,
      time_size);
  b_st.site = &kn_emlrtRSI;
  emxInit_real_T(&b_st, &y, 1, &vc_emlrtRTEI, true);
  k = y->size[0];
  y->size[0] = time_size[1] + 1;
  emxEnsureCapacity_real_T(&b_st, y, k, &sc_emlrtRTEI);
  y_data = y->data;
  y_data[0] = -1.7976931348623157E+308;
  k = time_size[1];
  for (i = 0; i < k; i++) {
    y_data[i + 1] = time_data[i];
  }
  b_i = obj->TimeTolerance;
  c_st.site = &yn_emlrtRSI;
  emxInit_int32_T(&c_st, &iidx, 1, &vc_emlrtRTEI);
  d_st.site = &ao_emlrtRSI;
  sort(&d_st, y, iidx);
  iidx_data = iidx->data;
  y_data = y->data;
  b_iidx = iidx->size[0];
  n = iidx->size[0];
  loop_ub = iidx->size[0];
  if (iidx->size[0] < 1600) {
    for (c_i = 0; c_i < b_iidx; c_i++) {
      if ((iidx_data[c_i] < 1) || (iidx_data[c_i] > b_iidx)) {
        emlrtDynamicBoundsCheckR2012b(iidx_data[c_i], 1, b_iidx, &m_emlrtBCI,
                                      &b_st);
      }
      tmp_data[c_i] = iidx_data[c_i];
    }
  } else {
    emlrtEnterParallelRegion(&b_st, omp_in_parallel());
    emlrtPushJmpBuf(&b_st, &emlrtJBStack);
    JIPDATracker_stepImpl_numThreads =
        emlrtAllocRegionTLSs(b_st.tls, omp_in_parallel(), omp_get_max_threads(),
                             omp_get_num_procs());
#pragma omp parallel num_threads(JIPDATracker_stepImpl_numThreads) private(    \
        e_st, emlrtJBEnviron) firstprivate(emlrtHadParallelError, b_st)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        e_st.prev = &b_st;
        e_st.tls = emlrtAllocTLS(&b_st, omp_get_thread_num());
        e_st.site = NULL;
        emlrtSetJmpBuf(&e_st, &emlrtJBEnviron);
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (c_i = 0; c_i < loop_ub; c_i++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          if ((iidx_data[c_i] < 1) || (iidx_data[c_i] > b_iidx)) {
            emlrtDynamicBoundsCheckR2012b(iidx_data[c_i], 1, b_iidx,
                                          &m_emlrtBCI, &e_st);
          }
          tmp_data[c_i] = iidx_data[c_i];
        } else {
          emlrtHadParallelError = true;
        }
      }
    }
    emlrtPopJmpBuf(&b_st, &emlrtJBStack);
    emlrtExitParallelRegion(&b_st, omp_in_parallel());
  }
  b_loop_ub = y->size[0];
  if (n != y->size[0]) {
    emlrtSubAssignSizeCheck1dR2017a(n, y->size[0], &emlrtECI, &b_st);
  }
  for (i = 0; i < n; i++) {
    iidx_data[tmp_data[i] - 1] = i + 1;
  }
  emxInit_real_T(&b_st, &processTimestamps, 1, &nd_emlrtRTEI, true);
  k = processTimestamps->size[0];
  processTimestamps->size[0] = 1;
  emxEnsureCapacity_real_T(&b_st, processTimestamps, k, &tc_emlrtRTEI);
  processTimestamps_data = processTimestamps->data;
  processTimestamps_data[0] = y_data[0];
  ic_size = y->size[0];
  emxInit_int8_T(&b_st, &ia, &pd_emlrtRTEI);
  k = ia->size[0];
  ia->size[0] = 1;
  emxEnsureCapacity_int8_T(&b_st, ia, k, &uc_emlrtRTEI);
  ia_data = ia->data;
  ia_data[0] = 1;
  k = 0;
  for (i = 0; i <= b_loop_ub - 2; i++) {
    if (k + 1 > b_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_loop_ub, &j_emlrtBCI, &b_st);
    }
    if (i + 2 > b_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, b_loop_ub, &i_emlrtBCI, &b_st);
    }
    if (muDoubleScalarAbs(y_data[k] - y_data[i + 1]) > b_i) {
      loop_ub = processTimestamps->size[0];
      n = processTimestamps->size[0];
      processTimestamps->size[0]++;
      emxEnsureCapacity_real_T(&b_st, processTimestamps, n, &vc_emlrtRTEI);
      processTimestamps_data = processTimestamps->data;
      if (i + 2 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 2, 1, b_loop_ub, &q_emlrtBCI, &b_st);
      }
      processTimestamps_data[loop_ub] = y_data[i + 1];
      loop_ub = ia->size[0];
      n = ia->size[0];
      ia->size[0]++;
      emxEnsureCapacity_int8_T(&b_st, ia, n, &vc_emlrtRTEI);
      ia_data = ia->data;
      ia_data[loop_ub] = (int8_T)(i + 2);
      k = i + 1;
    }
    if (i + 2 > ic_size) {
      emlrtDynamicBoundsCheckR2012b(i + 2, 1, ic_size, &p_emlrtBCI, &b_st);
    }
  }
  emxFree_real_T(&b_st, &y);
  loop_ub = ia->size[0];
  for (i = 0; i < loop_ub; i++) {
    n = ia_data[i];
    if ((n < 1) || (n > b_iidx)) {
      emlrtDynamicBoundsCheckR2012b(n, 1, b_iidx, &n_emlrtBCI, &b_st);
    }
  }
  emxFree_int8_T(&b_st, &ia);
  for (i = 0; i < b_iidx; i++) {
    k = iidx_data[i];
    if ((k < 1) || (k > ic_size)) {
      emlrtDynamicBoundsCheckR2012b(k, 1, ic_size, &o_emlrtBCI, &b_st);
    }
  }
  emxFree_int32_T(&b_st, &iidx);
  b_st.site = &ln_emlrtRSI;
  c_st.site = &bp_emlrtRSI;
  b_sort(&c_st, processTimestamps);
  processTimestamps_data = processTimestamps->data;
  loop_ub = processTimestamps->size[0];
  k = obj->ProcessingTimestamps->size[0];
  obj->ProcessingTimestamps->size[0] = processTimestamps->size[0];
  emxEnsureCapacity_real_T(&st, obj->ProcessingTimestamps, k, &wc_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    obj->ProcessingTimestamps->data[i] = processTimestamps_data[i];
  }
  emxFree_real_T(&st, &processTimestamps);
  obj->CurrentIndex = 1.0;
  emxInit_boolean_T(sp, &a__1, 1, &vc_emlrtRTEI, true);
  emxInit_real_T(sp, &assignment, 2, &vc_emlrtRTEI, true);
  c_emxInitStruct_fusion_tracker_(sp, &SD->u4.f12.obj, &xc_emlrtRTEI, true);
  c_emxInitStruct_fusion_tracker_(sp, &SD->u4.f12.b_obj, &kd_emlrtRTEI, true);
  emxInit_struct_T1(sp, &modelData, &vc_emlrtRTEI);
  do {
    exitg1 = 0;
    st.site = &ym_emlrtRSI;
    obj = b_tracker->Scheduler;
    tf = (obj->CurrentIndex == obj->ProcessingTimestamps->size[0]);
    if (!tf) {
      st.site = &an_emlrtRSI;
      obj = b_tracker->Scheduler;
      b_i = obj->CurrentIndex;
      b_st.site = &cp_emlrtRSI;
      indexShapeCheck(&b_st, obj->ProcessingTimestamps->size[0]);
      k = obj->ProcessingTimestamps->size[0];
      if (b_i != (int32_T)muDoubleScalarFloor(b_i)) {
        emlrtIntegerCheckR2012b(b_i, &emlrtDCI, &st);
      }
      if (((int32_T)b_i < 1) || ((int32_T)b_i > k)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_i, 1, k, &r_emlrtBCI, &st);
      }
      if (b_i + 1.0 != (int32_T)muDoubleScalarFloor(b_i + 1.0)) {
        emlrtIntegerCheckR2012b(b_i + 1.0, &emlrtDCI, &st);
      }
      if (((int32_T)(b_i + 1.0) < 1) || ((int32_T)(b_i + 1.0) > k)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(b_i + 1.0), 1, k, &r_emlrtBCI,
                                      &st);
      }
      timeInterval_idx_0 = obj->ProcessingTimestamps->data[(int32_T)b_i - 1];
      timeInterval_idx_1 =
          obj->ProcessingTimestamps->data[(int32_T)(b_i + 1.0) - 1];
      b_st.site = &dp_emlrtRSI;
      k = varargin_1_LookTime_size[1];
      n = 0;
      loop_ub = 0;
      for (i = 0; i < k; i++) {
        b_i = varargin_1_LookTime_data[i];
        tf = ((b_i > timeInterval_idx_0) && (b_i <= timeInterval_idx_1));
        if (tf) {
          n++;
          b_tmp_data[loop_ub] = (int8_T)i;
          loop_ub++;
        }
      }
      multiSensorData.LookTime.size[0] = 1;
      multiSensorData.LookTime.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = b_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_LookTime_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0, varargin_1_LookTime_size[1] - 1,
                                        &u_emlrtBCI, &b_st);
        }
        multiSensorData.LookTime.data[i] = varargin_1_LookTime_data[i1];
      }
      multiSensorData.LookAzimuth.size[0] = 1;
      multiSensorData.LookAzimuth.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = b_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_LookAzimuth_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(
              i1, 0, varargin_1_LookAzimuth_size[1] - 1, &v_emlrtBCI, &b_st);
        }
        multiSensorData.LookAzimuth.data[i] = varargin_1_LookAzimuth_data[i1];
      }
      multiSensorData.LookElevation.size[0] = 1;
      multiSensorData.LookElevation.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = b_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_LookElevation_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(
              i1, 0, varargin_1_LookElevation_size[1] - 1, &w_emlrtBCI, &b_st);
        }
        multiSensorData.LookElevation.data[i] =
            varargin_1_LookElevation_data[i1];
      }
      k = varargin_1_DetectionTime_size[1];
      c_st.site = &fp_emlrtRSI;
      n = 0;
      loop_ub = 0;
      for (i = 0; i < k; i++) {
        b_i = varargin_1_DetectionTime_data[i];
        tf = ((b_i > timeInterval_idx_0) && (b_i <= timeInterval_idx_1));
        if (tf) {
          n++;
          c_tmp_data[loop_ub] = (int8_T)i;
          loop_ub++;
        }
      }
      multiSensorData.DetectionTime.size[0] = 1;
      multiSensorData.DetectionTime.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_DetectionTime_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(
              i1, 0, varargin_1_DetectionTime_size[1] - 1, &y_emlrtBCI, &c_st);
        }
        multiSensorData.DetectionTime.data[i] =
            varargin_1_DetectionTime_data[i1];
      }
      multiSensorData.Azimuth.size[0] = 1;
      multiSensorData.Azimuth.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_Azimuth_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0, varargin_1_Azimuth_size[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.Azimuth.data[i] = varargin_1_Azimuth_data[i1];
      }
      multiSensorData.Elevation.size[0] = 1;
      multiSensorData.Elevation.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_Elevation_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0, varargin_1_Elevation_size[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.Elevation.data[i] = varargin_1_Elevation_data[i1];
      }
      multiSensorData.Range.size[0] = 1;
      multiSensorData.Range.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_Range_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0, varargin_1_Range_size[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.Range.data[i] = varargin_1_Range_data[i1];
      }
      multiSensorData.RangeRate.size[0] = 1;
      multiSensorData.RangeRate.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_RangeRate_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0, varargin_1_RangeRate_size[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.RangeRate.data[i] = varargin_1_RangeRate_data[i1];
      }
      multiSensorData.AzimuthAccuracy.size[0] = 1;
      multiSensorData.AzimuthAccuracy.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_AzimuthAccuracy_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0,
                                        varargin_1_AzimuthAccuracy_size[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.AzimuthAccuracy.data[i] =
            varargin_1_AzimuthAccuracy_data[i1];
      }
      multiSensorData.ElevationAccuracy.size[0] = 1;
      multiSensorData.ElevationAccuracy.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > d_varargin_1_ElevationAccuracy_[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0,
                                        d_varargin_1_ElevationAccuracy_[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.ElevationAccuracy.data[i] =
            c_varargin_1_ElevationAccuracy_[i1];
      }
      multiSensorData.RangeAccuracy.size[0] = 1;
      multiSensorData.RangeAccuracy.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > varargin_1_RangeAccuracy_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(
              i1, 0, varargin_1_RangeAccuracy_size[1] - 1, &y_emlrtBCI, &c_st);
        }
        multiSensorData.RangeAccuracy.data[i] =
            varargin_1_RangeAccuracy_data[i1];
      }
      multiSensorData.RangeRateAccuracy.size[0] = 1;
      multiSensorData.RangeRateAccuracy.size[1] = n;
      for (i = 0; i < n; i++) {
        i1 = c_tmp_data[i];
        if ((i1 < 0) || (i1 > d_varargin_1_RangeRateAccuracy_[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0,
                                        d_varargin_1_RangeRateAccuracy_[1] - 1,
                                        &y_emlrtBCI, &c_st);
        }
        multiSensorData.RangeRateAccuracy.data[i] =
            c_varargin_1_RangeRateAccuracy_[i1];
      }
      obj->CurrentIndex++;
      st.site = &bn_emlrtRSI;
      c_AerospaceMonostaticRadar_time(
          multiSensorData.LookTime.data, multiSensorData.LookTime.size,
          multiSensorData.DetectionTime.data,
          multiSensorData.DetectionTime.size, time_data, time_size);
      if (time_size[1] != 0) {
        tmp_size[0] = 1;
        k = time_size[1];
        tmp_size[1] = time_size[1];
        for (i = 0; i < k; i++) {
          d_tmp_data[i] = !muDoubleScalarIsNaN(time_data[i]);
        }
        e_tmp_data.data = &d_tmp_data[0];
        e_tmp_data.size = &tmp_size[0];
        e_tmp_data.allocatedSize = 100;
        e_tmp_data.numDimensions = 2;
        e_tmp_data.canFreeData = false;
        st.site = &cn_emlrtRSI;
        if (ifWhileCond(&st, &e_tmp_data)) {
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->u4.f12.obj, &b_tracker->TrackListManager, &id_emlrtRTEI);
          SD->u4.f12.b_tracker =
              b_tracker->Assigner[0]
                  .Estimator.StateEstimator.StateEstimator.Estimators.f1;
          SD->u4.f12.c_tracker =
              b_tracker->Assigner[0]
                  .Estimator.StateEstimator.StateEstimator.Estimators.f2;
          SD->u4.f12.d_tracker =
              b_tracker->Assigner[0]
                  .Estimator.StateEstimator.StateEstimator.Estimators.f3;
          st.site = &dn_emlrtRSI;
          JIPDATrackAssigner_assign(
              SD, &st, b_tracker->Assigner[0].AssignmentThreshold,
              b_tracker->Assigner[0].InitializationThreshold,
              b_tracker->Assigner[0].MaxNumEvents,
              &b_tracker->Assigner[0]
                   .Estimator.StateEstimator.StateEstimator
                   .SensorSpecifications[0],
              &SD->u4.f12.b_tracker, &SD->u4.f12.c_tracker,
              &SD->u4.f12.d_tracker,
              &b_tracker->Assigner[0]
                   .Estimator.StateEstimator.ExistenceEstimator
                   .SensorSpecifications[0],
              SD->u4.f12.obj.InternalTrackList, &multiSensorData, assignment,
              a__1, &unassignedSensorData_idx_0);
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->u4.f12.obj, &b_tracker->TrackListManager, &jd_emlrtRTEI);
          c_emxCopyStruct_fusion_tracker_(sp, &SD->u4.f12.b_obj,
                                          &b_tracker->TrackListManager,
                                          &kd_emlrtRTEI);
          loop_ub = SD->u4.f12.obj.InternalTrackList->size[0];
          k = SD->u4.f12.b_obj.InternalTrackList->size[0];
          SD->u4.f12.b_obj.InternalTrackList->size[0] =
              SD->u4.f12.obj.InternalTrackList->size[0];
          emxEnsureCapacity_struct_T(sp, SD->u4.f12.b_obj.InternalTrackList, k,
                                     &dd_emlrtRTEI);
          for (i = 0; i < loop_ub; i++) {
            SD->u4.f12.b_obj.InternalTrackList->data[i] =
                SD->u4.f12.obj.InternalTrackList->data[i];
          }
          SD->u4.f12.tracker = b_tracker->Updater[0].Estimator.StateEstimator;
          st.site = &en_emlrtRSI;
          JIPDATrackUpdater_update(
              SD, &st, b_tracker->Updater[0].AssignmentThreshold,
              &SD->u4.f12.tracker, SD->u4.f12.b_obj.InternalTrackList,
              multiSensorData.LookTime.data, multiSensorData.LookTime.size,
              multiSensorData.LookAzimuth.data,
              multiSensorData.LookAzimuth.size,
              multiSensorData.LookElevation.data,
              multiSensorData.LookElevation.size,
              multiSensorData.DetectionTime.data,
              multiSensorData.DetectionTime.size, multiSensorData.Azimuth.data,
              multiSensorData.Azimuth.size, multiSensorData.Elevation.data,
              multiSensorData.Elevation.size, multiSensorData.Range.data,
              multiSensorData.Range.size, multiSensorData.RangeRate.data,
              multiSensorData.RangeRate.size,
              multiSensorData.AzimuthAccuracy.data,
              multiSensorData.AzimuthAccuracy.size,
              multiSensorData.ElevationAccuracy.data,
              multiSensorData.ElevationAccuracy.size,
              multiSensorData.RangeAccuracy.data,
              multiSensorData.RangeAccuracy.size,
              multiSensorData.RangeRateAccuracy.data,
              multiSensorData.RangeRateAccuracy.size, assignment);
          c_emxCopyStruct_fusion_tracker_(sp, &b_tracker->TrackListManager,
                                          &SD->u4.f12.b_obj, &kd_emlrtRTEI);
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->u4.f12.obj, &b_tracker->TrackListManager, &ld_emlrtRTEI);
          st.site = &fn_emlrtRSI;
          SD->u4.f12.r2 = b_tracker->Initiator[0];
          SD->u4.f12.t98_Estimator_StateEstimator =
              SD->u4.f12.r2.Estimator.StateEstimator;
          c_t44_Estimator_StateEstimator_ =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f1.TargetSpecifications[0];
          SD->u4.f12.c_t44_Estimator_StateEstimator_ =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f1.SensorSpecifications[0];
          d_t44_Estimator_StateEstimator_ =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f2.TargetSpecifications[0];
          SD->u4.f12.d_t44_Estimator_StateEstimator_ =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f2.SensorSpecifications[0];
          e_t44_Estimator_StateEstimator_ =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f3.TargetSpecifications[0];
          SD->u4.f12.e_t44_Estimator_StateEstimator_ =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f3.SensorSpecifications[0];
          u = b_tracker->LastTrackID;
          b_st.site = &onb_emlrtRSI;
          c_AerospaceMonostaticRadar_pars(
              &b_st, unassignedSensorData_idx_0.LookTime.data,
              unassignedSensorData_idx_0.LookTime.size,
              unassignedSensorData_idx_0.LookAzimuth.data,
              unassignedSensorData_idx_0.LookAzimuth.size,
              unassignedSensorData_idx_0.LookElevation.data,
              unassignedSensorData_idx_0.LookElevation.size,
              unassignedSensorData_idx_0.DetectionTime.data,
              unassignedSensorData_idx_0.DetectionTime.size,
              unassignedSensorData_idx_0.Azimuth.data,
              unassignedSensorData_idx_0.Azimuth.size,
              unassignedSensorData_idx_0.Elevation.data,
              unassignedSensorData_idx_0.Elevation.size,
              unassignedSensorData_idx_0.Range.data,
              unassignedSensorData_idx_0.Range.size,
              unassignedSensorData_idx_0.RangeRate.data,
              unassignedSensorData_idx_0.RangeRate.size,
              unassignedSensorData_idx_0.AzimuthAccuracy.data,
              unassignedSensorData_idx_0.AzimuthAccuracy.size,
              unassignedSensorData_idx_0.ElevationAccuracy.data,
              unassignedSensorData_idx_0.ElevationAccuracy.size,
              unassignedSensorData_idx_0.RangeAccuracy.data,
              unassignedSensorData_idx_0.RangeAccuracy.size,
              unassignedSensorData_idx_0.RangeRateAccuracy.data,
              unassignedSensorData_idx_0.RangeRateAccuracy.size, z_data, z_size,
              modelData);
          modelData_data = modelData->data;
          b_st.site = &pnb_emlrtRSI;
          c_AerospaceMonostaticRadar_time(
              unassignedSensorData_idx_0.LookTime.data,
              unassignedSensorData_idx_0.LookTime.size,
              unassignedSensorData_idx_0.DetectionTime.data,
              unassignedSensorData_idx_0.DetectionTime.size, time_data,
              time_size);
          if (time_size[1] == 1) {
            b_time_size[0] = 1;
            b_time_size[1] = 1;
            k = time_size[0] - 1;
            if (k >= 0) {
              memcpy(&b_time_data[0], &time_data[0],
                     (uint32_T)(k + 1) * sizeof(real_T));
            }
            b_st.site = &qnb_emlrtRSI;
            repmat(&b_st, b_time_data, b_time_size, z_size[1], time_data,
                   time_size);
          }
          b_st.site = &rnb_emlrtRSI;
          c_st.site = &rnb_emlrtRSI;
          a.Time = c_TrackEstimator_sampleDistribu(
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f1.TrackingFilter,
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f2.TrackingFilter,
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f3.TrackingFilter,
              &a.TrackID, &a.Age, &a.IsConfirmed, &a.IsCoasted, a.Hypothesis,
              a.LogWeights, a.IsValid, &a.ExistenceProbability);
          c_st.site = &hq_emlrtRSI;
          k = z_size[1];
          loop_ub = z_size[1];
          for (i = 0; i < k; i++) {
            SD->u4.f12.newTracks_data[i] = a;
          }
          c_estimator_StateEstimator_Stat =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f1.TrackingFilter;
          d_estimator_StateEstimator_Stat =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f2.TrackingFilter;
          e_estimator_StateEstimator_Stat =
              SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                  .f3.TrackingFilter;
          if (modelData->size[0] == 1) {
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f1
                .StateTransitionModel.PropVelocityMean[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f1.StateTransitionModel
                    .PropVelocityMean[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f1
                .StateTransitionModel.PropVelocityMean[1] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f1.StateTransitionModel
                    .PropVelocityMean[1];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f1
                .StateTransitionModel.PropVelocityMean[2] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f1.StateTransitionModel
                    .PropVelocityMean[2];
            memcpy(&SD->u4.f12.r3.StateEstimator.StateEstimator
                        .TargetSpecifications.f1.StateTransitionModel
                        .PropVelocityVariance[0],
                   &SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                        .TargetSpecifications.f1.StateTransitionModel
                        .PropVelocityVariance[0],
                   9U * sizeof(real_T));
            memcpy(&SD->u4.f12.r3.StateEstimator.StateEstimator
                        .TargetSpecifications.f1.StateTransitionModel
                        .PropAccelerationVariance[0],
                   &SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                        .TargetSpecifications.f1.StateTransitionModel
                        .PropAccelerationVariance[0],
                   9U * sizeof(real_T));
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f1
                .SurvivalModel =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f1.SurvivalModel;
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f1
                .IsLockedDataType[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f1.IsLockedDataType[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f1
                .IsLockedDataType[1] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f1.IsLockedDataType[1];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f2
                .StateTransitionModel.PropVelocityMean[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f2.StateTransitionModel
                    .PropVelocityMean[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f2
                .StateTransitionModel.PropVelocityMean[1] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f2.StateTransitionModel
                    .PropVelocityMean[1];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f2
                .StateTransitionModel.PropVelocityMean[2] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f2.StateTransitionModel
                    .PropVelocityMean[2];
            memcpy(&SD->u4.f12.r3.StateEstimator.StateEstimator
                        .TargetSpecifications.f2.StateTransitionModel
                        .PropVelocityVariance[0],
                   &SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                        .TargetSpecifications.f2.StateTransitionModel
                        .PropVelocityVariance[0],
                   9U * sizeof(real_T));
            memcpy(&SD->u4.f12.r3.StateEstimator.StateEstimator
                        .TargetSpecifications.f2.StateTransitionModel
                        .PropAccelerationVariance[0],
                   &SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                        .TargetSpecifications.f2.StateTransitionModel
                        .PropAccelerationVariance[0],
                   9U * sizeof(real_T));
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f2
                .SurvivalModel =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f2.SurvivalModel;
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f2
                .IsLockedDataType[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f2.IsLockedDataType[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f2
                .IsLockedDataType[1] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f2.IsLockedDataType[1];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f3
                .StateTransitionModel.PropVelocityMean[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f3.StateTransitionModel
                    .PropVelocityMean[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f3
                .StateTransitionModel.PropVelocityMean[1] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f3.StateTransitionModel
                    .PropVelocityMean[1];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f3
                .StateTransitionModel.PropVelocityMean[2] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f3.StateTransitionModel
                    .PropVelocityMean[2];
            memcpy(&SD->u4.f12.r3.StateEstimator.StateEstimator
                        .TargetSpecifications.f3.StateTransitionModel
                        .PropVelocityVariance[0],
                   &SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                        .TargetSpecifications.f3.StateTransitionModel
                        .PropVelocityVariance[0],
                   9U * sizeof(real_T));
            memcpy(&SD->u4.f12.r3.StateEstimator.StateEstimator
                        .TargetSpecifications.f3.StateTransitionModel
                        .PropAccelerationVariance[0],
                   &SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                        .TargetSpecifications.f3.StateTransitionModel
                        .PropAccelerationVariance[0],
                   9U * sizeof(real_T));
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f3
                .SurvivalModel =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f3.SurvivalModel;
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f3
                .IsLockedDataType[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f3.IsLockedDataType[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.TargetSpecifications.f3
                .IsLockedDataType[1] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .TargetSpecifications.f3.IsLockedDataType[1];
            SD->u4.f12.r3.StateEstimator.StateEstimator
                .SensorSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .SensorSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.DeletionThreshold =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator
                    .DeletionThreshold;
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f1
                .TargetSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f1.TargetSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f1
                .SensorSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f1.SensorSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f1
                .TrackingFilter =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f1.TrackingFilter;
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f2
                .TargetSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f2.TargetSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f2
                .SensorSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f2.SensorSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f2
                .TrackingFilter =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f2.TrackingFilter;
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f3
                .TargetSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f3.TargetSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f3
                .SensorSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f3.SensorSpecifications[0];
            SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f3
                .TrackingFilter =
                SD->u4.f12.r2.Estimator.StateEstimator.StateEstimator.Estimators
                    .f3.TrackingFilter;
            SD->u4.f12.r3.StateEstimator.ExistenceEstimator
                .SensorSpecifications[0] =
                SD->u4.f12.r2.Estimator.StateEstimator.ExistenceEstimator
                    .SensorSpecifications[0];
            SD->u4.f12.r3.StateEstimator.ExistenceEstimator
                .DetectionProbability =
                SD->u4.f12.r2.Estimator.StateEstimator.ExistenceEstimator
                    .DetectionProbability;
            SD->u4.f12.r3.StateEstimator.ExistenceEstimator
                .SurvivalProbability =
                SD->u4.f12.r2.Estimator.StateEstimator.ExistenceEstimator
                    .SurvivalProbability;
            b_st.site = &snb_emlrtRSI;
            c_TrackEstimator_updateEstimato(
                SD, &b_st, &SD->u4.f12.r3, modelData_data[0].LookTime.data,
                modelData_data[0].LookTime.size,
                modelData_data[0].LookAzimuth.data,
                modelData_data[0].LookAzimuth.size,
                modelData_data[0].LookElevation.data,
                modelData_data[0].LookElevation.size,
                modelData_data[0].DetectionTime.data,
                modelData_data[0].DetectionTime.size,
                modelData_data[0].AzimuthNoise.data,
                modelData_data[0].AzimuthNoise.size,
                modelData_data[0].ElevationNoise.data,
                modelData_data[0].ElevationNoise.size,
                modelData_data[0].RangeNoise.data,
                modelData_data[0].RangeNoise.size,
                modelData_data[0].RangeRateNoise.data,
                modelData_data[0].RangeRateNoise.size);
            c_t44_Estimator_StateEstimator_ =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f1
                    .TargetSpecifications[0];
            SD->u4.f12.c_t44_Estimator_StateEstimator_ =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f1
                    .SensorSpecifications[0];
            c_estimator_StateEstimator_Stat =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f1
                    .TrackingFilter;
            d_t44_Estimator_StateEstimator_ =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f2
                    .TargetSpecifications[0];
            SD->u4.f12.d_t44_Estimator_StateEstimator_ =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f2
                    .SensorSpecifications[0];
            d_estimator_StateEstimator_Stat =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f2
                    .TrackingFilter;
            e_t44_Estimator_StateEstimator_ =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f3
                    .TargetSpecifications[0];
            SD->u4.f12.e_t44_Estimator_StateEstimator_ =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f3
                    .SensorSpecifications[0];
            e_estimator_StateEstimator_Stat =
                SD->u4.f12.r3.StateEstimator.StateEstimator.Estimators.f3
                    .TrackingFilter;
            SD->u4.f12.t98_Estimator_StateEstimator.ExistenceEstimator
                .SensorSpecifications[0] =
                SD->u4.f12.r3.StateEstimator.ExistenceEstimator
                    .SensorSpecifications[0];
          }
          for (i = 0; i < k; i++) {
            if (modelData->size[0] == 1) {
              if (i + 1 > k) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, k, &d_emlrtBCI, &st);
              }
              if (i + 1 > loop_ub) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &db_emlrtBCI,
                                              &st);
              }
              if (i + 1 > time_size[1]) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, time_size[1],
                                              &eb_emlrtBCI, &st);
              }
              b_st.site = &tnb_emlrtRSI;
              initializeTrack(&b_st, &c_t44_Estimator_StateEstimator_,
                              &SD->u4.f12.c_t44_Estimator_StateEstimator_,
                              c_estimator_StateEstimator_Stat,
                              &d_t44_Estimator_StateEstimator_,
                              &SD->u4.f12.d_t44_Estimator_StateEstimator_,
                              d_estimator_StateEstimator_Stat,
                              &e_t44_Estimator_StateEstimator_,
                              &SD->u4.f12.e_t44_Estimator_StateEstimator_,
                              e_estimator_StateEstimator_Stat,
                              &SD->u4.f12.t98_Estimator_StateEstimator
                                   .ExistenceEstimator.SensorSpecifications[0],
                              &SD->u4.f12.newTracks_data[i], &z_data[4 * i],
                              time_data[i]);
            } else {
              b_st.site = &unb_emlrtRSI;
              if (i + 1 > loop_ub) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &e_emlrtBCI,
                                              &b_st);
              }
              if (i + 1 > k) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, k, &f_emlrtBCI, &b_st);
              }
              if (i + 1 > time_size[1]) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, time_size[1],
                                              &g_emlrtBCI, &b_st);
              }
              if (i + 1 > modelData->size[0]) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, modelData->size[0],
                                              &h_emlrtBCI, &b_st);
              }
              c_st.site = &rob_emlrtRSI;
              d_st.site = &mq_emlrtRSI;
              f_st.site = &nq_emlrtRSI;
              g_st.site = &pq_emlrtRSI;
              SD->u4.f12.val = SD->u4.f12.c_t44_Estimator_StateEstimator_;
              h_st.site = &qq_emlrtRSI;
              c_AerospaceMonostaticRadar_upda(
                  &h_st, &SD->u4.f12.val, modelData_data[i].LookTime.data,
                  modelData_data[i].LookTime.size,
                  modelData_data[i].LookAzimuth.data,
                  modelData_data[i].LookAzimuth.size,
                  modelData_data[i].LookElevation.data,
                  modelData_data[i].LookElevation.size,
                  modelData_data[i].DetectionTime.data,
                  modelData_data[i].DetectionTime.size,
                  modelData_data[i].AzimuthNoise.data,
                  modelData_data[i].AzimuthNoise.size,
                  modelData_data[i].ElevationNoise.data,
                  modelData_data[i].ElevationNoise.size,
                  modelData_data[i].RangeNoise.data,
                  modelData_data[i].RangeNoise.size,
                  modelData_data[i].RangeRateNoise.data,
                  modelData_data[i].RangeRateNoise.size);
              g_st.site = &pq_emlrtRSI;
              SD->u4.f12.b_val = SD->u4.f12.d_t44_Estimator_StateEstimator_;
              h_st.site = &qq_emlrtRSI;
              c_AerospaceMonostaticRadar_upda(
                  &h_st, &SD->u4.f12.b_val, modelData_data[i].LookTime.data,
                  modelData_data[i].LookTime.size,
                  modelData_data[i].LookAzimuth.data,
                  modelData_data[i].LookAzimuth.size,
                  modelData_data[i].LookElevation.data,
                  modelData_data[i].LookElevation.size,
                  modelData_data[i].DetectionTime.data,
                  modelData_data[i].DetectionTime.size,
                  modelData_data[i].AzimuthNoise.data,
                  modelData_data[i].AzimuthNoise.size,
                  modelData_data[i].ElevationNoise.data,
                  modelData_data[i].ElevationNoise.size,
                  modelData_data[i].RangeNoise.data,
                  modelData_data[i].RangeNoise.size,
                  modelData_data[i].RangeRateNoise.data,
                  modelData_data[i].RangeRateNoise.size);
              g_st.site = &pq_emlrtRSI;
              SD->u4.f12.c_val = SD->u4.f12.e_t44_Estimator_StateEstimator_;
              h_st.site = &qq_emlrtRSI;
              c_AerospaceMonostaticRadar_upda(
                  &h_st, &SD->u4.f12.c_val, modelData_data[i].LookTime.data,
                  modelData_data[i].LookTime.size,
                  modelData_data[i].LookAzimuth.data,
                  modelData_data[i].LookAzimuth.size,
                  modelData_data[i].LookElevation.data,
                  modelData_data[i].LookElevation.size,
                  modelData_data[i].DetectionTime.data,
                  modelData_data[i].DetectionTime.size,
                  modelData_data[i].AzimuthNoise.data,
                  modelData_data[i].AzimuthNoise.size,
                  modelData_data[i].ElevationNoise.data,
                  modelData_data[i].ElevationNoise.size,
                  modelData_data[i].RangeNoise.data,
                  modelData_data[i].RangeNoise.size,
                  modelData_data[i].RangeRateNoise.data,
                  modelData_data[i].RangeRateNoise.size);
              f_st.site = &oq_emlrtRSI;
              SD->u4.f12.estimator_SensorSpecifications =
                  SD->u4.f12.t98_Estimator_StateEstimator.ExistenceEstimator
                      .SensorSpecifications[0];
              g_st.site = &dt_emlrtRSI;
              c_AerospaceMonostaticRadar_upda(
                  &g_st, &SD->u4.f12.estimator_SensorSpecifications,
                  modelData_data[i].LookTime.data,
                  modelData_data[i].LookTime.size,
                  modelData_data[i].LookAzimuth.data,
                  modelData_data[i].LookAzimuth.size,
                  modelData_data[i].LookElevation.data,
                  modelData_data[i].LookElevation.size,
                  modelData_data[i].DetectionTime.data,
                  modelData_data[i].DetectionTime.size,
                  modelData_data[i].AzimuthNoise.data,
                  modelData_data[i].AzimuthNoise.size,
                  modelData_data[i].ElevationNoise.data,
                  modelData_data[i].ElevationNoise.size,
                  modelData_data[i].RangeNoise.data,
                  modelData_data[i].RangeNoise.size,
                  modelData_data[i].RangeRateNoise.data,
                  modelData_data[i].RangeRateNoise.size);
              if (i + 1 > loop_ub) {
                emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &gb_emlrtBCI,
                                              &b_st);
              }
              c_st.site = &sob_emlrtRSI;
              initializeTrack(
                  &c_st, &c_t44_Estimator_StateEstimator_, &SD->u4.f12.val,
                  c_estimator_StateEstimator_Stat,
                  &d_t44_Estimator_StateEstimator_, &SD->u4.f12.b_val,
                  d_estimator_StateEstimator_Stat,
                  &e_t44_Estimator_StateEstimator_, &SD->u4.f12.c_val,
                  e_estimator_StateEstimator_Stat,
                  &SD->u4.f12.estimator_SensorSpecifications,
                  &SD->u4.f12.newTracks_data[i], &z_data[4 * i], time_data[i]);
            }
            if (i + 1 > loop_ub) {
              emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &fb_emlrtBCI,
                                            &st);
            }
            b_i = (real_T)u + ((real_T)i + 1.0);
            if (b_i < 4.294967296E+9) {
              u1 = (uint32_T)b_i;
            } else {
              u1 = MAX_uint32_T;
            }
            SD->u4.f12.newTracks_data[i].TrackID = u1;
          }
          b_i = (real_T)u + (real_T)z_size[1];
          if (b_i < 4.294967296E+9) {
            u = (uint32_T)b_i;
          } else {
            u = MAX_uint32_T;
          }
          c_emxCopyStruct_fusion_tracker_(sp, &SD->u4.f12.b_obj,
                                          &b_tracker->TrackListManager,
                                          &md_emlrtRTEI);
          k = SD->u4.f12.b_obj.InternalTrackList->size[0];
          SD->u4.f12.b_obj.InternalTrackList->size[0] =
              SD->u4.f12.obj.InternalTrackList->size[0] + z_size[1];
          emxEnsureCapacity_struct_T(sp, SD->u4.f12.b_obj.InternalTrackList, k,
                                     &dd_emlrtRTEI);
          k = SD->u4.f12.obj.InternalTrackList->size[0];
          for (i = 0; i < k; i++) {
            SD->u4.f12.b_obj.InternalTrackList->data[i] =
                SD->u4.f12.obj.InternalTrackList->data[i];
          }
          for (i = 0; i < loop_ub; i++) {
            SD->u4.f12.b_obj.InternalTrackList
                ->data[i + SD->u4.f12.obj.InternalTrackList->size[0]] =
                SD->u4.f12.newTracks_data[i];
          }
          c_emxCopyStruct_fusion_tracker_(sp, &b_tracker->TrackListManager,
                                          &SD->u4.f12.b_obj, &md_emlrtRTEI);
          b_tracker->LastTrackID = u;
        }
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  emxFree_struct_T1(sp, &modelData);
  c_emxFreeStruct_fusion_tracker_(sp, &SD->u4.f12.b_obj);
  emxFree_real_T(sp, &assignment);
  emxFree_boolean_T(sp, &a__1);
  c_emxCopyStruct_fusion_tracker_(sp, &SD->u4.f12.obj,
                                  &b_tracker->TrackListManager, &xc_emlrtRTEI);
  emxInit_struct_T(sp, &val, &xc_emlrtRTEI, true);
  loop_ub = SD->u4.f12.obj.InternalTrackList->size[0];
  k = val->size[0];
  val->size[0] = SD->u4.f12.obj.InternalTrackList->size[0];
  emxEnsureCapacity_struct_T(sp, val, k, &yc_emlrtRTEI);
  val_data = val->data;
  for (i = 0; i < loop_ub; i++) {
    val_data[i] = SD->u4.f12.obj.InternalTrackList->data[i];
  }
  st.site = &gn_emlrtRSI;
  SD->u4.f12.r = b_tracker->TrackMaintenance;
  b_i = SD->u4.f12.r.DeletionThreshold;
  for (i = 0; i < loop_ub; i++) {
    if (i + 1 > val->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, val->size[0], &b_emlrtBCI, &st);
    }
    if (val_data[i].IsConfirmed || (val_data[i].ExistenceProbability >=
                                    SD->u4.f12.r.ConfirmationThreshold)) {
      if (i + 1 > val->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, val->size[0], &s_emlrtBCI, &st);
      }
      val_data[i].IsConfirmed = true;
    }
  }
  emxInit_boolean_T(&st, &toDeleteTracks, 2, &ad_emlrtRTEI, true);
  k = toDeleteTracks->size[0] * toDeleteTracks->size[1];
  toDeleteTracks->size[0] = 1;
  loop_ub = val->size[0];
  toDeleteTracks->size[1] = val->size[0];
  emxEnsureCapacity_boolean_T(&st, toDeleteTracks, k, &ad_emlrtRTEI);
  toDeleteTracks_data = toDeleteTracks->data;
  for (i = 0; i < loop_ub; i++) {
    toDeleteTracks_data[i] = false;
  }
  k = val->size[0];
  if (val->size[0] < 1600) {
    for (d_i = 0; d_i < loop_ub; d_i++) {
      if (d_i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, loop_ub, &c_emlrtBCI, &st);
      }
      if (d_i + 1 > toDeleteTracks->size[1]) {
        emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, toDeleteTracks->size[1],
                                      &t_emlrtBCI, &st);
      }
      toDeleteTracks_data[d_i] = (val_data[d_i].ExistenceProbability < b_i);
    }
  } else {
    emlrtEnterParallelRegion(&st, omp_in_parallel());
    emlrtPushJmpBuf(&st, &emlrtJBStack);
    JIPDATracker_stepImpl_numThreads = emlrtAllocRegionTLSs(
        st.tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(JIPDATracker_stepImpl_numThreads) private(    \
        emlrtJBEnviron, i_st) firstprivate(emlrtHadParallelError, st)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        i_st.prev = &st;
        i_st.tls = emlrtAllocTLS(&st, omp_get_thread_num());
        i_st.site = NULL;
        emlrtSetJmpBuf(&i_st, &emlrtJBEnviron);
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (d_i = 0; d_i < k; d_i++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          if (d_i + 1 > val->size[0]) {
            emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, val->size[0], &c_emlrtBCI,
                                          &i_st);
          }
          if (d_i + 1 > toDeleteTracks->size[1]) {
            emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, toDeleteTracks->size[1],
                                          &t_emlrtBCI, &i_st);
          }
          toDeleteTracks_data[d_i] = (val_data[d_i].ExistenceProbability < b_i);
        } else {
          emlrtHadParallelError = true;
        }
      }
    }
    emlrtPopJmpBuf(&st, &emlrtJBStack);
    emlrtExitParallelRegion(&st, omp_in_parallel());
  }
  loop_ub = toDeleteTracks->size[1];
  n = 0;
  for (i = 0; i < loop_ub; i++) {
    if (!toDeleteTracks_data[i]) {
      n++;
    }
  }
  k = 0;
  for (i = 0; i < loop_ub; i++) {
    if (!toDeleteTracks_data[i]) {
      if (i > val->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, val->size[0] - 1, &x_emlrtBCI, &st);
      }
      val_data[k] = val_data[i];
      k++;
    }
  }
  emxFree_boolean_T(&st, &toDeleteTracks);
  k = val->size[0];
  val->size[0] = n;
  emxEnsureCapacity_struct_T(&st, val, k, &bd_emlrtRTEI);
  val_data = val->data;
  c_emxCopyStruct_fusion_tracker_(sp, &SD->u4.f12.obj,
                                  &b_tracker->TrackListManager, &cd_emlrtRTEI);
  k = SD->u4.f12.obj.InternalTrackList->size[0];
  SD->u4.f12.obj.InternalTrackList->size[0] = n;
  emxEnsureCapacity_struct_T(sp, SD->u4.f12.obj.InternalTrackList, k,
                             &dd_emlrtRTEI);
  for (i = 0; i < n; i++) {
    SD->u4.f12.obj.InternalTrackList->data[i] = val_data[i];
  }
  emxFree_struct_T(sp, &val);
  c_emxCopyStruct_fusion_tracker_(sp, &b_tracker->TrackListManager,
                                  &SD->u4.f12.obj, &cd_emlrtRTEI);
  c_emxCopyStruct_fusion_tracker_(sp, &SD->u4.f12.obj,
                                  &b_tracker->TrackListManager, &ed_emlrtRTEI);
  st.site = &hn_emlrtRSI;
  SD->u4.f12.r1 = b_tracker->Outputter;
  b_st.site = &tob_emlrtRSI;
  s1.TrackLogicState = SD->u4.f12.r1.SampleTrack.pTrackLogicState;
  s1.TrackID = SD->u4.f12.r1.SampleTrack.TrackID;
  s1.BranchID = SD->u4.f12.r1.SampleTrack.BranchID;
  s1.SourceIndex = SD->u4.f12.r1.SampleTrack.SourceIndex;
  s1.UpdateTime = SD->u4.f12.r1.SampleTrack.pUpdateTime;
  s1.Age = SD->u4.f12.r1.SampleTrack.Age;
  for (i = 0; i < 6; i++) {
    s1.State[i] = SD->u4.f12.r1.SampleTrack.pState[i];
  }
  memcpy(&s1.StateCovariance[0], &SD->u4.f12.r1.SampleTrack.pStateCovariance[0],
         36U * sizeof(real_T));
  s1.ObjectClassID = SD->u4.f12.r1.SampleTrack.ObjectClassID;
  s1.ObjectClassProbabilities[0] =
      SD->u4.f12.r1.SampleTrack.ObjectClassProbabilities[0];
  s1.ObjectClassProbabilities[1] =
      SD->u4.f12.r1.SampleTrack.ObjectClassProbabilities[1];
  s1.ObjectClassProbabilities[2] =
      SD->u4.f12.r1.SampleTrack.ObjectClassProbabilities[2];
  for (i = 0; i < 10; i++) {
    s1.TrackLogic[i] = cv1[i];
  }
  s1.IsConfirmed = SD->u4.f12.r1.SampleTrack.IsConfirmed;
  s1.IsCoasted = SD->u4.f12.r1.SampleTrack.IsCoasted;
  s1.IsSelfReported = SD->u4.f12.r1.SampleTrack.IsSelfReported;
  c_st.site = &vob_emlrtRSI;
  d_st.site = &hq_emlrtRSI;
  emxInit_struct1_T(&c_st, &tracksObjectTrack, &fd_emlrtRTEI);
  n = SD->u4.f12.obj.InternalTrackList->size[0];
  k = tracksObjectTrack->size[0];
  tracksObjectTrack->size[0] = SD->u4.f12.obj.InternalTrackList->size[0];
  emxEnsureCapacity_struct1_T(&c_st, tracksObjectTrack, k, &fd_emlrtRTEI);
  tracksObjectTrack_data = tracksObjectTrack->data;
  for (i = 0; i < n; i++) {
    tracksObjectTrack_data[i] = s1;
  }
  for (e_i = 0; e_i < n; e_i++) {
    if (e_i + 1 > n) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, n, &k_emlrtBCI, &b_st);
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &ab_emlrtBCI, &b_st);
    }
    loop_ub = IPDAEstimator_toObjectTrack(
        SD->u4.f12.obj.InternalTrackList->data[e_i].Hypothesis,
        SD->u4.f12.obj.InternalTrackList->data[e_i].LogWeights,
        SD->u4.f12.obj.InternalTrackList->data[e_i].ExistenceProbability,
        statePdf_State, statePdf_StateCovariance,
        c_statePdf_ObjectClassProbabili, expl_temp, &b_i);
    for (i = 0; i < 6; i++) {
      if (e_i + 1 > tracksObjectTrack->size[0]) {
        emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                      &bb_emlrtBCI, &b_st);
      }
      tracksObjectTrack_data[e_i].State[i] = statePdf_State[i];
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    for (i = 0; i < 36; i++) {
      tracksObjectTrack_data[e_i].StateCovariance[i] =
          statePdf_StateCovariance[i];
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].ObjectClassProbabilities[0] =
        c_statePdf_ObjectClassProbabili[0];
    tracksObjectTrack_data[e_i].ObjectClassProbabilities[1] =
        c_statePdf_ObjectClassProbabili[1];
    tracksObjectTrack_data[e_i].ObjectClassProbabilities[2] =
        c_statePdf_ObjectClassProbabili[2];
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].ObjectClassID = loop_ub;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    for (i = 0; i < 10; i++) {
      tracksObjectTrack_data[e_i].TrackLogic[i] = cv1[i];
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].TrackLogicState = b_i;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].Age =
        SD->u4.f12.obj.InternalTrackList->data[e_i].Age;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].IsConfirmed =
        SD->u4.f12.obj.InternalTrackList->data[e_i].IsConfirmed;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].IsCoasted =
        SD->u4.f12.obj.InternalTrackList->data[e_i].IsCoasted;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].TrackID =
        SD->u4.f12.obj.InternalTrackList->data[e_i].TrackID;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].UpdateTime =
        SD->u4.f12.obj.InternalTrackList->data[e_i].Time;
  }
  c_emxFreeStruct_fusion_tracker_(&b_st, &SD->u4.f12.obj);
  b_st.site = &uob_emlrtRSI;
  n = tracksObjectTrack->size[0];
  if (tracksObjectTrack->size[0] == 0) {
    k = 0;
  } else {
    k = tracksObjectTrack->size[0];
    c_st.site = &xob_emlrtRSI;
    if (tracksObjectTrack->size[0] > 2147483646) {
      d_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
  }
  emxInit_boolean_T(&b_st, &isConfirmed, 2, &od_emlrtRTEI, true);
  loop_ub = isConfirmed->size[0] * isConfirmed->size[1];
  isConfirmed->size[0] = 1;
  isConfirmed->size[1] = k;
  emxEnsureCapacity_boolean_T(&b_st, isConfirmed, loop_ub, &gd_emlrtRTEI);
  toDeleteTracks_data = isConfirmed->data;
  c_st.site = &wob_emlrtRSI;
  for (i = 0; i < n; i++) {
    toDeleteTracks_data[i] = tracksObjectTrack_data[i].IsConfirmed;
  }
  n = isConfirmed->size[1];
  k = 0;
  for (i = 0; i < n; i++) {
    if (toDeleteTracks_data[i]) {
      k++;
    }
  }
  loop_ub = varargout_1->size[0];
  varargout_1->size[0] = k;
  emxEnsureCapacity_struct1_T(&st, varargout_1, loop_ub, &hd_emlrtRTEI);
  varargout_1_data = varargout_1->data;
  loop_ub = 0;
  for (i = 0; i < n; i++) {
    if (toDeleteTracks_data[i]) {
      if (i > tracksObjectTrack->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(i, 0, tracksObjectTrack->size[0] - 1,
                                      &cb_emlrtBCI, &st);
      }
      varargout_1_data[loop_ub] = tracksObjectTrack_data[i];
      loop_ub++;
    }
  }
  for (i = 0; i < n; i++) {
    if ((!toDeleteTracks_data[i]) && (i > tracksObjectTrack->size[0] - 1)) {
      emlrtDynamicBoundsCheckR2012b(i, 0, tracksObjectTrack->size[0] - 1,
                                    &l_emlrtBCI, &st);
    }
  }
  emxFree_boolean_T(&st, &isConfirmed);
  emxFree_struct1_T(&st, &tracksObjectTrack);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_JIPDATracker_updateSensorSpec(trackingAlgorithmStackData *SD,
                                     const emlrtStack *sp,
                                     fusion_tracker_JIPDATracker *obj)
{
  c_fusion_tracker_internal_compo *b_obj;
  c_fusion_tracker_sensorspecs_Ae val_idx_0;
  g_fusion_tracker_internal_compo c_obj;
  real_T c_obj_TargetSpecifications_f1_S[9];
  real_T c_obj_TargetSpecifications_f2_S[9];
  real_T c_obj_TargetSpecifications_f3_S[9];
  real_T d_obj_TargetSpecifications_f1_S[9];
  real_T d_obj_TargetSpecifications_f2_S[9];
  real_T d_obj_TargetSpecifications_f3_S[9];
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj->Scheduler;
  obj->Scheduler = b_obj;
  val_idx_0 = obj->SensorSpecifications[0];
  SD->u1.f3.e_obj = obj->Initiator[0];
  SD->u1.f3.e_obj.SensorSpecifications[0] = val_idx_0;
  memcpy(&c_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.e_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.e_obj.Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] = val_idx_0;
  obj->Initiator[0] = SD->u1.f3.e_obj;
  val_idx_0 = obj->SensorSpecifications[0];
  SD->u1.f3.b_obj = obj->Assigner[0];
  SD->u1.f3.b_obj.SensorSpecifications[0] = val_idx_0;
  memcpy(&c_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.b_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.b_obj.Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] = val_idx_0;
  obj->Assigner[0] = SD->u1.f3.b_obj;
  val_idx_0 = obj->SensorSpecifications[0];
  SD->u1.f3.d_obj = obj->Updater[0];
  SD->u1.f3.d_obj.SensorSpecifications[0] = val_idx_0;
  memcpy(&c_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.d_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.d_obj.Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] = val_idx_0;
  obj->Updater[0] = SD->u1.f3.d_obj;
  c_emxInitStruct_fusion_tracker_(sp, &c_obj, &qc_emlrtRTEI, true);
  c_emxCopyStruct_fusion_tracker_(sp, &c_obj, &obj->TrackListManager,
                                  &qc_emlrtRTEI);
  c_obj.SensorSpecifications[0] = obj->SensorSpecifications[0];
  c_emxCopyStruct_fusion_tracker_(sp, &obj->TrackListManager, &c_obj,
                                  &rc_emlrtRTEI);
  c_emxFreeStruct_fusion_tracker_(sp, &c_obj);
  SD->u1.f3.c_obj = obj->TrackMaintenance;
  val_idx_0 = obj->SensorSpecifications[0];
  SD->u1.f3.c_obj.SensorSpecifications[0] = val_idx_0;
  memcpy(&c_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.c_obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.c_obj.Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] = val_idx_0;
  obj->TrackMaintenance = SD->u1.f3.c_obj;
  SD->u1.f3.obj = obj->Outputter;
  val_idx_0 = obj->SensorSpecifications[0];
  SD->u1.f3.obj.SensorSpecifications[0] = val_idx_0;
  memcpy(&c_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f1_S[0],
         &SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f2_S[0],
         &SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&c_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         9U * sizeof(real_T));
  memcpy(&d_obj_TargetSpecifications_f3_S[0],
         &SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         9U * sizeof(real_T));
  memcpy(&SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f1.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f1_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f2.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f2_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropVelocityVariance[0],
         &c_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  memcpy(&SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
              .TargetSpecifications.f3.StateTransitionModel
              .PropAccelerationVariance[0],
         &d_obj_TargetSpecifications_f3_S[0], 9U * sizeof(real_T));
  SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator.Estimators.f1
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator.Estimators.f2
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.obj.Estimator.StateEstimator.StateEstimator.Estimators.f3
      .SensorSpecifications[0] = val_idx_0;
  SD->u1.f3.obj.Estimator.StateEstimator.ExistenceEstimator
      .SensorSpecifications[0] = val_idx_0;
  obj->Outputter = SD->u1.f3.obj;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (JIPDATracker.c) */
