/*
 * JIPDATracker.c
 *
 * Code generation for function 'JIPDATracker'
 *
 */

/* Include files */
#include "JIPDATracker.h"
#include "AerospaceMonostaticRadar.h"
#include "JIPDATrackAssigner.h"
#include "JIPDATrackInitiator.h"
#include "JIPDATrackUpdater.h"
#include "TrackEstimator1.h"
#include "eml_int_forloop_overflow_check.h"
#include "ifWhileCond.h"
#include "indexShapeCheck.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo jm_emlrtRSI = {
    189,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo km_emlrtRSI = {
    197,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo lm_emlrtRSI = {
    200,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo mm_emlrtRSI = {
    217,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo nm_emlrtRSI = {
    220,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo om_emlrtRSI = {
    222,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo pm_emlrtRSI = {
    225,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo qm_emlrtRSI = {
    228,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo rm_emlrtRSI = {
    234,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo sm_emlrtRSI = {
    237,                     /* lineNo */
    "JIPDATracker/stepImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo tm_emlrtRSI = {
    51,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo um_emlrtRSI = {
    56,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo vm_emlrtRSI = {
    57,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo wm_emlrtRSI = {
    59,                             /* lineNo */
    "SensorDataScheduler/schedule", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo kn_emlrtRSI = {
    17,            /* lineNo */
    "uniquetolcg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m" /* pathName */
};

static emlrtRSInfo mo_emlrtRSI = {
    42,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\sort.m" /* pathName
                                                                         */
};

static emlrtRSInfo no_emlrtRSI = {
    65,                         /* lineNo */
    "SensorDataScheduler/next", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo oo_emlrtRSI = {
    67,                         /* lineNo */
    "SensorDataScheduler/next", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo qo_emlrtRSI = {
    280,                                   /* lineNo */
    "AerospaceMonostaticRadar/selectTime", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo xlb_emlrtRSI = {
    50,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo ylb_emlrtRSI = {
    51,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo amb_emlrtRSI = {
    55,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo bmb_emlrtRSI = {
    61,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo cmb_emlrtRSI = {
    67,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo dmb_emlrtRSI = {
    73,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo emb_emlrtRSI = {
    75,                             /* lineNo */
    "JIPDATrackInitiator/initiate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo xmb_emlrtRSI = {
    131,                             /* lineNo */
    "updateModelAndInitializeTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo ymb_emlrtRSI = {
    132,                             /* lineNo */
    "updateModelAndInitializeTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo anb_emlrtRSI = {
    44,                            /* lineNo */
    "ObjectTrackOutputter/output", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo bnb_emlrtRSI = {
    45,                            /* lineNo */
    "ObjectTrackOutputter/output", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo cnb_emlrtRSI = {
    69,                                             /* lineNo */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo dnb_emlrtRSI = {
    69,                  /* lineNo */
    "horzcatStructList", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\horzcatStructList.m" /* pathName */
};

static emlrtRSInfo enb_emlrtRSI = {
    25,                  /* lineNo */
    "horzcatStructList", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\horzcatStructList.m" /* pathName */
};

static emlrtBCInfo emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    35,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    44,                              /* lineNo */
    50,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    50,                              /* lineNo */
    67,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    73,                             /* lineNo */
    81,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    87,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    95,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    104,                            /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    118,                            /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    27,            /* lineNo */
    43,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    27,            /* lineNo */
    29,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtECInfo emlrtECI = {
    -1,            /* nDims */
    19,            /* lineNo */
    9,             /* colNo */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m" /* pName */
};

static emlrtBCInfo k_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    75,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    47,                            /* lineNo */
    13,                            /* colNo */
    "",                            /* aName */
    "ObjectTrackOutputter/output", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    19,            /* lineNo */
    19,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    35,            /* lineNo */
    21,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    36,            /* lineNo */
    17,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    33,            /* lineNo */
    16,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    28,            /* lineNo */
    37,            /* colNo */
    "",            /* aName */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m", /* pName */
    0                          /* checkKind */
};

static emlrtDCInfo emlrtDCI = {
    65,                         /* lineNo */
    53,                         /* colNo */
    "SensorDataScheduler/next", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m", /* pName */
    1     /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    65,                         /* lineNo */
    53,                         /* colNo */
    "",                         /* aName */
    "SensorDataScheduler/next", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    45,                              /* lineNo */
    74,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    50,                              /* lineNo */
    32,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    265,                                   /* lineNo */
    46,                                    /* colNo */
    "",                                    /* aName */
    "AerospaceMonostaticRadar/selectTime", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    266,                                   /* lineNo */
    52,                                    /* colNo */
    "",                                    /* aName */
    "AerospaceMonostaticRadar/selectTime", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    268,                                   /* lineNo */
    60,                                    /* colNo */
    "",                                    /* aName */
    "AerospaceMonostaticRadar/selectTime", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = {
    -1,                              /* iFirst */
    -1,                              /* iLast */
    52,                              /* lineNo */
    13,                              /* colNo */
    "",                              /* aName */
    "JIPDATrackMaintainer/maintain", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    97,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    17,                                             /* colNo */
    "",                                             /* aName */
    "ObjectTrackOutputter/trackListToObjectTracks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    46,                            /* lineNo */
    13,                            /* colNo */
    "",                            /* aName */
    "ObjectTrackOutputter/output", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    73,                             /* lineNo */
    73,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    73,                             /* lineNo */
    90,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    77,                             /* lineNo */
    27,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    75,                             /* lineNo */
    31,                             /* colNo */
    "",                             /* aName */
    "JIPDATrackInitiator/initiate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m", /* pName */
    0     /* checkKind */
};

static emlrtRTEInfo uc_emlrtRTEI = {
    57,                    /* lineNo */
    65,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pName */
};

static emlrtRTEInfo vc_emlrtRTEI = {
    57,                    /* lineNo */
    17,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pName */
};

static emlrtRTEInfo wc_emlrtRTEI = {
    23,            /* lineNo */
    9,             /* colNo */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m" /* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = {
    187,            /* lineNo */
    30,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo yc_emlrtRTEI = {
    59,                    /* lineNo */
    13,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pName */
};

static emlrtRTEInfo ad_emlrtRTEI = {
    234,            /* lineNo */
    84,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo bd_emlrtRTEI = {
    31,                 /* lineNo */
    13,                 /* colNo */
    "TrackListManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\TrackListManager.m" /* pName */
};

static emlrtRTEInfo cd_emlrtRTEI = {
    48,                     /* lineNo */
    13,                     /* colNo */
    "JIPDATrackMaintainer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pName */
};

static emlrtRTEInfo dd_emlrtRTEI = {
    52,                     /* lineNo */
    13,                     /* colNo */
    "JIPDATrackMaintainer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pName */
};

static emlrtRTEInfo ed_emlrtRTEI = {
    234,            /* lineNo */
    13,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo fd_emlrtRTEI = {
    36,                 /* lineNo */
    13,                 /* colNo */
    "TrackListManager", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\TrackListManager.m" /* pName */
};

static emlrtRTEInfo gd_emlrtRTEI = {
    237,            /* lineNo */
    63,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo hd_emlrtRTEI = {
    44,                     /* lineNo */
    13,                     /* colNo */
    "ObjectTrackOutputter", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pName */
};

static emlrtRTEInfo id_emlrtRTEI = {
    67,                  /* lineNo */
    10,                  /* colNo */
    "horzcatStructList", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\horzcatStructList.m" /* pName */
};

static emlrtRTEInfo jd_emlrtRTEI = {
    46,                     /* lineNo */
    13,                     /* colNo */
    "ObjectTrackOutputter", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pName */
};

static emlrtRTEInfo kd_emlrtRTEI = {
    222,            /* lineNo */
    95,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo ld_emlrtRTEI = {
    225,            /* lineNo */
    88,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo md_emlrtRTEI = {
    225,            /* lineNo */
    25,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo nd_emlrtRTEI = {
    228,            /* lineNo */
    115,            /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo od_emlrtRTEI = {
    228,            /* lineNo */
    26,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo pd_emlrtRTEI = {
    52,                    /* lineNo */
    13,                    /* colNo */
    "SensorDataScheduler", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pName */
};

static emlrtRTEInfo qd_emlrtRTEI = {
    45,                     /* lineNo */
    13,                     /* colNo */
    "ObjectTrackOutputter", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pName */
};

static emlrtRTEInfo rd_emlrtRTEI = {
    1,             /* lineNo */
    16,            /* colNo */
    "uniquetolcg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\uniquetolcg.m" /* pName */
};

/* Function Definitions */
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
  static const char_T logicType[10] = {'I', 'n', 't', 'e', 'g',
                                       'r', 'a', 't', 'e', 'd'};
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  b_emxArray_struct_T *modelData;
  b_struct_T *modelData_data;
  c_fusion_tracker_internal_compo *obj;
  c_fusion_tracker_targetspecs_Pa c_t32_Estimator_StateEstimator_;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
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
  emxArray_real_T_1x100 jnb_emlrtRSI;
  emxArray_struct1_T *tracksObjectTrack;
  emxArray_struct_T *val;
  struct0_T multiSensorData;
  struct0_T unassignedSensorData_idx_0;
  struct1_T s1;
  struct1_T *tracksObjectTrack_data;
  struct1_T *varargout_1_data;
  struct_T newTracks_data[50];
  struct_T a;
  struct_T *val_data;
  real_T time_data[5000];
  real_T z_data[200];
  real_T b_time_data[100];
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &jm_emlrtRSI;
  obj = b_tracker->Scheduler;
  b_st.site = &tm_emlrtRSI;
  c_AerospaceMonostaticRadar_time(
      varargin_1_LookTime_data, varargin_1_LookTime_size,
      varargin_1_DetectionTime_data, varargin_1_DetectionTime_size,
      jnb_emlrtRSI.data, jnb_emlrtRSI.size);
  b_st.site = &um_emlrtRSI;
  c_AerospaceMonostaticRadar_time(
      varargin_1_LookTime_data, varargin_1_LookTime_size,
      varargin_1_DetectionTime_data, varargin_1_DetectionTime_size, time_data,
      time_size);
  b_st.site = &vm_emlrtRSI;
  emxInit_real_T(&b_st, &y, 1, &xc_emlrtRTEI, true);
  k = y->size[0];
  y->size[0] = time_size[1] + 1;
  emxEnsureCapacity_real_T(&b_st, y, k, &uc_emlrtRTEI);
  y_data = y->data;
  y_data[0] = -1.7976931348623157E+308;
  k = time_size[1];
  for (i = 0; i < k; i++) {
    y_data[i + 1] = time_data[i];
  }
  b_i = obj->TimeTolerance;
  c_st.site = &kn_emlrtRSI;
  emxInit_int32_T(&c_st, &iidx, 1, &xc_emlrtRTEI);
  d_st.site = &ln_emlrtRSI;
  sort(&d_st, y, iidx);
  iidx_data = iidx->data;
  y_data = y->data;
  b_iidx = iidx->size[0];
  n = iidx->size[0];
  loop_ub = iidx->size[0];
  if (iidx->size[0] < 800) {
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
  emxInit_real_T(&b_st, &processTimestamps, 1, &pd_emlrtRTEI, true);
  k = processTimestamps->size[0];
  processTimestamps->size[0] = 1;
  emxEnsureCapacity_real_T(&b_st, processTimestamps, k, &vc_emlrtRTEI);
  processTimestamps_data = processTimestamps->data;
  processTimestamps_data[0] = y_data[0];
  ic_size = y->size[0];
  emxInit_int8_T(&b_st, &ia, &rd_emlrtRTEI);
  k = ia->size[0];
  ia->size[0] = 1;
  emxEnsureCapacity_int8_T(&b_st, ia, k, &wc_emlrtRTEI);
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
      emxEnsureCapacity_real_T(&b_st, processTimestamps, n, &xc_emlrtRTEI);
      processTimestamps_data = processTimestamps->data;
      if (i + 2 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 2, 1, b_loop_ub, &q_emlrtBCI, &b_st);
      }
      processTimestamps_data[loop_ub] = y_data[i + 1];
      loop_ub = ia->size[0];
      n = ia->size[0];
      ia->size[0]++;
      emxEnsureCapacity_int8_T(&b_st, ia, n, &xc_emlrtRTEI);
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
  b_st.site = &wm_emlrtRSI;
  c_st.site = &mo_emlrtRSI;
  b_sort(&c_st, processTimestamps);
  processTimestamps_data = processTimestamps->data;
  loop_ub = processTimestamps->size[0];
  k = obj->ProcessingTimestamps->size[0];
  obj->ProcessingTimestamps->size[0] = processTimestamps->size[0];
  emxEnsureCapacity_real_T(&st, obj->ProcessingTimestamps, k, &yc_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    obj->ProcessingTimestamps->data[i] = processTimestamps_data[i];
  }
  emxFree_real_T(&st, &processTimestamps);
  obj->CurrentIndex = 1.0;
  emxInit_boolean_T(sp, &a__1, 1, &xc_emlrtRTEI, true);
  emxInit_real_T(sp, &assignment, 2, &xc_emlrtRTEI, true);
  c_emxInitStruct_fusion_tracker_(sp, &SD->f3.obj, &ad_emlrtRTEI, true);
  c_emxInitStruct_fusion_tracker_(sp, &SD->f3.b_obj, &md_emlrtRTEI, true);
  emxInit_struct_T1(sp, &modelData, &xc_emlrtRTEI);
  do {
    exitg1 = 0;
    st.site = &km_emlrtRSI;
    obj = b_tracker->Scheduler;
    tf = (obj->CurrentIndex == obj->ProcessingTimestamps->size[0]);
    if (!tf) {
      st.site = &lm_emlrtRSI;
      obj = b_tracker->Scheduler;
      b_i = obj->CurrentIndex;
      b_st.site = &no_emlrtRSI;
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
      b_st.site = &oo_emlrtRSI;
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
      c_st.site = &qo_emlrtRSI;
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
      st.site = &mm_emlrtRSI;
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
        st.site = &nm_emlrtRSI;
        if (ifWhileCond(&st, &e_tmp_data)) {
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->f3.obj, &b_tracker->TrackListManager, &kd_emlrtRTEI);
          st.site = &om_emlrtRSI;
          JIPDATrackAssigner_assign(
              SD, &st, b_tracker->Assigner[0].AssignmentThreshold,
              b_tracker->Assigner[0].InitializationThreshold,
              b_tracker->Assigner[0].MaxNumEvents,
              &b_tracker->Assigner[0]
                   .Estimator.StateEstimator.StateEstimator
                   .TargetSpecifications[0],
              &b_tracker->Assigner[0]
                   .Estimator.StateEstimator.StateEstimator
                   .SensorSpecifications[0],
              b_tracker->Assigner[0]
                  .Estimator.StateEstimator.StateEstimator.TrackingFilter,
              &b_tracker->Assigner[0]
                   .Estimator.StateEstimator.ExistenceEstimator
                   .SensorSpecifications[0],
              SD->f3.obj.InternalTrackList, &multiSensorData, assignment, a__1,
              &unassignedSensorData_idx_0);
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->f3.obj, &b_tracker->TrackListManager, &ld_emlrtRTEI);
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->f3.b_obj, &b_tracker->TrackListManager, &md_emlrtRTEI);
          loop_ub = SD->f3.obj.InternalTrackList->size[0];
          k = SD->f3.b_obj.InternalTrackList->size[0];
          SD->f3.b_obj.InternalTrackList->size[0] =
              SD->f3.obj.InternalTrackList->size[0];
          emxEnsureCapacity_struct_T(sp, SD->f3.b_obj.InternalTrackList, k,
                                     &fd_emlrtRTEI);
          for (i = 0; i < loop_ub; i++) {
            SD->f3.b_obj.InternalTrackList->data[i] =
                SD->f3.obj.InternalTrackList->data[i];
          }
          st.site = &pm_emlrtRSI;
          JIPDATrackUpdater_update(
              SD, &st, b_tracker->Updater[0].AssignmentThreshold,
              &b_tracker->Updater[0]
                   .Estimator.StateEstimator.StateEstimator
                   .TargetSpecifications[0],
              &b_tracker->Updater[0]
                   .Estimator.StateEstimator.StateEstimator
                   .SensorSpecifications[0],
              b_tracker->Updater[0]
                  .Estimator.StateEstimator.StateEstimator.TrackingFilter,
              &b_tracker->Updater[0]
                   .Estimator.StateEstimator.ExistenceEstimator
                   .SensorSpecifications[0],
              SD->f3.b_obj.InternalTrackList, multiSensorData.LookTime.data,
              multiSensorData.LookTime.size, multiSensorData.LookAzimuth.data,
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
                                          &SD->f3.b_obj, &md_emlrtRTEI);
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->f3.obj, &b_tracker->TrackListManager, &nd_emlrtRTEI);
          st.site = &qm_emlrtRSI;
          SD->f3.r2 = b_tracker->Initiator[0];
          SD->f3.t39_Estimator_StateEstimator =
              SD->f3.r2.Estimator.StateEstimator;
          c_t32_Estimator_StateEstimator_ =
              SD->f3.r2.Estimator.StateEstimator.StateEstimator
                  .TargetSpecifications[0];
          SD->f3.val = SD->f3.r2.Estimator.StateEstimator.StateEstimator
                           .SensorSpecifications[0];
          u = b_tracker->LastTrackID;
          b_st.site = &xlb_emlrtRSI;
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
          b_st.site = &ylb_emlrtRSI;
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
            b_st.site = &amb_emlrtRSI;
            repmat(&b_st, b_time_data, b_time_size, z_size[1], time_data,
                   time_size);
          }
          b_st.site = &bmb_emlrtRSI;
          c_st.site = &bmb_emlrtRSI;
          a.Time = c_TrackEstimator_sampleDistribu(
              SD->f3.r2.Estimator.StateEstimator.StateEstimator.TrackingFilter,
              &a.TrackID, &a.Age, &a.IsConfirmed, &a.IsCoasted, a.State,
              a.StateCovariance, &a.ExistenceProbability);
          c_st.site = &sp_emlrtRSI;
          k = z_size[1];
          loop_ub = z_size[1];
          for (i = 0; i < k; i++) {
            newTracks_data[i] = a;
          }
          if (modelData->size[0] == 1) {
            b_st.site = &cmb_emlrtRSI;
            c_st.site = &xp_emlrtRSI;
            d_st.site = &yp_emlrtRSI;
            f_st.site = &bq_emlrtRSI;
            c_AerospaceMonostaticRadar_upda(
                &f_st, &SD->f3.val, modelData_data[0].LookTime.data,
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
            f_st.site = &bq_emlrtRSI;
            d_st.site = &aq_emlrtRSI;
            SD->f3.estimator_SensorSpecifications =
                SD->f3.r2.Estimator.StateEstimator.ExistenceEstimator
                    .SensorSpecifications[0];
            f_st.site = &ns_emlrtRSI;
            c_AerospaceMonostaticRadar_upda(
                &f_st, &SD->f3.estimator_SensorSpecifications,
                modelData_data[0].LookTime.data,
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
            SD->f3.t39_Estimator_StateEstimator.ExistenceEstimator
                .SensorSpecifications[0] =
                SD->f3.estimator_SensorSpecifications;
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
              b_st.site = &dmb_emlrtRSI;
              initializeTrack(&b_st, &c_t32_Estimator_StateEstimator_,
                              &SD->f3.val,
                              &SD->f3.t39_Estimator_StateEstimator
                                   .ExistenceEstimator.SensorSpecifications[0],
                              &newTracks_data[i], &z_data[4 * i], time_data[i]);
            } else {
              b_st.site = &emb_emlrtRSI;
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
              c_st.site = &xmb_emlrtRSI;
              d_st.site = &xp_emlrtRSI;
              f_st.site = &yp_emlrtRSI;
              SD->f3.b_val = SD->f3.val;
              g_st.site = &bq_emlrtRSI;
              c_AerospaceMonostaticRadar_upda(
                  &g_st, &SD->f3.b_val, modelData_data[i].LookTime.data,
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
              f_st.site = &aq_emlrtRSI;
              SD->f3.estimator_SensorSpecifications =
                  SD->f3.t39_Estimator_StateEstimator.ExistenceEstimator
                      .SensorSpecifications[0];
              g_st.site = &ns_emlrtRSI;
              c_AerospaceMonostaticRadar_upda(
                  &g_st, &SD->f3.estimator_SensorSpecifications,
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
              c_st.site = &ymb_emlrtRSI;
              initializeTrack(&c_st, &c_t32_Estimator_StateEstimator_,
                              &SD->f3.b_val,
                              &SD->f3.estimator_SensorSpecifications,
                              &newTracks_data[i], &z_data[4 * i], time_data[i]);
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
            newTracks_data[i].TrackID = u1;
          }
          b_i = (real_T)u + (real_T)z_size[1];
          if (b_i < 4.294967296E+9) {
            u = (uint32_T)b_i;
          } else {
            u = MAX_uint32_T;
          }
          c_emxCopyStruct_fusion_tracker_(
              sp, &SD->f3.b_obj, &b_tracker->TrackListManager, &od_emlrtRTEI);
          k = SD->f3.b_obj.InternalTrackList->size[0];
          SD->f3.b_obj.InternalTrackList->size[0] =
              SD->f3.obj.InternalTrackList->size[0] + z_size[1];
          emxEnsureCapacity_struct_T(sp, SD->f3.b_obj.InternalTrackList, k,
                                     &fd_emlrtRTEI);
          k = SD->f3.obj.InternalTrackList->size[0];
          for (i = 0; i < k; i++) {
            SD->f3.b_obj.InternalTrackList->data[i] =
                SD->f3.obj.InternalTrackList->data[i];
          }
          for (i = 0; i < loop_ub; i++) {
            SD->f3.b_obj.InternalTrackList
                ->data[i + SD->f3.obj.InternalTrackList->size[0]] =
                newTracks_data[i];
          }
          c_emxCopyStruct_fusion_tracker_(sp, &b_tracker->TrackListManager,
                                          &SD->f3.b_obj, &od_emlrtRTEI);
          b_tracker->LastTrackID = u;
        }
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  emxFree_struct_T1(sp, &modelData);
  c_emxFreeStruct_fusion_tracker_(sp, &SD->f3.b_obj);
  emxFree_real_T(sp, &assignment);
  emxFree_boolean_T(sp, &a__1);
  c_emxCopyStruct_fusion_tracker_(sp, &SD->f3.obj, &b_tracker->TrackListManager,
                                  &ad_emlrtRTEI);
  emxInit_struct_T(sp, &val, &ad_emlrtRTEI, true);
  loop_ub = SD->f3.obj.InternalTrackList->size[0];
  k = val->size[0];
  val->size[0] = SD->f3.obj.InternalTrackList->size[0];
  emxEnsureCapacity_struct_T(sp, val, k, &bd_emlrtRTEI);
  val_data = val->data;
  for (i = 0; i < loop_ub; i++) {
    val_data[i] = SD->f3.obj.InternalTrackList->data[i];
  }
  st.site = &rm_emlrtRSI;
  SD->f3.r = b_tracker->TrackMaintenance;
  b_i = SD->f3.r.DeletionThreshold;
  for (i = 0; i < loop_ub; i++) {
    if (i + 1 > val->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, val->size[0], &b_emlrtBCI, &st);
    }
    if (val_data[i].IsConfirmed ||
        (val_data[i].ExistenceProbability >= SD->f3.r.ConfirmationThreshold)) {
      if (i + 1 > val->size[0]) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, val->size[0], &s_emlrtBCI, &st);
      }
      val_data[i].IsConfirmed = true;
    }
  }
  emxInit_boolean_T(&st, &toDeleteTracks, 2, &cd_emlrtRTEI, true);
  k = toDeleteTracks->size[0] * toDeleteTracks->size[1];
  toDeleteTracks->size[0] = 1;
  loop_ub = val->size[0];
  toDeleteTracks->size[1] = val->size[0];
  emxEnsureCapacity_boolean_T(&st, toDeleteTracks, k, &cd_emlrtRTEI);
  toDeleteTracks_data = toDeleteTracks->data;
  for (i = 0; i < loop_ub; i++) {
    toDeleteTracks_data[i] = false;
  }
  k = val->size[0];
  if (val->size[0] < 800) {
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
        emlrtJBEnviron, h_st) firstprivate(emlrtHadParallelError, st)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        h_st.prev = &st;
        h_st.tls = emlrtAllocTLS(&st, omp_get_thread_num());
        h_st.site = NULL;
        emlrtSetJmpBuf(&h_st, &emlrtJBEnviron);
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
                                          &h_st);
          }
          if (d_i + 1 > toDeleteTracks->size[1]) {
            emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, toDeleteTracks->size[1],
                                          &t_emlrtBCI, &h_st);
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
  emxEnsureCapacity_struct_T(&st, val, k, &dd_emlrtRTEI);
  val_data = val->data;
  c_emxCopyStruct_fusion_tracker_(sp, &SD->f3.obj, &b_tracker->TrackListManager,
                                  &ed_emlrtRTEI);
  k = SD->f3.obj.InternalTrackList->size[0];
  SD->f3.obj.InternalTrackList->size[0] = n;
  emxEnsureCapacity_struct_T(sp, SD->f3.obj.InternalTrackList, k,
                             &fd_emlrtRTEI);
  for (i = 0; i < n; i++) {
    SD->f3.obj.InternalTrackList->data[i] = val_data[i];
  }
  emxFree_struct_T(sp, &val);
  c_emxCopyStruct_fusion_tracker_(sp, &b_tracker->TrackListManager, &SD->f3.obj,
                                  &ed_emlrtRTEI);
  c_emxCopyStruct_fusion_tracker_(sp, &SD->f3.obj, &b_tracker->TrackListManager,
                                  &gd_emlrtRTEI);
  st.site = &sm_emlrtRSI;
  SD->f3.r1 = b_tracker->Outputter;
  b_st.site = &anb_emlrtRSI;
  s1.TrackLogicState = SD->f3.r1.SampleTrack.pTrackLogicState;
  s1.TrackID = SD->f3.r1.SampleTrack.TrackID;
  s1.BranchID = SD->f3.r1.SampleTrack.BranchID;
  s1.SourceIndex = SD->f3.r1.SampleTrack.SourceIndex;
  s1.UpdateTime = SD->f3.r1.SampleTrack.pUpdateTime;
  s1.Age = SD->f3.r1.SampleTrack.Age;
  for (i = 0; i < 6; i++) {
    s1.State[i] = SD->f3.r1.SampleTrack.pState[i];
  }
  memcpy(&s1.StateCovariance[0], &SD->f3.r1.SampleTrack.pStateCovariance[0],
         36U * sizeof(real_T));
  s1.ObjectClassID = SD->f3.r1.SampleTrack.ObjectClassID;
  s1.ObjectClassProbabilities = SD->f3.r1.SampleTrack.ObjectClassProbabilities;
  for (i = 0; i < 10; i++) {
    s1.TrackLogic[i] = logicType[i];
  }
  s1.IsConfirmed = SD->f3.r1.SampleTrack.IsConfirmed;
  s1.IsCoasted = SD->f3.r1.SampleTrack.IsCoasted;
  s1.IsSelfReported = SD->f3.r1.SampleTrack.IsSelfReported;
  c_st.site = &cnb_emlrtRSI;
  d_st.site = &sp_emlrtRSI;
  emxInit_struct1_T(&c_st, &tracksObjectTrack, &hd_emlrtRTEI);
  loop_ub = SD->f3.obj.InternalTrackList->size[0];
  k = tracksObjectTrack->size[0];
  tracksObjectTrack->size[0] = SD->f3.obj.InternalTrackList->size[0];
  emxEnsureCapacity_struct1_T(&c_st, tracksObjectTrack, k, &hd_emlrtRTEI);
  tracksObjectTrack_data = tracksObjectTrack->data;
  for (i = 0; i < loop_ub; i++) {
    tracksObjectTrack_data[i] = s1;
  }
  for (e_i = 0; e_i < loop_ub; e_i++) {
    if (e_i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, loop_ub, &k_emlrtBCI, &b_st);
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &ab_emlrtBCI, &b_st);
    }
    for (i = 0; i < 6; i++) {
      if (e_i + 1 > tracksObjectTrack->size[0]) {
        emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                      &bb_emlrtBCI, &b_st);
      }
      tracksObjectTrack_data[e_i].State[i] =
          SD->f3.obj.InternalTrackList->data[e_i].State[i];
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    for (i = 0; i < 36; i++) {
      tracksObjectTrack_data[e_i].StateCovariance[i] =
          SD->f3.obj.InternalTrackList->data[e_i].StateCovariance[i];
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    for (i = 0; i < 10; i++) {
      tracksObjectTrack_data[e_i].TrackLogic[i] = logicType[i];
    }
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].TrackLogicState =
        SD->f3.obj.InternalTrackList->data[e_i].ExistenceProbability;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].Age =
        SD->f3.obj.InternalTrackList->data[e_i].Age;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].IsConfirmed =
        SD->f3.obj.InternalTrackList->data[e_i].IsConfirmed;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].IsCoasted =
        SD->f3.obj.InternalTrackList->data[e_i].IsCoasted;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].TrackID =
        SD->f3.obj.InternalTrackList->data[e_i].TrackID;
    if (e_i + 1 > tracksObjectTrack->size[0]) {
      emlrtDynamicBoundsCheckR2012b(e_i + 1, 1, tracksObjectTrack->size[0],
                                    &emlrtBCI, &b_st);
    }
    tracksObjectTrack_data[e_i].UpdateTime =
        SD->f3.obj.InternalTrackList->data[e_i].Time;
  }
  c_emxFreeStruct_fusion_tracker_(&b_st, &SD->f3.obj);
  b_st.site = &bnb_emlrtRSI;
  n = tracksObjectTrack->size[0];
  if (tracksObjectTrack->size[0] == 0) {
    k = 0;
  } else {
    k = tracksObjectTrack->size[0];
    c_st.site = &enb_emlrtRSI;
    if (tracksObjectTrack->size[0] > 2147483646) {
      d_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }
  }
  emxInit_boolean_T(&b_st, &isConfirmed, 2, &qd_emlrtRTEI, true);
  loop_ub = isConfirmed->size[0] * isConfirmed->size[1];
  isConfirmed->size[0] = 1;
  isConfirmed->size[1] = k;
  emxEnsureCapacity_boolean_T(&b_st, isConfirmed, loop_ub, &id_emlrtRTEI);
  toDeleteTracks_data = isConfirmed->data;
  c_st.site = &dnb_emlrtRSI;
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
  emxEnsureCapacity_struct1_T(&st, varargout_1, loop_ub, &jd_emlrtRTEI);
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

/* End of code generation (JIPDATracker.c) */
