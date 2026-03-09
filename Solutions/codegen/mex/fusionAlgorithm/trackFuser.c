/*
 * trackFuser.c
 *
 * Code generation for function 'trackFuser'
 *
 */

/* Include files */
#include "trackFuser.h"
#include "AssignerGNN.h"
#include "FuserManager.h"
#include "Fuserxcov.h"
#include "SystemCore.h"
#include "all.h"
#include "any.h"
#include "assertValidSizeArg.h"
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "eml_setop.h"
#include "find.h"
#include "fusionAlgorithm.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "gaussEKFilter.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "mrdivide_helper.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "unique.h"
#include "validateattributes.h"
#include "warning.h"
#include "xzgetrf.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo r_emlrtRSI = { 1,   /* lineNo */
  "SystemProp/matlabCodegenNotifyAnyProp",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemProp.p"/* pathName */
};

static emlrtRSInfo cj_emlrtRSI = { 1019,/* lineNo */
  "trackFuser/resetImpl",              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ej_emlrtRSI = { 476,/* lineNo */
  "trackFuser/stepImpl",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo fj_emlrtRSI = { 478,/* lineNo */
  "trackFuser/stepImpl",               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo gj_emlrtRSI = { 490,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo hj_emlrtRSI = { 491,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ij_emlrtRSI = { 492,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo jj_emlrtRSI = { 501,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo kj_emlrtRSI = { 504,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo lj_emlrtRSI = { 507,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo mj_emlrtRSI = { 508,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo nj_emlrtRSI = { 509,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo oj_emlrtRSI = { 513,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo pj_emlrtRSI = { 516,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo qj_emlrtRSI = { 521,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo rj_emlrtRSI = { 524,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo sj_emlrtRSI = { 527,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo tj_emlrtRSI = { 543,/* lineNo */
  "trackFuser/coreAlgorithm",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo uj_emlrtRSI = { 613,/* lineNo */
  "FuserManager/collectSourceIDs",     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtRSInfo vj_emlrtRSI = { 614,/* lineNo */
  "FuserManager/collectSourceIDs",     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtRSInfo ll_emlrtRSI = { 626,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ml_emlrtRSI = { 620,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo nl_emlrtRSI = { 619,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ol_emlrtRSI = { 613,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo pl_emlrtRSI = { 606,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ql_emlrtRSI = { 602,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo rl_emlrtRSI = { 595,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo sl_emlrtRSI = { 584,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo tl_emlrtRSI = { 577,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ul_emlrtRSI = { 576,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo vl_emlrtRSI = { 574,/* lineNo */
  "trackFuser/assign",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo wl_emlrtRSI = { 83, /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo xl_emlrtRSI = { 16, /* lineNo */
  "any",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\any.m"/* pathName */
};

static emlrtRSInfo em_emlrtRSI = { 1,  /* lineNo */
  "SystemCore/setupAndReset",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"/* pathName */
};

static emlrtRSInfo lo_emlrtRSI = { 42, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\sort.m"/* pathName */
};

static emlrtRSInfo mp_emlrtRSI = { 23, /* lineNo */
  "intersect",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\intersect.m"/* pathName */
};

static emlrtRSInfo np_emlrtRSI = { 97, /* lineNo */
  "eml_setop",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\private\\eml_setop.m"/* pathName */
};

static emlrtRSInfo up_emlrtRSI = { 634,/* lineNo */
  "trackFuser/getInitializing",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo vp_emlrtRSI = { 650,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo wp_emlrtRSI = { 659,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo xp_emlrtRSI = { 666,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo yp_emlrtRSI = { 668,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo aq_emlrtRSI = { 669,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo bq_emlrtRSI = { 671,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo cq_emlrtRSI = { 673,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo dq_emlrtRSI = { 675,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo eq_emlrtRSI = { 679,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo fq_emlrtRSI = { 680,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo gq_emlrtRSI = { 697,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo hq_emlrtRSI = { 703,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo iq_emlrtRSI = { 704,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo jq_emlrtRSI = { 705,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo kq_emlrtRSI = { 706,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo lq_emlrtRSI = { 715,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo mq_emlrtRSI = { 714,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo nq_emlrtRSI = { 723,/* lineNo */
  "trackFuser/initializeCentralTracks",/* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo oq_emlrtRSI = { 848,/* lineNo */
  "trackFuser/getSelfReporting",       /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo pq_emlrtRSI = { 758,/* lineNo */
  "trackFuser/distanceToCentralTrack", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo qq_emlrtRSI = { 762,/* lineNo */
  "trackFuser/distanceToCentralTrack", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo rq_emlrtRSI = { 763,/* lineNo */
  "trackFuser/distanceToCentralTrack", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo sq_emlrtRSI = { 766,/* lineNo */
  "trackFuser/distanceToCentralTrack", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo tq_emlrtRSI = { 767,/* lineNo */
  "trackFuser/distanceToCentralTrack", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo uq_emlrtRSI = { 770,/* lineNo */
  "trackFuser/distanceToCentralTrack", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo wq_emlrtRSI = { 15, /* lineNo */
  "sum",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\sum.m"/* pathName */
};

static emlrtRSInfo ar_emlrtRSI = { 39, /* lineNo */
  "cat",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pathName */
};

static emlrtRSInfo cr_emlrtRSI = { 781,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo dr_emlrtRSI = { 786,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo er_emlrtRSI = { 787,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo fr_emlrtRSI = { 789,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo gr_emlrtRSI = { 794,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo hr_emlrtRSI = { 800,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ir_emlrtRSI = { 801,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo jr_emlrtRSI = { 802,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo kr_emlrtRSI = { 803,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo lr_emlrtRSI = { 804,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo mr_emlrtRSI = { 806,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo nr_emlrtRSI = { 809,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo or_emlrtRSI = { 810,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo pr_emlrtRSI = { 814,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo qr_emlrtRSI = { 818,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo rr_emlrtRSI = { 819,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo sr_emlrtRSI = { 821,/* lineNo */
  "trackFuser/fuseAssigned",           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo fv_emlrtRSI = { 13, /* lineNo */
  "nullAssignment",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo gv_emlrtRSI = { 17, /* lineNo */
  "nullAssignment",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo hv_emlrtRSI = { 152,/* lineNo */
  "onearg_null_assignment",            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pathName */
};

static emlrtRSInfo iv_emlrtRSI = { 904,/* lineNo */
  "trackFuser/coastUnassigned",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo jv_emlrtRSI = { 910,/* lineNo */
  "trackFuser/coastUnassigned",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo kv_emlrtRSI = { 911,/* lineNo */
  "trackFuser/coastUnassigned",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo lv_emlrtRSI = { 915,/* lineNo */
  "trackFuser/coastUnassigned",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo mv_emlrtRSI = { 918,/* lineNo */
  "trackFuser/coastUnassigned",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo ov_emlrtRSI = { 928,/* lineNo */
  "trackFuser/predictTracks",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo pv_emlrtRSI = { 933,/* lineNo */
  "trackFuser/predictTracks",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo qv_emlrtRSI = { 934,/* lineNo */
  "trackFuser/predictTracks",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo rv_emlrtRSI = { 935,/* lineNo */
  "trackFuser/predictTracks",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pathName */
};

static emlrtRSInfo sv_emlrtRSI = { 762,/* lineNo */
  "FuserManager/formatOutput",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtRSInfo tv_emlrtRSI = { 763,/* lineNo */
  "FuserManager/formatOutput",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtRSInfo uv_emlrtRSI = { 764,/* lineNo */
  "FuserManager/formatOutput",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtRSInfo vv_emlrtRSI = { 808,/* lineNo */
  "FuserManager/modTrackLogicState",   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtRSInfo wv_emlrtRSI = { 235,/* lineNo */
  "trackHistoryLogic/output",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackHistoryLogic.m"/* pathName */
};

static emlrtRSInfo xv_emlrtRSI = { 795,/* lineNo */
  "FuserManager/formatTrackWithLogic", /* fcnName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pathName */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  799,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  799,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  798,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  798,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  797,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  797,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  796,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  796,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = { 795, /* lineNo */
  77,                                  /* colNo */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  795,                                 /* lineNo */
  77,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  794,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  794,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = { 793, /* lineNo */
  69,                                  /* colNo */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  793,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  792,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = { 792, /* lineNo */
  69,                                  /* colNo */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  792,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  791,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = { 791, /* lineNo */
  69,                                  /* colNo */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  791,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  790,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = { 790, /* lineNo */
  69,                                  /* colNo */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  790,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = { 789, /* lineNo */
  69,                                  /* colNo */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  789,                                 /* lineNo */
  69,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = { 808, /* lineNo */
  65,                                  /* colNo */
  "FuserManager/modTrackLogicState",   /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  479,                                 /* lineNo */
  36,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/stepImpl",               /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo i_emlrtDCI = { 762, /* lineNo */
  81,                                  /* colNo */
  "FuserManager/formatOutput",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  762,                                 /* lineNo */
  81,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatOutput",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = { 0,   /* iFirst */
  99,                                  /* iLast */
  925,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/predictTracks",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo y_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  524,                                 /* lineNo */
  70,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  516,                                 /* lineNo */
  71,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  635,                                 /* lineNo */
  32,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getInitializing",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  634,                                 /* lineNo */
  80,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getInitializing",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  827,                                 /* lineNo */
  44,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/trackIDs",             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  563,                                 /* lineNo */
  27,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getInputTrackTimes",     /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  563,                                 /* lineNo */
  49,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getInputTrackTimes",     /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  480,                                 /* lineNo */
  13,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/stepImpl",               /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  549,                                 /* lineNo */
  36,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  548,                                 /* lineNo */
  43,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  537,                                 /* lineNo */
  17,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = { 480, /* lineNo */
  60,                                  /* colNo */
  "trackFuser/stepImpl",               /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  480,                                 /* lineNo */
  60,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/stepImpl",               /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo t_emlrtRTEI = { 486,/* lineNo */
  17,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 493,/* lineNo */
  62,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtDCInfo k_emlrtDCI = { 498, /* lineNo */
  48,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  498,                                 /* lineNo */
  48,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo l_emlrtDCI = { 518, /* lineNo */
  48,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo mb_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  518,                                 /* lineNo */
  48,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo m_emlrtDCI = { 518, /* lineNo */
  62,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  518,                                 /* lineNo */
  62,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = { 532, /* lineNo */
  46,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo ob_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  532,                                 /* lineNo */
  46,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo o_emlrtDCI = { 549, /* lineNo */
  64,                                  /* colNo */
  "trackFuser/coreAlgorithm",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  607,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/collectSourceIDs",     /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo v_emlrtRTEI = { 609,/* lineNo */
  89,                                  /* colNo */
  "FuserManager/collectSourceIDs",     /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtBCInfo qb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  610,                                 /* lineNo */
  36,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/collectSourceIDs",     /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  614,                                 /* lineNo */
  59,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/collectSourceIDs",     /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  634,                                 /* lineNo */
  63,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getInitializing",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo w_emlrtRTEI = { 924,/* lineNo */
  21,                                  /* colNo */
  "trackFuser/predictTracks",          /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtBCInfo tb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  788,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  793,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  795,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/formatTrackWithLogic", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo cb_emlrtRTEI = { 1,/* lineNo */
  1,                                   /* colNo */
  "SystemCore/setup",                  /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"/* pName */
};

static emlrtDCInfo p_emlrtDCI = { 587, /* lineNo */
  40,                                  /* colNo */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo q_emlrtDCI = { 587, /* lineNo */
  40,                                  /* colNo */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  595,                                 /* lineNo */
  67,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  614,                                 /* lineNo */
  50,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  614,                                 /* lineNo */
  68,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  614,                                 /* lineNo */
  21,                                  /* colNo */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtBCInfo ac_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  607,                                 /* lineNo */
  40,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  607,                                 /* lineNo */
  56,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  607,                                 /* lineNo */
  21,                                  /* colNo */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtBCInfo cc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  608,                                 /* lineNo */
  40,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  608,                                 /* lineNo */
  56,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  608,                                 /* lineNo */
  21,                                  /* colNo */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtBCInfo ec_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  600,                                 /* lineNo */
  51,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  625,                                 /* lineNo */
  73,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  627,                                 /* lineNo */
  53,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  614,                                 /* lineNo */
  112,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ic_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  608,                                 /* lineNo */
  99,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/assign",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo r_emlrtDCI = { 710, /* lineNo */
  45,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo bd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  710,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  708,                                 /* lineNo */
  25,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo dd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  707,                                 /* lineNo */
  25,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ed_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  706,                                 /* lineNo */
  85,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  697,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  697,                                 /* lineNo */
  145,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo s_emlrtDCI = { 697, /* lineNo */
  102,                                 /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo hd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  697,                                 /* lineNo */
  102,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo id_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  696,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo t_emlrtDCI = { 685, /* lineNo */
  42,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo jd_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  685,                                 /* lineNo */
  42,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo u_emlrtDCI = { 685, /* lineNo */
  80,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo kd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  685,                                 /* lineNo */
  80,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo v_emlrtDCI = { 684, /* lineNo */
  35,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo ld_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  684,                                 /* lineNo */
  35,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo w_emlrtDCI = { 681, /* lineNo */
  45,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo md_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  681,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo x_emlrtDCI = { 680, /* lineNo */
  106,                                 /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo nd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  680,                                 /* lineNo */
  106,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo y_emlrtDCI = { 679, /* lineNo */
  43,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo od_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  679,                                 /* lineNo */
  43,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo ab_emlrtDCI = { 859,/* lineNo */
  34,                                  /* colNo */
  "trackFuser/getClassID",             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo pd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  859,                                 /* lineNo */
  34,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getClassID",             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo bb_emlrtDCI = { 674,/* lineNo */
  43,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo qd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  674,                                 /* lineNo */
  43,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  643,                                 /* lineNo */
  31,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  643,                                 /* lineNo */
  70,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo td_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  643,                                 /* lineNo */
  48,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo cb_emlrtDCI = { 37, /* lineNo */
  14,                                  /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo ud_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  649,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo db_emlrtDCI = { 663,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo eb_emlrtDCI = { 664,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo fb_emlrtDCI = { 665,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo gb_emlrtDCI = { 666,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo hb_emlrtDCI = { 667,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo ib_emlrtDCI = { 668,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo jb_emlrtDCI = { 669,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo kb_emlrtDCI = { 670,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo lb_emlrtDCI = { 671,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo mb_emlrtDCI = { 672,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo nb_emlrtDCI = { 673,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo ob_emlrtDCI = { 675,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo pb_emlrtDCI = { 676,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo qb_emlrtDCI = { 680,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtECInfo d_emlrtECI = { -1,  /* nDims */
  697,                                 /* lineNo */
  21,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtDCInfo rb_emlrtDCI = { 709,/* lineNo */
  41,                                  /* colNo */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo vd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  663,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo wd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  664,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo xd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  665,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo yd_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  666,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo ae_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  667,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo be_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  668,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ce_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  669,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo de_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  670,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo ee_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  671,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  672,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo ge_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  673,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo he_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  675,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo ie_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  676,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo je_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  680,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo ke_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  709,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/initializeCentralTracks",/* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo le_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  847,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getSelfReporting",       /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo me_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  863,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getClassID",             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo pb_emlrtRTEI = { 199,/* lineNo */
  35,                                  /* colNo */
  "trackHistoryLogic/init",            /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackHistoryLogic.m"/* pName */
};

static emlrtRTEInfo qb_emlrtRTEI = { 85,/* lineNo */
  27,                                  /* colNo */
  "validate_inputs",                   /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pName */
};

static emlrtRTEInfo rb_emlrtRTEI = { 158,/* lineNo */
  9,                                   /* colNo */
  "onearg_null_assignment",            /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\nullAssignment.m"/* pName */
};

static emlrtBCInfo ne_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  750,                                 /* lineNo */
  46,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/distanceToCentralTrack", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo oe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  750,                                 /* lineNo */
  56,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/distanceToCentralTrack", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  771,                                 /* lineNo */
  24,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/distanceToCentralTrack", /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  798,                                 /* lineNo */
  49,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo sb_emlrtDCI = { 803,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo tb_emlrtDCI = { 804,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo ub_emlrtDCI = { 806,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtRTEInfo tb_emlrtRTEI = { 809,/* lineNo */
  29,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtDCInfo vb_emlrtDCI = { 812,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo re_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  806,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtRTEInfo vb_emlrtRTEI = { 207,/* lineNo */
  56,                                  /* colNo */
  "trackHistoryLogic/hit",             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackHistoryLogic.m"/* pName */
};

static emlrtBCInfo se_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  833,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getCoasting",            /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo te_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  892,                                 /* lineNo */
  36,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ue_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  893,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ve_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  894,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo we_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  789,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xe_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  874,                                 /* lineNo */
  42,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getToFuse",              /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ye_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  875,                                 /* lineNo */
  43,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getToFuse",              /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo af_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  785,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  785,                                 /* lineNo */
  53,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  789,                                 /* lineNo */
  21,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo df_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  876,                                 /* lineNo */
  24,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getToFuse",              /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ef_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  803,                                 /* lineNo */
  75,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo wb_emlrtDCI = { 803,/* lineNo */
  75,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo ff_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  803,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo gf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  804,                                 /* lineNo */
  123,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  863,                                 /* lineNo */
  67,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getClassID",             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo if_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  806,                                 /* lineNo */
  87,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo xb_emlrtDCI = { 806,/* lineNo */
  87,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo jf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  806,                                 /* lineNo */
  140,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo kf_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  813,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo yb_emlrtDCI = { 813,/* lineNo */
  41,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo lf_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  810,                                 /* lineNo */
  46,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo ac_emlrtDCI = { 810,/* lineNo */
  46,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo mf_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  814,                                 /* lineNo */
  63,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo bc_emlrtDCI = { 814,/* lineNo */
  63,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo nf_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  815,                                 /* lineNo */
  45,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo cc_emlrtDCI = { 815,/* lineNo */
  45,                                  /* colNo */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo of_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  816,                                 /* lineNo */
  42,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo pf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  833,                                 /* lineNo */
  67,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getCoasting",            /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  819,                                 /* lineNo */
  115,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  892,                                 /* lineNo */
  56,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo sf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  893,                                 /* lineNo */
  67,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo tf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  894,                                 /* lineNo */
  65,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo uf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  822,                                 /* lineNo */
  29,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo vf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  895,                                 /* lineNo */
  54,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo wf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  895,                                 /* lineNo */
  74,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  895,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAttributes",         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo pg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  847,                                 /* lineNo */
  67,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/getSelfReporting",       /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  818,                                 /* lineNo */
  105,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo rg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  821,                                 /* lineNo */
  130,                                 /* colNo */
  "",                                  /* aName */
  "trackFuser/fuseAssigned",           /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo ac_emlrtRTEI = { 214,/* lineNo */
  56,                                  /* colNo */
  "trackHistoryLogic/miss",            /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\trackHistoryLogic.m"/* pName */
};

static emlrtRTEInfo bc_emlrtRTEI = { 739,/* lineNo */
  25,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtRTEInfo cc_emlrtRTEI = { 743,/* lineNo */
  29,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtDCInfo dc_emlrtDCI = { 747,/* lineNo */
  37,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo ec_emlrtDCI = { 748,/* lineNo */
  38,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo sg_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  750,                                 /* lineNo */
  82,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo e_emlrtECI = { -1,  /* nDims */
  750,                                 /* lineNo */
  21,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtBCInfo tg_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  751,                                 /* lineNo */
  68,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo f_emlrtECI = { -1,  /* nDims */
  751,                                 /* lineNo */
  21,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtBCInfo ug_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  747,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo vg_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  748,                                 /* lineNo */
  38,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo fc_emlrtDCI = { 909,/* lineNo */
  33,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo gc_emlrtDCI = { 908,/* lineNo */
  33,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo wg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  908,                                 /* lineNo */
  50,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo xg_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  908,                                 /* lineNo */
  76,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo hc_emlrtDCI = { 908,/* lineNo */
  76,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo yg_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  908,                                 /* lineNo */
  93,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ah_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  908,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo bh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  909,                                 /* lineNo */
  50,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ch_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  909,                                 /* lineNo */
  33,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo dh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  910,                                 /* lineNo */
  39,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo ic_emlrtDCI = { 910,/* lineNo */
  39,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo eh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  910,                                 /* lineNo */
  56,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  740,                                 /* lineNo */
  47,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  911,                                 /* lineNo */
  80,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo jc_emlrtDCI = { 911,/* lineNo */
  80,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo hh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  911,                                 /* lineNo */
  97,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ih_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  912,                                 /* lineNo */
  38,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo kc_emlrtDCI = { 912,/* lineNo */
  38,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo jh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  912,                                 /* lineNo */
  55,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo kh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  913,                                 /* lineNo */
  37,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo lc_emlrtDCI = { 913,/* lineNo */
  37,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo lh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  913,                                 /* lineNo */
  54,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo mh_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  911,                                 /* lineNo */
  26,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  3                                    /* checkKind */
};

static emlrtBCInfo nh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  911,                                 /* lineNo */
  43,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo oh_emlrtBCI = { 1,  /* iFirst */
  100,                                 /* iLast */
  914,                                 /* lineNo */
  29,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ph_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  914,                                 /* lineNo */
  46,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo qh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  744,                                 /* lineNo */
  64,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo mc_emlrtDCI = { 744,/* lineNo */
  64,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo rh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  744,                                 /* lineNo */
  41,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo nc_emlrtDCI = { 744,/* lineNo */
  41,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo sh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  745,                                 /* lineNo */
  66,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo oc_emlrtDCI = { 745,/* lineNo */
  66,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo th_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  745,                                 /* lineNo */
  42,                                  /* colNo */
  "",                                  /* aName */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  3                                    /* checkKind */
};

static emlrtDCInfo pc_emlrtDCI = { 745,/* lineNo */
  42,                                  /* colNo */
  "FuserManager/recycleTracks",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo uh_emlrtBCI = { 0,  /* iFirst */
  99,                                  /* iLast */
  915,                                 /* lineNo */
  44,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo qc_emlrtDCI = { 915,/* lineNo */
  44,                                  /* colNo */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo vh_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  915,                                 /* lineNo */
  61,                                  /* colNo */
  "",                                  /* aName */
  "trackFuser/coastUnassigned",        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo vc_emlrtRTEI = { 497,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo wc_emlrtRTEI = { 604,/* lineNo */
  13,                                  /* colNo */
  "FuserManager",                      /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = { 524,/* lineNo */
  70,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo yc_emlrtRTEI = { 524,/* lineNo */
  61,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo ad_emlrtRTEI = { 28,/* lineNo */
  9,                                   /* colNo */
  "colon",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pName */
};

static emlrtRTEInfo bd_emlrtRTEI = { 763,/* lineNo */
  21,                                  /* colNo */
  "FuserManager",                      /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+internal\\FuserManager.m"/* pName */
};

static emlrtRTEInfo cd_emlrtRTEI = { 504,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo dd_emlrtRTEI = { 521,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo ed_emlrtRTEI = { 475,/* lineNo */
  59,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo gd_emlrtRTEI = { 575,/* lineNo */
  17,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo hd_emlrtRTEI = { 587,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo id_emlrtRTEI = { 589,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo jd_emlrtRTEI = { 595,/* lineNo */
  36,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo kd_emlrtRTEI = { 600,/* lineNo */
  37,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo ld_emlrtRTEI = { 627,/* lineNo */
  53,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo md_emlrtRTEI = { 627,/* lineNo */
  34,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo nd_emlrtRTEI = { 614,/* lineNo */
  21,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo od_emlrtRTEI = { 627,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo pd_emlrtRTEI = { 608,/* lineNo */
  21,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo qd_emlrtRTEI = { 97,/* lineNo */
  35,                                  /* colNo */
  "eml_setop",                         /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\private\\eml_setop.m"/* pName */
};

static emlrtRTEInfo rd_emlrtRTEI = { 589,/* lineNo */
  52,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo td_emlrtRTEI = { 569,/* lineNo */
  49,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo ud_emlrtRTEI = { 620,/* lineNo */
  96,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo lf_emlrtRTEI = { 781,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo mf_emlrtRTEI = { 794,/* lineNo */
  37,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo nf_emlrtRTEI = { 796,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo of_emlrtRTEI = { 799,/* lineNo */
  17,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo pf_emlrtRTEI = { 801,/* lineNo */
  26,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo qf_emlrtRTEI = { 804,/* lineNo */
  123,                                 /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo rf_emlrtRTEI = { 806,/* lineNo */
  140,                                 /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo sf_emlrtRTEI = { 819,/* lineNo */
  103,                                 /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo tf_emlrtRTEI = { 821,/* lineNo */
  72,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo uf_emlrtRTEI = { 794,/* lineNo */
  30,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo vf_emlrtRTEI = { 890,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo wf_emlrtRTEI = { 781,/* lineNo */
  33,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo og_emlrtRTEI = { 806,/* lineNo */
  55,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo pg_emlrtRTEI = { 794,/* lineNo */
  13,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo qg_emlrtRTEI = { 801,/* lineNo */
  17,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo rg_emlrtRTEI = { 904,/* lineNo */
  39,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

static emlrtRTEInfo tg_emlrtRTEI = { 902,/* lineNo */
  49,                                  /* colNo */
  "trackFuser",                        /* fName */
  "C:\\Program Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m"/* pName */
};

/* Function Declarations */
static void b_trackFuser_fuseAssigned(const emlrtStack *sp, trackFuser *obj,
  const emxArray_struct0_T *localTracks, const emxArray_uint32_T *assignments,
  emxArray_boolean_T *updated);
static void c_trackFuser_distanceToCentralT(const emlrtStack *sp, trackFuser
  *obj, const emxArray_struct0_T *localTracks, uint32_T
  initializedCentralTrack_TrackID, uint32_T c_initializedCentralTrack_Branc,
  uint32_T initializedCentralTrack_Age, real_T c_initializedCentralTrack_Objec,
  real_T d_initializedCentralTrack_Objec, boolean_T
  c_initializedCentralTrack_IsCon, boolean_T c_initializedCentralTrack_IsCoa,
  boolean_T c_initializedCentralTrack_IsSel, const real_T
  initializedCentralTrack_pState[6], const real_T
  c_initializedCentralTrack_pStat[36], real_T c_initializedCentralTrack_pUpda,
  const uint32_T localInds_data[], int32_T localInds_size, real_T cost_data[],
  int32_T cost_size[2]);
static void c_trackFuser_initializeCentralT(const emlrtStack *sp, trackFuser
  *obj, const emxArray_struct0_T *localTracks, uint32_T
  unassignedlocalTracks_data[], int32_T unassignedlocalTracks_size);
static void trackFuser_assign(const emlrtStack *sp, trackFuser *obj, const
  emxArray_real_T *costMatrix, emxArray_uint32_T *overallAssignments,
  emxArray_uint32_T *overallUnassignedCentralTracks, uint32_T
  c_overallUnassignedLocalTracks_[], int32_T *d_overallUnassignedLocalTracks_);
static void trackFuser_coastUnassigned(const emlrtStack *sp, trackFuser *obj,
  const emxArray_uint32_T *unassignedTracks, const emxArray_real_T *notUpdated,
  boolean_T toDelete[100]);
static void trackFuser_fuseAssigned(const emlrtStack *sp, trackFuser *obj, const
  emxArray_struct0_T *localTracks, const uint32_T assignments_data[], const
  int32_T assignments_size[2]);
static boolean_T trackFuser_getSelfReporting(const emlrtStack *sp, trackFuser
  *obj, const emxArray_struct0_T *localTracks, const emxArray_uint32_T
  *assignedlocalTracks);

/* Function Definitions */
static void b_trackFuser_fuseAssigned(const emlrtStack *sp, trackFuser *obj,
  const emxArray_struct0_T *localTracks, const emxArray_uint32_T *assignments,
  emxArray_boolean_T *updated)
{
  b_objectTrack b_expl_temp;
  b_objectTrack jw_emlrtRSI;
  b_objectTrack kw_emlrtRSI;
  c_objectTrack b_obj;
  c_objectTrack *transformedTracks_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_boolean_T *inAssigned;
  emxArray_boolean_T *toFuse;
  emxArray_int32_T *ii;
  emxArray_int32_T *r;
  emxArray_int32_T *r1;
  emxArray_objectTrack *transformedTracks;
  emxArray_real_T *b_ii;
  emxArray_struct1_T *otherAttributes;
  emxArray_uint32_T *b_assignments;
  emxArray_uint32_T *uniqueAssigned;
  fuserSourceConfiguration *thisConfig;
  const struct0_T *localTracks_data;
  struct1_T *otherAttributes_data;
  trackHistoryLogic *c_obj;
  real_T b_dv[2];
  real_T *b_ii_data;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T i;
  int32_T loop_ub;
  int32_T nz;
  int32_T *ii_data;
  int32_T *r2;
  int32_T *r3;
  const uint32_T *assignments_data;
  uint32_T *b_assignments_data;
  uint32_T *uniqueAssigned_data;
  boolean_T *inAssigned_data;
  boolean_T *toFuse_data;
  boolean_T *updated_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  assignments_data = assignments->data;
  localTracks_data = localTracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj->pTracksList[99];
  b_dv[0] = 1.0;
  b_dv[1] = assignments->size[0];
  emxInit_objectTrack(sp, &transformedTracks, &lf_emlrtRTEI);
  st.site = &cr_emlrtRSI;
  repmat(&st, &b_obj, b_dv, transformedTracks);
  transformedTracks_data = transformedTracks->data;
  loop_ub = assignments->size[0];
  for (i = 0; i < loop_ub; i++) {
    struct0_T expl_temp;
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &bf_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    nz = (int32_T)assignments_data[i + assignments->size[0]];
    if ((nz < 1) || (nz > localTracks->size[0])) {
      emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0], &af_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    expl_temp = localTracks_data[nz - 1];
    st.site = &dr_emlrtRSI;
    b_st.site = &ug_emlrtRSI;
    thisConfig = FuserManager_getConfigByID(&b_st, obj, expl_temp.SourceIndex);
    st.site = &er_emlrtRSI;
    if (!thisConfig->pIsTransformToCentralValid) {
      b_st.site = &vg_emlrtRSI;
      c_st.site = &xg_emlrtRSI;
      local2central(&c_st, expl_temp.TrackID, expl_temp.BranchID,
                    expl_temp.SourceIndex, expl_temp.UpdateTime, expl_temp.Age,
                    expl_temp.State, expl_temp.StateCovariance,
                    expl_temp.ObjectClassID,
                    expl_temp.ObjectClassProbabilities.data,
                    expl_temp.ObjectClassProbabilities.size,
                    expl_temp.IsConfirmed, expl_temp.IsCoasted,
                    expl_temp.IsSelfReported, &b_expl_temp);
      c_st.site = &yg_emlrtRSI;

      /*  Pre-define output with longest TrackLogic to force codegen consistency */
      if (b_expl_temp.SourceIndex == 1U) {
        /*  Radar */
        d_st.site = &fi_emlrtRSI;
        Ecef2nedTrack(&d_st, b_expl_temp.TrackID, b_expl_temp.BranchID, 1U,
                      b_expl_temp.Age, b_expl_temp.ObjectClassID,
                      b_expl_temp.ObjectClassProbabilities.data,
                      b_expl_temp.ObjectClassProbabilities.size,
                      b_expl_temp.IsConfirmed, b_expl_temp.IsCoasted,
                      b_expl_temp.IsSelfReported, b_expl_temp.pState,
                      b_expl_temp.pStateCovariance, b_expl_temp.pUpdateTime,
                      &jw_emlrtRSI);
      } else if (b_expl_temp.SourceIndex == 2U) {
        /*  ADSB */
        d_st.site = &gi_emlrtRSI;
        Ned2ecefTrack(&d_st, b_expl_temp.TrackID, b_expl_temp.BranchID, 2U,
                      b_expl_temp.Age, b_expl_temp.ObjectClassID,
                      b_expl_temp.ObjectClassProbabilities.data,
                      b_expl_temp.ObjectClassProbabilities.size,
                      b_expl_temp.IsConfirmed, b_expl_temp.IsCoasted,
                      b_expl_temp.IsSelfReported, b_expl_temp.pState,
                      b_expl_temp.pStateCovariance, b_expl_temp.pUpdateTime,
                      &kw_emlrtRSI);
      }

      thisConfig->pIsTransformToCentralValid = true;
    }

    b_st.site = &wg_emlrtRSI;
    local2central(&b_st, expl_temp.TrackID, expl_temp.BranchID,
                  expl_temp.SourceIndex, expl_temp.UpdateTime, expl_temp.Age,
                  expl_temp.State, expl_temp.StateCovariance,
                  expl_temp.ObjectClassID,
                  expl_temp.ObjectClassProbabilities.data,
                  expl_temp.ObjectClassProbabilities.size, expl_temp.IsConfirmed,
                  expl_temp.IsCoasted, expl_temp.IsSelfReported, &b_expl_temp);
    st.site = &fr_emlrtRSI;
    nz = transformedTracks->size[1] - 1;
    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    b_st.site = &qc_emlrtRSI;
    c_st.site = &gb_emlrtRSI;
    if (b_expl_temp.pUpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }

    c_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(b_expl_temp.pUpdateTime) || muDoubleScalarIsNaN
        (b_expl_temp.pUpdateTime)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &m_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }

    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    transformedTracks_data[i].pUpdateTime = b_expl_temp.pUpdateTime;
    st.site = &fr_emlrtRSI;
    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    b_st.site = &rc_emlrtRSI;
    validateattributes(&b_st, b_expl_temp.pState);
    for (b_i = 0; b_i < 6; b_i++) {
      if (i > nz) {
        emlrtDynamicBoundsCheckR2012b(i, 0, nz, &cf_emlrtBCI, &st);
      }

      transformedTracks_data[i].pState[b_i] = b_expl_temp.pState[b_i];
    }

    st.site = &fr_emlrtRSI;
    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    b_st.site = &sc_emlrtRSI;
    b_validateattributes(&b_st, b_expl_temp.pStateCovariance);
    b_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&b_st, b_expl_temp.pStateCovariance);
    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    for (b_i = 0; b_i < 36; b_i++) {
      transformedTracks_data[i].pStateCovariance[b_i] =
        b_expl_temp.pStateCovariance[b_i];
    }

    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, (emlrtConstCTX)sp);
    }

    transformedTracks_data[i].IsConfirmed = b_expl_temp.IsConfirmed;
    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, (emlrtConstCTX)sp);
    }

    transformedTracks_data[i].IsCoasted = b_expl_temp.IsCoasted;
    if (i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, (emlrtConstCTX)sp);
    }

    transformedTracks_data[i].IsSelfReported = b_expl_temp.IsSelfReported;
  }

  st.site = &gr_emlrtRSI;
  emxInit_uint32_T(&st, &b_assignments, 1, &mf_emlrtRTEI);
  nz = b_assignments->size[0];
  b_assignments->size[0] = assignments->size[0];
  emxEnsureCapacity_uint32_T(&st, b_assignments, nz, &mf_emlrtRTEI);
  b_assignments_data = b_assignments->data;
  for (b_i = 0; b_i < loop_ub; b_i++) {
    b_assignments_data[b_i] = assignments_data[b_i];
  }

  emxInit_uint32_T(&st, &uniqueAssigned, 1, &pg_emlrtRTEI);
  b_st.site = &ib_emlrtRSI;
  b_unique_vector(&b_st, b_assignments, uniqueAssigned);
  uniqueAssigned_data = uniqueAssigned->data;
  nz = updated->size[0] * updated->size[1];
  updated->size[0] = 1;
  b_loop_ub = uniqueAssigned->size[0];
  updated->size[1] = uniqueAssigned->size[0];
  emxEnsureCapacity_boolean_T(sp, updated, nz, &nf_emlrtRTEI);
  updated_data = updated->data;
  for (b_i = 0; b_i < b_loop_ub; b_i++) {
    updated_data[b_i] = false;
  }

  emxInit_boolean_T(sp, &inAssigned, 1, &of_emlrtRTEI, true);
  emxInit_boolean_T(sp, &toFuse, 1, &qg_emlrtRTEI, true);
  emxInit_int32_T(sp, &r, 1, &qf_emlrtRTEI);
  emxInit_int32_T(sp, &ii, 1, &sd_emlrtRTEI);
  emxInit_struct1_T(sp, &otherAttributes, &vf_emlrtRTEI);
  emxInit_int32_T(sp, &r1, 1, &rf_emlrtRTEI);
  emxInit_real_T(sp, &b_ii, 1, &og_emlrtRTEI);
  for (c_i = 0; c_i < b_loop_ub; c_i++) {
    int32_T trueCount;
    if (c_i + 1 > b_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, b_loop_ub, &qe_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    nz = inAssigned->size[0];
    inAssigned->size[0] = loop_ub;
    emxEnsureCapacity_boolean_T(sp, inAssigned, nz, &of_emlrtRTEI);
    inAssigned_data = inAssigned->data;
    for (b_i = 0; b_i < loop_ub; b_i++) {
      inAssigned_data[b_i] = (uniqueAssigned_data[c_i] == assignments_data[b_i]);
    }

    st.site = &hr_emlrtRSI;
    b_st.site = &yl_emlrtRSI;
    eml_find(&b_st, inAssigned, ii);
    ii_data = ii->data;
    st.site = &ir_emlrtRSI;
    trueCount = ii->size[0];
    nz = toFuse->size[0];
    toFuse->size[0] = ii->size[0];
    emxEnsureCapacity_boolean_T(&st, toFuse, nz, &pf_emlrtRTEI);
    toFuse_data = toFuse->data;
    for (b_i = 0; b_i < trueCount; b_i++) {
      if (b_i + 1 > trueCount) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trueCount, &xe_emlrtBCI, &st);
      }

      nz = ii_data[b_i] - 1;
      if ((nz < 0) || (nz > transformedTracks->size[1] - 1)) {
        emlrtDynamicBoundsCheckR2012b(nz, 0, transformedTracks->size[1] - 1,
          &ye_emlrtBCI, &st);
      }

      if ((transformedTracks_data[ii_data[b_i] - 1].IsConfirmed &&
           (!transformedTracks_data[ii_data[b_i] - 1].IsCoasted)) ||
          transformedTracks_data[ii_data[b_i] - 1].IsSelfReported) {
        if (b_i + 1 > toFuse->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, toFuse->size[0],
            &df_emlrtBCI, &st);
        }

        toFuse_data[b_i] = true;
      } else {
        if (b_i + 1 > toFuse->size[0]) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, toFuse->size[0],
            &df_emlrtBCI, &st);
        }

        toFuse_data[b_i] = false;
      }
    }

    st.site = &jr_emlrtRSI;
    if (b_any(&st, toFuse)) {
      real_T id;
      int32_T c_loop_ub;
      int32_T end;
      int32_T obj_tmp;
      uint32_T u;
      boolean_T b_value;
      st.site = &kr_emlrtRSI;
      b_st.site = &wq_emlrtRSI;
      c_st.site = &bg_emlrtRSI;
      nz = b_combineVectorElements(&c_st, toFuse);
      obj_tmp = (int32_T)uniqueAssigned_data[c_i] - 1;
      b_obj = obj->pTracksList[obj_tmp];
      b_value = ((obj_tmp < 0) || (obj_tmp > 99));
      if (b_value) {
        emlrtDynamicBoundsCheckR2012b(obj_tmp, 0, 99, &ef_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      id = (real_T)obj->pTracksList[obj_tmp].Age + (real_T)nz;
      if (id < 4.294967296E+9) {
        if (id >= 0.0) {
          u = (uint32_T)id;
        } else {
          u = 0U;
        }
      } else {
        u = MAX_uint32_T;
      }

      b_obj.Age = u;
      obj->pTracksList[obj_tmp] = b_obj;
      trueCount = 0;
      for (i = 0; i < loop_ub; i++) {
        if (inAssigned_data[i]) {
          trueCount++;
        }
      }

      nz = r->size[0];
      r->size[0] = trueCount;
      emxEnsureCapacity_int32_T(sp, r, nz, &qf_emlrtRTEI);
      r2 = r->data;
      nz = 0;
      for (i = 0; i < loop_ub; i++) {
        if (inAssigned_data[i]) {
          r2[nz] = i;
          nz++;
        }
      }

      st.site = &lr_emlrtRSI;
      c_loop_ub = r->size[0];
      for (i = 0; i < c_loop_ub; i++) {
        if (r2[i] > assignments->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(r2[i], 0, assignments->size[0] - 1,
            &gf_emlrtBCI, &st);
        }
      }

      u = uniqueAssigned_data[c_i];
      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &ab_emlrtDCI, &st);
      }

      if (((int32_T)u - 1 < 0) || ((int32_T)u - 1 > 99)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &pd_emlrtBCI, &st);
      }

      id = obj->pTracksList[(int32_T)u - 1].ObjectClassID;
      nz = 0;
      while ((id == 0.0) && (nz < r->size[0])) {
        nz++;
        if (nz > c_loop_ub) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, c_loop_ub, &hf_emlrtBCI, &st);
        }

        trueCount = (int32_T)assignments_data[r2[nz - 1] + assignments->size[0]];
        if ((trueCount < 1) || (trueCount > localTracks->size[0])) {
          emlrtDynamicBoundsCheckR2012b(trueCount, 1, localTracks->size[0],
            &me_emlrtBCI, &st);
        }

        id = localTracks_data[trueCount - 1].ObjectClassID;
      }

      st.site = &lr_emlrtRSI;
      b_obj = obj->pTracksList[obj_tmp];
      b_st.site = &ig_emlrtRSI;
      c_validateattributes(&b_st, id);
      b_obj.ObjectClassID = id;
      obj->pTracksList[obj_tmp] = b_obj;
      if (b_value) {
        emlrtDynamicBoundsCheckR2012b(obj_tmp, 0, 99, &if_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      b_obj = obj->pTracksList[obj_tmp];
      end = toFuse->size[0];
      trueCount = 0;
      for (i = 0; i < end; i++) {
        if (toFuse_data[i]) {
          trueCount++;
        }
      }

      nz = r1->size[0];
      r1->size[0] = trueCount;
      emxEnsureCapacity_int32_T(sp, r1, nz, &rf_emlrtRTEI);
      r3 = r1->data;
      nz = 0;
      for (i = 0; i < end; i++) {
        if (toFuse_data[i]) {
          r3[nz] = i;
          nz++;
        }
      }

      trueCount = r1->size[0];
      nz = b_ii->size[0];
      b_ii->size[0] = r1->size[0];
      emxEnsureCapacity_real_T(sp, b_ii, nz, &og_emlrtRTEI);
      b_ii_data = b_ii->data;
      for (i = 0; i < trueCount; i++) {
        if (r3[i] > ii->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(r3[i], 0, ii->size[0] - 1, &jf_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        b_ii_data[i] = ii_data[r3[i]];
      }

      st.site = &mr_emlrtRSI;
      Fuserxcov_fuse(&st, &obj->cFuser, &b_obj, transformedTracks, b_ii);
      obj->pTracksList[obj_tmp] = b_obj;
      st.site = &nr_emlrtRSI;
      b_st.site = &wq_emlrtRSI;
      c_st.site = &bg_emlrtRSI;
      nz = b_combineVectorElements(&c_st, toFuse);
      emlrtForLoopVectorCheckR2021a(1.0, 1.0, nz, mxDOUBLE_CLASS, nz,
        &tb_emlrtRTEI, (emlrtConstCTX)sp);
      for (b_i = 0; b_i < nz; b_i++) {
        boolean_T bv[50];
        st.site = &or_emlrtRSI;
        c_obj = obj->pTrackLogics[obj_tmp];
        if (c_obj->pIsFirstUpdate) {
          emlrtErrorWithMessageIdR2018a(&st, &vb_emlrtRTEI,
            "shared_tracking:trackHistoryLogic:notInitialized",
            "shared_tracking:trackHistoryLogic:notInitialized", 3, 4, 3, "hit");
        }

        bv[0] = true;
        for (i = 0; i < 49; i++) {
          bv[i + 1] = c_obj->pRecentHistory[i];
        }

        for (i = 0; i < 50; i++) {
          c_obj->pRecentHistory[i] = bv[i];
        }

        c_obj->pIsFirstUpdate = false;
      }

      if (obj->pTracksList[obj_tmp].IsConfirmed) {
        b_value = true;
      } else {
        st.site = &pr_emlrtRSI;
        c_obj = obj->pTrackLogics[obj_tmp];
        if (c_obj->pIsFirstUpdate) {
          b_value = false;
        } else {
          boolean_T x_idx_1;
          boolean_T x_idx_2;
          b_value = c_obj->pRecentHistory[0];
          x_idx_1 = c_obj->pRecentHistory[1];
          x_idx_2 = c_obj->pRecentHistory[2];
          b_value = ((b_value + x_idx_1) + x_idx_2 >= 2);
        }

        if (b_value || (obj->pTracksList[obj_tmp].ObjectClassID > 0.0)) {
          b_value = true;
        } else {
          b_value = false;
        }
      }

      b_obj = obj->pTracksList[obj_tmp];
      b_obj.IsConfirmed = b_value;
      obj->pTracksList[obj_tmp] = b_obj;
      if (((int32_T)uniqueAssigned_data[c_i] < 1) || ((int32_T)
           uniqueAssigned_data[c_i] > 100)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)uniqueAssigned_data[c_i], 1, 100,
          &of_emlrtBCI, (emlrtConstCTX)sp);
      }

      obj->pConfirmedTracks[obj_tmp] = obj->pTracksList[obj_tmp].IsConfirmed;
      st.site = &qr_emlrtRSI;
      for (i = 0; i < c_loop_ub; i++) {
        if (r2[i] > assignments->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(r2[i], 0, assignments->size[0] - 1,
            &qg_emlrtBCI, &st);
        }
      }

      b_value = false;
      nz = 0;
      while ((!b_value) && (nz < r->size[0])) {
        nz++;
        if (nz > c_loop_ub) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, c_loop_ub, &pf_emlrtBCI, &st);
        }

        trueCount = (int32_T)assignments_data[r2[nz - 1] + assignments->size[0]];
        if ((trueCount < 1) || (trueCount > localTracks->size[0])) {
          emlrtDynamicBoundsCheckR2012b(trueCount, 1, localTracks->size[0],
            &se_emlrtBCI, &st);
        }

        b_value = localTracks_data[trueCount - 1].IsCoasted;
      }

      b_obj = obj->pTracksList[obj_tmp];
      b_obj.IsCoasted = b_value;
      obj->pTracksList[obj_tmp] = b_obj;
      b_obj = obj->pTracksList[obj_tmp];
      nz = b_assignments->size[0];
      b_assignments->size[0] = r->size[0];
      emxEnsureCapacity_uint32_T(sp, b_assignments, nz, &sf_emlrtRTEI);
      b_assignments_data = b_assignments->data;
      for (i = 0; i < c_loop_ub; i++) {
        if (r2[i] > assignments->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(r2[i], 0, assignments->size[0] - 1,
            &qf_emlrtBCI, (emlrtConstCTX)sp);
        }

        b_assignments_data[i] = assignments_data[r2[i] + assignments->size[0]];
      }

      st.site = &rr_emlrtRSI;
      b_obj.IsSelfReported = trackFuser_getSelfReporting(&st, obj, localTracks,
        b_assignments);
      obj->pTracksList[obj_tmp] = b_obj;
      b_obj = obj->pTracksList[obj_tmp];
      obj->pTracksList[obj_tmp] = b_obj;
      st.site = &sr_emlrtRSI;
      for (i = 0; i < c_loop_ub; i++) {
        if (r2[i] > assignments->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(r2[i], 0, assignments->size[0] - 1,
            &rg_emlrtBCI, &st);
        }
      }

      nz = otherAttributes->size[0] * otherAttributes->size[1];
      otherAttributes->size[0] = 1;
      otherAttributes->size[1] = r->size[0];
      emxEnsureCapacity_struct1_T(&st, otherAttributes, nz, &tf_emlrtRTEI);
      otherAttributes_data = otherAttributes->data;
      for (i = 0; i < c_loop_ub; i++) {
        if (i + 1 > c_loop_ub) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, c_loop_ub, &rf_emlrtBCI, &st);
        }

        nz = (int32_T)assignments_data[r2[i] + assignments->size[0]];
        if ((nz < 1) || (nz > localTracks->size[0])) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &te_emlrtBCI, &st);
        }

        if (i + 1 > c_loop_ub) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, c_loop_ub, &sf_emlrtBCI, &st);
        }

        if (nz > localTracks->size[0]) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &ue_emlrtBCI, &st);
        }

        if (i + 1 > c_loop_ub) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, c_loop_ub, &tf_emlrtBCI, &st);
        }

        if (nz > localTracks->size[0]) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &ve_emlrtBCI, &st);
        }

        if (i + 1 > c_loop_ub) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, c_loop_ub, &wf_emlrtBCI, &st);
        }

        if (nz > localTracks->size[0]) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &vf_emlrtBCI, &st);
        }

        if (i > otherAttributes->size[1] - 1) {
          emlrtDynamicBoundsCheckR2012b(i, 0, otherAttributes->size[1] - 1,
            &xf_emlrtBCI, &st);
        }

        otherAttributes_data[i] = localTracks_data[nz - 1].ObjectAttributes;
      }

      b_obj = obj->pTracksList[obj_tmp];
      obj->pTracksList[obj_tmp] = b_obj;
      if (c_i + 1 > updated->size[1]) {
        emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, updated->size[1], &uf_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      updated_data[c_i] = true;
    }
  }

  emxFree_real_T(sp, &b_ii);
  emxFree_uint32_T(sp, &b_assignments);
  emxFree_int32_T(sp, &r1);
  emxFree_struct1_T(sp, &otherAttributes);
  emxFree_int32_T(sp, &ii);
  emxFree_int32_T(sp, &r);
  emxFree_boolean_T(sp, &toFuse);
  emxFree_boolean_T(sp, &inAssigned);
  emxFree_uint32_T(sp, &uniqueAssigned);
  emxFree_objectTrack(sp, &transformedTracks);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void c_trackFuser_distanceToCentralT(const emlrtStack *sp, trackFuser
  *obj, const emxArray_struct0_T *localTracks, uint32_T
  initializedCentralTrack_TrackID, uint32_T c_initializedCentralTrack_Branc,
  uint32_T initializedCentralTrack_Age, real_T c_initializedCentralTrack_Objec,
  real_T d_initializedCentralTrack_Objec, boolean_T
  c_initializedCentralTrack_IsCon, boolean_T c_initializedCentralTrack_IsCoa,
  boolean_T c_initializedCentralTrack_IsSel, const real_T
  initializedCentralTrack_pState[6], const real_T
  c_initializedCentralTrack_pStat[36], real_T c_initializedCentralTrack_pUpda,
  const uint32_T localInds_data[], int32_T localInds_size, real_T cost_data[],
  int32_T cost_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  fuserSourceConfiguration *thisSource;
  objectTrack b_expl_temp;
  objectTrack unusedExpr;
  const struct0_T *localTracks_data;
  real_T S[72];
  real_T allCovars[72];
  real_T P[36];
  int32_T i;
  int32_T j;
  int32_T k;
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
  localTracks_data = localTracks->data;
  if (localInds_size == 0) {
    cost_size[0] = 1;
    cost_size[1] = 0;
  } else {
    cost_size[0] = 1;
    cost_size[1] = localInds_size;
    memset(&cost_data[0], 0, (uint32_T)localInds_size * sizeof(real_T));
    for (i = 0; i < localInds_size; i++) {
      struct0_T expl_temp;
      real_T allStates[12];
      real_T e[12];
      real_T x[6];
      real_T allCosts[2];
      uint32_T thisSource_tmp;
      if (i + 1 > localInds_size) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, localInds_size, &oe_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      thisSource_tmp = localInds_data[i];
      if (((int32_T)thisSource_tmp < 1) || ((int32_T)thisSource_tmp >
           localTracks->size[0])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)thisSource_tmp, 1,
          localTracks->size[0], &ne_emlrtBCI, (emlrtConstCTX)sp);
      }

      expl_temp = localTracks_data[(int32_T)thisSource_tmp - 1];
      memcpy(&P[0], &c_initializedCentralTrack_pStat[0], 36U * sizeof(real_T));
      st.site = &pq_emlrtRSI;
      b_gaussEKFilter_predict(&st, initializedCentralTrack_pState, P,
        obj->ProcessNoise, localTracks_data[(int32_T)thisSource_tmp - 1].
        UpdateTime - c_initializedCentralTrack_pUpda, x);
      st.site = &qq_emlrtRSI;
      b_st.site = &rc_emlrtRSI;
      validateattributes(&b_st, x);
      st.site = &rq_emlrtRSI;
      b_st.site = &sc_emlrtRSI;
      b_validateattributes(&b_st, P);
      b_st.site = &tc_emlrtRSI;
      isSymmetricPositiveSemiDefinite(&b_st, P);
      st.site = &sq_emlrtRSI;
      thisSource_tmp = localTracks_data[(int32_T)thisSource_tmp - 1].SourceIndex;
      b_st.site = &ug_emlrtRSI;
      thisSource = FuserManager_getConfigByID(&b_st, obj, thisSource_tmp);
      st.site = &tq_emlrtRSI;
      if (!thisSource->pIsTransformToLocalValid) {
        b_st.site = &nk_emlrtRSI;
        c_st.site = &pk_emlrtRSI;
        central2local(&c_st, initializedCentralTrack_TrackID,
                      c_initializedCentralTrack_Branc, thisSource_tmp,
                      initializedCentralTrack_Age,
                      c_initializedCentralTrack_Objec,
                      d_initializedCentralTrack_Objec,
                      c_initializedCentralTrack_IsCon,
                      c_initializedCentralTrack_IsCoa,
                      c_initializedCentralTrack_IsSel, x, P,
                      c_initializedCentralTrack_pUpda, &b_expl_temp);
        c_st.site = &qk_emlrtRSI;
        b_local2central(&c_st, b_expl_temp.TrackID, b_expl_temp.BranchID,
                        b_expl_temp.SourceIndex, b_expl_temp.Age,
                        b_expl_temp.ObjectClassID,
                        b_expl_temp.ObjectClassProbabilities,
                        b_expl_temp.IsConfirmed, b_expl_temp.IsCoasted,
                        b_expl_temp.IsSelfReported, b_expl_temp.pState,
                        b_expl_temp.pStateCovariance, b_expl_temp.pUpdateTime,
                        &unusedExpr);
        thisSource->pIsTransformToLocalValid = true;
      }

      b_st.site = &ok_emlrtRSI;
      central2local(&b_st, initializedCentralTrack_TrackID,
                    c_initializedCentralTrack_Branc, thisSource_tmp,
                    initializedCentralTrack_Age, c_initializedCentralTrack_Objec,
                    d_initializedCentralTrack_Objec,
                    c_initializedCentralTrack_IsCon,
                    c_initializedCentralTrack_IsCoa,
                    c_initializedCentralTrack_IsSel, x, P,
                    c_initializedCentralTrack_pUpda, &b_expl_temp);
      for (j = 0; j < 6; j++) {
        allStates[j] = b_expl_temp.pState[j];
        allStates[j + 6] = expl_temp.State[j];
      }

      for (j = 0; j < 36; j++) {
        allCovars[j] = b_expl_temp.pStateCovariance[j];
        allCovars[j + 36] = expl_temp.StateCovariance[j];
      }

      st.site = &uq_emlrtRSI;
      b_st.site = &sk_emlrtRSI;
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 6; k++) {
          __m128d r;
          __m128d r1;
          int32_T e_tmp;
          e_tmp = k + 6 * j;
          e[e_tmp] = allStates[k] - allStates[e_tmp];
          r = _mm_loadu_pd(&allCovars[6 * k]);
          e_tmp = 6 * k + 36 * j;
          r1 = _mm_loadu_pd(&allCovars[e_tmp]);
          _mm_storeu_pd(&S[e_tmp], _mm_add_pd(r, r1));
          r = _mm_loadu_pd(&allCovars[6 * k + 2]);
          r1 = _mm_loadu_pd(&allCovars[e_tmp + 2]);
          _mm_storeu_pd(&S[e_tmp + 2], _mm_add_pd(r, r1));
          r = _mm_loadu_pd(&allCovars[6 * k + 4]);
          r1 = _mm_loadu_pd(&allCovars[e_tmp + 4]);
          _mm_storeu_pd(&S[e_tmp + 4], _mm_add_pd(r, r1));
        }
      }

      for (k = 0; k < 2; k++) {
        real_T b_x;
        real_T y;
        int32_T ipiv[6];
        boolean_T isodd;
        c_st.site = &tk_emlrtRSI;
        for (j = 0; j < 6; j++) {
          x[j] = e[j + 6 * k];
        }

        d_st.site = &uk_emlrtRSI;
        mrdiv(&d_st, x, &S[36 * k]);
        c_st.site = &tk_emlrtRSI;
        d_st.site = &kl_emlrtRSI;
        memcpy(&P[0], &S[k * 36], 36U * sizeof(real_T));
        e_st.site = &cl_emlrtRSI;
        xzgetrf(&e_st, P, ipiv);
        y = P[0];
        isodd = false;
        for (j = 0; j < 5; j++) {
          y *= P[(j + 6 * (j + 1)) + 1];
          if (ipiv[j] > j + 1) {
            isodd = !isodd;
          }
        }

        if (isodd) {
          y = -y;
        }

        c_st.site = &tk_emlrtRSI;
        if (y < 0.0) {
          emlrtErrorWithMessageIdR2018a(&c_st, &sb_emlrtRTEI,
            "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError",
            3, 4, 3, "log");
        }

        b_x = 0.0;
        for (j = 0; j < 6; j++) {
          b_x += x[j] * e[j + 6 * k];
        }

        allCosts[k] = b_x + muDoubleScalarLog(y);
      }

      if (i + 1 > localInds_size) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, localInds_size, &pe_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      cost_data[i] = allCosts[1];
    }
  }
}

static void c_trackFuser_initializeCentralT(const emlrtStack *sp, trackFuser
  *obj, const emxArray_struct0_T *localTracks, uint32_T
  unassignedlocalTracks_data[], int32_T unassignedlocalTracks_size)
{
  b_objectTrack expl_temp;
  b_objectTrack jw_emlrtRSI;
  b_objectTrack kw_emlrtRSI;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  fuserSourceConfiguration *thisConfig;
  const struct0_T *localTracks_data;
  trackHistoryLogic *c_obj;
  real_T b_tmp_data[200];
  real_T costMatrix_data[200];
  int32_T allSourceInds_size;
  int32_T i;
  int32_T nz;
  uint32_T allSourceInds_data[200];
  uint32_T checkedUnassigned_data[200];
  uint32_T u1;
  uint8_T c_tmp_data[200];
  uint8_T tmp_data[200];
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  localTracks_data = localTracks->data;
  allSourceInds_size = unassignedlocalTracks_size;
  if (unassignedlocalTracks_size - 1 >= 0) {
    memset(&allSourceInds_data[0], 0, (uint32_T)unassignedlocalTracks_size *
           sizeof(uint32_T));
  }

  for (i = 0; i < unassignedlocalTracks_size; i++) {
    if (i + 1 > unassignedlocalTracks_size) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, unassignedlocalTracks_size,
        &rd_emlrtBCI, (emlrtConstCTX)sp);
    }

    if (i + 1 > unassignedlocalTracks_size) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, unassignedlocalTracks_size,
        &sd_emlrtBCI, (emlrtConstCTX)sp);
    }

    nz = (int32_T)unassignedlocalTracks_data[i];
    if ((nz < 1) || (nz > localTracks->size[0])) {
      emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0], &td_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    allSourceInds_data[i] = localTracks_data[nz - 1].SourceIndex;
  }

  exitg1 = false;
  while ((!exitg1) && (unassignedlocalTracks_size > 0)) {
    if (obj->pNumLiveTracks < 100.0) {
      int32_T thisConfig_tmp;
      uint32_T b_thisConfig_tmp;
      boolean_T tf;
      tf = (((int32_T)unassignedlocalTracks_data[0] < 1) || ((int32_T)
             unassignedlocalTracks_data[0] > localTracks->size[0]));
      if (tf) {
        emlrtDynamicBoundsCheckR2012b((int32_T)unassignedlocalTracks_data[0], 1,
          localTracks->size[0], &ud_emlrtBCI, (emlrtConstCTX)sp);
      }

      st.site = &vp_emlrtRSI;
      thisConfig_tmp = (int32_T)unassignedlocalTracks_data[0] - 1;
      b_thisConfig_tmp = localTracks_data[thisConfig_tmp].SourceIndex;
      b_st.site = &ug_emlrtRSI;
      thisConfig = FuserManager_getConfigByID(&b_st, obj, b_thisConfig_tmp);
      if ((!thisConfig->IsInternalSource) && localTracks_data[thisConfig_tmp].
          IsCoasted) {
        exitg1 = true;
      } else {
        c_objectTrack b_obj;
        real_T b_i;
        int32_T tmp_size[2];
        int32_T j;
        int32_T trueCount;
        uint32_T q0;
        uint32_T qY;
        uint8_T u;
        boolean_T assignedToNewTrack_data[200];
        boolean_T bv[50];
        boolean_T isr;
        st.site = &wp_emlrtRSI;
        if (!thisConfig->pIsTransformToCentralValid) {
          b_st.site = &vg_emlrtRSI;
          c_st.site = &xg_emlrtRSI;
          local2central(&c_st, localTracks_data[thisConfig_tmp].TrackID,
                        localTracks_data[thisConfig_tmp].BranchID,
                        b_thisConfig_tmp, localTracks_data[thisConfig_tmp].
                        UpdateTime, localTracks_data[thisConfig_tmp].Age,
                        localTracks_data[thisConfig_tmp].State,
                        localTracks_data[thisConfig_tmp].StateCovariance,
                        localTracks_data[thisConfig_tmp].ObjectClassID,
                        localTracks_data[thisConfig_tmp].
                        ObjectClassProbabilities.data,
                        localTracks_data[thisConfig_tmp].
                        ObjectClassProbabilities.size,
                        localTracks_data[thisConfig_tmp].IsConfirmed,
                        localTracks_data[thisConfig_tmp].IsCoasted,
                        localTracks_data[thisConfig_tmp].IsSelfReported,
                        &expl_temp);
          c_st.site = &yg_emlrtRSI;

          /*  Pre-define output with longest TrackLogic to force codegen consistency */
          if (expl_temp.SourceIndex == 1U) {
            /*  Radar */
            d_st.site = &fi_emlrtRSI;
            Ecef2nedTrack(&d_st, expl_temp.TrackID, expl_temp.BranchID, 1U,
                          expl_temp.Age, expl_temp.ObjectClassID,
                          expl_temp.ObjectClassProbabilities.data,
                          expl_temp.ObjectClassProbabilities.size,
                          expl_temp.IsConfirmed, expl_temp.IsCoasted,
                          expl_temp.IsSelfReported, expl_temp.pState,
                          expl_temp.pStateCovariance, expl_temp.pUpdateTime,
                          &jw_emlrtRSI);
          } else if (expl_temp.SourceIndex == 2U) {
            /*  ADSB */
            d_st.site = &gi_emlrtRSI;
            Ned2ecefTrack(&d_st, expl_temp.TrackID, expl_temp.BranchID, 2U,
                          expl_temp.Age, expl_temp.ObjectClassID,
                          expl_temp.ObjectClassProbabilities.data,
                          expl_temp.ObjectClassProbabilities.size,
                          expl_temp.IsConfirmed, expl_temp.IsCoasted,
                          expl_temp.IsSelfReported, expl_temp.pState,
                          expl_temp.pStateCovariance, expl_temp.pUpdateTime,
                          &kw_emlrtRSI);
          }

          thisConfig->pIsTransformToCentralValid = true;
        }

        b_st.site = &wg_emlrtRSI;
        local2central(&b_st, localTracks_data[thisConfig_tmp].TrackID,
                      localTracks_data[thisConfig_tmp].BranchID,
                      b_thisConfig_tmp, localTracks_data[thisConfig_tmp].
                      UpdateTime, localTracks_data[thisConfig_tmp].Age,
                      localTracks_data[thisConfig_tmp].State,
                      localTracks_data[thisConfig_tmp].StateCovariance,
                      localTracks_data[thisConfig_tmp].ObjectClassID,
                      localTracks_data[thisConfig_tmp].
                      ObjectClassProbabilities.data,
                      localTracks_data[thisConfig_tmp].
                      ObjectClassProbabilities.size,
                      localTracks_data[thisConfig_tmp].IsConfirmed,
                      localTracks_data[thisConfig_tmp].IsCoasted,
                      localTracks_data[thisConfig_tmp].IsSelfReported,
                      &expl_temp);
        obj->pNumLiveTracks++;
        q0 = obj->pLastTrackID;
        qY = q0 + 1U;
        if (q0 + 1U < q0) {
          qY = MAX_uint32_T;
        }

        obj->pLastTrackID = qY;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &db_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        b_obj.TrackID = obj->pLastTrackID;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &vd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &eb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        b_obj.BranchID = 0U;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &wd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &fb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        b_obj.SourceIndex = 3U;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &xd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &gb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        st.site = &xp_emlrtRSI;
        b_obj = obj->pTracksList[nz];
        b_st.site = &qc_emlrtRSI;
        c_st.site = &gb_emlrtRSI;
        if (expl_temp.pUpdateTime < 0.0) {
          emlrtErrorWithMessageIdR2018a(&c_st, &l_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedNonnegative",
            "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
        }

        c_st.site = &gb_emlrtRSI;
        if (muDoubleScalarIsInf(expl_temp.pUpdateTime) || muDoubleScalarIsNaN
            (expl_temp.pUpdateTime)) {
          emlrtErrorWithMessageIdR2018a(&c_st, &m_emlrtRTEI,
            "Coder:toolbox:ValidateattributesexpectedFinite",
            "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
        }

        b_obj.pUpdateTime = expl_temp.pUpdateTime;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &yd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &hb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        b_obj.Age = 1U;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ae_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &ib_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &be_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        st.site = &yp_emlrtRSI;
        b_obj = obj->pTracksList[nz];
        b_st.site = &rc_emlrtRSI;
        validateattributes(&b_st, expl_temp.pState);
        for (i = 0; i < 6; i++) {
          b_obj.pState[i] = expl_temp.pState[i];
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &jb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ce_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        st.site = &aq_emlrtRSI;
        b_obj = obj->pTracksList[nz];
        b_st.site = &sc_emlrtRSI;
        b_validateattributes(&b_st, expl_temp.pStateCovariance);
        b_st.site = &tc_emlrtRSI;
        isSymmetricPositiveSemiDefinite(&b_st, expl_temp.pStateCovariance);
        memcpy(&b_obj.pStateCovariance[0], &expl_temp.pStateCovariance[0], 36U *
               sizeof(real_T));
        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &kb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &de_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &lb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        st.site = &bq_emlrtRSI;
        b_obj = obj->pTracksList[nz];
        b_st.site = &ig_emlrtRSI;
        c_validateattributes(&b_st, expl_temp.ObjectClassID);
        b_obj.ObjectClassID = expl_temp.ObjectClassID;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ee_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &mb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        b_obj.IsCoasted = expl_temp.IsCoasted;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &fe_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &nb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        st.site = &cq_emlrtRSI;
        tf = false;
        b_st.site = &oq_emlrtRSI;
        c_st.site = &ug_emlrtRSI;
        thisConfig = FuserManager_getConfigByID(&c_st, obj, b_thisConfig_tmp);
        if (thisConfig->IsInternalSource && localTracks_data[thisConfig_tmp].
            IsSelfReported) {
          tf = true;
        }

        b_obj = obj->pTracksList[nz];
        b_obj.IsSelfReported = tf;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ge_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &bb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &qd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        isr = obj->pTracksList[nz].IsSelfReported;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &ob_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        st.site = &dq_emlrtRSI;
        b_i = obj->pNumLiveTracks;
        if (b_i != (int32_T)muDoubleScalarFloor(b_i)) {
          emlrtIntegerCheckR2012b(b_i, &ab_emlrtDCI, &st);
        }

        if (((int32_T)b_i - 1 < 0) || ((int32_T)b_i - 1 > 99)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_i - 1, 0, 99, &pd_emlrtBCI,
            &st);
        }

        b_i = obj->pTracksList[(int32_T)b_i - 1].ObjectClassID;
        j = 0;
        while ((b_i == 0.0) && (j < 1)) {
          j = 1;
          b_i = localTracks_data[thisConfig_tmp].ObjectClassID;
        }

        st.site = &dq_emlrtRSI;
        b_obj = obj->pTracksList[nz];
        b_st.site = &ig_emlrtRSI;
        c_validateattributes(&b_st, b_i);
        b_obj.ObjectClassID = b_i;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &he_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &pb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ie_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        st.site = &eq_emlrtRSI;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &y_emlrtDCI, &st);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &od_emlrtBCI, &st);
        }

        c_obj = obj->pTrackLogics[nz];
        if (!c_obj->pIsFirstUpdate) {
          emlrtErrorWithMessageIdR2018a(&st, &pb_emlrtRTEI,
            "shared_tracking:trackHistoryLogic:alreadyInitialized",
            "shared_tracking:trackHistoryLogic:alreadyInitialized", 0);
        }

        bv[0] = true;
        for (i = 0; i < 49; i++) {
          bv[i + 1] = c_obj->pRecentHistory[i];
        }

        for (i = 0; i < 50; i++) {
          c_obj->pRecentHistory[i] = bv[i];
        }

        c_obj->pIsFirstUpdate = false;
        st.site = &fq_emlrtRSI;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &x_emlrtDCI, &st);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &nd_emlrtBCI, &st);
        }

        c_obj = obj->pTrackLogics[nz];
        if (c_obj->pIsFirstUpdate) {
          tf = false;
        } else {
          boolean_T x_idx_1;
          boolean_T x_idx_2;
          tf = c_obj->pRecentHistory[0];
          x_idx_1 = c_obj->pRecentHistory[1];
          x_idx_2 = c_obj->pRecentHistory[2];
          tf = ((tf + x_idx_1) + x_idx_2 >= 2);
        }

        if (tf) {
          tf = true;
        } else {
          if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
              (obj->pNumLiveTracks)) {
            emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &w_emlrtDCI,
              (emlrtConstCTX)sp);
          }

          nz = (int32_T)obj->pNumLiveTracks - 1;
          if ((nz < 0) || (nz > 99)) {
            emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &md_emlrtBCI,
              (emlrtConstCTX)sp);
          }

          if (obj->pTracksList[nz].ObjectClassID > 0.0) {
            tf = true;
          } else {
            tf = false;
          }
        }

        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &qb_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        b_obj = obj->pTracksList[nz];
        b_obj.IsConfirmed = tf;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &je_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pTracksList[nz] = b_obj;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &v_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        if (((int32_T)obj->pNumLiveTracks < 1) || ((int32_T)obj->pNumLiveTracks >
             100)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)obj->pNumLiveTracks, 1, 100,
            &ld_emlrtBCI, (emlrtConstCTX)sp);
        }

        obj->pTrackIDs[(int32_T)obj->pNumLiveTracks - 1] = obj->pLastTrackID;
        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &t_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        if (((int32_T)obj->pNumLiveTracks < 1) || ((int32_T)obj->pNumLiveTracks >
             100)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)obj->pNumLiveTracks, 1, 100,
            &jd_emlrtBCI, (emlrtConstCTX)sp);
        }

        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &u_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &kd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        obj->pConfirmedTracks[(int32_T)obj->pNumLiveTracks - 1] =
          obj->pTracksList[nz].IsConfirmed;
        memset(&costMatrix_data[0], 0, (uint32_T)unassignedlocalTracks_size *
               sizeof(real_T));
        for (i = 0; i < allSourceInds_size; i++) {
          if (allSourceInds_data[i] == localTracks_data[thisConfig_tmp].
              SourceIndex) {
            if (i > unassignedlocalTracks_size - 1) {
              emlrtDynamicBoundsCheckR2012b(i, 0, unassignedlocalTracks_size - 1,
                &id_emlrtBCI, (emlrtConstCTX)sp);
            }

            costMatrix_data[i] = rtInf;
          }
        }

        trueCount = 0;
        for (i = 0; i < allSourceInds_size; i++) {
          if (allSourceInds_data[i] != localTracks_data[thisConfig_tmp].
              SourceIndex) {
            trueCount++;
          }
        }

        j = 0;
        for (i = 0; i < allSourceInds_size; i++) {
          if (allSourceInds_data[i] != localTracks_data[thisConfig_tmp].
              SourceIndex) {
            tmp_data[j] = (uint8_T)i;
            j++;
          }
        }

        if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
            (obj->pNumLiveTracks)) {
          emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &s_emlrtDCI,
            (emlrtConstCTX)sp);
        }

        nz = (int32_T)obj->pNumLiveTracks - 1;
        if ((nz < 0) || (nz > 99)) {
          emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &hd_emlrtBCI, (emlrtConstCTX)
            sp);
        }

        b_obj = obj->pTracksList[nz];
        for (i = 0; i < trueCount; i++) {
          u = tmp_data[i];
          if (u > unassignedlocalTracks_size - 1) {
            emlrtDynamicBoundsCheckR2012b(u, 0, unassignedlocalTracks_size - 1,
              &gd_emlrtBCI, (emlrtConstCTX)sp);
          }

          checkedUnassigned_data[i] = unassignedlocalTracks_data[u];
        }

        st.site = &gq_emlrtRSI;
        c_trackFuser_distanceToCentralT(&st, obj, localTracks, b_obj.TrackID,
          b_obj.BranchID, b_obj.Age, b_obj.ObjectClassID,
          b_obj.ObjectClassProbabilities, b_obj.IsConfirmed, b_obj.IsCoasted,
          b_obj.IsSelfReported, b_obj.pState, b_obj.pStateCovariance,
          b_obj.pUpdateTime, checkedUnassigned_data, trueCount, b_tmp_data,
          tmp_size);
        if (trueCount != tmp_size[1]) {
          emlrtSubAssignSizeCheck1dR2017a(trueCount, tmp_size[1], &d_emlrtECI,
            (emlrtConstCTX)sp);
        }

        for (i = 0; i < trueCount; i++) {
          u = tmp_data[i];
          if (u > unassignedlocalTracks_size - 1) {
            emlrtDynamicBoundsCheckR2012b(u, 0, unassignedlocalTracks_size - 1,
              &fd_emlrtBCI, (emlrtConstCTX)sp);
          }

          costMatrix_data[u] = b_tmp_data[i];
        }

        trueCount = unassignedlocalTracks_size;
        tmp_size[0] = 1;
        tmp_size[1] = unassignedlocalTracks_size;
        for (i = 0; i < unassignedlocalTracks_size; i++) {
          checkedUnassigned_data[i] = unassignedlocalTracks_data[i];
          assignedToNewTrack_data[i] = (costMatrix_data[i] < 100.0);
        }

        st.site = &hq_emlrtRSI;
        if (any(assignedToNewTrack_data, tmp_size)) {
          uint32_T a_data[400];
          st.site = &iq_emlrtRSI;
          b_st.site = &wq_emlrtRSI;
          c_st.site = &bg_emlrtRSI;
          nz = combineVectorElements(assignedToNewTrack_data, tmp_size);
          st.site = &jq_emlrtRSI;
          b_i = obj->pNumLiveTracks;
          b_st.site = &jk_emlrtRSI;
          if (nz < 0) {
            emlrtNonNegativeCheckR2012b(nz, &cb_emlrtDCI, &st);
          }

          trueCount = 0;
          j = 0;
          for (i = 0; i < unassignedlocalTracks_size; i++) {
            if (assignedToNewTrack_data[i]) {
              trueCount++;
              c_tmp_data[j] = (uint8_T)i;
              j++;
            }
          }

          st.site = &kq_emlrtRSI;
          for (i = 0; i < trueCount; i++) {
            u = c_tmp_data[i];
            if (u > unassignedlocalTracks_size - 1) {
              emlrtDynamicBoundsCheckR2012b(u, 0, unassignedlocalTracks_size - 1,
                &ed_emlrtBCI, &st);
            }

            checkedUnassigned_data[i] = unassignedlocalTracks_data[u];
          }

          b_st.site = &ar_emlrtRSI;
          c_st.site = &br_emlrtRSI;
          if (nz != trueCount) {
            emlrtErrorWithMessageIdR2018a(&c_st, &ob_emlrtRTEI,
              "MATLAB:catenate:matrixDimensionMismatch",
              "MATLAB:catenate:matrixDimensionMismatch", 0);
          }

          tmp_size[0] = nz;
          tmp_size[1] = 2;
          if (nz - 1 >= 0) {
            b_i = muDoubleScalarRound(b_i);
            if (b_i < 4.294967296E+9) {
              if (b_i >= 0.0) {
                u1 = (uint32_T)b_i;
              } else {
                u1 = 0U;
              }
            } else if (b_i >= 4.294967296E+9) {
              u1 = MAX_uint32_T;
            } else {
              u1 = 0U;
            }
          }

          for (i = 0; i < nz; i++) {
            a_data[i] = u1;
          }

          for (i = 0; i < trueCount; i++) {
            a_data[i + nz] = checkedUnassigned_data[i];
          }

          st.site = &kq_emlrtRSI;
          trackFuser_fuseAssigned(&st, obj, localTracks, a_data, tmp_size);
          trueCount = 0;
          for (i = 0; i < unassignedlocalTracks_size; i++) {
            if (!assignedToNewTrack_data[i]) {
              trueCount++;
            }
          }

          nz = 0;
          for (i = 0; i < unassignedlocalTracks_size; i++) {
            if (!assignedToNewTrack_data[i]) {
              if (i > unassignedlocalTracks_size - 1) {
                emlrtDynamicBoundsCheckR2012b(i, 0, unassignedlocalTracks_size -
                  1, &dd_emlrtBCI, (emlrtConstCTX)sp);
              }

              checkedUnassigned_data[nz] = unassignedlocalTracks_data[i];
              nz++;
            }
          }

          nz = 0;
          for (i = 0; i < unassignedlocalTracks_size; i++) {
            if (!assignedToNewTrack_data[i]) {
              nz++;
            }
          }

          j = 0;
          for (i = 0; i < unassignedlocalTracks_size; i++) {
            if (!assignedToNewTrack_data[i]) {
              if (i > allSourceInds_size - 1) {
                emlrtDynamicBoundsCheckR2012b(i, 0, allSourceInds_size - 1,
                  &cd_emlrtBCI, (emlrtConstCTX)sp);
              }

              allSourceInds_data[j] = allSourceInds_data[i];
              j++;
            }
          }

          allSourceInds_size = nz;
          if (isr) {
            tf = true;
          } else {
            if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
                (obj->pNumLiveTracks)) {
              emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &r_emlrtDCI,
                (emlrtConstCTX)sp);
            }

            nz = (int32_T)obj->pNumLiveTracks - 1;
            if ((nz < 0) || (nz > 99)) {
              emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &bd_emlrtBCI,
                (emlrtConstCTX)sp);
            }

            if (obj->pTracksList[nz].IsSelfReported) {
              tf = true;
            } else {
              tf = false;
            }
          }

          if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor
              (obj->pNumLiveTracks)) {
            emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &rb_emlrtDCI,
              (emlrtConstCTX)sp);
          }

          nz = (int32_T)obj->pNumLiveTracks - 1;
          b_obj = obj->pTracksList[nz];
          b_obj.IsSelfReported = tf;
          if ((nz < 0) || (nz > 99)) {
            emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ke_emlrtBCI,
              (emlrtConstCTX)sp);
          }

          obj->pTracksList[nz] = b_obj;
        }

        st.site = &mq_emlrtRSI;
        b_st.site = &fv_emlrtRSI;
        if (trueCount < 1) {
          emlrtErrorWithMessageIdR2018a(&b_st, &qb_emlrtRTEI,
            "MATLAB:subsdeldimmismatch", "MATLAB:subsdeldimmismatch", 0);
        }

        b_st.site = &gv_emlrtRSI;
        c_st.site = &hv_emlrtRSI;
        for (i = 0; i <= trueCount - 2; i++) {
          checkedUnassigned_data[i] = checkedUnassigned_data[i + 1];
        }

        if (trueCount - 1 > trueCount) {
          emlrtErrorWithMessageIdR2018a(&b_st, &rb_emlrtRTEI,
            "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed",
            0);
        }

        j = trueCount - 1;
        st.site = &lq_emlrtRSI;
        b_st.site = &fv_emlrtRSI;
        if (allSourceInds_size < 1) {
          emlrtErrorWithMessageIdR2018a(&b_st, &qb_emlrtRTEI,
            "MATLAB:subsdeldimmismatch", "MATLAB:subsdeldimmismatch", 0);
        }

        b_st.site = &gv_emlrtRSI;
        c_st.site = &hv_emlrtRSI;
        for (i = 0; i <= allSourceInds_size - 2; i++) {
          allSourceInds_data[i] = allSourceInds_data[i + 1];
        }

        if (allSourceInds_size - 1 > allSourceInds_size) {
          emlrtErrorWithMessageIdR2018a(&b_st, &rb_emlrtRTEI,
            "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed",
            0);
        }

        nz = allSourceInds_size - 1;
        allSourceInds_size--;
        if (nz != 0) {
          unassignedlocalTracks_size = trueCount - 1;
          if (j - 1 >= 0) {
            memcpy(&unassignedlocalTracks_data[0], &checkedUnassigned_data[0],
                   (uint32_T)j * sizeof(uint32_T));
          }
        } else {
          exitg1 = true;
        }
      }
    } else {
      st.site = &nq_emlrtRSI;
      e_warning(&st);
      exitg1 = true;
    }
  }
}

static void trackFuser_assign(const emlrtStack *sp, trackFuser *obj, const
  emxArray_real_T *costMatrix, emxArray_uint32_T *overallAssignments,
  emxArray_uint32_T *overallUnassignedCentralTracks, uint32_T
  c_overallUnassignedLocalTracks_[], int32_T *d_overallUnassignedLocalTracks_)
{
  jmp_buf * volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_boolean_T *b_obj;
  emxArray_int32_T *ib;
  emxArray_int32_T *ii;
  emxArray_int32_T *r3;
  emxArray_real_T *b_costMatrix;
  emxArray_real_T *y;
  emxArray_uint32_T f_overallUnassignedLocalTracks_;
  emxArray_uint32_T *varargout_1;
  emxArray_uint32_T *varargout_2;
  emxArray_uint32_T *varargout_3;
  emxArray_uint8_T *r1;
  emxArray_uint8_T *r2;
  real_T b_dv[2];
  const real_T *costMatrix_data;
  real_T numCentralTrks;
  real_T *y_data;
  int32_T b_i;
  int32_T b_i1;
  int32_T i;
  int32_T i2;
  int32_T s;
  int32_T trackFuser_assign_numThreads;
  int32_T *ii_data;
  uint32_T *c_overallUnassignedCentralTrack;
  uint32_T *overallAssignments_data;
  uint32_T *varargout_1_data;
  uint32_T *varargout_2_data;
  uint32_T *varargout_3_data;
  int8_T tmp_data[2];
  uint8_T b_tmp_data[200];
  uint8_T *r4;
  boolean_T *obj_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  costMatrix_data = costMatrix->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  numCentralTrks = obj->pNumLiveTracks;
  emxInit_real_T(sp, &y, 2, &rd_emlrtRTEI);
  y_data = y->data;
  if (costMatrix->size[1] == 0) {
    int32_T loop_ub;
    st.site = &vl_emlrtRSI;
    b_dv[0] = 0.0;
    b_dv[1] = 1.0;
    b_st.site = &jk_emlrtRSI;
    assertValidSizeArg(&b_st, b_dv);
    overallAssignments->size[0] = 0;
    overallAssignments->size[1] = 2;
    b_st.site = &wl_emlrtRSI;
    b_st.site = &wl_emlrtRSI;
    if (muDoubleScalarIsNaN(numCentralTrks)) {
      loop_ub = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(sp, y, loop_ub, &ad_emlrtRTEI);
      y_data = y->data;
      y_data[0] = rtNaN;
    } else if (numCentralTrks < 1.0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      int32_T vectorUB;
      loop_ub = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = (int32_T)(numCentralTrks - 1.0) + 1;
      emxEnsureCapacity_real_T(sp, y, loop_ub, &ad_emlrtRTEI);
      y_data = y->data;
      loop_ub = (int32_T)(numCentralTrks - 1.0);
      i2 = (((int32_T)(numCentralTrks - 1.0) + 1) / 2) << 1;
      vectorUB = i2 - 2;
      for (i = 0; i <= vectorUB; i += 2) {
        __m128d r;
        b_dv[0] = i;
        b_dv[1] = i + 1;
        r = _mm_loadu_pd(&b_dv[0]);
        _mm_storeu_pd(&y_data[i], _mm_add_pd(_mm_set1_pd(1.0), r));
      }

      for (i = i2; i <= loop_ub; i++) {
        y_data[i] = (real_T)i + 1.0;
      }
    }

    i2 = y->size[1];
    loop_ub = overallUnassignedCentralTracks->size[0];
    overallUnassignedCentralTracks->size[0] = y->size[1];
    emxEnsureCapacity_uint32_T(sp, overallUnassignedCentralTracks, loop_ub,
      &gd_emlrtRTEI);
    c_overallUnassignedCentralTrack = overallUnassignedCentralTracks->data;
    for (i = 0; i < i2; i++) {
      uint32_T lastAssigned;
      numCentralTrks = muDoubleScalarRound(y_data[i]);
      if (numCentralTrks < 4.294967296E+9) {
        if (numCentralTrks >= 0.0) {
          lastAssigned = (uint32_T)numCentralTrks;
        } else {
          lastAssigned = 0U;
        }
      } else if (numCentralTrks >= 4.294967296E+9) {
        lastAssigned = MAX_uint32_T;
      } else {
        lastAssigned = 0U;
      }

      c_overallUnassignedCentralTrack[i] = lastAssigned;
    }

    st.site = &ul_emlrtRSI;
    b_dv[0] = 0.0;
    b_dv[1] = 1.0;
    b_st.site = &jk_emlrtRSI;
    assertValidSizeArg(&b_st, b_dv);
    *d_overallUnassignedLocalTracks_ = 0;
    st.site = &tl_emlrtRSI;
    b_st.site = &r_emlrtRSI;
  } else {
    real_T d;
    int32_T b_loop_ub;
    int32_T c_loop_ub;
    int32_T i1;
    int32_T loop_ub;
    int32_T loop_ub_tmp;
    int32_T nz;
    int32_T trueCount;
    int32_T vectorUB;
    uint32_T e_overallUnassignedLocalTracks_[200];
    uint32_T lastAssigned;
    uint32_T lastUnassigned;
    boolean_T usedSources[2];
    st.site = &sl_emlrtRSI;
    b_st.site = &xl_emlrtRSI;
    usedSources[0] = false;
    usedSources[1] = false;
    i2 = 0;
    loop_ub = obj->pUsedConfigIDs->size[0];
    for (i = 0; i < 2; i++) {
      boolean_T exitg1;
      vectorUB = i2 + loop_ub;
      i1 = i2 + 1;
      i2 = vectorUB;
      c_st.site = &xc_emlrtRSI;
      if ((i1 <= vectorUB) && (vectorUB > 2147483646)) {
        d_st.site = &tb_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }

      exitg1 = false;
      while ((!exitg1) && (i1 <= vectorUB)) {
        if (obj->pUsedConfigIDs->data[i1 - 1]) {
          usedSources[i] = true;
          exitg1 = true;
        } else {
          i1++;
        }
      }
    }

    trueCount = 0;
    if (usedSources[0]) {
      trueCount = 1;
    }

    if (usedSources[1]) {
      trueCount++;
    }

    loop_ub = 0;
    if (usedSources[0]) {
      tmp_data[0] = 0;
      loop_ub = 1;
    }

    if (usedSources[1]) {
      tmp_data[loop_ub] = 1;
    }

    nz = usedSources[0] + usedSources[1];
    d = numCentralTrks * (real_T)nz;
    if (!(d >= 0.0)) {
      emlrtNonNegativeCheckR2012b(d, &p_emlrtDCI, (emlrtConstCTX)sp);
    }

    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &q_emlrtDCI, (emlrtConstCTX)sp);
    }

    loop_ub_tmp = (int32_T)d;
    b_loop_ub = (int32_T)d;
    loop_ub = overallAssignments->size[0] * overallAssignments->size[1];
    overallAssignments->size[0] = (int32_T)d;
    overallAssignments->size[1] = 2;
    emxEnsureCapacity_uint32_T(sp, overallAssignments, loop_ub, &hd_emlrtRTEI);
    overallAssignments_data = overallAssignments->data;
    loop_ub = (int32_T)d << 1;
    for (b_i = 0; b_i < loop_ub; b_i++) {
      overallAssignments_data[b_i] = 0U;
    }

    c_loop_ub = costMatrix->size[1];
    *d_overallUnassignedLocalTracks_ = costMatrix->size[1];
    for (i = 0; i < c_loop_ub; i++) {
      c_overallUnassignedLocalTracks_[i] = 0U;
    }

    if (muDoubleScalarIsNaN(numCentralTrks)) {
      loop_ub = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(sp, y, loop_ub, &ad_emlrtRTEI);
      y_data = y->data;
      y_data[0] = rtNaN;
    } else if (numCentralTrks < 1.0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      loop_ub = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = (int32_T)(numCentralTrks - 1.0) + 1;
      emxEnsureCapacity_real_T(sp, y, loop_ub, &ad_emlrtRTEI);
      y_data = y->data;
      loop_ub = (int32_T)(numCentralTrks - 1.0);
      vectorUB = (((int32_T)(numCentralTrks - 1.0) + 1) / 2) << 1;
      i2 = vectorUB - 2;
      for (i = 0; i <= i2; i += 2) {
        __m128d r;
        b_dv[0] = i;
        b_dv[1] = i + 1;
        r = _mm_loadu_pd(&b_dv[0]);
        _mm_storeu_pd(&y_data[i], _mm_add_pd(_mm_set1_pd(1.0), r));
      }

      for (i = vectorUB; i <= loop_ub; i++) {
        y_data[i] = (real_T)i + 1.0;
      }
    }

    vectorUB = y->size[1];
    loop_ub = overallUnassignedCentralTracks->size[0];
    overallUnassignedCentralTracks->size[0] = y->size[1];
    emxEnsureCapacity_uint32_T(sp, overallUnassignedCentralTracks, loop_ub,
      &id_emlrtRTEI);
    c_overallUnassignedCentralTrack = overallUnassignedCentralTracks->data;
    for (i = 0; i < vectorUB; i++) {
      numCentralTrks = muDoubleScalarRound(y_data[i]);
      if (numCentralTrks < 4.294967296E+9) {
        if (numCentralTrks >= 0.0) {
          lastAssigned = (uint32_T)numCentralTrks;
        } else {
          lastAssigned = 0U;
        }
      } else if (numCentralTrks >= 4.294967296E+9) {
        lastAssigned = MAX_uint32_T;
      } else {
        lastAssigned = 0U;
      }

      c_overallUnassignedCentralTrack[i] = lastAssigned;
    }

    lastAssigned = 0U;
    lastUnassigned = 0U;
    emxInit_uint8_T(sp, &r1, 2, &nd_emlrtRTEI);
    emxInit_uint8_T(sp, &r2, 1, &pd_emlrtRTEI);
    emxInit_int32_T(sp, &ii, 1, &sd_emlrtRTEI);
    emxInit_int32_T(sp, &ib, 1, &td_emlrtRTEI);
    emxInit_uint32_T(sp, &varargout_3, 1, &qd_emlrtRTEI);
    emxInit_uint32_T(sp, &varargout_2, 1, &ud_emlrtRTEI);
    emxInit_uint32_T(sp, &varargout_1, 2, &md_emlrtRTEI);
    emxInit_boolean_T(sp, &b_obj, 1, &jd_emlrtRTEI, true);
    emxInit_real_T(sp, &b_costMatrix, 2, &kd_emlrtRTEI);
    for (s = 0; s < nz; s++) {
      int32_T d_loop_ub;
      st.site = &rl_emlrtRSI;
      if (s + 1 > trueCount) {
        emlrtDynamicBoundsCheckR2012b(s + 1, 1, trueCount, &wb_emlrtBCI, &st);
      }

      vectorUB = tmp_data[s];
      i2 = obj->pUsedConfigIDs->size[0];
      loop_ub = b_obj->size[0];
      b_obj->size[0] = i2;
      emxEnsureCapacity_boolean_T(&st, b_obj, loop_ub, &jd_emlrtRTEI);
      obj_data = b_obj->data;
      for (i = 0; i < i2; i++) {
        obj_data[i] = obj->pUsedConfigIDs->data[i + obj->pUsedConfigIDs->size[0]
          * vectorUB];
      }

      b_st.site = &yl_emlrtRSI;
      eml_find(&b_st, b_obj, ii);
      ii_data = ii->data;
      d_loop_ub = ii->size[0];
      for (i = 0; i < d_loop_ub; i++) {
        loop_ub = ii_data[i];
        if ((loop_ub < 1) || (loop_ub > c_loop_ub)) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 1, c_loop_ub, &ec_emlrtBCI,
            (emlrtConstCTX)sp);
        }
      }

      st.site = &ql_emlrtRSI;
      if (obj->cAssigner.isInitialized == 2) {
        emlrtErrorWithMessageIdR2018a(&st, &g_emlrtRTEI,
          "MATLAB:system:methodCalledWhenReleasedCodegen",
          "MATLAB:system:methodCalledWhenReleasedCodegen", 3, 4, 4, "step");
      }

      if (obj->cAssigner.isInitialized != 1) {
        b_st.site = &vb_emlrtRSI;
        c_st.site = &em_emlrtRSI;
        if (obj->cAssigner.isInitialized != 0) {
          emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
            "MATLAB:system:methodCalledWhenLockedReleasedCodegen",
            "MATLAB:system:methodCalledWhenLockedReleasedCodegen", 3, 4, 5,
            "setup");
        }

        obj->cAssigner.isInitialized = 1;
        obj->cAssigner.isSetupComplete = true;
        c_st.site = &em_emlrtRSI;
        obj->cAssigner.pCostOfNonAssignment = obj->
          cAssigner.AssignmentThreshold[0] / 2.0;
      }

      vectorUB = costMatrix->size[0];
      loop_ub = b_costMatrix->size[0] * b_costMatrix->size[1];
      b_costMatrix->size[0] = costMatrix->size[0];
      b_costMatrix->size[1] = ii->size[0];
      emxEnsureCapacity_real_T(&st, b_costMatrix, loop_ub, &kd_emlrtRTEI);
      y_data = b_costMatrix->data;
      for (i = 0; i < d_loop_ub; i++) {
        for (b_i = 0; b_i < vectorUB; b_i++) {
          y_data[b_i + b_costMatrix->size[0] * i] = costMatrix_data[b_i +
            costMatrix->size[0] * (ii_data[i] - 1)];
        }
      }

      b_st.site = &vb_emlrtRSI;
      AssignerGNN_stepImpl(&b_st, &obj->cAssigner, b_costMatrix, varargout_1,
                           varargout_2, varargout_3);
      varargout_3_data = varargout_3->data;
      varargout_2_data = varargout_2->data;
      varargout_1_data = varargout_1->data;
      st.site = &pl_emlrtRSI;
      if (varargout_1->size[0] != 0) {
        int32_T varargout_1_tmp;
        numCentralTrks = (real_T)lastAssigned + (real_T)varargout_1->size[0];
        if ((real_T)lastAssigned + 1.0 > numCentralTrks) {
          vectorUB = 1;
          loop_ub = 0;
        } else {
          if (((int32_T)(lastAssigned + 1U) < 1) || ((int32_T)(lastAssigned + 1U)
               > (int32_T)d)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(lastAssigned + 1U), 1,
              (int32_T)d, &ac_emlrtBCI, (emlrtConstCTX)sp);
          }

          vectorUB = (int32_T)(lastAssigned + 1U);
          if (((int32_T)numCentralTrks < 1) || ((int32_T)numCentralTrks >
               (int32_T)d)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)numCentralTrks, 1, (int32_T)d,
              &bc_emlrtBCI, (emlrtConstCTX)sp);
          }

          loop_ub = (int32_T)numCentralTrks;
        }

        loop_ub -= vectorUB;
        i2 = loop_ub + 1;
        varargout_1_tmp = varargout_1->size[0];
        emlrtSubAssignSizeCheckR2012b(&i2, 1, &varargout_1->size[0], 1,
          &b_emlrtECI, (emlrtCTX)sp);
        for (i = 0; i <= loop_ub; i++) {
          overallAssignments_data[(vectorUB + i) - 1] = varargout_1_data[i];
        }

        if ((real_T)lastAssigned + 1.0 > numCentralTrks) {
          i1 = 1;
          vectorUB = 0;
        } else {
          if (((int32_T)(lastAssigned + 1U) < 1) || ((int32_T)(lastAssigned + 1U)
               > (int32_T)d)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(lastAssigned + 1U), 1,
              (int32_T)d, &cc_emlrtBCI, (emlrtConstCTX)sp);
          }

          i1 = (int32_T)(lastAssigned + 1U);
          if (((int32_T)numCentralTrks < 1) || ((int32_T)numCentralTrks >
               (int32_T)d)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)numCentralTrks, 1, (int32_T)d,
              &dc_emlrtBCI, (emlrtConstCTX)sp);
          }

          vectorUB = (int32_T)numCentralTrks;
        }

        loop_ub = r2->size[0];
        r2->size[0] = varargout_1->size[0];
        emxEnsureCapacity_uint8_T(sp, r2, loop_ub, &pd_emlrtRTEI);
        r4 = r2->data;
        for (i = 0; i < varargout_1_tmp; i++) {
          if (((int32_T)varargout_1_data[i + varargout_1->size[0]] < 1) ||
              ((int32_T)varargout_1_data[i + varargout_1->size[0]] > d_loop_ub))
          {
            emlrtDynamicBoundsCheckR2012b((int32_T)varargout_1_data[i +
              varargout_1->size[0]], 1, d_loop_ub, &ic_emlrtBCI, (emlrtConstCTX)
              sp);
          }

          r4[i] = (uint8_T)ii_data[(int32_T)varargout_1_data[i +
            varargout_1->size[0]] - 1];
        }

        loop_ub = vectorUB - i1;
        i2 = loop_ub + 1;
        emlrtSubAssignSizeCheckR2012b(&i2, 1, &r2->size[0], 1, &c_emlrtECI,
          (emlrtCTX)sp);
        for (i = 0; i <= loop_ub; i++) {
          overallAssignments_data[((i1 + i) + overallAssignments->size[0]) - 1] =
            r4[i];
        }

        lastAssigned += (uint32_T)varargout_1->size[0];
      }

      st.site = &ol_emlrtRSI;
      vectorUB = varargout_3->size[0];
      if (varargout_3->size[0] != 0) {
        numCentralTrks = (real_T)lastUnassigned + (real_T)varargout_3->size[0];
        if ((real_T)lastUnassigned + 1.0 > numCentralTrks) {
          i1 = 0;
          i2 = 0;
        } else {
          if (((int32_T)(lastUnassigned + 1U) < 1) || ((int32_T)(lastUnassigned
                + 1U) > c_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)(lastUnassigned + 1U), 1,
              c_loop_ub, &xb_emlrtBCI, (emlrtConstCTX)sp);
          }

          i1 = (int32_T)lastUnassigned;
          if (((int32_T)numCentralTrks < 1) || ((int32_T)numCentralTrks >
               c_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)numCentralTrks, 1, c_loop_ub,
              &yb_emlrtBCI, (emlrtConstCTX)sp);
          }

          i2 = (int32_T)numCentralTrks;
        }

        loop_ub = r1->size[0] * r1->size[1];
        r1->size[0] = 1;
        r1->size[1] = varargout_3->size[0];
        emxEnsureCapacity_uint8_T(sp, r1, loop_ub, &nd_emlrtRTEI);
        r4 = r1->data;
        for (i = 0; i < vectorUB; i++) {
          if (((int32_T)varargout_3_data[i] < 1) || ((int32_T)varargout_3_data[i]
               > d_loop_ub)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)varargout_3_data[i], 1,
              d_loop_ub, &hc_emlrtBCI, (emlrtConstCTX)sp);
          }

          r4[i] = (uint8_T)ii_data[(int32_T)varargout_3_data[i] - 1];
        }

        loop_ub = i2 - i1;
        vectorUB = r1->size[1];
        if (loop_ub != r1->size[1]) {
          emlrtSubAssignSizeCheck1dR2017a(loop_ub, r1->size[1], &emlrtECI,
            (emlrtConstCTX)sp);
        }

        for (i = 0; i < vectorUB; i++) {
          c_overallUnassignedLocalTracks_[i1 + i] = r4[i];
        }

        lastUnassigned += (uint32_T)varargout_3->size[0];
      }

      st.site = &nl_emlrtRSI;
      loop_ub = varargout_2->size[0];
      if (varargout_2->size[0] != 0) {
        st.site = &ml_emlrtRSI;
        b_st.site = &lo_emlrtRSI;
        sort(&b_st, varargout_2);
        st.site = &ml_emlrtRSI;
        b_st.site = &mp_emlrtRSI;
        loop_ub = varargout_3->size[0];
        varargout_3->size[0] = overallUnassignedCentralTracks->size[0];
        emxEnsureCapacity_uint32_T(&b_st, varargout_3, loop_ub, &qd_emlrtRTEI);
        varargout_3_data = varargout_3->data;
        loop_ub = overallUnassignedCentralTracks->size[0] - 1;
        for (i = 0; i <= loop_ub; i++) {
          varargout_3_data[i] = c_overallUnassignedCentralTrack[i];
        }

        c_st.site = &np_emlrtRSI;
        do_vectors(&c_st, varargout_3, varargout_2,
                   overallUnassignedCentralTracks, ii, ib);
        c_overallUnassignedCentralTrack = overallUnassignedCentralTracks->data;
      } else {
        overallUnassignedCentralTracks->size[0] = 0;
        for (i = 0; i < loop_ub; i++) {
          c_overallUnassignedCentralTrack[i] = varargout_2_data[i];
        }
      }
    }

    emxFree_real_T(sp, &b_costMatrix);
    emxFree_uint32_T(sp, &varargout_3);
    emxFree_int32_T(sp, &ib);
    emxFree_int32_T(sp, &ii);
    emxFree_uint8_T(sp, &r2);
    emxFree_uint8_T(sp, &r1);
    vectorUB = 0;
    loop_ub = 0;
    for (i = 0; i < c_loop_ub; i++) {
      if (c_overallUnassignedLocalTracks_[i] > 0U) {
        vectorUB++;
      }

      if (c_overallUnassignedLocalTracks_[i] > 0U) {
        b_tmp_data[loop_ub] = (uint8_T)i;
        loop_ub++;
      }
    }

    for (i = 0; i < vectorUB; i++) {
      uint8_T u;
      u = b_tmp_data[i];
      if (u > *d_overallUnassignedLocalTracks_ - 1) {
        emlrtDynamicBoundsCheckR2012b(u, 0, *d_overallUnassignedLocalTracks_ - 1,
          &fc_emlrtBCI, (emlrtConstCTX)sp);
      }

      e_overallUnassignedLocalTracks_[i] = c_overallUnassignedLocalTracks_[u];
    }

    *d_overallUnassignedLocalTracks_ = vectorUB;
    for (i = 0; i < vectorUB; i++) {
      c_overallUnassignedLocalTracks_[i] = e_overallUnassignedLocalTracks_[i];
    }

    st.site = &ll_emlrtRSI;
    f_overallUnassignedLocalTracks_.data = &c_overallUnassignedLocalTracks_[0];
    f_overallUnassignedLocalTracks_.size = d_overallUnassignedLocalTracks_;
    f_overallUnassignedLocalTracks_.allocatedSize = -1;
    f_overallUnassignedLocalTracks_.numDimensions = 1;
    f_overallUnassignedLocalTracks_.canFreeData = false;
    b_st.site = &ib_emlrtRSI;
    b_unique_vector(&b_st, &f_overallUnassignedLocalTracks_, varargout_2);
    varargout_2_data = varargout_2->data;
    i2 = varargout_2->size[0];
    *d_overallUnassignedLocalTracks_ = varargout_2->size[0];
    for (i = 0; i < i2; i++) {
      c_overallUnassignedLocalTracks_[i] = varargout_2_data[i];
    }

    emxFree_uint32_T(sp, &varargout_2);
    loop_ub = b_obj->size[0];
    b_obj->size[0] = (int32_T)d;
    emxEnsureCapacity_boolean_T(sp, b_obj, loop_ub, &ld_emlrtRTEI);
    obj_data = b_obj->data;
    i2 = (int32_T)d;
    if (overallAssignments->size[0] < 800) {
      for (b_i1 = 0; b_i1 < loop_ub_tmp; b_i1++) {
        obj_data[b_i1] = (overallAssignments_data[b_i1] > 0U);
      }
    } else {
      emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
      emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
      trackFuser_assign_numThreads = emlrtAllocRegionTLSs(sp->tls,
        omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());

#pragma omp parallel for \
 num_threads(trackFuser_assign_numThreads)

      for (b_i1 = 0; b_i1 < i2; b_i1++) {
        obj_data[b_i1] = (overallAssignments_data[b_i1] > 0U);
      }

      emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
      emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
    }

    i2 = 0;
    for (i = 0; i < b_loop_ub; i++) {
      if (obj_data[i]) {
        i2++;
      }
    }

    emxInit_int32_T(sp, &r3, 1, &ld_emlrtRTEI);
    loop_ub = r3->size[0];
    r3->size[0] = i2;
    emxEnsureCapacity_int32_T(sp, r3, loop_ub, &ld_emlrtRTEI);
    d_overallUnassignedLocalTracks_ = r3->data;
    loop_ub = 0;
    for (i = 0; i < b_loop_ub; i++) {
      if (obj_data[i]) {
        d_overallUnassignedLocalTracks_[loop_ub] = i;
        loop_ub++;
      }
    }

    emxFree_boolean_T(sp, &b_obj);
    i2 = r3->size[0];
    loop_ub = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = r3->size[0];
    varargout_1->size[1] = 2;
    emxEnsureCapacity_uint32_T(sp, varargout_1, loop_ub, &md_emlrtRTEI);
    varargout_1_data = varargout_1->data;
    for (i = 0; i < 2; i++) {
      for (b_i = 0; b_i < i2; b_i++) {
        if (d_overallUnassignedLocalTracks_[b_i] > overallAssignments->size[0] -
            1) {
          emlrtDynamicBoundsCheckR2012b(d_overallUnassignedLocalTracks_[b_i], 0,
            overallAssignments->size[0] - 1, &gc_emlrtBCI, (emlrtConstCTX)sp);
        }

        varargout_1_data[b_i + varargout_1->size[0] * i] =
          overallAssignments_data[d_overallUnassignedLocalTracks_[b_i] +
          overallAssignments->size[0] * i];
      }
    }

    emxFree_int32_T(sp, &r3);
    loop_ub = overallAssignments->size[0] * overallAssignments->size[1];
    overallAssignments->size[0] = varargout_1->size[0];
    overallAssignments->size[1] = 2;
    emxEnsureCapacity_uint32_T(sp, overallAssignments, loop_ub, &od_emlrtRTEI);
    overallAssignments_data = overallAssignments->data;
    loop_ub = varargout_1->size[0] << 1;
    for (i = 0; i < loop_ub; i++) {
      overallAssignments_data[i] = varargout_1_data[i];
    }

    emxFree_uint32_T(sp, &varargout_1);
  }

  emxFree_real_T(sp, &y);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void trackFuser_coastUnassigned(const emlrtStack *sp, trackFuser *obj,
  const emxArray_uint32_T *unassignedTracks, const emxArray_real_T *notUpdated,
  boolean_T toDelete[100])
{
  c_objectTrack temp;
  emlrtStack b_st;
  emlrtStack st;
  emxArray_uint32_T *b_unassignedTracks;
  emxArray_uint32_T *c_unassignedTracks;
  trackHistoryLogic *tempLogic;
  const real_T *notUpdated_data;
  real_T d;
  int32_T b_loop_ub;
  int32_T b_nz;
  int32_T i;
  int32_T idx;
  int32_T k;
  int32_T loop_ub;
  int32_T nz;
  const uint32_T *unassignedTracks_data;
  uint32_T q0;
  uint32_T *b_unassignedTracks_data;
  int8_T ii_data[100];
  boolean_T exitg1;
  boolean_T tentativeTrack;
  boolean_T x_idx_1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  notUpdated_data = notUpdated->data;
  unassignedTracks_data = unassignedTracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &iv_emlrtRSI;
  emxInit_uint32_T(&st, &b_unassignedTracks, 2, &rg_emlrtRTEI);
  nz = b_unassignedTracks->size[0] * b_unassignedTracks->size[1];
  b_unassignedTracks->size[0] = 1;
  b_unassignedTracks->size[1] = unassignedTracks->size[0] + notUpdated->size[1];
  emxEnsureCapacity_uint32_T(&st, b_unassignedTracks, nz, &rg_emlrtRTEI);
  b_unassignedTracks_data = b_unassignedTracks->data;
  nz = unassignedTracks->size[0];
  for (k = 0; k < nz; k++) {
    b_unassignedTracks_data[k] = unassignedTracks_data[k];
  }

  nz = notUpdated->size[1];
  for (k = 0; k < nz; k++) {
    d = muDoubleScalarRound(notUpdated_data[k]);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        q0 = (uint32_T)d;
      } else {
        q0 = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      q0 = MAX_uint32_T;
    } else {
      q0 = 0U;
    }

    b_unassignedTracks_data[k + unassignedTracks->size[0]] = q0;
  }

  emxInit_uint32_T(&st, &c_unassignedTracks, 2, &tg_emlrtRTEI);
  b_st.site = &ib_emlrtRSI;
  c_unique_vector(&b_st, b_unassignedTracks, c_unassignedTracks);
  b_unassignedTracks_data = c_unassignedTracks->data;
  emxFree_uint32_T(&st, &b_unassignedTracks);
  memset(&toDelete[0], 0, 100U * sizeof(boolean_T));
  loop_ub = c_unassignedTracks->size[1];
  for (i = 0; i < loop_ub; i++) {
    uint32_T qY;
    uint32_T u;
    boolean_T bv[50];
    boolean_T b;
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &wg_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    u = b_unassignedTracks_data[i];
    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &gc_emlrtDCI, (emlrtConstCTX)sp);
    }

    nz = (int32_T)b_unassignedTracks_data[i] - 1;
    temp = obj->pTracksList[nz];
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &yg_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &hc_emlrtDCI, (emlrtConstCTX)sp);
    }

    b = (((int32_T)u - 1 < 0) || ((int32_T)u - 1 > 99));
    if (b) {
      emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &xg_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    q0 = obj->pTracksList[(int32_T)u - 1].Age;
    qY = q0 + 1U;
    if (q0 + 1U < q0) {
      qY = MAX_uint32_T;
    }

    temp.Age = qY;
    if ((nz < 0) || (nz > 99)) {
      emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ah_emlrtBCI, (emlrtConstCTX)sp);
    }

    obj->pTracksList[nz] = temp;
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &bh_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &fc_emlrtDCI, (emlrtConstCTX)sp);
    }

    q0 = b_unassignedTracks_data[i];
    temp = obj->pTracksList[(int32_T)q0 - 1];
    temp.IsCoasted = true;
    if (((int32_T)q0 - 1 < 0) || ((int32_T)q0 - 1 > 99)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)q0 - 1, 0, 99, &ch_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    obj->pTracksList[(int32_T)q0 - 1] = temp;
    st.site = &jv_emlrtRSI;
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &eh_emlrtBCI, &st);
    }

    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &ic_emlrtDCI, &st);
    }

    if (b) {
      emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &dh_emlrtBCI, &st);
    }

    tempLogic = obj->pTrackLogics[(int32_T)u - 1];
    if (tempLogic->pIsFirstUpdate) {
      emlrtErrorWithMessageIdR2018a(&st, &ac_emlrtRTEI,
        "shared_tracking:trackHistoryLogic:notInitialized",
        "shared_tracking:trackHistoryLogic:notInitialized", 3, 4, 4, "miss");
    }

    bv[0] = false;
    for (k = 0; k < 49; k++) {
      bv[k + 1] = tempLogic->pRecentHistory[k];
    }

    for (k = 0; k < 50; k++) {
      tempLogic->pRecentHistory[k] = bv[k];
    }

    tempLogic->pIsFirstUpdate = false;
    st.site = &kv_emlrtRSI;
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &hh_emlrtBCI, &st);
    }

    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &jc_emlrtDCI, &st);
    }

    if (b) {
      emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &gh_emlrtBCI, &st);
    }

    tempLogic = obj->pTrackLogics[(int32_T)u - 1];
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &jh_emlrtBCI, &st);
    }

    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &kc_emlrtDCI, &st);
    }

    if (b) {
      emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &ih_emlrtBCI, &st);
    }

    tentativeTrack = !obj->pTracksList[(int32_T)u - 1].IsConfirmed;
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &lh_emlrtBCI, &st);
    }

    if ((real_T)u != (int32_T)u) {
      emlrtIntegerCheckR2012b(u, &lc_emlrtDCI, &st);
    }

    if (b) {
      emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &kh_emlrtBCI, &st);
    }

    q0 = obj->pTracksList[(int32_T)u - 1].Age;
    if (tempLogic->pIsFirstUpdate) {
      if (i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &nh_emlrtBCI, &st);
      }

      if (((int32_T)u < 1) || ((int32_T)u > 100)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, 100, &mh_emlrtBCI, &st);
      }

      toDelete[(int32_T)u - 1] = false;
    } else if (!tentativeTrack) {
      if (q0 > 2U) {
        tentativeTrack = !tempLogic->pRecentHistory[0];
        x_idx_1 = !tempLogic->pRecentHistory[1];
        if (i + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &nh_emlrtBCI, &st);
        }

        if (((int32_T)u < 1) || ((int32_T)u > 100)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, 100, &mh_emlrtBCI, &st);
        }

        toDelete[(int32_T)u - 1] = (tentativeTrack + x_idx_1 >= 2);
      } else {
        boolean_T x_data[200];
        b_loop_ub = (int32_T)q0;
        for (k = 0; k < b_loop_ub; k++) {
          x_data[k] = !tempLogic->pRecentHistory[k];
        }

        if ((int32_T)q0 == 0) {
          nz = 0;
        } else {
          nz = x_data[0];
          for (k = 2; k <= b_loop_ub; k++) {
            nz += x_data[1];
          }
        }

        if (i + 1 > loop_ub) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &nh_emlrtBCI, &st);
        }

        if (((int32_T)u < 1) || ((int32_T)u > 100)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, 100, &mh_emlrtBCI, &st);
        }

        if (nz < 0) {
          nz = 0;
        }

        toDelete[(int32_T)u - 1] = (nz >= 2);
      }
    } else {
      uint32_T b_qY;
      boolean_T x_idx_2;
      tentativeTrack = tempLogic->pRecentHistory[0];
      x_idx_1 = tempLogic->pRecentHistory[1];
      x_idx_2 = tempLogic->pRecentHistory[2];
      nz = (tentativeTrack + x_idx_1) + x_idx_2;
      if (i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &nh_emlrtBCI, &st);
      }

      if (((int32_T)u < 1) || ((int32_T)u > 100)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, 100, &mh_emlrtBCI, &st);
      }

      qY = 2U - (uint32_T)nz;
      if (2U - (uint32_T)nz > 2U) {
        qY = 0U;
      }

      b_qY = 3U - q0;
      if (3U - q0 > 3U) {
        b_qY = 0U;
      }

      toDelete[(int32_T)u - 1] = (qY > b_qY);
    }

    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &ph_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if (((int32_T)u < 1) || ((int32_T)u > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, 100, &oh_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    if (toDelete[(int32_T)u - 1]) {
      st.site = &lv_emlrtRSI;
      if (i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &vh_emlrtBCI, &st);
      }

      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &qc_emlrtDCI, &st);
      }

      if (b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &uh_emlrtBCI, &st);
      }

      tempLogic = obj->pTrackLogics[(int32_T)u - 1];
      for (k = 0; k < 50; k++) {
        tempLogic->pRecentHistory[k] = false;
      }

      tempLogic->pIsFirstUpdate = true;
    }
  }

  emxFree_uint32_T(sp, &c_unassignedTracks);
  st.site = &mv_emlrtRSI;
  b_nz = toDelete[0];
  for (k = 0; k < 99; k++) {
    b_nz += toDelete[k + 1];
  }

  idx = 0;
  nz = 0;
  exitg1 = false;
  while ((!exitg1) && (nz < 100)) {
    if (toDelete[nz]) {
      idx++;
      ii_data[idx - 1] = (int8_T)(nz + 1);
      if (idx >= 100) {
        exitg1 = true;
      } else {
        nz++;
      }
    } else {
      nz++;
    }
  }

  if (idx < 1) {
    idx = 0;
  }

  emlrtForLoopVectorCheckR2021a(b_nz, -1.0, 1.0, mxDOUBLE_CLASS, b_nz,
    &bc_emlrtRTEI, &st);
  for (i = 0; i < b_nz; i++) {
    uint32_T b_tmp_data[101];
    int8_T b_i;
    boolean_T tmp_data[101];
    nz = b_nz - i;
    if ((nz < 1) || (nz > idx)) {
      emlrtDynamicBoundsCheckR2012b(nz, 1, idx, &fh_emlrtBCI, &st);
    }

    b_i = ii_data[nz - 1];
    temp = obj->pTracksList[b_i - 1];
    tempLogic = obj->pTrackLogics[b_i - 1];
    d = obj->pNumLiveTracks;
    nz = (int32_T)(d + (1.0 - ((real_T)b_i + 1.0)));
    emlrtForLoopVectorCheckR2021a((real_T)b_i + 1.0, 1.0, d, mxDOUBLE_CLASS, nz,
      &cc_emlrtRTEI, &st);
    for (k = 0; k < nz; k++) {
      q0 = ((uint32_T)b_i + (uint32_T)k) + 1U;
      if ((real_T)q0 != (int32_T)q0) {
        emlrtIntegerCheckR2012b(q0, &mc_emlrtDCI, &st);
      }

      tentativeTrack = (((int32_T)q0 - 1 < 0) || ((int32_T)q0 - 1 > 99));
      if (tentativeTrack) {
        emlrtDynamicBoundsCheckR2012b((int32_T)q0 - 1, 0, 99, &qh_emlrtBCI, &st);
      }

      if ((real_T)q0 - 1.0 != (int32_T)((real_T)q0 - 1.0)) {
        emlrtIntegerCheckR2012b((real_T)q0 - 1.0, &nc_emlrtDCI, &st);
      }

      x_idx_1 = (((int32_T)((real_T)q0 - 1.0) - 1 < 0) || ((int32_T)((real_T)q0
        - 1.0) - 1 > 99));
      if (x_idx_1) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((real_T)q0 - 1.0) - 1, 0, 99,
          &rh_emlrtBCI, &st);
      }

      obj->pTracksList[(int32_T)((real_T)q0 - 1.0) - 1] = obj->pTracksList
        [(int32_T)q0 - 1];
      if ((real_T)q0 != (int32_T)q0) {
        emlrtIntegerCheckR2012b(q0, &oc_emlrtDCI, &st);
      }

      if (tentativeTrack) {
        emlrtDynamicBoundsCheckR2012b((int32_T)q0 - 1, 0, 99, &sh_emlrtBCI, &st);
      }

      if ((real_T)q0 - 1.0 != (int32_T)((real_T)q0 - 1.0)) {
        emlrtIntegerCheckR2012b((real_T)q0 - 1.0, &pc_emlrtDCI, &st);
      }

      if (x_idx_1) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((real_T)q0 - 1.0) - 1, 0, 99,
          &th_emlrtBCI, &st);
      }

      obj->pTrackLogics[(int32_T)((real_T)q0 - 1.0) - 1] = obj->pTrackLogics
        [(int32_T)q0 - 1];
    }

    if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor(obj->pNumLiveTracks))
    {
      emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &dc_emlrtDCI, &st);
    }

    nz = (int32_T)obj->pNumLiveTracks - 1;
    if ((nz < 0) || (nz > 99)) {
      emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &ug_emlrtBCI, &st);
    }

    obj->pTracksList[nz] = temp;
    if (obj->pNumLiveTracks != (int32_T)muDoubleScalarFloor(obj->pNumLiveTracks))
    {
      emlrtIntegerCheckR2012b(obj->pNumLiveTracks, &ec_emlrtDCI, &st);
    }

    nz = (int32_T)obj->pNumLiveTracks - 1;
    if ((nz < 0) || (nz > 99)) {
      emlrtDynamicBoundsCheckR2012b(nz, 0, 99, &vg_emlrtBCI, &st);
    }

    obj->pTrackLogics[nz] = tempLogic;
    obj->pNumLiveTracks--;
    if (b_i + 1 > 100) {
      b_loop_ub = 0;
      nz = 0;
    } else {
      if (b_i + 1 > 100) {
        emlrtDynamicBoundsCheckR2012b(101, 1, 100, &sg_emlrtBCI, &st);
      }

      b_loop_ub = b_i;
      nz = 100;
    }

    nz -= b_loop_ub;
    loop_ub = nz + 1;
    for (k = 0; k < nz; k++) {
      tmp_data[k] = obj->pConfirmedTracks[b_loop_ub + k];
    }

    tmp_data[nz] = false;
    if (101 - b_i != nz + 1) {
      emlrtSubAssignSizeCheck1dR2017a(101 - b_i, nz + 1, &e_emlrtECI, &st);
    }

    for (k = 0; k < loop_ub; k++) {
      obj->pConfirmedTracks[(b_i + k) - 1] = tmp_data[k];
    }

    if (b_i + 1 > 100) {
      b_loop_ub = 0;
      nz = 0;
    } else {
      if (b_i + 1 > 100) {
        emlrtDynamicBoundsCheckR2012b(101, 1, 100, &tg_emlrtBCI, &st);
      }

      b_loop_ub = b_i;
      nz = 100;
    }

    nz -= b_loop_ub;
    loop_ub = nz + 1;
    for (k = 0; k < nz; k++) {
      b_tmp_data[k] = obj->pTrackIDs[b_loop_ub + k];
    }

    b_tmp_data[nz] = 0U;
    if (101 - b_i != nz + 1) {
      emlrtSubAssignSizeCheck1dR2017a(101 - b_i, nz + 1, &f_emlrtECI, &st);
    }

    for (k = 0; k < loop_ub; k++) {
      obj->pTrackIDs[(b_i + k) - 1] = b_tmp_data[k];
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void trackFuser_fuseAssigned(const emlrtStack *sp, trackFuser *obj, const
  emxArray_struct0_T *localTracks, const uint32_T assignments_data[], const
  int32_T assignments_size[2])
{
  b_objectTrack b_expl_temp;
  b_objectTrack jw_emlrtRSI;
  b_objectTrack kw_emlrtRSI;
  c_objectTrack b_obj;
  c_objectTrack *r1;
  c_objectTrack *transformedTracks_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_boolean_T b_inAssigned_data;
  emxArray_boolean_T *toFuse;
  emxArray_int32_T *ii;
  emxArray_objectTrack *r;
  emxArray_objectTrack *transformedTracks;
  emxArray_real_T c_ii_data;
  emxArray_struct1_T *otherAttributes;
  emxArray_uint32_T c_assignments_data;
  emxArray_uint32_T d_assignments_data;
  emxArray_uint32_T *b;
  fuserSourceConfiguration *thisConfig;
  const struct0_T *localTracks_data;
  struct1_T *otherAttributes_data;
  trackHistoryLogic *c_obj;
  real_T b_ii_data[200];
  real_T b_dv[2];
  int32_T b_assignments_size;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T i;
  int32_T ii_size;
  int32_T inAssigned_size;
  int32_T j;
  int32_T loop_ub;
  int32_T nz;
  int32_T updated_size_idx_1;
  int32_T *ii_data;
  uint32_T b_assignments_data[200];
  uint32_T *b_data;
  uint8_T b_tmp_data[200];
  uint8_T tmp_data[200];
  boolean_T inAssigned_data[200];
  boolean_T *toFuse_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  localTracks_data = localTracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj->pTracksList[99];
  b_dv[0] = 1.0;
  b_dv[1] = assignments_size[0];
  emxInit_objectTrack(sp, &r, &wf_emlrtRTEI);
  st.site = &cr_emlrtRSI;
  repmat(&st, &b_obj, b_dv, r);
  r1 = r->data;
  emxInit_objectTrack(sp, &transformedTracks, &lf_emlrtRTEI);
  nz = transformedTracks->size[0] * transformedTracks->size[1];
  transformedTracks->size[0] = 1;
  j = r->size[1];
  transformedTracks->size[1] = r->size[1];
  emxEnsureCapacity_objectTrack(sp, transformedTracks, nz, &lf_emlrtRTEI);
  transformedTracks_data = transformedTracks->data;
  for (i = 0; i < j; i++) {
    transformedTracks_data[i] = r1[i];
  }

  emxFree_objectTrack(sp, &r);
  loop_ub = assignments_size[0];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    struct0_T expl_temp;
    if (b_i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, loop_ub, &bf_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    nz = (int32_T)assignments_data[b_i + assignments_size[0]];
    if ((nz < 1) || (nz > localTracks->size[0])) {
      emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0], &af_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    expl_temp = localTracks_data[nz - 1];
    st.site = &dr_emlrtRSI;
    b_st.site = &ug_emlrtRSI;
    thisConfig = FuserManager_getConfigByID(&b_st, obj, expl_temp.SourceIndex);
    st.site = &er_emlrtRSI;
    if (!thisConfig->pIsTransformToCentralValid) {
      b_st.site = &vg_emlrtRSI;
      c_st.site = &xg_emlrtRSI;
      local2central(&c_st, expl_temp.TrackID, expl_temp.BranchID,
                    expl_temp.SourceIndex, expl_temp.UpdateTime, expl_temp.Age,
                    expl_temp.State, expl_temp.StateCovariance,
                    expl_temp.ObjectClassID,
                    expl_temp.ObjectClassProbabilities.data,
                    expl_temp.ObjectClassProbabilities.size,
                    expl_temp.IsConfirmed, expl_temp.IsCoasted,
                    expl_temp.IsSelfReported, &b_expl_temp);
      c_st.site = &yg_emlrtRSI;

      /*  Pre-define output with longest TrackLogic to force codegen consistency */
      if (b_expl_temp.SourceIndex == 1U) {
        /*  Radar */
        d_st.site = &fi_emlrtRSI;
        Ecef2nedTrack(&d_st, b_expl_temp.TrackID, b_expl_temp.BranchID, 1U,
                      b_expl_temp.Age, b_expl_temp.ObjectClassID,
                      b_expl_temp.ObjectClassProbabilities.data,
                      b_expl_temp.ObjectClassProbabilities.size,
                      b_expl_temp.IsConfirmed, b_expl_temp.IsCoasted,
                      b_expl_temp.IsSelfReported, b_expl_temp.pState,
                      b_expl_temp.pStateCovariance, b_expl_temp.pUpdateTime,
                      &jw_emlrtRSI);
      } else if (b_expl_temp.SourceIndex == 2U) {
        /*  ADSB */
        d_st.site = &gi_emlrtRSI;
        Ned2ecefTrack(&d_st, b_expl_temp.TrackID, b_expl_temp.BranchID, 2U,
                      b_expl_temp.Age, b_expl_temp.ObjectClassID,
                      b_expl_temp.ObjectClassProbabilities.data,
                      b_expl_temp.ObjectClassProbabilities.size,
                      b_expl_temp.IsConfirmed, b_expl_temp.IsCoasted,
                      b_expl_temp.IsSelfReported, b_expl_temp.pState,
                      b_expl_temp.pStateCovariance, b_expl_temp.pUpdateTime,
                      &kw_emlrtRSI);
      }

      thisConfig->pIsTransformToCentralValid = true;
    }

    b_st.site = &wg_emlrtRSI;
    local2central(&b_st, expl_temp.TrackID, expl_temp.BranchID,
                  expl_temp.SourceIndex, expl_temp.UpdateTime, expl_temp.Age,
                  expl_temp.State, expl_temp.StateCovariance,
                  expl_temp.ObjectClassID,
                  expl_temp.ObjectClassProbabilities.data,
                  expl_temp.ObjectClassProbabilities.size, expl_temp.IsConfirmed,
                  expl_temp.IsCoasted, expl_temp.IsSelfReported, &b_expl_temp);
    st.site = &fr_emlrtRSI;
    nz = transformedTracks->size[1] - 1;
    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    b_st.site = &qc_emlrtRSI;
    c_st.site = &gb_emlrtRSI;
    if (b_expl_temp.pUpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }

    c_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(b_expl_temp.pUpdateTime) || muDoubleScalarIsNaN
        (b_expl_temp.pUpdateTime)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &m_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }

    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    transformedTracks_data[b_i].pUpdateTime = b_expl_temp.pUpdateTime;
    st.site = &fr_emlrtRSI;
    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    b_st.site = &rc_emlrtRSI;
    validateattributes(&b_st, b_expl_temp.pState);
    for (i = 0; i < 6; i++) {
      if (b_i > nz) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, nz, &cf_emlrtBCI, &st);
      }

      transformedTracks_data[b_i].pState[i] = b_expl_temp.pState[i];
    }

    st.site = &fr_emlrtRSI;
    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    b_st.site = &sc_emlrtRSI;
    b_validateattributes(&b_st, b_expl_temp.pStateCovariance);
    b_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&b_st, b_expl_temp.pStateCovariance);
    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, &st);
    }

    for (i = 0; i < 36; i++) {
      transformedTracks_data[b_i].pStateCovariance[i] =
        b_expl_temp.pStateCovariance[i];
    }

    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, (emlrtConstCTX)sp);
    }

    transformedTracks_data[b_i].IsConfirmed = b_expl_temp.IsConfirmed;
    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, (emlrtConstCTX)sp);
    }

    transformedTracks_data[b_i].IsCoasted = b_expl_temp.IsCoasted;
    if (b_i > transformedTracks->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, transformedTracks->size[1] - 1,
        &we_emlrtBCI, (emlrtConstCTX)sp);
    }

    transformedTracks_data[b_i].IsSelfReported = b_expl_temp.IsSelfReported;
  }

  emxInit_uint32_T(sp, &b, 1, &uf_emlrtRTEI);
  st.site = &gr_emlrtRSI;
  b_assignments_size = assignments_size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&b_assignments_data[0], &assignments_data[0], (uint32_T)loop_ub *
           sizeof(uint32_T));
  }

  c_assignments_data.data = &b_assignments_data[0];
  c_assignments_data.size = &b_assignments_size;
  c_assignments_data.allocatedSize = 200;
  c_assignments_data.numDimensions = 1;
  c_assignments_data.canFreeData = false;
  b_st.site = &ib_emlrtRSI;
  b_unique_vector(&b_st, &c_assignments_data, b);
  b_data = b->data;
  b_loop_ub = b->size[0];
  updated_size_idx_1 = b->size[0];
  emxInit_int32_T(sp, &ii, 1, &sd_emlrtRTEI);
  emxInit_struct1_T(sp, &otherAttributes, &vf_emlrtRTEI);
  emxInit_boolean_T(sp, &toFuse, 1, &pf_emlrtRTEI, true);
  for (c_i = 0; c_i < b_loop_ub; c_i++) {
    int32_T end;
    if (c_i + 1 > b_loop_ub) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, b_loop_ub, &qe_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    inAssigned_size = loop_ub;
    for (i = 0; i < loop_ub; i++) {
      inAssigned_data[i] = (b_data[c_i] == assignments_data[i]);
    }

    st.site = &hr_emlrtRSI;
    b_inAssigned_data.data = &inAssigned_data[0];
    b_inAssigned_data.size = &inAssigned_size;
    b_inAssigned_data.allocatedSize = 200;
    b_inAssigned_data.numDimensions = 1;
    b_inAssigned_data.canFreeData = false;
    b_st.site = &yl_emlrtRSI;
    eml_find(&b_st, &b_inAssigned_data, ii);
    ii_data = ii->data;
    st.site = &ir_emlrtRSI;
    end = ii->size[0];
    nz = toFuse->size[0];
    toFuse->size[0] = ii->size[0];
    emxEnsureCapacity_boolean_T(&st, toFuse, nz, &pf_emlrtRTEI);
    toFuse_data = toFuse->data;
    for (i = 0; i < end; i++) {
      if (i + 1 > end) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, end, &xe_emlrtBCI, &st);
      }

      j = ii_data[i] - 1;
      if ((j < 0) || (j > transformedTracks->size[1] - 1)) {
        emlrtDynamicBoundsCheckR2012b(j, 0, transformedTracks->size[1] - 1,
          &ye_emlrtBCI, &st);
      }

      if ((transformedTracks_data[ii_data[i] - 1].IsConfirmed &&
           (!transformedTracks_data[ii_data[i] - 1].IsCoasted)) ||
          transformedTracks_data[ii_data[i] - 1].IsSelfReported) {
        if (i + 1 > toFuse->size[0]) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, toFuse->size[0], &df_emlrtBCI,
            &st);
        }

        toFuse_data[i] = true;
      } else {
        if (i + 1 > toFuse->size[0]) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, toFuse->size[0], &df_emlrtBCI,
            &st);
        }

        toFuse_data[i] = false;
      }
    }

    st.site = &jr_emlrtRSI;
    if (b_any(&st, toFuse)) {
      real_T id;
      int32_T obj_tmp;
      int32_T trueCount;
      uint32_T u;
      uint32_T u1;
      uint8_T u2;
      boolean_T b_b;
      boolean_T b_value;
      u = b_data[c_i];
      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &sb_emlrtDCI, (emlrtConstCTX)sp);
      }

      st.site = &kr_emlrtRSI;
      b_st.site = &wq_emlrtRSI;
      c_st.site = &bg_emlrtRSI;
      nz = b_combineVectorElements(&c_st, toFuse);
      obj_tmp = (int32_T)b_data[c_i] - 1;
      b_obj = obj->pTracksList[obj_tmp];
      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &wb_emlrtDCI, (emlrtConstCTX)sp);
      }

      b_b = (((int32_T)u - 1 < 0) || ((int32_T)u - 1 > 99));
      if (b_b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &ef_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      id = (real_T)obj->pTracksList[(int32_T)u - 1].Age + (real_T)nz;
      if (id < 4.294967296E+9) {
        if (id >= 0.0) {
          u1 = (uint32_T)id;
        } else {
          u1 = 0U;
        }
      } else {
        u1 = MAX_uint32_T;
      }

      b_obj.Age = u1;
      if ((obj_tmp < 0) || (obj_tmp > 99)) {
        emlrtDynamicBoundsCheckR2012b(obj_tmp, 0, 99, &ff_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      obj->pTracksList[obj_tmp] = b_obj;
      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &tb_emlrtDCI, (emlrtConstCTX)sp);
      }

      trueCount = 0;
      j = 0;
      for (b_i = 0; b_i < loop_ub; b_i++) {
        if (inAssigned_data[b_i]) {
          trueCount++;
        }

        if (inAssigned_data[b_i]) {
          tmp_data[j] = (uint8_T)b_i;
          j++;
        }
      }

      st.site = &lr_emlrtRSI;
      for (b_i = 0; b_i < trueCount; b_i++) {
        u2 = tmp_data[b_i];
        if (u2 > assignments_size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(u2, 0, assignments_size[0] - 1,
            &gf_emlrtBCI, &st);
        }
      }

      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &ab_emlrtDCI, &st);
      }

      if (b_b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &pd_emlrtBCI, &st);
      }

      id = obj->pTracksList[(int32_T)u - 1].ObjectClassID;
      j = 0;
      while ((id == 0.0) && (j < trueCount)) {
        j++;
        if (j > trueCount) {
          emlrtDynamicBoundsCheckR2012b(j, 1, trueCount, &hf_emlrtBCI, &st);
        }

        nz = (int32_T)assignments_data[tmp_data[j - 1] + assignments_size[0]];
        if ((nz < 1) || (nz > localTracks->size[0])) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &me_emlrtBCI, &st);
        }

        id = localTracks_data[nz - 1].ObjectClassID;
      }

      st.site = &lr_emlrtRSI;
      b_obj = obj->pTracksList[obj_tmp];
      b_st.site = &ig_emlrtRSI;
      c_validateattributes(&b_st, id);
      b_obj.ObjectClassID = id;
      obj->pTracksList[obj_tmp] = b_obj;
      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &ub_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (b_b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &re_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &xb_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (b_b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &if_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      b_obj = obj->pTracksList[(int32_T)u - 1];
      end = toFuse->size[0];
      j = 0;
      for (b_i = 0; b_i < end; b_i++) {
        if (toFuse_data[b_i]) {
          j++;
        }
      }

      nz = 0;
      for (b_i = 0; b_i < end; b_i++) {
        if (toFuse_data[b_i]) {
          b_tmp_data[nz] = (uint8_T)b_i;
          nz++;
        }
      }

      ii_size = j;
      for (b_i = 0; b_i < j; b_i++) {
        u2 = b_tmp_data[b_i];
        if (u2 > ii->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(u2, 0, ii->size[0] - 1, &jf_emlrtBCI,
            (emlrtConstCTX)sp);
        }

        b_ii_data[b_i] = ii_data[u2];
      }

      c_ii_data.data = &b_ii_data[0];
      c_ii_data.size = &ii_size;
      c_ii_data.allocatedSize = 200;
      c_ii_data.numDimensions = 1;
      c_ii_data.canFreeData = false;
      st.site = &mr_emlrtRSI;
      Fuserxcov_fuse(&st, &obj->cFuser, &b_obj, transformedTracks, &c_ii_data);
      obj->pTracksList[obj_tmp] = b_obj;
      st.site = &nr_emlrtRSI;
      b_st.site = &wq_emlrtRSI;
      c_st.site = &bg_emlrtRSI;
      nz = b_combineVectorElements(&c_st, toFuse);
      emlrtForLoopVectorCheckR2021a(1.0, 1.0, nz, mxDOUBLE_CLASS, nz,
        &tb_emlrtRTEI, (emlrtConstCTX)sp);
      for (i = 0; i < nz; i++) {
        boolean_T bv[50];
        st.site = &or_emlrtRSI;
        if ((real_T)u != (int32_T)u) {
          emlrtIntegerCheckR2012b(u, &ac_emlrtDCI, &st);
        }

        if (b_b) {
          emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &lf_emlrtBCI, &st);
        }

        c_obj = obj->pTrackLogics[(int32_T)u - 1];
        if (c_obj->pIsFirstUpdate) {
          emlrtErrorWithMessageIdR2018a(&st, &vb_emlrtRTEI,
            "shared_tracking:trackHistoryLogic:notInitialized",
            "shared_tracking:trackHistoryLogic:notInitialized", 3, 4, 3, "hit");
        }

        bv[0] = true;
        for (b_i = 0; b_i < 49; b_i++) {
          bv[b_i + 1] = c_obj->pRecentHistory[b_i];
        }

        for (b_i = 0; b_i < 50; b_i++) {
          c_obj->pRecentHistory[b_i] = bv[b_i];
        }

        c_obj->pIsFirstUpdate = false;
      }

      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &yb_emlrtDCI, (emlrtConstCTX)sp);
      }

      if (b_b) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &kf_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      if (obj->pTracksList[(int32_T)u - 1].IsConfirmed) {
        b_value = true;
      } else {
        st.site = &pr_emlrtRSI;
        if ((real_T)u != (int32_T)u) {
          emlrtIntegerCheckR2012b(u, &bc_emlrtDCI, &st);
        }

        if (b_b) {
          emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &mf_emlrtBCI, &st);
        }

        c_obj = obj->pTrackLogics[(int32_T)u - 1];
        if (c_obj->pIsFirstUpdate) {
          b_value = false;
        } else {
          boolean_T x_idx_1;
          boolean_T x_idx_2;
          b_value = c_obj->pRecentHistory[0];
          x_idx_1 = c_obj->pRecentHistory[1];
          x_idx_2 = c_obj->pRecentHistory[2];
          b_value = ((b_value + x_idx_1) + x_idx_2 >= 2);
        }

        if (b_value) {
          b_value = true;
        } else {
          if ((real_T)u != (int32_T)u) {
            emlrtIntegerCheckR2012b(u, &cc_emlrtDCI, (emlrtConstCTX)sp);
          }

          if (b_b) {
            emlrtDynamicBoundsCheckR2012b((int32_T)u - 1, 0, 99, &nf_emlrtBCI,
              (emlrtConstCTX)sp);
          }

          if (obj->pTracksList[(int32_T)u - 1].ObjectClassID > 0.0) {
            b_value = true;
          } else {
            b_value = false;
          }
        }
      }

      if ((real_T)u != (int32_T)u) {
        emlrtIntegerCheckR2012b(u, &vb_emlrtDCI, (emlrtConstCTX)sp);
      }

      b_obj = obj->pTracksList[obj_tmp];
      b_obj.IsConfirmed = b_value;
      obj->pTracksList[obj_tmp] = b_obj;
      if (((int32_T)b_data[c_i] < 1) || ((int32_T)b_data[c_i] > 100)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_data[c_i], 1, 100, &of_emlrtBCI,
          (emlrtConstCTX)sp);
      }

      obj->pConfirmedTracks[obj_tmp] = obj->pTracksList[obj_tmp].IsConfirmed;
      st.site = &qr_emlrtRSI;
      b_value = false;
      nz = 0;
      while ((!b_value) && (nz < trueCount)) {
        nz++;
        if (nz > trueCount) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, trueCount, &pf_emlrtBCI, &st);
        }

        end = (int32_T)assignments_data[tmp_data[nz - 1] + assignments_size[0]];
        if ((end < 1) || (end > localTracks->size[0])) {
          emlrtDynamicBoundsCheckR2012b(end, 1, localTracks->size[0],
            &se_emlrtBCI, &st);
        }

        b_value = localTracks_data[end - 1].IsCoasted;
      }

      b_obj = obj->pTracksList[obj_tmp];
      b_obj.IsCoasted = b_value;
      obj->pTracksList[obj_tmp] = b_obj;
      b_obj = obj->pTracksList[obj_tmp];
      b_assignments_size = trueCount;
      for (b_i = 0; b_i < trueCount; b_i++) {
        u2 = tmp_data[b_i];
        if (u2 > assignments_size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(u2, 0, assignments_size[0] - 1,
            &qf_emlrtBCI, (emlrtConstCTX)sp);
        }

        b_assignments_data[b_i] = assignments_data[u2 + assignments_size[0]];
      }

      d_assignments_data.data = &b_assignments_data[0];
      d_assignments_data.size = &b_assignments_size;
      d_assignments_data.allocatedSize = 200;
      d_assignments_data.numDimensions = 1;
      d_assignments_data.canFreeData = false;
      st.site = &rr_emlrtRSI;
      b_obj.IsSelfReported = trackFuser_getSelfReporting(&st, obj, localTracks,
        &d_assignments_data);
      obj->pTracksList[obj_tmp] = b_obj;
      b_obj = obj->pTracksList[obj_tmp];
      obj->pTracksList[obj_tmp] = b_obj;
      st.site = &sr_emlrtRSI;
      j = otherAttributes->size[0] * otherAttributes->size[1];
      otherAttributes->size[0] = 1;
      otherAttributes->size[1] = trueCount;
      emxEnsureCapacity_struct1_T(&st, otherAttributes, j, &tf_emlrtRTEI);
      otherAttributes_data = otherAttributes->data;
      for (b_i = 0; b_i < trueCount; b_i++) {
        if (b_i + 1 > trueCount) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trueCount, &rf_emlrtBCI, &st);
        }

        nz = (int32_T)assignments_data[tmp_data[b_i] + assignments_size[0]];
        if ((nz < 1) || (nz > localTracks->size[0])) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &te_emlrtBCI, &st);
        }

        if (b_i + 1 > trueCount) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trueCount, &sf_emlrtBCI, &st);
        }

        if (nz > localTracks->size[0]) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &ue_emlrtBCI, &st);
        }

        if (b_i + 1 > trueCount) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trueCount, &tf_emlrtBCI, &st);
        }

        if (nz > localTracks->size[0]) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &ve_emlrtBCI, &st);
        }

        if (b_i + 1 > trueCount) {
          emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, trueCount, &wf_emlrtBCI, &st);
        }

        if (nz > localTracks->size[0]) {
          emlrtDynamicBoundsCheckR2012b(nz, 1, localTracks->size[0],
            &vf_emlrtBCI, &st);
        }

        if (b_i > otherAttributes->size[1] - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, otherAttributes->size[1] - 1,
            &xf_emlrtBCI, &st);
        }

        otherAttributes_data[b_i] = localTracks_data[nz - 1].ObjectAttributes;
      }

      b_obj = obj->pTracksList[obj_tmp];
      obj->pTracksList[obj_tmp] = b_obj;
      if (c_i + 1 > updated_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, updated_size_idx_1,
          &uf_emlrtBCI, (emlrtConstCTX)sp);
      }
    }
  }

  emxFree_boolean_T(sp, &toFuse);
  emxFree_struct1_T(sp, &otherAttributes);
  emxFree_int32_T(sp, &ii);
  emxFree_uint32_T(sp, &b);
  emxFree_objectTrack(sp, &transformedTracks);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static boolean_T trackFuser_getSelfReporting(const emlrtStack *sp, trackFuser
  *obj, const emxArray_struct0_T *localTracks, const emxArray_uint32_T
  *assignedlocalTracks)
{
  emlrtStack b_st;
  emlrtStack st;
  fuserSourceConfiguration *thisSource;
  const struct0_T *localTracks_data;
  int32_T i;
  const uint32_T *assignedlocalTracks_data;
  boolean_T exitg1;
  boolean_T tf;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  assignedlocalTracks_data = assignedlocalTracks->data;
  localTracks_data = localTracks->data;
  tf = false;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < assignedlocalTracks->size[0])) {
    int32_T b_i;
    i++;
    if (i > assignedlocalTracks->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i, 1, assignedlocalTracks->size[0],
        &pg_emlrtBCI, (emlrtConstCTX)sp);
    }

    b_i = (int32_T)assignedlocalTracks_data[i - 1];
    if ((b_i < 1) || (b_i > localTracks->size[0])) {
      emlrtDynamicBoundsCheckR2012b(b_i, 1, localTracks->size[0], &le_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    st.site = &oq_emlrtRSI;
    b_st.site = &ug_emlrtRSI;
    thisSource = FuserManager_getConfigByID(&b_st, obj, localTracks_data[b_i - 1]
      .SourceIndex);
    if (thisSource->IsInternalSource && localTracks_data[b_i - 1].IsSelfReported)
    {
      tf = true;
      exitg1 = true;
    }
  }

  return tf;
}

void trackFuser_resetImpl(const emlrtStack *sp, trackFuser *obj)
{
  emlrtStack st;
  trackHistoryLogic *b_obj;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  obj->pNumLiveTracks = 0.0;
  obj->pLastTimeStamp = -2.2204460492503131E-16;
  for (i = 0; i < 100; i++) {
    obj->pTrackIDs[i] = 0U;
  }

  for (i = 0; i < 100; i++) {
    obj->pConfirmedTracks[i] = false;
  }

  obj->pIsValidSource[0] = false;
  obj->pIsValidSource[1] = false;
  st.site = &cj_emlrtRSI;
  SystemCore_reset(&st, &obj->cAssigner);
  b_obj = obj->pTrackLogics[0];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[1];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[2];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[3];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[4];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[5];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[6];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[7];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[8];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[9];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[10];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[11];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[12];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[13];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[14];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[15];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[16];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[17];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[18];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[19];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[20];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[21];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[22];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[23];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[24];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[25];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[26];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[27];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[28];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[29];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[30];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[31];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[32];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[33];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[34];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[35];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[36];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[37];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[38];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[39];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[40];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[41];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[42];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[43];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[44];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[45];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[46];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[47];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[48];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[49];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[50];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[51];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[52];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[53];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[54];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[55];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[56];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[57];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[58];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[59];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[60];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[61];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[62];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[63];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[64];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[65];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[66];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[67];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[68];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[69];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[70];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[71];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[72];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[73];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[74];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[75];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[76];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[77];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[78];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[79];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[80];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[81];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[82];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[83];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[84];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[85];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[86];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[87];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[88];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[89];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[90];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[91];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[92];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[93];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[94];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[95];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[96];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[97];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[98];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  b_obj = obj->pTrackLogics[99];
  for (i = 0; i < 50; i++) {
    b_obj->pRecentHistory[i] = false;
  }

  b_obj->pIsFirstUpdate = true;
  obj->pLastTrackID = 0U;
}

int32_T trackFuser_stepImpl(const emlrtStack *sp, trackFuser *obj, const
  emxArray_struct0_T *localTracks, real_T tFusion, struct2_T confTracks_data[])
{
  static const char_T logicType[7] = { 'H', 'i', 's', 't', 'o', 'r', 'y' };

  __m128d r;
  b_objectTrack expl_temp;
  b_objectTrack jw_emlrtRSI;
  b_objectTrack kw_emlrtRSI;
  b_objectTrack lw_emlrtRSI;
  c_objectTrack track;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_boolean_T *updated;
  emxArray_int32_T *r1;
  emxArray_real_T *b_prevInds;
  emxArray_real_T *costMatrix;
  emxArray_real_T *prevInds;
  emxArray_struct2_T *allStructs;
  emxArray_uint32_T *assigned;
  emxArray_uint32_T *unassignedCentral;
  fuserSourceConfiguration *source;
  const struct0_T *localTracks_data;
  struct2_T oneStruct;
  struct2_T *allStructs_data;
  trackHistoryLogic *b_obj;
  real_T times_data[200];
  real_T b_dv[2];
  real_T dt;
  real_T prevNumLive;
  real_T *b_prevInds_data;
  real_T *prevInds_data;
  int32_T times_size[2];
  int32_T b_i;
  int32_T c_i;
  int32_T confTracks_size;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T times_size_idx_1;
  int32_T unassignedSource_size;
  int32_T *r2;
  uint32_T b_unassignedSource_data[200];
  uint32_T unassignedSource_data[200];
  uint32_T inKnownIDs_tmp_tmp;
  uint32_T *assigned_data;
  uint32_T *unassignedCentral_data;
  int8_T c_tmp_data[100];
  int8_T tmp_data[2];
  uint8_T b_tmp_data[200];
  boolean_T b_times_data[200];
  boolean_T isInitializing_data[200];
  boolean_T deleted[100];
  boolean_T y;
  boolean_T *updated_data;
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
  localTracks_data = localTracks->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ej_emlrtRSI;
  if (!(tFusion > obj->pLastTimeStamp)) {
    emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI,
      "fusion:trackFuser:TimeMustIncrease", "fusion:trackFuser:TimeMustIncrease",
      3, 4, 10, "trackFuser");
  }

  b_st.site = &gj_emlrtRSI;
  if (localTracks->size[0] == 0) {
    times_size_idx_1 = 0;
  } else {
    loop_ub = localTracks->size[0];
    times_size_idx_1 = localTracks->size[0];
    for (i = 0; i < loop_ub; i++) {
      if (i + 1 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, times_size_idx_1, &eb_emlrtBCI,
          &b_st);
      }

      if (i + 1 > loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &fb_emlrtBCI, &b_st);
      }

      times_data[i] = localTracks_data[i].UpdateTime;
    }
  }

  times_size[0] = 1;
  times_size[1] = times_size_idx_1;
  for (i = 0; i < times_size_idx_1; i++) {
    b_times_data[i] = (times_data[i] <= tFusion + 1.0E-5);
  }

  b_st.site = &hj_emlrtRSI;
  if (b_all(b_times_data, times_size)) {
    times_size[0] = 1;
    for (i = 0; i < times_size_idx_1; i++) {
      b_times_data[i] = (times_data[i] >= obj->pLastTimeStamp);
    }

    b_st.site = &ij_emlrtRSI;
    if (!b_all(b_times_data, times_size)) {
      emlrtErrorWithMessageIdR2018a(&st, &u_emlrtRTEI,
        "fusion:trackFuser:TrackFusionTimeMismatch",
        "fusion:trackFuser:TrackFusionTimeMismatch", 6, 4, 10, "UpdateTime", 4,
        7, "tFusion");
    }
  } else {
    emlrtErrorWithMessageIdR2018a(&st, &u_emlrtRTEI,
      "fusion:trackFuser:TrackFusionTimeMismatch",
      "fusion:trackFuser:TrackFusionTimeMismatch", 6, 4, 10, "UpdateTime", 4, 7,
      "tFusion");
  }

  prevNumLive = obj->pNumLiveTracks;
  emxInit_real_T(&st, &prevInds, 2, &vc_emlrtRTEI);
  prevInds_data = prevInds->data;
  if (muDoubleScalarIsNaN(prevNumLive)) {
    loop_ub = prevInds->size[0] * prevInds->size[1];
    prevInds->size[0] = 1;
    prevInds->size[1] = 1;
    emxEnsureCapacity_real_T(&st, prevInds, loop_ub, &vc_emlrtRTEI);
    prevInds_data = prevInds->data;
    prevInds_data[0] = rtNaN;
  } else if (prevNumLive < 1.0) {
    prevInds->size[0] = 1;
    prevInds->size[1] = 0;
  } else {
    loop_ub = prevInds->size[0] * prevInds->size[1];
    prevInds->size[0] = 1;
    prevInds->size[1] = (int32_T)(prevNumLive - 1.0) + 1;
    emxEnsureCapacity_real_T(&st, prevInds, loop_ub, &vc_emlrtRTEI);
    prevInds_data = prevInds->data;
    loop_ub = (int32_T)(prevNumLive - 1.0);
    times_size_idx_1 = (((int32_T)(prevNumLive - 1.0) + 1) / 2) << 1;
    confTracks_size = times_size_idx_1 - 2;
    for (i = 0; i <= confTracks_size; i += 2) {
      b_dv[0] = i;
      b_dv[1] = i + 1;
      r = _mm_loadu_pd(&b_dv[0]);
      _mm_storeu_pd(&prevInds_data[i], _mm_add_pd(_mm_set1_pd(1.0), r));
    }

    for (i = times_size_idx_1; i <= loop_ub; i++) {
      prevInds_data[i] = (real_T)i + 1.0;
    }
  }

  if (prevNumLive < 1.0) {
    b_i = 0;
  } else {
    if (prevNumLive != (int32_T)muDoubleScalarFloor(prevNumLive)) {
      emlrtIntegerCheckR2012b(prevNumLive, &k_emlrtDCI, &st);
    }

    if (((int32_T)prevNumLive < 1) || ((int32_T)prevNumLive > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)prevNumLive, 1, 100, &lb_emlrtBCI,
        &st);
    }

    b_i = (int32_T)prevNumLive;
  }

  b_st.site = &jj_emlrtRSI;
  i1 = localTracks->size[0];
  loop_ub = obj->pUsedConfigIDs->size[0] * obj->pUsedConfigIDs->size[1];
  obj->pUsedConfigIDs->size[0] = localTracks->size[0];
  obj->pUsedConfigIDs->size[1] = 2;
  emxEnsureCapacity_boolean_T(&b_st, obj->pUsedConfigIDs, loop_ub, &wc_emlrtRTEI);
  loop_ub = localTracks->size[0] << 1;
  for (i = 0; i < loop_ub; i++) {
    obj->pUsedConfigIDs->data[i] = false;
  }

  for (c_i = 0; c_i < i1; c_i++) {
    boolean_T x_data[10];
    boolean_T inKnownIDs[2];
    boolean_T exitg1;
    if (c_i + 1 > i1) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, i1, &pb_emlrtBCI, &b_st);
    }

    inKnownIDs_tmp_tmp = localTracks_data[c_i].SourceIndex;
    inKnownIDs[0] = (inKnownIDs_tmp_tmp == obj->pSourceConfigIDs[0]);
    inKnownIDs[1] = (inKnownIDs_tmp_tmp == obj->pSourceConfigIDs[1]);
    y = false;
    times_size_idx_1 = 0;
    exitg1 = false;
    while ((!exitg1) && (times_size_idx_1 < 2)) {
      if (inKnownIDs[times_size_idx_1]) {
        y = true;
        exitg1 = true;
      } else {
        times_size_idx_1++;
      }
    }

    if (!y) {
      emlrtErrorWithMessageIdR2018a(&b_st, &v_emlrtRTEI,
        "fusion:trackFuser:UnknownConfig", "fusion:trackFuser:UnknownConfig", 6,
        4, 11, "SourceIndex", 4, 20, "SourceConfigurations");
    }

    times_size_idx_1 = 0;
    if (inKnownIDs[0]) {
      times_size_idx_1 = 1;
    }

    if (inKnownIDs[1]) {
      times_size_idx_1++;
    }

    loop_ub = 0;
    if (inKnownIDs[0]) {
      tmp_data[0] = 0;
      loop_ub = 1;
    }

    if (inKnownIDs[1]) {
      tmp_data[loop_ub] = 1;
    }

    loop_ub = obj->pUsedConfigIDs->size[0];
    if (c_i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, loop_ub, &qb_emlrtBCI, &b_st);
    }

    for (i = 0; i < times_size_idx_1; i++) {
      obj->pUsedConfigIDs->data[c_i + obj->pUsedConfigIDs->size[0] * tmp_data[i]]
        = true;
    }

    c_st.site = &uj_emlrtRSI;
    for (i = 0; i < times_size_idx_1; i++) {
      x_data[i] = !obj->pIsValidSource[tmp_data[i]];
    }

    y = (times_size_idx_1 != 0);
    if (y) {
      d_st.site = &qg_emlrtRSI;
      e_st.site = &rg_emlrtRSI;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= times_size_idx_1 - 1)) {
        if (!x_data[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
    }

    if (y) {
      c_st.site = &vj_emlrtRSI;
      if (c_i + 1 > i1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, i1, &rb_emlrtBCI, &c_st);
      }

      d_st.site = &qi_emlrtRSI;
      source = FuserManager_getConfigByID(&d_st, obj, inKnownIDs_tmp_tmp);
      d_st.site = &ri_emlrtRSI;
      if (!source->pIsTransformToCentralValid) {
        e_st.site = &vg_emlrtRSI;
        f_st.site = &xg_emlrtRSI;
        local2central(&f_st, localTracks_data[c_i].TrackID, localTracks_data[c_i]
                      .BranchID, inKnownIDs_tmp_tmp, localTracks_data[c_i].
                      UpdateTime, localTracks_data[c_i].Age,
                      localTracks_data[c_i].State, localTracks_data[c_i].
                      StateCovariance, localTracks_data[c_i].ObjectClassID,
                      localTracks_data[c_i].ObjectClassProbabilities.data,
                      localTracks_data[c_i].ObjectClassProbabilities.size,
                      localTracks_data[c_i].IsConfirmed, localTracks_data[c_i].
                      IsCoasted, localTracks_data[c_i].IsSelfReported,
                      &expl_temp);
        f_st.site = &yg_emlrtRSI;

        /*  Pre-define output with longest TrackLogic to force codegen consistency */
        if (expl_temp.SourceIndex == 1U) {
          /*  Radar */
          g_st.site = &fi_emlrtRSI;
          Ecef2nedTrack(&g_st, expl_temp.TrackID, expl_temp.BranchID, 1U,
                        expl_temp.Age, expl_temp.ObjectClassID,
                        expl_temp.ObjectClassProbabilities.data,
                        expl_temp.ObjectClassProbabilities.size,
                        expl_temp.IsConfirmed, expl_temp.IsCoasted,
                        expl_temp.IsSelfReported, expl_temp.pState,
                        expl_temp.pStateCovariance, expl_temp.pUpdateTime,
                        &kw_emlrtRSI);
        } else if (expl_temp.SourceIndex == 2U) {
          /*  ADSB */
          g_st.site = &gi_emlrtRSI;
          Ned2ecefTrack(&g_st, expl_temp.TrackID, expl_temp.BranchID, 2U,
                        expl_temp.Age, expl_temp.ObjectClassID,
                        expl_temp.ObjectClassProbabilities.data,
                        expl_temp.ObjectClassProbabilities.size,
                        expl_temp.IsConfirmed, expl_temp.IsCoasted,
                        expl_temp.IsSelfReported, expl_temp.pState,
                        expl_temp.pStateCovariance, expl_temp.pUpdateTime,
                        &lw_emlrtRSI);
        }

        source->pIsTransformToCentralValid = true;
      }

      e_st.site = &wg_emlrtRSI;
      local2central(&e_st, localTracks_data[c_i].TrackID, localTracks_data[c_i].
                    BranchID, inKnownIDs_tmp_tmp, localTracks_data[c_i].
                    UpdateTime, localTracks_data[c_i].Age, localTracks_data[c_i]
                    .State, localTracks_data[c_i].StateCovariance,
                    localTracks_data[c_i].ObjectClassID, localTracks_data[c_i].
                    ObjectClassProbabilities.data, localTracks_data[c_i].
                    ObjectClassProbabilities.size, localTracks_data[c_i].
                    IsConfirmed, localTracks_data[c_i].IsCoasted,
                    localTracks_data[c_i].IsSelfReported, &jw_emlrtRSI);
      for (i = 0; i < times_size_idx_1; i++) {
        obj->pIsValidSource[tmp_data[i]] = true;
      }
    }
  }

  emxInit_real_T(&st, &costMatrix, 2, &cd_emlrtRTEI);
  b_st.site = &kj_emlrtRSI;
  FuserManager_distance(&b_st, obj, localTracks, costMatrix);
  emxInit_uint32_T(&st, &assigned, 2, &ed_emlrtRTEI);
  emxInit_uint32_T(&st, &unassignedCentral, 1, &ed_emlrtRTEI);
  b_st.site = &lj_emlrtRSI;
  trackFuser_assign(&b_st, obj, costMatrix, assigned, unassignedCentral,
                    unassignedSource_data, &unassignedSource_size);
  unassignedCentral_data = unassignedCentral->data;
  assigned_data = assigned->data;
  emxFree_real_T(&st, &costMatrix);
  b_st.site = &mj_emlrtRSI;
  confTracks_size = assigned->size[0];
  for (i = 0; i < confTracks_size; i++) {
    if (((int32_T)assigned_data[i] < 1) || ((int32_T)assigned_data[i] > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)assigned_data[i], 1, 100,
        &db_emlrtBCI, &b_st);
    }
  }

  b_st.site = &nj_emlrtRSI;
  loop_ub = unassignedCentral->size[0];
  for (i = 0; i < loop_ub; i++) {
    if (((int32_T)unassignedCentral_data[i] < 1) || ((int32_T)
         unassignedCentral_data[i] > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)unassignedCentral_data[i], 1, 100,
        &db_emlrtBCI, &b_st);
    }
  }

  b_st.site = &oj_emlrtRSI;
  for (i = 0; i < unassignedSource_size; i++) {
    c_st.site = &up_emlrtRSI;
    if (i + 1 > unassignedSource_size) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, unassignedSource_size,
        &cb_emlrtBCI, &c_st);
    }

    inKnownIDs_tmp_tmp = unassignedSource_data[i];
    if (((int32_T)inKnownIDs_tmp_tmp < 1) || ((int32_T)inKnownIDs_tmp_tmp > i1))
    {
      emlrtDynamicBoundsCheckR2012b((int32_T)inKnownIDs_tmp_tmp, 1, i1,
        &sb_emlrtBCI, &c_st);
    }

    d_st.site = &ug_emlrtRSI;
    source = FuserManager_getConfigByID(&d_st, obj, localTracks_data[(int32_T)
      inKnownIDs_tmp_tmp - 1].SourceIndex);
    if (i + 1 > unassignedSource_size) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, unassignedSource_size,
        &bb_emlrtBCI, &b_st);
    }

    isInitializing_data[i] = source->IsInitializingCentralTracks;
  }

  confTracks_size = 0;
  loop_ub = 0;
  for (i = 0; i < unassignedSource_size; i++) {
    if (isInitializing_data[i]) {
      confTracks_size++;
      b_tmp_data[loop_ub] = (uint8_T)i;
      loop_ub++;
    }
  }

  for (i = 0; i < confTracks_size; i++) {
    uint8_T u;
    u = b_tmp_data[i];
    if (u > unassignedSource_size - 1) {
      emlrtDynamicBoundsCheckR2012b(u, 0, unassignedSource_size - 1,
        &ab_emlrtBCI, &st);
    }

    b_unassignedSource_data[i] = unassignedSource_data[u];
  }

  b_st.site = &pj_emlrtRSI;
  c_trackFuser_initializeCentralT(&b_st, obj, localTracks,
    b_unassignedSource_data, confTracks_size);
  dt = obj->pNumLiveTracks;
  if (!(prevNumLive + 1.0 > dt)) {
    if (prevNumLive + 1.0 != (int32_T)muDoubleScalarFloor(prevNumLive + 1.0)) {
      emlrtIntegerCheckR2012b(prevNumLive + 1.0, &l_emlrtDCI, &st);
    }

    if (((int32_T)(prevNumLive + 1.0) < 1) || ((int32_T)(prevNumLive + 1.0) >
         100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(prevNumLive + 1.0), 1, 100,
        &mb_emlrtBCI, &st);
    }

    if (dt != (int32_T)muDoubleScalarFloor(dt)) {
      emlrtIntegerCheckR2012b(dt, &m_emlrtDCI, &st);
    }

    if (((int32_T)dt < 1) || ((int32_T)dt > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)dt, 1, 100, &nb_emlrtBCI, &st);
    }
  }

  emxInit_boolean_T(&st, &updated, 2, &dd_emlrtRTEI, true);
  b_st.site = &qj_emlrtRSI;
  b_trackFuser_fuseAssigned(&b_st, obj, localTracks, assigned, updated);
  updated_data = updated->data;
  emxFree_uint32_T(&st, &assigned);
  times_size_idx_1 = updated->size[1];
  confTracks_size = 0;
  for (i = 0; i < times_size_idx_1; i++) {
    if (!updated_data[i]) {
      confTracks_size++;
    }
  }

  emxInit_int32_T(&st, &r1, 2, &xc_emlrtRTEI);
  loop_ub = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = confTracks_size;
  emxEnsureCapacity_int32_T(&st, r1, loop_ub, &xc_emlrtRTEI);
  r2 = r1->data;
  loop_ub = 0;
  for (i = 0; i < times_size_idx_1; i++) {
    if (!updated_data[i]) {
      r2[loop_ub] = i;
      loop_ub++;
    }
  }

  emxInit_real_T(&st, &b_prevInds, 2, &yc_emlrtRTEI);
  loop_ub = b_prevInds->size[0] * b_prevInds->size[1];
  b_prevInds->size[0] = 1;
  confTracks_size = r1->size[1];
  b_prevInds->size[1] = r1->size[1];
  emxEnsureCapacity_real_T(&st, b_prevInds, loop_ub, &yc_emlrtRTEI);
  b_prevInds_data = b_prevInds->data;
  for (i = 0; i < confTracks_size; i++) {
    if (r2[i] > prevInds->size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(r2[i], 0, prevInds->size[1] - 1, &y_emlrtBCI,
        &st);
    }

    b_prevInds_data[i] = prevInds_data[r2[i]];
  }

  emxFree_int32_T(&st, &r1);
  b_st.site = &rj_emlrtRSI;
  trackFuser_coastUnassigned(&b_st, obj, unassignedCentral, b_prevInds, deleted);
  emxFree_real_T(&st, &b_prevInds);
  emxFree_uint32_T(&st, &unassignedCentral);
  b_st.site = &sj_emlrtRSI;
  dt = obj->pNumLiveTracks;
  loop_ub = (int32_T)dt;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, dt, mxDOUBLE_CLASS, (int32_T)dt,
    &w_emlrtRTEI, &b_st);
  for (i = 0; i < loop_ub; i++) {
    if (i > 99) {
      emlrtDynamicBoundsCheckR2012b(i, 0, 99, &x_emlrtBCI, &b_st);
    }

    track = obj->pTracksList[i];
    dt = tFusion - track.pUpdateTime;
    if (dt > 0.0) {
      c_objectTrack b_expl_temp;
      real_T x[6];
      track = obj->pTracksList[i];
      b_expl_temp = obj->pTracksList[i];
      c_st.site = &ov_emlrtRSI;
      b_gaussEKFilter_predict(&c_st, track.pState, b_expl_temp.pStateCovariance,
        obj->ProcessNoise, dt, x);
      track = obj->pTracksList[i];
      c_st.site = &pv_emlrtRSI;
      d_st.site = &rc_emlrtRSI;
      validateattributes(&d_st, x);
      for (c_i = 0; c_i < 6; c_i++) {
        track.pState[c_i] = x[c_i];
      }

      c_st.site = &qv_emlrtRSI;
      d_st.site = &sc_emlrtRSI;
      b_validateattributes(&d_st, b_expl_temp.pStateCovariance);
      d_st.site = &tc_emlrtRSI;
      isSymmetricPositiveSemiDefinite(&d_st, b_expl_temp.pStateCovariance);
      memcpy(&track.pStateCovariance[0], &b_expl_temp.pStateCovariance[0], 36U *
             sizeof(real_T));
      c_st.site = &rv_emlrtRSI;
      d_st.site = &qc_emlrtRSI;
      e_st.site = &gb_emlrtRSI;
      if (tFusion < 0.0) {
        emlrtErrorWithMessageIdR2018a(&e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
      }

      e_st.site = &gb_emlrtRSI;
      if (muDoubleScalarIsInf(tFusion)) {
        emlrtErrorWithMessageIdR2018a(&e_st, &m_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
      }

      track.pUpdateTime = tFusion;
      obj->pTracksList[i] = track;
    }
  }

  obj->pLastTimeStamp = tFusion;
  dt = obj->pNumLiveTracks;
  if (!(dt < 1.0)) {
    if (dt != (int32_T)muDoubleScalarFloor(dt)) {
      emlrtIntegerCheckR2012b(dt, &n_emlrtDCI, &st);
    }

    if (((int32_T)dt < 1) || ((int32_T)dt > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)dt, 1, 100, &ob_emlrtBCI, &st);
    }
  }

  if (unassignedSource_size != 0) {
    for (i = 0; i < unassignedSource_size; i++) {
      if ((!isInitializing_data[i]) && (i > unassignedSource_size - 1)) {
        emlrtDynamicBoundsCheckR2012b(i, 0, unassignedSource_size - 1,
          &jb_emlrtBCI, &st);
      }
    }
  }

  if (prevNumLive < 1.0) {
    loop_ub = 0;
  } else {
    if (prevNumLive != (int32_T)muDoubleScalarFloor(prevNumLive)) {
      emlrtIntegerCheckR2012b(prevNumLive, &o_emlrtDCI, &st);
    }

    loop_ub = (int32_T)prevNumLive;
  }

  b_st.site = &tj_emlrtRSI;
  c_st.site = &ar_emlrtRSI;
  d_st.site = &br_emlrtRSI;
  for (i = 0; i < times_size_idx_1; i++) {
    if (updated_data[i] && (i > (int8_T)b_i - 1)) {
      emlrtDynamicBoundsCheckR2012b(i, 0, (int8_T)b_i - 1, &ib_emlrtBCI, &st);
    }
  }

  emxFree_boolean_T(&st, &updated);
  for (i = 0; i < loop_ub; i++) {
    if (deleted[i] && (((int8_T)i < 0) || ((int8_T)i > (int8_T)b_i - 1))) {
      emlrtDynamicBoundsCheckR2012b((int8_T)i, 0, (int8_T)b_i - 1, &hb_emlrtBCI,
        &st);
    }
  }

  dt = obj->pNumLiveTracks;
  if (muDoubleScalarIsNaN(dt)) {
    loop_ub = prevInds->size[0] * prevInds->size[1];
    prevInds->size[0] = 1;
    prevInds->size[1] = 1;
    emxEnsureCapacity_real_T(sp, prevInds, loop_ub, &ad_emlrtRTEI);
    prevInds_data = prevInds->data;
    prevInds_data[0] = rtNaN;
  } else if (dt < 1.0) {
    prevInds->size[0] = 1;
    prevInds->size[1] = 0;
  } else {
    loop_ub = prevInds->size[0] * prevInds->size[1];
    prevInds->size[0] = 1;
    prevInds->size[1] = (int32_T)(dt - 1.0) + 1;
    emxEnsureCapacity_real_T(sp, prevInds, loop_ub, &ad_emlrtRTEI);
    prevInds_data = prevInds->data;
    loop_ub = (int32_T)(dt - 1.0);
    times_size_idx_1 = (((int32_T)(dt - 1.0) + 1) / 2) << 1;
    confTracks_size = times_size_idx_1 - 2;
    for (i = 0; i <= confTracks_size; i += 2) {
      b_dv[0] = i;
      b_dv[1] = i + 1;
      r = _mm_loadu_pd(&b_dv[0]);
      _mm_storeu_pd(&prevInds_data[i], _mm_add_pd(_mm_set1_pd(1.0), r));
    }

    for (i = times_size_idx_1; i <= loop_ub; i++) {
      prevInds_data[i] = (real_T)i + 1.0;
    }
  }

  st.site = &fj_emlrtRSI;
  emxInit_struct2_T(&st, &allStructs, &bd_emlrtRTEI);
  allStructs_data = allStructs->data;
  if (prevInds->size[1] > 0) {
    loop_ub = (int32_T)muDoubleScalarFloor(prevInds_data[0]);
    if (prevInds_data[0] != loop_ub) {
      emlrtIntegerCheckR2012b(prevInds_data[0], &i_emlrtDCI, &st);
    }

    times_size_idx_1 = (int32_T)prevInds_data[0] - 1;
    if ((times_size_idx_1 < 0) || (times_size_idx_1 > 99)) {
      emlrtDynamicBoundsCheckR2012b(times_size_idx_1, 0, 99, &w_emlrtBCI, &st);
    }

    track = obj->pTracksList[times_size_idx_1];
    b_st.site = &sv_emlrtRSI;
    oneStruct.TrackID = track.TrackID;
    oneStruct.BranchID = track.BranchID;
    oneStruct.SourceIndex = track.SourceIndex;
    oneStruct.UpdateTime = track.pUpdateTime;
    oneStruct.Age = track.Age;
    for (i = 0; i < 6; i++) {
      oneStruct.State[i] = track.pState[i];
    }

    memcpy(&oneStruct.StateCovariance[0], &track.pStateCovariance[0], 36U *
           sizeof(real_T));
    oneStruct.ObjectClassID = track.ObjectClassID;
    oneStruct.ObjectClassProbabilities = track.ObjectClassProbabilities;
    for (i = 0; i < 7; i++) {
      oneStruct.TrackLogic[i] = logicType[i];
    }

    c_st.site = &vv_emlrtRSI;
    if (prevInds_data[0] != loop_ub) {
      emlrtIntegerCheckR2012b(prevInds_data[0], &h_emlrtDCI, &c_st);
    }

    b_obj = obj->pTrackLogics[times_size_idx_1];
    d_st.site = &wv_emlrtRSI;
    oneStruct.TrackLogicState[0] = b_obj->pRecentHistory[0];
    oneStruct.TrackLogicState[1] = b_obj->pRecentHistory[1];
    oneStruct.TrackLogicState[2] = b_obj->pRecentHistory[2];
    oneStruct.IsConfirmed = track.IsConfirmed;
    oneStruct.IsCoasted = track.IsCoasted;
    oneStruct.IsSelfReported = track.IsSelfReported;
    b_st.site = &tv_emlrtRSI;
    c_st.site = &jk_emlrtRSI;
    times_size_idx_1 = prevInds->size[1];
    loop_ub = allStructs->size[0];
    allStructs->size[0] = prevInds->size[1];
    emxEnsureCapacity_struct2_T(&b_st, allStructs, loop_ub, &bd_emlrtRTEI);
    allStructs_data = allStructs->data;
    for (i = 0; i < times_size_idx_1; i++) {
      allStructs_data[i] = oneStruct;
    }

    b_st.site = &uv_emlrtRSI;
    for (c_i = 0; c_i <= times_size_idx_1 - 2; c_i++) {
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &tb_emlrtBCI,
          &b_st);
      }

      dt = prevInds_data[c_i + 1];
      loop_ub = (int32_T)muDoubleScalarFloor(dt);
      if (dt != loop_ub) {
        emlrtIntegerCheckR2012b(dt, &g_emlrtDCI, &b_st);
      }

      y = (((int32_T)dt - 1 < 0) || ((int32_T)dt - 1 > 99));
      if (y) {
        emlrtDynamicBoundsCheckR2012b((int32_T)dt - 1, 0, 99, &u_emlrtBCI, &b_st);
      }

      allStructs_data[c_i + 1].TrackID = obj->pTracksList[(int32_T)dt - 1].
        TrackID;
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &s_emlrtBCI,
          &b_st);
      }

      if (dt != loop_ub) {
        emlrtIntegerCheckR2012b(dt, &f_emlrtDCI, &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b((int32_T)dt - 1, 0, 99, &t_emlrtBCI, &b_st);
      }

      allStructs_data[c_i + 1].BranchID = obj->pTracksList[(int32_T)dt - 1].
        BranchID;
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &q_emlrtBCI,
          &b_st);
      }

      if (dt != loop_ub) {
        emlrtIntegerCheckR2012b(dt, &e_emlrtDCI, &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b((int32_T)dt - 1, 0, 99, &r_emlrtBCI, &b_st);
      }

      allStructs_data[c_i + 1].Age = obj->pTracksList[(int32_T)dt - 1].Age;
      if (dt != loop_ub) {
        emlrtIntegerCheckR2012b(dt, &d_emlrtDCI, &b_st);
      }

      confTracks_size = (int32_T)dt;
      y = ((confTracks_size - 1 < 0) || (confTracks_size - 1 > 99));
      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &p_emlrtBCI,
          &b_st);
      }

      track = obj->pTracksList[confTracks_size - 1];
      for (i = 0; i < 6; i++) {
        if (c_i + 2 > times_size_idx_1) {
          emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1,
            &o_emlrtBCI, &b_st);
        }

        allStructs_data[c_i + 1].State[i] = track.pState[i];
      }

      if (confTracks_size != loop_ub) {
        emlrtIntegerCheckR2012b(dt, &c_emlrtDCI, &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &n_emlrtBCI,
          &b_st);
      }

      track = obj->pTracksList[confTracks_size - 1];
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &ub_emlrtBCI,
          &b_st);
      }

      for (i = 0; i < 36; i++) {
        allStructs_data[c_i + 1].StateCovariance[i] = track.pStateCovariance[i];
      }

      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &l_emlrtBCI,
          &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &m_emlrtBCI,
          &b_st);
      }

      allStructs_data[c_i + 1].ObjectClassID = obj->pTracksList[confTracks_size
        - 1].ObjectClassID;
      c_st.site = &xv_emlrtRSI;
      if (confTracks_size != loop_ub) {
        emlrtIntegerCheckR2012b(dt, &b_emlrtDCI, &c_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &k_emlrtBCI,
          &c_st);
      }

      b_obj = obj->pTrackLogics[confTracks_size - 1];
      d_st.site = &wv_emlrtRSI;
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &vb_emlrtBCI,
          &d_st);
      }

      allStructs_data[c_i + 1].TrackLogicState[0] = b_obj->pRecentHistory[0];
      allStructs_data[c_i + 1].TrackLogicState[1] = b_obj->pRecentHistory[1];
      allStructs_data[c_i + 1].TrackLogicState[2] = b_obj->pRecentHistory[2];
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &i_emlrtBCI,
          &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &j_emlrtBCI,
          &b_st);
      }

      allStructs_data[c_i + 1].IsConfirmed = obj->pTracksList[confTracks_size -
        1].IsConfirmed;
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &g_emlrtBCI,
          &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &h_emlrtBCI,
          &b_st);
      }

      allStructs_data[c_i + 1].IsCoasted = obj->pTracksList[confTracks_size - 1]
        .IsCoasted;
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &e_emlrtBCI,
          &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &f_emlrtBCI,
          &b_st);
      }

      allStructs_data[c_i + 1].IsSelfReported = obj->pTracksList[confTracks_size
        - 1].IsSelfReported;
      if (c_i + 2 > times_size_idx_1) {
        emlrtDynamicBoundsCheckR2012b(c_i + 2, 1, times_size_idx_1, &c_emlrtBCI,
          &b_st);
      }

      if (y) {
        emlrtDynamicBoundsCheckR2012b(confTracks_size - 1, 0, 99, &d_emlrtBCI,
          &b_st);
      }
    }
  } else {
    allStructs->size[0] = 0;
  }

  emxFree_real_T(&st, &prevInds);
  confTracks_size = 0;
  for (i = 0; i < 100; i++) {
    if (obj->pConfirmedTracks[i]) {
      confTracks_size++;
    }
  }

  loop_ub = 0;
  for (i = 0; i < 100; i++) {
    if (obj->pConfirmedTracks[i]) {
      c_tmp_data[loop_ub] = (int8_T)i;
      loop_ub++;
    }
  }

  for (i = 0; i < confTracks_size; i++) {
    int8_T i2;
    i2 = c_tmp_data[i];
    if (i2 > allStructs->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(i2, 0, allStructs->size[0] - 1, &v_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    confTracks_data[i] = allStructs_data[i2];
  }

  dt = obj->pNumLiveTracks;
  if (dt < 1.0) {
    times_size_idx_1 = 0;
  } else {
    if (dt != (int32_T)muDoubleScalarFloor(dt)) {
      emlrtIntegerCheckR2012b(dt, &j_emlrtDCI, (emlrtConstCTX)sp);
    }

    if (((int32_T)dt < 1) || ((int32_T)dt > 100)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)dt, 1, 100, &kb_emlrtBCI,
        (emlrtConstCTX)sp);
    }

    times_size_idx_1 = (int32_T)dt;
  }

  for (i = 0; i < times_size_idx_1; i++) {
    if ((!obj->pConfirmedTracks[i]) && (i > allStructs->size[0] - 1)) {
      emlrtDynamicBoundsCheckR2012b(i, 0, allStructs->size[0] - 1, &gb_emlrtBCI,
        (emlrtConstCTX)sp);
    }
  }

  emxFree_struct2_T(sp, &allStructs);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return confTracks_size;
}

/* End of code generation (trackFuser.c) */
