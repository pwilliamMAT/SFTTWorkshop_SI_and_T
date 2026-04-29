/*
 * JVProblemSolutionPair.c
 *
 * Code generation for function 'JVProblemSolutionPair'
 *
 */

/* Include files */
#include "JVProblemSolutionPair.h"
#include "AbstractProblemSolutionPair.h"
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo qq_emlrtRSI = {
    238,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo oeb_emlrtRSI = {
    25,                                            /* lineNo */
    "JVProblemSolutionPair/JVProblemSolutionPair", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo peb_emlrtRSI = {
    63,                                                        /* lineNo */
    "AbstractProblemSolutionPair/AbstractProblemSolutionPair", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo qeb_emlrtRSI = {
    47,                                  /* lineNo */
    "lapPadForUnassignedRowsAndColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m" /* pathName */
};

static emlrtRSInfo reb_emlrtRSI = {
    60,                                  /* lineNo */
    "lapPadForUnassignedRowsAndColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m" /* pathName */
};

static emlrtRSInfo seb_emlrtRSI = {
    64,                                  /* lineNo */
    "lapPadForUnassignedRowsAndColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m" /* pathName */
};

static emlrtRSInfo teb_emlrtRSI = {
    77,                         /* lineNo */
    "parseCostOfNonAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m" /* pathName */
};

static emlrtRSInfo ueb_emlrtRSI = {
    78,                         /* lineNo */
    "parseCostOfNonAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m" /* pathName */
};

static emlrtRSInfo fhb_emlrtRSI = {
    61,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo ghb_emlrtRSI = {
    62,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo hhb_emlrtRSI = {
    68,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo ihb_emlrtRSI = {
    70,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo jhb_emlrtRSI = {
    71,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo khb_emlrtRSI = {
    72,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo lhb_emlrtRSI = {
    75,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo mhb_emlrtRSI = {
    79,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo nhb_emlrtRSI = {
    80,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo ohb_emlrtRSI = {
    35,                                         /* lineNo */
    "StrictSingleCoderUtilities/IntLogicalSum", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\+"
    "matlabshared\\+tracking\\+internal\\+fusion\\+codegen\\"
    "StrictSingleCoderUtilities.m" /* pathName */
};

static emlrtRSInfo phb_emlrtRSI = {
    43,                                    /* lineNo */
    "JVProblemSolutionPair/computeSlacks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo qhb_emlrtRSI = {
    37,                                          /* lineNo */
    "JVProblemSolutionPair/computeMinimumSlack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo rhb_emlrtRSI = {
    17,    /* lineNo */
    "max", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\max.m" /* pathName
                                                                        */
};

static emlrtRSInfo shb_emlrtRSI =
    {
        67,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo thb_emlrtRSI =
    {
        106,       /* lineNo */
        "maximum", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo uhb_emlrtRSI = {
    22,               /* lineNo */
    "nullAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtRSInfo vhb_emlrtRSI = {
    26,               /* lineNo */
    "nullAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtRSInfo whb_emlrtRSI = {
    274,           /* lineNo */
    "delete_rows", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtBCInfo me_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    65,                                  /* lineNo */
    33,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo ne_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    65,                                  /* lineNo */
    29,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo oe_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    65,                                  /* lineNo */
    21,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo pe_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    61,                                  /* lineNo */
    31,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo qe_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    61,                                  /* lineNo */
    23,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo re_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    61,                                  /* lineNo */
    20,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    27,                                            /* lineNo */
    40,                                            /* colNo */
    "JVProblemSolutionPair/JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    1                                                 /* checkKind */
};

static emlrtBCInfo se_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    62,                                  /* lineNo */
    52,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo te_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    62,                                  /* lineNo */
    20,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo ue_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    62,                                  /* lineNo */
    22,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo ve_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    66,                                  /* lineNo */
    52,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo we_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    66,                                  /* lineNo */
    20,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo xe_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    66,                                  /* lineNo */
    27,                                  /* colNo */
    "",                                  /* aName */
    "lapPadForUnassignedRowsAndColumns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\lapPadForUnassignedRowsAndCol"
    "umns.m", /* pName */
    0         /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    28,                                            /* lineNo */
    13,                                            /* colNo */
    "JVProblemSolutionPair/JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    1                                                 /* checkKind */
};

static emlrtRTEInfo vb_emlrtRTEI = {
    296,           /* lineNo */
    1,             /* colNo */
    "delete_rows", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pName */
};

static emlrtRTEInfo wb_emlrtRTEI = {
    81,                /* lineNo */
    27,                /* colNo */
    "validate_inputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pName */
};

static emlrtECInfo q_emlrtECI = {
    2,                                           /* nDims */
    37,                                          /* lineNo */
    28,                                          /* colNo */
    "JVProblemSolutionPair/computeMinimumSlack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtBCInfo ye_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    35,                                          /* lineNo */
    35,                                          /* colNo */
    "",                                          /* aName */
    "JVProblemSolutionPair/computeMinimumSlack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo af_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    33,                                          /* lineNo */
    38,                                          /* colNo */
    "",                                          /* aName */
    "JVProblemSolutionPair/computeMinimumSlack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo bf_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    83,                                         /* lineNo */
    36,                                         /* colNo */
    "",                                         /* aName */
    "AbstractProblemSolutionPair/enforceTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo cf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    43,                                    /* lineNo */
    61,                                    /* colNo */
    "",                                    /* aName */
    "JVProblemSolutionPair/computeSlacks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo df_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    69,                                /* lineNo */
    31,                                /* colNo */
    "",                                /* aName */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo ef_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    73,                                /* lineNo */
    34,                                /* colNo */
    "",                                /* aName */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtECInfo r_emlrtECI = {
    -1,                                /* nDims */
    68,                                /* lineNo */
    13,                                /* colNo */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtDCInfo h_emlrtDCI = {
    66,                                /* lineNo */
    115,                               /* colNo */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    4                                                 /* checkKind */
};

static emlrtRTEInfo ac_emlrtRTEI = {
    63,                                /* lineNo */
    13,                                /* colNo */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtECInfo s_emlrtECI = {
    1,                                 /* nDims */
    57,                                /* lineNo */
    23,                                /* colNo */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtBCInfo ff_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    68,                                /* lineNo */
    35,                                /* colNo */
    "",                                /* aName */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo gf_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    34,                                          /* lineNo */
    15,                                          /* colNo */
    "",                                          /* aName */
    "JVProblemSolutionPair/computeMinimumSlack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo hf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    43,                                    /* lineNo */
    24,                                    /* colNo */
    "",                                    /* aName */
    "JVProblemSolutionPair/computeSlacks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo if_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    95,                                        /* lineNo */
    34,                                        /* colNo */
    "",                                        /* aName */
    "AbstractProblemSolutionPair/removeTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo jf_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    95,                                        /* lineNo */
    43,                                        /* colNo */
    "",                                        /* aName */
    "AbstractProblemSolutionPair/removeTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo kf_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    96,                                        /* lineNo */
    25,                                        /* colNo */
    "",                                        /* aName */
    "AbstractProblemSolutionPair/removeTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo lf_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    97,                                        /* lineNo */
    25,                                        /* colNo */
    "",                                        /* aName */
    "AbstractProblemSolutionPair/removeTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo mf_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    78,                                /* lineNo */
    26,                                /* colNo */
    "",                                /* aName */
    "JVProblemSolutionPair/partition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m", /* pName */
    0                                                 /* checkKind */
};

static emlrtBCInfo nf_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    82,                                         /* lineNo */
    50,                                         /* colNo */
    "",                                         /* aName */
    "AbstractProblemSolutionPair/enforceTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo of_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    85,                                         /* lineNo */
    43,                                         /* colNo */
    "",                                         /* aName */
    "AbstractProblemSolutionPair/enforceTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtBCInfo pf_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    88,                                         /* lineNo */
    28,                                         /* colNo */
    "",                                         /* aName */
    "AbstractProblemSolutionPair/enforceTuple", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtRTEInfo ug_emlrtRTEI = {
    63,                            /* lineNo */
    13,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo vg_emlrtRTEI = {
    24,                      /* lineNo */
    18,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo wg_emlrtRTEI = {
    27,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo xg_emlrtRTEI = {
    28,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo gh_emlrtRTEI = {
    55,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo hh_emlrtRTEI = {
    57,                      /* lineNo */
    23,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo ih_emlrtRTEI = {
    57,                      /* lineNo */
    41,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo jh_emlrtRTEI = {
    68,                      /* lineNo */
    35,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo kh_emlrtRTEI = {
    68,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo lh_emlrtRTEI = {
    69,                      /* lineNo */
    31,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo mh_emlrtRTEI = {
    71,                      /* lineNo */
    17,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo nh_emlrtRTEI = {
    33,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo oh_emlrtRTEI = {
    37,                      /* lineNo */
    28,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo ph_emlrtRTEI = {
    75,                      /* lineNo */
    17,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo qh_emlrtRTEI = {
    78,                      /* lineNo */
    17,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo rh_emlrtRTEI = {
    80,                      /* lineNo */
    17,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo sh_emlrtRTEI = {
    57,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo th_emlrtRTEI = {
    66,                      /* lineNo */
    13,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo uh_emlrtRTEI = {
    62,                      /* lineNo */
    98,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRSInfo inb_emlrtRSI = {
    57,                                /* lineNo */
    "JVProblemSolutionPair/partition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

/* Function Declarations */
static void b_and(const emlrtStack *sp, emxArray_boolean_T *in1,
                  const emxArray_boolean_T *in2);

/* Function Definitions */
static void b_and(const emlrtStack *sp, emxArray_boolean_T *in1,
                  const emxArray_boolean_T *in2)
{
  jmp_buf *volatile emlrtJBStack;
  emxArray_boolean_T *b_in1;
  int32_T b_and_numThreads;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  const boolean_T *in2_data;
  boolean_T *b_in1_data;
  boolean_T *in1_data;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &b_in1, 1, &hh_emlrtRTEI, true);
  loop_ub = in1->size[0];
  stride_0_0 = b_in1->size[0];
  b_in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, b_in1, stride_0_0, &hh_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in2->size[0] != 1);
  if (loop_ub < 800) {
    for (i = 0; i < loop_ub; i++) {
      b_in1_data[i] = (in1_data[i] && in2_data[i * stride_0_0]);
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    b_and_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(b_and_numThreads)

    for (i = 0; i < loop_ub; i++) {
      b_in1_data[i] = (in1_data[i] && in2_data[i * stride_0_0]);
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  stride_0_0 = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, in1, stride_0_0, &hh_emlrtRTEI);
  in1_data = in1->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    in1_data[i1] = b_in1_data[i1];
  }
  emxFree_boolean_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void JVProblemSolutionPair_partition(
    const emlrtStack *sp, const emxArray_real_T *obj_PaddedCostMatrix,
    const emxArray_real_T *obj_RowSoln, const emxArray_real_T *obj_ColSoln,
    const emxArray_boolean_T *obj_IsEnforced, const int32_T obj_CostSize[2],
    real_T obj_BestSolutionCost, const emxArray_boolean_T *obj_IsDummySolution,
    const emxArray_real_T *obj_ColReduction,
    const emxArray_real_T *obj_RowReduction,
    c_emxArray_fusion_internal_assi *objArray)
{
  jmp_buf *volatile emlrtJBStack;
  c_fusion_internal_assignment_JV expl_temp;
  c_fusion_internal_assignment_JV *objArray_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack st;
  emxArray_boolean_T *isValid;
  emxArray_boolean_T *r;
  emxArray_int32_T *S;
  emxArray_int32_T *r3;
  emxArray_real_T *P_ColSoln;
  emxArray_real_T *P_PaddedCostMatrix;
  emxArray_real_T *P_RowSoln;
  emxArray_real_T *l;
  emxArray_real_T *r2;
  emxArray_real_T *slacks;
  const real_T *obj_ColReduction_data;
  const real_T *obj_ColSoln_data;
  const real_T *obj_PaddedCostMatrix_data;
  const real_T *obj_RowReduction_data;
  const real_T *obj_RowSoln_data;
  real_T *P_ColSoln_data;
  real_T *P_PaddedCostMatrix_data;
  real_T *P_RowSoln_data;
  real_T *l_data;
  real_T *slacks_data;
  int32_T JVProblemSolutionPair_partition_numThreads;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T c_loop_ub;
  int32_T d_i;
  int32_T d_loop_ub;
  int32_T e_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T last;
  int32_T loop_ub;
  int32_T nz;
  int32_T *S_data;
  int32_T *r4;
  const boolean_T *obj_IsDummySolution_data;
  const boolean_T *obj_IsEnforced_data;
  boolean_T *isValid_data;
  boolean_T *r1;
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
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  obj_RowReduction_data = obj_RowReduction->data;
  obj_ColReduction_data = obj_ColReduction->data;
  obj_IsDummySolution_data = obj_IsDummySolution->data;
  obj_IsEnforced_data = obj_IsEnforced->data;
  obj_ColSoln_data = obj_ColSoln->data;
  obj_RowSoln_data = obj_RowSoln->data;
  obj_PaddedCostMatrix_data = obj_PaddedCostMatrix->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &P_PaddedCostMatrix, 2, &gh_emlrtRTEI, true);
  loop_ub = P_PaddedCostMatrix->size[0] * P_PaddedCostMatrix->size[1];
  P_PaddedCostMatrix->size[0] = obj_PaddedCostMatrix->size[0];
  P_PaddedCostMatrix->size[1] = obj_PaddedCostMatrix->size[1];
  emxEnsureCapacity_real_T(sp, P_PaddedCostMatrix, loop_ub, &gh_emlrtRTEI);
  P_PaddedCostMatrix_data = P_PaddedCostMatrix->data;
  loop_ub = obj_PaddedCostMatrix->size[0] * obj_PaddedCostMatrix->size[1];
  for (i = 0; i < loop_ub; i++) {
    P_PaddedCostMatrix_data[i] = obj_PaddedCostMatrix_data[i];
  }
  emxInit_real_T(sp, &P_RowSoln, 1, &gh_emlrtRTEI, true);
  idx = obj_RowSoln->size[0];
  loop_ub = P_RowSoln->size[0];
  P_RowSoln->size[0] = obj_RowSoln->size[0];
  emxEnsureCapacity_real_T(sp, P_RowSoln, loop_ub, &gh_emlrtRTEI);
  P_RowSoln_data = P_RowSoln->data;
  for (i = 0; i < idx; i++) {
    P_RowSoln_data[i] = obj_RowSoln_data[i];
  }
  emxInit_real_T(sp, &P_ColSoln, 2, &gh_emlrtRTEI, true);
  loop_ub = P_ColSoln->size[0] * P_ColSoln->size[1];
  P_ColSoln->size[0] = 1;
  idx = obj_ColSoln->size[1];
  P_ColSoln->size[1] = obj_ColSoln->size[1];
  emxEnsureCapacity_real_T(sp, P_ColSoln, loop_ub, &gh_emlrtRTEI);
  P_ColSoln_data = P_ColSoln->data;
  for (i = 0; i < idx; i++) {
    P_ColSoln_data[i] = obj_ColSoln_data[i];
  }
  idx = obj_IsEnforced->size[0];
  c_emxInitStruct_fusion_internal(sp, &expl_temp, &qh_emlrtRTEI, true);
  loop_ub = expl_temp.IsEnforced->size[0];
  expl_temp.IsEnforced->size[0] = obj_IsEnforced->size[0];
  emxEnsureCapacity_boolean_T(sp, expl_temp.IsEnforced, loop_ub, &gh_emlrtRTEI);
  emxInit_boolean_T(sp, &isValid, 1, &sh_emlrtRTEI, true);
  loop_ub = isValid->size[0];
  isValid->size[0] = obj_IsEnforced->size[0];
  emxEnsureCapacity_boolean_T(sp, isValid, loop_ub, &hh_emlrtRTEI);
  isValid_data = isValid->data;
  for (i = 0; i < idx; i++) {
    expl_temp.IsEnforced->data[i] = obj_IsEnforced_data[i];
    isValid_data[i] = !obj_IsEnforced_data[i];
  }
  emxInit_boolean_T(sp, &r, 1, &hh_emlrtRTEI, true);
  b_loop_ub = obj_IsDummySolution->size[0];
  loop_ub = r->size[0];
  r->size[0] = obj_IsDummySolution->size[0];
  emxEnsureCapacity_boolean_T(sp, r, loop_ub, &ih_emlrtRTEI);
  r1 = r->data;
  loop_ub = obj_IsDummySolution->size[0];
  if (obj_IsDummySolution->size[0] < 800) {
    for (b_i = 0; b_i < b_loop_ub; b_i++) {
      r1[b_i] = !obj_IsDummySolution_data[b_i];
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    JVProblemSolutionPair_partition_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(JVProblemSolutionPair_partition_numThreads)

    for (b_i = 0; b_i < loop_ub; b_i++) {
      r1[b_i] = !obj_IsDummySolution_data[b_i];
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  if ((idx != b_loop_ub) && ((idx != 1) && (b_loop_ub != 1))) {
    emlrtDimSizeImpxCheckR2021b(idx, b_loop_ub, &s_emlrtECI, (emlrtConstCTX)sp);
  }
  if (isValid->size[0] == r->size[0]) {
    if (isValid->size[0] < 800) {
      for (i1 = 0; i1 < idx; i1++) {
        isValid_data[i1] = (isValid_data[i1] && r1[i1]);
      }
    } else {
      emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
      emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
      JVProblemSolutionPair_partition_numThreads =
          emlrtAllocRegionTLSs(sp->tls, omp_in_parallel(),
                               omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(JVProblemSolutionPair_partition_numThreads)

      for (i1 = 0; i1 < idx; i1++) {
        isValid_data[i1] = (isValid_data[i1] && r1[i1]);
      }
      emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
      emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
    }
  } else {
    st.site = &inb_emlrtRSI;
    b_and(&st, isValid, r);
    isValid_data = isValid->data;
  }
  emxFree_boolean_T(sp, &r);
  st.site = &fhb_emlrtRSI;
  b_st.site = &ohb_emlrtRSI;
  c_st.site = &obb_emlrtRSI;
  d_st.site = &pbb_emlrtRSI;
  nz = c_combineVectorElements(&d_st, isValid);
  emxInit_real_T(sp, &r2, 2, &uh_emlrtRTEI, true);
  st.site = &ghb_emlrtRSI;
  c_AbstractProblemSolutionPair_g(&st, obj_RowSoln, obj_CostSize, r2);
  if (nz > r2->size[0]) {
    emlrtErrorWithMessageIdR2018a(sp, &ac_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (nz < 0) {
    emlrtNonNegativeCheckR2012b(nz, &h_emlrtDCI, (emlrtConstCTX)sp);
  }
  st.site = &hhb_emlrtRSI;
  c_AbstractProblemSolutionPair_g(&st, obj_RowSoln, obj_CostSize, r2);
  slacks_data = r2->data;
  last = isValid->size[0];
  idx = 0;
  for (i = 0; i < last; i++) {
    if (isValid_data[i]) {
      idx++;
    }
  }
  emxInit_int32_T(sp, &r3, 1, &jh_emlrtRTEI);
  loop_ub = r3->size[0];
  r3->size[0] = idx;
  emxEnsureCapacity_int32_T(sp, r3, loop_ub, &jh_emlrtRTEI);
  r4 = r3->data;
  loop_ub = 0;
  for (i = 0; i < last; i++) {
    if (isValid_data[i]) {
      r4[loop_ub] = i;
      loop_ub++;
    }
  }
  emxFree_boolean_T(sp, &isValid);
  emxInit_int32_T(sp, &S, 2, &th_emlrtRTEI);
  idx = r3->size[0];
  loop_ub = S->size[0] * S->size[1];
  S->size[0] = r3->size[0];
  S->size[1] = 2;
  emxEnsureCapacity_int32_T(sp, S, loop_ub, &kh_emlrtRTEI);
  S_data = S->data;
  for (i = 0; i < 2; i++) {
    for (c_i = 0; c_i < idx; c_i++) {
      if (r4[c_i] > r2->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(r4[c_i], 0, r2->size[0] - 1, &ff_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      S_data[c_i + S->size[0] * i] =
          (int32_T)slacks_data[r4[c_i] + r2->size[0] * i];
    }
  }
  emxFree_int32_T(sp, &r3);
  emxFree_real_T(sp, &r2);
  loop_ub = nz << 1;
  idx = S->size[0] << 1;
  if (loop_ub != idx) {
    emlrtSubAssignSizeCheck1dR2017a(loop_ub, idx, &r_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  for (i = 0; i < nz; i++) {
    if (i > nz - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, nz - 1, &df_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
  }
  st.site = &ihb_emlrtRSI;
  if (nz > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  loop_ub = objArray->size[0];
  objArray->size[0] = nz;
  c_emxEnsureCapacity_fusion_inte(sp, objArray, loop_ub, &lh_emlrtRTEI);
  objArray_data = objArray->data;
  if (nz - 1 >= 0) {
    expl_temp.BestSolutionCost = obj_BestSolutionCost;
    c_loop_ub = b_loop_ub;
    expl_temp.IsSolved = false;
    d_loop_ub = obj_ColReduction->size[1];
    e_loop_ub = obj_RowReduction->size[0];
    expl_temp.CostSize[0] = obj_CostSize[0];
    expl_temp.CostSize[1] = obj_CostSize[1];
  }
  emxInit_real_T(sp, &slacks, 1, &mh_emlrtRTEI, true);
  emxInit_real_T(sp, &l, 2, &nh_emlrtRTEI, true);
  for (d_i = 0; d_i < nz; d_i++) {
    real_T d;
    real_T temp;
    int32_T f_loop_ub;
    int32_T g_loop_ub;
    int32_T iindx;
    boolean_T b;
    boolean_T exitg1;
    st.site = &jhb_emlrtRSI;
    f_loop_ub = S->size[0];
    loop_ub = slacks->size[0];
    slacks->size[0] = S->size[0];
    emxEnsureCapacity_real_T(&st, slacks, loop_ub, &mh_emlrtRTEI);
    slacks_data = slacks->data;
    for (i = 0; i < f_loop_ub; i++) {
      slacks_data[i] = 0.0;
    }
    for (c_i = 0; c_i < f_loop_ub; c_i++) {
      b_st.site = &phb_emlrtRSI;
      if (c_i + 1 > f_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, f_loop_ub, &cf_emlrtBCI,
                                      &b_st);
      }
      if ((S_data[c_i] < 1) || (S_data[c_i] > P_PaddedCostMatrix->size[0])) {
        emlrtDynamicBoundsCheckR2012b(
            S_data[c_i], 1, P_PaddedCostMatrix->size[0], &af_emlrtBCI, &b_st);
      }
      idx = S_data[c_i];
      loop_ub = l->size[0] * l->size[1];
      l->size[0] = 1;
      last = P_PaddedCostMatrix->size[1];
      l->size[1] = P_PaddedCostMatrix->size[1];
      emxEnsureCapacity_real_T(&b_st, l, loop_ub, &nh_emlrtRTEI);
      l_data = l->data;
      for (i = 0; i < last; i++) {
        l_data[i] =
            P_PaddedCostMatrix_data[(idx + P_PaddedCostMatrix->size[0] * i) -
                                    1];
      }
      if ((S_data[c_i + S->size[0]] < 1) ||
          (S_data[c_i + S->size[0]] > P_PaddedCostMatrix->size[1])) {
        emlrtDynamicBoundsCheckR2012b(S_data[c_i + S->size[0]], 1,
                                      P_PaddedCostMatrix->size[1], &gf_emlrtBCI,
                                      &b_st);
      }
      l_data[S_data[c_i + S->size[0]] - 1] = rtInf;
      if ((S_data[c_i] < 1) || (S_data[c_i] > obj_RowReduction->size[0])) {
        emlrtDynamicBoundsCheckR2012b(S_data[c_i], 1, obj_RowReduction->size[0],
                                      &ye_emlrtBCI, &b_st);
      }
      if ((l->size[1] != obj_ColReduction->size[1]) &&
          ((l->size[1] != 1) && (obj_ColReduction->size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(l->size[1], obj_ColReduction->size[1],
                                    &q_emlrtECI, &b_st);
      }
      c_st.site = &qhb_emlrtRSI;
      loop_ub = l->size[0] * l->size[1];
      l->size[0] = 1;
      emxEnsureCapacity_real_T(&c_st, l, loop_ub, &oh_emlrtRTEI);
      l_data = l->data;
      temp = obj_RowReduction_data[S_data[c_i] - 1];
      loop_ub = l->size[1] - 1;
      idx = (l->size[1] / 2) << 1;
      last = idx - 2;
      for (i = 0; i <= last; i += 2) {
        __m128d r5;
        r5 = _mm_loadu_pd(&l_data[i]);
        _mm_storeu_pd(&l_data[i],
                      _mm_sub_pd(_mm_sub_pd(r5, _mm_set1_pd(temp)),
                                 _mm_loadu_pd(&obj_ColReduction_data[i])));
      }
      for (i = idx; i <= loop_ub; i++) {
        l_data[i] = (l_data[i] - temp) - obj_ColReduction_data[i];
      }
      d_st.site = &jgb_emlrtRSI;
      e_st.site = &kgb_emlrtRSI;
      f_st.site = &lgb_emlrtRSI;
      if (l->size[1] < 1) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &xb_emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
            "Coder:toolbox:eml_min_or_max_varDimZero", 0);
      }
      g_st.site = &mgb_emlrtRSI;
      h_st.site = &ngb_emlrtRSI;
      last = l->size[1];
      if (l->size[1] <= 2) {
        if (l->size[1] == 1) {
          if (c_i + 1 > slacks->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, slacks->size[0],
                                          &hf_emlrtBCI, &h_st);
          }
          slacks_data[c_i] = l_data[0];
        } else if ((l_data[0] > l_data[1]) ||
                   (muDoubleScalarIsNaN(l_data[0]) &&
                    (!muDoubleScalarIsNaN(l_data[1])))) {
          if (c_i + 1 > slacks->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, slacks->size[0],
                                          &hf_emlrtBCI, &h_st);
          }
          slacks_data[c_i] = l_data[1];
        } else {
          if (c_i + 1 > slacks->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, slacks->size[0],
                                          &hf_emlrtBCI, &h_st);
          }
          slacks_data[c_i] = l_data[0];
        }
      } else {
        i_st.site = &hn_emlrtRSI;
        if (!muDoubleScalarIsNaN(l_data[0])) {
          idx = 1;
        } else {
          idx = 0;
          j_st.site = &in_emlrtRSI;
          if (l->size[1] > 2147483646) {
            k_st.site = &k_emlrtRSI;
            check_forloop_overflow_error(&k_st);
          }
          loop_ub = 2;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= last)) {
            if (!muDoubleScalarIsNaN(l_data[loop_ub - 1])) {
              idx = loop_ub;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
        }
        if (idx == 0) {
          if (c_i + 1 > slacks->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, slacks->size[0],
                                          &hf_emlrtBCI, &h_st);
          }
          slacks_data[c_i] = l_data[0];
        } else {
          i_st.site = &gn_emlrtRSI;
          temp = l_data[idx - 1];
          loop_ub = idx + 1;
          j_st.site = &jn_emlrtRSI;
          if ((idx + 1 <= l->size[1]) && (l->size[1] > 2147483646)) {
            k_st.site = &k_emlrtRSI;
            check_forloop_overflow_error(&k_st);
          }
          for (i = loop_ub; i <= last; i++) {
            d = l_data[i - 1];
            if (temp > d) {
              temp = d;
            }
          }
          if (c_i + 1 > slacks->size[0]) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, slacks->size[0],
                                          &hf_emlrtBCI, &h_st);
          }
          slacks_data[c_i] = temp;
        }
      }
    }
    st.site = &khb_emlrtRSI;
    b_st.site = &rhb_emlrtRSI;
    c_st.site = &shb_emlrtRSI;
    d_st.site = &thb_emlrtRSI;
    if (slacks->size[0] < 1) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &xb_emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }
    e_st.site = &qq_emlrtRSI;
    idx = slacks->size[0];
    if (slacks->size[0] <= 2) {
      if (slacks->size[0] == 1) {
        temp = slacks_data[0];
        iindx = 0;
      } else if ((slacks_data[0] < slacks_data[1]) ||
                 (muDoubleScalarIsNaN(slacks_data[0]) &&
                  (!muDoubleScalarIsNaN(slacks_data[1])))) {
        temp = slacks_data[1];
        iindx = 1;
      } else {
        temp = slacks_data[0];
        iindx = 0;
      }
    } else {
      f_st.site = &hn_emlrtRSI;
      if (!muDoubleScalarIsNaN(slacks_data[0])) {
        last = 1;
      } else {
        last = 0;
        g_st.site = &in_emlrtRSI;
        if (slacks->size[0] > 2147483646) {
          h_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }
        loop_ub = 2;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= idx)) {
          if (!muDoubleScalarIsNaN(slacks_data[loop_ub - 1])) {
            last = loop_ub;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
      }
      if (last == 0) {
        temp = slacks_data[0];
        iindx = 0;
      } else {
        f_st.site = &gn_emlrtRSI;
        temp = slacks_data[last - 1];
        iindx = last - 1;
        loop_ub = last + 1;
        g_st.site = &jn_emlrtRSI;
        if ((last + 1 <= slacks->size[0]) && (slacks->size[0] > 2147483646)) {
          h_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }
        for (i = loop_ub; i <= idx; i++) {
          d = slacks_data[i - 1];
          if (temp < d) {
            temp = d;
            iindx = i - 1;
          }
        }
      }
    }
    if ((iindx + 1 < 1) || (iindx + 1 > S->size[0])) {
      emlrtDynamicBoundsCheckR2012b(iindx + 1, 1, S->size[0], &ef_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    st.site = &lhb_emlrtRSI;
    last = P_PaddedCostMatrix->size[0];
    loop_ub = expl_temp.PaddedCostMatrix->size[0] *
              expl_temp.PaddedCostMatrix->size[1];
    expl_temp.PaddedCostMatrix->size[0] = P_PaddedCostMatrix->size[0];
    g_loop_ub = P_PaddedCostMatrix->size[1];
    expl_temp.PaddedCostMatrix->size[1] = P_PaddedCostMatrix->size[1];
    emxEnsureCapacity_real_T(&st, expl_temp.PaddedCostMatrix, loop_ub,
                             &ph_emlrtRTEI);
    loop_ub = P_PaddedCostMatrix->size[0] * P_PaddedCostMatrix->size[1];
    for (i = 0; i < loop_ub; i++) {
      expl_temp.PaddedCostMatrix->data[i] = P_PaddedCostMatrix_data[i];
    }
    idx = P_RowSoln->size[0];
    loop_ub = expl_temp.RowSoln->size[0];
    expl_temp.RowSoln->size[0] = P_RowSoln->size[0];
    emxEnsureCapacity_real_T(&st, expl_temp.RowSoln, loop_ub, &ph_emlrtRTEI);
    for (i = 0; i < idx; i++) {
      expl_temp.RowSoln->data[i] = P_RowSoln_data[i];
    }
    loop_ub = expl_temp.ColSoln->size[0] * expl_temp.ColSoln->size[1];
    expl_temp.ColSoln->size[0] = 1;
    idx = P_ColSoln->size[1];
    expl_temp.ColSoln->size[1] = P_ColSoln->size[1];
    emxEnsureCapacity_real_T(&st, expl_temp.ColSoln, loop_ub, &ph_emlrtRTEI);
    for (i = 0; i < idx; i++) {
      expl_temp.ColSoln->data[i] = P_ColSoln_data[i];
    }
    b = ((S_data[iindx] < 1) || (S_data[iindx] > P_PaddedCostMatrix->size[0]));
    if (b) {
      emlrtDynamicBoundsCheckR2012b(
          S_data[iindx], 1, P_PaddedCostMatrix->size[0], &if_emlrtBCI, &st);
    }
    if ((S_data[iindx + S->size[0]] < 1) ||
        (S_data[iindx + S->size[0]] > P_PaddedCostMatrix->size[1])) {
      emlrtDynamicBoundsCheckR2012b(S_data[iindx + S->size[0]], 1,
                                    P_PaddedCostMatrix->size[1], &jf_emlrtBCI,
                                    &st);
    }
    expl_temp.PaddedCostMatrix
        ->data[(S_data[iindx] + expl_temp.PaddedCostMatrix->size[0] *
                                    (S_data[iindx + S->size[0]] - 1)) -
               1] = rtInf;
    b = ((S_data[iindx] < 1) || (S_data[iindx] > P_RowSoln->size[0]));
    if (b) {
      emlrtDynamicBoundsCheckR2012b(S_data[iindx], 1, P_RowSoln->size[0],
                                    &kf_emlrtBCI, &st);
    }
    expl_temp.RowSoln->data[S_data[iindx] - 1] = rtNaN;
    b = ((S_data[iindx + S->size[0]] < 1) ||
         (S_data[iindx + S->size[0]] > P_ColSoln->size[1]));
    if (b) {
      emlrtDynamicBoundsCheckR2012b(S_data[iindx + S->size[0]], 1,
                                    P_ColSoln->size[1], &lf_emlrtBCI, &st);
    }
    expl_temp.ColSoln->data[S_data[iindx + S->size[0]] - 1] = rtNaN;
    loop_ub = expl_temp.IsDummySolution->size[0];
    expl_temp.IsDummySolution->size[0] = b_loop_ub;
    emxEnsureCapacity_boolean_T(sp, expl_temp.IsDummySolution, loop_ub,
                                &qh_emlrtRTEI);
    for (i = 0; i < c_loop_ub; i++) {
      expl_temp.IsDummySolution->data[i] = obj_IsDummySolution_data[i];
    }
    loop_ub = expl_temp.ColReduction->size[0] * expl_temp.ColReduction->size[1];
    expl_temp.ColReduction->size[0] = 1;
    expl_temp.ColReduction->size[1] = obj_ColReduction->size[1];
    emxEnsureCapacity_real_T(sp, expl_temp.ColReduction, loop_ub,
                             &qh_emlrtRTEI);
    for (i = 0; i < d_loop_ub; i++) {
      expl_temp.ColReduction->data[i] = obj_ColReduction_data[i];
    }
    loop_ub = expl_temp.RowReduction->size[0];
    expl_temp.RowReduction->size[0] = obj_RowReduction->size[0];
    emxEnsureCapacity_real_T(sp, expl_temp.RowReduction, loop_ub,
                             &qh_emlrtRTEI);
    for (i = 0; i < e_loop_ub; i++) {
      expl_temp.RowReduction->data[i] = obj_RowReduction_data[i];
    }
    expl_temp.LowerBound = obj_BestSolutionCost + temp;
    if (d_i > objArray->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(d_i, 0, objArray->size[0] - 1, &mf_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    c_emxCopyStruct_fusion_internal(sp, &objArray_data[d_i], &expl_temp,
                                    &qh_emlrtRTEI);
    st.site = &mhb_emlrtRSI;
    if ((S_data[iindx + S->size[0]] < 1) ||
        (S_data[iindx + S->size[0]] > P_PaddedCostMatrix->size[1])) {
      emlrtDynamicBoundsCheckR2012b(S_data[iindx + S->size[0]], 1,
                                    P_PaddedCostMatrix->size[1], &nf_emlrtBCI,
                                    &st);
    }
    temp = P_PaddedCostMatrix_data[(S_data[iindx] +
                                    P_PaddedCostMatrix->size[0] *
                                        (S_data[iindx + S->size[0]] - 1)) -
                                   1];
    if ((S_data[iindx + S->size[0]] < 1) ||
        (S_data[iindx + S->size[0]] > P_PaddedCostMatrix->size[1])) {
      emlrtDynamicBoundsCheckR2012b(S_data[iindx + S->size[0]], 1,
                                    P_PaddedCostMatrix->size[1], &bf_emlrtBCI,
                                    &st);
    }
    loop_ub = S_data[iindx + S->size[0]];
    for (i = 0; i < last; i++) {
      P_PaddedCostMatrix_data[i + P_PaddedCostMatrix->size[0] * (loop_ub - 1)] =
          rtInf;
    }
    loop_ub = S_data[iindx];
    for (i = 0; i < g_loop_ub; i++) {
      P_PaddedCostMatrix_data[(loop_ub + P_PaddedCostMatrix->size[0] * i) - 1] =
          rtInf;
    }
    if ((S_data[iindx + S->size[0]] < 1) ||
        (S_data[iindx + S->size[0]] > P_PaddedCostMatrix->size[1])) {
      emlrtDynamicBoundsCheckR2012b(S_data[iindx + S->size[0]], 1,
                                    P_PaddedCostMatrix->size[1], &of_emlrtBCI,
                                    &st);
    }
    P_PaddedCostMatrix_data[(S_data[iindx] +
                             P_PaddedCostMatrix->size[0] *
                                 (S_data[iindx + S->size[0]] - 1)) -
                            1] = temp;
    P_RowSoln_data[S_data[iindx] - 1] = S_data[iindx + S->size[0]];
    P_ColSoln_data[S_data[iindx + S->size[0]] - 1] = S_data[iindx];
    if ((S_data[iindx] < 1) ||
        (S_data[iindx] > expl_temp.IsEnforced->size[0])) {
      emlrtDynamicBoundsCheckR2012b(
          S_data[iindx], 1, expl_temp.IsEnforced->size[0], &pf_emlrtBCI, &st);
    }
    expl_temp.IsEnforced->data[S_data[iindx] - 1] = true;
    st.site = &nhb_emlrtRSI;
    b_st.site = &uhb_emlrtRSI;
    if (iindx + 1 > S->size[0]) {
      emlrtErrorWithMessageIdR2018a(&b_st, &wb_emlrtRTEI,
                                    "MATLAB:subsdeldimmismatch",
                                    "MATLAB:subsdeldimmismatch", 0);
    }
    b_st.site = &vhb_emlrtRSI;
    for (i = 0; i < 2; i++) {
      c_st.site = &whb_emlrtRSI;
      for (c_i = iindx + 1; c_i < f_loop_ub; c_i++) {
        S_data[(c_i + S->size[0] * i) - 1] = S_data[c_i + S->size[0] * i];
      }
    }
    if (S->size[0] - 1 > S->size[0]) {
      emlrtErrorWithMessageIdR2018a(&b_st, &vb_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (S->size[0] - 1 < 1) {
      idx = 0;
    } else {
      idx = S->size[0] - 1;
    }
    for (i = 0; i < 2; i++) {
      for (c_i = 0; c_i < idx; c_i++) {
        S_data[c_i + idx * i] = S_data[c_i + S->size[0] * i];
      }
    }
    loop_ub = S->size[0] * S->size[1];
    S->size[0] = idx;
    S->size[1] = 2;
    emxEnsureCapacity_int32_T(&b_st, S, loop_ub, &rh_emlrtRTEI);
    S_data = S->data;
  }
  c_emxFreeStruct_fusion_internal(sp, &expl_temp);
  emxFree_real_T(sp, &l);
  emxFree_real_T(sp, &slacks);
  emxFree_int32_T(sp, &S);
  emxFree_real_T(sp, &P_ColSoln);
  emxFree_real_T(sp, &P_RowSoln);
  emxFree_real_T(sp, &P_PaddedCostMatrix);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

real_T c_JVProblemSolutionPair_JVProbl(
    const emlrtStack *sp, const emxArray_real_T *costMatrix,
    const real_T costOfNonAssignment_f1_data[],
    int32_T costOfNonAssignment_f1_size,
    const emxArray_real_T *costOfNonAssignment_f2,
    emxArray_real_T *obj_PaddedCostMatrix, emxArray_real_T *obj_RowSoln,
    emxArray_real_T *obj_ColSoln, emxArray_boolean_T *obj_IsEnforced,
    int32_T obj_CostSize[2], emxArray_boolean_T *obj_IsDummySolution,
    emxArray_real_T *obj_ColReduction, emxArray_real_T *obj_RowReduction,
    boolean_T *obj_IsSolved, real_T *obj_LowerBound)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  const real_T *costMatrix_data;
  const real_T *costOfNonAssignment_f2_data;
  real_T obj_BestSolutionCost;
  real_T *obj_ColSoln_data;
  real_T *obj_PaddedCostMatrix_data;
  int32_T b_i;
  int32_T b_unnamed_idx_0;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T n;
  int32_T nCol;
  int32_T nRow;
  int32_T nx;
  int32_T unnamed_idx_0;
  uint32_T problemSize;
  boolean_T *obj_IsDummySolution_data;
  boolean_T *obj_IsEnforced_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  costOfNonAssignment_f2_data = costOfNonAssignment_f2->data;
  costMatrix_data = costMatrix->data;
  st.site = &oeb_emlrtRSI;
  b_st.site = &peb_emlrtRSI;
  nRow = costMatrix->size[0];
  nCol = costMatrix->size[1];
  c_st.site = &qeb_emlrtRSI;
  d_st.site = &teb_emlrtRSI;
  if (costMatrix->size[0] >
      muIntScalarMax_sint32(costOfNonAssignment_f1_size,
                            costOfNonAssignment_f1_size)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &o_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (costMatrix->size[0] != costOfNonAssignment_f1_size) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &p_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  d_st.site = &ueb_emlrtRSI;
  nx = costOfNonAssignment_f2->size[1];
  n = 1;
  if (costOfNonAssignment_f2->size[1] > 1) {
    n = costOfNonAssignment_f2->size[1];
  }
  if (costMatrix->size[1] > muIntScalarMax_sint32(nx, n)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &o_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (costMatrix->size[1] != costOfNonAssignment_f2->size[1]) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &p_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  unnamed_idx_0 = costMatrix->size[0] + costMatrix->size[1];
  nx = obj_PaddedCostMatrix->size[0] * obj_PaddedCostMatrix->size[1];
  obj_PaddedCostMatrix->size[0] = unnamed_idx_0;
  obj_PaddedCostMatrix->size[1] = unnamed_idx_0;
  emxEnsureCapacity_real_T(&b_st, obj_PaddedCostMatrix, nx, &ug_emlrtRTEI);
  obj_PaddedCostMatrix_data = obj_PaddedCostMatrix->data;
  nx = unnamed_idx_0 * unnamed_idx_0;
  for (i = 0; i < nx; i++) {
    obj_PaddedCostMatrix_data[i] = 0.0;
  }
  for (i = 0; i < nCol; i++) {
    for (b_i = 0; b_i < nRow; b_i++) {
      obj_PaddedCostMatrix_data[b_i + obj_PaddedCostMatrix->size[0] * i] =
          costMatrix_data[b_i + costMatrix->size[0] * i];
    }
  }
  if (costMatrix->size[0] + 1 > unnamed_idx_0) {
    i1 = 0;
    nx = 0;
  } else {
    i1 = costMatrix->size[0];
    nx = unnamed_idx_0;
  }
  if (costMatrix->size[1] + 1 > unnamed_idx_0) {
    i2 = 0;
    n = 0;
  } else {
    i2 = costMatrix->size[1];
    n = unnamed_idx_0;
  }
  b_unnamed_idx_0 = nx - i1;
  nx = n - i2;
  for (i = 0; i < nx; i++) {
    for (b_i = 0; b_i < b_unnamed_idx_0; b_i++) {
      obj_PaddedCostMatrix_data[(i1 + b_i) +
                                obj_PaddedCostMatrix->size[0] * (i2 + i)] = 0.0;
    }
  }
  c_st.site = &reb_emlrtRSI;
  for (b_i = 0; b_i < nRow; b_i++) {
    if (nCol + 1 > obj_PaddedCostMatrix->size[1]) {
      n = 0;
      nx = 0;
    } else {
      if ((nCol + 1 < 1) || (nCol + 1 > obj_PaddedCostMatrix->size[1])) {
        emlrtDynamicBoundsCheckR2012b(
            nCol + 1, 1, obj_PaddedCostMatrix->size[1], &qe_emlrtBCI, &b_st);
      }
      n = nCol;
      if (obj_PaddedCostMatrix->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(obj_PaddedCostMatrix->size[1], 1,
                                      obj_PaddedCostMatrix->size[1],
                                      &pe_emlrtBCI, &b_st);
      }
      nx = obj_PaddedCostMatrix->size[1];
    }
    if (b_i + 1 > obj_PaddedCostMatrix->size[0]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, obj_PaddedCostMatrix->size[0],
                                    &re_emlrtBCI, &b_st);
    }
    nx -= n;
    for (i = 0; i < nx; i++) {
      obj_PaddedCostMatrix_data[b_i + obj_PaddedCostMatrix->size[0] * (n + i)] =
          rtInf;
    }
    if (b_i + 1 > nRow) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, nRow, &se_emlrtBCI, &b_st);
    }
    if (b_i + 1 > obj_PaddedCostMatrix->size[0]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, obj_PaddedCostMatrix->size[0],
                                    &te_emlrtBCI, &b_st);
    }
    nx = (nCol + b_i) + 1;
    if ((nx < 1) || (nx > obj_PaddedCostMatrix->size[1])) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, obj_PaddedCostMatrix->size[1],
                                    &ue_emlrtBCI, &b_st);
    }
    obj_PaddedCostMatrix_data[b_i + obj_PaddedCostMatrix->size[0] * (nx - 1)] =
        costOfNonAssignment_f1_data[b_i];
  }
  c_st.site = &seb_emlrtRSI;
  if (costMatrix->size[1] > 2147483646) {
    d_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }
  for (b_i = 0; b_i < nCol; b_i++) {
    if (nRow + 1 > obj_PaddedCostMatrix->size[0]) {
      n = 1;
      nx = 0;
    } else {
      if ((nRow + 1 < 1) || (nRow + 1 > obj_PaddedCostMatrix->size[0])) {
        emlrtDynamicBoundsCheckR2012b(
            nRow + 1, 1, obj_PaddedCostMatrix->size[0], &oe_emlrtBCI, &b_st);
      }
      n = nRow + 1;
      if (obj_PaddedCostMatrix->size[0] < 1) {
        emlrtDynamicBoundsCheckR2012b(obj_PaddedCostMatrix->size[0], 1,
                                      obj_PaddedCostMatrix->size[0],
                                      &ne_emlrtBCI, &b_st);
      }
      nx = obj_PaddedCostMatrix->size[0];
    }
    if (b_i + 1 > obj_PaddedCostMatrix->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, obj_PaddedCostMatrix->size[1],
                                    &me_emlrtBCI, &b_st);
    }
    nx -= n;
    for (i = 0; i <= nx; i++) {
      obj_PaddedCostMatrix_data[((n + i) +
                                 obj_PaddedCostMatrix->size[0] * b_i) -
                                1] = rtInf;
    }
    if (b_i + 1 > nCol) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, nCol, &ve_emlrtBCI, &b_st);
    }
    nx = (nRow + b_i) + 1;
    if ((nx < 1) || (nx > obj_PaddedCostMatrix->size[0])) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, obj_PaddedCostMatrix->size[0],
                                    &we_emlrtBCI, &b_st);
    }
    if (b_i + 1 > obj_PaddedCostMatrix->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, obj_PaddedCostMatrix->size[1],
                                    &xe_emlrtBCI, &b_st);
    }
    obj_PaddedCostMatrix_data[(nx + obj_PaddedCostMatrix->size[0] * b_i) - 1] =
        costOfNonAssignment_f2_data[b_i];
  }
  obj_CostSize[0] = costMatrix->size[0];
  obj_CostSize[1] = costMatrix->size[1];
  nx = obj_RowSoln->size[0];
  obj_RowSoln->size[0] = unnamed_idx_0;
  emxEnsureCapacity_real_T(&st, obj_RowSoln, nx, &vg_emlrtRTEI);
  obj_PaddedCostMatrix_data = obj_RowSoln->data;
  nx = obj_ColSoln->size[0] * obj_ColSoln->size[1];
  obj_ColSoln->size[0] = 1;
  obj_ColSoln->size[1] = unnamed_idx_0;
  emxEnsureCapacity_real_T(&st, obj_ColSoln, nx, &vg_emlrtRTEI);
  obj_ColSoln_data = obj_ColSoln->data;
  nx = obj_IsEnforced->size[0];
  obj_IsEnforced->size[0] = unnamed_idx_0;
  emxEnsureCapacity_boolean_T(&st, obj_IsEnforced, nx, &vg_emlrtRTEI);
  obj_IsEnforced_data = obj_IsEnforced->data;
  nx = obj_IsDummySolution->size[0];
  obj_IsDummySolution->size[0] = unnamed_idx_0;
  emxEnsureCapacity_boolean_T(&st, obj_IsDummySolution, nx, &vg_emlrtRTEI);
  obj_IsDummySolution_data = obj_IsDummySolution->data;
  for (i = 0; i < unnamed_idx_0; i++) {
    obj_PaddedCostMatrix_data[i] = rtNaN;
    obj_ColSoln_data[i] = rtNaN;
    obj_IsEnforced_data[i] = false;
    obj_IsDummySolution_data[i] = false;
  }
  problemSize = (uint32_T)costMatrix->size[0] + (uint32_T)costMatrix->size[1];
  if ((real_T)problemSize != (int32_T)problemSize) {
    emlrtIntegerCheckR2012b(problemSize, &e_emlrtDCI, (emlrtConstCTX)sp);
  }
  nx = obj_ColReduction->size[0] * obj_ColReduction->size[1];
  obj_ColReduction->size[0] = 1;
  obj_ColReduction->size[1] = (int32_T)problemSize;
  emxEnsureCapacity_real_T(sp, obj_ColReduction, nx, &wg_emlrtRTEI);
  obj_PaddedCostMatrix_data = obj_ColReduction->data;
  n = (int32_T)problemSize;
  for (i = 0; i < n; i++) {
    obj_PaddedCostMatrix_data[i] = 0.0;
  }
  if ((real_T)problemSize != (int32_T)problemSize) {
    emlrtIntegerCheckR2012b(problemSize, &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  nx = obj_RowReduction->size[0];
  obj_RowReduction->size[0] = (int32_T)problemSize;
  emxEnsureCapacity_real_T(sp, obj_RowReduction, nx, &xg_emlrtRTEI);
  obj_PaddedCostMatrix_data = obj_RowReduction->data;
  if ((real_T)problemSize != (int32_T)problemSize) {
    emlrtIntegerCheckR2012b(problemSize, &f_emlrtDCI, (emlrtConstCTX)sp);
  }
  for (i = 0; i < n; i++) {
    obj_PaddedCostMatrix_data[i] = 0.0;
  }
  obj_BestSolutionCost = rtInf;
  *obj_IsSolved = false;
  *obj_LowerBound = rtInf;
  return obj_BestSolutionCost;
}

/* End of code generation (JVProblemSolutionPair.c) */
