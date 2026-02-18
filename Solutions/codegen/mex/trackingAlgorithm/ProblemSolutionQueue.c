/*
 * ProblemSolutionQueue.c
 *
 * Code generation for function 'ProblemSolutionQueue'
 *
 */

/* Include files */
#include "ProblemSolutionQueue.h"
#include "AbstractProblemSolutionPair.h"
#include "all.h"
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "kbestRemoveUnassigned.h"
#include "lapDijkstra.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo cgb_emlrtRSI = {
    26,                                          /* lineNo */
    "ProblemSolutionQueue/ProblemSolutionQueue", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo dgb_emlrtRSI = {
    158,      /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRSInfo egb_emlrtRSI = {
    45,                                         /* lineNo */
    "ProblemSolutionQueue/extractBestSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo fgb_emlrtRSI = {
    48,                                         /* lineNo */
    "ProblemSolutionQueue/extractBestSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo ggb_emlrtRSI = {
    50,                                         /* lineNo */
    "ProblemSolutionQueue/extractBestSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo hgb_emlrtRSI = {
    52,                                         /* lineNo */
    "ProblemSolutionQueue/extractBestSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo igb_emlrtRSI = {
    67,                                         /* lineNo */
    "ProblemSolutionQueue/extractBestSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo jgb_emlrtRSI = {
    68,                                         /* lineNo */
    "ProblemSolutionQueue/extractBestSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo kgb_emlrtRSI = {
    120,                                 /* lineNo */
    "AbstractProblemSolutionPair/solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo lgb_emlrtRSI = {
    121,                                 /* lineNo */
    "AbstractProblemSolutionPair/solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo mgb_emlrtRSI = {
    125,                                 /* lineNo */
    "AbstractProblemSolutionPair/solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo ngb_emlrtRSI = {
    127,                                 /* lineNo */
    "AbstractProblemSolutionPair/solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo ogb_emlrtRSI = {
    131,                                 /* lineNo */
    "AbstractProblemSolutionPair/solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo pgb_emlrtRSI = {
    133,                                 /* lineNo */
    "AbstractProblemSolutionPair/solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo qgb_emlrtRSI = {
    165,                                      /* lineNo */
    "AbstractProblemSolutionPair/isFeasible", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo sgb_emlrtRSI = {
    193,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\combin"
    "eVectorElements.m" /* pathName */
};

static emlrtRSInfo tgb_emlrtRSI = {
    211,                /* lineNo */
    "colMajorFlatIter", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\combin"
    "eVectorElements.m" /* pathName */
};

static emlrtRSInfo ugb_emlrtRSI = {
    49,                                   /* lineNo */
    "JVProblemSolutionPair/solveProblem", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo vgb_emlrtRSI = {
    50,                                   /* lineNo */
    "JVProblemSolutionPair/solveProblem", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo uhb_emlrtRSI = {
    290,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo vhb_emlrtRSI = {
    383,                     /* lineNo */
    "unaryMinOrMaxDispatch", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo whb_emlrtRSI = {
    457,          /* lineNo */
    "minOrMax2D", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo xhb_emlrtRSI = {
    562,                         /* lineNo */
    "minOrMax2DColumnMajorDim2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo yhb_emlrtRSI = {
    561,                         /* lineNo */
    "minOrMax2DColumnMajorDim2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo aib_emlrtRSI = {
    558,                         /* lineNo */
    "minOrMax2DColumnMajorDim2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo eib_emlrtRSI = {
    111,                                   /* lineNo */
    "AbstractProblemSolutionPair/isValid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtRSInfo fib_emlrtRSI = {
    139,                              /* lineNo */
    "ProblemSolutionQueue/sortQueue", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo gib_emlrtRSI = {
    143,                              /* lineNo */
    "ProblemSolutionQueue/sortQueue", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo hib_emlrtRSI = {
    148,                              /* lineNo */
    "ProblemSolutionQueue/sortQueue", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo iib_emlrtRSI = {
    76,                                      /* lineNo */
    "ProblemSolutionQueue/removeTopProblem", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo djb_emlrtRSI = {
    161,                                   /* lineNo */
    "ProblemSolutionQueue/formatSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo ejb_emlrtRSI = {
    163,                                   /* lineNo */
    "ProblemSolutionQueue/formatSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pathName */
};

static emlrtRSInfo fjb_emlrtRSI = {
    103,                                          /* lineNo */
    "AbstractProblemSolutionPair/formatSolution", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pathName */
};

static emlrtBCInfo rf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    163,                                   /* lineNo */
    30,                                    /* colNo */
    "",                                    /* aName */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo sf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    162,                                   /* lineNo */
    46,                                    /* colNo */
    "",                                    /* aName */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtRTEInfo cc_emlrtRTEI = {
    156,                                   /* lineNo */
    13,                                    /* colNo */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtDCInfo i_emlrtDCI = {
    157,                                   /* lineNo */
    32,                                    /* colNo */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    4                                                /* checkKind */
};

static emlrtBCInfo tf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    157,                                   /* lineNo */
    34,                                    /* colNo */
    "",                                    /* aName */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo uf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    163,                                   /* lineNo */
    68,                                    /* colNo */
    "",                                    /* aName */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo vf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    163,                                   /* lineNo */
    77,                                    /* colNo */
    "",                                    /* aName */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo wf_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    163,                                   /* lineNo */
    34,                                    /* colNo */
    "",                                    /* aName */
    "ProblemSolutionQueue/formatSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo ah_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    37,                                         /* lineNo */
    45,                                         /* colNo */
    "",                                         /* aName */
    "ProblemSolutionQueue/extractBestSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo bh_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    49,                                         /* lineNo */
    40,                                         /* colNo */
    "",                                         /* aName */
    "ProblemSolutionQueue/extractBestSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo ch_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    62,                                         /* lineNo */
    49,                                         /* colNo */
    "",                                         /* aName */
    "ProblemSolutionQueue/extractBestSolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtECInfo gb_emlrtECI = {
    1,                                   /* nDims */
    130,                                 /* lineNo */
    39,                                  /* colNo */
    "AbstractProblemSolutionPair/solve", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo gc_emlrtRTEI = {
    53,       /* lineNo */
    15,       /* colNo */
    "bsxfun", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\bsxfun.m" /* pName
                                                                         */
};

static emlrtBCInfo dh_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    77,                                      /* lineNo */
    60,                                      /* colNo */
    "",                                      /* aName */
    "ProblemSolutionQueue/removeTopProblem", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo eh_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    77,                                      /* lineNo */
    36,                                      /* colNo */
    "",                                      /* aName */
    "ProblemSolutionQueue/removeTopProblem", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo fh_emlrtBCI = {
    -1,                                  /* iFirst */
    -1,                                  /* iLast */
    133,                                 /* lineNo */
    49,                                  /* colNo */
    "",                                  /* aName */
    "AbstractProblemSolutionPair/solve", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m", /* pName */
    0                                                       /* checkKind */
};

static emlrtRTEInfo mc_emlrtRTEI = {
    137,                              /* lineNo */
    17,                               /* colNo */
    "ProblemSolutionQueue/sortQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtBCInfo aj_emlrtBCI = {
    -1,                               /* iFirst */
    -1,                               /* iLast */
    140,                              /* lineNo */
    71,                               /* colNo */
    "",                               /* aName */
    "ProblemSolutionQueue/sortQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = {
    138,                              /* lineNo */
    17,                               /* colNo */
    "ProblemSolutionQueue/sortQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    4                                                /* checkKind */
};

static emlrtBCInfo bj_emlrtBCI = {
    -1,                               /* iFirst */
    -1,                               /* iLast */
    140,                              /* lineNo */
    33,                               /* colNo */
    "",                               /* aName */
    "ProblemSolutionQueue/sortQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo cj_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    177,                           /* lineNo */
    20,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo dj_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    178,                           /* lineNo */
    28,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo ej_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    178,                           /* lineNo */
    38,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo fj_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    178,                           /* lineNo */
    14,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo gj_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    179,                           /* lineNo */
    14,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo hj_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    179,                           /* lineNo */
    24,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo ij_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    180,                           /* lineNo */
    43,                            /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtBCInfo jj_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    180,                           /* lineNo */
    5,                             /* colNo */
    "",                            /* aName */
    "cellArrayIdxMemoryEfficient", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m", /* pName */
    0                                                /* checkKind */
};

static emlrtRTEInfo yg_emlrtRTEI = {
    106,      /* lineNo */
    25,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRTEInfo ah_emlrtRTEI = {
    26,                     /* lineNo */
    13,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo vh_emlrtRTEI = {
    152,                    /* lineNo */
    72,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo wh_emlrtRTEI = {
    160,                    /* lineNo */
    20,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo xh_emlrtRTEI = {
    157,                    /* lineNo */
    34,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo yh_emlrtRTEI = {
    103,                           /* lineNo */
    14,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo ci_emlrtRTEI = {
    163,                    /* lineNo */
    34,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo di_emlrtRTEI = {
    103,                           /* lineNo */
    117,                           /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo ui_emlrtRTEI = {
    37,                     /* lineNo */
    13,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo vi_emlrtRTEI = {
    14,         /* lineNo */
    5,          /* colNo */
    "isfinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\isfinite.m" /* pName
                                                                           */
};

static emlrtRTEInfo wi_emlrtRTEI = {
    164,                           /* lineNo */
    13,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo xi_emlrtRTEI = {
    165,                           /* lineNo */
    22,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo yi_emlrtRTEI = {
    68,                     /* lineNo */
    17,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo aj_emlrtRTEI = {
    60,       /* lineNo */
    20,       /* colNo */
    "bsxfun", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\bsxfun.m" /* pName
                                                                         */
};

static emlrtRTEInfo bj_emlrtRTEI = {
    130,                           /* lineNo */
    39,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo cj_emlrtRTEI = {
    130,                           /* lineNo */
    61,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo dj_emlrtRTEI = {
    45,                     /* lineNo */
    17,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo ej_emlrtRTEI = {
    165,                           /* lineNo */
    56,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo fj_emlrtRTEI = {
    49,                     /* lineNo */
    21,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo gj_emlrtRTEI = {
    550,             /* lineNo */
    21,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtRTEInfo hj_emlrtRTEI = {
    62,                     /* lineNo */
    17,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo ij_emlrtRTEI = {
    52,                     /* lineNo */
    21,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo jj_emlrtRTEI = {
    133,                           /* lineNo */
    44,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo kj_emlrtRTEI = {
    127,                           /* lineNo */
    17,                            /* colNo */
    "AbstractProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\AbstractProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo lj_emlrtRTEI = {
    50,                      /* lineNo */
    36,                      /* colNo */
    "JVProblemSolutionPair", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\JVProblemSolutionPair.m" /* pName */
};

static emlrtRTEInfo sk_emlrtRTEI = {
    138,                    /* lineNo */
    17,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo tk_emlrtRTEI = {
    177,                    /* lineNo */
    5,                      /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo uk_emlrtRTEI = {
    148,                    /* lineNo */
    17,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo vk_emlrtRTEI = {
    180,                    /* lineNo */
    15,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo wk_emlrtRTEI = {
    134,                    /* lineNo */
    17,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

static emlrtRTEInfo xk_emlrtRTEI = {
    127,                    /* lineNo */
    24,                     /* colNo */
    "ProblemSolutionQueue", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\+assignment\\ProblemSolutionQueue.m" /* pName */
};

/* Function Definitions */
void ProblemSolutionQueue_sortQueue(const emlrtStack *sp,
                                    c_fusion_internal_assignment_Pr *obj)
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  c_fusion_internal_assignment_JV tmp;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_int32_T *iidx;
  emxArray_int32_T *r;
  emxArray_real_T *costsToSort;
  real_T b_costsToSort;
  real_T *costsToSort_data;
  int32_T ProblemSolutionQueue_sortQueue_numThreads;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T c_i;
  int32_T d_i;
  int32_T end;
  int32_T i;
  int32_T loop_ub;
  int32_T partialTrueCount;
  int32_T *iidx_data;
  boolean_T emlrtHadParallelError = false;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (obj->NumProblems > obj->MaxNumSubProblems) {
    emlrtErrorWithMessageIdR2018a(sp, &mc_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  emxInit_real_T(sp, &costsToSort, 1, &wk_emlrtRTEI, true);
  loop_ub = obj->NumProblems;
  if (loop_ub < 0) {
    emlrtNonNegativeCheckR2012b(loop_ub, &j_emlrtDCI, (emlrtConstCTX)sp);
  }
  partialTrueCount = costsToSort->size[0];
  costsToSort->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, costsToSort, partialTrueCount, &sk_emlrtRTEI);
  costsToSort_data = costsToSort->data;
  for (i = 0; i < loop_ub; i++) {
    costsToSort_data[i] = 0.0;
  }
  st.site = &fib_emlrtRSI;
  if (obj->NumProblems > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  if (loop_ub < 800) {
    for (b_i = 0; b_i < loop_ub; b_i++) {
      partialTrueCount = obj->AllProblemList->size[0] - 1;
      if (b_i > partialTrueCount) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, partialTrueCount, &aj_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (b_i + 1 > costsToSort->size[0]) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, costsToSort->size[0],
                                      &bj_emlrtBCI, (emlrtConstCTX)sp);
      }
      costsToSort_data[b_i] =
          muDoubleScalarMax(obj->AllProblemList->data[b_i].BestSolutionCost,
                            obj->AllProblemList->data[b_i].LowerBound);
    }
  } else {
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    ProblemSolutionQueue_sortQueue_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(                                              \
        ProblemSolutionQueue_sortQueue_numThreads) private(c_st,               \
                                                               emlrtJBEnviron, \
                                                               c_i)            \
    firstprivate(emlrtHadParallelError)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        c_st.prev = sp;
        c_st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
        c_st.site = NULL;
        emlrtSetJmpBuf(&c_st, &emlrtJBEnviron);
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (b_i = 0; b_i < loop_ub; b_i++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          c_i = obj->AllProblemList->size[0] - 1;
          if (b_i > c_i) {
            emlrtDynamicBoundsCheckR2012b(b_i, 0, c_i, &aj_emlrtBCI, &c_st);
          }
          if (b_i + 1 > costsToSort->size[0]) {
            emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, costsToSort->size[0],
                                          &bj_emlrtBCI, &c_st);
          }
          costsToSort_data[b_i] =
              muDoubleScalarMax(obj->AllProblemList->data[b_i].BestSolutionCost,
                                obj->AllProblemList->data[b_i].LowerBound);
        } else {
          emlrtHadParallelError = true;
        }
      }
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
  }
  st.site = &gib_emlrtRSI;
  emxInit_int32_T(&st, &iidx, 1, &xk_emlrtRTEI);
  b_st.site = &yn_emlrtRSI;
  sort(&b_st, costsToSort, iidx);
  iidx_data = iidx->data;
  b_loop_ub = iidx->size[0];
  partialTrueCount = costsToSort->size[0];
  costsToSort->size[0] = iidx->size[0];
  emxEnsureCapacity_real_T(&st, costsToSort, partialTrueCount, &mf_emlrtRTEI);
  costsToSort_data = costsToSort->data;
  for (i = 0; i < b_loop_ub; i++) {
    costsToSort_data[i] = iidx_data[i];
  }
  emxFree_int32_T(&st, &iidx);
  st.site = &hib_emlrtRSI;
  c_emxInitStruct_fusion_internal(&st, &tmp, &tk_emlrtRTEI, true);
  emxInit_int32_T(&st, &r, 1, &vk_emlrtRTEI);
  for (d_i = 0; d_i < b_loop_ub; d_i++) {
    partialTrueCount = obj->AllProblemList->size[0] - 1;
    if (d_i > partialTrueCount) {
      emlrtDynamicBoundsCheckR2012b(d_i, 0, partialTrueCount, &cj_emlrtBCI,
                                    &st);
    }
    c_emxCopyStruct_fusion_internal(&st, &tmp, &obj->AllProblemList->data[d_i],
                                    &tk_emlrtRTEI);
    end = costsToSort->size[0];
    if (d_i + 1 > costsToSort->size[0]) {
      emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, costsToSort->size[0],
                                    &ej_emlrtBCI, &st);
    }
    b_costsToSort = costsToSort_data[d_i];
    if (((int32_T)b_costsToSort - 1 < 0) ||
        ((int32_T)b_costsToSort - 1 > partialTrueCount)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)b_costsToSort - 1, 0,
                                    partialTrueCount, &dj_emlrtBCI, &st);
    }
    if (d_i > partialTrueCount) {
      emlrtDynamicBoundsCheckR2012b(d_i, 0, partialTrueCount, &fj_emlrtBCI,
                                    &st);
    }
    c_emxCopyStruct_fusion_internal(
        &st, &obj->AllProblemList->data[d_i],
        &obj->AllProblemList->data[(int32_T)b_costsToSort - 1], &uk_emlrtRTEI);
    partialTrueCount = obj->AllProblemList->size[0] - 1;
    if (d_i + 1 > costsToSort->size[0]) {
      emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, costsToSort->size[0],
                                    &hj_emlrtBCI, &st);
    }
    if (((int32_T)b_costsToSort - 1 < 0) ||
        ((int32_T)b_costsToSort - 1 > partialTrueCount)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)b_costsToSort - 1, 0,
                                    partialTrueCount, &gj_emlrtBCI, &st);
    }
    c_emxCopyStruct_fusion_internal(
        &st, &obj->AllProblemList->data[(int32_T)b_costsToSort - 1], &tmp,
        &uk_emlrtRTEI);
    loop_ub = 0;
    for (i = 0; i < end; i++) {
      if ((int32_T)costsToSort_data[i] == d_i + 1) {
        loop_ub++;
      }
    }
    partialTrueCount = r->size[0];
    r->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(&st, r, partialTrueCount, &vk_emlrtRTEI);
    iidx_data = r->data;
    partialTrueCount = 0;
    for (i = 0; i < end; i++) {
      if ((int32_T)costsToSort_data[i] == d_i + 1) {
        iidx_data[partialTrueCount] = i;
        partialTrueCount++;
      }
    }
    if (d_i + 1 > costsToSort->size[0]) {
      emlrtDynamicBoundsCheckR2012b(d_i + 1, 1, costsToSort->size[0],
                                    &ij_emlrtBCI, &st);
    }
    partialTrueCount = r->size[0];
    for (i = 0; i < partialTrueCount; i++) {
      if (iidx_data[i] > costsToSort->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(iidx_data[i], 0, costsToSort->size[0] - 1,
                                      &jj_emlrtBCI, &st);
      }
      costsToSort_data[iidx_data[i]] = b_costsToSort;
    }
  }
  emxFree_int32_T(&st, &r);
  c_emxFreeStruct_fusion_internal(&st, &tmp);
  emxFree_real_T(&st, &costsToSort);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

int32_T c_ProblemSolutionQueue_ProblemS(
    const emlrtStack *sp, const c_fusion_internal_assignment_JV *problem,
    int32_T maxNumSubProblems,
    c_emxArray_fusion_internal_assi *obj_AllProblemList,
    int32_T *obj_MaxNumSubProblems)
{
  c_fusion_internal_assignment_JV *obj_AllProblemList_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T itilerow;
  int32_T obj_NumProblems;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &cgb_emlrtRSI;
  b_st.site = &gq_emlrtRSI;
  if (maxNumSubProblems < 0) {
    emlrtNonNegativeCheckR2012b(maxNumSubProblems, &g_emlrtDCI, &st);
  }
  obj_NumProblems = obj_AllProblemList->size[0];
  obj_AllProblemList->size[0] = maxNumSubProblems;
  c_emxEnsureCapacity_fusion_inte(&st, obj_AllProblemList, obj_NumProblems,
                                  &yg_emlrtRTEI);
  obj_AllProblemList_data = obj_AllProblemList->data;
  if (maxNumSubProblems != 0) {
    b_st.site = &dgb_emlrtRSI;
    if (maxNumSubProblems > 2147483646) {
      c_st.site = &k_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (itilerow = 0; itilerow < maxNumSubProblems; itilerow++) {
      c_emxCopyStruct_fusion_internal(&st, &obj_AllProblemList_data[itilerow],
                                      problem, &ah_emlrtRTEI);
    }
  }
  obj_NumProblems = 0;
  *obj_MaxNumSubProblems = maxNumSubProblems;
  return obj_NumProblems;
}

boolean_T
c_ProblemSolutionQueue_extractB(const emlrtStack *sp,
                                c_fusion_internal_assignment_Pr *obj,
                                c_fusion_internal_assignment_JV *topProblem)
{
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
  emxArray_boolean_T *c_nz;
  emxArray_boolean_T *d_nz;
  emxArray_boolean_T *isFiniteMatrix;
  emxArray_boolean_T *r;
  emxArray_int32_T *b_nz;
  emxArray_int32_T *nz;
  emxArray_real_T *assignment;
  emxArray_real_T *b_topProblem;
  emxArray_real_T *varargin_1;
  real_T *assignment_data;
  real_T *topProblem_data;
  int32_T b_j;
  int32_T csz_idx_1;
  int32_T j;
  int32_T m;
  int32_T *nz_data;
  boolean_T isValidProblem;
  boolean_T *b_nz_data;
  boolean_T *isFiniteMatrix_data;
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  m = obj->AllProblemList->size[0] - 1;
  if (m < 0) {
    emlrtDynamicBoundsCheckR2012b(0, 0, m, &ah_emlrtBCI, (emlrtConstCTX)sp);
  }
  c_emxCopyStruct_fusion_internal(sp, topProblem, &obj->AllProblemList->data[0],
                                  &ui_emlrtRTEI);
  emxInit_real_T(sp, &assignment, 2, &kj_emlrtRTEI, true);
  emxInit_boolean_T(sp, &r, 1, &bj_emlrtRTEI, true);
  emxInit_boolean_T(sp, &isFiniteMatrix, 2, &wi_emlrtRTEI, true);
  emxInit_int32_T(sp, &nz, 2, &ag_emlrtRTEI);
  emxInit_int32_T(sp, &b_nz, 1, &ag_emlrtRTEI);
  emxInit_real_T(sp, &varargin_1, 2, &lj_emlrtRTEI, true);
  emxInit_boolean_T(sp, &c_nz, 2, &xi_emlrtRTEI, true);
  emxInit_boolean_T(sp, &d_nz, 1, &ej_emlrtRTEI, true);
  emxInit_real_T(sp, &b_topProblem, 1, &jj_emlrtRTEI, true);
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (!topProblem->IsSolved) {
      real_T b;
      int32_T loop_ub;
      int32_T n;
      int32_T xoffset;
      st.site = &egb_emlrtRSI;
      b_st.site = &kgb_emlrtRSI;
      loop_ub = topProblem->PaddedCostMatrix->size[0];
      xoffset = isFiniteMatrix->size[0] * isFiniteMatrix->size[1];
      isFiniteMatrix->size[0] = topProblem->PaddedCostMatrix->size[0];
      n = topProblem->PaddedCostMatrix->size[1];
      isFiniteMatrix->size[1] = topProblem->PaddedCostMatrix->size[1];
      emxEnsureCapacity_boolean_T(&b_st, isFiniteMatrix, xoffset,
                                  &wi_emlrtRTEI);
      isFiniteMatrix_data = isFiniteMatrix->data;
      xoffset = topProblem->PaddedCostMatrix->size[0] *
                topProblem->PaddedCostMatrix->size[1];
      for (j = 0; j < xoffset; j++) {
        isFiniteMatrix_data[j] =
            ((!muDoubleScalarIsInf(topProblem->PaddedCostMatrix->data[j])) &&
             (!muDoubleScalarIsNaN(topProblem->PaddedCostMatrix->data[j])));
      }
      c_st.site = &qgb_emlrtRSI;
      d_st.site = &edb_emlrtRSI;
      e_st.site = &mw_emlrtRSI;
      b_combineVectorElements(&e_st, isFiniteMatrix, nz);
      nz_data = nz->data;
      m = c_nz->size[0] * c_nz->size[1];
      c_nz->size[0] = 1;
      xoffset = nz->size[1];
      c_nz->size[1] = nz->size[1];
      emxEnsureCapacity_boolean_T(&b_st, c_nz, m, &xi_emlrtRTEI);
      b_nz_data = c_nz->data;
      for (j = 0; j < xoffset; j++) {
        b_nz_data[j] = (nz_data[j] > 0);
      }
      c_st.site = &qgb_emlrtRSI;
      if (all(&c_st, c_nz)) {
        c_st.site = &qgb_emlrtRSI;
        d_st.site = &edb_emlrtRSI;
        e_st.site = &mw_emlrtRSI;
        f_st.site = &fdb_emlrtRSI;
        xoffset = b_nz->size[0];
        b_nz->size[0] = topProblem->PaddedCostMatrix->size[0];
        emxEnsureCapacity_int32_T(&f_st, b_nz, xoffset, &bh_emlrtRTEI);
        nz_data = b_nz->data;
        g_st.site = &sgb_emlrtRSI;
        if (isFiniteMatrix->size[0] > 2147483646) {
          h_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }
        for (j = 0; j < loop_ub; j++) {
          nz_data[j] = isFiniteMatrix_data[j];
        }
        g_st.site = &gdb_emlrtRSI;
        if (isFiniteMatrix->size[1] > 2147483646) {
          h_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }
        for (j = 2; j <= n; j++) {
          xoffset = (j - 1) * loop_ub;
          g_st.site = &tgb_emlrtRSI;
          for (b_j = 0; b_j < loop_ub; b_j++) {
            nz_data[b_j] += isFiniteMatrix_data[xoffset + b_j];
          }
        }
        m = d_nz->size[0];
        d_nz->size[0] = topProblem->PaddedCostMatrix->size[0];
        emxEnsureCapacity_boolean_T(&b_st, d_nz, m, &ej_emlrtRTEI);
        isFiniteMatrix_data = d_nz->data;
        for (j = 0; j < loop_ub; j++) {
          isFiniteMatrix_data[j] = (nz_data[j] > 0);
        }
        c_st.site = &qgb_emlrtRSI;
        if (b_all(&c_st, d_nz)) {
          isValidProblem = true;
        } else {
          isValidProblem = false;
        }
      } else {
        isValidProblem = false;
      }
      if (isValidProblem) {
        b_st.site = &lgb_emlrtRSI;
        c_st.site = &ugb_emlrtRSI;
        lapDijkstra(&c_st, topProblem->PaddedCostMatrix, topProblem->RowSoln,
                    topProblem->ColSoln, topProblem->ColReduction);
        c_st.site = &vgb_emlrtRSI;
        d_st.site = &vgb_emlrtRSI;
        if (topProblem->PaddedCostMatrix->size[1] ==
            topProblem->ColReduction->size[1]) {
          csz_idx_1 = topProblem->PaddedCostMatrix->size[1];
        } else {
          emlrtErrorWithMessageIdR2018a(
              &d_st, &gc_emlrtRTEI, "MATLAB:bsxfun:arrayDimensionsMustMatch",
              "MATLAB:bsxfun:arrayDimensionsMustMatch", 0);
        }
        m = varargin_1->size[0] * varargin_1->size[1];
        varargin_1->size[0] = topProblem->PaddedCostMatrix->size[0];
        varargin_1->size[1] = csz_idx_1;
        emxEnsureCapacity_real_T(&d_st, varargin_1, m, &aj_emlrtRTEI);
        assignment_data = varargin_1->data;
        for (j = 0; j < csz_idx_1; j++) {
          m = varargin_1->size[0];
          xoffset = (varargin_1->size[0] / 2) << 1;
          n = xoffset - 2;
          for (b_j = 0; b_j <= n; b_j += 2) {
            __m128d r1;
            r1 = _mm_loadu_pd(
                &topProblem->PaddedCostMatrix
                     ->data[b_j + topProblem->PaddedCostMatrix->size[0] * j]);
            _mm_storeu_pd(
                &assignment_data[b_j + varargin_1->size[0] * j],
                _mm_sub_pd(r1, _mm_set1_pd(topProblem->ColReduction->data[j])));
          }
          for (b_j = xoffset; b_j < m; b_j++) {
            assignment_data[b_j + varargin_1->size[0] * j] =
                topProblem->PaddedCostMatrix
                    ->data[b_j + topProblem->PaddedCostMatrix->size[0] * j] -
                topProblem->ColReduction->data[j];
          }
        }
        d_st.site = &phb_emlrtRSI;
        e_st.site = &qhb_emlrtRSI;
        f_st.site = &rhb_emlrtRSI;
        g_st.site = &uhb_emlrtRSI;
        h_st.site = &vhb_emlrtRSI;
        i_st.site = &whb_emlrtRSI;
        m = varargin_1->size[0];
        n = varargin_1->size[1];
        xoffset = topProblem->RowReduction->size[0];
        topProblem->RowReduction->size[0] = varargin_1->size[0];
        emxEnsureCapacity_real_T(&i_st, topProblem->RowReduction, xoffset,
                                 &gj_emlrtRTEI);
        j_st.site = &aib_emlrtRSI;
        if (varargin_1->size[0] > 2147483646) {
          k_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&k_st);
        }
        for (j = 0; j < m; j++) {
          topProblem->RowReduction->data[j] = assignment_data[j];
        }
        j_st.site = &yhb_emlrtRSI;
        if (varargin_1->size[1] > 2147483646) {
          k_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&k_st);
        }
        for (j = 2; j <= n; j++) {
          j_st.site = &xhb_emlrtRSI;
          if (m > 2147483646) {
            k_st.site = &k_emlrtRSI;
            check_forloop_overflow_error(&k_st);
          }
          for (b_j = 0; b_j < m; b_j++) {
            b = assignment_data[b_j + varargin_1->size[0] * (j - 1)];
            if (muDoubleScalarIsNaN(b)) {
              isValidProblem = false;
            } else if (muDoubleScalarIsNaN(
                           topProblem->RowReduction->data[b_j])) {
              isValidProblem = true;
            } else {
              isValidProblem = (topProblem->RowReduction->data[b_j] > b);
            }
            if (isValidProblem) {
              topProblem->RowReduction->data[b_j] = b;
            }
          }
        }
      }
      n = topProblem->RowSoln->size[0];
      m = d_nz->size[0];
      d_nz->size[0] = topProblem->RowSoln->size[0];
      emxEnsureCapacity_boolean_T(&st, d_nz, m, &vi_emlrtRTEI);
      isFiniteMatrix_data = d_nz->data;
      for (j = 0; j < n; j++) {
        isFiniteMatrix_data[j] =
            ((!muDoubleScalarIsInf(topProblem->RowSoln->data[j])) &&
             (!muDoubleScalarIsNaN(topProblem->RowSoln->data[j])));
      }
      b_st.site = &mgb_emlrtRSI;
      if (b_all(&b_st, d_nz)) {
        int32_T sz[2];
        int32_T b_loop_ub;
        uint32_T varargin_2[2];
        boolean_T exitg2;
        b_st.site = &ngb_emlrtRSI;
        c_AbstractProblemSolutionPair_g(&b_st, topProblem->RowSoln,
                                        topProblem->CostSize, assignment);
        assignment_data = assignment->data;
        sz[0] = topProblem->CostSize[0];
        sz[1] = topProblem->CostSize[1];
        m = d_nz->size[0];
        d_nz->size[0] = topProblem->RowSoln->size[0];
        emxEnsureCapacity_boolean_T(&st, d_nz, m, &bj_emlrtRTEI);
        isFiniteMatrix_data = d_nz->data;
        for (j = 0; j < n; j++) {
          isFiniteMatrix_data[j] = (topProblem->RowSoln->data[j] > sz[1]);
        }
        b_loop_ub = assignment->size[0];
        xoffset = r->size[0];
        r->size[0] = assignment->size[0];
        emxEnsureCapacity_boolean_T(&st, r, xoffset, &cj_emlrtRTEI);
        b_nz_data = r->data;
        for (j = 0; j < b_loop_ub; j++) {
          b_nz_data[j] = (assignment_data[j] > sz[0]);
        }
        if ((topProblem->RowSoln->size[0] != assignment->size[0]) &&
            ((topProblem->RowSoln->size[0] != 1) &&
             (assignment->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(topProblem->RowSoln->size[0],
                                      assignment->size[0], &gb_emlrtECI, &st);
        }
        xoffset = topProblem->IsDummySolution->size[0];
        topProblem->IsDummySolution->size[0] = topProblem->RowSoln->size[0];
        emxEnsureCapacity_boolean_T(&st, topProblem->IsDummySolution, xoffset,
                                    &dj_emlrtRTEI);
        for (j = 0; j < n; j++) {
          topProblem->IsDummySolution->data[j] =
              (isFiniteMatrix_data[j] && b_nz_data[j]);
        }
        b_st.site = &ogb_emlrtRSI;
        c_st.site = &dib_emlrtRSI;
        loop_ub = topProblem->PaddedCostMatrix->size[0];
        n = topProblem->PaddedCostMatrix->size[1];
        m = 0;
        exitg2 = false;
        while ((!exitg2) && (m <= assignment->size[0] - 1)) {
          if ((assignment_data[m] >= 1.0) && (assignment_data[m] <= loop_ub)) {
            m++;
          } else {
            emlrtErrorWithMessageIdR2018a(&c_st, &ub_emlrtRTEI,
                                          "MATLAB:sub2ind:IndexOutOfRange",
                                          "MATLAB:sub2ind:IndexOutOfRange", 0);
          }
        }
        sz[0] = assignment->size[0];
        sz[1] = 1;
        varargin_2[0] = (uint32_T)assignment->size[0];
        varargin_2[1] = 1U;
        isValidProblem = true;
        xoffset = 0;
        exitg2 = false;
        while ((!exitg2) && (xoffset < 2)) {
          if (sz[xoffset] != (int32_T)varargin_2[xoffset]) {
            isValidProblem = false;
            exitg2 = true;
          } else {
            xoffset++;
          }
        }
        if (!isValidProblem) {
          emlrtErrorWithMessageIdR2018a(
              &c_st, &vb_emlrtRTEI, "MATLAB:sub2ind:SubscriptVectorSize",
              "MATLAB:sub2ind:SubscriptVectorSize", 0);
        }
        xoffset = 0;
        exitg2 = false;
        while ((!exitg2) && (xoffset <= assignment->size[0] - 1)) {
          if ((assignment_data[xoffset + assignment->size[0]] >= 1.0) &&
              (assignment_data[xoffset + assignment->size[0]] <= n)) {
            xoffset++;
          } else {
            emlrtErrorWithMessageIdR2018a(&c_st, &ub_emlrtRTEI,
                                          "MATLAB:sub2ind:IndexOutOfRange",
                                          "MATLAB:sub2ind:IndexOutOfRange", 0);
          }
        }
        n = topProblem->PaddedCostMatrix->size[0] *
            topProblem->PaddedCostMatrix->size[1];
        xoffset = b_topProblem->size[0];
        b_topProblem->size[0] = assignment->size[0];
        emxEnsureCapacity_real_T(&st, b_topProblem, xoffset, &jj_emlrtRTEI);
        topProblem_data = b_topProblem->data;
        for (j = 0; j < b_loop_ub; j++) {
          m = (int32_T)assignment_data[j] +
              loop_ub * ((int32_T)assignment_data[j + assignment->size[0]] - 1);
          if ((m < 1) || (m > n)) {
            emlrtDynamicBoundsCheckR2012b(m, 1, n, &fh_emlrtBCI, &st);
          }
          topProblem_data[j] = topProblem->PaddedCostMatrix->data[m - 1];
        }
        b_st.site = &pgb_emlrtRSI;
        topProblem->BestSolutionCost = c_sum(&b_st, b_topProblem);
      } else {
        topProblem->BestSolutionCost = rtInf;
      }
      topProblem->IsSolved = true;
      st.site = &fgb_emlrtRSI;
      xoffset = topProblem->RowSoln->size[0];
      m = d_nz->size[0];
      d_nz->size[0] = topProblem->RowSoln->size[0];
      emxEnsureCapacity_boolean_T(&st, d_nz, m, &vi_emlrtRTEI);
      isFiniteMatrix_data = d_nz->data;
      for (j = 0; j < xoffset; j++) {
        isFiniteMatrix_data[j] =
            ((!muDoubleScalarIsInf(topProblem->RowSoln->data[j])) &&
             (!muDoubleScalarIsNaN(topProblem->RowSoln->data[j])));
      }
      b_st.site = &eib_emlrtRSI;
      if (b_all(&b_st, d_nz)) {
        xoffset = topProblem->ColSoln->size[1];
        m = d_nz->size[0];
        d_nz->size[0] = topProblem->ColSoln->size[1];
        emxEnsureCapacity_boolean_T(&st, d_nz, m, &vi_emlrtRTEI);
        isFiniteMatrix_data = d_nz->data;
        for (j = 0; j < xoffset; j++) {
          b = topProblem->ColSoln->data[j];
          isFiniteMatrix_data[j] =
              ((!muDoubleScalarIsInf(b)) && (!muDoubleScalarIsNaN(b)));
        }
        b_st.site = &eib_emlrtRSI;
        if (b_all(&b_st, d_nz) &&
            ((!muDoubleScalarIsInf(topProblem->BestSolutionCost)) &&
             (!muDoubleScalarIsNaN(topProblem->BestSolutionCost)))) {
          isValidProblem = true;
        } else {
          isValidProblem = false;
        }
      } else {
        isValidProblem = false;
      }
      if (isValidProblem) {
        m = obj->AllProblemList->size[0] - 1;
        if (m < 0) {
          emlrtDynamicBoundsCheckR2012b(0, 0, m, &bh_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        c_emxCopyStruct_fusion_internal(sp, &obj->AllProblemList->data[0],
                                        topProblem, &fj_emlrtRTEI);
        st.site = &ggb_emlrtRSI;
        ProblemSolutionQueue_sortQueue(&st, obj);
      } else {
        st.site = &hgb_emlrtRSI;
        m = obj->NumProblems;
        b_st.site = &iib_emlrtRSI;
        if (obj->NumProblems - 1 > 2147483646) {
          c_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (j = 0; j <= m - 2; j++) {
          xoffset = obj->AllProblemList->size[0] - 1;
          if ((j + 1 < 0) || (j + 1 > xoffset)) {
            emlrtDynamicBoundsCheckR2012b(j + 1, 0, xoffset, &dh_emlrtBCI, &st);
          }
          if (j > xoffset) {
            emlrtDynamicBoundsCheckR2012b(j, 0, xoffset, &eh_emlrtBCI, &st);
          }
          c_emxCopyStruct_fusion_internal(&st, &obj->AllProblemList->data[j],
                                          &obj->AllProblemList->data[j + 1],
                                          &ij_emlrtRTEI);
        }
        obj->NumProblems--;
      }
      if (obj->NumProblems == 0) {
        isValidProblem = false;
        exitg1 = 1;
      } else {
        m = obj->AllProblemList->size[0] - 1;
        if (m < 0) {
          emlrtDynamicBoundsCheckR2012b(0, 0, m, &ch_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        c_emxCopyStruct_fusion_internal(
            sp, topProblem, &obj->AllProblemList->data[0], &hj_emlrtRTEI);
      }
    } else {
      int32_T xoffset;
      st.site = &igb_emlrtRSI;
      xoffset = topProblem->RowSoln->size[0];
      m = d_nz->size[0];
      d_nz->size[0] = topProblem->RowSoln->size[0];
      emxEnsureCapacity_boolean_T(&st, d_nz, m, &vi_emlrtRTEI);
      isFiniteMatrix_data = d_nz->data;
      for (j = 0; j < xoffset; j++) {
        isFiniteMatrix_data[j] =
            ((!muDoubleScalarIsInf(topProblem->RowSoln->data[j])) &&
             (!muDoubleScalarIsNaN(topProblem->RowSoln->data[j])));
      }
      b_st.site = &eib_emlrtRSI;
      if (b_all(&b_st, d_nz)) {
        xoffset = topProblem->ColSoln->size[1];
        m = d_nz->size[0];
        d_nz->size[0] = topProblem->ColSoln->size[1];
        emxEnsureCapacity_boolean_T(&st, d_nz, m, &vi_emlrtRTEI);
        isFiniteMatrix_data = d_nz->data;
        for (j = 0; j < xoffset; j++) {
          real_T b;
          b = topProblem->ColSoln->data[j];
          isFiniteMatrix_data[j] =
              ((!muDoubleScalarIsInf(b)) && (!muDoubleScalarIsNaN(b)));
        }
        b_st.site = &eib_emlrtRSI;
        if (b_all(&b_st, d_nz) &&
            ((!muDoubleScalarIsInf(topProblem->BestSolutionCost)) &&
             (!muDoubleScalarIsNaN(topProblem->BestSolutionCost)))) {
          isValidProblem = true;
        } else {
          isValidProblem = false;
        }
      } else {
        isValidProblem = false;
      }
      if (isValidProblem) {
        st.site = &jgb_emlrtRSI;
        m = obj->NumProblems;
        b_st.site = &iib_emlrtRSI;
        if (obj->NumProblems - 1 > 2147483646) {
          c_st.site = &k_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (j = 0; j <= m - 2; j++) {
          xoffset = obj->AllProblemList->size[0] - 1;
          if ((j + 1 < 0) || (j + 1 > xoffset)) {
            emlrtDynamicBoundsCheckR2012b(j + 1, 0, xoffset, &dh_emlrtBCI, &st);
          }
          if (j > xoffset) {
            emlrtDynamicBoundsCheckR2012b(j, 0, xoffset, &eh_emlrtBCI, &st);
          }
          c_emxCopyStruct_fusion_internal(&st, &obj->AllProblemList->data[j],
                                          &obj->AllProblemList->data[j + 1],
                                          &yi_emlrtRTEI);
        }
        obj->NumProblems--;
        isValidProblem = true;
      } else {
        isValidProblem = false;
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  emxFree_real_T(sp, &b_topProblem);
  emxFree_boolean_T(sp, &d_nz);
  emxFree_boolean_T(sp, &c_nz);
  emxFree_real_T(sp, &varargin_1);
  emxFree_int32_T(sp, &b_nz);
  emxFree_int32_T(sp, &nz);
  emxFree_boolean_T(sp, &isFiniteMatrix);
  emxFree_boolean_T(sp, &r);
  emxFree_real_T(sp, &assignment);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return isValidProblem;
}

void c_ProblemSolutionQueue_formatSo(
    const emlrtStack *sp, int32_T obj_NumProblems,
    int32_T obj_MaxNumSubProblems,
    const c_emxArray_fusion_internal_assi *obj_AllProblemList,
    emxArray_cell_wrap_81 *assignments, emxArray_cell_wrap_82 *unassignedRows,
    emxArray_cell_wrap_82 *unassignedCols, emxArray_real_T *cost)
{
  const c_fusion_internal_assignment_JV *obj_AllProblemList_data;
  cell_wrap_81 *assignments_data;
  cell_wrap_82 *unassignedCols_data;
  cell_wrap_82 *unassignedRows_data;
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *r;
  emxArray_uint32_T *b_assignments;
  real_T *cost_data;
  real_T *r1;
  int32_T idx_data[51];
  int32_T iwork_data[51];
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T pEnd;
  int32_T qEnd;
  uint32_T b_unassignedRows_data[51];
  uint32_T *b_assignments_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  obj_AllProblemList_data = obj_AllProblemList->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (obj_NumProblems > obj_MaxNumSubProblems) {
    emlrtErrorWithMessageIdR2018a(sp, &cc_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (obj_NumProblems < 0) {
    emlrtNonNegativeCheckR2012b(obj_NumProblems, &i_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  pEnd = assignments->size[0];
  assignments->size[0] = obj_NumProblems;
  emxEnsureCapacity_cell_wrap_81(sp, assignments, pEnd, &vh_emlrtRTEI);
  assignments_data = assignments->data;
  for (k = 0; k < obj_NumProblems; k++) {
    if (k > assignments->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(k, 0, assignments->size[0] - 1,
                                    &tf_emlrtBCI, (emlrtConstCTX)sp);
    }
    assignments_data[k].f1.size[0] = 0;
    if (k > assignments->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(k, 0, assignments->size[0] - 1,
                                    &tf_emlrtBCI, (emlrtConstCTX)sp);
    }
    assignments_data[k].f1.size[1] = 2;
  }
  pEnd = unassignedRows->size[0];
  unassignedRows->size[0] = obj_NumProblems;
  emxEnsureCapacity_cell_wrap_82(sp, unassignedRows, pEnd, &vh_emlrtRTEI);
  unassignedRows_data = unassignedRows->data;
  for (k = 0; k < obj_NumProblems; k++) {
    if (k > unassignedRows->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(k, 0, unassignedRows->size[0] - 1,
                                    &tf_emlrtBCI, (emlrtConstCTX)sp);
    }
    unassignedRows_data[k].f1->size[0] = 0;
  }
  pEnd = unassignedCols->size[0];
  unassignedCols->size[0] = obj_NumProblems;
  emxEnsureCapacity_cell_wrap_82(sp, unassignedCols, pEnd, &vh_emlrtRTEI);
  unassignedCols_data = unassignedCols->data;
  for (k = 0; k < obj_NumProblems; k++) {
    if (k > unassignedCols->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(k, 0, unassignedCols->size[0] - 1,
                                    &tf_emlrtBCI, (emlrtConstCTX)sp);
    }
    unassignedCols_data[k].f1->size[0] = 0;
  }
  pEnd = cost->size[0];
  cost->size[0] = obj_NumProblems;
  emxEnsureCapacity_real_T(sp, cost, pEnd, &wh_emlrtRTEI);
  cost_data = cost->data;
  st.site = &djb_emlrtRSI;
  if (obj_NumProblems > 2147483646) {
    b_st.site = &k_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  pEnd = unassignedCols->size[0];
  unassignedCols->size[0] = obj_NumProblems;
  emxEnsureCapacity_cell_wrap_82(sp, unassignedCols, pEnd, &xh_emlrtRTEI);
  unassignedCols_data = unassignedCols->data;
  pEnd = unassignedRows->size[0];
  unassignedRows->size[0] = obj_NumProblems;
  emxEnsureCapacity_cell_wrap_82(sp, unassignedRows, pEnd, &xh_emlrtRTEI);
  unassignedRows_data = unassignedRows->data;
  pEnd = assignments->size[0];
  assignments->size[0] = obj_NumProblems;
  emxEnsureCapacity_cell_wrap_81(sp, assignments, pEnd, &xh_emlrtRTEI);
  assignments_data = assignments->data;
  emxInit_uint32_T(sp, &b_assignments, 2, &vh_emlrtRTEI, true);
  emxInit_real_T(sp, &r, 2, &di_emlrtRTEI, true);
  for (i = 0; i < obj_NumProblems; i++) {
    int32_T b_i;
    int32_T loop_ub;
    int32_T n;
    int32_T unassignedRows_size;
    uint32_T ycol_data[51];
    uint32_T v1;
    if (i > obj_AllProblemList->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, obj_AllProblemList->size[0] - 1,
                                    &sf_emlrtBCI, (emlrtConstCTX)sp);
    }
    st.site = &ejb_emlrtRSI;
    b_st.site = &fjb_emlrtRSI;
    c_AbstractProblemSolutionPair_g(&b_st, obj_AllProblemList_data[i].RowSoln,
                                    obj_AllProblemList_data[i].CostSize, r);
    r1 = r->data;
    pEnd = b_assignments->size[0] * b_assignments->size[1];
    b_assignments->size[0] = r->size[0];
    b_assignments->size[1] = 2;
    emxEnsureCapacity_uint32_T(&st, b_assignments, pEnd, &yh_emlrtRTEI);
    b_assignments_data = b_assignments->data;
    pEnd = r->size[0] << 1;
    for (k = 0; k < pEnd; k++) {
      real_T d;
      d = muDoubleScalarRound(r1[k]);
      if (d < 4.294967296E+9) {
        if (d >= 0.0) {
          v1 = (uint32_T)d;
        } else {
          v1 = 0U;
        }
      } else if (d >= 4.294967296E+9) {
        v1 = MAX_uint32_T;
      } else {
        v1 = 0U;
      }
      b_assignments_data[k] = v1;
    }
    b_i = unassignedCols->size[0] - 1;
    if (i > unassignedCols->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, unassignedCols->size[0] - 1,
                                    &uf_emlrtBCI, &st);
    }
    b_st.site = &fjb_emlrtRSI;
    unassignedRows_size = kbestRemoveUnassigned(
        &b_st, b_assignments, obj_AllProblemList_data[i].CostSize,
        b_unassignedRows_data, unassignedCols_data[i].f1);
    b_assignments_data = b_assignments->data;
    n = b_assignments->size[0] + 1;
    loop_ub = b_assignments->size[0];
    if (loop_ub - 1 >= 0) {
      memset(&idx_data[0], 0, (uint32_T)loop_ub * sizeof(int32_T));
    }
    if (b_assignments->size[0] == 0) {
      for (k = 0; k <= n - 2; k++) {
        idx_data[k] = k + 1;
      }
    } else {
      int32_T b_p;
      int32_T c_i;
      uint32_T v2;
      boolean_T exitg1;
      boolean_T p;
      pEnd = b_assignments->size[0] - 1;
      for (k = 1; k <= pEnd; k += 2) {
        p = true;
        b_p = 0;
        exitg1 = false;
        while ((!exitg1) && (b_p < 2)) {
          v1 = b_assignments_data[(k + b_assignments->size[0] * b_p) - 1];
          v2 = b_assignments_data[k + b_assignments->size[0] * b_p];
          if (v1 == v2) {
            b_p++;
          } else {
            p = (v1 <= v2);
            exitg1 = true;
          }
        }
        if (p) {
          idx_data[k - 1] = k;
          idx_data[k] = k + 1;
        } else {
          idx_data[k - 1] = k + 1;
          idx_data[k] = k;
        }
      }
      if (((uint32_T)b_assignments->size[0] & 1U) != 0U) {
        idx_data[b_assignments->size[0] - 1] = b_assignments->size[0];
      }
      c_i = 2;
      while (c_i < n - 1) {
        int32_T b_j;
        int32_T i2;
        i2 = c_i << 1;
        b_j = 1;
        for (pEnd = c_i + 1; pEnd < n; pEnd = qEnd + c_i) {
          int32_T b_k;
          int32_T kEnd;
          int32_T q;
          b_p = b_j;
          q = pEnd;
          qEnd = b_j + i2;
          if (qEnd > n) {
            qEnd = n;
          }
          b_k = 0;
          kEnd = qEnd - b_j;
          while (b_k < kEnd) {
            int32_T c_k;
            p = true;
            c_k = 0;
            exitg1 = false;
            while ((!exitg1) && (c_k < 2)) {
              v1 = b_assignments_data[(idx_data[b_p - 1] +
                                       b_assignments->size[0] * c_k) -
                                      1];
              v2 = b_assignments_data[(idx_data[q - 1] +
                                       b_assignments->size[0] * c_k) -
                                      1];
              if (v1 == v2) {
                c_k++;
              } else {
                p = (v1 <= v2);
                exitg1 = true;
              }
            }
            if (p) {
              iwork_data[b_k] = idx_data[b_p - 1];
              b_p++;
              if (b_p == pEnd) {
                while (q < qEnd) {
                  b_k++;
                  iwork_data[b_k] = idx_data[q - 1];
                  q++;
                }
              }
            } else {
              iwork_data[b_k] = idx_data[q - 1];
              q++;
              if (q == qEnd) {
                while (b_p < pEnd) {
                  b_k++;
                  iwork_data[b_k] = idx_data[b_p - 1];
                  b_p++;
                }
              }
            }
            b_k++;
          }
          for (k = 0; k < kEnd; k++) {
            idx_data[(b_j + k) - 1] = iwork_data[k];
          }
          b_j = qEnd;
        }
        c_i = i2;
      }
    }
    for (j = 0; j < 2; j++) {
      for (k = 0; k < loop_ub; k++) {
        ycol_data[k] =
            b_assignments_data[(idx_data[k] + b_assignments->size[0] * j) - 1];
      }
      for (k = 0; k < loop_ub; k++) {
        b_assignments_data[k + b_assignments->size[0] * j] = ycol_data[k];
      }
    }
    if (i + 1 > cost->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, cost->size[0], &vf_emlrtBCI, &st);
    }
    cost_data[i] = obj_AllProblemList_data[i].BestSolutionCost;
    if (i > unassignedCols->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, unassignedCols->size[0] - 1,
                                    &rf_emlrtBCI, (emlrtConstCTX)sp);
    }
    assignments_data[i].f1.size[0] = b_assignments->size[0];
    assignments_data[i].f1.size[1] = 2;
    pEnd = b_assignments->size[0] << 1;
    for (k = 0; k < pEnd; k++) {
      assignments_data[i].f1.data[k] = b_assignments_data[k];
    }
    if (i > unassignedCols->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(i, 0, unassignedCols->size[0] - 1,
                                    &wf_emlrtBCI, (emlrtConstCTX)sp);
    }
    pEnd = unassignedRows_data[i].f1->size[0];
    unassignedRows_data[i].f1->size[0] = unassignedRows_size;
    emxEnsureCapacity_uint32_T(sp, unassignedRows_data[i].f1, pEnd,
                               &ci_emlrtRTEI);
    for (k = 0; k < unassignedRows_size; k++) {
      if (i > b_i) {
        emlrtDynamicBoundsCheckR2012b(i, 0, b_i, &wf_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      unassignedRows_data[i].f1->data[k] = b_unassignedRows_data[k];
    }
  }
  emxFree_real_T(sp, &r);
  emxFree_uint32_T(sp, &b_assignments);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (ProblemSolutionQueue.c) */
