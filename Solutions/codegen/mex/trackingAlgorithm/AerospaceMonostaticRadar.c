/*
 * AerospaceMonostaticRadar.c
 *
 * Code generation for function 'AerospaceMonostaticRadar'
 *
 */

/* Include files */
#include "AerospaceMonostaticRadar.h"
#include "feul2qparts.h"
#include "quat2rotmat.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "validator_check_size.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ip_emlrtRSI = {
    217,                                   /* lineNo */
    "AerospaceMonostaticRadar/parseInput", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo jp_emlrtRSI = {
    219,                                   /* lineNo */
    "AerospaceMonostaticRadar/parseInput", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo kp_emlrtRSI = {
    402,                                          /* lineNo */
    "AerospaceMonostaticRadar/parseMeasurements", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo np_emlrtRSI = {
    444,                                       /* lineNo */
    "AerospaceMonostaticRadar/parseModelData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo op_emlrtRSI = {
    449,                                       /* lineNo */
    "AerospaceMonostaticRadar/parseModelData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo pp_emlrtRSI = {
    455,                                       /* lineNo */
    "AerospaceMonostaticRadar/parseModelData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo qp_emlrtRSI = {
    462,                                       /* lineNo */
    "AerospaceMonostaticRadar/parseModelData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo rp_emlrtRSI = {
    463,                                       /* lineNo */
    "AerospaceMonostaticRadar/parseModelData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo cq_emlrtRSI = {
    228,                                    /* lineNo */
    "AerospaceMonostaticRadar/updateModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo dq_emlrtRSI = {
    229,                                    /* lineNo */
    "AerospaceMonostaticRadar/updateModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo eq_emlrtRSI = {
    322,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo fq_emlrtRSI = {
    342,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo gq_emlrtRSI = {
    343,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo hq_emlrtRSI = {
    346,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo iq_emlrtRSI = {
    347,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo jq_emlrtRSI = {
    348,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo kq_emlrtRSI = {
    349,                                               /* lineNo */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo lq_emlrtRSI =
    {
        19,    /* lineNo */
        "abs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elfun\\abs.m" /* pathName
                                                                          */
};

static emlrtRSInfo nq_emlrtRSI = {
    17,    /* lineNo */
    "min", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\min.m" /* pathName
                                                                        */
};

static emlrtRSInfo oq_emlrtRSI =
    {
        69,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo pq_emlrtRSI =
    {
        119,       /* lineNo */
        "minimum", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo rq_emlrtRSI = {
    126,          /* lineNo */
    "ypr2rotmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\radarfusion\\+fusion\\+internal\\+"
    "frames\\ypr2rotmat.m" /* pathName */
};

static emlrtRSInfo vq_emlrtRSI = {
    80, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/Orientation (generated property "
    "set method)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo wq_emlrtRSI =
    {
        80, /* lineNo */
        "AzimuthElevationRangeAndRangeRateModel/Orientation (property "
        "validation)", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
        "tracker\\+measurement\\AzimuthElevationRangeAndRange"
        "RateModel.m" /* pathName */
};

static emlrtRSInfo xq_emlrtRSI = {
    84, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/AzimuthVariance (generated "
    "property set method)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo yq_emlrtRSI = {
    84, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/AzimuthVariance (property "
    "validation)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo cr_emlrtRSI = {
    88, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/ElevationVariance (generated "
    "property set method)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo dr_emlrtRSI = {
    88, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/ElevationVariance (property "
    "validation)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo er_emlrtRSI = {
    92, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/RangeVariance (generated property "
    "set method)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo fr_emlrtRSI = {
    92, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/RangeVariance (property "
    "validation)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo gr_emlrtRSI = {
    96, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/RangeRateVariance (generated "
    "property set method)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo hr_emlrtRSI = {
    96, /* lineNo */
    "AzimuthElevationRangeAndRangeRateModel/RangeRateVariance (property "
    "validation)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "measurement\\AzimuthElevationRangeAndRange"
    "RateModel.m" /* pathName */
};

static emlrtRSInfo ir_emlrtRSI = {
    373,                                                 /* lineNo */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo jr_emlrtRSI = {
    390,                                                 /* lineNo */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pathName */
};

static emlrtRSInfo kr_emlrtRSI = {
    125,          /* lineNo */
    "ypr2rotmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\radarfusion\\+fusion\\+internal\\+"
    "frames\\ypr2rotmat.m" /* pathName */
};

static emlrtRSInfo lr_emlrtRSI = {
    111,        /* lineNo */
    "ypr2quat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\radarfusion\\+fusion\\+internal\\+"
    "frames\\ypr2quat.m" /* pathName */
};

static emlrtRSInfo mr_emlrtRSI = {
    12,                          /* lineNo */
    "quaternioncg/quaternioncg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\+coder\\@quat"
    "ernioncg\\quaternioncg.m" /* pathName */
};

static emlrtRSInfo nr_emlrtRSI = {
    69,                              /* lineNo */
    "quaternionBase/quaternionBase", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\quaternionBase.m" /* pathName */
};

static emlrtRSInfo or_emlrtRSI = {
    191,                   /* lineNo */
    "partsFromConversion", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\quaternionBase.m" /* pathName */
};

static emlrtRSInfo pr_emlrtRSI = {
    269,              /* lineNo */
    "partsFromEuler", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\quaternionBase.m" /* pathName */
};

static emlrtRSInfo ls_emlrtRSI =
    {
        77, /* lineNo */
        "FieldOfViewAndRangeRateModel/Orientation (generated property set "
        "method)", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
        "tracker\\+detectability\\FieldOfViewAndRangeRateMode"
        "l.m" /* pathName */
};

static emlrtRSInfo ms_emlrtRSI = {
    77, /* lineNo */
    "FieldOfViewAndRangeRateModel/Orientation (property validation)", /* fcnName
                                                                       */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "detectability\\FieldOfViewAndRangeRateMode"
    "l.m" /* pathName */
};

static emlrtBCInfo fc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    461,                                       /* lineNo */
    31,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo gc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    462,                                       /* lineNo */
    76,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo hc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    462,                                       /* lineNo */
    31,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo ic_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    463,                                       /* lineNo */
    72,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo jc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    463,                                       /* lineNo */
    31,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo kc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    464,                                       /* lineNo */
    31,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo lc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    465,                                       /* lineNo */
    31,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo mc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    461,                                       /* lineNo */
    21,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo nc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    464,                                       /* lineNo */
    21,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo oc_emlrtBCI = {
    -1,                                        /* iFirst */
    -1,                                        /* iLast */
    465,                                       /* lineNo */
    21,                                        /* colNo */
    "",                                        /* aName */
    "AerospaceMonostaticRadar/parseModelData", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo rg_emlrtBCI = {
    -1,                                                /* iFirst */
    -1,                                                /* iLast */
    338,                                               /* lineNo */
    24,                                                /* colNo */
    "",                                                /* aName */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo sg_emlrtBCI = {
    -1,                                                /* iFirst */
    -1,                                                /* iLast */
    338,                                               /* lineNo */
    59,                                                /* colNo */
    "",                                                /* aName */
    "AerospaceMonostaticRadar/updateMeasurementModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo t_emlrtECI = {
    1,             /* nDims */
    114,           /* lineNo */
    14,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo u_emlrtECI = {
    1,             /* nDims */
    114,           /* lineNo */
    33,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo v_emlrtECI = {
    1,             /* nDims */
    113,           /* lineNo */
    14,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo w_emlrtECI = {
    1,             /* nDims */
    113,           /* lineNo */
    33,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo x_emlrtECI = {
    1,             /* nDims */
    112,           /* lineNo */
    14,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo y_emlrtECI = {
    1,             /* nDims */
    112,           /* lineNo */
    33,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo ab_emlrtECI = {
    1,             /* nDims */
    111,           /* lineNo */
    14,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo bb_emlrtECI = {
    1,             /* nDims */
    111,           /* lineNo */
    33,            /* colNo */
    "feul2qparts", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\rotations_utils\\+matlabshared\\+"
    "rotations\\+internal\\feul2qparts.m" /* pName */
};

static emlrtECInfo cb_emlrtECI = {
    -1,                                                  /* nDims */
    390,                                                 /* lineNo */
    17,                                                  /* colNo */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pName */
};

static emlrtBCInfo tg_emlrtBCI = {
    -1,                                                  /* iFirst */
    -1,                                                  /* iLast */
    390,                                                 /* lineNo */
    60,                                                  /* colNo */
    "",                                                  /* aName */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo db_emlrtECI = {
    -1,                                                  /* nDims */
    372,                                                 /* lineNo */
    13,                                                  /* colNo */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pName */
};

static emlrtECInfo eb_emlrtECI = {
    -1,                                                  /* nDims */
    371,                                                 /* lineNo */
    13,                                                  /* colNo */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pName */
};

static emlrtBCInfo ug_emlrtBCI = {
    -1,                                                  /* iFirst */
    -1,                                                  /* iLast */
    372,                                                 /* lineNo */
    53,                                                  /* colNo */
    "",                                                  /* aName */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo vg_emlrtBCI = {
    -1,                                                  /* iFirst */
    -1,                                                  /* iLast */
    372,                                                 /* lineNo */
    51,                                                  /* colNo */
    "",                                                  /* aName */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo wg_emlrtBCI = {
    -1,                                                  /* iFirst */
    -1,                                                  /* iLast */
    371,                                                 /* lineNo */
    50,                                                  /* colNo */
    "",                                                  /* aName */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo xg_emlrtBCI = {
    -1,                                                  /* iFirst */
    -1,                                                  /* iLast */
    371,                                                 /* lineNo */
    48,                                                  /* colNo */
    "",                                                  /* aName */
    "AerospaceMonostaticRadar/updateDetectabilityModel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m", /* pName */
    0                                          /* checkKind */
};

static emlrtRTEInfo le_emlrtRTEI = {
    219,                        /* lineNo */
    13,                         /* colNo */
    "AerospaceMonostaticRadar", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "sensorspecs\\AerospaceMonostaticRadar.m" /* pName */
};

/* Function Declarations */
static void
d_AerospaceMonostaticRadar_upda(const emlrtStack *sp,
                                c_fusion_tracker_sensorspecs_Ae *obj,
                                const int32_T modelData_LookTime_size[2],
                                const real_T modelData_LookAzimuth_data[],
                                const int32_T modelData_LookAzimuth_size[2],
                                const real_T modelData_LookElevation_data[],
                                const int32_T modelData_LookElevation_size[2]);

static void minus(real_T in1_data[], int32_T in1_size[2],
                  const real_T in2_data[], const int32_T in2_size[2],
                  const real_T in3_data[], const int32_T in3_size[2]);

/* Function Definitions */
static void
d_AerospaceMonostaticRadar_upda(const emlrtStack *sp,
                                c_fusion_tracker_sensorspecs_Ae *obj,
                                const int32_T modelData_LookTime_size[2],
                                const real_T modelData_LookAzimuth_data[],
                                const int32_T modelData_LookAzimuth_size[2],
                                const real_T modelData_LookElevation_data[],
                                const int32_T modelData_LookElevation_size[2])
{
  __m128d r;
  __m128d r1;
  c_fusion_tracker_detectability_ beamModel;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T lookRot_data[900];
  real_T ypr_data[300];
  real_T a_data[100];
  real_T b_data[100];
  real_T b_tmp_data[100];
  real_T c_data[100];
  real_T c_tmp_data[100];
  real_T d_tmp_data[100];
  real_T q_b_data[100];
  real_T q_c_data[100];
  real_T q_d_data[100];
  real_T sina_data[100];
  real_T sinb_data[100];
  real_T sinc_data[100];
  real_T tmp_data[100];
  int32_T lookRot_size[3];
  int32_T b_iv[2];
  int32_T b_iv1[2];
  int32_T tmp_size[2];
  int32_T b_loop_ub;
  int32_T b_size;
  int32_T b_tmp_size;
  int32_T c_loop_ub;
  int32_T c_tmp_size;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T ypr_size_idx_0;
  boolean_T p;
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
  loop_ub = modelData_LookTime_size[1];
  obj->DetectabilityModel.NumModels = modelData_LookTime_size[1];
  beamModel = obj->DetectabilityModel.FieldsOfView[0];
  ypr_size_idx_0 = modelData_LookTime_size[1];
  b_loop_ub = modelData_LookTime_size[1] * 3;
  if (b_loop_ub - 1 >= 0) {
    memset(&ypr_data[0], 0, (uint32_T)b_loop_ub * sizeof(real_T));
  }
  if (modelData_LookTime_size[1] < 1) {
    c_loop_ub = 0;
  } else {
    if (modelData_LookAzimuth_size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, modelData_LookAzimuth_size[1],
                                    &xg_emlrtBCI, (emlrtConstCTX)sp);
    }
    if (modelData_LookTime_size[1] > modelData_LookAzimuth_size[1]) {
      emlrtDynamicBoundsCheckR2012b(modelData_LookTime_size[1], 1,
                                    modelData_LookAzimuth_size[1], &wg_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    c_loop_ub = modelData_LookTime_size[1];
  }
  b_iv[0] = 1;
  b_iv[1] = c_loop_ub;
  emlrtSubAssignSizeCheckR2012b(&modelData_LookTime_size[1], 1, &b_iv[0], 2,
                                &eb_emlrtECI, (emlrtCTX)sp);
  if (loop_ub - 1 >= 0) {
    memcpy(&ypr_data[0], &modelData_LookAzimuth_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  if (modelData_LookTime_size[1] < 1) {
    c_loop_ub = 0;
  } else {
    if (modelData_LookElevation_size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, modelData_LookElevation_size[1],
                                    &vg_emlrtBCI, (emlrtConstCTX)sp);
    }
    if (modelData_LookTime_size[1] > modelData_LookElevation_size[1]) {
      emlrtDynamicBoundsCheckR2012b(modelData_LookTime_size[1], 1,
                                    modelData_LookElevation_size[1],
                                    &ug_emlrtBCI, (emlrtConstCTX)sp);
    }
    c_loop_ub = modelData_LookTime_size[1];
  }
  tmp_size[0] = 1;
  tmp_size[1] = c_loop_ub;
  scalarLB = (c_loop_ub / 2) << 1;
  vectorUB = scalarLB - 2;
  for (k = 0; k <= vectorUB; k += 2) {
    _mm_storeu_pd(&tmp_data[k],
                  _mm_mul_pd(_mm_loadu_pd(&modelData_LookElevation_data[k]),
                             _mm_set1_pd(-1.0)));
  }
  for (k = scalarLB; k < c_loop_ub; k++) {
    tmp_data[k] = -modelData_LookElevation_data[k];
  }
  emlrtSubAssignSizeCheckR2012b(&modelData_LookTime_size[1], 1, &tmp_size[0], 2,
                                &db_emlrtECI, (emlrtCTX)sp);
  for (k = 0; k < loop_ub; k++) {
    ypr_data[k + ypr_size_idx_0] = tmp_data[k];
  }
  st.site = &ir_emlrtRSI;
  b_st.site = &kr_emlrtRSI;
  c_st.site = &lr_emlrtRSI;
  d_st.site = &mr_emlrtRSI;
  e_st.site = &nr_emlrtRSI;
  f_st.site = &or_emlrtRSI;
  g_st.site = &pr_emlrtRSI;
  c_loop_ub = (b_loop_ub / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&ypr_data[k]);
    _mm_storeu_pd(
        &ypr_data[k],
        _mm_div_pd(_mm_div_pd(_mm_mul_pd(r, _mm_set1_pd(3.1415926535897931)),
                              _mm_set1_pd(180.0)),
                   _mm_set1_pd(2.0)));
  }
  for (k = c_loop_ub; k < b_loop_ub; k++) {
    ypr_data[k] = ypr_data[k] * 3.1415926535897931 / 180.0 / 2.0;
  }
  b_size = modelData_LookTime_size[1];
  if (ypr_size_idx_0 - 1 >= 0) {
    memcpy(&a_data[0], &ypr_data[0], (uint32_T)ypr_size_idx_0 * sizeof(real_T));
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    b_data[k] = ypr_data[k + ypr_size_idx_0];
    c_data[k] = ypr_data[k + ypr_size_idx_0 * 2];
  }
  if (ypr_size_idx_0 - 1 >= 0) {
    memcpy(&sina_data[0], &ypr_data[0],
           (uint32_T)ypr_size_idx_0 * sizeof(real_T));
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    sina_data[k] = muDoubleScalarSin(sina_data[k]);
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    sinb_data[k] = ypr_data[k + ypr_size_idx_0];
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    sinb_data[k] = muDoubleScalarSin(sinb_data[k]);
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    sinc_data[k] = ypr_data[k + ypr_size_idx_0 * 2];
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    sinc_data[k] = muDoubleScalarSin(sinc_data[k]);
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    a_data[k] = muDoubleScalarCos(a_data[k]);
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    b_data[k] = muDoubleScalarCos(b_data[k]);
  }
  for (k = 0; k < ypr_size_idx_0; k++) {
    c_data[k] = muDoubleScalarCos(c_data[k]);
  }
  b_tmp_size = modelData_LookTime_size[1];
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&a_data[k]);
    r1 = _mm_loadu_pd(&b_data[k]);
    _mm_storeu_pd(&b_tmp_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    b_tmp_data[k] = a_data[k] * b_data[k];
  }
  p = ((b_tmp_size != modelData_LookTime_size[1]) &&
       ((b_tmp_size != 1) && (modelData_LookTime_size[1] != 1)));
  if (p) {
    emlrtDimSizeImpxCheckR2021b(b_tmp_size, modelData_LookTime_size[1],
                                &ab_emlrtECI, &g_st);
  }
  vectorUB = modelData_LookTime_size[1];
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&sina_data[k]);
    r1 = _mm_loadu_pd(&sinb_data[k]);
    _mm_storeu_pd(&c_tmp_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    c_tmp_data[k] = sina_data[k] * sinb_data[k];
  }
  if ((vectorUB != modelData_LookTime_size[1]) &&
      ((vectorUB != 1) && (modelData_LookTime_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(vectorUB, modelData_LookTime_size[1],
                                &bb_emlrtECI, &g_st);
  }
  if (b_tmp_size == modelData_LookTime_size[1]) {
    c_tmp_size = b_tmp_size;
    c_loop_ub = (b_tmp_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_tmp_data[k]);
      r1 = _mm_loadu_pd(&c_data[k]);
      _mm_storeu_pd(&d_tmp_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_tmp_size; k++) {
      d_tmp_data[k] = b_tmp_data[k] * c_data[k];
    }
  } else {
    c_tmp_size = b_times(d_tmp_data, b_tmp_data, &b_tmp_size, c_data,
                         &modelData_LookTime_size[1]);
  }
  if (vectorUB == modelData_LookTime_size[1]) {
    c_loop_ub = (vectorUB / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&c_tmp_data[k]);
      r1 = _mm_loadu_pd(&sinc_data[k]);
      _mm_storeu_pd(&c_tmp_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < vectorUB; k++) {
      c_tmp_data[k] *= sinc_data[k];
    }
  } else {
    times(c_tmp_data, &vectorUB, sinc_data, &modelData_LookTime_size[1]);
  }
  if ((c_tmp_size != vectorUB) && ((c_tmp_size != 1) && (vectorUB != 1))) {
    emlrtDimSizeImpxCheckR2021b(c_tmp_size, vectorUB, &ab_emlrtECI, &g_st);
  }
  if (c_tmp_size == vectorUB) {
    c_loop_ub = (c_tmp_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&d_tmp_data[k]);
      r1 = _mm_loadu_pd(&c_tmp_data[k]);
      _mm_storeu_pd(&tmp_data[k], _mm_add_pd(r, r1));
    }
    for (k = c_loop_ub; k < c_tmp_size; k++) {
      tmp_data[k] = d_tmp_data[k] + c_tmp_data[k];
    }
  } else {
    c_tmp_size =
        b_plus(tmp_data, d_tmp_data, &c_tmp_size, c_tmp_data, &vectorUB);
  }
  if (p) {
    emlrtDimSizeImpxCheckR2021b(b_tmp_size, modelData_LookTime_size[1],
                                &x_emlrtECI, &g_st);
  }
  vectorUB = modelData_LookTime_size[1];
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&c_data[k]);
    r1 = _mm_loadu_pd(&sina_data[k]);
    _mm_storeu_pd(&c_tmp_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    c_tmp_data[k] = c_data[k] * sina_data[k];
  }
  if ((vectorUB != modelData_LookTime_size[1]) &&
      ((vectorUB != 1) && (modelData_LookTime_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(vectorUB, modelData_LookTime_size[1],
                                &y_emlrtECI, &g_st);
  }
  if (b_tmp_size == modelData_LookTime_size[1]) {
    c_loop_ub = (b_tmp_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_tmp_data[k]);
      r1 = _mm_loadu_pd(&sinc_data[k]);
      _mm_storeu_pd(&b_tmp_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_tmp_size; k++) {
      b_tmp_data[k] *= sinc_data[k];
    }
  } else {
    times(b_tmp_data, &b_tmp_size, sinc_data, &modelData_LookTime_size[1]);
  }
  if (vectorUB == modelData_LookTime_size[1]) {
    c_loop_ub = (vectorUB / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&c_tmp_data[k]);
      r1 = _mm_loadu_pd(&sinb_data[k]);
      _mm_storeu_pd(&c_tmp_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < vectorUB; k++) {
      c_tmp_data[k] *= sinb_data[k];
    }
  } else {
    times(c_tmp_data, &vectorUB, sinb_data, &modelData_LookTime_size[1]);
  }
  if ((b_tmp_size != vectorUB) && ((b_tmp_size != 1) && (vectorUB != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_tmp_size, vectorUB, &x_emlrtECI, &g_st);
  }
  if (b_tmp_size == vectorUB) {
    b_loop_ub = b_tmp_size;
    c_loop_ub = (b_tmp_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_tmp_data[k]);
      r1 = _mm_loadu_pd(&c_tmp_data[k]);
      _mm_storeu_pd(&q_b_data[k], _mm_sub_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_tmp_size; k++) {
      q_b_data[k] = b_tmp_data[k] - c_tmp_data[k];
    }
  } else {
    b_loop_ub =
        b_minus(q_b_data, b_tmp_data, &b_tmp_size, c_tmp_data, &vectorUB);
  }
  b_tmp_size = modelData_LookTime_size[1];
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&a_data[k]);
    r1 = _mm_loadu_pd(&c_data[k]);
    _mm_storeu_pd(&b_tmp_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    b_tmp_data[k] = a_data[k] * c_data[k];
  }
  if ((b_tmp_size != modelData_LookTime_size[1]) &&
      ((b_tmp_size != 1) && (modelData_LookTime_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_tmp_size, modelData_LookTime_size[1],
                                &v_emlrtECI, &g_st);
  }
  vectorUB = modelData_LookTime_size[1];
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&b_data[k]);
    r1 = _mm_loadu_pd(&sina_data[k]);
    _mm_storeu_pd(&c_tmp_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    c_tmp_data[k] = b_data[k] * sina_data[k];
  }
  if ((vectorUB != modelData_LookTime_size[1]) &&
      ((vectorUB != 1) && (modelData_LookTime_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(vectorUB, modelData_LookTime_size[1],
                                &w_emlrtECI, &g_st);
  }
  if (b_tmp_size == modelData_LookTime_size[1]) {
    c_loop_ub = (b_tmp_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_tmp_data[k]);
      r1 = _mm_loadu_pd(&sinb_data[k]);
      _mm_storeu_pd(&b_tmp_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_tmp_size; k++) {
      b_tmp_data[k] *= sinb_data[k];
    }
  } else {
    times(b_tmp_data, &b_tmp_size, sinb_data, &modelData_LookTime_size[1]);
  }
  if (vectorUB == modelData_LookTime_size[1]) {
    c_loop_ub = (vectorUB / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&c_tmp_data[k]);
      r1 = _mm_loadu_pd(&sinc_data[k]);
      _mm_storeu_pd(&c_tmp_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < vectorUB; k++) {
      c_tmp_data[k] *= sinc_data[k];
    }
  } else {
    times(c_tmp_data, &vectorUB, sinc_data, &modelData_LookTime_size[1]);
  }
  if ((b_tmp_size != vectorUB) && ((b_tmp_size != 1) && (vectorUB != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_tmp_size, vectorUB, &v_emlrtECI, &g_st);
  }
  if (b_tmp_size == vectorUB) {
    c_loop_ub = (b_tmp_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_tmp_data[k]);
      r1 = _mm_loadu_pd(&c_tmp_data[k]);
      _mm_storeu_pd(&q_c_data[k], _mm_add_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_tmp_size; k++) {
      q_c_data[k] = b_tmp_data[k] + c_tmp_data[k];
    }
  } else {
    b_tmp_size =
        b_plus(q_c_data, b_tmp_data, &b_tmp_size, c_tmp_data, &vectorUB);
  }
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&b_data[k]);
    r1 = _mm_loadu_pd(&c_data[k]);
    _mm_storeu_pd(&b_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    b_data[k] *= c_data[k];
  }
  if ((b_size != modelData_LookTime_size[1]) &&
      ((b_size != 1) && (modelData_LookTime_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_size, modelData_LookTime_size[1], &t_emlrtECI,
                                &g_st);
  }
  c_loop_ub = (modelData_LookTime_size[1] / 2) << 1;
  scalarLB = c_loop_ub - 2;
  for (k = 0; k <= scalarLB; k += 2) {
    r = _mm_loadu_pd(&a_data[k]);
    r1 = _mm_loadu_pd(&sinb_data[k]);
    _mm_storeu_pd(&a_data[k], _mm_mul_pd(r, r1));
  }
  for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
    a_data[k] *= sinb_data[k];
  }
  if ((ypr_size_idx_0 != modelData_LookTime_size[1]) &&
      ((ypr_size_idx_0 != 1) && (modelData_LookTime_size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(ypr_size_idx_0, modelData_LookTime_size[1],
                                &u_emlrtECI, &g_st);
  }
  if (b_size == modelData_LookTime_size[1]) {
    c_loop_ub = (b_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_data[k]);
      r1 = _mm_loadu_pd(&sina_data[k]);
      _mm_storeu_pd(&b_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_size; k++) {
      b_data[k] *= sina_data[k];
    }
  } else {
    times(b_data, &b_size, sina_data, &modelData_LookTime_size[1]);
  }
  if (ypr_size_idx_0 == modelData_LookTime_size[1]) {
    c_loop_ub = (ypr_size_idx_0 / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&a_data[k]);
      r1 = _mm_loadu_pd(&sinc_data[k]);
      _mm_storeu_pd(&a_data[k], _mm_mul_pd(r, r1));
    }
    for (k = c_loop_ub; k < ypr_size_idx_0; k++) {
      a_data[k] *= sinc_data[k];
    }
  } else {
    times(a_data, &ypr_size_idx_0, sinc_data, &modelData_LookTime_size[1]);
  }
  if ((b_size != ypr_size_idx_0) && ((b_size != 1) && (ypr_size_idx_0 != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_size, ypr_size_idx_0, &t_emlrtECI, &g_st);
  }
  if (b_size == ypr_size_idx_0) {
    c_loop_ub = (b_size / 2) << 1;
    scalarLB = c_loop_ub - 2;
    for (k = 0; k <= scalarLB; k += 2) {
      r = _mm_loadu_pd(&b_data[k]);
      r1 = _mm_loadu_pd(&a_data[k]);
      _mm_storeu_pd(&q_d_data[k], _mm_sub_pd(r, r1));
    }
    for (k = c_loop_ub; k < b_size; k++) {
      q_d_data[k] = b_data[k] - a_data[k];
    }
  } else {
    b_size = b_minus(q_d_data, b_data, &b_size, a_data, &ypr_size_idx_0);
  }
  b_st.site = &rq_emlrtRSI;
  quat2rotmat(&b_st, tmp_data, c_tmp_size, q_b_data, b_loop_ub, q_c_data,
              b_tmp_size, q_d_data, b_size, lookRot_data, lookRot_size);
  if (modelData_LookTime_size[1] - 1 >= 0) {
    b_iv[0] = 3;
    b_iv1[0] = 3;
    b_iv[1] = 3;
    b_iv1[1] = 3;
  }
  for (i = 0; i < loop_ub; i++) {
    if (i + 1 > lookRot_size[2]) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, lookRot_size[2], &tg_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &b_iv1[0], 2, &cb_emlrtECI,
                                  (emlrtCTX)sp);
    for (k = 0; k < 3; k++) {
      c_loop_ub = 3 * k + 9 * i;
      beamModel.Orientation[3 * k] = lookRot_data[c_loop_ub];
      beamModel.Orientation[3 * k + 1] = lookRot_data[c_loop_ub + 1];
      beamModel.Orientation[3 * k + 2] = lookRot_data[c_loop_ub + 2];
    }
    st.site = &jr_emlrtRSI;
    b_st.site = &ls_emlrtRSI;
    c_st.site = &ms_emlrtRSI;
    p = true;
    for (k = 0; k < 27; k++) {
      if (p) {
        real_T d;
        d = beamModel.Orientation[k];
        if (muDoubleScalarIsInf(d) || muDoubleScalarIsNaN(d)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (!p) {
      emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
                                    "MATLAB:validators:mustBeFinite",
                                    "MATLAB:validators:mustBeFinite", 0);
    }
    obj->DetectabilityModel.FieldsOfView[i] = beamModel;
  }
}

static void minus(real_T in1_data[], int32_T in1_size[2],
                  const real_T in2_data[], const int32_T in2_size[2],
                  const real_T in3_data[], const int32_T in3_size[2])
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in1_size[0] = 1;
  if (in3_size[1] == 1) {
    loop_ub = in2_size[1];
  } else {
    loop_ub = in3_size[1];
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_1] - in3_data[i * stride_1_1];
  }
}

void c_AerospaceMonostaticRadar_pars(
    const emlrtStack *sp, const real_T sensorData_LookTime_data[],
    const int32_T sensorData_LookTime_size[2],
    const real_T sensorData_LookAzimuth_data[],
    const int32_T sensorData_LookAzimuth_size[2],
    const real_T sensorData_LookElevation_data[],
    const int32_T sensorData_LookElevation_size[2],
    const real_T sensorData_DetectionTime_data[],
    const int32_T sensorData_DetectionTime_size[2],
    const real_T sensorData_Azimuth_data[],
    const int32_T sensorData_Azimuth_size[2],
    const real_T sensorData_Elevation_data[],
    const int32_T sensorData_Elevation_size[2],
    const real_T sensorData_Range_data[],
    const int32_T sensorData_Range_size[2],
    const real_T sensorData_RangeRate_data[],
    const int32_T sensorData_RangeRate_size[2],
    const real_T sensorData_AzimuthAccuracy_data[],
    const int32_T sensorData_AzimuthAccuracy_size[2],
    const real_T c_sensorData_ElevationAccuracy_[],
    const int32_T d_sensorData_ElevationAccuracy_[2],
    const real_T sensorData_RangeAccuracy_data[],
    const int32_T sensorData_RangeAccuracy_size[2],
    const real_T c_sensorData_RangeRateAccuracy_[],
    const int32_T d_sensorData_RangeRateAccuracy_[2], real_T z_data[],
    int32_T z_size[2], b_emxArray_struct_T *modelData)
{
  b_struct_T sampleModelData;
  b_struct_T *modelData_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T elNoise_data[50];
  real_T rrNoise_data[50];
  int32_T i;
  int32_T loop_ub;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  st.site = &ip_emlrtRSI;
  b_st.site = &kp_emlrtRSI;
  c_st.site = &lp_emlrtRSI;
  d_st.site = &mp_emlrtRSI;
  if (sensorData_Elevation_size[1] != sensorData_Azimuth_size[1]) {
    emlrtErrorWithMessageIdR2018a(&d_st, &l_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if (sensorData_Range_size[1] != sensorData_Azimuth_size[1]) {
    emlrtErrorWithMessageIdR2018a(&d_st, &l_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if (sensorData_RangeRate_size[1] != sensorData_Azimuth_size[1]) {
    emlrtErrorWithMessageIdR2018a(&d_st, &l_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  z_size[0] = 4;
  loop_ub = sensorData_Azimuth_size[1];
  z_size[1] = sensorData_Azimuth_size[1];
  for (i = 0; i < loop_ub; i++) {
    z_data[4 * i] = sensorData_Azimuth_data[i];
  }
  loop_ub = sensorData_Elevation_size[1];
  for (i = 0; i < loop_ub; i++) {
    z_data[4 * i + 1] = sensorData_Elevation_data[i];
  }
  loop_ub = sensorData_Range_size[1];
  for (i = 0; i < loop_ub; i++) {
    z_data[4 * i + 2] = sensorData_Range_data[i];
  }
  loop_ub = sensorData_RangeRate_size[1];
  for (i = 0; i < loop_ub; i++) {
    z_data[4 * i + 3] = sensorData_RangeRate_data[i];
  }
  st.site = &jp_emlrtRSI;
  sampleModelData.LookTime.size[0] = 1;
  loop_ub = sensorData_LookTime_size[1];
  sampleModelData.LookTime.size[1] = sensorData_LookTime_size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&sampleModelData.LookTime.data[0], &sensorData_LookTime_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  sampleModelData.LookAzimuth.size[0] = 1;
  loop_ub = sensorData_LookAzimuth_size[1];
  sampleModelData.LookAzimuth.size[1] = sensorData_LookAzimuth_size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&sampleModelData.LookAzimuth.data[0],
           &sensorData_LookAzimuth_data[0], (uint32_T)loop_ub * sizeof(real_T));
  }
  sampleModelData.DetectionTime.size[0] = 1;
  sampleModelData.DetectionTime.size[1] = 0;
  sampleModelData.AzimuthNoise.size[0] = 1;
  sampleModelData.AzimuthNoise.size[1] = 0;
  sampleModelData.ElevationNoise.size[0] = 1;
  sampleModelData.ElevationNoise.size[1] = 0;
  sampleModelData.RangeNoise.size[0] = 1;
  sampleModelData.RangeNoise.size[1] = 0;
  sampleModelData.RangeRateNoise.size[0] = 1;
  sampleModelData.RangeRateNoise.size[1] = 0;
  sampleModelData.LookElevation.size[0] = 1;
  loop_ub = sensorData_LookElevation_size[1];
  sampleModelData.LookElevation.size[1] = sensorData_LookElevation_size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&sampleModelData.LookElevation.data[0],
           &sensorData_LookElevation_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  if (sensorData_DetectionTime_size[1] > 0) {
    __m128d r;
    real_T varargin_1;
    int32_T b_loop_ub;
    int32_T c_loop_ub;
    int32_T d_loop_ub;
    int32_T vectorUB;
    b_st.site = &np_emlrtRSI;
    c_st.site = &sp_emlrtRSI;
    b_loop_ub = sensorData_DetectionTime_size[1];
    loop_ub = modelData->size[0];
    modelData->size[0] = sensorData_DetectionTime_size[1];
    emxEnsureCapacity_struct_T1(&b_st, modelData, loop_ub, &le_emlrtRTEI);
    modelData_data = modelData->data;
    for (i = 0; i < b_loop_ub; i++) {
      modelData_data[i] = sampleModelData;
    }
    b_st.site = &op_emlrtRSI;
    c_st.site = &jd_emlrtRSI;
    c_loop_ub = d_sensorData_ElevationAccuracy_[1];
    loop_ub = c_loop_ub / 2 * 2;
    vectorUB = loop_ub - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&c_sensorData_ElevationAccuracy_[i]);
      r = _mm_mul_pd(r, r);
      _mm_storeu_pd(&elNoise_data[i], r);
    }
    for (i = loop_ub; i < c_loop_ub; i++) {
      varargin_1 = c_sensorData_ElevationAccuracy_[i];
      elNoise_data[i] = varargin_1 * varargin_1;
    }
    b_st.site = &pp_emlrtRSI;
    c_st.site = &jd_emlrtRSI;
    d_loop_ub = d_sensorData_RangeRateAccuracy_[1];
    loop_ub = d_loop_ub / 2 * 2;
    vectorUB = loop_ub - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&c_sensorData_RangeRateAccuracy_[i]);
      r = _mm_mul_pd(r, r);
      _mm_storeu_pd(&rrNoise_data[i], r);
    }
    for (i = loop_ub; i < d_loop_ub; i++) {
      varargin_1 = c_sensorData_RangeRateAccuracy_[i];
      rrNoise_data[i] = varargin_1 * varargin_1;
    }
    for (i = 0; i < b_loop_ub; i++) {
      if (i + 1 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &fc_emlrtBCI, &st);
      }
      modelData_data[i].DetectionTime.size[0] = 1;
      modelData_data[i].DetectionTime.size[1] = 1;
      if (i + 1 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &mc_emlrtBCI, &st);
      }
      modelData_data[i].DetectionTime.data[0] =
          sensorData_DetectionTime_data[i];
      b_st.site = &qp_emlrtRSI;
      if (i + 1 > sensorData_AzimuthAccuracy_size[1]) {
        emlrtDynamicBoundsCheckR2012b(
            i + 1, 1, sensorData_AzimuthAccuracy_size[1], &gc_emlrtBCI, &b_st);
      }
      c_st.site = &id_emlrtRSI;
      if (i + 1 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &hc_emlrtBCI, &st);
      }
      modelData_data[i].AzimuthNoise.size[0] = 1;
      modelData_data[i].AzimuthNoise.size[1] = 1;
      varargin_1 = sensorData_AzimuthAccuracy_data[i];
      modelData_data[i].AzimuthNoise.data[0] = varargin_1 * varargin_1;
      b_st.site = &rp_emlrtRSI;
      if (i + 1 > sensorData_RangeAccuracy_size[1]) {
        emlrtDynamicBoundsCheckR2012b(
            i + 1, 1, sensorData_RangeAccuracy_size[1], &ic_emlrtBCI, &b_st);
      }
      c_st.site = &id_emlrtRSI;
      if (i + 1 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &jc_emlrtBCI, &st);
      }
      modelData_data[i].RangeNoise.size[0] = 1;
      modelData_data[i].RangeNoise.size[1] = 1;
      varargin_1 = sensorData_RangeAccuracy_data[i];
      modelData_data[i].RangeNoise.data[0] = varargin_1 * varargin_1;
      if (i + 1 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &kc_emlrtBCI, &st);
      }
      modelData_data[i].ElevationNoise.size[0] = 1;
      modelData_data[i].ElevationNoise.size[1] = 1;
      if (i + 1 > c_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, c_loop_ub, &nc_emlrtBCI, &st);
      }
      modelData_data[i].ElevationNoise.data[0] = elNoise_data[i];
      if (i + 1 > b_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_loop_ub, &lc_emlrtBCI, &st);
      }
      modelData_data[i].RangeRateNoise.size[0] = 1;
      modelData_data[i].RangeRateNoise.size[1] = 1;
      if (i + 1 > d_loop_ub) {
        emlrtDynamicBoundsCheckR2012b(i + 1, 1, d_loop_ub, &oc_emlrtBCI, &st);
      }
      modelData_data[i].RangeRateNoise.data[0] = rrNoise_data[i];
    }
  } else {
    loop_ub = modelData->size[0];
    modelData->size[0] = 1;
    emxEnsureCapacity_struct_T1(&st, modelData, loop_ub, &le_emlrtRTEI);
    modelData_data = modelData->data;
    modelData_data[0] = sampleModelData;
  }
}

void c_AerospaceMonostaticRadar_time(const real_T in_LookTime_data[],
                                     const int32_T in_LookTime_size[2],
                                     const real_T in_DetectionTime_data[],
                                     const int32_T in_DetectionTime_size[2],
                                     real_T t_data[], int32_T t_size[2])
{
  int32_T b_k;
  int32_T loop_ub;
  t_size[0] = 1;
  loop_ub = in_LookTime_size[1];
  t_size[1] = in_LookTime_size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&t_data[0], &in_LookTime_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  if (in_LookTime_size[1] != 0) {
    real_T b_ex;
    real_T d;
    int32_T idx;
    int32_T k;
    boolean_T exitg1;
    boolean_T guard1;
    guard1 = false;
    if (in_DetectionTime_size[1] != 0) {
      real_T ex;
      int32_T last;
      if (in_LookTime_size[1] <= 2) {
        if (in_LookTime_size[1] == 1) {
          ex = in_LookTime_data[0];
        } else if ((in_LookTime_data[0] < in_LookTime_data[1]) ||
                   (muDoubleScalarIsNaN(in_LookTime_data[0]) &&
                    (!muDoubleScalarIsNaN(in_LookTime_data[1])))) {
          ex = in_LookTime_data[1];
        } else {
          ex = in_LookTime_data[0];
        }
      } else {
        if (!muDoubleScalarIsNaN(in_LookTime_data[0])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k <= loop_ub)) {
            if (!muDoubleScalarIsNaN(in_LookTime_data[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }
        if (idx == 0) {
          ex = in_LookTime_data[0];
        } else {
          ex = in_LookTime_data[idx - 1];
          idx++;
          for (b_k = idx; b_k <= loop_ub; b_k++) {
            b_ex = in_LookTime_data[b_k - 1];
            if (ex < b_ex) {
              ex = b_ex;
            }
          }
        }
      }
      last = in_DetectionTime_size[1];
      if (in_DetectionTime_size[1] <= 2) {
        if (in_DetectionTime_size[1] == 1) {
          b_ex = in_DetectionTime_data[0];
        } else if ((in_DetectionTime_data[0] < in_DetectionTime_data[1]) ||
                   (muDoubleScalarIsNaN(in_DetectionTime_data[0]) &&
                    (!muDoubleScalarIsNaN(in_DetectionTime_data[1])))) {
          b_ex = in_DetectionTime_data[1];
        } else {
          b_ex = in_DetectionTime_data[0];
        }
      } else {
        if (!muDoubleScalarIsNaN(in_DetectionTime_data[0])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k <= last)) {
            if (!muDoubleScalarIsNaN(in_DetectionTime_data[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }
        if (idx == 0) {
          b_ex = in_DetectionTime_data[0];
        } else {
          b_ex = in_DetectionTime_data[idx - 1];
          idx++;
          for (b_k = idx; b_k <= last; b_k++) {
            d = in_DetectionTime_data[b_k - 1];
            if (b_ex < d) {
              b_ex = d;
            }
          }
        }
      }
      if (ex > b_ex) {
        if (in_LookTime_size[1] <= 2) {
          if (in_LookTime_size[1] == 1) {
            b_ex = in_LookTime_data[0];
          } else if ((in_LookTime_data[0] < in_LookTime_data[1]) ||
                     (muDoubleScalarIsNaN(in_LookTime_data[0]) &&
                      (!muDoubleScalarIsNaN(in_LookTime_data[1])))) {
            b_ex = in_LookTime_data[1];
          } else {
            b_ex = in_LookTime_data[0];
          }
        } else {
          if (!muDoubleScalarIsNaN(in_LookTime_data[0])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k <= loop_ub)) {
              if (!muDoubleScalarIsNaN(in_LookTime_data[k - 1])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            b_ex = in_LookTime_data[0];
          } else {
            b_ex = in_LookTime_data[idx - 1];
            k = idx + 1;
            for (b_k = k; b_k <= loop_ub; b_k++) {
              d = in_LookTime_data[b_k - 1];
              if (b_ex < d) {
                b_ex = d;
              }
            }
          }
        }
        t_size[0] = 1;
        t_size[1] = in_DetectionTime_size[1] + 1;
        memcpy(&t_data[0], &in_DetectionTime_data[0],
               (uint32_T)last * sizeof(real_T));
        t_data[in_DetectionTime_size[1]] = b_ex;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      if (in_DetectionTime_size[1] == 0) {
        if (in_LookTime_size[1] <= 2) {
          if (in_LookTime_size[1] == 1) {
            b_ex = in_LookTime_data[0];
          } else if ((in_LookTime_data[0] < in_LookTime_data[1]) ||
                     (muDoubleScalarIsNaN(in_LookTime_data[0]) &&
                      (!muDoubleScalarIsNaN(in_LookTime_data[1])))) {
            b_ex = in_LookTime_data[1];
          } else {
            b_ex = in_LookTime_data[0];
          }
        } else {
          if (!muDoubleScalarIsNaN(in_LookTime_data[0])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k <= loop_ub)) {
              if (!muDoubleScalarIsNaN(in_LookTime_data[k - 1])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            b_ex = in_LookTime_data[0];
          } else {
            b_ex = in_LookTime_data[idx - 1];
            idx++;
            for (b_k = idx; b_k <= loop_ub; b_k++) {
              d = in_LookTime_data[b_k - 1];
              if (b_ex < d) {
                b_ex = d;
              }
            }
          }
        }
        t_size[0] = 1;
        t_size[1] = 1;
        t_data[0] = b_ex;
      } else {
        t_size[0] = 1;
        idx = in_DetectionTime_size[1];
        t_size[1] = in_DetectionTime_size[1];
        memcpy(&t_data[0], &in_DetectionTime_data[0],
               (uint32_T)idx * sizeof(real_T));
      }
    }
  }
}

void c_AerospaceMonostaticRadar_upda(
    const emlrtStack *sp, c_fusion_tracker_sensorspecs_Ae *obj,
    const real_T modelData_LookTime_data[],
    const int32_T modelData_LookTime_size[2],
    const real_T modelData_LookAzimuth_data[],
    const int32_T modelData_LookAzimuth_size[2],
    const real_T modelData_LookElevation_data[],
    const int32_T modelData_LookElevation_size[2],
    const real_T modelData_DetectionTime_data[],
    const int32_T modelData_DetectionTime_size[2],
    const real_T modelData_AzimuthNoise_data[],
    const int32_T modelData_AzimuthNoise_size[2],
    const real_T modelData_ElevationNoise_data[],
    const int32_T modelData_ElevationNoise_size[2],
    const real_T modelData_RangeNoise_data[],
    const int32_T modelData_RangeNoise_size[2],
    const real_T modelData_RangeRateNoise_data[],
    const int32_T modelData_RangeRateNoise_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T x_data[100];
  real_T ypr[3];
  int32_T x_size[2];
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
  st.site = &cq_emlrtRSI;
  if (modelData_DetectionTime_size[1] != 0) {
    __m128d r;
    real_T varargin_1_data[100];
    real_T aasq;
    real_T cosa;
    real_T cosb;
    real_T cosc;
    real_T q_a;
    real_T q_b;
    real_T q_c;
    real_T q_d;
    real_T sina;
    real_T sinb;
    real_T sinc;
    int32_T idx;
    int32_T last;
    boolean_T p;
    b_st.site = &eq_emlrtRSI;
    c_st.site = &eq_emlrtRSI;
    if (modelData_LookTime_size[1] == 1) {
      x_size[1] = 1;
      x_data[0] = modelData_DetectionTime_data[0] - modelData_LookTime_data[0];
    } else {
      minus(x_data, x_size, modelData_DetectionTime_data,
            modelData_DetectionTime_size, modelData_LookTime_data,
            modelData_LookTime_size);
    }
    d_st.site = &lq_emlrtRSI;
    last = x_size[1];
    for (k = 0; k < last; k++) {
      varargin_1_data[k] = muDoubleScalarAbs(x_data[k]);
    }
    c_st.site = &nq_emlrtRSI;
    d_st.site = &oq_emlrtRSI;
    e_st.site = &pq_emlrtRSI;
    if (x_size[1] < 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &xb_emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }
    if (x_size[1] <= 2) {
      if (x_size[1] == 1) {
        idx = 1;
      } else if ((varargin_1_data[0] > varargin_1_data[1]) ||
                 (muDoubleScalarIsNaN(varargin_1_data[0]) &&
                  (!muDoubleScalarIsNaN(varargin_1_data[1])))) {
        idx = 2;
      } else {
        idx = 1;
      }
    } else {
      int32_T b_k;
      if (!muDoubleScalarIsNaN(varargin_1_data[0])) {
        idx = 1;
      } else {
        boolean_T exitg1;
        idx = 0;
        b_k = 2;
        exitg1 = false;
        while ((!exitg1) && (b_k <= last)) {
          if (!muDoubleScalarIsNaN(varargin_1_data[b_k - 1])) {
            idx = b_k;
            exitg1 = true;
          } else {
            b_k++;
          }
        }
      }
      if (idx == 0) {
        idx = 1;
      } else {
        sina = varargin_1_data[idx - 1];
        b_k = idx + 1;
        for (k = b_k; k <= last; k++) {
          aasq = varargin_1_data[k - 1];
          if (sina > aasq) {
            sina = aasq;
            idx = k;
          }
        }
      }
    }
    if (idx > modelData_LookAzimuth_size[1]) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, modelData_LookAzimuth_size[1],
                                    &rg_emlrtBCI, &st);
    }
    if (idx > modelData_LookElevation_size[1]) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, modelData_LookElevation_size[1],
                                    &sg_emlrtBCI, &st);
    }
    ypr[2] = 0.0;
    ypr[0] = modelData_LookAzimuth_data[idx - 1];
    ypr[1] = -modelData_LookElevation_data[idx - 1];
    b_st.site = &fq_emlrtRSI;
    r = _mm_loadu_pd(&ypr[0]);
    _mm_storeu_pd(
        &ypr[0],
        _mm_div_pd(_mm_div_pd(_mm_mul_pd(r, _mm_set1_pd(3.1415926535897931)),
                              _mm_set1_pd(180.0)),
                   _mm_set1_pd(2.0)));
    ypr[2] = ypr[2] * 3.1415926535897931 / 180.0 / 2.0;
    sina = muDoubleScalarSin(ypr[0]);
    sinb = muDoubleScalarSin(ypr[1]);
    sinc = muDoubleScalarSin(ypr[2]);
    cosa = muDoubleScalarCos(ypr[0]);
    cosb = muDoubleScalarCos(ypr[1]);
    cosc = muDoubleScalarCos(ypr[2]);
    aasq = cosa * cosb;
    q_a = aasq * cosc + sina * sinb * sinc;
    q_b = aasq * sinc - cosc * sina * sinb;
    q_c = cosa * cosc * sinb + cosb * sina * sinc;
    q_d = cosb * cosc * sina - cosa * sinb * sinc;
    c_st.site = &rq_emlrtRSI;
    d_st.site = &sq_emlrtRSI;
    sina =
        muDoubleScalarSqrt(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
    q_a /= sina;
    q_b /= sina;
    q_c /= sina;
    q_d /= sina;
    sina = q_a * q_b * 2.0;
    sinb = q_a * q_c * 2.0;
    sinc = q_a * q_d * 2.0;
    cosa = q_b * q_c * 2.0;
    cosb = q_b * q_d * 2.0;
    cosc = q_c * q_d * 2.0;
    aasq = q_a * q_a * 2.0 - 1.0;
    obj->MeasurementModel.Orientation[0] = aasq + q_b * q_b * 2.0;
    obj->MeasurementModel.Orientation[3] = cosa + sinc;
    obj->MeasurementModel.Orientation[6] = cosb - sinb;
    obj->MeasurementModel.Orientation[1] = cosa - sinc;
    obj->MeasurementModel.Orientation[4] = aasq + q_c * q_c * 2.0;
    obj->MeasurementModel.Orientation[7] = cosc + sina;
    obj->MeasurementModel.Orientation[2] = cosb + sinb;
    obj->MeasurementModel.Orientation[5] = cosc - sina;
    obj->MeasurementModel.Orientation[8] = aasq + q_d * q_d * 2.0;
    b_st.site = &gq_emlrtRSI;
    c_st.site = &vq_emlrtRSI;
    d_st.site = &wq_emlrtRSI;
    p = true;
    for (k = 0; k < 27; k++) {
      if (p) {
        sina = obj->MeasurementModel.Orientation[k];
        if (muDoubleScalarIsInf(sina) || muDoubleScalarIsNaN(sina)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (!p) {
      emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI,
                                    "MATLAB:validators:mustBeFinite",
                                    "MATLAB:validators:mustBeFinite", 0);
    }
    b_st.site = &hq_emlrtRSI;
    c_st.site = &xq_emlrtRSI;
    d_st.site = &yq_emlrtRSI;
    sina = validator_check_size(&d_st, modelData_AzimuthNoise_data,
                                modelData_AzimuthNoise_size);
    obj->MeasurementModel.AzimuthVariance = sina;
    d_st.site = &yq_emlrtRSI;
    if (muDoubleScalarIsInf(sina) || muDoubleScalarIsNaN(sina)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI,
                                    "MATLAB:validators:mustBeFinite",
                                    "MATLAB:validators:mustBeFinite", 0);
    }
    b_st.site = &iq_emlrtRSI;
    c_st.site = &cr_emlrtRSI;
    d_st.site = &dr_emlrtRSI;
    sina = validator_check_size(&d_st, modelData_ElevationNoise_data,
                                modelData_ElevationNoise_size);
    obj->MeasurementModel.ElevationVariance = sina;
    d_st.site = &dr_emlrtRSI;
    if (muDoubleScalarIsInf(sina) || muDoubleScalarIsNaN(sina)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI,
                                    "MATLAB:validators:mustBeFinite",
                                    "MATLAB:validators:mustBeFinite", 0);
    }
    b_st.site = &jq_emlrtRSI;
    c_st.site = &er_emlrtRSI;
    d_st.site = &fr_emlrtRSI;
    sina = validator_check_size(&d_st, modelData_RangeNoise_data,
                                modelData_RangeNoise_size);
    obj->MeasurementModel.RangeVariance = sina;
    d_st.site = &fr_emlrtRSI;
    if (muDoubleScalarIsInf(sina) || muDoubleScalarIsNaN(sina)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI,
                                    "MATLAB:validators:mustBeFinite",
                                    "MATLAB:validators:mustBeFinite", 0);
    }
    b_st.site = &kq_emlrtRSI;
    c_st.site = &gr_emlrtRSI;
    d_st.site = &hr_emlrtRSI;
    sina = validator_check_size(&d_st, modelData_RangeRateNoise_data,
                                modelData_RangeRateNoise_size);
    obj->MeasurementModel.RangeRateVariance = sina;
    d_st.site = &hr_emlrtRSI;
    if (muDoubleScalarIsInf(sina) || muDoubleScalarIsNaN(sina)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &s_emlrtRTEI,
                                    "MATLAB:validators:mustBeFinite",
                                    "MATLAB:validators:mustBeFinite", 0);
    }
  }
  st.site = &dq_emlrtRSI;
  d_AerospaceMonostaticRadar_upda(
      &st, obj, modelData_LookTime_size, modelData_LookAzimuth_data,
      modelData_LookAzimuth_size, modelData_LookElevation_data,
      modelData_LookElevation_size);
}

/* End of code generation (AerospaceMonostaticRadar.c) */
