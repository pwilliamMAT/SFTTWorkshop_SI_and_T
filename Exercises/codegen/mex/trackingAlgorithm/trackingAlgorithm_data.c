/*
 * trackingAlgorithm_data.c
 *
 * Code generation for function 'trackingAlgorithm_data'
 *
 */

/* Include files */
#include "trackingAlgorithm_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131675U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "trackingAlgorithm",                                  /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {4273687613U, 2092124783U, 1785127431U, 4270424318U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

emlrtRSInfo i_emlrtRSI =
    {
        1,                                        /* lineNo */
        "SystemProp/clearTunablePropertyChanged", /* fcnName */
        "/MATLAB/toolbox/shared/system/coder/+matlab/+system/+coder/"
        "SystemProp.p" /* pathName */
};

emlrtRSInfo k_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m" /* pathName
                                                                           */
};

emlrtRSInfo l_emlrtRSI =
    {
        1,                                       /* lineNo */
        "SystemProp/matlabCodegenNotifyAnyProp", /* fcnName */
        "/MATLAB/toolbox/shared/system/coder/+matlab/+system/+coder/"
        "SystemProp.p" /* pathName */
};

emlrtRSInfo bd_emlrtRSI = {
    62,                                                          /* lineNo */
    "ceval_xsyheev",                                             /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xsyheev.m" /* pathName */
};

emlrtRSInfo cd_emlrtRSI = {
    20,        /* lineNo */
    "xzlarfg", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarfg.m" /* pathName
                                                                     */
};

emlrtRSInfo dd_emlrtRSI = {
    41,        /* lineNo */
    "xzlarfg", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarfg.m" /* pathName
                                                                     */
};

emlrtRSInfo ed_emlrtRSI = {
    53,        /* lineNo */
    "xzlarfg", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarfg.m" /* pathName
                                                                     */
};

emlrtRSInfo fd_emlrtRSI = {
    68,        /* lineNo */
    "xzlarfg", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarfg.m" /* pathName
                                                                     */
};

emlrtRSInfo gd_emlrtRSI = {
    71,        /* lineNo */
    "xzlarfg", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarfg.m" /* pathName
                                                                     */
};

emlrtRSInfo hd_emlrtRSI = {
    81,        /* lineNo */
    "xzlarfg", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarfg.m" /* pathName
                                                                     */
};

emlrtRSInfo ld_emlrtRSI = {
    31,                                                      /* lineNo */
    "xscal",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xscal.m" /* pathName */
};

emlrtRSInfo md_emlrtRSI = {
    18,                                                         /* lineNo */
    "xscal",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xscal.m" /* pathName */
};

emlrtRSInfo sd_emlrtRSI = {
    44,                                              /* lineNo */
    "mpower",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/mpower.m" /* pathName */
};

emlrtRSInfo td_emlrtRSI = {
    71,                                          /* lineNo */
    "power",                                     /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/power.m" /* pathName */
};

emlrtRSInfo ge_emlrtRSI = {
    93,                                                        /* lineNo */
    "validateattributes",                                      /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/lang/validateattributes.m" /* pathName */
};

emlrtRSInfo he_emlrtRSI = {
    18,                                /* lineNo */
    "isSymmetricPositiveSemiDefinite", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/isSymmetricPositiveSemiDefinite.m" /* pathName */
};

emlrtRSInfo ie_emlrtRSI = {
    20,                                /* lineNo */
    "isSymmetricPositiveSemiDefinite", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/isSymmetricPositiveSemiDefinite.m" /* pathName */
};

emlrtRSInfo je_emlrtRSI = {
    13,                                        /* lineNo */
    "all",                                     /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/all.m" /* pathName */
};

emlrtRSInfo ke_emlrtRSI = {
    143,                                                  /* lineNo */
    "allOrAny",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/allOrAny.m" /* pathName */
};

emlrtRSInfo le_emlrtRSI = {
    81,                                           /* lineNo */
    "eig",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/eig.m" /* pathName */
};

emlrtRSInfo me_emlrtRSI = {
    127,                                          /* lineNo */
    "eig",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/eig.m" /* pathName */
};

emlrtRSInfo ne_emlrtRSI = {
    135,                                          /* lineNo */
    "eig",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/eig.m" /* pathName */
};

emlrtRSInfo oe_emlrtRSI = {
    143,                                          /* lineNo */
    "eig",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/eig.m" /* pathName */
};

emlrtRSInfo pe_emlrtRSI = {
    29,                                                       /* lineNo */
    "anyNonFinite",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/anyNonFinite.m" /* pathName */
};

emlrtRSInfo qe_emlrtRSI = {
    45,                                                    /* lineNo */
    "vAllOrAny",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/vAllOrAny.m" /* pathName */
};

emlrtRSInfo se_emlrtRSI = {
    13,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eigHermitianStandard.m" /* pathName
                                                                            */
};

emlrtRSInfo te_emlrtRSI = {
    40,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eigHermitianStandard.m" /* pathName
                                                                            */
};

emlrtRSInfo ue_emlrtRSI = {
    8,                                                           /* lineNo */
    "xsyheev",                                                   /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xsyheev.m" /* pathName */
};

emlrtRSInfo ve_emlrtRSI =
    {
        10,                         /* lineNo */
        "eigSkewHermitianStandard", /* fcnName */
        "/MATLAB/toolbox/eml/lib/matlab/matfun/private/"
        "eigSkewHermitianStandard.m" /* pathName */
};

emlrtRSInfo we_emlrtRSI = {
    19,                             /* lineNo */
    "eigRealSkewSymmetricStandard", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/"
    "eigRealSkewSymmetricStandard.m" /* pathName */
};

emlrtRSInfo xe_emlrtRSI = {
    35,                                             /* lineNo */
    "schur",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/schur.m" /* pathName */
};

emlrtRSInfo ye_emlrtRSI = {
    52,                                             /* lineNo */
    "schur",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/schur.m" /* pathName */
};

emlrtRSInfo af_emlrtRSI = {
    54,                                             /* lineNo */
    "schur",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/schur.m" /* pathName */
};

emlrtRSInfo bf_emlrtRSI = {
    83,                                             /* lineNo */
    "schur",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/schur.m" /* pathName */
};

emlrtRSInfo cf_emlrtRSI = {
    18,                                                         /* lineNo */
    "xgehrd",                                                   /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xgehrd.m" /* pathName */
};

emlrtRSInfo df_emlrtRSI = {
    46,        /* lineNo */
    "xzgehrd", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgehrd.m" /* pathName
                                                                     */
};

emlrtRSInfo ef_emlrtRSI = {
    50,        /* lineNo */
    "xzgehrd", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgehrd.m" /* pathName
                                                                     */
};

emlrtRSInfo ff_emlrtRSI = {
    58,        /* lineNo */
    "xzgehrd", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgehrd.m" /* pathName
                                                                     */
};

emlrtRSInfo gf_emlrtRSI = {
    84,                                                            /* lineNo */
    "xzlarf",                                                      /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarf.m" /* pathName
                                                                    */
};

emlrtRSInfo hf_emlrtRSI = {
    91,                                                            /* lineNo */
    "xzlarf",                                                      /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarf.m" /* pathName
                                                                    */
};

emlrtRSInfo if_emlrtRSI = {
    86,                                                      /* lineNo */
    "xgemv",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m" /* pathName */
};

emlrtRSInfo jf_emlrtRSI = {
    58,                                                         /* lineNo */
    "xgemv",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xgemv.m" /* pathName */
};

emlrtRSInfo kf_emlrtRSI = {
    37,                                                         /* lineNo */
    "xgemv",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xgemv.m" /* pathName */
};

emlrtRSInfo lf_emlrtRSI = {
    45,                                                      /* lineNo */
    "xgerc",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xgerc.m" /* pathName */
};

emlrtRSInfo mf_emlrtRSI = {
    45,                                                     /* lineNo */
    "xger",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xger.m" /* pathName */
};

emlrtRSInfo nf_emlrtRSI = {
    15,                                                        /* lineNo */
    "xger",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xger.m" /* pathName */
};

emlrtRSInfo of_emlrtRSI = {
    41,                                                         /* lineNo */
    "xgerx",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xgerx.m" /* pathName */
};

emlrtRSInfo pf_emlrtRSI = {
    54,                                                         /* lineNo */
    "xgerx",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xgerx.m" /* pathName */
};

emlrtRSInfo qf_emlrtRSI = {
    50,                                                            /* lineNo */
    "xzlarf",                                                      /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarf.m" /* pathName
                                                                    */
};

emlrtRSInfo rf_emlrtRSI = {
    68,                                                            /* lineNo */
    "xzlarf",                                                      /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarf.m" /* pathName
                                                                    */
};

emlrtRSInfo sf_emlrtRSI = {
    75,                                                            /* lineNo */
    "xzlarf",                                                      /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzlarf.m" /* pathName
                                                                    */
};

emlrtRSInfo uf_emlrtRSI = {
    74,                                                         /* lineNo */
    "xgemv",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xgemv.m" /* pathName */
};

emlrtRSInfo dg_emlrtRSI = {
    243,       /* lineNo */
    "xdlahqr", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xdlahqr.m" /* pathName
                                                                     */
};

emlrtRSInfo qg_emlrtRSI = {
    34,                                                           /* lineNo */
    "eigStandard",                                                /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eigStandard.m" /* pathName */
};

emlrtRSInfo rg_emlrtRSI = {
    45,                                                           /* lineNo */
    "eigStandard",                                                /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eigStandard.m" /* pathName */
};

emlrtRSInfo nh_emlrtRSI = {
    30,                                                      /* lineNo */
    "xswap",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xswap.m" /* pathName */
};

emlrtRSInfo oh_emlrtRSI = {
    20,                                                         /* lineNo */
    "xswap",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/xswap.m" /* pathName */
};

emlrtRSInfo ph_emlrtRSI = {
    23,                                                       /* lineNo */
    "ixamax",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/ixamax.m" /* pathName */
};

emlrtRSInfo qh_emlrtRSI = {
    24,                                                          /* lineNo */
    "ixamax",                                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+refblas/ixamax.m" /* pathName */
};

emlrtRSInfo gl_emlrtRSI = {
    16, /* lineNo */
    "JIPDATrackMaintainer/ConfirmationThreshold (generated property set "
    "method)", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pathName */
};

emlrtRSInfo hl_emlrtRSI = {
    16, /* lineNo */
    "JIPDATrackMaintainer/ConfirmationThreshold (property validation)", /* fcnName
                                                                         */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pathName */
};

emlrtRSInfo il_emlrtRSI = {
    8,                                                           /* lineNo */
    "mustBeLessThan",                                            /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/validators/mustBeLessThan.m" /* pathName */
};

emlrtRSInfo jl_emlrtRSI =
    {
        17, /* lineNo */
        "JIPDATrackMaintainer/DeletionThreshold (generated property set "
        "method)", /* fcnName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
        "+components/JIPDATrackMaintainer.m" /* pathName */
};

emlrtRSInfo kl_emlrtRSI = {
    17,                                                             /* lineNo */
    "JIPDATrackMaintainer/DeletionThreshold (property validation)", /* fcnName
                                                                     */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackMaintainer.m" /* pathName */
};

emlrtRSInfo pn_emlrtRSI = {
    15,                                            /* lineNo */
    "max",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/max.m" /* pathName */
};

emlrtRSInfo qn_emlrtRSI = {
    73,                                                   /* lineNo */
    "minOrMax",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName */
};

emlrtRSInfo rn_emlrtRSI = {
    108,                                                  /* lineNo */
    "maximum",                                            /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName */
};

emlrtRSInfo un_emlrtRSI = {
    73,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/vectorMinOrMaxInPlace.m" /* pathName
                                                                        */
};

emlrtRSInfo vn_emlrtRSI = {
    65,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/vectorMinOrMaxInPlace.m" /* pathName
                                                                        */
};

emlrtRSInfo wn_emlrtRSI = {
    114,         /* lineNo */
    "findFirst", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/vectorMinOrMaxInPlace.m" /* pathName
                                                                        */
};

emlrtRSInfo xn_emlrtRSI = {
    131,                        /* lineNo */
    "minOrMaxRealVectorKernel", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/vectorMinOrMaxInPlace.m" /* pathName
                                                                        */
};

emlrtRSInfo ao_emlrtRSI = {
    37,                                             /* lineNo */
    "sort",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sort.m" /* pathName */
};

emlrtRSInfo aq_emlrtRSI = {
    39,                                              /* lineNo */
    "cat",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/cat.m" /* pathName */
};

emlrtRSInfo bq_emlrtRSI = {
    65,                                              /* lineNo */
    "cat_impl",                                      /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/cat.m" /* pathName */
};

emlrtRSInfo hq_emlrtRSI = {
    34,                                             /* lineNo */
    "repmat",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/repmat.m" /* pathName */
};

emlrtRSInfo mq_emlrtRSI = {
    142,                                   /* lineNo */
    "TrackEstimator/updateEstimatorModel", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

emlrtRSInfo nq_emlrtRSI = {
    207,                                  /* lineNo */
    "IPDAEstimator/updateEstimatorModel", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

emlrtRSInfo oq_emlrtRSI = {
    208,                                  /* lineNo */
    "IPDAEstimator/updateEstimatorModel", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

emlrtRSInfo pq_emlrtRSI = {
    297,                                        /* lineNo */
    "MultiModalEstimator/updateEstimatorModel", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

emlrtRSInfo qq_emlrtRSI = {
    230,                                      /* lineNo */
    "EKFStateEstimator/updateEstimatorModel", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

emlrtRSInfo ir_emlrtRSI =
    {
        101,           /* lineNo */
        "quat2rotmat", /* fcnName */
        "/MATLAB/toolbox/shared/radarfusion/+fusion/+internal/+frames/"
        "quat2rotmat.m" /* pathName */
};

emlrtRSInfo
    ns_emlrtRSI =
        {
            44,                           /* lineNo */
            "applyScalarFunctionInPlace", /* fcnName */
            "/MATLAB/toolbox/eml/eml/+coder/+internal/"
            "applyScalarFunctionInPlace.m" /* pathName */
};

emlrtRSInfo ys_emlrtRSI = {
    52,                                                            /* lineNo */
    "reshapeSizeChecks",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m" /* pathName
                                                                    */
};

emlrtRSInfo dt_emlrtRSI = {
    91,                                        /* lineNo */
    "ExistenceEstimator/updateEstimatorModel", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "ExistenceEstimator.m" /* pathName */
};

emlrtRSInfo tt_emlrtRSI = {
    93,                      /* lineNo */
    "IPDAEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

emlrtRSInfo ut_emlrtRSI = {
    92,                      /* lineNo */
    "IPDAEstimator/predict", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "IPDAEstimator.m" /* pathName */
};

emlrtRSInfo tu_emlrtRSI = {
    1906,                                                          /* lineNo */
    "ExtendedKalmanFilter/ensureStateAndStateCovarianceIsDefined", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/ExtendedKalmanFilter.m" /* pathName */
};

emlrtRSInfo pv_emlrtRSI = {
    20,         /* lineNo */
    "qrFactor", /* fcnName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/qrFactor.m" /* pathName */
};

emlrtRSInfo qv_emlrtRSI = {
    24,                                          /* lineNo */
    "qr",                                        /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/qr.m" /* pathName */
};

emlrtRSInfo rv_emlrtRSI = {
    35,                                                      /* lineNo */
    "eml_qr",                                                /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eml_qr.m" /* pathName */
};

emlrtRSInfo sv_emlrtRSI = {
    166,                                                     /* lineNo */
    "qr_econ",                                               /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eml_qr.m" /* pathName */
};

emlrtRSInfo tv_emlrtRSI = {
    175,                                                     /* lineNo */
    "qr_econ",                                               /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eml_qr.m" /* pathName */
};

emlrtRSInfo uv_emlrtRSI = {
    187,                                                     /* lineNo */
    "qr_econ",                                               /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eml_qr.m" /* pathName */
};

emlrtRSInfo vv_emlrtRSI = {
    30,                                                         /* lineNo */
    "xgeqrf",                                                   /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m" /* pathName */
};

emlrtRSInfo wv_emlrtRSI = {
    116,       /* lineNo */
    "xzgeqp3", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgeqp3.m" /* pathName
                                                                     */
};

emlrtRSInfo xv_emlrtRSI = {
    160,   /* lineNo */
    "qrf", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgeqp3.m" /* pathName
                                                                     */
};

emlrtRSInfo yv_emlrtRSI = {
    146,   /* lineNo */
    "qrf", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgeqp3.m" /* pathName
                                                                     */
};

emlrtRSInfo aw_emlrtRSI = {
    202,                                                     /* lineNo */
    "unpackQR",                                              /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/matfun/private/eml_qr.m" /* pathName */
};

emlrtRSInfo bw_emlrtRSI = {
    17,                                                         /* lineNo */
    "xorgqr",                                                   /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m" /* pathName */
};

emlrtRSInfo cw_emlrtRSI = {
    46,        /* lineNo */
    "xzungqr", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzungqr.m" /* pathName
                                                                     */
};

emlrtRSInfo dw_emlrtRSI = {
    41,        /* lineNo */
    "xzungqr", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzungqr.m" /* pathName
                                                                     */
};

emlrtRSInfo ew_emlrtRSI = {
    34,        /* lineNo */
    "xzungqr", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzungqr.m" /* pathName
                                                                     */
};

emlrtRSInfo lw_emlrtRSI = {
    17,          /* lineNo */
    "logsumexp", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+internal/logsumexp.m" /* pathName
                                                                        */
};

emlrtRSInfo mw_emlrtRSI = {
    10,                                          /* lineNo */
    "exp",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elfun/exp.m" /* pathName */
};

emlrtRSInfo nw_emlrtRSI = {
    20,                                            /* lineNo */
    "sum",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sum.m" /* pathName */
};

emlrtRSInfo ow_emlrtRSI = {
    99,                                                        /* lineNo */
    "sumprod",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/sumprod.m" /* pathName */
};

emlrtRSInfo
    pw_emlrtRSI =
        {
            86,                      /* lineNo */
            "combineVectorElements", /* fcnName */
            "/MATLAB/toolbox/eml/lib/matlab/datafun/private/"
            "combineVectorElements.m" /* pathName */
};

emlrtRSInfo rw_emlrtRSI = {
    22,                    /* lineNo */
    "sumMatrixIncludeNaN", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/sumMatrixIncludeNaN.m" /* pathName
                                                                            */
};

emlrtRSInfo sw_emlrtRSI = {
    42,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/private/sumMatrixIncludeNaN.m" /* pathName
                                                                            */
};

emlrtRSInfo ax_emlrtRSI =
    {
        20, /* lineNo */
        "ExistenceEstimator/SurvivalProbability (generated property set "
        "method)", /* fcnName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
        "+estimators/ExistenceEstimator.m" /* pathName */
};

emlrtRSInfo bx_emlrtRSI = {
    20,                                                             /* lineNo */
    "ExistenceEstimator/SurvivalProbability (property validation)", /* fcnName
                                                                     */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "ExistenceEstimator.m" /* pathName */
};

emlrtRSInfo sx_emlrtRSI = {
    115,                                        /* lineNo */
    "MultiModalEstimator/detectionProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

emlrtRSInfo tx_emlrtRSI = {
    118,                                        /* lineNo */
    "MultiModalEstimator/detectionProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

emlrtRSInfo ux_emlrtRSI = {
    119,                                        /* lineNo */
    "MultiModalEstimator/detectionProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "MultiModalEstimator.m" /* pathName */
};

emlrtRSInfo vx_emlrtRSI = {
    80,                                       /* lineNo */
    "EKFStateEstimator/detectionProbability", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "EKFStateEstimator.m" /* pathName */
};

emlrtRSInfo cy_emlrtRSI =
    {
        19, /* lineNo */
        "ExistenceEstimator/DetectionProbability (generated property set "
        "method)", /* fcnName */
        "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/"
        "+estimators/ExistenceEstimator.m" /* pathName */
};

emlrtRSInfo dy_emlrtRSI = {
    19, /* lineNo */
    "ExistenceEstimator/DetectionProbability (property validation)", /* fcnName
                                                                      */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "ExistenceEstimator.m" /* pathName */
};

emlrtRSInfo mbb_emlrtRSI = {
    20,                                                          /* lineNo */
    "mrdivide_helper",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/mrdivide_helper.m" /* pathName */
};

emlrtRSInfo nbb_emlrtRSI = {
    42,                                                          /* lineNo */
    "mrdiv",                                                     /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/mrdivide_helper.m" /* pathName */
};

emlrtRSInfo obb_emlrtRSI = {
    67,                                                  /* lineNo */
    "lusolve",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName */
};

emlrtRSInfo pbb_emlrtRSI = {
    107,                                                 /* lineNo */
    "lusolveNxN",                                        /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName */
};

emlrtRSInfo qbb_emlrtRSI = {
    112,                                                 /* lineNo */
    "lusolveNxN",                                        /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName */
};

emlrtRSInfo rbb_emlrtRSI = {
    135,                                                 /* lineNo */
    "XtimesInvA",                                        /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName */
};

emlrtRSInfo sbb_emlrtRSI = {
    140,                                                 /* lineNo */
    "XtimesInvA",                                        /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName */
};

emlrtRSInfo tbb_emlrtRSI = {
    36,                                                         /* lineNo */
    "xgetrf",                                                   /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/xgetrf.m" /* pathName */
};

emlrtRSInfo ubb_emlrtRSI = {
    41,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgetrf.m" /* pathName
                                                                     */
};

emlrtRSInfo vbb_emlrtRSI = {
    55,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgetrf.m" /* pathName
                                                                     */
};

emlrtRSInfo wbb_emlrtRSI = {
    63,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+reflapack/xzgetrf.m" /* pathName
                                                                     */
};

emlrtRSInfo xbb_emlrtRSI = {
    45,                                                      /* lineNo */
    "xgeru",                                                 /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+blas/xgeru.m" /* pathName */
};

emlrtRSInfo bcb_emlrtRSI = {
    90,                                                  /* lineNo */
    "warn_singular",                                     /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName */
};

emlrtRSInfo gdb_emlrtRSI = {
    15,                                            /* lineNo */
    "sum",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sum.m" /* pathName */
};

emlrtRSInfo
    hdb_emlrtRSI =
        {
            149,                     /* lineNo */
            "combineVectorElements", /* fcnName */
            "/MATLAB/toolbox/eml/lib/matlab/datafun/private/"
            "combineVectorElements.m" /* pathName */
};

emlrtRSInfo
    idb_emlrtRSI =
        {
            209,                /* lineNo */
            "colMajorFlatIter", /* fcnName */
            "/MATLAB/toolbox/eml/lib/matlab/datafun/private/"
            "combineVectorElements.m" /* pathName */
};

emlrtRSInfo jdb_emlrtRSI = {
    28,                                          /* lineNo */
    "colon",                                     /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

emlrtRSInfo kdb_emlrtRSI = {
    93,                                          /* lineNo */
    "colon",                                     /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/ops/colon.m" /* pathName */
};

emlrtRSInfo ndb_emlrtRSI = {
    39,                                           /* lineNo */
    "find",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pathName */
};

emlrtRSInfo ffb_emlrtRSI = {
    17,                                          /* lineNo */
    "log",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elfun/log.m" /* pathName */
};

emlrtRSInfo rhb_emlrtRSI = {
    15,                                            /* lineNo */
    "min",                                         /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/min.m" /* pathName */
};

emlrtRSInfo shb_emlrtRSI = {
    75,                                                   /* lineNo */
    "minOrMax",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName */
};

emlrtRSInfo thb_emlrtRSI = {
    121,                                                  /* lineNo */
    "minimum",                                            /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/minOrMax.m" /* pathName */
};

emlrtRSInfo uhb_emlrtRSI = {
    273,                                                       /* lineNo */
    "unaryMinOrMax",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/unaryMinOrMax.m" /* pathName */
};

emlrtRSInfo vhb_emlrtRSI = {
    962,                                                       /* lineNo */
    "minRealVectorOmitNaN",                                    /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/unaryMinOrMax.m" /* pathName */
};

emlrtRSInfo fib_emlrtRSI = {
    18,                                              /* lineNo */
    "sub2ind",                                       /* fcnName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/sub2ind.m" /* pathName */
};

emlrtRSInfo mkb_emlrtRSI = {
    41,                                              /* lineNo */
    "cat",                                           /* fcnName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/cat.m" /* pathName */
};

emlrtMCInfo d_emlrtMCI = {
    53,        /* lineNo */
    19,        /* colNo */
    "flt2str", /* fName */
    "/MATLAB/toolbox/shared/coder/coder/lib/+coder/+internal/flt2str.m" /* pName
                                                                         */
};

omp_lock_t emlrtLockGlobal;

omp_nest_lock_t trackingAlgorithm_nestLockGlobal;

emlrtRTEInfo b_emlrtRTEI = {
    12,                                                          /* lineNo */
    23,                                                          /* colNo */
    "mustBePositive",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/validators/mustBePositive.m" /* pName */
};

emlrtRTEInfo c_emlrtRTEI =
    {
        21,                     /* lineNo */
        9,                      /* colNo */
        "binaryRelopValidator", /* fName */
        "/MATLAB/toolbox/eml/lib/matlab/validators/private/"
        "binaryRelopValidator.m" /* pName */
};

emlrtRTEInfo d_emlrtRTEI = {
    13,                                           /* lineNo */
    9,                                            /* colNo */
    "sqrt",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elfun/sqrt.m" /* pName */
};

emlrtRTEInfo f_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatefinite", /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+valattr/validatefinite.m" /* pName
                                                                          */
};

emlrtRTEInfo g_emlrtRTEI = {
    24,                                /* lineNo */
    5,                                 /* colNo */
    "isSymmetricPositiveSemiDefinite", /* fName */
    "/MATLAB/toolbox/shared/tracking/trackinglib/+matlabshared/+tracking/"
    "+internal/isSymmetricPositiveSemiDefinite.m" /* pName */
};

emlrtRTEInfo h_emlrtRTEI = {
    45,                                                            /* lineNo */
    13,                                                            /* colNo */
    "infocheck",                                                   /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/infocheck.m" /* pName */
};

emlrtRTEInfo i_emlrtRTEI = {
    48,                                                            /* lineNo */
    13,                                                            /* colNo */
    "infocheck",                                                   /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+lapack/infocheck.m" /* pName */
};

emlrtBCInfo y_emlrtBCI = {
    -1,                                            /* iFirst */
    -1,                                            /* iLast */
    300,                                           /* lineNo */
    42,                                            /* colNo */
    "",                                            /* aName */
    "AerospaceMonostaticRadar/selectMeasurements", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+sensorspecs/"
    "AerospaceMonostaticRadar.m", /* pName */
    0                             /* checkKind */
};

emlrtRTEInfo l_emlrtRTEI = {
    11,                                                             /* lineNo */
    24,                                                             /* colNo */
    "mustBeNonnegative",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/validators/mustBeNonnegative.m" /* pName */
};

emlrtRTEInfo m_emlrtRTEI = {
    225,                                             /* lineNo */
    27,                                              /* colNo */
    "check_non_axis_size",                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/cat.m" /* pName */
};

emlrtRTEInfo p_emlrtRTEI = {
    81,                                                            /* lineNo */
    13,                                                            /* colNo */
    "reshapeSizeChecks",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m" /* pName */
};

emlrtRTEInfo q_emlrtRTEI = {
    88,                                                            /* lineNo */
    23,                                                            /* colNo */
    "reshapeSizeChecks",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m" /* pName */
};

emlrtRTEInfo t_emlrtRTEI = {
    9,                                                         /* lineNo */
    23,                                                        /* colNo */
    "mustBeFinite",                                            /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/validators/mustBeFinite.m" /* pName */
};

emlrtRTEInfo v_emlrtRTEI = {
    14,                                          /* lineNo */
    9,                                           /* colNo */
    "log",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elfun/log.m" /* pName */
};

emlrtRTEInfo x_emlrtRTEI = {
    198,                                                       /* lineNo */
    27,                                                        /* colNo */
    "unaryMinOrMax",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/unaryMinOrMax.m" /* pName */
};

emlrtRTEInfo nb_emlrtRTEI =
    {
        14,                    /* lineNo */
        37,                    /* colNo */
        "validatenonnegative", /* fName */
        "/MATLAB/toolbox/eml/eml/+coder/+internal/+valattr/"
        "validatenonnegative.m" /* pName */
};

emlrtRTEInfo pb_emlrtRTEI = {
    13,                /* lineNo */
    37,                /* colNo */
    "validateinteger", /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/+valattr/validateinteger.m" /* pName
                                                                           */
};

emlrtRTEInfo ub_emlrtRTEI = {
    28,                                                  /* lineNo */
    19,                                                  /* colNo */
    "sub2ind",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sub2ind.m" /* pName */
};

emlrtRTEInfo vb_emlrtRTEI = {
    18,                                                  /* lineNo */
    23,                                                  /* colNo */
    "sub2ind",                                           /* fName */
    "/MATLAB/toolbox/eml/eml/+coder/+internal/sub2ind.m" /* pName */
};

emlrtDCInfo g_emlrtDCI = {
    37,                                              /* lineNo */
    14,                                              /* colNo */
    "repmat",                                        /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/repmat.m", /* pName */
    4                                                /* checkKind */
};

emlrtRTEInfo ie_emlrtRTEI = {
    31,                                           /* lineNo */
    6,                                            /* colNo */
    "find",                                       /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/elmat/find.m" /* pName */
};

emlrtRTEInfo mf_emlrtRTEI = {
    38,                                             /* lineNo */
    5,                                              /* colNo */
    "sort",                                         /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sort.m" /* pName */
};

emlrtRTEInfo ag_emlrtRTEI = {
    15,                                            /* lineNo */
    9,                                             /* colNo */
    "sum",                                         /* fName */
    "/MATLAB/toolbox/eml/lib/matlab/datafun/sum.m" /* pName */
};

emlrtRTEInfo
    bh_emlrtRTEI =
        {
            181,                     /* lineNo */
            24,                      /* colNo */
            "combineVectorElements", /* fName */
            "/MATLAB/toolbox/eml/lib/matlab/datafun/private/"
            "combineVectorElements.m" /* pName */
};

const int8_T iv[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                       0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};

const int8_T iv1[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

const char_T cv[13] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                       '_', 'd', 's', 'y', 'e', 'v'};

const char_T cv1[10] = {'I', 'n', 't', 'e', 'g', 'r', 'a', 't', 'e', 'd'};

emlrtRSInfo bpb_emlrtRSI = {
    53,        /* lineNo */
    "flt2str", /* fcnName */
    "/MATLAB/toolbox/shared/coder/coder/lib/+coder/+internal/flt2str.m" /* pathName
                                                                         */
};

/* End of code generation (trackingAlgorithm_data.c) */
