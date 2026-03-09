/*
 * fusionAlgorithm_data.c
 *
 * Code generation for function 'fusionAlgorithm_data'
 *
 */

/* Include files */
#include "fusionAlgorithm_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131675U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "fusionAlgorithm",                                    /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {3264196028U, 2617517587U, 3094936123U, 1834023453U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

emlrtRSInfo e_emlrtRSI = {
    88,                                                  /* lineNo */
    "fuserSourceConfiguration/fuserSourceConfiguration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo f_emlrtRSI = {
    101,                                                 /* lineNo */
    "fuserSourceConfiguration/fuserSourceConfiguration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo g_emlrtRSI = {
    110,                                                 /* lineNo */
    "fuserSourceConfiguration/fuserSourceConfiguration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo p_emlrtRSI = {
    1,                           /* lineNo */
    "SystemCore/parenReference", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pathName */
};

emlrtRSInfo gb_emlrtRSI = {
    93,                   /* lineNo */
    "validateattributes", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\validateattributes"
    ".m" /* pathName */
};

emlrtRSInfo ib_emlrtRSI = {
    48,       /* lineNo */
    "unique", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

emlrtRSInfo tb_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pathName */
};

emlrtRSInfo vb_emlrtRSI = {
    1,                   /* lineNo */
    "SystemCore/delete", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pathName */
};

emlrtRSInfo qc_emlrtRSI =
    {
        277,                          /* lineNo */
        "objectTrack/set.UpdateTime", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

emlrtRSInfo rc_emlrtRSI =
    {
        287,                     /* lineNo */
        "objectTrack/set.State", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

emlrtRSInfo sc_emlrtRSI =
    {
        293,                               /* lineNo */
        "objectTrack/set.StateCovariance", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

emlrtRSInfo tc_emlrtRSI =
    {
        295,                               /* lineNo */
        "objectTrack/set.StateCovariance", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

emlrtRSInfo uc_emlrtRSI = {
    18,                                /* lineNo */
    "isSymmetricPositiveSemiDefinite", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\isSymmetricPosit"
    "iveSemiDefinite.m" /* pathName */
};

emlrtRSInfo vc_emlrtRSI = {
    20,                                /* lineNo */
    "isSymmetricPositiveSemiDefinite", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\isSymmetricPosit"
    "iveSemiDefinite.m" /* pathName */
};

emlrtRSInfo xc_emlrtRSI =
    {
        143,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

emlrtRSInfo yc_emlrtRSI = {
    81,    /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

emlrtRSInfo dd_emlrtRSI = {
    29,             /* lineNo */
    "anyNonFinite", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\anyNonFinite."
    "m" /* pathName */
};

emlrtRSInfo ed_emlrtRSI =
    {
        45,          /* lineNo */
        "vAllOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\vAllOrAny.m" /* pathName */
};

emlrtRSInfo fd_emlrtRSI =
    {
        121,                  /* lineNo */
        "flatVectorAllOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\vAllOrAny.m" /* pathName */
};

emlrtRSInfo hd_emlrtRSI = {
    40,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

emlrtRSInfo jd_emlrtRSI = {
    62,              /* lineNo */
    "ceval_xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

emlrtRSInfo kd_emlrtRSI = {
    20,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

emlrtRSInfo ld_emlrtRSI = {
    41,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

emlrtRSInfo md_emlrtRSI = {
    53,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

emlrtRSInfo nd_emlrtRSI = {
    68,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

emlrtRSInfo od_emlrtRSI = {
    71,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

emlrtRSInfo pd_emlrtRSI = {
    81,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

emlrtRSInfo sd_emlrtRSI = {
    31,      /* lineNo */
    "xscal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xscal."
    "m" /* pathName */
};

emlrtRSInfo td_emlrtRSI = {
    18,      /* lineNo */
    "xscal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xscal.m" /* pathName */
};

emlrtRSInfo yd_emlrtRSI = {
    35,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

emlrtRSInfo ce_emlrtRSI = {
    83,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

emlrtRSInfo de_emlrtRSI = {
    18,       /* lineNo */
    "xgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgehrd.m" /* pathName */
};

emlrtRSInfo je_emlrtRSI = {
    86,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xgemv."
    "m" /* pathName */
};

emlrtRSInfo le_emlrtRSI = {
    37,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemv.m" /* pathName */
};

emlrtRSInfo me_emlrtRSI = {
    45,      /* lineNo */
    "xgerc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xgerc."
    "m" /* pathName */
};

emlrtRSInfo ne_emlrtRSI =
    {
        45,     /* lineNo */
        "xger", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
        "blas\\xger.m" /* pathName */
};

emlrtRSInfo oe_emlrtRSI = {
    15,     /* lineNo */
    "xger", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xger.m" /* pathName */
};

emlrtRSInfo pe_emlrtRSI = {
    41,      /* lineNo */
    "xgerx", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgerx.m" /* pathName */
};

emlrtRSInfo qe_emlrtRSI = {
    54,      /* lineNo */
    "xgerx", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgerx.m" /* pathName */
};

emlrtRSInfo cf_emlrtRSI = {
    243,       /* lineNo */
    "xdlahqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdlahqr.m" /* pathName */
};

emlrtRSInfo gf_emlrtRSI = {
    342,       /* lineNo */
    "xdlahqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdlahqr.m" /* pathName */
};

emlrtRSInfo hf_emlrtRSI = {
    345,       /* lineNo */
    "xdlahqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdlahqr.m" /* pathName */
};

emlrtRSInfo sf_emlrtRSI = {
    45,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

emlrtRSInfo tf_emlrtRSI = {
    159,           /* lineNo */
    "ceval_xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

emlrtRSInfo bg_emlrtRSI = {
    99,        /* lineNo */
    "sumprod", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumpro"
    "d.m" /* pathName */
};

emlrtRSInfo ig_emlrtRSI =
    {
        350,                             /* lineNo */
        "objectTrack/set.ObjectClassID", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

emlrtRSInfo
    jg_emlrtRSI =
        {
            91,       /* lineNo */
            "strcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

emlrtRSInfo
    kg_emlrtRSI =
        {
            167,          /* lineNo */
            "loc_strcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

emlrtRSInfo
    lg_emlrtRSI =
        {
            240,       /* lineNo */
            "charcmp", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\strcmp.m" /* pathName */
};

emlrtRSInfo qg_emlrtRSI =
    {
        18,            /* lineNo */
        "ifWhileCond", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\ifWhileCond.m" /* pathName */
};

emlrtRSInfo rg_emlrtRSI =
    {
        31,            /* lineNo */
        "checkNoNaNs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\ifWhileCond.m" /* pathName */
};

emlrtRSInfo ug_emlrtRSI = {
    538,                             /* lineNo */
    "FuserManager/getConfigByTrack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

emlrtRSInfo vg_emlrtRSI = {
    124,                                           /* lineNo */
    "fuserSourceConfiguration/transformToCentral", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo wg_emlrtRSI = {
    126,                                           /* lineNo */
    "fuserSourceConfiguration/transformToCentral", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo xg_emlrtRSI = {
    267,                                                   /* lineNo */
    "fuserSourceConfiguration/validateTransformToCentral", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo yg_emlrtRSI = {
    275,                                                   /* lineNo */
    "fuserSourceConfiguration/validateTransformToCentral", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo ah_emlrtRSI = {
    53,              /* lineNo */
    "local2central", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

emlrtRSInfo bh_emlrtRSI = {
    55,              /* lineNo */
    "local2central", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

emlrtRSInfo ph_emlrtRSI =
    {
        262,                       /* lineNo */
        "objectTrack/set.TrackID", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

emlrtRSInfo fi_emlrtRSI = {
    67,              /* lineNo */
    "central2local", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

emlrtRSInfo gi_emlrtRSI = {
    69,              /* lineNo */
    "central2local", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

emlrtRSInfo hi_emlrtRSI = {
    17,                      /* lineNo */
    "gaussEKFilter/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m" /* pathName */
};

emlrtRSInfo ii_emlrtRSI = {
    30,                      /* lineNo */
    "gaussEKFilter/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m" /* pathName */
};

emlrtRSInfo ji_emlrtRSI = {
    31,                      /* lineNo */
    "gaussEKFilter/predict", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussEKFilter.m" /* pathName */
};

emlrtRSInfo ki_emlrtRSI =
    {
        98,         /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

emlrtRSInfo li_emlrtRSI = {
    69,                /* lineNo */
    "numericJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\numericJacobian."
    "m" /* pathName */
};

emlrtRSInfo mi_emlrtRSI = {
    79,                /* lineNo */
    "numericJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\numericJacobian."
    "m" /* pathName */
};

emlrtRSInfo ni_emlrtRSI =
    {
        142,        /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

emlrtRSInfo oi_emlrtRSI =
    {
        159,        /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

emlrtRSInfo pi_emlrtRSI =
    {
        165,        /* lineNo */
        "constvel", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pathName */
};

emlrtRSInfo qi_emlrtRSI = {
    622,                                 /* lineNo */
    "FuserManager/validateSourceConfig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

emlrtRSInfo ri_emlrtRSI = {
    623,                                 /* lineNo */
    "FuserManager/validateSourceConfig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

emlrtRSInfo jk_emlrtRSI = {
    34,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

emlrtRSInfo mk_emlrtRSI = {
    78,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

emlrtRSInfo nk_emlrtRSI = {
    136,                                         /* lineNo */
    "fuserSourceConfiguration/transformToLocal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo ok_emlrtRSI = {
    138,                                         /* lineNo */
    "fuserSourceConfiguration/transformToLocal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo pk_emlrtRSI = {
    293,                                                 /* lineNo */
    "fuserSourceConfiguration/validateTransformToLocal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo qk_emlrtRSI = {
    301,                                                 /* lineNo */
    "fuserSourceConfiguration/validateTransformToLocal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

emlrtRSInfo sk_emlrtRSI = {
    33,              /* lineNo */
    "gaussNormDiff", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pathName */
};

emlrtRSInfo tk_emlrtRSI = {
    44,           /* lineNo */
    "calcOneRow", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\gaussNormDiff.m" /* pathName */
};

emlrtRSInfo uk_emlrtRSI = {
    20,                /* lineNo */
    "mrdivide_helper", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\mrdivide_"
    "helper.m" /* pathName */
};

emlrtRSInfo
    wk_emlrtRSI =
        {
            67,        /* lineNo */
            "lusolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

emlrtRSInfo
    yk_emlrtRSI =
        {
            112,          /* lineNo */
            "lusolveNxN", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

emlrtRSInfo cl_emlrtRSI = {
    30,       /* lineNo */
    "xgetrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrf.m" /* pathName */
};

emlrtRSInfo
    jl_emlrtRSI =
        {
            90,              /* lineNo */
            "warn_singular", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

emlrtRSInfo kl_emlrtRSI = {
    21,    /* lineNo */
    "det", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\det.m" /* pathName
                                                                       */
};

emlrtRSInfo yl_emlrtRSI = {
    39,     /* lineNo */
    "find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

emlrtRSInfo am_emlrtRSI = {
    138,        /* lineNo */
    "eml_find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

emlrtRSInfo bm_emlrtRSI = {
    376,                  /* lineNo */
    "find_first_indices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

emlrtRSInfo br_emlrtRSI = {
    65,         /* lineNo */
    "cat_impl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

omp_lock_t emlrtLockGlobal;

omp_nest_lock_t fusionAlgorithm_nestLockGlobal;

emlrtRTEInfo c_emlrtRTEI = {
    13,                /* lineNo */
    37,                /* colNo */
    "validateinteger", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validateinteger.m" /* pName */
};

emlrtRTEInfo g_emlrtRTEI = {
    1,                   /* lineNo */
    1,                   /* colNo */
    "SystemCore/delete", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pName */
};

emlrtRTEInfo i_emlrtRTEI = {
    15,                      /* lineNo */
    9,                       /* colNo */
    "assertSupportedString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertSupportedString.m" /* pName */
};

emlrtRTEInfo l_emlrtRTEI = {
    14,                    /* lineNo */
    37,                    /* colNo */
    "validatenonnegative", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonnegative.m" /* pName */
};

emlrtRTEInfo m_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatefinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatefinite.m" /* pName */
};

emlrtRTEInfo o_emlrtRTEI = {
    24,                                /* lineNo */
    5,                                 /* colNo */
    "isSymmetricPositiveSemiDefinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\trackinglib\\+"
    "matlabshared\\+tracking\\+internal\\isSymmetricPosit"
    "iveSemiDefinite.m" /* pName */
};

emlrtRTEInfo p_emlrtRTEI = {
    45,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

emlrtRTEInfo q_emlrtRTEI = {
    48,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

emlrtRTEInfo y_emlrtRTEI = {
    49,                   /* lineNo */
    19,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

emlrtRTEInfo ab_emlrtRTEI = {
    64,                   /* lineNo */
    15,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

emlrtRTEInfo db_emlrtRTEI = {
    386,                  /* lineNo */
    1,                    /* colNo */
    "find_first_indices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pName
                                                                       */
};

emlrtBCInfo jc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    117,                   /* lineNo */
    26,                    /* colNo */
    "",                    /* aName */
    "minPriorityQueue/LE", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

emlrtBCInfo kc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    116,                   /* lineNo */
    26,                    /* colNo */
    "",                    /* aName */
    "minPriorityQueue/LE", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

emlrtBCInfo qc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    118,                   /* lineNo */
    23,                    /* colNo */
    "",                    /* aName */
    "minPriorityQueue/LE", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

emlrtBCInfo rc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    118,                   /* lineNo */
    33,                    /* colNo */
    "",                    /* aName */
    "minPriorityQueue/LE", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

emlrtBCInfo sc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    118,                   /* lineNo */
    46,                    /* colNo */
    "",                    /* aName */
    "minPriorityQueue/LE", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

emlrtBCInfo tc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    118,                   /* lineNo */
    57,                    /* colNo */
    "",                    /* aName */
    "minPriorityQueue/LE", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\matlab\\graphfun\\codegen\\+matlab\\+"
    "internal\\+coder\\minPriorityQueue.m", /* pName */
    0                                       /* checkKind */
};

emlrtRTEInfo ob_emlrtRTEI = {
    225,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

emlrtRTEInfo sb_emlrtRTEI =
    {
        14,    /* lineNo */
        9,     /* colNo */
        "log", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elfun\\log.m" /* pName
                                                                          */
};

emlrtRTEInfo sd_emlrtRTEI = {
    31,     /* lineNo */
    6,      /* colNo */
    "find", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pName
                                                                       */
};

emlrtRTEInfo bh_emlrtRTEI = {
    73,       /* lineNo */
    28,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

emlrtRTEInfo fh_emlrtRTEI = {
    60,       /* lineNo */
    20,       /* colNo */
    "bsxfun", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\elmat\\bsxfun.m" /* pName
                                                                         */
};

emlrtRTEInfo qh_emlrtRTEI =
    {
        165,        /* lineNo */
        25,         /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

emlrtRTEInfo rh_emlrtRTEI =
    {
        166,        /* lineNo */
        23,         /* colNo */
        "constvel", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\constvel."
        "m" /* pName */
};

const char_T cv[128] = {
    '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\a',   '\b',
    '\t',   '\n',   '\v',   '\f',   '\r',   '\x0e', '\x0f', '\x10', '\x11',
    '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
    '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
    '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
    '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
    '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
    '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
    'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
    'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
    'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
    'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
    'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
    'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
    '~',    '\x7f'};

const int8_T iv[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                       0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};

const char_T cv1[13] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                        '_', 'd', 's', 'y', 'e', 'v'};

const char_T cv2[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                        '_', 'd', 'g', 'e', 'e', 'v', 'x'};

const int32_T iv1[4] = {0, 1, 2, 3};

/* End of code generation (fusionAlgorithm_data.c) */
