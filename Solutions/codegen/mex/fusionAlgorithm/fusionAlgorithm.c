/*
 * fusionAlgorithm.c
 *
 * Code generation for function 'fusionAlgorithm'
 *
 */

/* Include files */
#include "fusionAlgorithm.h"
#include "SystemCore.h"
#include "cosd.h"
#include "fuserSourceConfiguration.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "geodetic2ecefFormula.h"
#include "isSymmetricPositiveSemiDefinite.h"
#include "objectTrack.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "unique.h"
#include "validateattributes.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static trackFuser fuser;

static boolean_T fuser_not_empty;

static emlrtRSInfo emlrtRSI = {
    11,                /* lineNo */
    "fusionAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    17,                /* lineNo */
    "fusionAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    25,                /* lineNo */
    "fusionAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    38,                /* lineNo */
    "fusionAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    1,                                                         /* lineNo */
    "AbstractFusingConfiguration/AbstractFusingConfiguration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\AbstractFusingConfiguration.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    323,                                      /* lineNo */
    "fuserSourceConfiguration/setProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    324,                                      /* lineNo */
    "fuserSourceConfiguration/setProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    325,                                      /* lineNo */
    "fuserSourceConfiguration/setProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    326,                                      /* lineNo */
    "fuserSourceConfiguration/setProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\fuserSourceConfigura"
    "tion.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    295,                         /* lineNo */
    "FuserManager/FuserManager", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    1,               /* lineNo */
    "System/System", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\System.p" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    1,                                        /* lineNo */
    "SystemProp/clearTunablePropertyChanged", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemProp.p" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    1,                                                     /* lineNo */
    "ExportToSimulinkInterface/ExportToSimulinkInterface", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\ExportToSimulinkInterface.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    351,                     /* lineNo */
    "trackFuser/trackFuser", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m" /* pathName
                                                                          */
};

static emlrtRSInfo t_emlrtRSI = {
    357,                     /* lineNo */
    "trackFuser/trackFuser", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m" /* pathName
                                                                          */
};

static emlrtRSInfo u_emlrtRSI = {
    353,                     /* lineNo */
    "trackFuser/trackFuser", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m" /* pathName
                                                                          */
};

static emlrtRSInfo v_emlrtRSI = {
    1,                          /* lineNo */
    "SystemProp/setProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemProp.p" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    1,                                /* lineNo */
    "ProcessConstructorArguments/do", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\ProcessConstructorArguments.p" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    1,                                           /* lineNo */
    "ProcessConstructorArguments/setProperties", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\ProcessConstructorArguments.p" /* pathName */
};

static emlrtRSInfo y_emlrtRSI = {
    466,                                   /* lineNo */
    "trackFuser/set.SourceConfigurations", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m" /* pathName
                                                                          */
};

static emlrtRSInfo ab_emlrtRSI = {
    584,                       /* lineNo */
    "FuserManager/setSources", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    566,                       /* lineNo */
    "FuserManager/setSources", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pathName */
};

static emlrtRSInfo ch_emlrtRSI = {
    83,              /* lineNo */
    "Ned2ecefTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo dh_emlrtRSI = {
    87,              /* lineNo */
    "Ned2ecefTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo eh_emlrtRSI = {
    114,             /* lineNo */
    "Ned2ecefTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo fh_emlrtRSI = {
    115,             /* lineNo */
    "Ned2ecefTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo gh_emlrtRSI = {
    169,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo hh_emlrtRSI = {
    170,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo ih_emlrtRSI = {
    171,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo jh_emlrtRSI = {
    172,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo kh_emlrtRSI = {
    173,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo lh_emlrtRSI = {
    174,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo mh_emlrtRSI = {
    175,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo nh_emlrtRSI = {
    184,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo oh_emlrtRSI = {
    185,         /* lineNo */
    "syncTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo qh_emlrtRSI =
    {
        267,                        /* lineNo */
        "objectTrack/set.BranchID", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo rh_emlrtRSI =
    {
        272,                           /* lineNo */
        "objectTrack/set.SourceIndex", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo sh_emlrtRSI =
    {
        282,                   /* lineNo */
        "objectTrack/set.Age", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\tracking\\fusionlib\\objectTra"
        "ck.m" /* pathName */
};

static emlrtRSInfo th_emlrtRSI =
    {
        11,         /* lineNo */
        "ned2ecef", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\geodesy\\ned2ecef.m" /* pathName
                                                                       */
};

static emlrtRSInfo uh_emlrtRSI = {
    17,                /* lineNo */
    "enu2ecefFormula", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\geodesy\\private\\enu2ecefFormula."
    "m" /* pathName */
};

static emlrtRSInfo yh_emlrtRSI = {
    129,             /* lineNo */
    "Ecef2nedTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo ai_emlrtRSI = {
    133,             /* lineNo */
    "Ecef2nedTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo bi_emlrtRSI = {
    160,             /* lineNo */
    "Ecef2nedTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo ci_emlrtRSI = {
    161,             /* lineNo */
    "Ecef2nedTrack", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pathName */
};

static emlrtRSInfo di_emlrtRSI =
    {
        11,         /* lineNo */
        "ecef2ned", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\geodesy\\ecef2ned.m" /* pathName
                                                                       */
};

static emlrtRSInfo ei_emlrtRSI = {
    17,                /* lineNo */
    "ecef2enuFormula", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\geodesy\\private\\ecef2enuFormula."
    "m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI = {
    580,                       /* lineNo */
    21,                        /* colNo */
    "FuserManager/setSources", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtDCInfo emlrtDCI = {
    583,                       /* lineNo */
    29,                        /* colNo */
    "FuserManager/setSources", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    1                           /* checkKind */
};

static emlrtBCInfo emlrtBCI = {
    1,                         /* iFirst */
    2,                         /* iLast */
    583,                       /* lineNo */
    29,                        /* colNo */
    "",                        /* aName */
    "FuserManager/setSources", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    0                           /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = {
    585,                       /* lineNo */
    61,                        /* colNo */
    "FuserManager/setSources", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                         /* iFirst */
    2,                         /* iLast */
    581,                       /* lineNo */
    21,                        /* colNo */
    "",                        /* aName */
    "FuserManager/setSources", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "internal\\FuserManager.m", /* pName */
    3                           /* checkKind */
};

static emlrtRTEInfo lc_emlrtRTEI = {
    5,                 /* lineNo */
    12,                /* colNo */
    "fusionAlgorithm", /* fName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pName */
};

static emlrtRTEInfo mc_emlrtRTEI = {
    48,       /* lineNo */
    13,       /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static const real_T dv[9] = {
    -0.21995941811230382, 0.63733924591527369, 0.7385232156133259,
    0.94528735240751938,  0.326238902306856,   0.0,
    -0.24093500318982097, 0.69811665517860844, -0.67422804747366671};

static const real_T dv1[36] = {-0.21995941811230382,
                               0.63733924591527369,
                               0.7385232156133259,
                               0.0,
                               0.0,
                               0.0,
                               0.94528735240751938,
                               0.326238902306856,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               -0.24093500318982097,
                               0.69811665517860844,
                               -0.67422804747366671,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               -0.21995941811230382,
                               0.63733924591527369,
                               0.7385232156133259,
                               0.0,
                               0.0,
                               0.0,
                               0.94528735240751938,
                               0.326238902306856,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               -0.24093500318982097,
                               0.69811665517860844,
                               -0.67422804747366671};

static const real_T dv2[36] = {-0.21995941811230382,
                               0.94528735240751938,
                               -0.24093500318982097,
                               0.0,
                               0.0,
                               0.0,
                               0.63733924591527369,
                               0.326238902306856,
                               0.69811665517860844,
                               0.0,
                               0.0,
                               0.0,
                               0.7385232156133259,
                               0.0,
                               -0.67422804747366671,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               -0.21995941811230382,
                               0.94528735240751938,
                               -0.24093500318982097,
                               0.0,
                               0.0,
                               0.0,
                               0.63733924591527369,
                               0.326238902306856,
                               0.69811665517860844,
                               0.0,
                               0.0,
                               0.0,
                               0.7385232156133259,
                               0.0,
                               -0.67422804747366671};

/* Function Definitions */
void Ecef2nedTrack(const emlrtStack *sp, uint32_T centralTrack_TrackID,
                   uint32_T centralTrack_BranchID,
                   uint32_T centralTrack_SourceIndex, uint32_T centralTrack_Age,
                   real_T centralTrack_ObjectClassID,
                   const real_T c_centralTrack_ObjectClassProba[],
                   const int32_T d_centralTrack_ObjectClassProba[2],
                   boolean_T centralTrack_IsConfirmed,
                   boolean_T centralTrack_IsCoasted,
                   boolean_T centralTrack_IsSelfReported,
                   const real_T centralTrack_pState[6],
                   const real_T centralTrack_pStateCovariance[36],
                   real_T centralTrack_pUpdateTime, b_objectTrack *radarTrack)
{
  static const int8_T b_iv[6] = {0, 2, 4, 1, 3, 5};
  static const int8_T b_iv1[6] = {0, 3, 1, 4, 2, 5};
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T a[36];
  real_T b_a[36];
  real_T b_t;
  real_T b_y0;
  real_T cosLambda_tmp;
  real_T sinLambda_tmp;
  real_T t;
  real_T u;
  real_T v;
  real_T x0;
  real_T z0;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  /*  A function to transform a track in the central state-space to a track in
   */
  /*  the radar state-space. */
  /*  Initialize a track of the correct state size */
  /*  Force 'Integrated' for codegen consistency */
  /*  Sync properties of radarTrack except State and StateCovariance with */
  /*  radarTrack See syncTrack defined below. */
  st.site = &yh_emlrtRSI;
  /*  Updated to comply with codegen (different from local code only in that */
  /*  we're casting values for consistency) */
  /*  Copy scalars (cast to double/logical for objectTrack assignments) */
  b_st.site = &gh_emlrtRSI;
  radarTrack->TrackID = objectTrack_set_TrackID(
      &b_st, centralTrack_TrackID, &radarTrack->BranchID,
      &radarTrack->SourceIndex, &radarTrack->Age, &radarTrack->ObjectClassID,
      radarTrack->ObjectClassProbabilities.data,
      radarTrack->ObjectClassProbabilities.size, &radarTrack->IsConfirmed,
      &radarTrack->IsCoasted, &radarTrack->IsSelfReported, radarTrack->pState,
      radarTrack->pStateCovariance, &radarTrack->pUpdateTime);
  b_st.site = &hh_emlrtRSI;
  c_st.site = &qh_emlrtRSI;
  radarTrack->BranchID = centralTrack_BranchID;
  b_st.site = &ih_emlrtRSI;
  c_st.site = &rh_emlrtRSI;
  radarTrack->SourceIndex = centralTrack_SourceIndex;
  b_st.site = &jh_emlrtRSI;
  c_st.site = &qc_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (centralTrack_pUpdateTime < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
  }
  d_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(centralTrack_pUpdateTime) ||
      muDoubleScalarIsNaN(centralTrack_pUpdateTime)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
  }
  radarTrack->pUpdateTime = centralTrack_pUpdateTime;
  b_st.site = &kh_emlrtRSI;
  c_st.site = &sh_emlrtRSI;
  radarTrack->Age = centralTrack_Age;
  b_st.site = &lh_emlrtRSI;
  c_st.site = &ig_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (centralTrack_ObjectClassID < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 13, "ObjectClassID");
  }
  d_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(centralTrack_ObjectClassID) ||
      muDoubleScalarIsNaN(centralTrack_ObjectClassID) ||
      (!(muDoubleScalarFloor(centralTrack_ObjectClassID) ==
         centralTrack_ObjectClassID))) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:objectTrack:expectedInteger", 3, 4, 13, "ObjectClassID");
  }
  radarTrack->ObjectClassID = centralTrack_ObjectClassID;
  b_st.site = &mh_emlrtRSI;
  c_objectTrack_set_ObjectClassPr(&b_st, radarTrack,
                                  c_centralTrack_ObjectClassProba,
                                  d_centralTrack_ObjectClassProba);
  /* dst.TrackLogic = src.TrackLogic; */
  /*  Skip TrackLogicState - fuser manages this internally, and copying causes
   * codegen size issues */
  /* dst.TrackLogicState = double(src.TrackLogicState); */
  radarTrack->IsConfirmed = centralTrack_IsConfirmed;
  radarTrack->IsCoasted = centralTrack_IsCoasted;
  radarTrack->IsSelfReported = centralTrack_IsSelfReported;
  /*  States / covariance */
  b_st.site = &nh_emlrtRSI;
  c_st.site = &rc_emlrtRSI;
  validateattributes(&c_st, centralTrack_pState);
  b_st.site = &oh_emlrtRSI;
  c_st.site = &sc_emlrtRSI;
  b_validateattributes(&c_st, centralTrack_pStateCovariance);
  c_st.site = &tc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&c_st, centralTrack_pStateCovariance);
  /*  Parameters and Attributes */
  /*  Convert ECEF state to NED */
  st.site = &ai_emlrtRSI;
  b_st.site = &di_emlrtRSI;
  c_st.site = &ei_emlrtRSI;
  x0 = geodetic2ecefFormula(&c_st, &b_y0, &z0);
  u = centralTrack_pState[0] - x0;
  v = centralTrack_pState[2] - b_y0;
  x0 = centralTrack_pState[4] - z0;
  b_y0 = 42.39423231362;
  b_cosd(&b_y0);
  z0 = 42.39423231362;
  b_sind(&z0);
  cosLambda_tmp = -70.95934958874;
  b_cosd(&cosLambda_tmp);
  sinLambda_tmp = -70.95934958874;
  b_sind(&sinLambda_tmp);
  t = cosLambda_tmp * u + sinLambda_tmp * v;
  /*  ECEF->NED velocity via built-in */
  b_t = cosLambda_tmp * centralTrack_pState[1] +
        sinLambda_tmp * centralTrack_pState[3];
  radarTrack->pState[0] = -z0 * t + b_y0 * x0;
  radarTrack->pState[1] = -z0 * b_t + b_y0 * centralTrack_pState[5];
  radarTrack->pState[2] = -sinLambda_tmp * u + cosLambda_tmp * v;
  radarTrack->pState[3] = -sinLambda_tmp * centralTrack_pState[1] +
                          cosLambda_tmp * centralTrack_pState[3];
  radarTrack->pState[4] = -(b_y0 * t + z0 * x0);
  radarTrack->pState[5] = -(b_y0 * b_t + z0 * centralTrack_pState[5]);
  /*  Convert ECEF state COVARIANCE to NED state COVARIANCE */
  /*  Rotation matrix ECEF->NED (codegen-compatible) */
  /*  Note dcmecef2ned not supported for codegen */
  /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
  /*  Permute covariance matrix to group pos/vel */
  /*  ECEF->NED covariance: P_ned = R * P_ecef * R' */
  /*  Unpermute back to original interleaved format */
  memset(&a[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    i1 = 6 * i + 2;
    i2 = 6 * i + 4;
    for (i3 = 0; i3 < 6; i3++) {
      r = _mm_loadu_pd(&a[6 * i]);
      r1 = _mm_set1_pd(centralTrack_pStateCovariance[b_iv[i3] + 6 * b_iv[i]]);
      _mm_storeu_pd(&a[6 * i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3]), r1)));
      r = _mm_loadu_pd(&a[i1]);
      _mm_storeu_pd(
          &a[i1],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 2]), r1)));
      r = _mm_loadu_pd(&a[i2]);
      _mm_storeu_pd(
          &a[i2],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 4]), r1)));
    }
  }
  memset(&b_a[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    i1 = 6 * i + 2;
    i2 = 6 * i + 4;
    for (i3 = 0; i3 < 6; i3++) {
      __m128d r2;
      r = _mm_loadu_pd(&a[6 * i3]);
      r1 = _mm_loadu_pd(&b_a[6 * i]);
      r2 = _mm_set1_pd(dv1[i3 + 6 * i]);
      _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i3 + 2]);
      r1 = _mm_loadu_pd(&b_a[i1]);
      _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i3 + 4]);
      r1 = _mm_loadu_pd(&b_a[i2]);
      _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i < 6; i++) {
    for (i3 = 0; i3 < 6; i3++) {
      radarTrack->pStateCovariance[i3 + 6 * i] = b_a[b_iv1[i3] + 6 * b_iv1[i]];
    }
  }
  /*  Set state and covariance of radar track */
  st.site = &bi_emlrtRSI;
  b_st.site = &rc_emlrtRSI;
  validateattributes(&b_st, radarTrack->pState);
  st.site = &ci_emlrtRSI;
  b_st.site = &sc_emlrtRSI;
  b_validateattributes(&b_st, radarTrack->pStateCovariance);
  b_st.site = &tc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&b_st, radarTrack->pStateCovariance);
}

void Ned2ecefTrack(const emlrtStack *sp, uint32_T radarTrack_TrackID,
                   uint32_T radarTrack_BranchID,
                   uint32_T radarTrack_SourceIndex, uint32_T radarTrack_Age,
                   real_T radarTrack_ObjectClassID,
                   const real_T c_radarTrack_ObjectClassProbabi[],
                   const int32_T d_radarTrack_ObjectClassProbabi[2],
                   boolean_T radarTrack_IsConfirmed,
                   boolean_T radarTrack_IsCoasted,
                   boolean_T radarTrack_IsSelfReported,
                   const real_T radarTrack_pState[6],
                   const real_T radarTrack_pStateCovariance[36],
                   real_T radarTrack_pUpdateTime, b_objectTrack *centralTrack)
{
  static const int8_T b_iv[6] = {0, 2, 4, 1, 3, 5};
  static const int8_T b_iv1[6] = {0, 3, 1, 4, 2, 5};
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T a[36];
  real_T b_a[36];
  real_T velos[3];
  real_T b_y0;
  real_T cosLambda;
  real_T cosPhi;
  real_T sinLambda;
  real_T sinPhi;
  real_T t;
  real_T x0;
  real_T z0;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  /*  Initialize a track of the correct state size */
  /*  Force 'Integrated' for codegen consistency */
  /*  Sync properties of radarTrack except State and StateCovariance with */
  /*  radarTrack See syncTrack defined below. */
  st.site = &ch_emlrtRSI;
  /*  Updated to comply with codegen (different from local code only in that */
  /*  we're casting values for consistency) */
  /*  Copy scalars (cast to double/logical for objectTrack assignments) */
  b_st.site = &gh_emlrtRSI;
  centralTrack->TrackID = objectTrack_set_TrackID(
      &b_st, radarTrack_TrackID, &centralTrack->BranchID,
      &centralTrack->SourceIndex, &centralTrack->Age,
      &centralTrack->ObjectClassID, centralTrack->ObjectClassProbabilities.data,
      centralTrack->ObjectClassProbabilities.size, &centralTrack->IsConfirmed,
      &centralTrack->IsCoasted, &centralTrack->IsSelfReported,
      centralTrack->pState, centralTrack->pStateCovariance,
      &centralTrack->pUpdateTime);
  b_st.site = &hh_emlrtRSI;
  c_st.site = &qh_emlrtRSI;
  centralTrack->BranchID = radarTrack_BranchID;
  b_st.site = &ih_emlrtRSI;
  c_st.site = &rh_emlrtRSI;
  centralTrack->SourceIndex = radarTrack_SourceIndex;
  b_st.site = &jh_emlrtRSI;
  c_st.site = &qc_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (radarTrack_pUpdateTime < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
  }
  d_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(radarTrack_pUpdateTime) ||
      muDoubleScalarIsNaN(radarTrack_pUpdateTime)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
  }
  centralTrack->pUpdateTime = radarTrack_pUpdateTime;
  b_st.site = &kh_emlrtRSI;
  c_st.site = &sh_emlrtRSI;
  centralTrack->Age = radarTrack_Age;
  b_st.site = &lh_emlrtRSI;
  c_st.site = &ig_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (radarTrack_ObjectClassID < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &l_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:objectTrack:expectedNonnegative", 3, 4, 13, "ObjectClassID");
  }
  d_st.site = &gb_emlrtRSI;
  if (muDoubleScalarIsInf(radarTrack_ObjectClassID) ||
      muDoubleScalarIsNaN(radarTrack_ObjectClassID) ||
      (!(muDoubleScalarFloor(radarTrack_ObjectClassID) ==
         radarTrack_ObjectClassID))) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &c_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:objectTrack:expectedInteger", 3, 4, 13, "ObjectClassID");
  }
  centralTrack->ObjectClassID = radarTrack_ObjectClassID;
  b_st.site = &mh_emlrtRSI;
  c_objectTrack_set_ObjectClassPr(&b_st, centralTrack,
                                  c_radarTrack_ObjectClassProbabi,
                                  d_radarTrack_ObjectClassProbabi);
  /* dst.TrackLogic = src.TrackLogic; */
  /*  Skip TrackLogicState - fuser manages this internally, and copying causes
   * codegen size issues */
  /* dst.TrackLogicState = double(src.TrackLogicState); */
  centralTrack->IsConfirmed = radarTrack_IsConfirmed;
  centralTrack->IsCoasted = radarTrack_IsCoasted;
  centralTrack->IsSelfReported = radarTrack_IsSelfReported;
  /*  States / covariance */
  b_st.site = &nh_emlrtRSI;
  c_st.site = &rc_emlrtRSI;
  validateattributes(&c_st, radarTrack_pState);
  b_st.site = &oh_emlrtRSI;
  c_st.site = &sc_emlrtRSI;
  b_validateattributes(&c_st, radarTrack_pStateCovariance);
  c_st.site = &tc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&c_st, radarTrack_pStateCovariance);
  /*  Parameters and Attributes */
  /*  Convert NED state to ECEF state */
  st.site = &dh_emlrtRSI;
  b_st.site = &th_emlrtRSI;
  c_st.site = &uh_emlrtRSI;
  x0 = geodetic2ecefFormula(&c_st, &b_y0, &z0);
  cosPhi = 42.39423231362;
  b_cosd(&cosPhi);
  sinPhi = 42.39423231362;
  b_sind(&sinPhi);
  cosLambda = -70.95934958874;
  b_cosd(&cosLambda);
  sinLambda = -70.95934958874;
  b_sind(&sinLambda);
  t = cosPhi * -radarTrack_pState[4] - sinPhi * radarTrack_pState[0];
  /*  Rotation matrix ECEF->NED (codegen-compatible) */
  /*  Note dcmecef2ned not supported for codegen */
  /*  NED->ECEF velocity: v_ecef = R' * v_ned */
  memset(&velos[0], 0, 3U * sizeof(real_T));
  r = _mm_loadu_pd(&velos[0]);
  _mm_storeu_pd(&velos[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[0]),
                                         _mm_set1_pd(radarTrack_pState[1]))));
  velos[2] += 0.7385232156133259 * radarTrack_pState[1];
  r = _mm_loadu_pd(&velos[0]);
  _mm_storeu_pd(&velos[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[3]),
                                         _mm_set1_pd(radarTrack_pState[3]))));
  velos[2] += 0.0 * radarTrack_pState[3];
  r = _mm_loadu_pd(&velos[0]);
  _mm_storeu_pd(&velos[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[6]),
                                         _mm_set1_pd(radarTrack_pState[5]))));
  velos[2] += -0.67422804747366671 * radarTrack_pState[5];
  centralTrack->pState[0] =
      x0 + (cosLambda * t - sinLambda * radarTrack_pState[2]);
  centralTrack->pState[1] = velos[0];
  centralTrack->pState[2] =
      b_y0 + (sinLambda * t + cosLambda * radarTrack_pState[2]);
  centralTrack->pState[3] = velos[1];
  centralTrack->pState[4] =
      z0 + (sinPhi * -radarTrack_pState[4] + cosPhi * radarTrack_pState[0]);
  centralTrack->pState[5] = velos[2];
  /*  Convert NED state COVARIANCE to ECEF state COVARIANCE */
  /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
  /*  Permute covariance matrix to group pos/vel */
  /*  NED->ECEF covariance: P_ecef = R' * P_ned * R */
  /*  Unpermute back to original interleaved format */
  memset(&a[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    i1 = 6 * i + 2;
    i2 = 6 * i + 4;
    for (i3 = 0; i3 < 6; i3++) {
      r = _mm_loadu_pd(&a[6 * i]);
      r1 = _mm_set1_pd(radarTrack_pStateCovariance[b_iv[i3] + 6 * b_iv[i]]);
      _mm_storeu_pd(&a[6 * i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3]), r1)));
      r = _mm_loadu_pd(&a[i1]);
      _mm_storeu_pd(
          &a[i1],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 2]), r1)));
      r = _mm_loadu_pd(&a[i2]);
      _mm_storeu_pd(
          &a[i2],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 4]), r1)));
    }
  }
  memset(&b_a[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    i1 = 6 * i + 2;
    i2 = 6 * i + 4;
    for (i3 = 0; i3 < 6; i3++) {
      __m128d r2;
      r = _mm_loadu_pd(&a[6 * i3]);
      r1 = _mm_loadu_pd(&b_a[6 * i]);
      r2 = _mm_set1_pd(dv2[i3 + 6 * i]);
      _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i3 + 2]);
      r1 = _mm_loadu_pd(&b_a[i1]);
      _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&a[6 * i3 + 4]);
      r1 = _mm_loadu_pd(&b_a[i2]);
      _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
    }
  }
  for (i = 0; i < 6; i++) {
    for (i3 = 0; i3 < 6; i3++) {
      centralTrack->pStateCovariance[i3 + 6 * i] =
          b_a[b_iv1[i3] + 6 * b_iv1[i]];
    }
  }
  /*  Set state and covariance of central track */
  st.site = &eh_emlrtRSI;
  b_st.site = &rc_emlrtRSI;
  validateattributes(&b_st, centralTrack->pState);
  st.site = &fh_emlrtRSI;
  b_st.site = &sc_emlrtRSI;
  b_validateattributes(&b_st, centralTrack->pStateCovariance);
  b_st.site = &tc_emlrtRSI;
  isSymmetricPositiveSemiDefinite(&b_st, centralTrack->pStateCovariance);
}

void b_local2central(const emlrtStack *sp, uint32_T localTrack_TrackID,
                     uint32_T localTrack_BranchID,
                     uint32_T localTrack_SourceIndex, uint32_T localTrack_Age,
                     real_T localTrack_ObjectClassID,
                     real_T c_localTrack_ObjectClassProbabi,
                     boolean_T localTrack_IsConfirmed,
                     boolean_T localTrack_IsCoasted,
                     boolean_T localTrack_IsSelfReported,
                     const real_T localTrack_pState[6],
                     const real_T localTrack_pStateCovariance[36],
                     real_T localTrack_pUpdateTime, objectTrack *centralTrack)
{
  static const int8_T b_iv[6] = {0, 2, 4, 1, 3, 5};
  static const int8_T b_iv1[6] = {0, 3, 1, 4, 2, 5};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T a[36];
  real_T b_a[36];
  real_T velos[3];
  real_T b_y0;
  real_T cosLambda;
  real_T cosPhi;
  real_T sinLambda;
  real_T sinPhi;
  real_T x0;
  real_T z0;
  int32_T i;
  int32_T i3;
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
  /*  HELPER FUNCTIONS */
  /*  ---- Wrappers (homogeneous handles) ---- */
  /*  Pre-define output - force 'Integrated' for codegen consistency */
  centralTrack->TrackID = objectTrack_objectTrack(
      &centralTrack->BranchID, &centralTrack->SourceIndex, &centralTrack->Age,
      &centralTrack->ObjectClassID, &centralTrack->ObjectClassProbabilities,
      &centralTrack->IsConfirmed, &centralTrack->IsCoasted,
      &centralTrack->IsSelfReported, centralTrack->pState,
      centralTrack->pStateCovariance, &centralTrack->pUpdateTime);
  if (localTrack_SourceIndex == 1U) {
    __m128d r;
    __m128d r1;
    real_T t;
    int32_T i1;
    int32_T i2;
    /*  Radar */
    st.site = &ah_emlrtRSI;
    /*  Initialize a track of the correct state size */
    /*  Force 'Integrated' for codegen consistency */
    /*  Sync properties of radarTrack except State and StateCovariance with */
    /*  radarTrack See syncTrack defined below. */
    b_st.site = &ch_emlrtRSI;
    /*  Updated to comply with codegen (different from local code only in that
     */
    /*  we're casting values for consistency) */
    /*  Copy scalars (cast to double/logical for objectTrack assignments) */
    c_st.site = &gh_emlrtRSI;
    centralTrack->ObjectClassProbabilities = 1.0;
    d_st.site = &ph_emlrtRSI;
    centralTrack->TrackID = localTrack_TrackID;
    c_st.site = &hh_emlrtRSI;
    d_st.site = &qh_emlrtRSI;
    centralTrack->BranchID = localTrack_BranchID;
    c_st.site = &ih_emlrtRSI;
    d_st.site = &rh_emlrtRSI;
    centralTrack->SourceIndex = 1U;
    c_st.site = &jh_emlrtRSI;
    d_st.site = &qc_emlrtRSI;
    e_st.site = &gb_emlrtRSI;
    if (localTrack_pUpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }
    e_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(localTrack_pUpdateTime) ||
        muDoubleScalarIsNaN(localTrack_pUpdateTime)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }
    centralTrack->pUpdateTime = localTrack_pUpdateTime;
    c_st.site = &kh_emlrtRSI;
    d_st.site = &sh_emlrtRSI;
    centralTrack->Age = localTrack_Age;
    c_st.site = &lh_emlrtRSI;
    d_st.site = &ig_emlrtRSI;
    c_validateattributes(&d_st, localTrack_ObjectClassID);
    centralTrack->ObjectClassID = localTrack_ObjectClassID;
    c_st.site = &mh_emlrtRSI;
    d_objectTrack_set_ObjectClassPr(&c_st, centralTrack,
                                    c_localTrack_ObjectClassProbabi);
    /* dst.TrackLogic = src.TrackLogic; */
    /*  Skip TrackLogicState - fuser manages this internally, and copying causes
     * codegen size issues */
    /* dst.TrackLogicState = double(src.TrackLogicState); */
    centralTrack->IsConfirmed = localTrack_IsConfirmed;
    centralTrack->IsCoasted = localTrack_IsCoasted;
    centralTrack->IsSelfReported = localTrack_IsSelfReported;
    /*  States / covariance */
    c_st.site = &nh_emlrtRSI;
    d_st.site = &rc_emlrtRSI;
    validateattributes(&d_st, localTrack_pState);
    c_st.site = &oh_emlrtRSI;
    d_st.site = &sc_emlrtRSI;
    b_validateattributes(&d_st, localTrack_pStateCovariance);
    d_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&d_st, localTrack_pStateCovariance);
    /*  Parameters and Attributes */
    /*  Convert NED state to ECEF state */
    b_st.site = &dh_emlrtRSI;
    c_st.site = &th_emlrtRSI;
    d_st.site = &uh_emlrtRSI;
    x0 = geodetic2ecefFormula(&d_st, &b_y0, &z0);
    cosPhi = 42.39423231362;
    b_cosd(&cosPhi);
    sinPhi = 42.39423231362;
    b_sind(&sinPhi);
    cosLambda = -70.95934958874;
    b_cosd(&cosLambda);
    sinLambda = -70.95934958874;
    b_sind(&sinLambda);
    t = cosPhi * -localTrack_pState[4] - sinPhi * localTrack_pState[0];
    /*  Rotation matrix ECEF->NED (codegen-compatible) */
    /*  Note dcmecef2ned not supported for codegen */
    /*  NED->ECEF velocity: v_ecef = R' * v_ned */
    memset(&velos[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(&velos[0],
                  _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[0]),
                                           _mm_set1_pd(localTrack_pState[1]))));
    velos[2] += 0.7385232156133259 * localTrack_pState[1];
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(&velos[0],
                  _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[3]),
                                           _mm_set1_pd(localTrack_pState[3]))));
    velos[2] += 0.0 * localTrack_pState[3];
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(&velos[0],
                  _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[6]),
                                           _mm_set1_pd(localTrack_pState[5]))));
    velos[2] += -0.67422804747366671 * localTrack_pState[5];
    centralTrack->pState[0] =
        x0 + (cosLambda * t - sinLambda * localTrack_pState[2]);
    centralTrack->pState[1] = velos[0];
    centralTrack->pState[2] =
        b_y0 + (sinLambda * t + cosLambda * localTrack_pState[2]);
    centralTrack->pState[3] = velos[1];
    centralTrack->pState[4] =
        z0 + (sinPhi * -localTrack_pState[4] + cosPhi * localTrack_pState[0]);
    centralTrack->pState[5] = velos[2];
    /*  Convert NED state COVARIANCE to ECEF state COVARIANCE */
    /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
    /*  Permute covariance matrix to group pos/vel */
    /*  NED->ECEF covariance: P_ecef = R' * P_ned * R */
    /*  Unpermute back to original interleaved format */
    memset(&a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        r = _mm_loadu_pd(&a[6 * i]);
        r1 = _mm_set1_pd(localTrack_pStateCovariance[b_iv[i3] + 6 * b_iv[i]]);
        _mm_storeu_pd(
            &a[6 * i],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3]), r1)));
        r = _mm_loadu_pd(&a[i1]);
        _mm_storeu_pd(
            &a[i1],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 2]), r1)));
        r = _mm_loadu_pd(&a[i2]);
        _mm_storeu_pd(
            &a[i2],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 4]), r1)));
      }
    }
    memset(&b_a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        __m128d r2;
        r = _mm_loadu_pd(&a[6 * i3]);
        r1 = _mm_loadu_pd(&b_a[6 * i]);
        r2 = _mm_set1_pd(dv2[i3 + 6 * i]);
        _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 2]);
        r1 = _mm_loadu_pd(&b_a[i1]);
        _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 4]);
        r1 = _mm_loadu_pd(&b_a[i2]);
        _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (i = 0; i < 6; i++) {
      for (i3 = 0; i3 < 6; i3++) {
        centralTrack->pStateCovariance[i3 + 6 * i] =
            b_a[b_iv1[i3] + 6 * b_iv1[i]];
      }
    }
    /*  Set state and covariance of central track */
    b_st.site = &eh_emlrtRSI;
    c_st.site = &rc_emlrtRSI;
    validateattributes(&c_st, centralTrack->pState);
    b_st.site = &fh_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    b_validateattributes(&c_st, centralTrack->pStateCovariance);
    c_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&c_st, centralTrack->pStateCovariance);
  } else if (localTrack_SourceIndex == 2U) {
    __m128d r;
    __m128d r1;
    real_T t;
    real_T u;
    int32_T i1;
    int32_T i2;
    /*  ADSB */
    st.site = &bh_emlrtRSI;
    /*  A function to transform a track in the central state-space to a track in
     */
    /*  the radar state-space. */
    /*  Initialize a track of the correct state size */
    /*  Force 'Integrated' for codegen consistency */
    /*  Sync properties of radarTrack except State and StateCovariance with */
    /*  radarTrack See syncTrack defined below. */
    b_st.site = &yh_emlrtRSI;
    /*  Updated to comply with codegen (different from local code only in that
     */
    /*  we're casting values for consistency) */
    /*  Copy scalars (cast to double/logical for objectTrack assignments) */
    c_st.site = &gh_emlrtRSI;
    centralTrack->ObjectClassProbabilities = 1.0;
    d_st.site = &ph_emlrtRSI;
    centralTrack->TrackID = localTrack_TrackID;
    c_st.site = &hh_emlrtRSI;
    d_st.site = &qh_emlrtRSI;
    centralTrack->BranchID = localTrack_BranchID;
    c_st.site = &ih_emlrtRSI;
    d_st.site = &rh_emlrtRSI;
    centralTrack->SourceIndex = 2U;
    c_st.site = &jh_emlrtRSI;
    d_st.site = &qc_emlrtRSI;
    e_st.site = &gb_emlrtRSI;
    if (localTrack_pUpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }
    e_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(localTrack_pUpdateTime) ||
        muDoubleScalarIsNaN(localTrack_pUpdateTime)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }
    centralTrack->pUpdateTime = localTrack_pUpdateTime;
    c_st.site = &kh_emlrtRSI;
    d_st.site = &sh_emlrtRSI;
    centralTrack->Age = localTrack_Age;
    c_st.site = &lh_emlrtRSI;
    d_st.site = &ig_emlrtRSI;
    c_validateattributes(&d_st, localTrack_ObjectClassID);
    centralTrack->ObjectClassID = localTrack_ObjectClassID;
    c_st.site = &mh_emlrtRSI;
    d_objectTrack_set_ObjectClassPr(&c_st, centralTrack,
                                    c_localTrack_ObjectClassProbabi);
    /* dst.TrackLogic = src.TrackLogic; */
    /*  Skip TrackLogicState - fuser manages this internally, and copying causes
     * codegen size issues */
    /* dst.TrackLogicState = double(src.TrackLogicState); */
    centralTrack->IsConfirmed = localTrack_IsConfirmed;
    centralTrack->IsCoasted = localTrack_IsCoasted;
    centralTrack->IsSelfReported = localTrack_IsSelfReported;
    /*  States / covariance */
    c_st.site = &nh_emlrtRSI;
    d_st.site = &rc_emlrtRSI;
    validateattributes(&d_st, localTrack_pState);
    c_st.site = &oh_emlrtRSI;
    d_st.site = &sc_emlrtRSI;
    b_validateattributes(&d_st, localTrack_pStateCovariance);
    d_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&d_st, localTrack_pStateCovariance);
    /*  Parameters and Attributes */
    /*  Convert ECEF state to NED */
    b_st.site = &ai_emlrtRSI;
    c_st.site = &di_emlrtRSI;
    d_st.site = &ei_emlrtRSI;
    b_y0 = geodetic2ecefFormula(&d_st, &z0, &x0);
    u = localTrack_pState[0] - b_y0;
    t = localTrack_pState[2] - z0;
    b_y0 = localTrack_pState[4] - x0;
    z0 = 42.39423231362;
    b_cosd(&z0);
    x0 = 42.39423231362;
    b_sind(&x0);
    cosPhi = -70.95934958874;
    b_cosd(&cosPhi);
    sinPhi = -70.95934958874;
    b_sind(&sinPhi);
    cosLambda = cosPhi * u + sinPhi * t;
    /*  ECEF->NED velocity via built-in */
    sinLambda = cosPhi * localTrack_pState[1] + sinPhi * localTrack_pState[3];
    centralTrack->pState[0] = -x0 * cosLambda + z0 * b_y0;
    centralTrack->pState[1] = -x0 * sinLambda + z0 * localTrack_pState[5];
    centralTrack->pState[2] = -sinPhi * u + cosPhi * t;
    centralTrack->pState[3] =
        -sinPhi * localTrack_pState[1] + cosPhi * localTrack_pState[3];
    centralTrack->pState[4] = -(z0 * cosLambda + x0 * b_y0);
    centralTrack->pState[5] = -(z0 * sinLambda + x0 * localTrack_pState[5]);
    /*  Convert ECEF state COVARIANCE to NED state COVARIANCE */
    /*  Rotation matrix ECEF->NED (codegen-compatible) */
    /*  Note dcmecef2ned not supported for codegen */
    /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
    /*  Permute covariance matrix to group pos/vel */
    /*  ECEF->NED covariance: P_ned = R * P_ecef * R' */
    /*  Unpermute back to original interleaved format */
    memset(&a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        r = _mm_loadu_pd(&a[6 * i]);
        r1 = _mm_set1_pd(localTrack_pStateCovariance[b_iv[i3] + 6 * b_iv[i]]);
        _mm_storeu_pd(
            &a[6 * i],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3]), r1)));
        r = _mm_loadu_pd(&a[i1]);
        _mm_storeu_pd(
            &a[i1],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 2]), r1)));
        r = _mm_loadu_pd(&a[i2]);
        _mm_storeu_pd(
            &a[i2],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 4]), r1)));
      }
    }
    memset(&b_a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        __m128d r2;
        r = _mm_loadu_pd(&a[6 * i3]);
        r1 = _mm_loadu_pd(&b_a[6 * i]);
        r2 = _mm_set1_pd(dv1[i3 + 6 * i]);
        _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 2]);
        r1 = _mm_loadu_pd(&b_a[i1]);
        _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 4]);
        r1 = _mm_loadu_pd(&b_a[i2]);
        _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (i = 0; i < 6; i++) {
      for (i3 = 0; i3 < 6; i3++) {
        centralTrack->pStateCovariance[i3 + 6 * i] =
            b_a[b_iv1[i3] + 6 * b_iv1[i]];
      }
    }
    /*  Set state and covariance of radar track */
    b_st.site = &bi_emlrtRSI;
    c_st.site = &rc_emlrtRSI;
    validateattributes(&c_st, centralTrack->pState);
    b_st.site = &ci_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    b_validateattributes(&c_st, centralTrack->pStateCovariance);
    c_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&c_st, centralTrack->pStateCovariance);
  }
}

void central2local(const emlrtStack *sp, uint32_T centralTrack_TrackID,
                   uint32_T centralTrack_BranchID,
                   uint32_T centralTrack_SourceIndex, uint32_T centralTrack_Age,
                   real_T centralTrack_ObjectClassID,
                   real_T c_centralTrack_ObjectClassProba,
                   boolean_T centralTrack_IsConfirmed,
                   boolean_T centralTrack_IsCoasted,
                   boolean_T centralTrack_IsSelfReported,
                   const real_T centralTrack_pState[6],
                   const real_T centralTrack_pStateCovariance[36],
                   real_T centralTrack_pUpdateTime, objectTrack *localTrack)
{
  static const int8_T b_iv[6] = {0, 2, 4, 1, 3, 5};
  static const int8_T b_iv1[6] = {0, 3, 1, 4, 2, 5};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T a[36];
  real_T b_a[36];
  real_T velos[3];
  real_T b_t;
  real_T b_y0;
  real_T c_t;
  real_T cosPhi;
  real_T sinPhi;
  real_T x0;
  real_T z0;
  int32_T i;
  int32_T i3;
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
  /*  Pre-define output with longest TrackLogic to force codegen consistency */
  localTrack->TrackID = objectTrack_objectTrack(
      &localTrack->BranchID, &localTrack->SourceIndex, &localTrack->Age,
      &localTrack->ObjectClassID, &localTrack->ObjectClassProbabilities,
      &localTrack->IsConfirmed, &localTrack->IsCoasted,
      &localTrack->IsSelfReported, localTrack->pState,
      localTrack->pStateCovariance, &localTrack->pUpdateTime);
  if (centralTrack_SourceIndex == 1U) {
    __m128d r;
    __m128d r1;
    real_T t;
    real_T u;
    int32_T i1;
    int32_T i2;
    /*  Radar */
    st.site = &fi_emlrtRSI;
    /*  A function to transform a track in the central state-space to a track in
     */
    /*  the radar state-space. */
    /*  Initialize a track of the correct state size */
    /*  Force 'Integrated' for codegen consistency */
    /*  Sync properties of radarTrack except State and StateCovariance with */
    /*  radarTrack See syncTrack defined below. */
    b_st.site = &yh_emlrtRSI;
    /*  Updated to comply with codegen (different from local code only in that
     */
    /*  we're casting values for consistency) */
    /*  Copy scalars (cast to double/logical for objectTrack assignments) */
    c_st.site = &gh_emlrtRSI;
    localTrack->ObjectClassProbabilities = 1.0;
    d_st.site = &ph_emlrtRSI;
    localTrack->TrackID = centralTrack_TrackID;
    c_st.site = &hh_emlrtRSI;
    d_st.site = &qh_emlrtRSI;
    localTrack->BranchID = centralTrack_BranchID;
    c_st.site = &ih_emlrtRSI;
    d_st.site = &rh_emlrtRSI;
    localTrack->SourceIndex = 1U;
    c_st.site = &jh_emlrtRSI;
    d_st.site = &qc_emlrtRSI;
    e_st.site = &gb_emlrtRSI;
    if (centralTrack_pUpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }
    e_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(centralTrack_pUpdateTime) ||
        muDoubleScalarIsNaN(centralTrack_pUpdateTime)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }
    localTrack->pUpdateTime = centralTrack_pUpdateTime;
    c_st.site = &kh_emlrtRSI;
    d_st.site = &sh_emlrtRSI;
    localTrack->Age = centralTrack_Age;
    c_st.site = &lh_emlrtRSI;
    d_st.site = &ig_emlrtRSI;
    c_validateattributes(&d_st, centralTrack_ObjectClassID);
    localTrack->ObjectClassID = centralTrack_ObjectClassID;
    c_st.site = &mh_emlrtRSI;
    d_objectTrack_set_ObjectClassPr(&c_st, localTrack,
                                    c_centralTrack_ObjectClassProba);
    /* dst.TrackLogic = src.TrackLogic; */
    /*  Skip TrackLogicState - fuser manages this internally, and copying causes
     * codegen size issues */
    /* dst.TrackLogicState = double(src.TrackLogicState); */
    localTrack->IsConfirmed = centralTrack_IsConfirmed;
    localTrack->IsCoasted = centralTrack_IsCoasted;
    localTrack->IsSelfReported = centralTrack_IsSelfReported;
    /*  States / covariance */
    c_st.site = &nh_emlrtRSI;
    d_st.site = &rc_emlrtRSI;
    validateattributes(&d_st, centralTrack_pState);
    c_st.site = &oh_emlrtRSI;
    d_st.site = &sc_emlrtRSI;
    b_validateattributes(&d_st, centralTrack_pStateCovariance);
    d_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&d_st, centralTrack_pStateCovariance);
    /*  Parameters and Attributes */
    /*  Convert ECEF state to NED */
    b_st.site = &ai_emlrtRSI;
    c_st.site = &di_emlrtRSI;
    d_st.site = &ei_emlrtRSI;
    x0 = geodetic2ecefFormula(&d_st, &b_y0, &z0);
    u = centralTrack_pState[0] - x0;
    t = centralTrack_pState[2] - b_y0;
    x0 = centralTrack_pState[4] - z0;
    b_y0 = 42.39423231362;
    b_cosd(&b_y0);
    z0 = 42.39423231362;
    b_sind(&z0);
    cosPhi = -70.95934958874;
    b_cosd(&cosPhi);
    sinPhi = -70.95934958874;
    b_sind(&sinPhi);
    b_t = cosPhi * u + sinPhi * t;
    /*  ECEF->NED velocity via built-in */
    c_t = cosPhi * centralTrack_pState[1] + sinPhi * centralTrack_pState[3];
    localTrack->pState[0] = -z0 * b_t + b_y0 * x0;
    localTrack->pState[1] = -z0 * c_t + b_y0 * centralTrack_pState[5];
    localTrack->pState[2] = -sinPhi * u + cosPhi * t;
    localTrack->pState[3] =
        -sinPhi * centralTrack_pState[1] + cosPhi * centralTrack_pState[3];
    localTrack->pState[4] = -(b_y0 * b_t + z0 * x0);
    localTrack->pState[5] = -(b_y0 * c_t + z0 * centralTrack_pState[5]);
    /*  Convert ECEF state COVARIANCE to NED state COVARIANCE */
    /*  Rotation matrix ECEF->NED (codegen-compatible) */
    /*  Note dcmecef2ned not supported for codegen */
    /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
    /*  Permute covariance matrix to group pos/vel */
    /*  ECEF->NED covariance: P_ned = R * P_ecef * R' */
    /*  Unpermute back to original interleaved format */
    memset(&a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        r = _mm_loadu_pd(&a[6 * i]);
        r1 = _mm_set1_pd(centralTrack_pStateCovariance[b_iv[i3] + 6 * b_iv[i]]);
        _mm_storeu_pd(
            &a[6 * i],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3]), r1)));
        r = _mm_loadu_pd(&a[i1]);
        _mm_storeu_pd(
            &a[i1],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 2]), r1)));
        r = _mm_loadu_pd(&a[i2]);
        _mm_storeu_pd(
            &a[i2],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 4]), r1)));
      }
    }
    memset(&b_a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        __m128d r2;
        r = _mm_loadu_pd(&a[6 * i3]);
        r1 = _mm_loadu_pd(&b_a[6 * i]);
        r2 = _mm_set1_pd(dv1[i3 + 6 * i]);
        _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 2]);
        r1 = _mm_loadu_pd(&b_a[i1]);
        _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 4]);
        r1 = _mm_loadu_pd(&b_a[i2]);
        _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (i = 0; i < 6; i++) {
      for (i3 = 0; i3 < 6; i3++) {
        localTrack->pStateCovariance[i3 + 6 * i] =
            b_a[b_iv1[i3] + 6 * b_iv1[i]];
      }
    }
    /*  Set state and covariance of radar track */
    b_st.site = &bi_emlrtRSI;
    c_st.site = &rc_emlrtRSI;
    validateattributes(&c_st, localTrack->pState);
    b_st.site = &ci_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    b_validateattributes(&c_st, localTrack->pStateCovariance);
    c_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&c_st, localTrack->pStateCovariance);
  } else if (centralTrack_SourceIndex == 2U) {
    __m128d r;
    __m128d r1;
    real_T t;
    int32_T i1;
    int32_T i2;
    /*  ADSB */
    st.site = &gi_emlrtRSI;
    /*  Initialize a track of the correct state size */
    /*  Force 'Integrated' for codegen consistency */
    /*  Sync properties of radarTrack except State and StateCovariance with */
    /*  radarTrack See syncTrack defined below. */
    b_st.site = &ch_emlrtRSI;
    /*  Updated to comply with codegen (different from local code only in that
     */
    /*  we're casting values for consistency) */
    /*  Copy scalars (cast to double/logical for objectTrack assignments) */
    c_st.site = &gh_emlrtRSI;
    localTrack->ObjectClassProbabilities = 1.0;
    d_st.site = &ph_emlrtRSI;
    localTrack->TrackID = centralTrack_TrackID;
    c_st.site = &hh_emlrtRSI;
    d_st.site = &qh_emlrtRSI;
    localTrack->BranchID = centralTrack_BranchID;
    c_st.site = &ih_emlrtRSI;
    d_st.site = &rh_emlrtRSI;
    localTrack->SourceIndex = 2U;
    c_st.site = &jh_emlrtRSI;
    d_st.site = &qc_emlrtRSI;
    e_st.site = &gb_emlrtRSI;
    if (centralTrack_pUpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }
    e_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(centralTrack_pUpdateTime) ||
        muDoubleScalarIsNaN(centralTrack_pUpdateTime)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }
    localTrack->pUpdateTime = centralTrack_pUpdateTime;
    c_st.site = &kh_emlrtRSI;
    d_st.site = &sh_emlrtRSI;
    localTrack->Age = centralTrack_Age;
    c_st.site = &lh_emlrtRSI;
    d_st.site = &ig_emlrtRSI;
    c_validateattributes(&d_st, centralTrack_ObjectClassID);
    localTrack->ObjectClassID = centralTrack_ObjectClassID;
    c_st.site = &mh_emlrtRSI;
    d_objectTrack_set_ObjectClassPr(&c_st, localTrack,
                                    c_centralTrack_ObjectClassProba);
    /* dst.TrackLogic = src.TrackLogic; */
    /*  Skip TrackLogicState - fuser manages this internally, and copying causes
     * codegen size issues */
    /* dst.TrackLogicState = double(src.TrackLogicState); */
    localTrack->IsConfirmed = centralTrack_IsConfirmed;
    localTrack->IsCoasted = centralTrack_IsCoasted;
    localTrack->IsSelfReported = centralTrack_IsSelfReported;
    /*  States / covariance */
    c_st.site = &nh_emlrtRSI;
    d_st.site = &rc_emlrtRSI;
    validateattributes(&d_st, centralTrack_pState);
    c_st.site = &oh_emlrtRSI;
    d_st.site = &sc_emlrtRSI;
    b_validateattributes(&d_st, centralTrack_pStateCovariance);
    d_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&d_st, centralTrack_pStateCovariance);
    /*  Parameters and Attributes */
    /*  Convert NED state to ECEF state */
    b_st.site = &dh_emlrtRSI;
    c_st.site = &th_emlrtRSI;
    d_st.site = &uh_emlrtRSI;
    z0 = geodetic2ecefFormula(&d_st, &x0, &b_y0);
    cosPhi = 42.39423231362;
    b_cosd(&cosPhi);
    sinPhi = 42.39423231362;
    b_sind(&sinPhi);
    b_t = -70.95934958874;
    b_cosd(&b_t);
    c_t = -70.95934958874;
    b_sind(&c_t);
    t = cosPhi * -centralTrack_pState[4] - sinPhi * centralTrack_pState[0];
    /*  Rotation matrix ECEF->NED (codegen-compatible) */
    /*  Note dcmecef2ned not supported for codegen */
    /*  NED->ECEF velocity: v_ecef = R' * v_ned */
    memset(&velos[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(
        &velos[0],
        _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[0]),
                                 _mm_set1_pd(centralTrack_pState[1]))));
    velos[2] += 0.7385232156133259 * centralTrack_pState[1];
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(
        &velos[0],
        _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[3]),
                                 _mm_set1_pd(centralTrack_pState[3]))));
    velos[2] += 0.0 * centralTrack_pState[3];
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(
        &velos[0],
        _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[6]),
                                 _mm_set1_pd(centralTrack_pState[5]))));
    velos[2] += -0.67422804747366671 * centralTrack_pState[5];
    localTrack->pState[0] = z0 + (b_t * t - c_t * centralTrack_pState[2]);
    localTrack->pState[1] = velos[0];
    localTrack->pState[2] = x0 + (c_t * t + b_t * centralTrack_pState[2]);
    localTrack->pState[3] = velos[1];
    localTrack->pState[4] = b_y0 + (sinPhi * -centralTrack_pState[4] +
                                    cosPhi * centralTrack_pState[0]);
    localTrack->pState[5] = velos[2];
    /*  Convert NED state COVARIANCE to ECEF state COVARIANCE */
    /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
    /*  Permute covariance matrix to group pos/vel */
    /*  NED->ECEF covariance: P_ecef = R' * P_ned * R */
    /*  Unpermute back to original interleaved format */
    memset(&a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        r = _mm_loadu_pd(&a[6 * i]);
        r1 = _mm_set1_pd(centralTrack_pStateCovariance[b_iv[i3] + 6 * b_iv[i]]);
        _mm_storeu_pd(
            &a[6 * i],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3]), r1)));
        r = _mm_loadu_pd(&a[i1]);
        _mm_storeu_pd(
            &a[i1],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 2]), r1)));
        r = _mm_loadu_pd(&a[i2]);
        _mm_storeu_pd(
            &a[i2],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 4]), r1)));
      }
    }
    memset(&b_a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        __m128d r2;
        r = _mm_loadu_pd(&a[6 * i3]);
        r1 = _mm_loadu_pd(&b_a[6 * i]);
        r2 = _mm_set1_pd(dv2[i3 + 6 * i]);
        _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 2]);
        r1 = _mm_loadu_pd(&b_a[i1]);
        _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 4]);
        r1 = _mm_loadu_pd(&b_a[i2]);
        _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (i = 0; i < 6; i++) {
      for (i3 = 0; i3 < 6; i3++) {
        localTrack->pStateCovariance[i3 + 6 * i] =
            b_a[b_iv1[i3] + 6 * b_iv1[i]];
      }
    }
    /*  Set state and covariance of central track */
    b_st.site = &eh_emlrtRSI;
    c_st.site = &rc_emlrtRSI;
    validateattributes(&c_st, localTrack->pState);
    b_st.site = &fh_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    b_validateattributes(&c_st, localTrack->pStateCovariance);
    c_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&c_st, localTrack->pStateCovariance);
  }
}

emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

void fusionAlgorithm(const emlrtStack *sp, const emxArray_struct0_T *tracks,
                     real_T b_time, struct2_T fusedTracks_data[],
                     int32_T fusedTracks_size[1])
{
  static fuserSourceConfiguration gobj_1[2];
  static const real_T varargin_6[9] = {33.3333, 0.0, 0.0, 0.0,   33.3333,
                                       0.0,     0.0, 0.0, 0.3333};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  emxArray_real_T b_ids_data;
  emxArray_real_T *r;
  trackFuser *obj;
  real_T ids[2];
  real_T ids_data[2];
  int32_T ids_size[2];
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  /*  tracks: struct array (codegen-friendly, not objectTrack) */
  /*  time:   scalar double */
  if (!fuser_not_empty) {
    real_T d;
    int32_T loop_ub;
    boolean_T flag;
    /*  Use ONE pair of transform handles for ALL sources */
    /*  (homogeneous handles; per-source dispatch happens inside these
     * functions) */
    /*  NOTE: Do not assign to variables named like the functions; pass the
     * handles directly. */
    /*  MUST match main script configuration: Radar updates, ADSB initializes */
    st.site = &emlrtRSI;
    gobj_1[0].pIsTransformToCentralValid = false;
    gobj_1[0].pIsTransformToLocalValid = false;
    b_st.site = &e_emlrtRSI;
    c_st.site = &h_emlrtRSI;
    b_st.site = &f_emlrtRSI;
    gobj_1[0].SourceIndex = 1.0;
    b_st.site = &g_emlrtRSI;
    c_st.site = &i_emlrtRSI;
    gobj_1[0].IsInternalSource = true;
    c_st.site = &j_emlrtRSI;
    gobj_1[0].IsInitializingCentralTracks = false;
    c_st.site = &k_emlrtRSI;
    c_st.site = &l_emlrtRSI;
    st.site = &b_emlrtRSI;
    gobj_1[1].pIsTransformToCentralValid = false;
    gobj_1[1].pIsTransformToLocalValid = false;
    b_st.site = &e_emlrtRSI;
    c_st.site = &h_emlrtRSI;
    b_st.site = &f_emlrtRSI;
    gobj_1[1].SourceIndex = 2.0;
    b_st.site = &g_emlrtRSI;
    c_st.site = &i_emlrtRSI;
    gobj_1[1].IsInternalSource = true;
    c_st.site = &j_emlrtRSI;
    gobj_1[1].IsInitializingCentralTracks = true;
    c_st.site = &k_emlrtRSI;
    c_st.site = &l_emlrtRSI;
    /*  Name-value pairs are literals (compile-time constants) when written like
     * this */
    st.site = &c_emlrtRSI;
    obj = &fuser;
    b_st.site = &s_emlrtRSI;
    c_st.site = &m_emlrtRSI;
    d_st.site = &n_emlrtRSI;
    e_st.site = &o_emlrtRSI;
    d_st.site = &n_emlrtRSI;
    e_st.site = &p_emlrtRSI;
    fuser.isInitialized = 0;
    b_st.site = &s_emlrtRSI;
    c_st.site = &q_emlrtRSI;
    b_st.site = &u_emlrtRSI;
    c_st.site = &v_emlrtRSI;
    d_st.site = &w_emlrtRSI;
    e_st.site = &x_emlrtRSI;
    f_st.site = &o_emlrtRSI;
    flag = (fuser.isInitialized == 1);
    if (flag) {
      fuser.TunablePropsChanged = true;
    }
    memcpy(&fuser.ProcessNoise[0], &varargin_6[0], 9U * sizeof(real_T));
    e_st.site = &x_emlrtRSI;
    f_st.site = &o_emlrtRSI;
    flag = (fuser.isInitialized == 1);
    if (flag) {
      fuser.TunablePropsChanged = true;
    }
    e_st.site = &x_emlrtRSI;
    f_st.site = &y_emlrtRSI;
    g_st.site = &bb_emlrtRSI;
    obj->pSourceConfigurations[0] = fuserSourceConfiguration_clone(
        &g_st, &gobj_1[0], &fuser.coder_buffer_pobj1[0]);
    g_st.site = &bb_emlrtRSI;
    obj->pSourceConfigurations[1] = fuserSourceConfiguration_clone(
        &g_st, &gobj_1[0], &fuser.coder_buffer_pobj1[1]);
    fuser.pSourceConfigurations[0] = &gobj_1[0];
    fuser.pSourceConfigurations[1] = &gobj_1[1];
    fuser.pNumUsedConfigs = 2.0;
    ids[0] = 0.0;
    ids[1] = 0.0;
    d = fuser.pNumUsedConfigs;
    loop_ub = (int32_T)d;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                  &emlrtRTEI, &f_st);
    for (i = 0; i < loop_ub; i++) {
      if (((int32_T)((uint32_T)i + 1U) < 1) ||
          ((int32_T)((uint32_T)i + 1U) > 2)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, 2,
                                      &b_emlrtBCI, &f_st);
      }
      ids[i] = fuser.pSourceConfigurations[i]->SourceIndex;
    }
    d = fuser.pNumUsedConfigs;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &emlrtDCI, &f_st);
      }
      if (((int32_T)d < 1) || ((int32_T)d > 2)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 2, &emlrtBCI, &f_st);
      }
      loop_ub = (int32_T)d;
    }
    g_st.site = &ab_emlrtRSI;
    ids_size[0] = 1;
    ids_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      memcpy(&ids_data[0], &ids[0], (uint32_T)loop_ub * sizeof(real_T));
    }
    b_ids_data.data = &ids_data[0];
    b_ids_data.size = &ids_size[0];
    b_ids_data.allocatedSize = 2;
    b_ids_data.numDimensions = 2;
    b_ids_data.canFreeData = false;
    emxInit_real_T(&g_st, &r, 2, &mc_emlrtRTEI);
    h_st.site = &ib_emlrtRSI;
    unique_vector(&h_st, &b_ids_data, r);
    if (loop_ub != r->size[1]) {
      emlrtErrorWithMessageIdR2018a(
          &f_st, &b_emlrtRTEI, "fusion:trackFuser:ExpectedUniqueConfigIDs",
          "fusion:trackFuser:ExpectedUniqueConfigIDs", 6, 4, 20,
          "SourceConfigurations", 4, 11, "SourceIndex");
    }
    fuser.pSourceConfigIDs[0] = ids[0];
    fuser.pSourceConfigIDs[1] = ids[1];
    fuser.pIsValidSource[0] = false;
    fuser.pIsValidSource[1] = false;
    b_st.site = &t_emlrtRSI;
    c_st.site = &o_emlrtRSI;
    flag = (fuser.isInitialized == 1);
    if (flag) {
      fuser.TunablePropsChanged = true;
    }
    b_st.site = &t_emlrtRSI;
    c_st.site = &y_emlrtRSI;
    fuser.pSourceConfigurations[0] = &gobj_1[0];
    fuser.pSourceConfigurations[1] = &gobj_1[1];
    fuser.pNumUsedConfigs = 2.0;
    ids[0] = 0.0;
    ids[1] = 0.0;
    d = fuser.pNumUsedConfigs;
    loop_ub = (int32_T)d;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                  &emlrtRTEI, &c_st);
    for (i = 0; i < loop_ub; i++) {
      if (((int32_T)((uint32_T)i + 1U) < 1) ||
          ((int32_T)((uint32_T)i + 1U) > 2)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)i + 1U), 1, 2,
                                      &b_emlrtBCI, &c_st);
      }
      ids[i] = fuser.pSourceConfigurations[i]->SourceIndex;
    }
    d = fuser.pNumUsedConfigs;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &emlrtDCI, &c_st);
      }
      if (((int32_T)d < 1) || ((int32_T)d > 2)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 2, &emlrtBCI, &c_st);
      }
      loop_ub = (int32_T)d;
    }
    d_st.site = &ab_emlrtRSI;
    ids_size[0] = 1;
    ids_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      memcpy(&ids_data[0], &ids[0], (uint32_T)loop_ub * sizeof(real_T));
    }
    b_ids_data.data = &ids_data[0];
    b_ids_data.size = &ids_size[0];
    b_ids_data.allocatedSize = 2;
    b_ids_data.numDimensions = 2;
    b_ids_data.canFreeData = false;
    e_st.site = &ib_emlrtRSI;
    unique_vector(&e_st, &b_ids_data, r);
    if (loop_ub != r->size[1]) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &b_emlrtRTEI, "fusion:trackFuser:ExpectedUniqueConfigIDs",
          "fusion:trackFuser:ExpectedUniqueConfigIDs", 6, 4, 20,
          "SourceConfigurations", 4, 11, "SourceIndex");
    }
    emxFree_real_T(&c_st, &r);
    fuser.pSourceConfigIDs[0] = ids[0];
    fuser.pSourceConfigIDs[1] = ids[1];
    fuser.pIsValidSource[0] = false;
    fuser.pIsValidSource[1] = false;
    fuser.matlabCodegenIsDeleted = false;
    fuser_not_empty = true;
  }
  /*  Step the fuser */
  st.site = &d_emlrtRSI;
  b_st.site = &p_emlrtRSI;
  fusedTracks_size[0] =
      SystemCore_step(&b_st, &fuser, tracks, b_time, fusedTracks_data);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void fusionAlgorithm_delete(void)
{
  if (!fuser.matlabCodegenIsDeleted) {
    fuser.matlabCodegenIsDeleted = true;
    if (fuser.isInitialized == 1) {
      fuser.isInitialized = 2;
      SystemCore_releaseWrapper(&fuser);
    }
  }
}

void fusionAlgorithm_emx_free(const emlrtStack *sp)
{
  emxFreeStruct_trackFuser(sp, &fuser);
}

void fusionAlgorithm_emx_init(const emlrtStack *sp)
{
  emxInitStruct_trackFuser(sp, &fuser, &lc_emlrtRTEI);
}

void fusionAlgorithm_init(void)
{
  fuser_not_empty = false;
}

void fusionAlgorithm_new(void)
{
  fuser.matlabCodegenIsDeleted = true;
}

void local2central(const emlrtStack *sp, uint32_T localTrack_TrackID,
                   uint32_T localTrack_BranchID,
                   uint32_T localTrack_SourceIndex,
                   real_T localTrack_UpdateTime, uint32_T localTrack_Age,
                   const real_T localTrack_State[6],
                   const real_T localTrack_StateCovariance[36],
                   real_T localTrack_ObjectClassID,
                   const real_T c_localTrack_ObjectClassProbabi[],
                   const int32_T d_localTrack_ObjectClassProbabi[2],
                   boolean_T localTrack_IsConfirmed,
                   boolean_T localTrack_IsCoasted,
                   boolean_T localTrack_IsSelfReported,
                   b_objectTrack *centralTrack)
{
  static const int8_T b_iv[6] = {0, 2, 4, 1, 3, 5};
  static const int8_T b_iv1[6] = {0, 3, 1, 4, 2, 5};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T a[36];
  real_T b_a[36];
  real_T velos[3];
  real_T b_y0;
  real_T cosLambda;
  real_T cosPhi;
  real_T sinLambda;
  real_T sinPhi;
  real_T x0;
  real_T z0;
  int32_T i;
  int32_T i3;
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
  /*  HELPER FUNCTIONS */
  /*  ---- Wrappers (homogeneous handles) ---- */
  /*  Pre-define output - force 'Integrated' for codegen consistency */
  centralTrack->TrackID = b_objectTrack_objectTrack(
      &centralTrack->BranchID, &centralTrack->SourceIndex, &centralTrack->Age,
      &centralTrack->ObjectClassID, centralTrack->ObjectClassProbabilities.data,
      centralTrack->ObjectClassProbabilities.size, &centralTrack->IsConfirmed,
      &centralTrack->IsCoasted, &centralTrack->IsSelfReported,
      centralTrack->pState, centralTrack->pStateCovariance,
      &centralTrack->pUpdateTime);
  if (localTrack_SourceIndex == 1U) {
    __m128d r;
    __m128d r1;
    real_T t;
    int32_T i1;
    int32_T i2;
    /*  Radar */
    st.site = &ah_emlrtRSI;
    /*  Initialize a track of the correct state size */
    /*  Force 'Integrated' for codegen consistency */
    /*  Sync properties of radarTrack except State and StateCovariance with */
    /*  radarTrack See syncTrack defined below. */
    b_st.site = &ch_emlrtRSI;
    /*  Updated to comply with codegen (different from local code only in that
     */
    /*  we're casting values for consistency) */
    /*  Copy scalars (cast to double/logical for objectTrack assignments) */
    c_st.site = &gh_emlrtRSI;
    centralTrack->TrackID = objectTrack_set_TrackID(
        &c_st, localTrack_TrackID, &centralTrack->BranchID,
        &centralTrack->SourceIndex, &centralTrack->Age,
        &centralTrack->ObjectClassID,
        centralTrack->ObjectClassProbabilities.data,
        centralTrack->ObjectClassProbabilities.size, &centralTrack->IsConfirmed,
        &centralTrack->IsCoasted, &centralTrack->IsSelfReported,
        centralTrack->pState, centralTrack->pStateCovariance,
        &centralTrack->pUpdateTime);
    c_st.site = &hh_emlrtRSI;
    d_st.site = &qh_emlrtRSI;
    centralTrack->BranchID = localTrack_BranchID;
    c_st.site = &ih_emlrtRSI;
    d_st.site = &rh_emlrtRSI;
    centralTrack->SourceIndex = 1U;
    c_st.site = &jh_emlrtRSI;
    d_st.site = &qc_emlrtRSI;
    e_st.site = &gb_emlrtRSI;
    if (localTrack_UpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }
    e_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(localTrack_UpdateTime) ||
        muDoubleScalarIsNaN(localTrack_UpdateTime)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }
    centralTrack->pUpdateTime = localTrack_UpdateTime;
    c_st.site = &kh_emlrtRSI;
    d_st.site = &sh_emlrtRSI;
    centralTrack->Age = localTrack_Age;
    c_st.site = &lh_emlrtRSI;
    d_st.site = &ig_emlrtRSI;
    c_validateattributes(&d_st, localTrack_ObjectClassID);
    centralTrack->ObjectClassID = localTrack_ObjectClassID;
    c_st.site = &mh_emlrtRSI;
    c_objectTrack_set_ObjectClassPr(&c_st, centralTrack,
                                    c_localTrack_ObjectClassProbabi,
                                    d_localTrack_ObjectClassProbabi);
    /* dst.TrackLogic = src.TrackLogic; */
    /*  Skip TrackLogicState - fuser manages this internally, and copying causes
     * codegen size issues */
    /* dst.TrackLogicState = double(src.TrackLogicState); */
    centralTrack->IsConfirmed = localTrack_IsConfirmed;
    centralTrack->IsCoasted = localTrack_IsCoasted;
    centralTrack->IsSelfReported = localTrack_IsSelfReported;
    /*  States / covariance */
    c_st.site = &nh_emlrtRSI;
    d_st.site = &rc_emlrtRSI;
    validateattributes(&d_st, localTrack_State);
    c_st.site = &oh_emlrtRSI;
    d_st.site = &sc_emlrtRSI;
    b_validateattributes(&d_st, localTrack_StateCovariance);
    d_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&d_st, localTrack_StateCovariance);
    /*  Parameters and Attributes */
    /*  Convert NED state to ECEF state */
    b_st.site = &dh_emlrtRSI;
    c_st.site = &th_emlrtRSI;
    d_st.site = &uh_emlrtRSI;
    x0 = geodetic2ecefFormula(&d_st, &b_y0, &z0);
    cosPhi = 42.39423231362;
    b_cosd(&cosPhi);
    sinPhi = 42.39423231362;
    b_sind(&sinPhi);
    cosLambda = -70.95934958874;
    b_cosd(&cosLambda);
    sinLambda = -70.95934958874;
    b_sind(&sinLambda);
    t = cosPhi * -localTrack_State[4] - sinPhi * localTrack_State[0];
    /*  Rotation matrix ECEF->NED (codegen-compatible) */
    /*  Note dcmecef2ned not supported for codegen */
    /*  NED->ECEF velocity: v_ecef = R' * v_ned */
    memset(&velos[0], 0, 3U * sizeof(real_T));
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(&velos[0],
                  _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[0]),
                                           _mm_set1_pd(localTrack_State[1]))));
    velos[2] += 0.7385232156133259 * localTrack_State[1];
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(&velos[0],
                  _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[3]),
                                           _mm_set1_pd(localTrack_State[3]))));
    velos[2] += 0.0 * localTrack_State[3];
    r = _mm_loadu_pd(&velos[0]);
    _mm_storeu_pd(&velos[0],
                  _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv[6]),
                                           _mm_set1_pd(localTrack_State[5]))));
    velos[2] += -0.67422804747366671 * localTrack_State[5];
    centralTrack->pState[0] =
        x0 + (cosLambda * t - sinLambda * localTrack_State[2]);
    centralTrack->pState[1] = velos[0];
    centralTrack->pState[2] =
        b_y0 + (sinLambda * t + cosLambda * localTrack_State[2]);
    centralTrack->pState[3] = velos[1];
    centralTrack->pState[4] =
        z0 + (sinPhi * -localTrack_State[4] + cosPhi * localTrack_State[0]);
    centralTrack->pState[5] = velos[2];
    /*  Convert NED state COVARIANCE to ECEF state COVARIANCE */
    /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
    /*  Permute covariance matrix to group pos/vel */
    /*  NED->ECEF covariance: P_ecef = R' * P_ned * R */
    /*  Unpermute back to original interleaved format */
    memset(&a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        r = _mm_loadu_pd(&a[6 * i]);
        r1 = _mm_set1_pd(localTrack_StateCovariance[b_iv[i3] + 6 * b_iv[i]]);
        _mm_storeu_pd(
            &a[6 * i],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3]), r1)));
        r = _mm_loadu_pd(&a[i1]);
        _mm_storeu_pd(
            &a[i1],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 2]), r1)));
        r = _mm_loadu_pd(&a[i2]);
        _mm_storeu_pd(
            &a[i2],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv1[6 * i3 + 4]), r1)));
      }
    }
    memset(&b_a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        __m128d r2;
        r = _mm_loadu_pd(&a[6 * i3]);
        r1 = _mm_loadu_pd(&b_a[6 * i]);
        r2 = _mm_set1_pd(dv2[i3 + 6 * i]);
        _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 2]);
        r1 = _mm_loadu_pd(&b_a[i1]);
        _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 4]);
        r1 = _mm_loadu_pd(&b_a[i2]);
        _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (i = 0; i < 6; i++) {
      for (i3 = 0; i3 < 6; i3++) {
        centralTrack->pStateCovariance[i3 + 6 * i] =
            b_a[b_iv1[i3] + 6 * b_iv1[i]];
      }
    }
    /*  Set state and covariance of central track */
    b_st.site = &eh_emlrtRSI;
    c_st.site = &rc_emlrtRSI;
    validateattributes(&c_st, centralTrack->pState);
    b_st.site = &fh_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    b_validateattributes(&c_st, centralTrack->pStateCovariance);
    c_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&c_st, centralTrack->pStateCovariance);
  } else if (localTrack_SourceIndex == 2U) {
    __m128d r;
    __m128d r1;
    real_T t;
    real_T u;
    int32_T i1;
    int32_T i2;
    /*  ADSB */
    st.site = &bh_emlrtRSI;
    /*  A function to transform a track in the central state-space to a track in
     */
    /*  the radar state-space. */
    /*  Initialize a track of the correct state size */
    /*  Force 'Integrated' for codegen consistency */
    /*  Sync properties of radarTrack except State and StateCovariance with */
    /*  radarTrack See syncTrack defined below. */
    b_st.site = &yh_emlrtRSI;
    /*  Updated to comply with codegen (different from local code only in that
     */
    /*  we're casting values for consistency) */
    /*  Copy scalars (cast to double/logical for objectTrack assignments) */
    c_st.site = &gh_emlrtRSI;
    centralTrack->TrackID = objectTrack_set_TrackID(
        &c_st, localTrack_TrackID, &centralTrack->BranchID,
        &centralTrack->SourceIndex, &centralTrack->Age,
        &centralTrack->ObjectClassID,
        centralTrack->ObjectClassProbabilities.data,
        centralTrack->ObjectClassProbabilities.size, &centralTrack->IsConfirmed,
        &centralTrack->IsCoasted, &centralTrack->IsSelfReported,
        centralTrack->pState, centralTrack->pStateCovariance,
        &centralTrack->pUpdateTime);
    c_st.site = &hh_emlrtRSI;
    d_st.site = &qh_emlrtRSI;
    centralTrack->BranchID = localTrack_BranchID;
    c_st.site = &ih_emlrtRSI;
    d_st.site = &rh_emlrtRSI;
    centralTrack->SourceIndex = 2U;
    c_st.site = &jh_emlrtRSI;
    d_st.site = &qc_emlrtRSI;
    e_st.site = &gb_emlrtRSI;
    if (localTrack_UpdateTime < 0.0) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &l_emlrtRTEI,
          "Coder:toolbox:ValidateattributesexpectedNonnegative",
          "MATLAB:objectTrack:expectedNonnegative", 3, 4, 10, "UpdateTime");
    }
    e_st.site = &gb_emlrtRSI;
    if (muDoubleScalarIsInf(localTrack_UpdateTime) ||
        muDoubleScalarIsNaN(localTrack_UpdateTime)) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
          "MATLAB:objectTrack:expectedFinite", 3, 4, 10, "UpdateTime");
    }
    centralTrack->pUpdateTime = localTrack_UpdateTime;
    c_st.site = &kh_emlrtRSI;
    d_st.site = &sh_emlrtRSI;
    centralTrack->Age = localTrack_Age;
    c_st.site = &lh_emlrtRSI;
    d_st.site = &ig_emlrtRSI;
    c_validateattributes(&d_st, localTrack_ObjectClassID);
    centralTrack->ObjectClassID = localTrack_ObjectClassID;
    c_st.site = &mh_emlrtRSI;
    c_objectTrack_set_ObjectClassPr(&c_st, centralTrack,
                                    c_localTrack_ObjectClassProbabi,
                                    d_localTrack_ObjectClassProbabi);
    /* dst.TrackLogic = src.TrackLogic; */
    /*  Skip TrackLogicState - fuser manages this internally, and copying causes
     * codegen size issues */
    /* dst.TrackLogicState = double(src.TrackLogicState); */
    centralTrack->IsConfirmed = localTrack_IsConfirmed;
    centralTrack->IsCoasted = localTrack_IsCoasted;
    centralTrack->IsSelfReported = localTrack_IsSelfReported;
    /*  States / covariance */
    c_st.site = &nh_emlrtRSI;
    d_st.site = &rc_emlrtRSI;
    validateattributes(&d_st, localTrack_State);
    c_st.site = &oh_emlrtRSI;
    d_st.site = &sc_emlrtRSI;
    b_validateattributes(&d_st, localTrack_StateCovariance);
    d_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&d_st, localTrack_StateCovariance);
    /*  Parameters and Attributes */
    /*  Convert ECEF state to NED */
    b_st.site = &ai_emlrtRSI;
    c_st.site = &di_emlrtRSI;
    d_st.site = &ei_emlrtRSI;
    b_y0 = geodetic2ecefFormula(&d_st, &z0, &x0);
    u = localTrack_State[0] - b_y0;
    t = localTrack_State[2] - z0;
    b_y0 = localTrack_State[4] - x0;
    z0 = 42.39423231362;
    b_cosd(&z0);
    x0 = 42.39423231362;
    b_sind(&x0);
    cosPhi = -70.95934958874;
    b_cosd(&cosPhi);
    sinPhi = -70.95934958874;
    b_sind(&sinPhi);
    cosLambda = cosPhi * u + sinPhi * t;
    /*  ECEF->NED velocity via built-in */
    sinLambda = cosPhi * localTrack_State[1] + sinPhi * localTrack_State[3];
    centralTrack->pState[0] = -x0 * cosLambda + z0 * b_y0;
    centralTrack->pState[1] = -x0 * sinLambda + z0 * localTrack_State[5];
    centralTrack->pState[2] = -sinPhi * u + cosPhi * t;
    centralTrack->pState[3] =
        -sinPhi * localTrack_State[1] + cosPhi * localTrack_State[3];
    centralTrack->pState[4] = -(z0 * cosLambda + x0 * b_y0);
    centralTrack->pState[5] = -(z0 * sinLambda + x0 * localTrack_State[5]);
    /*  Convert ECEF state COVARIANCE to NED state COVARIANCE */
    /*  Rotation matrix ECEF->NED (codegen-compatible) */
    /*  Note dcmecef2ned not supported for codegen */
    /*  Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz] */
    /*  Permute covariance matrix to group pos/vel */
    /*  ECEF->NED covariance: P_ned = R * P_ecef * R' */
    /*  Unpermute back to original interleaved format */
    memset(&a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        r = _mm_loadu_pd(&a[6 * i]);
        r1 = _mm_set1_pd(localTrack_StateCovariance[b_iv[i3] + 6 * b_iv[i]]);
        _mm_storeu_pd(
            &a[6 * i],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3]), r1)));
        r = _mm_loadu_pd(&a[i1]);
        _mm_storeu_pd(
            &a[i1],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 2]), r1)));
        r = _mm_loadu_pd(&a[i2]);
        _mm_storeu_pd(
            &a[i2],
            _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&dv2[6 * i3 + 4]), r1)));
      }
    }
    memset(&b_a[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      i1 = 6 * i + 2;
      i2 = 6 * i + 4;
      for (i3 = 0; i3 < 6; i3++) {
        __m128d r2;
        r = _mm_loadu_pd(&a[6 * i3]);
        r1 = _mm_loadu_pd(&b_a[6 * i]);
        r2 = _mm_set1_pd(dv1[i3 + 6 * i]);
        _mm_storeu_pd(&b_a[6 * i], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 2]);
        r1 = _mm_loadu_pd(&b_a[i1]);
        _mm_storeu_pd(&b_a[i1], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&a[6 * i3 + 4]);
        r1 = _mm_loadu_pd(&b_a[i2]);
        _mm_storeu_pd(&b_a[i2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (i = 0; i < 6; i++) {
      for (i3 = 0; i3 < 6; i3++) {
        centralTrack->pStateCovariance[i3 + 6 * i] =
            b_a[b_iv1[i3] + 6 * b_iv1[i]];
      }
    }
    /*  Set state and covariance of radar track */
    b_st.site = &bi_emlrtRSI;
    c_st.site = &rc_emlrtRSI;
    validateattributes(&c_st, centralTrack->pState);
    b_st.site = &ci_emlrtRSI;
    c_st.site = &sc_emlrtRSI;
    b_validateattributes(&c_st, centralTrack->pStateCovariance);
    c_st.site = &tc_emlrtRSI;
    isSymmetricPositiveSemiDefinite(&c_st, centralTrack->pStateCovariance);
  }
}

/* End of code generation (fusionAlgorithm.c) */
