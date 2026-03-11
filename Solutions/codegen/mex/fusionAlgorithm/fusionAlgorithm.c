/*
 * fusionAlgorithm.c
 *
 * Code generation for function 'fusionAlgorithm'
 *
 */

/* Include files */
#include "fusionAlgorithm.h"
#include "SystemCore.h"
#include "fuserSourceConfiguration.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include "mwmathutil.h"
#include "omp.h"
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

static emlrtRTEInfo kc_emlrtRTEI = {
    5,                 /* lineNo */
    12,                /* colNo */
    "fusionAlgorithm", /* fName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\fusionAlgorithm"
    ".m" /* pName */
};

static emlrtRTEInfo lc_emlrtRTEI = {
    48,       /* lineNo */
    13,       /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

/* Function Definitions */
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
    /* , ... */
    /* 'CentralToLocalTransformFcn', @(x) x, ...  */
    /* 'LocalToCentralTransformFcn', @(x) x);  */
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
    /* , ... */
    /* 'CentralToLocalTransformFcn', @(x) x, ...  */
    /* 'LocalToCentralTransformFcn', @(x) x);  */
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
    emxInit_real_T(&g_st, &r, 2, &lc_emlrtRTEI);
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
  SystemCore_step(&b_st, &fuser, tracks, b_time, fusedTracks_data,
                  &fusedTracks_size[0]);
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
  emxInitStruct_trackFuser(sp, &fuser, &kc_emlrtRTEI);
}

void fusionAlgorithm_init(void)
{
  fuser_not_empty = false;
}

void fusionAlgorithm_new(void)
{
  fuser.matlabCodegenIsDeleted = true;
}

/* End of code generation (fusionAlgorithm.c) */
