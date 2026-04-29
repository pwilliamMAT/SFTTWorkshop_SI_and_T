/*
 * SystemCore.c
 *
 * Code generation for function 'SystemCore'
 *
 */

/* Include files */
#include "SystemCore.h"
#include "JIPDATracker.h"
#include "ObjectTrackOutputter.h"
#include "TrackListManager.h"
#include "rt_nonfinite.h"
#include "trackEstimator.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo i_emlrtRSI = {
    1,                                        /* lineNo */
    "SystemProp/clearTunablePropertyChanged", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemProp.p" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    1,                                       /* lineNo */
    "SystemProp/matlabCodegenNotifyAnyProp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemProp.p" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    1,                 /* lineNo */
    "SystemCore/step", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    128,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    131,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    135,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    140,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    179,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    149,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    160,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    170,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    175,                      /* lineNo */
    "JIPDATracker/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo vj_emlrtRSI = {
    42,                                        /* lineNo */
    "SensorDataScheduler/SensorDataScheduler", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\SensorDataScheduler"
    ".m" /* pathName */
};

static emlrtRSInfo wj_emlrtRSI = {
    38,                          /* lineNo */
    "JIPDATrackInitiator/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo xj_emlrtRSI = {
    37,                         /* lineNo */
    "JIPDATrackAssigner/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo yj_emlrtRSI = {
    36,                        /* lineNo */
    "JIPDATrackUpdater/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo qk_emlrtRSI = {
    29,                                          /* lineNo */
    "JIPDATrackMaintainer/JIPDATrackMaintainer", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo rk_emlrtRSI = {
    16, /* lineNo */
    "JIPDATrackMaintainer/ConfirmationThreshold (generated property set "
    "method)", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo sk_emlrtRSI = {
    16, /* lineNo */
    "JIPDATrackMaintainer/ConfirmationThreshold (property validation)", /* fcnName
                                                                         */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo uk_emlrtRSI =
    {
        17, /* lineNo */
        "JIPDATrackMaintainer/DeletionThreshold (generated property set "
        "method)", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
        "tracker\\+internal\\+components\\JIPDATrackMaintaine"
        "r.m" /* pathName */
};

static emlrtRSInfo vk_emlrtRSI = {
    17,                                                             /* lineNo */
    "JIPDATrackMaintainer/DeletionThreshold (property validation)", /* fcnName
                                                                     */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo xk_emlrtRSI = {
    34,                           /* lineNo */
    "JIPDATrackMaintainer/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo yk_emlrtRSI = {
    246,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo al_emlrtRSI = {
    248,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo bl_emlrtRSI = {
    252,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo cl_emlrtRSI = {
    254,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo dl_emlrtRSI = {
    258,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo el_emlrtRSI = {
    260,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo fl_emlrtRSI = {
    264,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo gl_emlrtRSI = {
    266,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo hl_emlrtRSI = {
    270,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo il_emlrtRSI = {
    272,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo jl_emlrtRSI = {
    276,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo kl_emlrtRSI = {
    278,                                       /* lineNo */
    "JIPDATracker/processTunedPropertiesImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo ll_emlrtRSI = {
    284,                                      /* lineNo */
    "JIPDATracker/updateSensorSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo ml_emlrtRSI = {
    286,                                      /* lineNo */
    "JIPDATracker/updateSensorSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo nl_emlrtRSI = {
    289,                                      /* lineNo */
    "JIPDATracker/updateSensorSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo ol_emlrtRSI = {
    292,                                      /* lineNo */
    "JIPDATracker/updateSensorSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo pl_emlrtRSI = {
    295,                                      /* lineNo */
    "JIPDATracker/updateSensorSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo ql_emlrtRSI = {
    296,                                      /* lineNo */
    "JIPDATracker/updateSensorSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo rl_emlrtRSI = {
    96,                                               /* lineNo */
    "JIPDATrackInitiator/updateSensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo sl_emlrtRSI = {
    104,                                             /* lineNo */
    "JIPDATrackAssigner/updateSensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo tl_emlrtRSI = {
    80,                                             /* lineNo */
    "JIPDATrackUpdater/updateSensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo ul_emlrtRSI = {
    63,                                                /* lineNo */
    "JIPDATrackMaintainer/updateSensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo vl_emlrtRSI = {
    58,                                                /* lineNo */
    "ObjectTrackOutputter/updateSensorSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo wl_emlrtRSI = {
    302,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo xl_emlrtRSI = {
    305,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo yl_emlrtRSI = {
    308,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo am_emlrtRSI = {
    311,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo bm_emlrtRSI = {
    312,                                      /* lineNo */
    "JIPDATracker/updateTargetSpecification", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo cm_emlrtRSI = {
    89,                                               /* lineNo */
    "JIPDATrackInitiator/updateTargetSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackInitiator"
    ".m" /* pathName */
};

static emlrtRSInfo dm_emlrtRSI = {
    99,                                              /* lineNo */
    "JIPDATrackAssigner/updateTargetSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackAssigner."
    "m" /* pathName */
};

static emlrtRSInfo em_emlrtRSI = {
    75,                                             /* lineNo */
    "JIPDATrackUpdater/updateTargetSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo fm_emlrtRSI = {
    58,                                                /* lineNo */
    "JIPDATrackMaintainer/updateTargetSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\JIPDATrackMaintaine"
    "r.m" /* pathName */
};

static emlrtRSInfo gm_emlrtRSI = {
    53,                                                /* lineNo */
    "ObjectTrackOutputter/updateTargetSpecifications", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\ObjectTrackOutputte"
    "r.m" /* pathName */
};

static emlrtRSInfo hm_emlrtRSI = {
    317,                                        /* lineNo */
    "JIPDATracker/updateConfirmationThreshold", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRSInfo im_emlrtRSI = {
    322,                                    /* lineNo */
    "JIPDATracker/updateDeletionThreshold", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI = {
    1,                 /* lineNo */
    1,                 /* colNo */
    "SystemCore/step", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pName */
};

static emlrtRTEInfo b_emlrtRTEI = {
    12,               /* lineNo */
    23,               /* colNo */
    "mustBePositive", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\validators\\mustBePositi"
    "ve.m" /* pName */
};

static emlrtRTEInfo mc_emlrtRTEI = {
    1,            /* lineNo */
    1,            /* colNo */
    "SystemCore", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pName */
};

static emlrtRTEInfo nc_emlrtRTEI = {
    344,            /* lineNo */
    42,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo oc_emlrtRTEI = {
    344,            /* lineNo */
    13,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo pc_emlrtRTEI = {
    135,            /* lineNo */
    46,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo qc_emlrtRTEI = {
    310,            /* lineNo */
    63,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo rc_emlrtRTEI = {
    310,            /* lineNo */
    13,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo sc_emlrtRTEI = {
    294,            /* lineNo */
    63,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

static emlrtRTEInfo tc_emlrtRTEI = {
    294,            /* lineNo */
    13,             /* colNo */
    "JIPDATracker", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+"
    "tracker\\JIPDATracker.m" /* pName */
};

/* Function Declarations */
static void SystemCore_checkTunableProps(trackingAlgorithmStackData *SD,
                                         const emlrtStack *sp,
                                         fusion_tracker_JIPDATracker *obj);

/* Function Definitions */
static void SystemCore_checkTunableProps(trackingAlgorithmStackData *SD,
                                         const emlrtStack *sp,
                                         fusion_tracker_JIPDATracker *obj)
{
  c_fusion_tracker_internal_compo *c_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  g_fusion_tracker_internal_compo b_obj;
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (obj->TunablePropsChanged) {
    real_T val;
    boolean_T sensorSpecChange;
    obj->TunablePropsChanged = false;
    st.site = &m_emlrtRSI;
    b_st.site = &yk_emlrtRSI;
    sensorSpecChange = obj->tunablePropertyChanged[1];
    c_emxInitStruct_fusion_tracker_(&st, &b_obj, &sc_emlrtRTEI, true);
    if (sensorSpecChange) {
      c_fusion_tracker_sensorspecs_Ae val_idx_0;
      b_st.site = &al_emlrtRSI;
      c_st.site = &ll_emlrtRSI;
      c_obj = obj->Scheduler;
      obj->Scheduler = c_obj;
      val_idx_0 = obj->SensorSpecifications[0];
      c_st.site = &ml_emlrtRSI;
      SD->u1.f2.e_obj = obj->Initiator[0];
      SD->u1.f2.e_obj.SensorSpecifications[0] = val_idx_0;
      d_st.site = &rl_emlrtRSI;
      e_st.site = &yb_emlrtRSI;
      SD->u1.f2.e_obj.Estimator.StateEstimator.StateEstimator
          .SensorSpecifications[0] = val_idx_0;
      SD->u1.f2.e_obj.Estimator.StateEstimator.ExistenceEstimator
          .SensorSpecifications[0] = val_idx_0;
      obj->Initiator[0] = SD->u1.f2.e_obj;
      val_idx_0 = obj->SensorSpecifications[0];
      c_st.site = &nl_emlrtRSI;
      SD->u1.f2.b_obj = obj->Assigner[0];
      SD->u1.f2.b_obj.SensorSpecifications[0] = val_idx_0;
      d_st.site = &sl_emlrtRSI;
      e_st.site = &yb_emlrtRSI;
      SD->u1.f2.b_obj.Estimator.StateEstimator.StateEstimator
          .SensorSpecifications[0] = val_idx_0;
      SD->u1.f2.b_obj.Estimator.StateEstimator.ExistenceEstimator
          .SensorSpecifications[0] = val_idx_0;
      obj->Assigner[0] = SD->u1.f2.b_obj;
      val_idx_0 = obj->SensorSpecifications[0];
      c_st.site = &ol_emlrtRSI;
      SD->u1.f2.d_obj = obj->Updater[0];
      SD->u1.f2.d_obj.SensorSpecifications[0] = val_idx_0;
      d_st.site = &tl_emlrtRSI;
      e_st.site = &yb_emlrtRSI;
      SD->u1.f2.d_obj.Estimator.StateEstimator.StateEstimator
          .SensorSpecifications[0] = val_idx_0;
      SD->u1.f2.d_obj.Estimator.StateEstimator.ExistenceEstimator
          .SensorSpecifications[0] = val_idx_0;
      obj->Updater[0] = SD->u1.f2.d_obj;
      c_emxCopyStruct_fusion_tracker_(&b_st, &b_obj, &obj->TrackListManager,
                                      &sc_emlrtRTEI);
      b_obj.SensorSpecifications[0] = obj->SensorSpecifications[0];
      c_emxCopyStruct_fusion_tracker_(&b_st, &obj->TrackListManager, &b_obj,
                                      &tc_emlrtRTEI);
      c_st.site = &pl_emlrtRSI;
      SD->u1.f2.c_obj = obj->TrackMaintenance;
      val_idx_0 = obj->SensorSpecifications[0];
      SD->u1.f2.c_obj.SensorSpecifications[0] = val_idx_0;
      d_st.site = &ul_emlrtRSI;
      e_st.site = &yb_emlrtRSI;
      SD->u1.f2.c_obj.Estimator.StateEstimator.StateEstimator
          .SensorSpecifications[0] = val_idx_0;
      SD->u1.f2.c_obj.Estimator.StateEstimator.ExistenceEstimator
          .SensorSpecifications[0] = val_idx_0;
      obj->TrackMaintenance = SD->u1.f2.c_obj;
      c_st.site = &ql_emlrtRSI;
      SD->u1.f2.obj = obj->Outputter;
      val_idx_0 = obj->SensorSpecifications[0];
      SD->u1.f2.obj.SensorSpecifications[0] = val_idx_0;
      d_st.site = &vl_emlrtRSI;
      e_st.site = &yb_emlrtRSI;
      SD->u1.f2.obj.Estimator.StateEstimator.StateEstimator
          .SensorSpecifications[0] = val_idx_0;
      SD->u1.f2.obj.Estimator.StateEstimator.ExistenceEstimator
          .SensorSpecifications[0] = val_idx_0;
      obj->Outputter = SD->u1.f2.obj;
    }
    b_st.site = &bl_emlrtRSI;
    sensorSpecChange = obj->tunablePropertyChanged[0];
    if (sensorSpecChange) {
      c_fusion_tracker_targetspecs_Pa b_val_idx_0;
      b_st.site = &cl_emlrtRSI;
      c_st.site = &wl_emlrtRSI;
      SD->u1.f2.e_obj = obj->Initiator[0];
      b_val_idx_0 = obj->TargetSpecifications[0];
      SD->u1.f2.e_obj.TargetSpecifications[0] = b_val_idx_0;
      d_st.site = &cm_emlrtRSI;
      e_st.site = &wb_emlrtRSI;
      SD->u1.f2.e_obj.Estimator.StateEstimator.StateEstimator
          .TargetSpecifications[0] = b_val_idx_0;
      obj->Initiator[0] = SD->u1.f2.e_obj;
      c_st.site = &xl_emlrtRSI;
      SD->u1.f2.b_obj = obj->Assigner[0];
      b_val_idx_0 = obj->TargetSpecifications[0];
      SD->u1.f2.b_obj.TargetSpecifications[0] = b_val_idx_0;
      d_st.site = &dm_emlrtRSI;
      e_st.site = &wb_emlrtRSI;
      SD->u1.f2.b_obj.Estimator.StateEstimator.StateEstimator
          .TargetSpecifications[0] = b_val_idx_0;
      obj->Assigner[0] = SD->u1.f2.b_obj;
      c_st.site = &yl_emlrtRSI;
      SD->u1.f2.d_obj = obj->Updater[0];
      b_val_idx_0 = obj->TargetSpecifications[0];
      SD->u1.f2.d_obj.TargetSpecifications[0] = b_val_idx_0;
      d_st.site = &em_emlrtRSI;
      e_st.site = &wb_emlrtRSI;
      SD->u1.f2.d_obj.Estimator.StateEstimator.StateEstimator
          .TargetSpecifications[0] = b_val_idx_0;
      obj->Updater[0] = SD->u1.f2.d_obj;
      c_emxCopyStruct_fusion_tracker_(&b_st, &b_obj, &obj->TrackListManager,
                                      &qc_emlrtRTEI);
      b_obj.TargetSpecifications[0] = obj->TargetSpecifications[0];
      c_emxCopyStruct_fusion_tracker_(&b_st, &obj->TrackListManager, &b_obj,
                                      &rc_emlrtRTEI);
      c_st.site = &am_emlrtRSI;
      SD->u1.f2.c_obj = obj->TrackMaintenance;
      b_val_idx_0 = obj->TargetSpecifications[0];
      SD->u1.f2.c_obj.TargetSpecifications[0] = b_val_idx_0;
      d_st.site = &fm_emlrtRSI;
      e_st.site = &wb_emlrtRSI;
      SD->u1.f2.c_obj.Estimator.StateEstimator.StateEstimator
          .TargetSpecifications[0] = b_val_idx_0;
      obj->TrackMaintenance = SD->u1.f2.c_obj;
      c_st.site = &bm_emlrtRSI;
      SD->u1.f2.obj = obj->Outputter;
      b_val_idx_0 = obj->TargetSpecifications[0];
      SD->u1.f2.obj.TargetSpecifications[0] = b_val_idx_0;
      d_st.site = &gm_emlrtRSI;
      e_st.site = &wb_emlrtRSI;
      SD->u1.f2.obj.Estimator.StateEstimator.StateEstimator
          .TargetSpecifications[0] = b_val_idx_0;
      obj->Outputter = SD->u1.f2.obj;
    }
    c_emxFreeStruct_fusion_tracker_(&st, &b_obj);
    b_st.site = &dl_emlrtRSI;
    sensorSpecChange = obj->tunablePropertyChanged[3];
    if (sensorSpecChange) {
      b_st.site = &el_emlrtRSI;
      c_st.site = &hm_emlrtRSI;
      SD->u1.f2.c_obj = obj->TrackMaintenance;
      val = obj->c_ConfirmationExistenceProbabil;
      d_st.site = &rk_emlrtRSI;
      e_st.site = &sk_emlrtRSI;
      if (!(val > 0.0)) {
        emlrtErrorWithMessageIdR2018a(&e_st, &b_emlrtRTEI,
                                      "MATLAB:validators:mustBePositive",
                                      "MATLAB:validators:mustBePositive", 0);
      }
      e_st.site = &sk_emlrtRSI;
      f_st.site = &tk_emlrtRSI;
      if (!(val < 1.0)) {
        emlrtErrorWithMessageIdR2018a(
            &f_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
            "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
      }
      SD->u1.f2.c_obj.ConfirmationThreshold = val;
      obj->TrackMaintenance = SD->u1.f2.c_obj;
    }
    b_st.site = &fl_emlrtRSI;
    sensorSpecChange = obj->tunablePropertyChanged[4];
    if (sensorSpecChange) {
      b_st.site = &gl_emlrtRSI;
      c_st.site = &im_emlrtRSI;
      SD->u1.f2.c_obj = obj->TrackMaintenance;
      val = obj->DeletionExistenceProbability;
      d_st.site = &uk_emlrtRSI;
      e_st.site = &vk_emlrtRSI;
      if (!(val > 0.0)) {
        emlrtErrorWithMessageIdR2018a(&e_st, &b_emlrtRTEI,
                                      "MATLAB:validators:mustBePositive",
                                      "MATLAB:validators:mustBePositive", 0);
      }
      e_st.site = &vk_emlrtRSI;
      SD->u1.f2.c_obj.DeletionThreshold = val;
      obj->TrackMaintenance = SD->u1.f2.c_obj;
    }
    b_st.site = &hl_emlrtRSI;
    sensorSpecChange = obj->tunablePropertyChanged[2];
    if (sensorSpecChange) {
      b_st.site = &il_emlrtRSI;
      obj->Assigner[0].AssignmentThreshold = obj->MaxMahalanobisDistance;
      obj->Updater[0].AssignmentThreshold = obj->MaxMahalanobisDistance;
    }
    b_st.site = &jl_emlrtRSI;
    sensorSpecChange = obj->tunablePropertyChanged[5];
    if (sensorSpecChange) {
      b_st.site = &kl_emlrtRSI;
      obj->Assigner[0].MaxNumEvents = obj->MaxNumEvents;
    }
    st.site = &m_emlrtRSI;
    for (i = 0; i < 7; i++) {
      obj->tunablePropertyChanged[i] = false;
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void SystemCore_step(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                     fusion_tracker_JIPDATracker *obj,
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
  c_fusion_tracker_targetspecs_Pa c_obj_Estimator_StateEstimator_;
  c_fusion_tracker_targetspecs_Pa val;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  trackingEKF *d_obj_Estimator_StateEstimator_;
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
  if (obj->isInitialized == 2) {
    emlrtErrorWithMessageIdR2018a(
        sp, &emlrtRTEI, "MATLAB:system:methodCalledWhenReleasedCodegen",
        "MATLAB:system:methodCalledWhenReleasedCodegen", 3, 4, 4, "step");
  }
  if (obj->isInitialized != 1) {
    real_T e_obj_Estimator_StateEstimator_;
    real_T f_obj_Estimator_StateEstimator_;
    real_T g_obj_Estimator_StateEstimator_;
    real_T h_obj_Estimator_StateEstimator_;
    boolean_T flag;
    st.site = &m_emlrtRSI;
    b_st.site = &j_emlrtRSI;
    if (obj->isInitialized != 0) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &emlrtRTEI,
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen",
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen", 3, 4, 5,
          "setup");
    }
    obj->isInitialized = 1;
    c_st.site = &m_emlrtRSI;
    SD->f4.spec = obj->SensorSpecifications[0];
    obj->SensorSpecifications[0] = SD->f4.spec;
    d_st.site = &n_emlrtRSI;
    e_st.site = &l_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
      obj->tunablePropertyChanged[1] = true;
    }
    val = obj->TargetSpecifications[0];
    d_st.site = &o_emlrtRSI;
    e_st.site = &i_emlrtRSI;
    flag = (obj->isInitialized == 1);
    if (flag) {
      obj->TunablePropsChanged = true;
      obj->tunablePropertyChanged[0] = true;
    }
    d_st.site = &o_emlrtRSI;
    obj->TargetSpecifications[0] = val;
    c_emxInitStruct_fusion_tracker_(&c_st, &SD->f4.obj, &pc_emlrtRTEI, true);
    SD->f4.obj.TargetSpecifications[0] = obj->TargetSpecifications[0];
    SD->f4.obj.SensorSpecifications[0] = obj->SensorSpecifications[0];
    SD->f4.obj.InternalTrackList->size[0] = 0;
    d_st.site = &p_emlrtRSI;
    TrackListManager_setup(&d_st, &SD->f4.obj);
    c_emxCopyStruct_fusion_tracker_(&c_st, &obj->TrackListManager, &SD->f4.obj,
                                    &mc_emlrtRTEI);
    d_st.site = &q_emlrtRSI;
    obj->coder_buffer_pobj1.TimeTolerance = 1.0E-6;
    e_st.site = &vj_emlrtRSI;
    obj->Scheduler = &obj->coder_buffer_pobj1;
    SD->f4.spec = obj->SensorSpecifications[0];
    val = obj->TargetSpecifications[0];
    d_st.site = &s_emlrtRSI;
    e_st.site = &wj_emlrtRSI;
    d_obj_Estimator_StateEstimator_ =
        trackEstimator(&e_st, &val, &SD->f4.spec, &obj->coder_buffer_pobj0[0],
                       &c_obj_Estimator_StateEstimator_,
                       &SD->f4.c_obj_Estimator_StateEstimator_,
                       &SD->f4.d_obj_Estimator_StateEstimator_);
    SD->f4.initiators.TargetSpecifications[0] = val;
    SD->f4.initiators.SensorSpecifications[0] = SD->f4.spec;
    SD->f4.initiators.Estimator.StateEstimator.StateEstimator
        .TargetSpecifications[0] = c_obj_Estimator_StateEstimator_;
    SD->f4.initiators.Estimator.StateEstimator.StateEstimator
        .SensorSpecifications[0] = SD->f4.c_obj_Estimator_StateEstimator_;
    SD->f4.initiators.Estimator.StateEstimator.StateEstimator.TrackingFilter =
        d_obj_Estimator_StateEstimator_;
    SD->f4.initiators.Estimator.StateEstimator.ExistenceEstimator
        .SensorSpecifications[0] = SD->f4.d_obj_Estimator_StateEstimator_;
    SD->f4.initiators.Estimator.StateEstimator.ExistenceEstimator
        .DetectionProbability = e_obj_Estimator_StateEstimator_;
    SD->f4.initiators.Estimator.StateEstimator.ExistenceEstimator
        .SurvivalProbability = f_obj_Estimator_StateEstimator_;
    obj->Initiator[0] = SD->f4.initiators;
    SD->f4.spec = obj->SensorSpecifications[0];
    val = obj->TargetSpecifications[0];
    e_obj_Estimator_StateEstimator_ = obj->MaxMahalanobisDistance;
    f_obj_Estimator_StateEstimator_ = obj->MaxNumEvents;
    d_st.site = &t_emlrtRSI;
    e_st.site = &xj_emlrtRSI;
    d_obj_Estimator_StateEstimator_ =
        trackEstimator(&e_st, &val, &SD->f4.spec, &obj->coder_buffer_pobj0[1],
                       &c_obj_Estimator_StateEstimator_,
                       &SD->f4.c_obj_Estimator_StateEstimator_,
                       &SD->f4.d_obj_Estimator_StateEstimator_);
    SD->f4.assigners.TargetSpecifications[0] = val;
    SD->f4.assigners.SensorSpecifications[0] = SD->f4.spec;
    SD->f4.assigners.AssignmentThreshold = e_obj_Estimator_StateEstimator_;
    SD->f4.assigners.InitializationThreshold = 0.0;
    SD->f4.assigners.MaxNumEvents = f_obj_Estimator_StateEstimator_;
    SD->f4.assigners.Estimator.StateEstimator.StateEstimator
        .TargetSpecifications[0] = c_obj_Estimator_StateEstimator_;
    SD->f4.assigners.Estimator.StateEstimator.StateEstimator
        .SensorSpecifications[0] = SD->f4.c_obj_Estimator_StateEstimator_;
    SD->f4.assigners.Estimator.StateEstimator.StateEstimator.TrackingFilter =
        d_obj_Estimator_StateEstimator_;
    SD->f4.assigners.Estimator.StateEstimator.ExistenceEstimator
        .SensorSpecifications[0] = SD->f4.d_obj_Estimator_StateEstimator_;
    SD->f4.assigners.Estimator.StateEstimator.ExistenceEstimator
        .DetectionProbability = g_obj_Estimator_StateEstimator_;
    SD->f4.assigners.Estimator.StateEstimator.ExistenceEstimator
        .SurvivalProbability = h_obj_Estimator_StateEstimator_;
    obj->Assigner[0] = SD->f4.assigners;
    SD->f4.spec = obj->SensorSpecifications[0];
    val = obj->TargetSpecifications[0];
    e_obj_Estimator_StateEstimator_ = obj->MaxMahalanobisDistance;
    d_st.site = &u_emlrtRSI;
    e_st.site = &yj_emlrtRSI;
    d_obj_Estimator_StateEstimator_ =
        trackEstimator(&e_st, &val, &SD->f4.spec, &obj->coder_buffer_pobj0[2],
                       &c_obj_Estimator_StateEstimator_,
                       &SD->f4.c_obj_Estimator_StateEstimator_,
                       &SD->f4.d_obj_Estimator_StateEstimator_);
    SD->f4.updaters.TargetSpecifications[0] = val;
    SD->f4.updaters.SensorSpecifications[0] = SD->f4.spec;
    SD->f4.updaters.AssignmentThreshold = e_obj_Estimator_StateEstimator_;
    SD->f4.updaters.Estimator.StateEstimator.StateEstimator
        .TargetSpecifications[0] = c_obj_Estimator_StateEstimator_;
    SD->f4.updaters.Estimator.StateEstimator.StateEstimator
        .SensorSpecifications[0] = SD->f4.c_obj_Estimator_StateEstimator_;
    SD->f4.updaters.Estimator.StateEstimator.StateEstimator.TrackingFilter =
        d_obj_Estimator_StateEstimator_;
    SD->f4.updaters.Estimator.StateEstimator.ExistenceEstimator
        .SensorSpecifications[0] = SD->f4.d_obj_Estimator_StateEstimator_;
    SD->f4.updaters.Estimator.StateEstimator.ExistenceEstimator
        .DetectionProbability = f_obj_Estimator_StateEstimator_;
    SD->f4.updaters.Estimator.StateEstimator.ExistenceEstimator
        .SurvivalProbability = g_obj_Estimator_StateEstimator_;
    obj->Updater[0] = SD->f4.updaters;
    val = obj->TargetSpecifications[0];
    SD->f4.spec = obj->SensorSpecifications[0];
    SD->f4.r.TargetSpecifications[0] = val;
    SD->f4.r.SensorSpecifications[0] = SD->f4.spec;
    d_st.site = &v_emlrtRSI;
    ObjectTrackOutputter_setup(&d_st, &SD->f4.r, &obj->coder_buffer_pobj0[3]);
    obj->Outputter = SD->f4.r;
    d_st.site = &r_emlrtRSI;
    val = obj->TargetSpecifications[0];
    SD->f4.spec = obj->SensorSpecifications[0];
    e_obj_Estimator_StateEstimator_ = obj->c_ConfirmationExistenceProbabil;
    f_obj_Estimator_StateEstimator_ = obj->DeletionExistenceProbability;
    e_st.site = &qk_emlrtRSI;
    f_st.site = &rk_emlrtRSI;
    g_st.site = &sk_emlrtRSI;
    if (!(e_obj_Estimator_StateEstimator_ > 0.0)) {
      emlrtErrorWithMessageIdR2018a(&g_st, &b_emlrtRTEI,
                                    "MATLAB:validators:mustBePositive",
                                    "MATLAB:validators:mustBePositive", 0);
    }
    g_st.site = &sk_emlrtRSI;
    h_st.site = &tk_emlrtRSI;
    if (!(e_obj_Estimator_StateEstimator_ < 1.0)) {
      emlrtErrorWithMessageIdR2018a(
          &h_st, &c_emlrtRTEI, "MATLAB:validators:mustBeLessThan",
          "MATLAB:validators:mustBeLessThan", 3, 4, 1, "1");
    }
    e_st.site = &qk_emlrtRSI;
    f_st.site = &uk_emlrtRSI;
    g_st.site = &vk_emlrtRSI;
    if (!(f_obj_Estimator_StateEstimator_ > 0.0)) {
      emlrtErrorWithMessageIdR2018a(&g_st, &b_emlrtRTEI,
                                    "MATLAB:validators:mustBePositive",
                                    "MATLAB:validators:mustBePositive", 0);
    }
    g_st.site = &vk_emlrtRSI;
    d_st.site = &r_emlrtRSI;
    e_st.site = &xk_emlrtRSI;
    d_obj_Estimator_StateEstimator_ =
        trackEstimator(&e_st, &val, &SD->f4.spec, &obj->coder_buffer_pobj0[4],
                       &c_obj_Estimator_StateEstimator_,
                       &SD->f4.c_obj_Estimator_StateEstimator_,
                       &SD->f4.d_obj_Estimator_StateEstimator_);
    obj->TrackMaintenance.TargetSpecifications[0] = val;
    obj->TrackMaintenance.SensorSpecifications[0] = SD->f4.spec;
    obj->TrackMaintenance.ConfirmationThreshold =
        e_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.DeletionThreshold = f_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.Estimator.StateEstimator.StateEstimator
        .TargetSpecifications[0] = c_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.Estimator.StateEstimator.StateEstimator
        .SensorSpecifications[0] = SD->f4.c_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.Estimator.StateEstimator.StateEstimator
        .TrackingFilter = d_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.Estimator.StateEstimator.ExistenceEstimator
        .SensorSpecifications[0] = SD->f4.d_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.Estimator.StateEstimator.ExistenceEstimator
        .DetectionProbability = g_obj_Estimator_StateEstimator_;
    obj->TrackMaintenance.Estimator.StateEstimator.ExistenceEstimator
        .SurvivalProbability = h_obj_Estimator_StateEstimator_;
    obj->TunablePropsChanged = false;
    b_st.site = &j_emlrtRSI;
    c_emxCopyStruct_fusion_tracker_(&b_st, &SD->f4.obj, &obj->TrackListManager,
                                    &nc_emlrtRTEI);
    SD->f4.obj.InternalTrackList->size[0] = 0;
    c_emxCopyStruct_fusion_tracker_(&b_st, &obj->TrackListManager, &SD->f4.obj,
                                    &oc_emlrtRTEI);
    c_emxFreeStruct_fusion_tracker_(&b_st, &SD->f4.obj);
    obj->LastTrackID = 0U;
  }
  st.site = &j_emlrtRSI;
  SystemCore_checkTunableProps(SD, &st, obj);
  st.site = &m_emlrtRSI;
  JIPDATracker_stepImpl(
      SD, &st, obj, varargin_1_LookTime_data, varargin_1_LookTime_size,
      varargin_1_LookAzimuth_data, varargin_1_LookAzimuth_size,
      varargin_1_LookElevation_data, varargin_1_LookElevation_size,
      varargin_1_DetectionTime_data, varargin_1_DetectionTime_size,
      varargin_1_Azimuth_data, varargin_1_Azimuth_size,
      varargin_1_Elevation_data, varargin_1_Elevation_size,
      varargin_1_Range_data, varargin_1_Range_size, varargin_1_RangeRate_data,
      varargin_1_RangeRate_size, varargin_1_AzimuthAccuracy_data,
      varargin_1_AzimuthAccuracy_size, c_varargin_1_ElevationAccuracy_,
      d_varargin_1_ElevationAccuracy_, varargin_1_RangeAccuracy_data,
      varargin_1_RangeAccuracy_size, c_varargin_1_RangeRateAccuracy_,
      d_varargin_1_RangeRateAccuracy_, varargout_1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (SystemCore.c) */
