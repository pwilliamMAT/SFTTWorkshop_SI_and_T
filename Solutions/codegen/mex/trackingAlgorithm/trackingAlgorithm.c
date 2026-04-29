/*
 * trackingAlgorithm.c
 *
 * Code generation for function 'trackingAlgorithm'
 *
 */

/* Include files */
#include "trackingAlgorithm.h"
#include "SystemCore.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "omp.h"

/* Variable Definitions */
static fusion_tracker_JIPDATracker tracker;

static boolean_T tracker_not_empty;

static emlrtRSInfo emlrtRSI = {
    12,                  /* lineNo */
    "trackingAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\trackingAlgorit"
    "hm.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    17,                  /* lineNo */
    "trackingAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\trackingAlgorit"
    "hm.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    11,                  /* lineNo */
    "trackingAlgorithm", /* fcnName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\trackingAlgorit"
    "hm.m" /* pathName */
};

static emlrtRTEInfo lc_emlrtRTEI = {
    5,                   /* lineNo */
    12,                  /* colNo */
    "trackingAlgorithm", /* fName */
    "C:\\Users\\pwilliam\\OneDrive - "
    "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTWorkshop_SI_and_"
    "T\\Solutions\\trackingAlgorit"
    "hm.m" /* pName */
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

void trackingAlgorithm(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                       const struct0_T dets[1],
                       const c_fusion_tracker_targetspecs_Pa targetSpec[1],
                       const c_fusion_tracker_sensorspecs_Ae *sensorSpec,
                       emxArray_struct1_T *tracks)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  /*  trackingAlgorithm Summary of this function goes here */
  /*  Define the tracker as a persistent variable */
  /*  Initialize the tracker on first call using isempty */
  if (!tracker_not_empty) {
    boolean_T flag;
    /*  Instantiate tracker */
    st.site = &c_emlrtRSI;
    tracker.MaxMahalanobisDistance = 5.0;
    tracker.DeletionExistenceProbability = 0.1;
    tracker.MaxNumEvents = rtInf;
    tracker.isInitialized = 0;
    for (i = 0; i < 7; i++) {
      tracker.tunablePropertyChanged[i] = false;
    }
    flag = (tracker.isInitialized == 1);
    if (flag) {
      tracker.TunablePropsChanged = true;
      tracker.tunablePropertyChanged[0] = true;
    }
    tracker.TargetSpecifications[0] = targetSpec[0];
    flag = (tracker.isInitialized == 1);
    if (flag) {
      tracker.TunablePropsChanged = true;
      tracker.tunablePropertyChanged[1] = true;
    }
    tracker.SensorSpecifications[0] = *sensorSpec;
    tracker_not_empty = true;
    st.site = &emlrtRSI;
    flag = (tracker.isInitialized == 1);
    if (flag) {
      tracker.TunablePropsChanged = true;
      tracker.tunablePropertyChanged[3] = true;
    }
    st.site = &emlrtRSI;
    tracker.c_ConfirmationExistenceProbabil = 0.95;
    /*  Increased from default because clutter density is high */
  }
  /*  Update the tracker every step using current detections and time stamp */
  st.site = &b_emlrtRSI;
  b_st.site = &j_emlrtRSI;
  SystemCore_step(
      SD, &b_st, &tracker, dets[0].LookTime.data, dets[0].LookTime.size,
      dets[0].LookAzimuth.data, dets[0].LookAzimuth.size,
      dets[0].LookElevation.data, dets[0].LookElevation.size,
      dets[0].DetectionTime.data, dets[0].DetectionTime.size,
      dets[0].Azimuth.data, dets[0].Azimuth.size, dets[0].Elevation.data,
      dets[0].Elevation.size, dets[0].Range.data, dets[0].Range.size,
      dets[0].RangeRate.data, dets[0].RangeRate.size,
      dets[0].AzimuthAccuracy.data, dets[0].AzimuthAccuracy.size,
      dets[0].ElevationAccuracy.data, dets[0].ElevationAccuracy.size,
      dets[0].RangeAccuracy.data, dets[0].RangeAccuracy.size,
      dets[0].RangeRateAccuracy.data, dets[0].RangeRateAccuracy.size, tracks);
}

void trackingAlgorithm_emx_free(const emlrtStack *sp)
{
  d_emxFreeStruct_fusion_tracker_(sp, &tracker);
}

void trackingAlgorithm_emx_init(const emlrtStack *sp)
{
  d_emxInitStruct_fusion_tracker_(sp, &tracker, &lc_emlrtRTEI);
}

void trackingAlgorithm_init(void)
{
  tracker_not_empty = false;
}

/* End of code generation (trackingAlgorithm.c) */
