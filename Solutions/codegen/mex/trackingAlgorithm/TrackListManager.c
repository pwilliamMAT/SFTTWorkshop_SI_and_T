/*
 * TrackListManager.c
 *
 * Code generation for function 'TrackListManager'
 *
 */

/* Include files */
#include "TrackListManager.h"
#include "TrackEstimator1.h"
#include "rt_nonfinite.h"
#include "trackEstimator.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo ab_emlrtRSI = {
    51,                       /* lineNo */
    "TrackListManager/setup", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+components\\TrackListManager.m" /* pathName */
};

/* Function Definitions */
void TrackListManager_setup(const emlrtStack *sp,
                            g_fusion_tracker_internal_compo *obj)
{
  c_fusion_tracker_sensorspecs_Ae d_trackEstimator_StateEstimator;
  c_fusion_tracker_sensorspecs_Ae e_trackEstimator_StateEstimator;
  c_fusion_tracker_targetspecs_Pa c_trackEstimator_StateEstimator;
  emlrtStack st;
  trackingEKF lobj_0;
  trackingEKF *g_trackEstimator_StateEstimator;
  real_T f_expl_temp[36];
  real_T e_expl_temp[6];
  real_T f_trackEstimator_StateEstimator;
  uint32_T b_expl_temp;
  uint32_T expl_temp;
  boolean_T c_expl_temp;
  boolean_T d_expl_temp;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ab_emlrtRSI;
  g_trackEstimator_StateEstimator = trackEstimator(
      &st, &obj->TargetSpecifications[0], &obj->SensorSpecifications[0],
      &lobj_0, &c_trackEstimator_StateEstimator,
      &d_trackEstimator_StateEstimator, &e_trackEstimator_StateEstimator);
  c_TrackEstimator_sampleDistribu(
      g_trackEstimator_StateEstimator, &expl_temp, &b_expl_temp, &c_expl_temp,
      &d_expl_temp, e_expl_temp, f_expl_temp, &f_trackEstimator_StateEstimator);
  obj->InternalTrackList->size[0] = 0;
}

/* End of code generation (TrackListManager.c) */
