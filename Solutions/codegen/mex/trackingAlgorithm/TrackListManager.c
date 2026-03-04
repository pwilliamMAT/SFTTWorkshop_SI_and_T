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
static emlrtRSInfo bb_emlrtRSI = {
    51,                       /* lineNo */
    "TrackListManager/setup", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "TrackListManager.m" /* pathName */
};

/* Function Definitions */
void TrackListManager_setup(trackingAlgorithmStackData *SD,
                            const emlrtStack *sp,
                            g_fusion_tracker_internal_compo *obj)
{
  emlrtStack st;
  struct_T e_expl_temp[3];
  trackingEKF lobj_0[3];
  real_T f_expl_temp[3];
  real_T h_expl_temp;
  uint32_T b_expl_temp;
  uint32_T expl_temp;
  boolean_T g_expl_temp[3];
  boolean_T c_expl_temp;
  boolean_T d_expl_temp;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bb_emlrtRSI;
  trackEstimator(SD, &st, &obj->TargetSpecifications.f1,
                 &obj->TargetSpecifications.f2, &obj->TargetSpecifications.f3,
                 &obj->SensorSpecifications[0], &lobj_0[0],
                 &SD->u3.f11.expl_temp);
  c_TrackEstimator_sampleDistribu(
      SD->u3.f11.expl_temp.StateEstimator.StateEstimator.Estimators.f1
          .TrackingFilter,
      SD->u3.f11.expl_temp.StateEstimator.StateEstimator.Estimators.f2
          .TrackingFilter,
      SD->u3.f11.expl_temp.StateEstimator.StateEstimator.Estimators.f3
          .TrackingFilter,
      &expl_temp, &b_expl_temp, &c_expl_temp, &d_expl_temp, e_expl_temp,
      f_expl_temp, g_expl_temp, &h_expl_temp);
  obj->InternalTrackList->size[0] = 0;
}

/* End of code generation (TrackListManager.c) */
