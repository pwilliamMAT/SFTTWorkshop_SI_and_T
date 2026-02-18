/*
 * trackEstimator.c
 *
 * Code generation for function 'trackEstimator'
 *
 */

/* Include files */
#include "trackEstimator.h"
#include "TrackEstimator1.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRSInfo ib_emlrtRSI = {
    26,               /* lineNo */
    "trackEstimator", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\+fusion\\+tracker\\+"
    "internal\\+utils\\trackEstimator.m" /* pathName */
};

/* Function Definitions */
void trackEstimator(trackingAlgorithmStackData *SD, const emlrtStack *sp,
                    const c_fusion_tracker_targetspecs_Pa *tgtSpecs_f1,
                    const c_fusion_tracker_targetspecs_Ge *tgtSpecs_f2,
                    const c_fusion_tracker_targetspecs_He *tgtSpecs_f3,
                    const c_fusion_tracker_sensorspecs_Ae *sensorSpecs,
                    trackingEKF *iobj_0,
                    i_fusion_tracker_internal_estim *trkEstimator)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  SD->u2.f8.stateEstimator.DeletionThreshold = 1.0E-6;
  SD->u2.f8.stateEstimator.Estimators.f1 = SD->u2.f8.estimator;
  SD->u2.f8.stateEstimator.Estimators.f2 = SD->u2.f8.b_estimator;
  SD->u2.f8.stateEstimator.Estimators.f3 = SD->u2.f8.c_estimator;
  SD->u2.f8.ipdaEstimator.StateEstimator = SD->u2.f8.stateEstimator;
  trkEstimator->StateEstimator = SD->u2.f8.ipdaEstimator;
  SD->u2.f8.r = *trkEstimator;
  c_TrackEstimator_set_TargetSpec(SD, &SD->u2.f8.r, tgtSpecs_f1, tgtSpecs_f2,
                                  tgtSpecs_f3);
  *trkEstimator = SD->u2.f8.r;
  SD->u2.f8.ipdaEstimator = trkEstimator->StateEstimator;
  SD->u2.f8.stateEstimator = trkEstimator->StateEstimator.StateEstimator;
  SD->u2.f8.stateEstimator.SensorSpecifications[0] = *sensorSpecs;
  SD->u2.f8.estimator =
      trkEstimator->StateEstimator.StateEstimator.Estimators.f1;
  SD->u2.f8.estimator.SensorSpecifications[0] = *sensorSpecs;
  SD->u2.f8.stateEstimator.Estimators.f1 = SD->u2.f8.estimator;
  SD->u2.f8.b_estimator =
      trkEstimator->StateEstimator.StateEstimator.Estimators.f2;
  SD->u2.f8.b_estimator.SensorSpecifications[0] = *sensorSpecs;
  SD->u2.f8.stateEstimator.Estimators.f2 = SD->u2.f8.b_estimator;
  SD->u2.f8.c_estimator =
      trkEstimator->StateEstimator.StateEstimator.Estimators.f3;
  SD->u2.f8.c_estimator.SensorSpecifications[0] = *sensorSpecs;
  SD->u2.f8.stateEstimator.Estimators.f3 = SD->u2.f8.c_estimator;
  SD->u2.f8.ipdaEstimator.StateEstimator = SD->u2.f8.stateEstimator;
  SD->u2.f8.ipdaEstimator.ExistenceEstimator.SensorSpecifications[0] =
      *sensorSpecs;
  trkEstimator->StateEstimator = SD->u2.f8.ipdaEstimator;
  SD->u2.f8.r = *trkEstimator;
  st.site = &ib_emlrtRSI;
  TrackEstimator_setup(SD, &st, &SD->u2.f8.r, &iobj_0[0]);
  *trkEstimator = SD->u2.f8.r;
}

/* End of code generation (trackEstimator.c) */
