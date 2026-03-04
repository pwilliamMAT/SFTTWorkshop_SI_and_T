/*
 * JIPDATrackUpdater.c
 *
 * Code generation for function 'JIPDATrackUpdater'
 *
 */

/* Include files */
#include "JIPDATrackUpdater.h"
#include "AerospaceMonostaticRadar.h"
#include "IPDAEstimator.h"
#include "TrackEstimator1.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo ykb_emlrtRSI = {
    45,                         /* lineNo */
    "JIPDATrackUpdater/update", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo alb_emlrtRSI = {
    46,                         /* lineNo */
    "JIPDATrackUpdater/update", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo blb_emlrtRSI = {
    53,                         /* lineNo */
    "JIPDATrackUpdater/update", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo clb_emlrtRSI = {
    55,                         /* lineNo */
    "JIPDATrackUpdater/update", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo dlb_emlrtRSI = {
    69,                         /* lineNo */
    "JIPDATrackUpdater/update", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo elb_emlrtRSI = {
    86,            /* lineNo */
    "updateTrack", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pathName */
};

static emlrtRSInfo flb_emlrtRSI = {
    100,                          /* lineNo */
    "TrackEstimator/correctJPDA", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo glb_emlrtRSI = {
    99,                           /* lineNo */
    "TrackEstimator/correctJPDA", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtRSInfo ilb_emlrtRSI = {
    97,                           /* lineNo */
    "TrackEstimator/correctJPDA", /* fcnName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m" /* pathName */
};

static emlrtBCInfo tj_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    69,                         /* lineNo */
    27,                         /* colNo */
    "",                         /* aName */
    "JIPDATrackUpdater/update", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m", /* pName */
    0                      /* checkKind */
};

static emlrtBCInfo uj_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    58,                         /* lineNo */
    26,                         /* colNo */
    "",                         /* aName */
    "JIPDATrackUpdater/update", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m", /* pName */
    0                      /* checkKind */
};

static emlrtBCInfo vj_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    69,                         /* lineNo */
    65,                         /* colNo */
    "",                         /* aName */
    "JIPDATrackUpdater/update", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m", /* pName */
    0                      /* checkKind */
};

static emlrtBCInfo wj_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    69,                         /* lineNo */
    102,                        /* colNo */
    "",                         /* aName */
    "JIPDATrackUpdater/update", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m", /* pName */
    0                      /* checkKind */
};

static emlrtBCInfo xj_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    100,                          /* lineNo */
    82,                           /* colNo */
    "",                           /* aName */
    "TrackEstimator/correctJPDA", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+estimators/"
    "TrackEstimator.m", /* pName */
    0                   /* checkKind */
};

static emlrtRTEInfo kl_emlrtRTEI = {
    43,                  /* lineNo */
    30,                  /* colNo */
    "JIPDATrackUpdater", /* fName */
    "/MATLAB/toolbox/fusion/core/fusion/+fusion/+tracker/+internal/+components/"
    "JIPDATrackUpdater.m" /* pName */
};

/* Function Definitions */
void JIPDATrackUpdater_update(
    trackingAlgorithmStackData *SD, const emlrtStack *sp,
    real_T updater_AssignmentThreshold,
    h_fusion_tracker_internal_estim *c_updater_Estimator_StateEstima,
    emxArray_struct_T *trackList, const real_T sensorData_LookTime_data[],
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
    const int32_T d_sensorData_RangeRateAccuracy_[2],
    const emxArray_real_T *assignment)
{
  b_emxArray_struct_T *modelData;
  b_struct_T track;
  b_struct_T *trackList_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T d_assignment_data;
  real_T z_data[200];
  real_T t_data[100];
  real_T b_t_data[50];
  real_T c_assignment_data[50];
  const real_T *assignment_data;
  real_T t;
  int32_T b_t_size[2];
  int32_T t_size[2];
  int32_T z_size[2];
  int32_T assignment_size;
  int32_T b_k;
  int32_T i;
  int32_T idx;
  int32_T k;
  int32_T last;
  int32_T scalarLB;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  assignment_data = assignment->data;
  trackList_data = trackList->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_struct_T1(sp, &modelData, &kl_emlrtRTEI);
  st.site = &ykb_emlrtRSI;
  c_AerospaceMonostaticRadar_pars(
      &st, sensorData_LookTime_data, sensorData_LookTime_size,
      sensorData_LookAzimuth_data, sensorData_LookAzimuth_size,
      sensorData_LookElevation_data, sensorData_LookElevation_size,
      sensorData_DetectionTime_data, sensorData_DetectionTime_size,
      sensorData_Azimuth_data, sensorData_Azimuth_size,
      sensorData_Elevation_data, sensorData_Elevation_size,
      sensorData_Range_data, sensorData_Range_size, sensorData_RangeRate_data,
      sensorData_RangeRate_size, sensorData_AzimuthAccuracy_data,
      sensorData_AzimuthAccuracy_size, c_sensorData_ElevationAccuracy_,
      d_sensorData_ElevationAccuracy_, sensorData_RangeAccuracy_data,
      sensorData_RangeAccuracy_size, c_sensorData_RangeRateAccuracy_,
      d_sensorData_RangeRateAccuracy_, z_data, z_size, modelData);
  st.site = &alb_emlrtRSI;
  c_AerospaceMonostaticRadar_time(
      sensorData_LookTime_data, sensorData_LookTime_size,
      sensorData_DetectionTime_data, sensorData_DetectionTime_size, t_data,
      t_size);
  if (z_size[1] > 0) {
    if (t_size[1] == 1) {
      st.site = &blb_emlrtRSI;
      b_st.site = &hq_emlrtRSI;
      t = t_data[0];
      idx = z_size[1];
      t_size[1] = z_size[1];
      for (b_k = 0; b_k < idx; b_k++) {
        t_data[b_k] = t;
      }
    } else {
      st.site = &clb_emlrtRSI;
      b_st.site = &ys_emlrtRSI;
      idx = 1;
      if (t_size[1] > 1) {
        idx = t_size[1];
      }
      if (z_size[1] > muIntScalarMax_sint32(t_size[1], idx)) {
        emlrtErrorWithMessageIdR2018a(
            &st, &p_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
            "Coder:toolbox:reshape_emptyReshapeLimit", 0);
      }
      if (z_size[1] != t_size[1]) {
        emlrtErrorWithMessageIdR2018a(
            &st, &q_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
            "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
      }
      t_size[1] = z_size[1];
    }
  } else {
    if (t_size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, t_size[1], &uj_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    t = t_data[0];
    t_size[1] = 1;
    t_data[0] = t;
  }
  i = trackList->size[0];
  if (i - 1 >= 0) {
    last = t_size[1];
    b_t_size[0] = 1;
    b_t_size[1] = t_size[1];
    scalarLB = (t_size[1] / 2) << 1;
    vectorUB = scalarLB - 2;
  }
  for (k = 0; k < i; k++) {
    real_T b_assignment_data[51];
    real_T d;
    int32_T i1;
    int32_T loop_ub;
    boolean_T exitg1;
    st.site = &dlb_emlrtRSI;
    i1 = trackList->size[0];
    if (k + 1 > i1) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1, &vj_emlrtBCI, &st);
    }
    if (k + 1 > assignment->size[1]) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, assignment->size[1], &wj_emlrtBCI,
                                    &st);
    }
    b_st.site = &elb_emlrtRSI;
    if (t_size[1] <= 2) {
      if (t_size[1] == 1) {
        t = t_data[0];
      } else if ((t_data[0] > t_data[1]) ||
                 (muDoubleScalarIsNaN(t_data[0]) &&
                  (!muDoubleScalarIsNaN(t_data[1])))) {
        t = t_data[1];
      } else {
        t = t_data[0];
      }
    } else {
      if (!muDoubleScalarIsNaN(t_data[0])) {
        idx = 1;
      } else {
        idx = 0;
        loop_ub = 2;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= last)) {
          if (!muDoubleScalarIsNaN(t_data[loop_ub - 1])) {
            idx = loop_ub;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
      }
      if (idx == 0) {
        t = t_data[0];
      } else {
        t = t_data[idx - 1];
        idx++;
        for (b_k = idx; b_k <= last; b_k++) {
          d = t_data[b_k - 1];
          if (t > d) {
            t = d;
          }
        }
      }
    }
    track = trackList_data[k];
    c_st.site = &ilb_emlrtRSI;
    TrackEstimator_predict(&c_st,
                           &c_updater_Estimator_StateEstima->StateEstimator
                                .Estimators.f1.TargetSpecifications[0],
                           c_updater_Estimator_StateEstima->StateEstimator
                               .Estimators.f1.TrackingFilter,
                           &c_updater_Estimator_StateEstima->StateEstimator
                                .Estimators.f2.TargetSpecifications[0],
                           c_updater_Estimator_StateEstima->StateEstimator
                               .Estimators.f2.TrackingFilter,
                           &c_updater_Estimator_StateEstima->StateEstimator
                                .Estimators.f3.TargetSpecifications[0],
                           c_updater_Estimator_StateEstima->StateEstimator
                               .Estimators.f3.TrackingFilter,
                           &track, t);
    if (t_size[1] <= 2) {
      if (t_size[1] == 1) {
        t = t_data[0];
      } else if ((t_data[0] > t_data[1]) ||
                 (muDoubleScalarIsNaN(t_data[0]) &&
                  (!muDoubleScalarIsNaN(t_data[1])))) {
        t = t_data[1];
      } else {
        t = t_data[0];
      }
    } else {
      if (!muDoubleScalarIsNaN(t_data[0])) {
        idx = 1;
      } else {
        idx = 0;
        loop_ub = 2;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= last)) {
          if (!muDoubleScalarIsNaN(t_data[loop_ub - 1])) {
            idx = loop_ub;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
      }
      if (idx == 0) {
        t = t_data[0];
      } else {
        t = t_data[idx - 1];
        idx++;
        for (b_k = idx; b_k <= last; b_k++) {
          d = t_data[b_k - 1];
          if (t > d) {
            t = d;
          }
        }
      }
    }
    for (b_k = 0; b_k <= vectorUB; b_k += 2) {
      __m128d r;
      r = _mm_loadu_pd(&t_data[b_k]);
      _mm_storeu_pd(&b_t_data[b_k], _mm_sub_pd(r, _mm_set1_pd(t)));
    }
    for (b_k = scalarLB; b_k < last; b_k++) {
      b_t_data[b_k] = t_data[b_k] - t;
    }
    loop_ub = assignment->size[0];
    idx = assignment->size[0];
    for (b_k = 0; b_k < loop_ub; b_k++) {
      b_assignment_data[b_k] = assignment_data[b_k + assignment->size[0] * k];
    }
    SD->u3.f9.updater_Estimator_StateEstimato =
        *c_updater_Estimator_StateEstima;
    c_st.site = &glb_emlrtRSI;
    IPDAEstimator_correctJPDA(
        SD, &c_st, &SD->u3.f9.updater_Estimator_StateEstimato, &track, z_data,
        z_size, b_t_data, b_t_size, modelData, b_assignment_data, idx,
        updater_AssignmentThreshold);
    if (assignment->size[0] == 1) {
      track.IsCoasted = true;
    } else {
      int32_T b_iv[2];
      if ((assignment->size[0] - 1 < 1) ||
          (assignment->size[0] - 1 > assignment->size[0])) {
        emlrtDynamicBoundsCheckR2012b(assignment->size[0] - 1, 1,
                                      assignment->size[0], &xj_emlrtBCI, &b_st);
      }
      b_iv[0] = 1;
      b_iv[1] = assignment->size[0] - 1;
      c_st.site = &flb_emlrtRSI;
      b_indexShapeCheck(&c_st, assignment->size[0], b_iv);
      assignment_size = assignment->size[0] - 1;
      for (b_k = 0; b_k <= loop_ub - 2; b_k++) {
        c_assignment_data[b_k] = assignment_data[b_k + assignment->size[0] * k];
      }
      d_assignment_data.data = &c_assignment_data[0];
      d_assignment_data.size = &assignment_size;
      d_assignment_data.allocatedSize = 50;
      d_assignment_data.numDimensions = 1;
      d_assignment_data.canFreeData = false;
      c_st.site = &flb_emlrtRSI;
      if (!(c_sum(&c_st, &d_assignment_data) > 0.0)) {
        track.IsCoasted = true;
      } else {
        uint32_T qY;
        track.IsCoasted = false;
        qY = track.Age + 1U;
        if (track.Age + 1U < track.Age) {
          qY = MAX_uint32_T;
        }
        track.Age = qY;
      }
    }
    if (k + 1 > i1) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, i1, &tj_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    trackList_data[k] = track;
  }
  emxFree_struct_T1(sp, &modelData);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (JIPDATrackUpdater.c) */
