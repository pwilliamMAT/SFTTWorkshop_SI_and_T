/*
 * _coder_trackingAlgorithm_api.c
 *
 * Code generation for function '_coder_trackingAlgorithm_api'
 *
 */

/* Include files */
#include "_coder_trackingAlgorithm_api.h"
#include "rt_nonfinite.h"
#include "trackingAlgorithm.h"
#include "trackingAlgorithm_data.h"
#include "trackingAlgorithm_emxutil.h"
#include "trackingAlgorithm_types.h"

/* Variable Definitions */
static emlrtRTEInfo li_emlrtRTEI = {
    1,                              /* lineNo */
    1,                              /* colNo */
    "_coder_trackingAlgorithm_api", /* fName */
    ""                              /* pName */
};

/* Function Declarations */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                d_fusion_tracker_detectability_ *y);

static void ac_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, char_T ret[3]);

static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                c_fusion_tracker_detectability_ y[108]);

static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                c_fusion_tracker_detectability_ *y);

static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                real_T y[2]);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct0_T *y);

static int32_T eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const emxArray_struct1_T *u);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T *y);

static c_fusion_tracker_birth_UniformP
fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                    const emlrtMsgIdentifier *parentId);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T *y);

static c_fusion_tracker_clutter_Unifor
gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                    const emlrtMsgIdentifier *parentId);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2]);

static void hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2]);

static void ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                boolean_T y[5]);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, cell_4 *y);

static char_T jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, cell_4 *y);

static void kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                char_T y[3]);

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_targetspecs_Pa *y);

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_transition_Con *y);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static void nb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret_data[], int32_T ret_size[2]);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9]);

static void ob_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret_data[], int32_T ret_size[2]);

static c_fusion_tracker_survival_Unifo
p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                   const emlrtMsgIdentifier *parentId);

static void pb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[3]);

static real_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static void qb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[9]);

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               boolean_T y[2]);

static real_T rb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId);

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void sb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                boolean_T ret[2]);

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_targetspecs_Ge *y);

static void tb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId);

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_targetspecs_He *y);

static void ub_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[27]);

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier,
                               c_fusion_tracker_sensorspecs_Ae *y);

static void vb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[2]);

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_sensorspecs_Ae *y);

static int32_T wb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static void x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_measurement_Az *y);

static void xb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                boolean_T ret[5]);

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[27]);

static char_T yb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                d_fusion_tracker_detectability_ *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[2];
  const char_T *propClasses[2] = {
      "fusion.tracker.detectability.CompositeFieldOfViewModel",
      "fusion.tracker.detectability.CompositeFieldOfViewModel"};
  const char_T *propNames[2] = {"FieldsOfView", "NumModels"};
  propValues[0] = NULL;
  propValues[1] = NULL;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(
      (emlrtCTX)sp, parentId, u,
      "fusion.tracker.detectability.CompositeFieldOfViewModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 2, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "FieldsOfView";
  bb_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId, y->FieldsOfView);
  thisId.fIdentifier = "NumModels";
  y->NumModels = eb_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId);
  emlrtDestroyArrays(2, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void ac_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, char_T ret[3])
{
  static const int32_T dims[2] = {1, 3};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 3);
  emlrtDestroyArray(&src);
}

static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                c_fusion_tracker_detectability_ y[108])
{
  emlrtMsgIdentifier thisId;
  int32_T i;
  int32_T i1;
  char_T str[11];
  boolean_T b;
  thisId.fParent = parentId;
  thisId.bParentIsCell = true;
  b = false;
  i = 108;
  emlrtCheckCell((emlrtCTX)sp, parentId, u, 1U, &i, &b);
  for (i1 = 0; i1 < 108; i1++) {
    emlrtMexSnprintf(&str[0], (size_t)11U, "%d", i1 + 1);
    thisId.fIdentifier = &str[0];
    cb_emlrt_marshallIn(sp,
                        emlrtAlias(emlrtGetCell((emlrtCTX)sp, parentId, u, i1)),
                        &thisId, &y[i1]);
  }
  emlrtDestroyArray(&u);
}

static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                c_fusion_tracker_detectability_ *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[9];
  int32_T i;
  const char_T *propClasses[9] = {
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel",
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel"};
  const char_T *propNames[9] = {
      "OriginPosition",  "OriginVelocity",       "Orientation",
      "AzimuthLimits",   "ElevationLimits",      "RangeLimits",
      "RangeRateLimits", "DetectionProbability", "MinDetectionProbability"};
  for (i = 0; i < 9; i++) {
    propValues[i] = NULL;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(
      (emlrtCTX)sp, parentId, u,
      "fusion.tracker.detectability.FieldOfViewAndRangeRateModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 9, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "OriginPosition";
  o_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId, y->OriginPosition);
  thisId.fIdentifier = "OriginVelocity";
  o_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId, y->OriginVelocity);
  thisId.fIdentifier = "Orientation";
  y_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId, y->Orientation);
  thisId.fIdentifier = "AzimuthLimits";
  db_emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId, y->AzimuthLimits);
  thisId.fIdentifier = "ElevationLimits";
  db_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId,
                      y->ElevationLimits);
  thisId.fIdentifier = "RangeLimits";
  db_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId, y->RangeLimits);
  thisId.fIdentifier = "RangeRateLimits";
  db_emlrt_marshallIn(sp, emlrtAlias(propValues[6]), &thisId,
                      y->RangeRateLimits);
  thisId.fIdentifier = "DetectionProbability";
  y->DetectionProbability =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[7]), &thisId);
  thisId.fIdentifier = "MinDetectionProbability";
  y->MinDetectionProbability =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[8]), &thisId);
  emlrtDestroyArrays(9, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId, real_T y[2])
{
  vb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static int32_T eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = wb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const emxArray_struct1_T *u)
{
  static const int32_T b_iv[2] = {6, 6};
  static const int32_T b_iv1[2] = {1, 3};
  static const int32_T iv2[2] = {1, 10};
  static const int32_T b_i = 6;
  static const char_T *sv[16] = {"TrackID",
                                 "BranchID",
                                 "SourceIndex",
                                 "UpdateTime",
                                 "Age",
                                 "State",
                                 "StateCovariance",
                                 "StateParameters",
                                 "ObjectClassID",
                                 "ObjectClassProbabilities",
                                 "TrackLogic",
                                 "TrackLogicState",
                                 "IsConfirmed",
                                 "IsCoasted",
                                 "IsSelfReported",
                                 "ObjectAttributes"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *j_y;
  const mxArray *k_y;
  const mxArray *l_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *n_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *q_y;
  const mxArray *y;
  const struct1_T *u_data;
  real_T *pData;
  int32_T b_j0;
  int32_T c_i;
  int32_T d_i;
  int32_T i;
  u_data = u->data;
  y = NULL;
  emlrtAssign(
      &y, emlrtCreateStructArray(1, &u->size[0], 16, (const char_T **)&sv[0]));
  emlrtCreateField(y, "TrackID");
  emlrtCreateField(y, "BranchID");
  emlrtCreateField(y, "SourceIndex");
  emlrtCreateField(y, "UpdateTime");
  emlrtCreateField(y, "Age");
  emlrtCreateField(y, "State");
  emlrtCreateField(y, "StateCovariance");
  emlrtCreateField(y, "StateParameters");
  emlrtCreateField(y, "ObjectClassID");
  emlrtCreateField(y, "ObjectClassProbabilities");
  emlrtCreateField(y, "TrackLogic");
  emlrtCreateField(y, "TrackLogicState");
  emlrtCreateField(y, "IsConfirmed");
  emlrtCreateField(y, "IsCoasted");
  emlrtCreateField(y, "IsSelfReported");
  emlrtCreateField(y, "ObjectAttributes");
  i = 0;
  for (b_j0 = 0; b_j0 < u->size[0U]; b_j0++) {
    int32_T i1;
    b_y = NULL;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)emlrtMxGetData(m) = u_data[b_j0].TrackID;
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "TrackID", b_y, 0);
    c_y = NULL;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)emlrtMxGetData(m) = u_data[b_j0].BranchID;
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(y, i, "BranchID", c_y, 1);
    d_y = NULL;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)emlrtMxGetData(m) = u_data[b_j0].SourceIndex;
    emlrtAssign(&d_y, m);
    emlrtSetFieldR2017b(y, i, "SourceIndex", d_y, 2);
    e_y = NULL;
    m = emlrtCreateDoubleScalar(u_data[b_j0].UpdateTime);
    emlrtAssign(&e_y, m);
    emlrtSetFieldR2017b(y, i, "UpdateTime", e_y, 3);
    f_y = NULL;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)emlrtMxGetData(m) = u_data[b_j0].Age;
    emlrtAssign(&f_y, m);
    emlrtSetFieldR2017b(y, i, "Age", f_y, 4);
    g_y = NULL;
    m = emlrtCreateNumericArray(1, (const void *)&b_i, mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    for (c_i = 0; c_i < 6; c_i++) {
      pData[c_i] = u_data[b_j0].State[c_i];
    }
    emlrtAssign(&g_y, m);
    emlrtSetFieldR2017b(y, i, "State", g_y, 5);
    h_y = NULL;
    m = emlrtCreateNumericArray(2, (const void *)&b_iv[0], mxDOUBLE_CLASS,
                                mxREAL);
    pData = emlrtMxGetPr(m);
    i1 = 0;
    for (c_i = 0; c_i < 6; c_i++) {
      for (d_i = 0; d_i < 6; d_i++) {
        pData[i1 + d_i] = u_data[b_j0].StateCovariance[d_i + 6 * c_i];
      }
      i1 += 6;
    }
    emlrtAssign(&h_y, m);
    emlrtSetFieldR2017b(y, i, "StateCovariance", h_y, 6);
    i_y = NULL;
    emlrtAssign(&i_y, emlrtCreateStructMatrix(1, 1, 0, NULL));
    emlrtSetFieldR2017b(y, i, "StateParameters", i_y, 7);
    j_y = NULL;
    m = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    *(int32_T *)emlrtMxGetData(m) = u_data[b_j0].ObjectClassID;
    emlrtAssign(&j_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassID", j_y, 8);
    k_y = NULL;
    m = emlrtCreateNumericArray(2, (const void *)&b_iv1[0], mxDOUBLE_CLASS,
                                mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = u_data[b_j0].ObjectClassProbabilities[0];
    pData[1] = u_data[b_j0].ObjectClassProbabilities[1];
    pData[2] = u_data[b_j0].ObjectClassProbabilities[2];
    emlrtAssign(&k_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassProbabilities", k_y, 9);
    l_y = NULL;
    m = emlrtCreateCharArray(2, &iv2[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 10, m,
                             &u_data[b_j0].TrackLogic[0]);
    emlrtAssign(&l_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogic", l_y, 10);
    m_y = NULL;
    m = emlrtCreateDoubleScalar(u_data[b_j0].TrackLogicState);
    emlrtAssign(&m_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogicState", m_y, 11);
    n_y = NULL;
    m = emlrtCreateLogicalScalar(u_data[b_j0].IsConfirmed);
    emlrtAssign(&n_y, m);
    emlrtSetFieldR2017b(y, i, "IsConfirmed", n_y, 12);
    o_y = NULL;
    m = emlrtCreateLogicalScalar(u_data[b_j0].IsCoasted);
    emlrtAssign(&o_y, m);
    emlrtSetFieldR2017b(y, i, "IsCoasted", o_y, 13);
    p_y = NULL;
    m = emlrtCreateLogicalScalar(u_data[b_j0].IsSelfReported);
    emlrtAssign(&p_y, m);
    emlrtSetFieldR2017b(y, i, "IsSelfReported", p_y, 14);
    q_y = NULL;
    emlrtAssign(&q_y, emlrtCreateStructMatrix(1, 1, 0, NULL));
    emlrtSetFieldR2017b(y, i, "ObjectAttributes", q_y, 15);
    i++;
  }
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  int32_T i;
  char_T str[11];
  boolean_T b;
  thisId.fParent = parentId;
  thisId.bParentIsCell = true;
  b = false;
  i = 1;
  emlrtCheckCell((emlrtCTX)sp, parentId, u, 1U, &i, &b);
  emlrtMexSnprintf(&str[0], (size_t)11U, "%d", 1);
  thisId.fIdentifier = &str[0];
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell((emlrtCTX)sp, parentId, u, 0)),
                     &thisId, y);
  emlrtDestroyArray(&u);
}

static c_fusion_tracker_birth_UniformP
fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                    const emlrtMsgIdentifier *parentId)
{
  c_fusion_tracker_birth_UniformP y;
  emlrtMsgIdentifier thisId;
  const mxArray *propValues;
  const char_T *propClasses = "fusion.tracker.birth.UniformPoissonModel";
  const char_T *propNames = "BirthDensity";
  propValues = NULL;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.birth.UniformPoissonModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 1, (const char_T **)&propNames,
                        (const char_T **)&propClasses, &propValues);
  thisId.fIdentifier = "BirthDensity";
  y.BirthDensity = q_emlrt_marshallIn(sp, emlrtAlias(propValues), &thisId);
  emlrtDestroyArrays(1, &propValues);
  emlrtDestroyArray(&u);
  return y;
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct0_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[12] = {
      "LookTime",        "LookAzimuth",
      "LookElevation",   "DetectionTime",
      "Azimuth",         "Elevation",
      "Range",           "RangeRate",
      "AzimuthAccuracy", "ElevationAccuracy",
      "RangeAccuracy",   "RangeRateAccuracy"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 12,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "LookTime";
  h_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "LookTime")),
      &thisId, y->LookTime.data, y->LookTime.size);
  thisId.fIdentifier = "LookAzimuth";
  h_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1,
                                                    "LookAzimuth")),
                     &thisId, y->LookAzimuth.data, y->LookAzimuth.size);
  thisId.fIdentifier = "LookElevation";
  h_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 2,
                                                    "LookElevation")),
                     &thisId, y->LookElevation.data, y->LookElevation.size);
  thisId.fIdentifier = "DetectionTime";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 3,
                                                    "DetectionTime")),
                     &thisId, y->DetectionTime.data, y->DetectionTime.size);
  thisId.fIdentifier = "Azimuth";
  i_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 4, "Azimuth")),
      &thisId, y->Azimuth.data, y->Azimuth.size);
  thisId.fIdentifier = "Elevation";
  i_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 5, "Elevation")),
      &thisId, y->Elevation.data, y->Elevation.size);
  thisId.fIdentifier = "Range";
  i_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 6, "Range")),
      &thisId, y->Range.data, y->Range.size);
  thisId.fIdentifier = "RangeRate";
  i_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 7, "RangeRate")),
      &thisId, y->RangeRate.data, y->RangeRate.size);
  thisId.fIdentifier = "AzimuthAccuracy";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 8,
                                                    "AzimuthAccuracy")),
                     &thisId, y->AzimuthAccuracy.data, y->AzimuthAccuracy.size);
  thisId.fIdentifier = "ElevationAccuracy";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 9,
                                                    "ElevationAccuracy")),
                     &thisId, y->ElevationAccuracy.data,
                     y->ElevationAccuracy.size);
  thisId.fIdentifier = "RangeAccuracy";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 10,
                                                    "RangeAccuracy")),
                     &thisId, y->RangeAccuracy.data, y->RangeAccuracy.size);
  thisId.fIdentifier = "RangeRateAccuracy";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 11,
                                                    "RangeRateAccuracy")),
                     &thisId, y->RangeRateAccuracy.data,
                     y->RangeRateAccuracy.size);
  emlrtDestroyArray(&u);
}

static c_fusion_tracker_clutter_Unifor
gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                    const emlrtMsgIdentifier *parentId)
{
  c_fusion_tracker_clutter_Unifor y;
  emlrtMsgIdentifier thisId;
  const mxArray *propValues;
  const char_T *propClasses = "fusion.tracker.clutter.UniformPoissonModel";
  const char_T *propNames = "ClutterDensity";
  propValues = NULL;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.clutter.UniformPoissonModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 1, (const char_T **)&propNames,
                        (const char_T **)&propClasses, &propValues);
  thisId.fIdentifier = "ClutterDensity";
  y.ClutterDensity = q_emlrt_marshallIn(sp, emlrtAlias(propValues), &thisId);
  emlrtDestroyArrays(1, &propValues);
  emlrtDestroyArray(&u);
  return y;
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2])
{
  nb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId)
{
  emlrtCheckMcosClass2017a(
      (emlrtCTX)sp, parentId, u,
      "fusion.tracker.internal.stateInitiator.GaussianStateInitiator");
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2])
{
  ob_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                boolean_T y[5])
{
  xb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, cell_4 *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  k_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static char_T jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId)
{
  char_T y;
  y = yb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, cell_4 *y)
{
  emlrtMsgIdentifier thisId;
  int32_T b_iv[2];
  boolean_T bv[2];
  thisId.fParent = parentId;
  thisId.bParentIsCell = true;
  bv[0] = false;
  b_iv[0] = 1;
  bv[1] = false;
  b_iv[1] = 3;
  emlrtCheckCell((emlrtCTX)sp, parentId, u, 2U, &b_iv[0], &bv[0]);
  thisId.fIdentifier = "1";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell((emlrtCTX)sp, parentId, u, 0)),
                     &thisId, &y->f1);
  thisId.fIdentifier = "2";
  t_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell((emlrtCTX)sp, parentId, u, 1)),
                     &thisId, &y->f2);
  thisId.fIdentifier = "3";
  u_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell((emlrtCTX)sp, parentId, u, 2)),
                     &thisId, &y->f3);
  emlrtDestroyArray(&u);
}

static void kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                const emlrtMsgIdentifier *parentId, char_T y[3])
{
  ac_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_targetspecs_Pa *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[8];
  int32_T i;
  const char_T *propClasses[8] = {
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification"};
  const char_T *propNames[8] = {"StateTransitionModel",
                                "SurvivalModel",
                                "IsLockedDataType",
                                "MaxHorizontalSpeedUnits",
                                "MaxVerticalSpeedUnits",
                                "MaxHorizontalAccelerationUnits",
                                "MaxVerticalAccelerationUnits",
                                "MotionModel"};
  for (i = 0; i < 8; i++) {
    propValues[i] = NULL;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.targetspecs.PassengerAircraft");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 8, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "StateTransitionModel";
  m_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId,
                     &y->StateTransitionModel);
  thisId.fIdentifier = "SurvivalModel";
  y->SurvivalModel = p_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId);
  thisId.fIdentifier = "IsLockedDataType";
  r_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId,
                     y->IsLockedDataType);
  thisId.fIdentifier = "MaxHorizontalSpeedUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId);
  thisId.fIdentifier = "MaxVerticalSpeedUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "MaxHorizontalAccelerationUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId);
  thisId.fIdentifier = "MaxVerticalAccelerationUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[6]), &thisId);
  thisId.fIdentifier = "MotionModel";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[7]), &thisId);
  emlrtDestroyArrays(8, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_transition_Con *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[3];
  const char_T *propClasses[3] = {
      "fusion.tracker.transition.ConstantVelocityModel",
      "fusion.tracker.transition.ConstantVelocityModel",
      "fusion.tracker.transition.ConstantVelocityModel"};
  const char_T *propNames[3] = {"PropVelocityMean", "PropVelocityVariance",
                                "PropAccelerationVariance"};
  propValues[0] = NULL;
  propValues[1] = NULL;
  propValues[2] = NULL;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.transition.ConstantVelocityModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 3, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "PropVelocityMean";
  n_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId,
                     y->PropVelocityMean);
  thisId.fIdentifier = "PropVelocityVariance";
  o_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId,
                     y->PropVelocityVariance);
  thisId.fIdentifier = "PropAccelerationVariance";
  o_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId,
                     y->PropAccelerationVariance);
  emlrtDestroyArrays(3, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  pb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void nb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2] = {1, 100};
  boolean_T bv[2] = {false, true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &ret_size[0]);
  emlrtImportArrayR2015b((emlrtConstCTX)sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9])
{
  qb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void ob_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2] = {1, 50};
  boolean_T bv[2] = {false, true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &ret_size[0]);
  emlrtImportArrayR2015b((emlrtConstCTX)sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

static c_fusion_tracker_survival_Unifo
p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                   const emlrtMsgIdentifier *parentId)
{
  c_fusion_tracker_survival_Unifo y;
  emlrtMsgIdentifier thisId;
  const mxArray *propValues;
  const char_T *propClasses =
      "fusion.tracker.survival.UniformSurvivalRateModel";
  const char_T *propNames = "SurvivalRate";
  propValues = NULL;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.survival.UniformSurvivalRateModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 1, (const char_T **)&propNames,
                        (const char_T **)&propClasses, &propValues);
  thisId.fIdentifier = "SurvivalRate";
  y.SurvivalRate = q_emlrt_marshallIn(sp, emlrtAlias(propValues), &thisId);
  emlrtDestroyArrays(1, &propValues);
  emlrtDestroyArray(&u);
  return y;
}

static void pb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims = 3;
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static real_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = rb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void qb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2] = {3, 3};
  real_T(*r)[9];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[9])emlrtMxGetData(src);
  for (i = 0; i < 9; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               boolean_T y[2])
{
  sb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T rb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  tb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static void sb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                boolean_T ret[2])
{
  static const int32_T dims[2] = {1, 2};
  boolean_T(*r)[2];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 2U,
                          (const void *)&dims[0]);
  r = (boolean_T(*)[2])emlrtMxGetLogicals(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  emlrtDestroyArray(&src);
}

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_targetspecs_Ge *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[8];
  int32_T i;
  const char_T *propClasses[8] = {
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification"};
  const char_T *propNames[8] = {"StateTransitionModel",
                                "SurvivalModel",
                                "IsLockedDataType",
                                "MaxHorizontalSpeedUnits",
                                "MaxVerticalSpeedUnits",
                                "MaxHorizontalAccelerationUnits",
                                "MaxVerticalAccelerationUnits",
                                "MotionModel"};
  for (i = 0; i < 8; i++) {
    propValues[i] = NULL;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.targetspecs.GeneralAviation");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 8, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "StateTransitionModel";
  m_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId,
                     &y->StateTransitionModel);
  thisId.fIdentifier = "SurvivalModel";
  y->SurvivalModel = p_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId);
  thisId.fIdentifier = "IsLockedDataType";
  r_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId,
                     y->IsLockedDataType);
  thisId.fIdentifier = "MaxHorizontalSpeedUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId);
  thisId.fIdentifier = "MaxVerticalSpeedUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "MaxHorizontalAccelerationUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId);
  thisId.fIdentifier = "MaxVerticalAccelerationUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[6]), &thisId);
  thisId.fIdentifier = "MotionModel";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[7]), &thisId);
  emlrtDestroyArrays(8, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void tb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims[2] = {0, 0};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_targetspecs_He *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[8];
  int32_T i;
  const char_T *propClasses[8] = {
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification",
      "fusion.tracker.targetspecs.internal.AircraftSpecification"};
  const char_T *propNames[8] = {"StateTransitionModel",
                                "SurvivalModel",
                                "IsLockedDataType",
                                "MaxHorizontalSpeedUnits",
                                "MaxVerticalSpeedUnits",
                                "MaxHorizontalAccelerationUnits",
                                "MaxVerticalAccelerationUnits",
                                "MotionModel"};
  for (i = 0; i < 8; i++) {
    propValues[i] = NULL;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)sp, parentId, u,
                           "fusion.tracker.targetspecs.Helicopter");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 8, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "StateTransitionModel";
  m_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId,
                     &y->StateTransitionModel);
  thisId.fIdentifier = "SurvivalModel";
  y->SurvivalModel = p_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId);
  thisId.fIdentifier = "IsLockedDataType";
  r_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId,
                     y->IsLockedDataType);
  thisId.fIdentifier = "MaxHorizontalSpeedUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId);
  thisId.fIdentifier = "MaxVerticalSpeedUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "MaxHorizontalAccelerationUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId);
  thisId.fIdentifier = "MaxVerticalAccelerationUnits";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[6]), &thisId);
  thisId.fIdentifier = "MotionModel";
  s_emlrt_marshallIn(sp, emlrtAlias(propValues[7]), &thisId);
  emlrtDestroyArrays(8, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void ub_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[27])
{
  static const int32_T dims[3] = {3, 3, 3};
  real_T(*r)[27];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 3U,
                          (const void *)&dims[0]);
  r = (real_T(*)[27])emlrtMxGetData(src);
  for (i = 0; i < 27; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier,
                               c_fusion_tracker_sensorspecs_Ae *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  w_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static void vb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[2])
{
  static const int32_T dims[2] = {1, 2};
  real_T(*r)[2];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[2])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  emlrtDestroyArray(&src);
}

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_sensorspecs_Ae *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[22];
  int32_T i;
  const char_T *propClasses[22] = {
      "fusion.tracker.sensorspecs.internal.BuiltinSensorSpecification",
      "fusion.tracker.sensorspecs.internal.BuiltinSensorSpecification",
      "fusion.tracker.sensorspecs.internal.BuiltinSensorSpecification",
      "fusion.tracker.sensorspecs.internal.BuiltinSensorSpecification",
      "fusion.tracker.sensorspecs.internal.BuiltinSensorSpecification",
      "fusion.tracker.sensorspecs.internal.BuiltinSensorSpecification",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar"};
  const char_T *propNames[22] = {"MeasurementModel",
                                 "DetectabilityModel",
                                 "BirthModel",
                                 "ClutterModel",
                                 "StateInitiator",
                                 "IsLockedDataType",
                                 "AzimuthResolution",
                                 "ElevationResolution",
                                 "RangeResolution",
                                 "RangeRateResolution",
                                 "FalseAlarmRate",
                                 "BirthRate",
                                 "MountingLocationUnits",
                                 "MountingAnglesUnits",
                                 "PlatformPositionUnits",
                                 "FieldOfViewUnits",
                                 "RangeLimitsUnits",
                                 "RangeRateLimitsUnits",
                                 "AzimuthResolutionUnits",
                                 "ElevationResolutionUnits",
                                 "RangeResolutionUnits",
                                 "RangeRateResolutionUnits"};
  for (i = 0; i < 22; i++) {
    propValues[i] = NULL;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(
      (emlrtCTX)sp, parentId, u,
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 22, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "MeasurementModel";
  x_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId,
                     &y->MeasurementModel);
  thisId.fIdentifier = "DetectabilityModel";
  ab_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId,
                      &y->DetectabilityModel);
  thisId.fIdentifier = "BirthModel";
  y->BirthModel = fb_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId);
  thisId.fIdentifier = "ClutterModel";
  y->ClutterModel = gb_emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId);
  thisId.fIdentifier = "StateInitiator";
  hb_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "IsLockedDataType";
  ib_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId,
                      y->IsLockedDataType);
  thisId.fIdentifier = "AzimuthResolution";
  y->AzimuthResolution =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[6]), &thisId);
  thisId.fIdentifier = "ElevationResolution";
  y->ElevationResolution =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[7]), &thisId);
  thisId.fIdentifier = "RangeResolution";
  y->RangeResolution =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[8]), &thisId);
  thisId.fIdentifier = "RangeRateResolution";
  y->RangeRateResolution =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[9]), &thisId);
  thisId.fIdentifier = "FalseAlarmRate";
  y->FalseAlarmRate =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[10]), &thisId);
  thisId.fIdentifier = "BirthRate";
  y->BirthRate = q_emlrt_marshallIn(sp, emlrtAlias(propValues[11]), &thisId);
  thisId.fIdentifier = "MountingLocationUnits";
  y->MountingLocationUnits =
      jb_emlrt_marshallIn(sp, emlrtAlias(propValues[12]), &thisId);
  thisId.fIdentifier = "MountingAnglesUnits";
  kb_emlrt_marshallIn(sp, emlrtAlias(propValues[13]), &thisId,
                      y->MountingAnglesUnits);
  thisId.fIdentifier = "PlatformPositionUnits";
  y->PlatformPositionUnits =
      jb_emlrt_marshallIn(sp, emlrtAlias(propValues[14]), &thisId);
  thisId.fIdentifier = "FieldOfViewUnits";
  kb_emlrt_marshallIn(sp, emlrtAlias(propValues[15]), &thisId,
                      y->FieldOfViewUnits);
  thisId.fIdentifier = "RangeLimitsUnits";
  y->RangeLimitsUnits =
      jb_emlrt_marshallIn(sp, emlrtAlias(propValues[16]), &thisId);
  thisId.fIdentifier = "RangeRateLimitsUnits";
  kb_emlrt_marshallIn(sp, emlrtAlias(propValues[17]), &thisId,
                      y->RangeRateLimitsUnits);
  thisId.fIdentifier = "AzimuthResolutionUnits";
  kb_emlrt_marshallIn(sp, emlrtAlias(propValues[18]), &thisId,
                      y->AzimuthResolutionUnits);
  thisId.fIdentifier = "ElevationResolutionUnits";
  kb_emlrt_marshallIn(sp, emlrtAlias(propValues[19]), &thisId,
                      y->ElevationResolutionUnits);
  thisId.fIdentifier = "RangeResolutionUnits";
  y->RangeResolutionUnits =
      jb_emlrt_marshallIn(sp, emlrtAlias(propValues[20]), &thisId);
  thisId.fIdentifier = "RangeRateResolutionUnits";
  kb_emlrt_marshallIn(sp, emlrtAlias(propValues[21]), &thisId,
                      y->RangeRateResolutionUnits);
  emlrtDestroyArrays(22, &propValues[0]);
  emlrtDestroyArray(&u);
}

static int32_T wb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  int32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "int32", false, 0U,
                          (const void *)&dims);
  ret = *(int32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               c_fusion_tracker_measurement_Az *y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[7];
  int32_T i;
  const char_T *propClasses[7] = {
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel",
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel",
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel",
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel",
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel",
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel",
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel"};
  const char_T *propNames[7] = {"OriginPosition",    "OriginVelocity",
                                "Orientation",       "AzimuthVariance",
                                "ElevationVariance", "RangeVariance",
                                "RangeRateVariance"};
  for (i = 0; i < 7; i++) {
    propValues[i] = NULL;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(
      (emlrtCTX)sp, parentId, u,
      "fusion.tracker.measurement.AzimuthElevationRangeAndRangeRateModel");
  emlrtGetAllProperties((emlrtCTX)sp, u, 0, 7, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "OriginPosition";
  o_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId, y->OriginPosition);
  thisId.fIdentifier = "OriginVelocity";
  o_emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId, y->OriginVelocity);
  thisId.fIdentifier = "Orientation";
  y_emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId, y->Orientation);
  thisId.fIdentifier = "AzimuthVariance";
  y->AzimuthVariance =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId);
  thisId.fIdentifier = "ElevationVariance";
  y->ElevationVariance =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "RangeVariance";
  y->RangeVariance = q_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId);
  thisId.fIdentifier = "RangeRateVariance";
  y->RangeRateVariance =
      q_emlrt_marshallIn(sp, emlrtAlias(propValues[6]), &thisId);
  emlrtDestroyArrays(7, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void xb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                boolean_T ret[5])
{
  static const int32_T dims[2] = {1, 5};
  int32_T i;
  boolean_T(*r)[5];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 2U,
                          (const void *)&dims[0]);
  r = (boolean_T(*)[5])emlrtMxGetLogicals(src);
  for (i = 0; i < 5; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[27])
{
  ub_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static char_T yb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  char_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 0U,
                          (const void *)&dims);
  emlrtImportCharR2015b((emlrtCTX)sp, src, &ret);
  emlrtDestroyArray(&src);
  return ret;
}

void trackingAlgorithm_api(trackingAlgorithmStackData *SD,
                           const mxArray *const prhs[3], const mxArray **plhs)
{
  static const int32_T b_iv[28] = {0, 0, 3, 3, 4, 3, 4, 5, 5, 0, 1, 3, 3, 4,
                                   3, 4, 5, 5, 0, 2, 3, 3, 4, 3, 4, 5, 5, 5};
  static const int32_T b_iv1[11] = {3, 4, 3, 4, 3, 4, 3, 4, 3, 4, 5};
  static const uint32_T uv[24] = {
      4205784331U, 2264327993U, 829637541U,  2033545091U, 2771799987U,
      1170139851U, 3519338680U, 321885583U,  4205784331U, 2264327993U,
      829637541U,  2033545091U, 2771799987U, 1170139851U, 3519338680U,
      321885583U,  4205784331U, 2264327993U, 829637541U,  2033545091U,
      2771799987U, 1170139851U, 3519338680U, 321885583U};
  static const uint32_T uv1[20] = {
      1605257538U, 1258761973U, 733959435U,  3705873537U, 1379285247U,
      1405699837U, 3074730638U, 1915819171U, 2771799987U, 1170139851U,
      3519338680U, 321885583U,  2771799987U, 1170139851U, 3519338680U,
      321885583U,  2771799987U, 1170139851U, 3519338680U, 321885583U};
  static const char_T *sv[36] = {
      "targetSpec{1}",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "StateTransitionModel",
      "targetSpec{1}.StateTransitionModel",
      "fusion.tracker.transition.ConstantVelocityModel",
      "NumMotionDimensions",
      "targetSpec{1}.StateTransitionModel.NumMotionDimensions",
      "targetSpec{1}.StateTransitionModel.NumMotionDimensions",
      "fusion.tracker.transition.ConstantVelocityModel",
      "IsLocked",
      "targetSpec{1}.StateTransitionModel.IsLocked",
      "targetSpec{1}.StateTransitionModel.IsLocked",
      "targetSpec{2}",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "StateTransitionModel",
      "targetSpec{2}.StateTransitionModel",
      "fusion.tracker.transition.ConstantVelocityModel",
      "NumMotionDimensions",
      "targetSpec{2}.StateTransitionModel.NumMotionDimensions",
      "targetSpec{2}.StateTransitionModel.NumMotionDimensions",
      "fusion.tracker.transition.ConstantVelocityModel",
      "IsLocked",
      "targetSpec{2}.StateTransitionModel.IsLocked",
      "targetSpec{2}.StateTransitionModel.IsLocked",
      "targetSpec{3}",
      "fusion.tracker.targetspecs.internal.BuiltinTargetSpecification",
      "StateTransitionModel",
      "targetSpec{3}.StateTransitionModel",
      "fusion.tracker.transition.ConstantVelocityModel",
      "NumMotionDimensions",
      "targetSpec{3}.StateTransitionModel.NumMotionDimensions",
      "targetSpec{3}.StateTransitionModel.NumMotionDimensions",
      "fusion.tracker.transition.ConstantVelocityModel",
      "IsLocked",
      "targetSpec{3}.StateTransitionModel.IsLocked",
      "targetSpec{3}.StateTransitionModel.IsLocked"};
  static const char_T *sv1[20] = {
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "MaxNumLooksPerUpdate",
      "sensorSpec.MaxNumLooksPerUpdate",
      "sensorSpec.MaxNumLooksPerUpdate",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "MaxNumMeasurementsPerUpdate",
      "sensorSpec.MaxNumMeasurementsPerUpdate",
      "sensorSpec.MaxNumMeasurementsPerUpdate",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "IsPlatformStationary",
      "sensorSpec.IsPlatformStationary",
      "sensorSpec.IsPlatformStationary",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "HasElevation",
      "sensorSpec.HasElevation",
      "sensorSpec.HasElevation",
      "fusion.tracker.sensorspecs.AerospaceMonostaticRadar",
      "HasRangeRate",
      "sensorSpec.HasRangeRate",
      "sensorSpec.HasRangeRate"};
  cell_4 targetSpec;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  emxArray_struct1_T *tracks;
  struct0_T dets;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  /* Check constant function inputs */
  emlrtCheckArrayChecksumR2018b(&st, prhs[1], false, &b_iv[0],
                                (const char_T **)&sv[0], &uv[0]);
  emlrtCheckArrayChecksumR2018b(&st, prhs[2], false, &b_iv1[0],
                                (const char_T **)&sv1[0], &uv1[0]);
  /* Marshall function inputs */
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "dets", &dets);
  j_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "targetSpec", &targetSpec);
  v_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "sensorSpec",
                     &SD->f15.sensorSpec);
  /* Invoke the target function */
  emxInit_struct1_T(&st, &tracks, &li_emlrtRTEI);
  trackingAlgorithm(SD, &st, &dets, &targetSpec, &SD->f15.sensorSpec, tracks);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(&st, tracks);
  emxFree_struct1_T(&st, &tracks);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_trackingAlgorithm_api.c) */
