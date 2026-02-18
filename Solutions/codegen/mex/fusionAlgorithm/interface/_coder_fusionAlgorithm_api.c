/*
 * _coder_fusionAlgorithm_api.c
 *
 * Code generation for function '_coder_fusionAlgorithm_api'
 *
 */

/* Include files */
#include "_coder_fusionAlgorithm_api.h"
#include "fusionAlgorithm.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_emxutil.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRTEInfo ug_emlrtRTEI = {
    1,                            /* lineNo */
    1,                            /* colNo */
    "_coder_fusionAlgorithm_api", /* fName */
    ""                            /* pName */
};

/* Function Declarations */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, emxArray_struct0_T *y);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_struct0_T *y);

static uint32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct2_T u_data[],
                                        const int32_T u_size);

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[6]);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[36]);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2]);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               char_T y[10]);

static boolean_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static struct1_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[8]);

static adsbCategory o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                       const emlrtMsgIdentifier *parentId);

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier);

static uint32_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static real_T s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6]);

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[36]);

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2]);

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[10]);

static boolean_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[8]);

/* Function Definitions */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, emxArray_struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_struct0_T *y)
{
  static const int32_T dims = 200;
  static const char_T *fieldNames[16] = {"TrackID",
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
  emlrtMsgIdentifier thisId;
  struct0_T *y_data;
  int32_T i;
  int32_T sizes;
  boolean_T b = true;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckVsStructR2012b((emlrtCTX)sp, parentId, u, 16,
                           (const char_T **)&fieldNames[0], 1U,
                           (const void *)&dims, &b, &sizes);
  i = y->size[0];
  y->size[0] = sizes;
  emxEnsureCapacity_struct0_T(sp, y, i, (emlrtRTEInfo *)NULL);
  y_data = y->data;
  for (i = 0; i < sizes; i++) {
    thisId.fIdentifier = "TrackID";
    y_data[i].TrackID = e_emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i, 0, "TrackID")),
        &thisId);
    thisId.fIdentifier = "BranchID";
    y_data[i].BranchID = e_emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i, 1, "BranchID")),
        &thisId);
    thisId.fIdentifier = "SourceIndex";
    y_data[i].SourceIndex =
        e_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u,
                                                          i, 2, "SourceIndex")),
                           &thisId);
    thisId.fIdentifier = "UpdateTime";
    y_data[i].UpdateTime =
        f_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u,
                                                          i, 3, "UpdateTime")),
                           &thisId);
    thisId.fIdentifier = "Age";
    y_data[i].Age = e_emlrt_marshallIn(
        sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i, 4, "Age")),
        &thisId);
    thisId.fIdentifier = "State";
    g_emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i, 5, "State")),
        &thisId, y_data[i].State);
    thisId.fIdentifier = "StateCovariance";
    h_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i,
                                                      6, "StateCovariance")),
                       &thisId, y_data[i].StateCovariance);
    thisId.fIdentifier = "StateParameters";
    i_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i,
                                                      7, "StateParameters")),
                       &thisId);
    thisId.fIdentifier = "ObjectClassID";
    y_data[i].ObjectClassID =
        f_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtConstCTX)sp, u, i, 8, "ObjectClassID")),
                           &thisId);
    thisId.fIdentifier = "ObjectClassProbabilities";
    j_emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i, 9,
                                       "ObjectClassProbabilities")),
        &thisId, y_data[i].ObjectClassProbabilities.data,
        y_data[i].ObjectClassProbabilities.size);
    thisId.fIdentifier = "TrackLogic";
    k_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i,
                                                      10, "TrackLogic")),
                       &thisId, y_data[i].TrackLogic);
    thisId.fIdentifier = "TrackLogicState";
    j_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i,
                                                      11, "TrackLogicState")),
                       &thisId, y_data[i].TrackLogicState.data,
                       y_data[i].TrackLogicState.size);
    thisId.fIdentifier = "IsConfirmed";
    y_data[i].IsConfirmed =
        l_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtConstCTX)sp, u, i, 12, "IsConfirmed")),
                           &thisId);
    thisId.fIdentifier = "IsCoasted";
    y_data[i].IsCoasted =
        l_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u,
                                                          i, 13, "IsCoasted")),
                           &thisId);
    thisId.fIdentifier = "IsSelfReported";
    y_data[i].IsSelfReported =
        l_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtConstCTX)sp, u, i, 14, "IsSelfReported")),
                           &thisId);
    thisId.fIdentifier = "ObjectAttributes";
    y_data[i].ObjectAttributes = m_emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, i, 15,
                                       "ObjectAttributes")),
        &thisId);
  }
  emlrtDestroyArray(&u);
}

static uint32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint32_T y;
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct2_T u_data[],
                                        const int32_T u_size)
{
  static const int32_T b_iv[2] = {6, 6};
  static const int32_T b_iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 4};
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
  real_T *pData;
  int32_T b_j0;
  int32_T c_i;
  int32_T d_i;
  int32_T i;
  y = NULL;
  emlrtAssign(&y,
              emlrtCreateStructArray(1, &u_size, 16, (const char_T **)&sv[0]));
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
  for (b_j0 = 0; b_j0 < u_size; b_j0++) {
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
    m = emlrtCreateDoubleScalar(u_data[b_j0].ObjectClassID);
    emlrtAssign(&j_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassID", j_y, 8);
    k_y = NULL;
    m = emlrtCreateDoubleScalar(u_data[b_j0].ObjectClassProbabilities);
    emlrtAssign(&k_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassProbabilities", k_y, 9);
    l_y = NULL;
    m = emlrtCreateCharArray(2, &b_iv1[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m,
                             &u_data[b_j0].TrackLogic[0]);
    emlrtAssign(&l_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogic", l_y, 10);
    m_y = NULL;
    m = emlrtCreateLogicalArray(2, &iv2[0]);
    emlrtInitLogicalArray(4, m, &u_data[b_j0].TrackLogicState[0]);
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

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[6])
{
  t_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[36])
{
  u_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims = 0;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 0, NULL, 0U,
                         (const void *)&dims);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2])
{
  v_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[10])
{
  w_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static boolean_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = x_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static struct1_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[2] = {"Callsign", "Category"};
  emlrtMsgIdentifier thisId;
  struct1_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 2,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "Callsign";
  n_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "Callsign")),
      &thisId, y.Callsign);
  thisId.fIdentifier = "Category";
  y.Category = o_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "Category")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[8])
{
  y_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static adsbCategory o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                       const emlrtMsgIdentifier *parentId)
{
  static const int32_T enumValues[16] = {0, 1, 2,  3,  4,  5,  6,  7,
                                         8, 9, 10, 11, 12, 13, 14, 15};
  static const int32_T dims = 0;
  static const char_T *enumNames[16] = {"No_Category_Information",
                                        "Light",
                                        "Small",
                                        "Large",
                                        "High_Vortex_Large",
                                        "Heavy",
                                        "High_Performance",
                                        "Rotorcraft",
                                        "Glider_Sailplane",
                                        "Lighter_than_air",
                                        "Parachutist_Skydiver",
                                        "Ultralight",
                                        "Unmanned_Aerial_Vehicle",
                                        "Space_Vehicle",
                                        "Surface_Vehicle",
                                        "Obstacle"};
  adsbCategory y;
  emlrtCheckEnumR2012b((emlrtConstCTX)sp, "adsbCategory", 16,
                       (const char_T **)&enumNames[0], &enumValues[0]);
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, parentId, u, "adsbCategory", false,
                          0U, (const void *)&dims);
  y = (adsbCategory)emlrtGetEnumElementR2009a(u, 0);
  emlrtDestroyArray(&u);
  return y;
}

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static uint32_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint32", false, 0U,
                          (const void *)&dims);
  ret = *(uint32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims = 6;
  real_T(*r)[6];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[6])emlrtMxGetData(src);
  for (i = 0; i < 6; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[36])
{
  static const int32_T dims[2] = {6, 6};
  real_T(*r)[36];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[36])emlrtMxGetData(src);
  for (i = 0; i < 36; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2] = {1, 10};
  boolean_T bv[2] = {false, true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &ret_size[0]);
  emlrtImportArrayR2015b((emlrtConstCTX)sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[10])
{
  static const int32_T dims[2] = {1, 10};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 10);
  emlrtDestroyArray(&src);
}

static boolean_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[8])
{
  static const int32_T dims[2] = {1, 8};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 8);
  emlrtDestroyArray(&src);
}

void fusionAlgorithm_api(const mxArray *const prhs[2], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  emxArray_struct0_T *tracks;
  struct2_T fusedTracks_data[100];
  real_T b_time;
  int32_T fusedTracks_size;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  /* Marshall function inputs */
  emxInit_struct0_T(&st, &tracks, &ug_emlrtRTEI);
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "tracks", tracks);
  b_time = p_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "time");
  /* Invoke the target function */
  fusionAlgorithm(&st, tracks, b_time, fusedTracks_data, &fusedTracks_size);
  emxFree_struct0_T(&st, &tracks);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(&st, fusedTracks_data, fusedTracks_size);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_fusionAlgorithm_api.c) */
