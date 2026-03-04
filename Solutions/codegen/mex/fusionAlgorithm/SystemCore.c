/*
 * SystemCore.c
 *
 * Code generation for function 'SystemCore'
 *
 */

/* Include files */
#include "SystemCore.h"
#include "FuserManager.h"
#include "fusionAlgorithm_data.h"
#include "fusionAlgorithm_types.h"
#include "rt_nonfinite.h"
#include "trackFuser.h"

/* Variable Definitions */
static emlrtRSInfo ub_emlrtRSI = {
    1,                 /* lineNo */
    "SystemCore/step", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pathName */
};

static emlrtRSInfo wb_emlrtRSI = {
    950,                    /* lineNo */
    "trackFuser/setupImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m" /* pathName
                                                                          */
};

static emlrtRTEInfo f_emlrtRTEI = {
    1,                 /* lineNo */
    1,                 /* colNo */
    "SystemCore/step", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+"
    "coder\\SystemCore.p" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    947,                    /* lineNo */
    17,                     /* colNo */
    "trackFuser/setupImpl", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\fusion\\core\\fusion\\trackFuser.m" /* pName
                                                                          */
};

/* Function Definitions */
void SystemCore_releaseWrapper(trackFuser *obj)
{
  trackHistoryLogic *b_obj;
  int32_T i;
  if (obj->isSetupComplete) {
    obj->pNumLiveTracks = 0.0;
    for (i = 0; i < 100; i++) {
      obj->pTrackIDs[i] = 0U;
    }
    for (i = 0; i < 100; i++) {
      obj->pConfirmedTracks[i] = false;
    }
    if (obj->cAssigner.isInitialized == 1) {
      obj->cAssigner.isInitialized = 2;
    }
    b_obj = obj->pTrackLogics[0];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[1];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[2];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[3];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[4];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[5];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[6];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[7];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[8];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[9];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[10];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[11];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[12];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[13];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[14];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[15];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[16];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[17];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[18];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[19];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[20];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[21];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[22];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[23];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[24];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[25];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[26];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[27];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[28];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[29];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[30];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[31];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[32];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[33];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[34];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[35];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[36];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[37];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[38];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[39];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[40];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[41];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[42];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[43];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[44];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[45];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[46];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[47];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[48];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[49];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[50];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[51];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[52];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[53];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[54];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[55];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[56];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[57];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[58];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[59];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[60];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[61];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[62];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[63];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[64];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[65];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[66];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[67];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[68];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[69];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[70];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[71];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[72];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[73];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[74];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[75];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[76];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[77];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[78];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[79];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[80];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[81];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[82];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[83];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[84];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[85];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[86];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[87];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[88];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[89];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[90];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[91];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[92];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[93];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[94];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[95];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[96];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[97];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[98];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    b_obj = obj->pTrackLogics[99];
    for (i = 0; i < 50; i++) {
      b_obj->pRecentHistory[i] = false;
    }
    b_obj->pIsFirstUpdate = true;
    obj->pLastTrackID = 0U;
  }
}

void SystemCore_reset(const emlrtStack *sp,
                      c_matlabshared_tracking_interna *obj)
{
  if (obj->isInitialized == 2) {
    emlrtErrorWithMessageIdR2018a(
        sp, &g_emlrtRTEI, "MATLAB:system:methodCalledWhenReleasedCodegen",
        "MATLAB:system:methodCalledWhenReleasedCodegen", 3, 4, 5, "reset");
  }
  if (obj->isInitialized == 1) {
    obj->pCostOfNonAssignment = obj->AssignmentThreshold[0] / 2.0;
  }
}

int32_T SystemCore_step(const emlrtStack *sp, trackFuser *obj,
                        const emxArray_struct0_T *varargin_1, real_T varargin_2,
                        struct2_T varargout_1_data[])
{
  static const char_T params[5] = {'t', 'r', 'a', 'c', 'e'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int32_T i;
  int32_T varargout_1_size;
  uint8_T inSize[8];
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  if (obj->isInitialized == 2) {
    emlrtErrorWithMessageIdR2018a(
        sp, &f_emlrtRTEI, "MATLAB:system:methodCalledWhenReleasedCodegen",
        "MATLAB:system:methodCalledWhenReleasedCodegen", 3, 4, 4, "step");
  }
  if (obj->isInitialized != 1) {
    cell_wrap_3 varSizes[2];
    real_T Q[9];
    st.site = &ub_emlrtRSI;
    b_st.site = &p_emlrtRSI;
    obj->isSetupComplete = false;
    if (obj->isInitialized != 0) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &g_emlrtRTEI,
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen",
          "MATLAB:system:methodCalledWhenLockedReleasedCodegen", 3, 4, 5,
          "setup");
    }
    obj->isInitialized = 1;
    c_st.site = &p_emlrtRSI;
    varSizes[0].f1[0] = (uint32_T)varargin_1->size[0];
    varSizes[0].f1[1] = 1U;
    for (i = 0; i < 6; i++) {
      varSizes[0].f1[i + 2] = 1U;
    }
    for (i = 0; i < 8; i++) {
      varSizes[1].f1[i] = 1U;
    }
    obj->inputVarSize[0] = varSizes[0];
    obj->inputVarSize[1] = varSizes[1];
    c_st.site = &vb_emlrtRSI;
    if (varargin_1->size[0] == 0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &h_emlrtRTEI,
                                    "fusion:trackFuser:EmptyTrackInput",
                                    "fusion:trackFuser:EmptyTrackInput", 0);
    }
    d_st.site = &wb_emlrtRSI;
    FuserManager_setupImpl(&d_st, obj, varargin_1);
    obj->cAssigner.isInitialized = 0;
    obj->cAssigner.AssignmentThreshold[0] = 100.0;
    obj->cAssigner.AssignmentThreshold[1] = 250.0;
    for (i = 0; i < 9; i++) {
      Q[i] = obj->ProcessNoise[i];
    }
    for (i = 0; i < 9; i++) {
      obj->cFuser.ProcessNoise[i] = Q[i];
    }
    for (i = 0; i < 5; i++) {
      obj->cFuser.StateFusionParameters[i] = params[i];
    }
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[0].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[0] = &obj->coder_buffer_pobj0[0];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[1].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[1] = &obj->coder_buffer_pobj0[1];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[2].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[2] = &obj->coder_buffer_pobj0[2];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[3].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[3] = &obj->coder_buffer_pobj0[3];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[4].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[4] = &obj->coder_buffer_pobj0[4];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[5].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[5] = &obj->coder_buffer_pobj0[5];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[6].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[6] = &obj->coder_buffer_pobj0[6];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[7].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[7] = &obj->coder_buffer_pobj0[7];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[8].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[8] = &obj->coder_buffer_pobj0[8];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[9].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[9] = &obj->coder_buffer_pobj0[9];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[10].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[10] = &obj->coder_buffer_pobj0[10];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[11].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[11] = &obj->coder_buffer_pobj0[11];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[12].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[12] = &obj->coder_buffer_pobj0[12];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[13].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[13] = &obj->coder_buffer_pobj0[13];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[14].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[14] = &obj->coder_buffer_pobj0[14];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[15].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[15] = &obj->coder_buffer_pobj0[15];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[16].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[16] = &obj->coder_buffer_pobj0[16];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[17].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[17] = &obj->coder_buffer_pobj0[17];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[18].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[18] = &obj->coder_buffer_pobj0[18];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[19].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[19] = &obj->coder_buffer_pobj0[19];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[20].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[20] = &obj->coder_buffer_pobj0[20];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[21].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[21] = &obj->coder_buffer_pobj0[21];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[22].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[22] = &obj->coder_buffer_pobj0[22];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[23].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[23] = &obj->coder_buffer_pobj0[23];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[24].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[24] = &obj->coder_buffer_pobj0[24];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[25].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[25] = &obj->coder_buffer_pobj0[25];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[26].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[26] = &obj->coder_buffer_pobj0[26];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[27].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[27] = &obj->coder_buffer_pobj0[27];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[28].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[28] = &obj->coder_buffer_pobj0[28];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[29].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[29] = &obj->coder_buffer_pobj0[29];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[30].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[30] = &obj->coder_buffer_pobj0[30];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[31].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[31] = &obj->coder_buffer_pobj0[31];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[32].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[32] = &obj->coder_buffer_pobj0[32];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[33].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[33] = &obj->coder_buffer_pobj0[33];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[34].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[34] = &obj->coder_buffer_pobj0[34];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[35].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[35] = &obj->coder_buffer_pobj0[35];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[36].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[36] = &obj->coder_buffer_pobj0[36];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[37].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[37] = &obj->coder_buffer_pobj0[37];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[38].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[38] = &obj->coder_buffer_pobj0[38];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[39].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[39] = &obj->coder_buffer_pobj0[39];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[40].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[40] = &obj->coder_buffer_pobj0[40];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[41].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[41] = &obj->coder_buffer_pobj0[41];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[42].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[42] = &obj->coder_buffer_pobj0[42];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[43].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[43] = &obj->coder_buffer_pobj0[43];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[44].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[44] = &obj->coder_buffer_pobj0[44];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[45].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[45] = &obj->coder_buffer_pobj0[45];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[46].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[46] = &obj->coder_buffer_pobj0[46];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[47].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[47] = &obj->coder_buffer_pobj0[47];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[48].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[48] = &obj->coder_buffer_pobj0[48];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[49].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[49] = &obj->coder_buffer_pobj0[49];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[50].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[50] = &obj->coder_buffer_pobj0[50];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[51].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[51] = &obj->coder_buffer_pobj0[51];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[52].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[52] = &obj->coder_buffer_pobj0[52];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[53].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[53] = &obj->coder_buffer_pobj0[53];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[54].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[54] = &obj->coder_buffer_pobj0[54];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[55].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[55] = &obj->coder_buffer_pobj0[55];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[56].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[56] = &obj->coder_buffer_pobj0[56];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[57].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[57] = &obj->coder_buffer_pobj0[57];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[58].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[58] = &obj->coder_buffer_pobj0[58];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[59].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[59] = &obj->coder_buffer_pobj0[59];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[60].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[60] = &obj->coder_buffer_pobj0[60];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[61].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[61] = &obj->coder_buffer_pobj0[61];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[62].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[62] = &obj->coder_buffer_pobj0[62];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[63].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[63] = &obj->coder_buffer_pobj0[63];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[64].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[64] = &obj->coder_buffer_pobj0[64];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[65].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[65] = &obj->coder_buffer_pobj0[65];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[66].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[66] = &obj->coder_buffer_pobj0[66];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[67].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[67] = &obj->coder_buffer_pobj0[67];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[68].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[68] = &obj->coder_buffer_pobj0[68];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[69].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[69] = &obj->coder_buffer_pobj0[69];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[70].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[70] = &obj->coder_buffer_pobj0[70];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[71].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[71] = &obj->coder_buffer_pobj0[71];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[72].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[72] = &obj->coder_buffer_pobj0[72];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[73].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[73] = &obj->coder_buffer_pobj0[73];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[74].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[74] = &obj->coder_buffer_pobj0[74];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[75].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[75] = &obj->coder_buffer_pobj0[75];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[76].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[76] = &obj->coder_buffer_pobj0[76];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[77].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[77] = &obj->coder_buffer_pobj0[77];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[78].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[78] = &obj->coder_buffer_pobj0[78];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[79].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[79] = &obj->coder_buffer_pobj0[79];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[80].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[80] = &obj->coder_buffer_pobj0[80];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[81].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[81] = &obj->coder_buffer_pobj0[81];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[82].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[82] = &obj->coder_buffer_pobj0[82];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[83].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[83] = &obj->coder_buffer_pobj0[83];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[84].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[84] = &obj->coder_buffer_pobj0[84];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[85].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[85] = &obj->coder_buffer_pobj0[85];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[86].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[86] = &obj->coder_buffer_pobj0[86];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[87].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[87] = &obj->coder_buffer_pobj0[87];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[88].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[88] = &obj->coder_buffer_pobj0[88];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[89].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[89] = &obj->coder_buffer_pobj0[89];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[90].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[90] = &obj->coder_buffer_pobj0[90];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[91].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[91] = &obj->coder_buffer_pobj0[91];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[92].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[92] = &obj->coder_buffer_pobj0[92];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[93].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[93] = &obj->coder_buffer_pobj0[93];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[94].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[94] = &obj->coder_buffer_pobj0[94];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[95].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[95] = &obj->coder_buffer_pobj0[95];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[96].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[96] = &obj->coder_buffer_pobj0[96];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[97].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[97] = &obj->coder_buffer_pobj0[97];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[98].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[98] = &obj->coder_buffer_pobj0[98];
    for (i = 0; i < 50; i++) {
      obj->coder_buffer_pobj0[99].pRecentHistory[i] = false;
    }
    obj->pTrackLogics[99] = &obj->coder_buffer_pobj0[99];
    obj->isSetupComplete = true;
    obj->TunablePropsChanged = false;
    b_st.site = &p_emlrtRSI;
    trackFuser_resetImpl(&b_st, obj);
  }
  st.site = &ub_emlrtRSI;
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
  }
  st.site = &ub_emlrtRSI;
  inSize[0] = (uint8_T)varargin_1->size[0];
  inSize[1] = 1U;
  for (i = 0; i < 6; i++) {
    inSize[i + 2] = 1U;
  }
  varargout_1_size = 0;
  exitg1 = false;
  while ((!exitg1) && (varargout_1_size < 8)) {
    if (obj->inputVarSize[0].f1[varargout_1_size] != inSize[varargout_1_size]) {
      for (i = 0; i < 8; i++) {
        obj->inputVarSize[0].f1[i] = inSize[i];
      }
      exitg1 = true;
    } else {
      varargout_1_size++;
    }
  }
  st.site = &ub_emlrtRSI;
  return trackFuser_stepImpl(&st, obj, varargin_1, varargin_2,
                             varargout_1_data);
}

/* End of code generation (SystemCore.c) */
