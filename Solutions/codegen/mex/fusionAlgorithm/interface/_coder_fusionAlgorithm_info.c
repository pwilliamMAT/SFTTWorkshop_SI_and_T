/*
 * _coder_fusionAlgorithm_info.c
 *
 * Code generation for function 'fusionAlgorithm'
 *
 */

/* Include files */
#include "_coder_fusionAlgorithm_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[6] = {
      "789cdd55416fd3301476d0404888d113e71d7665878aa101176847a732b6554a5a90eaa9"
      "388ed778b5e3623b5b873870e1cc5fd811f10bf82570e1c0811fc191"
      "25a9dbc65294b1a14cf00e79f9f23dbfeff92579064e7bc701002c83ccbedccafced29ae"
      "4dfd3590379b77ac38271f0eae83a5dc3ac37f9c7a2c224d263a0311",
      "e264b632109c4628d2dec998004994604724489903ca8847397117c16e82786b819a8184"
      "4aee9b21c12337e640866a5e215b04b37ebc2ed8ef52493f6cb3fb61"
      "c719bdf105f54cfe3b257a863f881515d153361492ea905f7abf46ef46a17ec6282d63ac"
      "e7fbfd7149bd77857a79beff6cbff9087615910a8e8f29631471b817",
      "914d498fc8cabd951da4c397428e14dc1438e624d20aba2dcf4b2f29118a7107495dcf3d"
      "19b8ed018a82c1599c60b13e6ba982566bd778a25fd65ff35ecaf66b"
      "fb79fccdd43f3efdee54a9b7f1f967a34a3d6357a53729c877deeff56e815ecde2ebbd87"
      "7e5d34de7a7c7dfb4d77e8bb1ede78b035afa353a253560728c055e5",
      "3f2d587fde3e3e29c85fb3f87ebbb9bfca9166c89742e855a88560be984015224902a825"
      "c2231a0da77f2da33e14fe21c1da4b8835feb7e6f2b285ed7a0d9fd6"
      "f3420c294ecfbb2b3b873e5d50cfe46f96e819bedfdefdd3f7936f51365e2b9b0787dfbe"
      "563a5fdf773efcaa52cfd8ff3e5f03d1ddbbef73d6c1ebbda0f19ce1",
      "9357bdadd6bf3f5f7f03df7fc21f",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3024U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *epFieldName[7] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Visible"};
  const char_T *propFieldName[7] = {
      "Version",      "ResolvedFunctions", "Checksum", "EntryPoints",
      "CoverageInfo", "IsPolymorphic",     "AuxData"};
  uint8_T v[216] = {
      0U,   1U,   73U,  77U,  0U,   0U,   0U,   0U,   14U,  0U,   0U,   0U,
      200U, 0U,   0U,   0U,   6U,   0U,   0U,   0U,   8U,   0U,   0U,   0U,
      2U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   5U,   0U,   0U,   0U,
      8U,   0U,   0U,   0U,   1U,   0U,   0U,   0U,   1U,   0U,   0U,   0U,
      1U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   5U,   0U,   4U,   0U,
      17U,  0U,   0U,   0U,   1U,   0U,   0U,   0U,   17U,  0U,   0U,   0U,
      67U,  108U, 97U,  115U, 115U, 69U,  110U, 116U, 114U, 121U, 80U,  111U,
      105U, 110U, 116U, 115U, 0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      14U,  0U,   0U,   0U,   112U, 0U,   0U,   0U,   6U,   0U,   0U,   0U,
      8U,   0U,   0U,   0U,   2U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      5U,   0U,   0U,   0U,   8U,   0U,   0U,   0U,   1U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   1U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      5U,   0U,   4U,   0U,   14U,  0U,   0U,   0U,   1U,   0U,   0U,   0U,
      56U,  0U,   0U,   0U,   81U,  117U, 97U,  108U, 105U, 102U, 105U, 101U,
      100U, 78U,  97U,  109U, 101U, 0U,   77U,  101U, 116U, 104U, 111U, 100U,
      115U, 0U,   0U,   0U,   0U,   0U,   0U,   0U,   80U,  114U, 111U, 112U,
      101U, 114U, 116U, 105U, 101U, 115U, 0U,   0U,   0U,   0U,   72U,  97U,
      110U, 100U, 108U, 101U, 0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 2);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("fusionAlgorithm"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("C:\\Users\\pwilliam\\OneDrive - "
                          "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTW"
                          "orkshop_SI_and_T\\Solutions\\fusionAlgorithm"
                          ".m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740030.56168981479));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("25.2.0.2998904 (R2025b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("4baEpJIBwY4ntkABQOOmfC"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "AuxData",
                emlrtMxCreateRowVectorUINT8((const uint8_T *)&v, 216U));
  return xResult;
}

/* End of code generation (_coder_fusionAlgorithm_info.c) */
