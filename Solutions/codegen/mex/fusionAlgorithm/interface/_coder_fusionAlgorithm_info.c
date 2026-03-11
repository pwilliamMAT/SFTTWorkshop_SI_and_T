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
      "789cdd55cf6fd3301476d0404813a327ce3becba49a021214e40b74e9dd856a9e936a99e"
      "8ae3b88d57ffa86c6774d30e5c38f32f70e4c695bf048404127f0407"
      "8e2c49dd3696a28c0d65d2de212f5fbee7f73dbf24cfc06bee7800802590d997c5cc3f98"
      "e0dac4df01797379cf89f3f2e1e02e58c8adb3fc8789c75218323619",
      "108893e9ca50722a9030fee9880045b46427244c993e65c4a79cb4e7c16e8278638e9a82"
      "844aeeeb11c1c376cc818af4ac42360fa6fd7853b0df85927eb8e6f6"
      "c38db37aa32bead9fc0f4bf42cdf8f3595e2251b48454dc4afbd5fab77af503f63b45131"
      "36b3fdfebaa6de79a15e9eef6e1ed59fc38e264ac3d15bca18451cee",
      "09b2a1e809595e5dde41263a906aa8e186c43127c268d86ef87e7a4989488e5a489927b9"
      "27bd76b38744d8bb88932c36172dd5d069ed1a4ff4cbfa6bdf4bd97e"
      "5d3f8bbf9ffa9f9f7f7855ea7df77f6f55a967eda6f4c605f92efbbd3e2ad0ab393cc307"
      "6af3acb34ecec8e36df22c3eecf3816accea6895e894d5010a7055f9",
      "3f16acbf6c1f5f14e4af397cb7593f5ae1c830142829cd0a3452b2408ea18e902221340a"
      "e1211583c95fcb680065704cb0f113628dffafb9bce460b75ecba7f5"
      "bc96038ad3f3eec6cea14f57d4b3f9eb257a96ef3677fff5fde45b948dd7cae6c1f1b7af"
      "95ced777adf77faad4b376dbe76b283b7beb01672dfc743f7cb5cdf0",
      "e9e1fed62d98af7f01f0f1c41c",
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
                emlrtMxCreateDoubleScalar(740050.4975));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("25.2.0.2998904 (R2025b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("v6lIvUuhEUhhPulJTgT1ZG"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "AuxData",
                emlrtMxCreateRowVectorUINT8((const uint8_T *)&v, 216U));
  return xResult;
}

/* End of code generation (_coder_fusionAlgorithm_info.c) */
