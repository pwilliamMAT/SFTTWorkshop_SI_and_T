/*
 * _coder_trackingAlgorithm_info.c
 *
 * Code generation for function 'trackingAlgorithm'
 *
 */

/* Include files */
#include "_coder_trackingAlgorithm_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[6] = {
      "789ce555cf6fd3301476d1909010a35c3873d80d2d39148e48641d85b0b69435db98aaa9"
      "739cb7c6d4b183ed8eeeafe0863872e4cfe0c2ffc0913f8535adfbc3"
      "28ca58a520b4777979faecf77dfe223fa38adfaa208436d1347ede9de67bb3ba3acbb7d0"
      "6ad878c55a57595d8e6ea38d957d06ff34cb44700d633d2d384e60be",
      "331209e598ebe02205244109760e51869c5106014da0bb5cb42755d25882e6c5049a7cd7"
      "6320c3ee284132560b856cb998fb719a73de8d023fecb0fdb0d719be"
      "f135f94cff07057c06d7129321e5038f0d84a43a4e2c1d5fd6d4512bd061f0b391a2823b"
      "991c908e02ae84542910e57820854a318196e04269ac29d9c711962b",
      "3abfafa9f3b840a7c17b2f4edc961734bd9d47bb929e83bb0fa950545f9a07caed3682e0"
      "48c8a18a45daeffa7dcca37eb05d735f014b4136469ce8cb532af70f"
      "d71ddbf7d31cbdf7af781e3b2fd6dfc9f2e3679f69997c266e0adfbaf7f7610e5fd5c26b"
      "9e1784fef101fbf80193a351b33386bdb7bb0b1d9d029e221d28a72e",
      "abffd79cfd57f5f1794effaa85f7fcfac9568235c3a114426fb95a08168ab1ab622c219a"
      "df58773aa8180d5d11be07a28309e024466fbaa6de4dabb6f51a3cd3"
      "d314034ab2f7f09fbd53dfaec967fad70bf80cdef3db7ffb7f562d9a4dd8b2e6c18ff6af"
      "7e997c266e0a5f59f33512076f9e8409eb90a787d1ce6b462ede1dbe",
      "6cfcfff3f5379327c0cc",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3056U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 3);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("trackingAlgorithm"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("/MATLAB "
                          "Drive/Repositories/SFTTWorkshop_SI_and_T-3/"
                          "HelperFunctions/trackingAlgorithm.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740031.93679398147));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("25.2.0.3150157 (R2025b) Update 4"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("1HYHUrhFUxGgJe1OcjUY8"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "AuxData",
                emlrtMxCreateRowVectorUINT8((const uint8_T *)&v, 216U));
  return xResult;
}

/* End of code generation (_coder_trackingAlgorithm_info.c) */
