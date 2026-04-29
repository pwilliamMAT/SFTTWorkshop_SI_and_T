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
      "789cdd55316f13311476509190102548880da943574ea2858589903628a86903b9864ab9"
      "2a757c4ecf8ded77b29d922c888519464646067e0f2c0cfc8c8e3497"
      "38c9199dae342895fa86f33d7fcfeffbfceefc8c0ad55a0121b48ac676767b3cde99f8c5"
      "c97803a5cdc50b4e5c211d8e6ea295d43a8b7f9a8c04a4a103337624",
      "1674ba3204c12496c61fc61429aa819fd23041ba8c539f09da987776479ea8cc41536704"
      "8ddecb1125bd465f2015e999423eef4ceb7194b1df959c7ab8e6d6c3"
      "8db37c834bf2d9fcf772f82c6e14263d268f4bfc1814339170747c5950c7668e0e8b77fb"
      "9a81f4123954799a4a0d4ac79468af4415e818135a0309da60c3c81b",
      "1c6295d2f97b419def73745abcb57d587e16ec6baa7410bf639c332c823d49b7143ba56b"
      "8fd66ad8446f41f574b005a42fa8343a68547c3f79244004711d2bb3"
      "919a6937aa6d2cc3f6791cf0be392f850efefa349ec8ff1fef5e70bfee388bbf359efffe"
      "2b8196c5f7f0f3ebfbcbe4b376557c8b9eef07197c45071f0c23b1b9",
      "bde1970f6a3b5dde7bdc2c6152aacc74d47378f274a00c7f59f9bf66acbf681d9f67e42f"
      "3a78ab5a3e5c17d870dc5100663d3000bc0383404758d1707a588371"
      "23e3ac1340e78412e38f004f58bdf1827a571ddfd56bf144cf0e1c3392dc9757768f7dbb"
      "249fcd5fcee1b378abbafbafdf275d224fa4741f65e8fa5ffde0e4e7",
      "8fa5f6d70ff58f67cbe4b376ddfb6b08fb7b4f3a82d7c9d366f8e21527c383e6cb6bd05f"
      "ff003bd6d54f",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3088U, &nameCaptureInfo);
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
      emlrtMxCreateString("C:\\Users\\pwilliam\\OneDrive - "
                          "MathWorks\\Documents\\SFTT\\SFTTWorkshopPart2\\SFTTW"
                          "orkshop_SI_and_T\\Solutions\\trackingAlgorit"
                          "hm.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740045.54346064816));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("25.2.0.2998904 (R2025b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("SJUfm8gfHXwWzRy23juMU"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "AuxData",
                emlrtMxCreateRowVectorUINT8((const uint8_T *)&v, 216U));
  return xResult;
}

/* End of code generation (_coder_trackingAlgorithm_info.c) */
