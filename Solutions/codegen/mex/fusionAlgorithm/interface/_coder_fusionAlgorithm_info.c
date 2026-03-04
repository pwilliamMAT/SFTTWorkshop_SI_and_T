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
      "789cdd55316fd430147650414815e526e60eb7d201958509b872e580b681a4e5d0b93a1c"
      "c7bdb867c727db39ae52872eccec4c8c482c8cfc12581898f80d8c34"
      "c9f9ee62294a69512af50d79f9f23dbfeff92579064e67cb0100ac80dcbe2de7fed61437"
      "a6fe1a289acd3b569c530c07d7c152619de13f4c3d16b126139d8318",
      "71325b190a4e63146bff684480244ab0310933e68032e2534ebc45b09d22de5ea06620a5"
      "d2fb5644f0d04b3890919a57c816c1ac1f6f4bf6bb54d10fdbec7ed8"
      "71466f744e3d93ff76859ee10f124545fc880d84a43ae217deafd1bb51aa9f334acb04eb"
      "f97e7f5d50efb854afc8f79eecb71ec05d45a482a37794318a38dc89",
      "c986a463b27a77750be9e8b590430537044e3889b5825edbf7b34b464462e422a9ef159e"
      "f4bd4e1fc561ff344eb0449fb65441abb56b3cd5afeaaf792f55fbb5"
      "fd3cfe66e67f7ff9e9d4a9d75cfefab14e3d6397a53729c977d6eff54e895ec3e2bb5d3f"
      "7af96aec26ebdcf53acf71b4193c7dd39ed7e156e854d5014a705df9",
      "3f95ac3f6b1f1f96e46f587cafd3da6f72a4190aa410ba09b5102c1013a822244908b544"
      "7848e3c1f4af65348022382458fb29b1c6ffd75c5eb1b05dafe1b37a"
      "5e8801c5d9797769e7d0e773ea99fcad0a3dc3f73adbfffa7e8a2dcac76b6df3e0f0c7f7"
      "5ae7eb89fbfe4f9d7ac6aefa7c0dc5eece7ac0998befef858f9f317c",
      "d4dddbbc02f3f52fffdfc2b4",
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
                emlrtMxCreateDoubleScalar(740045.41185185185));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("25.2.0.2998904 (R2025b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("4yDRwgAaRvFxOtzEWxEU4D"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "AuxData",
                emlrtMxCreateRowVectorUINT8((const uint8_T *)&v, 216U));
  return xResult;
}

/* End of code generation (_coder_fusionAlgorithm_info.c) */
