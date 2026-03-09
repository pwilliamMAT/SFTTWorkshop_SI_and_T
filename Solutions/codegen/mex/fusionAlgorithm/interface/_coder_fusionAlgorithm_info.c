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
      "789cdd55416fd3301476d0404888d113e71d7665420826c18991d251605b45b382544fc5"
      "71bcc6ab1d47b6b37513072e9c11ff8023127f80bfc01f800b077e01"
      "670e1c5892ba6d2c45191bcaa4bd435ebe7ccfef7b7e499e81d3de7000008b20b72fd772"
      "7f7d821b137f0914cde61d2bce298683cb60a1b0cef0ef271e8b4893",
      "b1ce41843899ae0c04a7118ab47718132089126c9f0419b34b19f12827dd79b09922de9a"
      "a3a620a5d27b372478d44d3890a19a55c8e6c1b41faf4bf6bb50d10f"
      "dbec7ed871462f3ea59ec97fa342cff0bb89a2225a634321a90ef999f76bf4ae94eae78c"
      "d232c17ab6df9f67d47b53aa57e4fb8f77dc07705b11a9607c4019a3",
      "88c3ad883425dd274bb79636900e5f0a3952b02970c249a415ecb63c2fbb644428e20e92"
      "fa4ee1c9a0db1ea028181cc70996e8e3962a68b57685a7fa55fd35ef"
      "a56abfb69fc55fcdfcafcf3f9c3af5c61ffe7cad53cfd879e98d4bf29df47bbd59a2d7b0"
      "f8d5a3e68b839e7f74fb59a25675fce4fe70cd5d6fcdeae854e854d5",
      "014a705df93f96ac3f691f1f96e46f587cbfedee2c73a419f2a5107a196a21982fc65085"
      "4892006a89f08846c3c95fcba80f85bf47b0f6526285ffafb9bc6861"
      "bb5ec367f53c17438ab3f3eedccea14fa7d433f9dd0a3dc3f7db9bfffa7e8a2dcac76b6d"
      "f360effbb75ae7ebdbcebbdf75ea19bbe8f33510db5b777dce3af85e",
      "2f78f494e1c357bd8b305fff02be6dc3f9",
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
                emlrtMxCreateDoubleScalar(740045.45253472216));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("25.2.0.2998904 (R2025b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("ybbBS7UvMIZk3BWuAwE0oG"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "AuxData",
                emlrtMxCreateRowVectorUINT8((const uint8_T *)&v, 216U));
  return xResult;
}

/* End of code generation (_coder_fusionAlgorithm_info.c) */
