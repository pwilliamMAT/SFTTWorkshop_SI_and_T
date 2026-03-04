/*
 * fusionAlgorithm_data.h
 *
 * Code generation for function 'fusionAlgorithm_data'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include "omp.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;
extern emlrtRSInfo e_emlrtRSI;
extern emlrtRSInfo f_emlrtRSI;
extern emlrtRSInfo g_emlrtRSI;
extern emlrtRSInfo p_emlrtRSI;
extern emlrtRSInfo gb_emlrtRSI;
extern emlrtRSInfo ib_emlrtRSI;
extern emlrtRSInfo tb_emlrtRSI;
extern emlrtRSInfo vb_emlrtRSI;
extern emlrtRSInfo qc_emlrtRSI;
extern emlrtRSInfo rc_emlrtRSI;
extern emlrtRSInfo sc_emlrtRSI;
extern emlrtRSInfo tc_emlrtRSI;
extern emlrtRSInfo uc_emlrtRSI;
extern emlrtRSInfo vc_emlrtRSI;
extern emlrtRSInfo xc_emlrtRSI;
extern emlrtRSInfo yc_emlrtRSI;
extern emlrtRSInfo dd_emlrtRSI;
extern emlrtRSInfo ed_emlrtRSI;
extern emlrtRSInfo fd_emlrtRSI;
extern emlrtRSInfo hd_emlrtRSI;
extern emlrtRSInfo jd_emlrtRSI;
extern emlrtRSInfo kd_emlrtRSI;
extern emlrtRSInfo ld_emlrtRSI;
extern emlrtRSInfo md_emlrtRSI;
extern emlrtRSInfo nd_emlrtRSI;
extern emlrtRSInfo od_emlrtRSI;
extern emlrtRSInfo pd_emlrtRSI;
extern emlrtRSInfo td_emlrtRSI;
extern emlrtRSInfo ud_emlrtRSI;
extern emlrtRSInfo ae_emlrtRSI;
extern emlrtRSInfo de_emlrtRSI;
extern emlrtRSInfo ee_emlrtRSI;
extern emlrtRSInfo ke_emlrtRSI;
extern emlrtRSInfo me_emlrtRSI;
extern emlrtRSInfo ne_emlrtRSI;
extern emlrtRSInfo oe_emlrtRSI;
extern emlrtRSInfo pe_emlrtRSI;
extern emlrtRSInfo qe_emlrtRSI;
extern emlrtRSInfo re_emlrtRSI;
extern emlrtRSInfo df_emlrtRSI;
extern emlrtRSInfo hf_emlrtRSI;
extern emlrtRSInfo if_emlrtRSI;
extern emlrtRSInfo tf_emlrtRSI;
extern emlrtRSInfo uf_emlrtRSI;
extern emlrtRSInfo cg_emlrtRSI;
extern emlrtRSInfo jg_emlrtRSI;
extern emlrtRSInfo kg_emlrtRSI;
extern emlrtRSInfo lg_emlrtRSI;
extern emlrtRSInfo mg_emlrtRSI;
extern emlrtRSInfo rg_emlrtRSI;
extern emlrtRSInfo sg_emlrtRSI;
extern emlrtRSInfo vg_emlrtRSI;
extern emlrtRSInfo wg_emlrtRSI;
extern emlrtRSInfo xg_emlrtRSI;
extern emlrtRSInfo yg_emlrtRSI;
extern emlrtRSInfo ah_emlrtRSI;
extern emlrtRSInfo bh_emlrtRSI;
extern emlrtRSInfo ch_emlrtRSI;
extern emlrtRSInfo qh_emlrtRSI;
extern emlrtRSInfo gi_emlrtRSI;
extern emlrtRSInfo hi_emlrtRSI;
extern emlrtRSInfo ii_emlrtRSI;
extern emlrtRSInfo ji_emlrtRSI;
extern emlrtRSInfo ki_emlrtRSI;
extern emlrtRSInfo li_emlrtRSI;
extern emlrtRSInfo mi_emlrtRSI;
extern emlrtRSInfo ni_emlrtRSI;
extern emlrtRSInfo oi_emlrtRSI;
extern emlrtRSInfo pi_emlrtRSI;
extern emlrtRSInfo qi_emlrtRSI;
extern emlrtRSInfo ri_emlrtRSI;
extern emlrtRSInfo si_emlrtRSI;
extern emlrtRSInfo kk_emlrtRSI;
extern emlrtRSInfo nk_emlrtRSI;
extern emlrtRSInfo ok_emlrtRSI;
extern emlrtRSInfo pk_emlrtRSI;
extern emlrtRSInfo qk_emlrtRSI;
extern emlrtRSInfo rk_emlrtRSI;
extern emlrtRSInfo tk_emlrtRSI;
extern emlrtRSInfo uk_emlrtRSI;
extern emlrtRSInfo vk_emlrtRSI;
extern emlrtRSInfo xk_emlrtRSI;
extern emlrtRSInfo al_emlrtRSI;
extern emlrtRSInfo dl_emlrtRSI;
extern emlrtRSInfo kl_emlrtRSI;
extern emlrtRSInfo ll_emlrtRSI;
extern emlrtRSInfo am_emlrtRSI;
extern emlrtRSInfo bm_emlrtRSI;
extern emlrtRSInfo cm_emlrtRSI;
extern emlrtRSInfo cr_emlrtRSI;
extern omp_lock_t emlrtLockGlobal;
extern omp_nest_lock_t fusionAlgorithm_nestLockGlobal;
extern emlrtRTEInfo c_emlrtRTEI;
extern emlrtRTEInfo g_emlrtRTEI;
extern emlrtRTEInfo i_emlrtRTEI;
extern emlrtRTEInfo l_emlrtRTEI;
extern emlrtRTEInfo m_emlrtRTEI;
extern emlrtRTEInfo o_emlrtRTEI;
extern emlrtRTEInfo p_emlrtRTEI;
extern emlrtRTEInfo q_emlrtRTEI;
extern emlrtRTEInfo y_emlrtRTEI;
extern emlrtRTEInfo ab_emlrtRTEI;
extern emlrtRTEInfo db_emlrtRTEI;
extern emlrtBCInfo jc_emlrtBCI;
extern emlrtBCInfo kc_emlrtBCI;
extern emlrtBCInfo qc_emlrtBCI;
extern emlrtBCInfo rc_emlrtBCI;
extern emlrtBCInfo sc_emlrtBCI;
extern emlrtBCInfo tc_emlrtBCI;
extern emlrtRTEInfo ob_emlrtRTEI;
extern emlrtRTEInfo sb_emlrtRTEI;
extern emlrtRTEInfo sd_emlrtRTEI;
extern emlrtRTEInfo bh_emlrtRTEI;
extern emlrtRTEInfo fh_emlrtRTEI;
extern emlrtRTEInfo qh_emlrtRTEI;
extern emlrtRTEInfo rh_emlrtRTEI;
extern const char_T cv[128];
extern const int8_T iv[36];
extern const char_T cv1[13];
extern const char_T cv2[14];
extern const int32_T iv1[4];

/* End of code generation (fusionAlgorithm_data.h) */
