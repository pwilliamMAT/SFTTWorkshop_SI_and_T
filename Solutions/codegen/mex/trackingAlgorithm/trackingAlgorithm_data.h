/*
 * trackingAlgorithm_data.h
 *
 * Code generation for function 'trackingAlgorithm_data'
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
extern emlrtRSInfo j_emlrtRSI;
extern emlrtRSInfo k_emlrtRSI;
extern emlrtRSInfo wb_emlrtRSI;
extern emlrtRSInfo yb_emlrtRSI;
extern emlrtRSInfo rc_emlrtRSI;
extern emlrtRSInfo sc_emlrtRSI;
extern emlrtRSInfo tc_emlrtRSI;
extern emlrtRSInfo uc_emlrtRSI;
extern emlrtRSInfo vc_emlrtRSI;
extern emlrtRSInfo wc_emlrtRSI;
extern emlrtRSInfo xc_emlrtRSI;
extern emlrtRSInfo bd_emlrtRSI;
extern emlrtRSInfo cd_emlrtRSI;
extern emlrtRSInfo id_emlrtRSI;
extern emlrtRSInfo jd_emlrtRSI;
extern emlrtRSInfo vd_emlrtRSI;
extern emlrtRSInfo wd_emlrtRSI;
extern emlrtRSInfo xd_emlrtRSI;
extern emlrtRSInfo yd_emlrtRSI;
extern emlrtRSInfo ae_emlrtRSI;
extern emlrtRSInfo be_emlrtRSI;
extern emlrtRSInfo ce_emlrtRSI;
extern emlrtRSInfo de_emlrtRSI;
extern emlrtRSInfo ee_emlrtRSI;
extern emlrtRSInfo fe_emlrtRSI;
extern emlrtRSInfo ge_emlrtRSI;
extern emlrtRSInfo ie_emlrtRSI;
extern emlrtRSInfo je_emlrtRSI;
extern emlrtRSInfo ke_emlrtRSI;
extern emlrtRSInfo le_emlrtRSI;
extern emlrtRSInfo me_emlrtRSI;
extern emlrtRSInfo ne_emlrtRSI;
extern emlrtRSInfo oe_emlrtRSI;
extern emlrtRSInfo pe_emlrtRSI;
extern emlrtRSInfo qe_emlrtRSI;
extern emlrtRSInfo re_emlrtRSI;
extern emlrtRSInfo se_emlrtRSI;
extern emlrtRSInfo te_emlrtRSI;
extern emlrtRSInfo ue_emlrtRSI;
extern emlrtRSInfo ve_emlrtRSI;
extern emlrtRSInfo we_emlrtRSI;
extern emlrtRSInfo xe_emlrtRSI;
extern emlrtRSInfo ye_emlrtRSI;
extern emlrtRSInfo af_emlrtRSI;
extern emlrtRSInfo bf_emlrtRSI;
extern emlrtRSInfo cf_emlrtRSI;
extern emlrtRSInfo df_emlrtRSI;
extern emlrtRSInfo ef_emlrtRSI;
extern emlrtRSInfo ff_emlrtRSI;
extern emlrtRSInfo gf_emlrtRSI;
extern emlrtRSInfo hf_emlrtRSI;
extern emlrtRSInfo if_emlrtRSI;
extern emlrtRSInfo kf_emlrtRSI;
extern emlrtRSInfo sf_emlrtRSI;
extern emlrtRSInfo gg_emlrtRSI;
extern emlrtRSInfo hg_emlrtRSI;
extern emlrtRSInfo dh_emlrtRSI;
extern emlrtRSInfo eh_emlrtRSI;
extern emlrtRSInfo fh_emlrtRSI;
extern emlrtRSInfo gh_emlrtRSI;
extern emlrtRSInfo sj_emlrtRSI;
extern emlrtRSInfo tj_emlrtRSI;
extern emlrtRSInfo tk_emlrtRSI;
extern emlrtRSInfo bn_emlrtRSI;
extern emlrtRSInfo cn_emlrtRSI;
extern emlrtRSInfo dn_emlrtRSI;
extern emlrtRSInfo gn_emlrtRSI;
extern emlrtRSInfo hn_emlrtRSI;
extern emlrtRSInfo in_emlrtRSI;
extern emlrtRSInfo jn_emlrtRSI;
extern emlrtRSInfo ln_emlrtRSI;
extern emlrtRSInfo lp_emlrtRSI;
extern emlrtRSInfo mp_emlrtRSI;
extern emlrtRSInfo sp_emlrtRSI;
extern emlrtRSInfo xp_emlrtRSI;
extern emlrtRSInfo yp_emlrtRSI;
extern emlrtRSInfo aq_emlrtRSI;
extern emlrtRSInfo bq_emlrtRSI;
extern emlrtRSInfo sq_emlrtRSI;
extern emlrtRSInfo xr_emlrtRSI;
extern emlrtRSInfo js_emlrtRSI;
extern emlrtRSInfo ks_emlrtRSI;
extern emlrtRSInfo ns_emlrtRSI;
extern emlrtRSInfo dt_emlrtRSI;
extern emlrtRSInfo et_emlrtRSI;
extern emlrtRSInfo cu_emlrtRSI;
extern emlrtRSInfo xu_emlrtRSI;
extern emlrtRSInfo yu_emlrtRSI;
extern emlrtRSInfo av_emlrtRSI;
extern emlrtRSInfo bv_emlrtRSI;
extern emlrtRSInfo cv_emlrtRSI;
extern emlrtRSInfo dv_emlrtRSI;
extern emlrtRSInfo ev_emlrtRSI;
extern emlrtRSInfo fv_emlrtRSI;
extern emlrtRSInfo gv_emlrtRSI;
extern emlrtRSInfo hv_emlrtRSI;
extern emlrtRSInfo iv_emlrtRSI;
extern emlrtRSInfo jv_emlrtRSI;
extern emlrtRSInfo kv_emlrtRSI;
extern emlrtRSInfo lv_emlrtRSI;
extern emlrtRSInfo mv_emlrtRSI;
extern emlrtRSInfo nv_emlrtRSI;
extern emlrtRSInfo ov_emlrtRSI;
extern emlrtRSInfo pv_emlrtRSI;
extern emlrtRSInfo qv_emlrtRSI;
extern emlrtRSInfo nw_emlrtRSI;
extern emlrtRSInfo ow_emlrtRSI;
extern emlrtRSInfo wy_emlrtRSI;
extern emlrtRSInfo xy_emlrtRSI;
extern emlrtRSInfo yy_emlrtRSI;
extern emlrtRSInfo aab_emlrtRSI;
extern emlrtRSInfo bab_emlrtRSI;
extern emlrtRSInfo cab_emlrtRSI;
extern emlrtRSInfo dab_emlrtRSI;
extern emlrtRSInfo eab_emlrtRSI;
extern emlrtRSInfo fab_emlrtRSI;
extern emlrtRSInfo gab_emlrtRSI;
extern emlrtRSInfo hab_emlrtRSI;
extern emlrtRSInfo iab_emlrtRSI;
extern emlrtRSInfo lab_emlrtRSI;
extern emlrtRSInfo obb_emlrtRSI;
extern emlrtRSInfo pbb_emlrtRSI;
extern emlrtRSInfo qbb_emlrtRSI;
extern emlrtRSInfo rbb_emlrtRSI;
extern emlrtRSInfo sbb_emlrtRSI;
extern emlrtRSInfo tbb_emlrtRSI;
extern emlrtRSInfo wbb_emlrtRSI;
extern emlrtRSInfo xbb_emlrtRSI;
extern emlrtRSInfo ybb_emlrtRSI;
extern emlrtRSInfo mcb_emlrtRSI;
extern emlrtRSInfo ncb_emlrtRSI;
extern emlrtRSInfo wdb_emlrtRSI;
extern emlrtRSInfo igb_emlrtRSI;
extern emlrtRSInfo jgb_emlrtRSI;
extern emlrtRSInfo kgb_emlrtRSI;
extern emlrtRSInfo lgb_emlrtRSI;
extern emlrtRSInfo mgb_emlrtRSI;
extern emlrtRSInfo ngb_emlrtRSI;
extern emlrtRSInfo wgb_emlrtRSI;
extern emlrtRSInfo tib_emlrtRSI;
extern emlrtRSInfo jjb_emlrtRSI;
extern emlrtMCInfo d_emlrtMCI;
extern omp_lock_t emlrtLockGlobal;
extern omp_nest_lock_t trackingAlgorithm_nestLockGlobal;
extern emlrtRTEInfo c_emlrtRTEI;
extern emlrtRTEInfo d_emlrtRTEI;
extern emlrtRTEInfo f_emlrtRTEI;
extern emlrtRTEInfo g_emlrtRTEI;
extern emlrtRTEInfo h_emlrtRTEI;
extern emlrtRTEInfo i_emlrtRTEI;
extern emlrtBCInfo y_emlrtBCI;
extern emlrtRTEInfo l_emlrtRTEI;
extern emlrtRTEInfo o_emlrtRTEI;
extern emlrtRTEInfo p_emlrtRTEI;
extern emlrtRTEInfo s_emlrtRTEI;
extern emlrtRTEInfo u_emlrtRTEI;
extern emlrtRTEInfo x_emlrtRTEI;
extern emlrtRTEInfo eb_emlrtRTEI;
extern emlrtRTEInfo mb_emlrtRTEI;
extern emlrtRTEInfo rb_emlrtRTEI;
extern emlrtRTEInfo sb_emlrtRTEI;
extern emlrtDCInfo g_emlrtDCI;
extern emlrtRTEInfo ub_emlrtRTEI;
extern emlrtRTEInfo xb_emlrtRTEI;
extern emlrtRTEInfo dc_emlrtRTEI;
extern emlrtRTEInfo ke_emlrtRTEI;
extern emlrtRTEInfo mf_emlrtRTEI;
extern emlrtRTEInfo ag_emlrtRTEI;
extern emlrtRTEInfo bh_emlrtRTEI;
extern const int8_T iv[36];
extern const int8_T iv1[16];
extern const char_T cv[13];
extern emlrtRSInfo hnb_emlrtRSI;

/* End of code generation (trackingAlgorithm_data.h) */
