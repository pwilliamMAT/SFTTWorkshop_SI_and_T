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
extern emlrtRSInfo i_emlrtRSI;
extern emlrtRSInfo k_emlrtRSI;
extern emlrtRSInfo l_emlrtRSI;
extern emlrtRSInfo bd_emlrtRSI;
extern emlrtRSInfo cd_emlrtRSI;
extern emlrtRSInfo dd_emlrtRSI;
extern emlrtRSInfo ed_emlrtRSI;
extern emlrtRSInfo fd_emlrtRSI;
extern emlrtRSInfo gd_emlrtRSI;
extern emlrtRSInfo hd_emlrtRSI;
extern emlrtRSInfo kd_emlrtRSI;
extern emlrtRSInfo ld_emlrtRSI;
extern emlrtRSInfo rd_emlrtRSI;
extern emlrtRSInfo sd_emlrtRSI;
extern emlrtRSInfo fe_emlrtRSI;
extern emlrtRSInfo ge_emlrtRSI;
extern emlrtRSInfo he_emlrtRSI;
extern emlrtRSInfo ie_emlrtRSI;
extern emlrtRSInfo je_emlrtRSI;
extern emlrtRSInfo ke_emlrtRSI;
extern emlrtRSInfo le_emlrtRSI;
extern emlrtRSInfo me_emlrtRSI;
extern emlrtRSInfo ne_emlrtRSI;
extern emlrtRSInfo oe_emlrtRSI;
extern emlrtRSInfo pe_emlrtRSI;
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
extern emlrtRSInfo jf_emlrtRSI;
extern emlrtRSInfo kf_emlrtRSI;
extern emlrtRSInfo lf_emlrtRSI;
extern emlrtRSInfo mf_emlrtRSI;
extern emlrtRSInfo nf_emlrtRSI;
extern emlrtRSInfo of_emlrtRSI;
extern emlrtRSInfo pf_emlrtRSI;
extern emlrtRSInfo qf_emlrtRSI;
extern emlrtRSInfo rf_emlrtRSI;
extern emlrtRSInfo tf_emlrtRSI;
extern emlrtRSInfo cg_emlrtRSI;
extern emlrtRSInfo pg_emlrtRSI;
extern emlrtRSInfo qg_emlrtRSI;
extern emlrtRSInfo mh_emlrtRSI;
extern emlrtRSInfo nh_emlrtRSI;
extern emlrtRSInfo oh_emlrtRSI;
extern emlrtRSInfo ph_emlrtRSI;
extern emlrtRSInfo fl_emlrtRSI;
extern emlrtRSInfo gl_emlrtRSI;
extern emlrtRSInfo hl_emlrtRSI;
extern emlrtRSInfo il_emlrtRSI;
extern emlrtRSInfo jl_emlrtRSI;
extern emlrtRSInfo on_emlrtRSI;
extern emlrtRSInfo pn_emlrtRSI;
extern emlrtRSInfo qn_emlrtRSI;
extern emlrtRSInfo tn_emlrtRSI;
extern emlrtRSInfo un_emlrtRSI;
extern emlrtRSInfo vn_emlrtRSI;
extern emlrtRSInfo wn_emlrtRSI;
extern emlrtRSInfo yn_emlrtRSI;
extern emlrtRSInfo yp_emlrtRSI;
extern emlrtRSInfo aq_emlrtRSI;
extern emlrtRSInfo gq_emlrtRSI;
extern emlrtRSInfo lq_emlrtRSI;
extern emlrtRSInfo mq_emlrtRSI;
extern emlrtRSInfo nq_emlrtRSI;
extern emlrtRSInfo oq_emlrtRSI;
extern emlrtRSInfo pq_emlrtRSI;
extern emlrtRSInfo hr_emlrtRSI;
extern emlrtRSInfo ms_emlrtRSI;
extern emlrtRSInfo xs_emlrtRSI;
extern emlrtRSInfo ys_emlrtRSI;
extern emlrtRSInfo ct_emlrtRSI;
extern emlrtRSInfo rt_emlrtRSI;
extern emlrtRSInfo st_emlrtRSI;
extern emlrtRSInfo ru_emlrtRSI;
extern emlrtRSInfo nv_emlrtRSI;
extern emlrtRSInfo ov_emlrtRSI;
extern emlrtRSInfo pv_emlrtRSI;
extern emlrtRSInfo qv_emlrtRSI;
extern emlrtRSInfo rv_emlrtRSI;
extern emlrtRSInfo sv_emlrtRSI;
extern emlrtRSInfo tv_emlrtRSI;
extern emlrtRSInfo uv_emlrtRSI;
extern emlrtRSInfo vv_emlrtRSI;
extern emlrtRSInfo wv_emlrtRSI;
extern emlrtRSInfo xv_emlrtRSI;
extern emlrtRSInfo yv_emlrtRSI;
extern emlrtRSInfo aw_emlrtRSI;
extern emlrtRSInfo bw_emlrtRSI;
extern emlrtRSInfo cw_emlrtRSI;
extern emlrtRSInfo jw_emlrtRSI;
extern emlrtRSInfo kw_emlrtRSI;
extern emlrtRSInfo lw_emlrtRSI;
extern emlrtRSInfo mw_emlrtRSI;
extern emlrtRSInfo nw_emlrtRSI;
extern emlrtRSInfo pw_emlrtRSI;
extern emlrtRSInfo qw_emlrtRSI;
extern emlrtRSInfo xw_emlrtRSI;
extern emlrtRSInfo yw_emlrtRSI;
extern emlrtRSInfo qx_emlrtRSI;
extern emlrtRSInfo rx_emlrtRSI;
extern emlrtRSInfo sx_emlrtRSI;
extern emlrtRSInfo tx_emlrtRSI;
extern emlrtRSInfo ay_emlrtRSI;
extern emlrtRSInfo by_emlrtRSI;
extern emlrtRSInfo kbb_emlrtRSI;
extern emlrtRSInfo lbb_emlrtRSI;
extern emlrtRSInfo mbb_emlrtRSI;
extern emlrtRSInfo nbb_emlrtRSI;
extern emlrtRSInfo obb_emlrtRSI;
extern emlrtRSInfo pbb_emlrtRSI;
extern emlrtRSInfo qbb_emlrtRSI;
extern emlrtRSInfo rbb_emlrtRSI;
extern emlrtRSInfo sbb_emlrtRSI;
extern emlrtRSInfo tbb_emlrtRSI;
extern emlrtRSInfo ubb_emlrtRSI;
extern emlrtRSInfo vbb_emlrtRSI;
extern emlrtRSInfo ybb_emlrtRSI;
extern emlrtRSInfo edb_emlrtRSI;
extern emlrtRSInfo fdb_emlrtRSI;
extern emlrtRSInfo gdb_emlrtRSI;
extern emlrtRSInfo hdb_emlrtRSI;
extern emlrtRSInfo idb_emlrtRSI;
extern emlrtRSInfo ldb_emlrtRSI;
extern emlrtRSInfo dfb_emlrtRSI;
extern emlrtRSInfo phb_emlrtRSI;
extern emlrtRSInfo qhb_emlrtRSI;
extern emlrtRSInfo rhb_emlrtRSI;
extern emlrtRSInfo shb_emlrtRSI;
extern emlrtRSInfo thb_emlrtRSI;
extern emlrtRSInfo dib_emlrtRSI;
extern emlrtRSInfo kkb_emlrtRSI;
extern emlrtMCInfo d_emlrtMCI;
extern omp_lock_t emlrtLockGlobal;
extern omp_nest_lock_t trackingAlgorithm_nestLockGlobal;
extern emlrtRTEInfo b_emlrtRTEI;
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
extern emlrtRTEInfo v_emlrtRTEI;
extern emlrtRTEInfo x_emlrtRTEI;
extern emlrtRTEInfo nb_emlrtRTEI;
extern emlrtRTEInfo pb_emlrtRTEI;
extern emlrtRTEInfo ub_emlrtRTEI;
extern emlrtRTEInfo vb_emlrtRTEI;
extern emlrtDCInfo g_emlrtDCI;
extern emlrtRTEInfo ie_emlrtRTEI;
extern emlrtRTEInfo mf_emlrtRTEI;
extern emlrtRTEInfo ag_emlrtRTEI;
extern emlrtRTEInfo bh_emlrtRTEI;
extern const int8_T iv[36];
extern const int8_T iv1[16];
extern const char_T cv[13];
extern const char_T cv1[10];
extern emlrtRSInfo yob_emlrtRSI;

/* End of code generation (trackingAlgorithm_data.h) */
