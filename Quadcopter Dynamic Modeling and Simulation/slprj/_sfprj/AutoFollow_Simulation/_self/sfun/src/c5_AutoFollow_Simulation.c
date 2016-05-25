/* Include files */

#include <stddef.h>
#include "blas.h"
#include "AutoFollow_Simulation_sfun.h"
#include "c5_AutoFollow_Simulation.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "AutoFollow_Simulation_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c5_debug_family_names[14] = { "Rback", "cam", "human",
  "nargin", "nargout", "thetaq", "psiq", "phiq", "hr", "htheta", "hpsi", "hx",
  "hy", "hz" };

static const char * c5_b_debug_family_names[8] = { "nargin", "nargout", "r",
  "theta", "psi", "x", "y", "z" };

/* Function Declarations */
static void initialize_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initialize_params_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void enable_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void disable_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void set_sim_state_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c5_st);
static void finalize_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void sf_gateway_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initSimStructsc5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static real_T c5_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_hz, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_c_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[3]);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_d_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[9]);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(const char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u);
static void c5_eml_scalar_eg(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c5_eml_xgemm(SFc5_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c5_A[9], real_T c5_B[3], real_T c5_C[3], real_T c5_b_C[3]);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_e_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_f_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_AutoFollow_Simulation, const
  char_T *c5_identifier);
static uint8_T c5_g_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_eml_xgemm(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c5_A[9], real_T c5_B[3], real_T c5_C[3]);
static void init_dsm_address_info(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_is_active_c5_AutoFollow_Simulation = 0U;
}

static void initialize_params_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_update_debugger_state_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  real_T c5_hoistedGlobal;
  real_T c5_u;
  const mxArray *c5_b_y = NULL;
  real_T c5_b_hoistedGlobal;
  real_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T c5_c_hoistedGlobal;
  real_T c5_c_u;
  const mxArray *c5_d_y = NULL;
  uint8_T c5_d_hoistedGlobal;
  uint8_T c5_d_u;
  const mxArray *c5_e_y = NULL;
  real_T *c5_hx;
  real_T *c5_hy;
  real_T *c5_hz;
  c5_hz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_hy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_hx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(4, 1), false);
  c5_hoistedGlobal = *c5_hx;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_b_hoistedGlobal = *c5_hy;
  c5_b_u = c5_b_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_c_hoistedGlobal = *c5_hz;
  c5_c_u = c5_c_hoistedGlobal;
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", &c5_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 2, c5_d_y);
  c5_d_hoistedGlobal = chartInstance->c5_is_active_c5_AutoFollow_Simulation;
  c5_d_u = c5_d_hoistedGlobal;
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", &c5_d_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 3, c5_e_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T *c5_hx;
  real_T *c5_hy;
  real_T *c5_hz;
  c5_hz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_hy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_hx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = true;
  c5_u = sf_mex_dup(c5_st);
  *c5_hx = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
    "hx");
  *c5_hy = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
    "hy");
  *c5_hz = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 2)),
    "hz");
  chartInstance->c5_is_active_c5_AutoFollow_Simulation = c5_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 3)),
     "is_active_c5_AutoFollow_Simulation");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_AutoFollow_Simulation(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_c_hoistedGlobal;
  real_T c5_d_hoistedGlobal;
  real_T c5_e_hoistedGlobal;
  real_T c5_f_hoistedGlobal;
  real_T c5_thetaq;
  real_T c5_psiq;
  real_T c5_phiq;
  real_T c5_hr;
  real_T c5_htheta;
  real_T c5_hpsi;
  uint32_T c5_debug_family_var_map[14];
  real_T c5_Rback[9];
  real_T c5_cam[3];
  real_T c5_human[3];
  real_T c5_nargin = 6.0;
  real_T c5_nargout = 3.0;
  real_T c5_hx;
  real_T c5_hy;
  real_T c5_hz;
  real_T c5_r;
  real_T c5_theta;
  real_T c5_psi;
  uint32_T c5_b_debug_family_var_map[8];
  real_T c5_b_nargin = 3.0;
  real_T c5_b_nargout = 3.0;
  real_T c5_b_hx;
  real_T c5_b_hy;
  real_T c5_b_hz;
  real_T c5_x;
  real_T c5_b_x;
  real_T c5_c_x;
  real_T c5_d_x;
  real_T c5_e_x;
  real_T c5_f_x;
  real_T c5_g_x;
  real_T c5_h_x;
  real_T c5_i_x;
  real_T c5_j_x;
  real_T c5_k_x;
  real_T c5_l_x;
  real_T c5_m_x;
  real_T c5_n_x;
  real_T c5_o_x;
  real_T c5_p_x;
  real_T c5_q_x;
  real_T c5_r_x;
  real_T c5_s_x;
  real_T c5_t_x;
  real_T c5_u_x;
  real_T c5_v_x;
  real_T c5_w_x;
  real_T c5_x_x;
  real_T c5_y_x;
  real_T c5_ab_x;
  real_T c5_bb_x;
  real_T c5_cb_x;
  real_T c5_db_x;
  real_T c5_eb_x;
  real_T c5_fb_x;
  real_T c5_gb_x;
  real_T c5_hb_x;
  real_T c5_ib_x;
  real_T c5_jb_x;
  real_T c5_kb_x;
  real_T c5_lb_x;
  real_T c5_mb_x;
  real_T c5_nb_x;
  real_T c5_ob_x;
  real_T c5_pb_x;
  real_T c5_qb_x;
  real_T c5_rb_x;
  real_T c5_sb_x;
  real_T c5_tb_x;
  real_T c5_ub_x;
  real_T c5_vb_x;
  real_T c5_wb_x;
  real_T c5_xb_x;
  real_T c5_yb_x;
  real_T c5_ac_x;
  real_T c5_bc_x;
  real_T c5_cc_x;
  real_T c5_dc_x;
  real_T c5_ec_x;
  real_T c5_fc_x;
  real_T c5_gc_x;
  real_T c5_hc_x;
  real_T c5_ic_x;
  real_T c5_jc_x;
  real_T c5_kc_x;
  real_T c5_lc_x;
  real_T c5_mc_x;
  real_T c5_nc_x;
  real_T c5_oc_x;
  real_T c5_pc_x;
  real_T c5_qc_x;
  real_T c5_rc_x;
  int32_T c5_i0;
  real_T c5_a[9];
  int32_T c5_i1;
  real_T c5_b[3];
  int32_T c5_i2;
  int32_T c5_i3;
  int32_T c5_i4;
  real_T c5_dv0[9];
  int32_T c5_i5;
  real_T c5_dv1[3];
  int32_T c5_i6;
  real_T c5_dv2[9];
  int32_T c5_i7;
  real_T c5_dv3[3];
  real_T *c5_c_hz;
  real_T *c5_c_hy;
  real_T *c5_c_hx;
  real_T *c5_b_hpsi;
  real_T *c5_b_htheta;
  real_T *c5_b_hr;
  real_T *c5_b_phiq;
  real_T *c5_b_psiq;
  real_T *c5_b_thetaq;
  c5_c_hz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_c_hy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_b_hpsi = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c5_b_htheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c5_b_hr = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c5_b_phiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_b_psiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_c_hx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_thetaq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c5_b_thetaq, 0U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_thetaq;
  c5_b_hoistedGlobal = *c5_b_psiq;
  c5_c_hoistedGlobal = *c5_b_phiq;
  c5_d_hoistedGlobal = *c5_b_hr;
  c5_e_hoistedGlobal = *c5_b_htheta;
  c5_f_hoistedGlobal = *c5_b_hpsi;
  c5_thetaq = c5_hoistedGlobal;
  c5_psiq = c5_b_hoistedGlobal;
  c5_phiq = c5_c_hoistedGlobal;
  c5_hr = c5_d_hoistedGlobal;
  c5_htheta = c5_e_hoistedGlobal;
  c5_hpsi = c5_f_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Rback, 0U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_cam, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_human, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 4U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_thetaq, 5U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_psiq, 6U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_phiq, 7U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_hr, 8U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_htheta, 9U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_hpsi, 10U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_hx, 11U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_hy, 12U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_hz, 13U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
  c5_r = c5_hr;
  c5_theta = c5_htheta;
  c5_psi = c5_hpsi;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c5_b_debug_family_names,
    c5_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_r, 2U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_theta, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_psi, 4U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_hx, 5U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_hy, 6U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_hz, 7U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 18);
  c5_x = c5_psi;
  c5_b_x = c5_x;
  c5_b_x = muDoubleScalarSin(c5_b_x);
  c5_c_x = c5_theta;
  c5_d_x = c5_c_x;
  c5_d_x = muDoubleScalarCos(c5_d_x);
  c5_b_hx = c5_r * c5_b_x * c5_d_x;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 19);
  c5_e_x = c5_psi;
  c5_f_x = c5_e_x;
  c5_f_x = muDoubleScalarSin(c5_f_x);
  c5_g_x = c5_theta;
  c5_h_x = c5_g_x;
  c5_h_x = muDoubleScalarSin(c5_h_x);
  c5_b_hy = c5_r * c5_f_x * c5_h_x;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 20);
  c5_i_x = c5_psi;
  c5_j_x = c5_i_x;
  c5_j_x = muDoubleScalarCos(c5_j_x);
  c5_b_hz = c5_r * c5_j_x;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -20);
  _SFD_SYMBOL_SCOPE_POP();
  c5_hx = c5_b_hx;
  c5_hy = c5_b_hy;
  c5_hz = c5_b_hz;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 6);
  c5_k_x = c5_psiq;
  c5_l_x = c5_k_x;
  c5_l_x = muDoubleScalarCos(c5_l_x);
  c5_m_x = c5_thetaq;
  c5_n_x = c5_m_x;
  c5_n_x = muDoubleScalarCos(c5_n_x);
  c5_o_x = c5_psiq;
  c5_p_x = c5_o_x;
  c5_p_x = muDoubleScalarCos(c5_p_x);
  c5_q_x = c5_thetaq;
  c5_r_x = c5_q_x;
  c5_r_x = muDoubleScalarSin(c5_r_x);
  c5_s_x = c5_phiq;
  c5_t_x = c5_s_x;
  c5_t_x = muDoubleScalarSin(c5_t_x);
  c5_u_x = c5_psiq;
  c5_v_x = c5_u_x;
  c5_v_x = muDoubleScalarSin(c5_v_x);
  c5_w_x = c5_phiq;
  c5_x_x = c5_w_x;
  c5_x_x = muDoubleScalarCos(c5_x_x);
  c5_y_x = c5_psiq;
  c5_ab_x = c5_y_x;
  c5_ab_x = muDoubleScalarCos(c5_ab_x);
  c5_bb_x = c5_thetaq;
  c5_cb_x = c5_bb_x;
  c5_cb_x = muDoubleScalarSin(c5_cb_x);
  c5_db_x = c5_phiq;
  c5_eb_x = c5_db_x;
  c5_eb_x = muDoubleScalarCos(c5_eb_x);
  c5_fb_x = c5_psiq;
  c5_gb_x = c5_fb_x;
  c5_gb_x = muDoubleScalarSin(c5_gb_x);
  c5_hb_x = c5_phiq;
  c5_ib_x = c5_hb_x;
  c5_ib_x = muDoubleScalarSin(c5_ib_x);
  c5_jb_x = c5_psiq;
  c5_kb_x = c5_jb_x;
  c5_kb_x = muDoubleScalarSin(c5_kb_x);
  c5_lb_x = c5_thetaq;
  c5_mb_x = c5_lb_x;
  c5_mb_x = muDoubleScalarCos(c5_mb_x);
  c5_nb_x = c5_psiq;
  c5_ob_x = c5_nb_x;
  c5_ob_x = muDoubleScalarSin(c5_ob_x);
  c5_pb_x = c5_thetaq;
  c5_qb_x = c5_pb_x;
  c5_qb_x = muDoubleScalarSin(c5_qb_x);
  c5_rb_x = c5_phiq;
  c5_sb_x = c5_rb_x;
  c5_sb_x = muDoubleScalarSin(c5_sb_x);
  c5_tb_x = c5_psiq;
  c5_ub_x = c5_tb_x;
  c5_ub_x = muDoubleScalarCos(c5_ub_x);
  c5_vb_x = c5_phiq;
  c5_wb_x = c5_vb_x;
  c5_wb_x = muDoubleScalarCos(c5_wb_x);
  c5_xb_x = c5_psiq;
  c5_yb_x = c5_xb_x;
  c5_yb_x = muDoubleScalarSin(c5_yb_x);
  c5_ac_x = c5_thetaq;
  c5_bc_x = c5_ac_x;
  c5_bc_x = muDoubleScalarSin(c5_bc_x);
  c5_cc_x = c5_phiq;
  c5_dc_x = c5_cc_x;
  c5_dc_x = muDoubleScalarCos(c5_dc_x);
  c5_ec_x = c5_psiq;
  c5_fc_x = c5_ec_x;
  c5_fc_x = muDoubleScalarCos(c5_fc_x);
  c5_gc_x = c5_phiq;
  c5_hc_x = c5_gc_x;
  c5_hc_x = muDoubleScalarSin(c5_hc_x);
  c5_ic_x = c5_thetaq;
  c5_jc_x = c5_ic_x;
  c5_jc_x = muDoubleScalarSin(c5_jc_x);
  c5_kc_x = c5_thetaq;
  c5_lc_x = c5_kc_x;
  c5_lc_x = muDoubleScalarCos(c5_lc_x);
  c5_mc_x = c5_phiq;
  c5_nc_x = c5_mc_x;
  c5_nc_x = muDoubleScalarSin(c5_nc_x);
  c5_oc_x = c5_thetaq;
  c5_pc_x = c5_oc_x;
  c5_pc_x = muDoubleScalarCos(c5_pc_x);
  c5_qc_x = c5_phiq;
  c5_rc_x = c5_qc_x;
  c5_rc_x = muDoubleScalarCos(c5_rc_x);
  c5_Rback[0] = c5_l_x * c5_n_x;
  c5_Rback[3] = c5_p_x * c5_r_x * c5_t_x - c5_v_x * c5_x_x;
  c5_Rback[6] = c5_ab_x * c5_cb_x * c5_eb_x + c5_gb_x * c5_ib_x;
  c5_Rback[1] = c5_kb_x * c5_mb_x;
  c5_Rback[4] = c5_ob_x * c5_qb_x * c5_sb_x + c5_ub_x * c5_wb_x;
  c5_Rback[7] = c5_yb_x * c5_bc_x * c5_dc_x - c5_fc_x * c5_hc_x;
  c5_Rback[2] = -c5_jc_x;
  c5_Rback[5] = c5_lc_x * c5_nc_x;
  c5_Rback[8] = c5_pc_x * c5_rc_x;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 9);
  c5_cam[0] = c5_hx;
  c5_cam[1] = c5_hy;
  c5_cam[2] = c5_hz;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  for (c5_i0 = 0; c5_i0 < 9; c5_i0++) {
    c5_a[c5_i0] = c5_Rback[c5_i0];
  }

  for (c5_i1 = 0; c5_i1 < 3; c5_i1++) {
    c5_b[c5_i1] = c5_cam[c5_i1];
  }

  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  for (c5_i2 = 0; c5_i2 < 3; c5_i2++) {
    c5_human[c5_i2] = 0.0;
  }

  for (c5_i3 = 0; c5_i3 < 3; c5_i3++) {
    c5_human[c5_i3] = 0.0;
  }

  for (c5_i4 = 0; c5_i4 < 9; c5_i4++) {
    c5_dv0[c5_i4] = c5_a[c5_i4];
  }

  for (c5_i5 = 0; c5_i5 < 3; c5_i5++) {
    c5_dv1[c5_i5] = c5_b[c5_i5];
  }

  for (c5_i6 = 0; c5_i6 < 9; c5_i6++) {
    c5_dv2[c5_i6] = c5_dv0[c5_i6];
  }

  for (c5_i7 = 0; c5_i7 < 3; c5_i7++) {
    c5_dv3[c5_i7] = c5_dv1[c5_i7];
  }

  c5_b_eml_xgemm(chartInstance, c5_dv2, c5_dv3, c5_human);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
  c5_hx = c5_human[0];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 13);
  c5_hy = c5_human[1];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 14);
  c5_hz = c5_human[2];
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -14);
  _SFD_SYMBOL_SCOPE_POP();
  *c5_c_hx = c5_hx;
  *c5_c_hy = c5_hy;
  *c5_c_hz = c5_hz;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_AutoFollow_SimulationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*c5_c_hx, 1U);
  _SFD_DATA_RANGE_CHECK(*c5_b_psiq, 2U);
  _SFD_DATA_RANGE_CHECK(*c5_b_phiq, 3U);
  _SFD_DATA_RANGE_CHECK(*c5_b_hr, 4U);
  _SFD_DATA_RANGE_CHECK(*c5_b_htheta, 5U);
  _SFD_DATA_RANGE_CHECK(*c5_b_hpsi, 6U);
  _SFD_DATA_RANGE_CHECK(*c5_c_hy, 7U);
  _SFD_DATA_RANGE_CHECK(*c5_c_hz, 8U);
}

static void initSimStructsc5_AutoFollow_Simulation
  (SFc5_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber)
{
  (void)c5_machineNumber;
  (void)c5_chartNumber;
  (void)c5_instanceNumber;
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_hz, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_hz), &c5_thisId);
  sf_mex_destroy(&c5_hz);
  return c5_y;
}

static real_T c5_b_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_hz;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_hz = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_hz), &c5_thisId);
  sf_mex_destroy(&c5_hz);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i8;
  real_T c5_b_inData[3];
  int32_T c5_i9;
  real_T c5_u[3];
  const mxArray *c5_y = NULL;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i8 = 0; c5_i8 < 3; c5_i8++) {
    c5_b_inData[c5_i8] = (*(real_T (*)[3])c5_inData)[c5_i8];
  }

  for (c5_i9 = 0; c5_i9 < 3; c5_i9++) {
    c5_u[c5_i9] = c5_b_inData[c5_i9];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_c_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[3])
{
  real_T c5_dv4[3];
  int32_T c5_i10;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv4, 1, 0, 0U, 1, 0U, 1, 3);
  for (c5_i10 = 0; c5_i10 < 3; c5_i10++) {
    c5_y[c5_i10] = c5_dv4[c5_i10];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_human;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[3];
  int32_T c5_i11;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_human = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_human), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_human);
  for (c5_i11 = 0; c5_i11 < 3; c5_i11++) {
    (*(real_T (*)[3])c5_outData)[c5_i11] = c5_y[c5_i11];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i12;
  int32_T c5_i13;
  int32_T c5_i14;
  real_T c5_b_inData[9];
  int32_T c5_i15;
  int32_T c5_i16;
  int32_T c5_i17;
  real_T c5_u[9];
  const mxArray *c5_y = NULL;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i12 = 0;
  for (c5_i13 = 0; c5_i13 < 3; c5_i13++) {
    for (c5_i14 = 0; c5_i14 < 3; c5_i14++) {
      c5_b_inData[c5_i14 + c5_i12] = (*(real_T (*)[9])c5_inData)[c5_i14 + c5_i12];
    }

    c5_i12 += 3;
  }

  c5_i15 = 0;
  for (c5_i16 = 0; c5_i16 < 3; c5_i16++) {
    for (c5_i17 = 0; c5_i17 < 3; c5_i17++) {
      c5_u[c5_i17 + c5_i15] = c5_b_inData[c5_i17 + c5_i15];
    }

    c5_i15 += 3;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_d_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[9])
{
  real_T c5_dv5[9];
  int32_T c5_i18;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv5, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c5_i18 = 0; c5_i18 < 9; c5_i18++) {
    c5_y[c5_i18] = c5_dv5[c5_i18];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_Rback;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[9];
  int32_T c5_i19;
  int32_T c5_i20;
  int32_T c5_i21;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_Rback = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_Rback), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_Rback);
  c5_i19 = 0;
  for (c5_i20 = 0; c5_i20 < 3; c5_i20++) {
    for (c5_i21 = 0; c5_i21 < 3; c5_i21++) {
      (*(real_T (*)[9])c5_outData)[c5_i21 + c5_i19] = c5_y[c5_i21 + c5_i19];
    }

    c5_i19 += 3;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray *sf_c5_AutoFollow_Simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 17, 1),
                false);
  c5_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  const mxArray *c5_rhs3 = NULL;
  const mxArray *c5_lhs3 = NULL;
  const mxArray *c5_rhs4 = NULL;
  const mxArray *c5_lhs4 = NULL;
  const mxArray *c5_rhs5 = NULL;
  const mxArray *c5_lhs5 = NULL;
  const mxArray *c5_rhs6 = NULL;
  const mxArray *c5_lhs6 = NULL;
  const mxArray *c5_rhs7 = NULL;
  const mxArray *c5_lhs7 = NULL;
  const mxArray *c5_rhs8 = NULL;
  const mxArray *c5_lhs8 = NULL;
  const mxArray *c5_rhs9 = NULL;
  const mxArray *c5_lhs9 = NULL;
  const mxArray *c5_rhs10 = NULL;
  const mxArray *c5_lhs10 = NULL;
  const mxArray *c5_rhs11 = NULL;
  const mxArray *c5_lhs11 = NULL;
  const mxArray *c5_rhs12 = NULL;
  const mxArray *c5_lhs12 = NULL;
  const mxArray *c5_rhs13 = NULL;
  const mxArray *c5_lhs13 = NULL;
  const mxArray *c5_rhs14 = NULL;
  const mxArray *c5_lhs14 = NULL;
  const mxArray *c5_rhs15 = NULL;
  const mxArray *c5_lhs15 = NULL;
  const mxArray *c5_rhs16 = NULL;
  const mxArray *c5_lhs16 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sin"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("cos"), "name", "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c5_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c5_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c5_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c5_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c5_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c5_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xgemm"), "name", "name", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c5_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c5_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c5_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c5_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c5_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c5_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c5_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c5_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs16), "lhs", "lhs",
                  16);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
  sf_mex_destroy(&c5_rhs3);
  sf_mex_destroy(&c5_lhs3);
  sf_mex_destroy(&c5_rhs4);
  sf_mex_destroy(&c5_lhs4);
  sf_mex_destroy(&c5_rhs5);
  sf_mex_destroy(&c5_lhs5);
  sf_mex_destroy(&c5_rhs6);
  sf_mex_destroy(&c5_lhs6);
  sf_mex_destroy(&c5_rhs7);
  sf_mex_destroy(&c5_lhs7);
  sf_mex_destroy(&c5_rhs8);
  sf_mex_destroy(&c5_lhs8);
  sf_mex_destroy(&c5_rhs9);
  sf_mex_destroy(&c5_lhs9);
  sf_mex_destroy(&c5_rhs10);
  sf_mex_destroy(&c5_lhs10);
  sf_mex_destroy(&c5_rhs11);
  sf_mex_destroy(&c5_lhs11);
  sf_mex_destroy(&c5_rhs12);
  sf_mex_destroy(&c5_lhs12);
  sf_mex_destroy(&c5_rhs13);
  sf_mex_destroy(&c5_lhs13);
  sf_mex_destroy(&c5_rhs14);
  sf_mex_destroy(&c5_lhs14);
  sf_mex_destroy(&c5_rhs15);
  sf_mex_destroy(&c5_lhs15);
  sf_mex_destroy(&c5_rhs16);
  sf_mex_destroy(&c5_lhs16);
}

static const mxArray *c5_emlrt_marshallOut(const char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), false);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), false);
  return c5_y;
}

static void c5_eml_scalar_eg(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c5_eml_xgemm(SFc5_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c5_A[9], real_T c5_B[3], real_T c5_C[3], real_T c5_b_C[3])
{
  int32_T c5_i22;
  int32_T c5_i23;
  real_T c5_b_A[9];
  int32_T c5_i24;
  real_T c5_b_B[3];
  for (c5_i22 = 0; c5_i22 < 3; c5_i22++) {
    c5_b_C[c5_i22] = c5_C[c5_i22];
  }

  for (c5_i23 = 0; c5_i23 < 9; c5_i23++) {
    c5_b_A[c5_i23] = c5_A[c5_i23];
  }

  for (c5_i24 = 0; c5_i24 < 3; c5_i24++) {
    c5_b_B[c5_i24] = c5_B[c5_i24];
  }

  c5_b_eml_xgemm(chartInstance, c5_b_A, c5_b_B, c5_b_C);
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static int32_T c5_e_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i25;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i25, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i25;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_f_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_AutoFollow_Simulation, const
  char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_AutoFollow_Simulation), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_AutoFollow_Simulation);
  return c5_y;
}

static uint8_T c5_g_emlrt_marshallIn(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_eml_xgemm(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c5_A[9], real_T c5_B[3], real_T c5_C[3])
{
  int32_T c5_i26;
  int32_T c5_i27;
  int32_T c5_i28;
  (void)chartInstance;
  for (c5_i26 = 0; c5_i26 < 3; c5_i26++) {
    c5_C[c5_i26] = 0.0;
    c5_i27 = 0;
    for (c5_i28 = 0; c5_i28 < 3; c5_i28++) {
      c5_C[c5_i26] += c5_A[c5_i27 + c5_i26] * c5_B[c5_i28];
      c5_i27 += 3;
    }
  }
}

static void init_dsm_address_info(SFc5_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c5_AutoFollow_Simulation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(638916204U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3928780594U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3354222582U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(774811785U);
}

mxArray *sf_c5_AutoFollow_Simulation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("8gjAqZTpYOmyJGOAsoyXrH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_AutoFollow_Simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_AutoFollow_Simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c5_AutoFollow_Simulation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"hx\",},{M[1],M[15],T\"hy\",},{M[1],M[16],T\"hz\",},{M[8],M[0],T\"is_active_c5_AutoFollow_Simulation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_AutoFollow_Simulation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _AutoFollow_SimulationMachineNumber_,
           5,
           1,
           1,
           0,
           9,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_AutoFollow_SimulationMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_AutoFollow_SimulationMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _AutoFollow_SimulationMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"thetaq");
          _SFD_SET_DATA_PROPS(1,2,0,1,"hx");
          _SFD_SET_DATA_PROPS(2,1,1,0,"psiq");
          _SFD_SET_DATA_PROPS(3,1,1,0,"phiq");
          _SFD_SET_DATA_PROPS(4,1,1,0,"hr");
          _SFD_SET_DATA_PROPS(5,1,1,0,"htheta");
          _SFD_SET_DATA_PROPS(6,1,1,0,"hpsi");
          _SFD_SET_DATA_PROPS(7,2,0,1,"hy");
          _SFD_SET_DATA_PROPS(8,2,0,1,"hz");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,2,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,678);
        _SFD_CV_INIT_EML_FCN(0,1,"sphere2cart",680,-1,806);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);

        {
          real_T *c5_thetaq;
          real_T *c5_hx;
          real_T *c5_psiq;
          real_T *c5_phiq;
          real_T *c5_hr;
          real_T *c5_htheta;
          real_T *c5_hpsi;
          real_T *c5_hy;
          real_T *c5_hz;
          c5_hz = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c5_hy = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c5_hpsi = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c5_htheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c5_hr = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c5_phiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c5_psiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_hx = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c5_thetaq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_thetaq);
          _SFD_SET_DATA_VALUE_PTR(1U, c5_hx);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_psiq);
          _SFD_SET_DATA_VALUE_PTR(3U, c5_phiq);
          _SFD_SET_DATA_VALUE_PTR(4U, c5_hr);
          _SFD_SET_DATA_VALUE_PTR(5U, c5_htheta);
          _SFD_SET_DATA_VALUE_PTR(6U, c5_hpsi);
          _SFD_SET_DATA_VALUE_PTR(7U, c5_hy);
          _SFD_SET_DATA_VALUE_PTR(8U, c5_hz);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _AutoFollow_SimulationMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "rqghAeRqUFNXi1Hhi13ACE";
}

static void sf_opaque_initialize_c5_AutoFollow_Simulation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_AutoFollow_Simulation
    ((SFc5_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
  initialize_c5_AutoFollow_Simulation((SFc5_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_AutoFollow_Simulation(void *chartInstanceVar)
{
  enable_c5_AutoFollow_Simulation((SFc5_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_AutoFollow_Simulation(void *chartInstanceVar)
{
  disable_c5_AutoFollow_Simulation((SFc5_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_AutoFollow_Simulation(void *chartInstanceVar)
{
  sf_gateway_c5_AutoFollow_Simulation((SFc5_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_AutoFollow_Simulation
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_AutoFollow_Simulation
    ((SFc5_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_AutoFollow_Simulation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c5_AutoFollow_Simulation(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c5_AutoFollow_Simulation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_AutoFollow_Simulation
    ((SFc5_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_AutoFollow_Simulation(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c5_AutoFollow_Simulation(S);
}

static void sf_opaque_set_sim_state_c5_AutoFollow_Simulation(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c5_AutoFollow_Simulation(S, st);
}

static void sf_opaque_terminate_c5_AutoFollow_Simulation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_AutoFollow_SimulationInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_AutoFollow_Simulation_optimization_info();
    }

    finalize_c5_AutoFollow_Simulation((SFc5_AutoFollow_SimulationInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_AutoFollow_Simulation
    ((SFc5_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_AutoFollow_Simulation(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c5_AutoFollow_Simulation
      ((SFc5_AutoFollow_SimulationInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_AutoFollow_Simulation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,5,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2083285972U));
  ssSetChecksum1(S,(646007995U));
  ssSetChecksum2(S,(2312391282U));
  ssSetChecksum3(S,(132072014U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_AutoFollow_Simulation(SimStruct *S)
{
  SFc5_AutoFollow_SimulationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc5_AutoFollow_SimulationInstanceStruct *)utMalloc(sizeof
    (SFc5_AutoFollow_SimulationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_AutoFollow_SimulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_AutoFollow_Simulation;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c5_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_AutoFollow_Simulation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_AutoFollow_Simulation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
