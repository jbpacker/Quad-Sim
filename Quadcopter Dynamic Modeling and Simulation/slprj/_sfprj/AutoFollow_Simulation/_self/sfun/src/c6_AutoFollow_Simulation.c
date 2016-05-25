/* Include files */

#include <stddef.h>
#include "blas.h"
#include "AutoFollow_Simulation_sfun.h"
#include "c6_AutoFollow_Simulation.h"
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
static const char * c6_debug_family_names[37] = { "dt", "A", "cart", "x", "y",
  "z", "r", "theta", "psi", "dr_dx", "dr_dy", "dr_dz", "dtheta_dx", "dtheta_dy",
  "dtheta_dz", "dpsi_dx", "dpsi_dy", "dpsi_dz", "C", "meas", "K", "nargin",
  "nargout", "thetaq", "phiq", "psiq", "hr", "htheta", "hpsi", "t", "xhatOut",
  "prevT", "prevMeas", "P", "xhat", "Q", "R" };

/* Function Declarations */
static void initialize_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initialize_params_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void enable_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void disable_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void set_sim_state_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c6_st);
static void finalize_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void sf_gateway_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c6_chartstep_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initSimStructsc6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static void c6_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_R, const char_T *c6_identifier, real_T
  c6_y[9]);
static void c6_b_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_Q, const char_T *c6_identifier, real_T
  c6_y[36]);
static void c6_d_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[36]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_e_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_xhat, const char_T *c6_identifier, real_T
  c6_y[6]);
static void c6_f_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_g_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_P, const char_T *c6_identifier, real_T
  c6_y[36]);
static void c6_h_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[36]);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_i_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_prevMeas, const char_T *c6_identifier,
  real_T c6_y[3]);
static void c6_j_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_k_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_prevT, const char_T *c6_identifier);
static real_T c6_l_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_m_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_xhatOut, const char_T *c6_identifier, real_T
  c6_y[6]);
static void c6_n_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6]);
static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_o_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_p_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[18]);
static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_q_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_k_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_r_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[18]);
static void c6_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_l_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_s_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[36]);
static void c6_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(const mxArray **c6_info);
static const mxArray *c6_emlrt_marshallOut(const char * c6_u);
static const mxArray *c6_b_emlrt_marshallOut(const uint32_T c6_u);
static void c6_b_info_helper(const mxArray **c6_info);
static void c6_diag(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                    real_T c6_v[6], real_T c6_d[36]);
static real_T c6_mpower(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c6_a);
static void c6_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_b_diag(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c6_v[3], real_T c6_d[9]);
static void c6_b_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_threshold(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static real_T c6_sqrt(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c6_x);
static void c6_eml_error(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance);
static real_T c6_atan2(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c6_y, real_T c6_x);
static real_T c6_b_mpower(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c6_a);
static void c6_c_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_d_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_e_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_f_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_g_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_mrdivide(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c6_A[18], real_T c6_B[9], real_T c6_y[18]);
static void c6_eml_lusolve(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c6_A[9], real_T c6_B[18], real_T c6_X[18]);
static void c6_eml_warning(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_h_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_i_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c6_eye(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c6_I[36]);
static void c6_j_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static const mxArray *c6_m_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_t_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_u_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_AutoFollow_Simulation, const
  char_T *c6_identifier);
static uint8_T c6_v_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_sqrt(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T *c6_x);
static void init_dsm_address_info(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c6_prevT_not_empty = false;
  chartInstance->c6_prevMeas_not_empty = false;
  chartInstance->c6_P_not_empty = false;
  chartInstance->c6_xhat_not_empty = false;
  chartInstance->c6_Q_not_empty = false;
  chartInstance->c6_R_not_empty = false;
  chartInstance->c6_is_active_c6_AutoFollow_Simulation = 0U;
}

static void initialize_params_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c6_update_debugger_state_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i0;
  real_T c6_u[6];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i1;
  real_T c6_b_u[36];
  const mxArray *c6_c_y = NULL;
  int32_T c6_i2;
  real_T c6_c_u[36];
  const mxArray *c6_d_y = NULL;
  int32_T c6_i3;
  real_T c6_d_u[9];
  const mxArray *c6_e_y = NULL;
  int32_T c6_i4;
  real_T c6_e_u[3];
  const mxArray *c6_f_y = NULL;
  real_T c6_hoistedGlobal;
  real_T c6_f_u;
  const mxArray *c6_g_y = NULL;
  int32_T c6_i5;
  real_T c6_g_u[6];
  const mxArray *c6_h_y = NULL;
  uint8_T c6_b_hoistedGlobal;
  uint8_T c6_h_u;
  const mxArray *c6_i_y = NULL;
  real_T (*c6_xhatOut)[6];
  c6_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellmatrix(8, 1), false);
  for (c6_i0 = 0; c6_i0 < 6; c6_i0++) {
    c6_u[c6_i0] = (*c6_xhatOut)[c6_i0];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  for (c6_i1 = 0; c6_i1 < 36; c6_i1++) {
    c6_b_u[c6_i1] = chartInstance->c6_P[c6_i1];
  }

  c6_c_y = NULL;
  if (!chartInstance->c6_P_not_empty) {
    sf_mex_assign(&c6_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_b_u, 0, 0U, 1U, 0U, 2, 6, 6),
                  false);
  }

  sf_mex_setcell(c6_y, 1, c6_c_y);
  for (c6_i2 = 0; c6_i2 < 36; c6_i2++) {
    c6_c_u[c6_i2] = chartInstance->c6_Q[c6_i2];
  }

  c6_d_y = NULL;
  if (!chartInstance->c6_Q_not_empty) {
    sf_mex_assign(&c6_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_c_u, 0, 0U, 1U, 0U, 2, 6, 6),
                  false);
  }

  sf_mex_setcell(c6_y, 2, c6_d_y);
  for (c6_i3 = 0; c6_i3 < 9; c6_i3++) {
    c6_d_u[c6_i3] = chartInstance->c6_R[c6_i3];
  }

  c6_e_y = NULL;
  if (!chartInstance->c6_R_not_empty) {
    sf_mex_assign(&c6_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c6_e_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 2, 3, 3),
                  false);
  }

  sf_mex_setcell(c6_y, 3, c6_e_y);
  for (c6_i4 = 0; c6_i4 < 3; c6_i4++) {
    c6_e_u[c6_i4] = chartInstance->c6_prevMeas[c6_i4];
  }

  c6_f_y = NULL;
  if (!chartInstance->c6_prevMeas_not_empty) {
    sf_mex_assign(&c6_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c6_f_y, sf_mex_create("y", c6_e_u, 0, 0U, 1U, 0U, 1, 3),
                  false);
  }

  sf_mex_setcell(c6_y, 4, c6_f_y);
  c6_hoistedGlobal = chartInstance->c6_prevT;
  c6_f_u = c6_hoistedGlobal;
  c6_g_y = NULL;
  if (!chartInstance->c6_prevT_not_empty) {
    sf_mex_assign(&c6_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c6_g_y, sf_mex_create("y", &c6_f_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c6_y, 5, c6_g_y);
  for (c6_i5 = 0; c6_i5 < 6; c6_i5++) {
    c6_g_u[c6_i5] = chartInstance->c6_xhat[c6_i5];
  }

  c6_h_y = NULL;
  if (!chartInstance->c6_xhat_not_empty) {
    sf_mex_assign(&c6_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c6_h_y, sf_mex_create("y", c6_g_u, 0, 0U, 1U, 0U, 1, 6),
                  false);
  }

  sf_mex_setcell(c6_y, 6, c6_h_y);
  c6_b_hoistedGlobal = chartInstance->c6_is_active_c6_AutoFollow_Simulation;
  c6_h_u = c6_b_hoistedGlobal;
  c6_i_y = NULL;
  sf_mex_assign(&c6_i_y, sf_mex_create("y", &c6_h_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c6_y, 7, c6_i_y);
  sf_mex_assign(&c6_st, c6_y, false);
  return c6_st;
}

static void set_sim_state_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv0[6];
  int32_T c6_i6;
  real_T c6_dv1[36];
  int32_T c6_i7;
  real_T c6_dv2[36];
  int32_T c6_i8;
  real_T c6_dv3[9];
  int32_T c6_i9;
  real_T c6_dv4[3];
  int32_T c6_i10;
  real_T c6_dv5[6];
  int32_T c6_i11;
  real_T (*c6_xhatOut)[6];
  c6_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = true;
  c6_u = sf_mex_dup(c6_st);
  c6_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
                        "xhatOut", c6_dv0);
  for (c6_i6 = 0; c6_i6 < 6; c6_i6++) {
    (*c6_xhatOut)[c6_i6] = c6_dv0[c6_i6];
  }

  c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)), "P",
                        c6_dv1);
  for (c6_i7 = 0; c6_i7 < 36; c6_i7++) {
    chartInstance->c6_P[c6_i7] = c6_dv1[c6_i7];
  }

  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 2)), "Q",
                        c6_dv2);
  for (c6_i8 = 0; c6_i8 < 36; c6_i8++) {
    chartInstance->c6_Q[c6_i8] = c6_dv2[c6_i8];
  }

  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 3)), "R",
                      c6_dv3);
  for (c6_i9 = 0; c6_i9 < 9; c6_i9++) {
    chartInstance->c6_R[c6_i9] = c6_dv3[c6_i9];
  }

  c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 4)),
                        "prevMeas", c6_dv4);
  for (c6_i10 = 0; c6_i10 < 3; c6_i10++) {
    chartInstance->c6_prevMeas[c6_i10] = c6_dv4[c6_i10];
  }

  chartInstance->c6_prevT = c6_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c6_u, 5)), "prevT");
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 6)),
                        "xhat", c6_dv5);
  for (c6_i11 = 0; c6_i11 < 6; c6_i11++) {
    chartInstance->c6_xhat[c6_i11] = c6_dv5[c6_i11];
  }

  chartInstance->c6_is_active_c6_AutoFollow_Simulation = c6_u_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 7)),
     "is_active_c6_AutoFollow_Simulation");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_AutoFollow_Simulation(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  int32_T c6_i12;
  real_T *c6_thetaq;
  real_T *c6_phiq;
  real_T *c6_psiq;
  real_T *c6_hr;
  real_T *c6_htheta;
  real_T *c6_hpsi;
  real_T *c6_t;
  real_T (*c6_xhatOut)[6];
  c6_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c6_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_hpsi = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c6_htheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c6_hr = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_psiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c6_phiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c6_thetaq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c6_thetaq, 0U);
  _SFD_DATA_RANGE_CHECK(*c6_phiq, 1U);
  _SFD_DATA_RANGE_CHECK(*c6_psiq, 2U);
  _SFD_DATA_RANGE_CHECK(*c6_hr, 3U);
  _SFD_DATA_RANGE_CHECK(*c6_htheta, 4U);
  _SFD_DATA_RANGE_CHECK(*c6_hpsi, 5U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  c6_chartstep_c6_AutoFollow_Simulation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_AutoFollow_SimulationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c6_i12 = 0; c6_i12 < 6; c6_i12++) {
    _SFD_DATA_RANGE_CHECK((*c6_xhatOut)[c6_i12], 6U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_t, 7U);
}

static void c6_chartstep_c6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  real_T c6_hoistedGlobal;
  real_T c6_b_hoistedGlobal;
  real_T c6_c_hoistedGlobal;
  real_T c6_d_hoistedGlobal;
  real_T c6_e_hoistedGlobal;
  real_T c6_f_hoistedGlobal;
  real_T c6_g_hoistedGlobal;
  real_T c6_thetaq;
  real_T c6_phiq;
  real_T c6_psiq;
  real_T c6_hr;
  real_T c6_htheta;
  real_T c6_hpsi;
  real_T c6_t;
  uint32_T c6_debug_family_var_map[37];
  real_T c6_dt;
  real_T c6_A[36];
  real_T c6_cart[3];
  real_T c6_x;
  real_T c6_y;
  real_T c6_z;
  real_T c6_r;
  real_T c6_theta;
  real_T c6_psi;
  real_T c6_dr_dx;
  real_T c6_dr_dy;
  real_T c6_dr_dz;
  real_T c6_dtheta_dx;
  real_T c6_dtheta_dy;
  real_T c6_dtheta_dz;
  real_T c6_dpsi_dx;
  real_T c6_dpsi_dy;
  real_T c6_dpsi_dz;
  real_T c6_C[18];
  real_T c6_meas[3];
  real_T c6_K[18];
  real_T c6_nargin = 7.0;
  real_T c6_nargout = 1.0;
  real_T c6_xhatOut[6];
  int32_T c6_i13;
  static real_T c6_dv6[6] = { 0.0, 0.0, -10.0, 0.0, 0.0, 0.0 };

  int32_T c6_i14;
  int32_T c6_i15;
  static real_T c6_dv7[6] = { 0.0, 0.0, 0.0, 0.1, 0.1, 0.1 };

  real_T c6_dv8[6];
  real_T c6_dv9[36];
  int32_T c6_i16;
  int32_T c6_i17;
  static real_T c6_dv10[3] = { 25.0, 25.0, 2500.0 };

  real_T c6_dv11[3];
  real_T c6_dv12[9];
  int32_T c6_i18;
  int32_T c6_i19;
  int32_T c6_i20;
  int32_T c6_i21;
  static real_T c6_dv13[6] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 };

  int32_T c6_i22;
  int32_T c6_i23;
  static real_T c6_dv14[6] = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 };

  int32_T c6_i24;
  int32_T c6_i25;
  static real_T c6_dv15[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_j_x;
  real_T c6_k_x;
  real_T c6_l_x;
  real_T c6_m_x;
  real_T c6_n_x;
  real_T c6_o_x;
  real_T c6_p_x;
  real_T c6_q_x;
  real_T c6_r_x;
  real_T c6_s_x;
  real_T c6_t_x;
  real_T c6_u_x;
  real_T c6_v_x;
  real_T c6_w_x;
  real_T c6_x_x;
  real_T c6_y_x;
  real_T c6_ab_x;
  real_T c6_bb_x;
  real_T c6_cb_x;
  real_T c6_db_x;
  real_T c6_eb_x;
  real_T c6_fb_x;
  real_T c6_gb_x;
  real_T c6_hb_x;
  real_T c6_ib_x;
  real_T c6_jb_x;
  real_T c6_kb_x;
  real_T c6_lb_x;
  real_T c6_mb_x;
  real_T c6_nb_x;
  real_T c6_ob_x;
  real_T c6_pb_x;
  real_T c6_qb_x;
  real_T c6_rb_x;
  real_T c6_sb_x;
  real_T c6_tb_x;
  real_T c6_ub_x;
  real_T c6_vb_x;
  real_T c6_wb_x;
  real_T c6_xb_x;
  real_T c6_yb_x;
  real_T c6_ac_x;
  real_T c6_bc_x;
  real_T c6_cc_x;
  real_T c6_dc_x;
  real_T c6_ec_x;
  real_T c6_fc_x;
  real_T c6_gc_x;
  real_T c6_hc_x;
  real_T c6_ic_x;
  real_T c6_jc_x;
  real_T c6_kc_x;
  real_T c6_lc_x;
  real_T c6_mc_x;
  real_T c6_nc_x;
  real_T c6_oc_x;
  real_T c6_pc_x;
  real_T c6_qc_x;
  real_T c6_rc_x;
  real_T c6_sc_x;
  int32_T c6_i26;
  real_T c6_h_hoistedGlobal[9];
  int32_T c6_i27;
  real_T c6_b[3];
  int32_T c6_i28;
  int32_T c6_i29;
  int32_T c6_i30;
  real_T c6_b_C[3];
  int32_T c6_i31;
  int32_T c6_i32;
  int32_T c6_i33;
  int32_T c6_i34;
  int32_T c6_i35;
  int32_T c6_i36;
  real_T c6_d0;
  real_T c6_b_A;
  real_T c6_B;
  real_T c6_tc_x;
  real_T c6_b_y;
  real_T c6_uc_x;
  real_T c6_c_y;
  real_T c6_vc_x;
  real_T c6_d_y;
  real_T c6_c_A;
  real_T c6_b_B;
  real_T c6_wc_x;
  real_T c6_e_y;
  real_T c6_xc_x;
  real_T c6_f_y;
  real_T c6_yc_x;
  real_T c6_g_y;
  real_T c6_d_A;
  real_T c6_c_B;
  real_T c6_ad_x;
  real_T c6_h_y;
  real_T c6_bd_x;
  real_T c6_i_y;
  real_T c6_cd_x;
  real_T c6_j_y;
  real_T c6_dd_x;
  real_T c6_ed_x;
  real_T c6_e_A;
  real_T c6_d_B;
  real_T c6_fd_x;
  real_T c6_k_y;
  real_T c6_gd_x;
  real_T c6_l_y;
  real_T c6_hd_x;
  real_T c6_m_y;
  real_T c6_id_x;
  real_T c6_jd_x;
  real_T c6_f_A;
  real_T c6_e_B;
  real_T c6_kd_x;
  real_T c6_n_y;
  real_T c6_ld_x;
  real_T c6_o_y;
  real_T c6_md_x;
  real_T c6_p_y;
  real_T c6_nd_x;
  real_T c6_od_x;
  real_T c6_g_A;
  real_T c6_f_B;
  real_T c6_pd_x;
  real_T c6_q_y;
  real_T c6_qd_x;
  real_T c6_r_y;
  real_T c6_rd_x;
  real_T c6_s_y;
  real_T c6_sd_x;
  real_T c6_td_x;
  real_T c6_h_A;
  real_T c6_g_B;
  real_T c6_ud_x;
  real_T c6_t_y;
  real_T c6_vd_x;
  real_T c6_u_y;
  real_T c6_wd_x;
  real_T c6_v_y;
  real_T c6_xd_x;
  real_T c6_yd_x;
  real_T c6_i_A;
  real_T c6_h_B;
  real_T c6_ae_x;
  real_T c6_w_y;
  real_T c6_be_x;
  real_T c6_x_y;
  real_T c6_ce_x;
  real_T c6_y_y;
  int32_T c6_i37;
  real_T c6_i_hoistedGlobal[6];
  int32_T c6_i38;
  real_T c6_a[36];
  int32_T c6_i39;
  real_T c6_ab_y[6];
  int32_T c6_i40;
  int32_T c6_i41;
  int32_T c6_i42;
  int32_T c6_i43;
  real_T c6_j_hoistedGlobal[36];
  int32_T c6_i44;
  int32_T c6_i45;
  int32_T c6_i46;
  int32_T c6_i47;
  real_T c6_bb_y[36];
  int32_T c6_i48;
  int32_T c6_i49;
  int32_T c6_i50;
  int32_T c6_i51;
  int32_T c6_i52;
  int32_T c6_i53;
  int32_T c6_i54;
  int32_T c6_i55;
  int32_T c6_i56;
  int32_T c6_i57;
  int32_T c6_i58;
  int32_T c6_i59;
  int32_T c6_i60;
  int32_T c6_i61;
  int32_T c6_i62;
  int32_T c6_i63;
  int32_T c6_i64;
  real_T c6_b_b[18];
  int32_T c6_i65;
  int32_T c6_i66;
  int32_T c6_i67;
  real_T c6_cb_y[18];
  int32_T c6_i68;
  int32_T c6_i69;
  int32_T c6_i70;
  int32_T c6_i71;
  real_T c6_b_a[18];
  int32_T c6_i72;
  int32_T c6_i73;
  int32_T c6_i74;
  int32_T c6_i75;
  real_T c6_db_y[18];
  int32_T c6_i76;
  int32_T c6_i77;
  int32_T c6_i78;
  int32_T c6_i79;
  int32_T c6_i80;
  int32_T c6_i81;
  int32_T c6_i82;
  int32_T c6_i83;
  int32_T c6_i84;
  int32_T c6_i85;
  int32_T c6_i86;
  int32_T c6_i87;
  int32_T c6_i88;
  real_T c6_eb_y[18];
  int32_T c6_i89;
  real_T c6_k_hoistedGlobal[9];
  real_T c6_dv16[18];
  int32_T c6_i90;
  int32_T c6_i91;
  int32_T c6_i92;
  int32_T c6_i93;
  int32_T c6_i94;
  int32_T c6_i95;
  int32_T c6_i96;
  int32_T c6_i97;
  int32_T c6_i98;
  int32_T c6_i99;
  int32_T c6_i100;
  int32_T c6_i101;
  int32_T c6_i102;
  int32_T c6_i103;
  int32_T c6_i104;
  int32_T c6_i105;
  int32_T c6_i106;
  int32_T c6_i107;
  int32_T c6_i108;
  int32_T c6_i109;
  int32_T c6_i110;
  int32_T c6_i111;
  int32_T c6_i112;
  int32_T c6_i113;
  int32_T c6_i114;
  int32_T c6_i115;
  int32_T c6_i116;
  int32_T c6_i117;
  int32_T c6_i118;
  int32_T c6_i119;
  int32_T c6_i120;
  int32_T c6_i121;
  real_T *c6_b_thetaq;
  real_T *c6_b_phiq;
  real_T *c6_b_psiq;
  real_T *c6_b_hr;
  real_T *c6_b_htheta;
  real_T *c6_b_hpsi;
  real_T *c6_b_t;
  real_T (*c6_b_xhatOut)[6];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  c6_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c6_b_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_hpsi = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c6_b_htheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c6_b_hr = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_b_psiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_phiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_thetaq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_thetaq;
  c6_b_hoistedGlobal = *c6_b_phiq;
  c6_c_hoistedGlobal = *c6_b_psiq;
  c6_d_hoistedGlobal = *c6_b_hr;
  c6_e_hoistedGlobal = *c6_b_htheta;
  c6_f_hoistedGlobal = *c6_b_hpsi;
  c6_g_hoistedGlobal = *c6_b_t;
  c6_thetaq = c6_hoistedGlobal;
  c6_phiq = c6_b_hoistedGlobal;
  c6_psiq = c6_c_hoistedGlobal;
  c6_hr = c6_d_hoistedGlobal;
  c6_htheta = c6_e_hoistedGlobal;
  c6_hpsi = c6_f_hoistedGlobal;
  c6_t = c6_g_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 37U, 37U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dt, 0U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_A, 1U, c6_l_sf_marshallOut,
    c6_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_cart, 2U, c6_j_sf_marshallOut,
    c6_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_x, 3U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_y, 4U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_z, 5U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_r, 6U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_theta, 7U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_psi, 8U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dr_dx, 9U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dr_dy, 10U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dr_dz, 11U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dtheta_dx, 12U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dtheta_dy, 13U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dtheta_dz, 14U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dpsi_dx, 15U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dpsi_dy, 16U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_dpsi_dz, 17U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_C, 18U, c6_k_sf_marshallOut,
    c6_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_meas, 19U, c6_j_sf_marshallOut,
    c6_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_K, 20U, c6_i_sf_marshallOut,
    c6_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 21U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 22U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_thetaq, 23U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_phiq, 24U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_psiq, 25U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_hr, 26U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_htheta, 27U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_hpsi, 28U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_t, 29U, c6_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_xhatOut, 30U, c6_g_sf_marshallOut,
    c6_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c6_prevT, 31U,
    c6_f_sf_marshallOut, c6_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c6_prevMeas, 32U,
    c6_e_sf_marshallOut, c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c6_P, 33U,
    c6_d_sf_marshallOut, c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c6_xhat, 34U,
    c6_c_sf_marshallOut, c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c6_Q, 35U,
    c6_b_sf_marshallOut, c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c6_R, 36U,
    c6_sf_marshallOut, c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 10);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 12);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c6_P_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 14);
    for (c6_i13 = 0; c6_i13 < 6; c6_i13++) {
      chartInstance->c6_xhat[c6_i13] = c6_dv6[c6_i13];
    }

    chartInstance->c6_xhat_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 17);
    for (c6_i14 = 0; c6_i14 < 36; c6_i14++) {
      chartInstance->c6_P[c6_i14] = 0.0;
    }

    chartInstance->c6_P_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 20);
    for (c6_i15 = 0; c6_i15 < 6; c6_i15++) {
      c6_dv8[c6_i15] = c6_dv7[c6_i15];
    }

    c6_diag(chartInstance, c6_dv8, c6_dv9);
    for (c6_i16 = 0; c6_i16 < 36; c6_i16++) {
      chartInstance->c6_Q[c6_i16] = c6_dv9[c6_i16];
    }

    chartInstance->c6_Q_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 23);
    for (c6_i17 = 0; c6_i17 < 3; c6_i17++) {
      c6_dv11[c6_i17] = c6_dv10[c6_i17];
    }

    c6_b_diag(chartInstance, c6_dv11, c6_dv12);
    for (c6_i18 = 0; c6_i18 < 9; c6_i18++) {
      chartInstance->c6_R[c6_i18] = c6_dv12[c6_i18];
    }

    chartInstance->c6_R_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 26);
    chartInstance->c6_prevT = 0.0;
    chartInstance->c6_prevT_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 29);
    for (c6_i19 = 0; c6_i19 < 3; c6_i19++) {
      chartInstance->c6_prevMeas[c6_i19] = 0.0;
    }

    chartInstance->c6_prevMeas_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 31);
  c6_dt = c6_t - chartInstance->c6_prevT;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 34);
  c6_A[0] = 1.0;
  c6_A[6] = 0.0;
  c6_A[12] = 0.0;
  c6_A[18] = c6_dt;
  c6_A[24] = 0.0;
  c6_A[30] = 0.0;
  c6_A[1] = 0.0;
  c6_A[7] = 1.0;
  c6_A[13] = 0.0;
  c6_A[19] = 0.0;
  c6_A[25] = c6_dt;
  c6_A[31] = 0.0;
  c6_A[2] = 0.0;
  c6_A[8] = 0.0;
  c6_A[14] = 1.0;
  c6_A[20] = 0.0;
  c6_A[26] = 0.0;
  c6_A[32] = c6_dt;
  c6_i20 = 0;
  for (c6_i21 = 0; c6_i21 < 6; c6_i21++) {
    c6_A[c6_i20 + 3] = c6_dv13[c6_i21];
    c6_i20 += 6;
  }

  c6_i22 = 0;
  for (c6_i23 = 0; c6_i23 < 6; c6_i23++) {
    c6_A[c6_i22 + 4] = c6_dv14[c6_i23];
    c6_i22 += 6;
  }

  c6_i24 = 0;
  for (c6_i25 = 0; c6_i25 < 6; c6_i25++) {
    c6_A[c6_i24 + 5] = c6_dv15[c6_i25];
    c6_i24 += 6;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 41);
  c6_b_x = c6_hpsi;
  c6_c_x = c6_b_x;
  c6_c_x = muDoubleScalarSin(c6_c_x);
  c6_d_x = c6_htheta;
  c6_e_x = c6_d_x;
  c6_e_x = muDoubleScalarCos(c6_e_x);
  c6_f_x = c6_hpsi;
  c6_g_x = c6_f_x;
  c6_g_x = muDoubleScalarSin(c6_g_x);
  c6_h_x = c6_htheta;
  c6_i_x = c6_h_x;
  c6_i_x = muDoubleScalarSin(c6_i_x);
  c6_j_x = c6_hpsi;
  c6_k_x = c6_j_x;
  c6_k_x = muDoubleScalarCos(c6_k_x);
  c6_cart[0] = c6_hr * c6_c_x * c6_e_x;
  c6_cart[1] = c6_hr * c6_g_x * c6_i_x;
  c6_cart[2] = c6_hr * c6_k_x;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 45);
  c6_l_x = c6_psiq;
  c6_m_x = c6_l_x;
  c6_m_x = muDoubleScalarCos(c6_m_x);
  c6_n_x = c6_thetaq;
  c6_o_x = c6_n_x;
  c6_o_x = muDoubleScalarCos(c6_o_x);
  c6_p_x = c6_psiq;
  c6_q_x = c6_p_x;
  c6_q_x = muDoubleScalarCos(c6_q_x);
  c6_r_x = c6_thetaq;
  c6_s_x = c6_r_x;
  c6_s_x = muDoubleScalarSin(c6_s_x);
  c6_t_x = c6_phiq;
  c6_u_x = c6_t_x;
  c6_u_x = muDoubleScalarSin(c6_u_x);
  c6_v_x = c6_psiq;
  c6_w_x = c6_v_x;
  c6_w_x = muDoubleScalarSin(c6_w_x);
  c6_x_x = c6_phiq;
  c6_y_x = c6_x_x;
  c6_y_x = muDoubleScalarCos(c6_y_x);
  c6_ab_x = c6_psiq;
  c6_bb_x = c6_ab_x;
  c6_bb_x = muDoubleScalarCos(c6_bb_x);
  c6_cb_x = c6_thetaq;
  c6_db_x = c6_cb_x;
  c6_db_x = muDoubleScalarSin(c6_db_x);
  c6_eb_x = c6_phiq;
  c6_fb_x = c6_eb_x;
  c6_fb_x = muDoubleScalarCos(c6_fb_x);
  c6_gb_x = c6_psiq;
  c6_hb_x = c6_gb_x;
  c6_hb_x = muDoubleScalarSin(c6_hb_x);
  c6_ib_x = c6_phiq;
  c6_jb_x = c6_ib_x;
  c6_jb_x = muDoubleScalarSin(c6_jb_x);
  c6_kb_x = c6_psiq;
  c6_lb_x = c6_kb_x;
  c6_lb_x = muDoubleScalarSin(c6_lb_x);
  c6_mb_x = c6_thetaq;
  c6_nb_x = c6_mb_x;
  c6_nb_x = muDoubleScalarCos(c6_nb_x);
  c6_ob_x = c6_psiq;
  c6_pb_x = c6_ob_x;
  c6_pb_x = muDoubleScalarSin(c6_pb_x);
  c6_qb_x = c6_thetaq;
  c6_rb_x = c6_qb_x;
  c6_rb_x = muDoubleScalarSin(c6_rb_x);
  c6_sb_x = c6_phiq;
  c6_tb_x = c6_sb_x;
  c6_tb_x = muDoubleScalarSin(c6_tb_x);
  c6_ub_x = c6_psiq;
  c6_vb_x = c6_ub_x;
  c6_vb_x = muDoubleScalarCos(c6_vb_x);
  c6_wb_x = c6_phiq;
  c6_xb_x = c6_wb_x;
  c6_xb_x = muDoubleScalarCos(c6_xb_x);
  c6_yb_x = c6_psiq;
  c6_ac_x = c6_yb_x;
  c6_ac_x = muDoubleScalarSin(c6_ac_x);
  c6_bc_x = c6_thetaq;
  c6_cc_x = c6_bc_x;
  c6_cc_x = muDoubleScalarSin(c6_cc_x);
  c6_dc_x = c6_phiq;
  c6_ec_x = c6_dc_x;
  c6_ec_x = muDoubleScalarCos(c6_ec_x);
  c6_fc_x = c6_psiq;
  c6_gc_x = c6_fc_x;
  c6_gc_x = muDoubleScalarCos(c6_gc_x);
  c6_hc_x = c6_phiq;
  c6_ic_x = c6_hc_x;
  c6_ic_x = muDoubleScalarSin(c6_ic_x);
  c6_jc_x = c6_thetaq;
  c6_kc_x = c6_jc_x;
  c6_kc_x = muDoubleScalarSin(c6_kc_x);
  c6_lc_x = c6_thetaq;
  c6_mc_x = c6_lc_x;
  c6_mc_x = muDoubleScalarCos(c6_mc_x);
  c6_nc_x = c6_phiq;
  c6_oc_x = c6_nc_x;
  c6_oc_x = muDoubleScalarSin(c6_oc_x);
  c6_pc_x = c6_thetaq;
  c6_qc_x = c6_pc_x;
  c6_qc_x = muDoubleScalarCos(c6_qc_x);
  c6_rc_x = c6_phiq;
  c6_sc_x = c6_rc_x;
  c6_sc_x = muDoubleScalarCos(c6_sc_x);
  chartInstance->c6_R[0] = c6_m_x * c6_o_x;
  chartInstance->c6_R[3] = c6_q_x * c6_s_x * c6_u_x - c6_w_x * c6_y_x;
  chartInstance->c6_R[6] = c6_bb_x * c6_db_x * c6_fb_x + c6_hb_x * c6_jb_x;
  chartInstance->c6_R[1] = c6_lb_x * c6_nb_x;
  chartInstance->c6_R[4] = c6_pb_x * c6_rb_x * c6_tb_x + c6_vb_x * c6_xb_x;
  chartInstance->c6_R[7] = c6_ac_x * c6_cc_x * c6_ec_x - c6_gc_x * c6_ic_x;
  chartInstance->c6_R[2] = -c6_kc_x;
  chartInstance->c6_R[5] = c6_mc_x * c6_oc_x;
  chartInstance->c6_R[8] = c6_qc_x * c6_sc_x;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 48);
  for (c6_i26 = 0; c6_i26 < 9; c6_i26++) {
    c6_h_hoistedGlobal[c6_i26] = chartInstance->c6_R[c6_i26];
  }

  for (c6_i27 = 0; c6_i27 < 3; c6_i27++) {
    c6_b[c6_i27] = c6_cart[c6_i27];
  }

  c6_b_eml_scalar_eg(chartInstance);
  c6_b_eml_scalar_eg(chartInstance);
  for (c6_i28 = 0; c6_i28 < 3; c6_i28++) {
    c6_cart[c6_i28] = 0.0;
  }

  for (c6_i29 = 0; c6_i29 < 3; c6_i29++) {
    c6_cart[c6_i29] = 0.0;
  }

  for (c6_i30 = 0; c6_i30 < 3; c6_i30++) {
    c6_b_C[c6_i30] = c6_cart[c6_i30];
  }

  for (c6_i31 = 0; c6_i31 < 3; c6_i31++) {
    c6_cart[c6_i31] = c6_b_C[c6_i31];
  }

  c6_threshold(chartInstance);
  for (c6_i32 = 0; c6_i32 < 3; c6_i32++) {
    c6_b_C[c6_i32] = c6_cart[c6_i32];
  }

  for (c6_i33 = 0; c6_i33 < 3; c6_i33++) {
    c6_cart[c6_i33] = c6_b_C[c6_i33];
  }

  for (c6_i34 = 0; c6_i34 < 3; c6_i34++) {
    c6_cart[c6_i34] = 0.0;
    c6_i35 = 0;
    for (c6_i36 = 0; c6_i36 < 3; c6_i36++) {
      c6_cart[c6_i34] += c6_h_hoistedGlobal[c6_i35 + c6_i34] * c6_b[c6_i36];
      c6_i35 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 49);
  c6_x = c6_cart[0];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 50);
  c6_y = c6_cart[1];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 51);
  c6_z = c6_cart[2];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 52);
  c6_r = (c6_mpower(chartInstance, c6_x) + c6_mpower(chartInstance, c6_y)) +
    c6_mpower(chartInstance, c6_z);
  c6_b_sqrt(chartInstance, &c6_r);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 53);
  c6_theta = c6_atan2(chartInstance, c6_y, c6_z);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 54);
  c6_d0 = c6_mpower(chartInstance, c6_x) + c6_mpower(chartInstance, c6_y);
  c6_b_sqrt(chartInstance, &c6_d0);
  c6_psi = c6_atan2(chartInstance, c6_d0, c6_z);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 56);
  c6_b_A = c6_x;
  c6_B = c6_r;
  c6_tc_x = c6_b_A;
  c6_b_y = c6_B;
  c6_uc_x = c6_tc_x;
  c6_c_y = c6_b_y;
  c6_vc_x = c6_uc_x;
  c6_d_y = c6_c_y;
  c6_dr_dx = c6_vc_x / c6_d_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 57);
  c6_c_A = c6_y;
  c6_b_B = c6_r;
  c6_wc_x = c6_c_A;
  c6_e_y = c6_b_B;
  c6_xc_x = c6_wc_x;
  c6_f_y = c6_e_y;
  c6_yc_x = c6_xc_x;
  c6_g_y = c6_f_y;
  c6_dr_dy = c6_yc_x / c6_g_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 58);
  c6_d_A = c6_z;
  c6_c_B = c6_r;
  c6_ad_x = c6_d_A;
  c6_h_y = c6_c_B;
  c6_bd_x = c6_ad_x;
  c6_i_y = c6_h_y;
  c6_cd_x = c6_bd_x;
  c6_j_y = c6_i_y;
  c6_dr_dz = c6_cd_x / c6_j_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 59);
  c6_dd_x = c6_theta;
  c6_ed_x = c6_dd_x;
  c6_ed_x = muDoubleScalarSin(c6_ed_x);
  c6_e_A = c6_x * c6_z;
  c6_d_B = c6_b_mpower(chartInstance, c6_r) * c6_ed_x;
  c6_fd_x = c6_e_A;
  c6_k_y = c6_d_B;
  c6_gd_x = c6_fd_x;
  c6_l_y = c6_k_y;
  c6_hd_x = c6_gd_x;
  c6_m_y = c6_l_y;
  c6_dtheta_dx = c6_hd_x / c6_m_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 60);
  c6_id_x = c6_theta;
  c6_jd_x = c6_id_x;
  c6_jd_x = muDoubleScalarSin(c6_jd_x);
  c6_f_A = c6_y * c6_z;
  c6_e_B = c6_b_mpower(chartInstance, c6_r) * c6_jd_x;
  c6_kd_x = c6_f_A;
  c6_n_y = c6_e_B;
  c6_ld_x = c6_kd_x;
  c6_o_y = c6_n_y;
  c6_md_x = c6_ld_x;
  c6_p_y = c6_o_y;
  c6_dtheta_dy = c6_md_x / c6_p_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 61);
  c6_nd_x = c6_theta;
  c6_od_x = c6_nd_x;
  c6_od_x = muDoubleScalarSin(c6_od_x);
  c6_g_A = -c6_od_x;
  c6_f_B = c6_r;
  c6_pd_x = c6_g_A;
  c6_q_y = c6_f_B;
  c6_qd_x = c6_pd_x;
  c6_r_y = c6_q_y;
  c6_rd_x = c6_qd_x;
  c6_s_y = c6_r_y;
  c6_dtheta_dz = c6_rd_x / c6_s_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 62);
  c6_sd_x = c6_theta;
  c6_td_x = c6_sd_x;
  c6_td_x = muDoubleScalarSin(c6_td_x);
  c6_h_A = -c6_y;
  c6_g_B = c6_mpower(chartInstance, c6_r) * c6_mpower(chartInstance, c6_td_x);
  c6_ud_x = c6_h_A;
  c6_t_y = c6_g_B;
  c6_vd_x = c6_ud_x;
  c6_u_y = c6_t_y;
  c6_wd_x = c6_vd_x;
  c6_v_y = c6_u_y;
  c6_dpsi_dx = c6_wd_x / c6_v_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 63);
  c6_xd_x = c6_theta;
  c6_yd_x = c6_xd_x;
  c6_yd_x = muDoubleScalarSin(c6_yd_x);
  c6_i_A = c6_x;
  c6_h_B = c6_mpower(chartInstance, c6_r) * c6_mpower(chartInstance, c6_yd_x);
  c6_ae_x = c6_i_A;
  c6_w_y = c6_h_B;
  c6_be_x = c6_ae_x;
  c6_x_y = c6_w_y;
  c6_ce_x = c6_be_x;
  c6_y_y = c6_x_y;
  c6_dpsi_dy = c6_ce_x / c6_y_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 64);
  c6_dpsi_dz = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 68);
  c6_C[0] = c6_dr_dx;
  c6_C[3] = c6_dr_dy;
  c6_C[6] = c6_dr_dz;
  c6_C[9] = 0.0;
  c6_C[12] = 0.0;
  c6_C[15] = 0.0;
  c6_C[1] = c6_dtheta_dx;
  c6_C[4] = c6_dtheta_dy;
  c6_C[7] = c6_dtheta_dz;
  c6_C[10] = 0.0;
  c6_C[13] = 0.0;
  c6_C[16] = 0.0;
  c6_C[2] = c6_dpsi_dx;
  c6_C[5] = c6_dpsi_dy;
  c6_C[8] = c6_dpsi_dz;
  c6_C[11] = 0.0;
  c6_C[14] = 0.0;
  c6_C[17] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 73);
  for (c6_i37 = 0; c6_i37 < 6; c6_i37++) {
    c6_i_hoistedGlobal[c6_i37] = chartInstance->c6_xhat[c6_i37];
  }

  for (c6_i38 = 0; c6_i38 < 36; c6_i38++) {
    c6_a[c6_i38] = c6_A[c6_i38];
  }

  c6_c_eml_scalar_eg(chartInstance);
  c6_c_eml_scalar_eg(chartInstance);
  c6_threshold(chartInstance);
  for (c6_i39 = 0; c6_i39 < 6; c6_i39++) {
    c6_ab_y[c6_i39] = 0.0;
    c6_i40 = 0;
    for (c6_i41 = 0; c6_i41 < 6; c6_i41++) {
      c6_ab_y[c6_i39] += c6_a[c6_i40 + c6_i39] * c6_i_hoistedGlobal[c6_i41];
      c6_i40 += 6;
    }
  }

  for (c6_i42 = 0; c6_i42 < 6; c6_i42++) {
    chartInstance->c6_xhat[c6_i42] = c6_ab_y[c6_i42];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 74);
  for (c6_i43 = 0; c6_i43 < 36; c6_i43++) {
    c6_j_hoistedGlobal[c6_i43] = chartInstance->c6_P[c6_i43];
  }

  for (c6_i44 = 0; c6_i44 < 36; c6_i44++) {
    c6_a[c6_i44] = c6_A[c6_i44];
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  c6_threshold(chartInstance);
  for (c6_i45 = 0; c6_i45 < 6; c6_i45++) {
    c6_i46 = 0;
    for (c6_i47 = 0; c6_i47 < 6; c6_i47++) {
      c6_bb_y[c6_i46 + c6_i45] = 0.0;
      c6_i48 = 0;
      for (c6_i49 = 0; c6_i49 < 6; c6_i49++) {
        c6_bb_y[c6_i46 + c6_i45] += c6_a[c6_i48 + c6_i45] *
          c6_j_hoistedGlobal[c6_i49 + c6_i46];
        c6_i48 += 6;
      }

      c6_i46 += 6;
    }
  }

  c6_i50 = 0;
  for (c6_i51 = 0; c6_i51 < 6; c6_i51++) {
    c6_i52 = 0;
    for (c6_i53 = 0; c6_i53 < 6; c6_i53++) {
      c6_a[c6_i53 + c6_i50] = c6_A[c6_i52 + c6_i51];
      c6_i52 += 6;
    }

    c6_i50 += 6;
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  c6_threshold(chartInstance);
  for (c6_i54 = 0; c6_i54 < 6; c6_i54++) {
    c6_i55 = 0;
    for (c6_i56 = 0; c6_i56 < 6; c6_i56++) {
      c6_j_hoistedGlobal[c6_i55 + c6_i54] = 0.0;
      c6_i57 = 0;
      for (c6_i58 = 0; c6_i58 < 6; c6_i58++) {
        c6_j_hoistedGlobal[c6_i55 + c6_i54] += c6_bb_y[c6_i57 + c6_i54] *
          c6_a[c6_i58 + c6_i55];
        c6_i57 += 6;
      }

      c6_i55 += 6;
    }
  }

  for (c6_i59 = 0; c6_i59 < 36; c6_i59++) {
    chartInstance->c6_P[c6_i59] = c6_j_hoistedGlobal[c6_i59] +
      chartInstance->c6_Q[c6_i59];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 77);
  c6_meas[0] = c6_r;
  c6_meas[1] = c6_theta;
  c6_meas[2] = c6_psi;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 79);
  guard1 = false;
  guard2 = false;
  if (CV_EML_COND(0, 1, 0, c6_meas[0] == chartInstance->c6_prevMeas[0])) {
    if (CV_EML_COND(0, 1, 1, c6_meas[1] == chartInstance->c6_prevMeas[1])) {
      if (CV_EML_COND(0, 1, 2, c6_meas[2] == chartInstance->c6_prevMeas[2])) {
        CV_EML_MCDC(0, 1, 0, true);
        CV_EML_IF(0, 1, 1, true);
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 80);
        for (c6_i60 = 0; c6_i60 < 36; c6_i60++) {
          c6_j_hoistedGlobal[c6_i60] = chartInstance->c6_P[c6_i60];
        }

        c6_i61 = 0;
        for (c6_i62 = 0; c6_i62 < 3; c6_i62++) {
          c6_i63 = 0;
          for (c6_i64 = 0; c6_i64 < 6; c6_i64++) {
            c6_b_b[c6_i64 + c6_i61] = c6_C[c6_i63 + c6_i62];
            c6_i63 += 3;
          }

          c6_i61 += 6;
        }

        c6_e_eml_scalar_eg(chartInstance);
        c6_e_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i65 = 0; c6_i65 < 6; c6_i65++) {
          c6_i66 = 0;
          for (c6_i67 = 0; c6_i67 < 3; c6_i67++) {
            c6_cb_y[c6_i66 + c6_i65] = 0.0;
            c6_i68 = 0;
            for (c6_i69 = 0; c6_i69 < 6; c6_i69++) {
              c6_cb_y[c6_i66 + c6_i65] += c6_j_hoistedGlobal[c6_i68 + c6_i65] *
                c6_b_b[c6_i69 + c6_i66];
              c6_i68 += 6;
            }

            c6_i66 += 6;
          }
        }

        for (c6_i70 = 0; c6_i70 < 36; c6_i70++) {
          c6_j_hoistedGlobal[c6_i70] = chartInstance->c6_P[c6_i70];
        }

        for (c6_i71 = 0; c6_i71 < 18; c6_i71++) {
          c6_b_a[c6_i71] = c6_C[c6_i71];
        }

        c6_f_eml_scalar_eg(chartInstance);
        c6_f_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i72 = 0; c6_i72 < 3; c6_i72++) {
          c6_i73 = 0;
          c6_i74 = 0;
          for (c6_i75 = 0; c6_i75 < 6; c6_i75++) {
            c6_db_y[c6_i73 + c6_i72] = 0.0;
            c6_i76 = 0;
            for (c6_i77 = 0; c6_i77 < 6; c6_i77++) {
              c6_db_y[c6_i73 + c6_i72] += c6_b_a[c6_i76 + c6_i72] *
                c6_j_hoistedGlobal[c6_i77 + c6_i74];
              c6_i76 += 3;
            }

            c6_i73 += 3;
            c6_i74 += 6;
          }
        }

        c6_i78 = 0;
        for (c6_i79 = 0; c6_i79 < 3; c6_i79++) {
          c6_i80 = 0;
          for (c6_i81 = 0; c6_i81 < 6; c6_i81++) {
            c6_b_b[c6_i81 + c6_i78] = c6_C[c6_i80 + c6_i79];
            c6_i80 += 3;
          }

          c6_i78 += 6;
        }

        c6_g_eml_scalar_eg(chartInstance);
        c6_g_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i82 = 0; c6_i82 < 3; c6_i82++) {
          c6_i83 = 0;
          c6_i84 = 0;
          for (c6_i85 = 0; c6_i85 < 3; c6_i85++) {
            c6_h_hoistedGlobal[c6_i83 + c6_i82] = 0.0;
            c6_i86 = 0;
            for (c6_i87 = 0; c6_i87 < 6; c6_i87++) {
              c6_h_hoistedGlobal[c6_i83 + c6_i82] += c6_db_y[c6_i86 + c6_i82] *
                c6_b_b[c6_i87 + c6_i84];
              c6_i86 += 3;
            }

            c6_i83 += 3;
            c6_i84 += 6;
          }
        }

        for (c6_i88 = 0; c6_i88 < 18; c6_i88++) {
          c6_eb_y[c6_i88] = c6_cb_y[c6_i88];
        }

        for (c6_i89 = 0; c6_i89 < 9; c6_i89++) {
          c6_k_hoistedGlobal[c6_i89] = c6_h_hoistedGlobal[c6_i89] +
            chartInstance->c6_R[c6_i89];
        }

        c6_mrdivide(chartInstance, c6_eb_y, c6_k_hoistedGlobal, c6_dv16);
        for (c6_i90 = 0; c6_i90 < 18; c6_i90++) {
          c6_K[c6_i90] = c6_dv16[c6_i90];
        }

        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 81);
        for (c6_i91 = 0; c6_i91 < 6; c6_i91++) {
          c6_i_hoistedGlobal[c6_i91] = chartInstance->c6_xhat[c6_i91];
        }

        for (c6_i92 = 0; c6_i92 < 6; c6_i92++) {
          c6_ab_y[c6_i92] = chartInstance->c6_xhat[c6_i92];
        }

        for (c6_i93 = 0; c6_i93 < 18; c6_i93++) {
          c6_b_a[c6_i93] = c6_C[c6_i93];
        }

        c6_h_eml_scalar_eg(chartInstance);
        c6_h_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i94 = 0; c6_i94 < 3; c6_i94++) {
          c6_b[c6_i94] = 0.0;
          c6_i95 = 0;
          for (c6_i96 = 0; c6_i96 < 6; c6_i96++) {
            c6_b[c6_i94] += c6_b_a[c6_i95 + c6_i94] * c6_ab_y[c6_i96];
            c6_i95 += 3;
          }
        }

        for (c6_i97 = 0; c6_i97 < 18; c6_i97++) {
          c6_b_b[c6_i97] = c6_K[c6_i97];
        }

        for (c6_i98 = 0; c6_i98 < 3; c6_i98++) {
          c6_b[c6_i98] = c6_meas[c6_i98] - c6_b[c6_i98];
        }

        c6_i_eml_scalar_eg(chartInstance);
        c6_i_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i99 = 0; c6_i99 < 6; c6_i99++) {
          c6_ab_y[c6_i99] = 0.0;
          c6_i100 = 0;
          for (c6_i101 = 0; c6_i101 < 3; c6_i101++) {
            c6_ab_y[c6_i99] += c6_b_b[c6_i100 + c6_i99] * c6_b[c6_i101];
            c6_i100 += 6;
          }
        }

        for (c6_i102 = 0; c6_i102 < 6; c6_i102++) {
          chartInstance->c6_xhat[c6_i102] = c6_i_hoistedGlobal[c6_i102] +
            c6_ab_y[c6_i102];
        }

        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 82);
        for (c6_i103 = 0; c6_i103 < 18; c6_i103++) {
          c6_b_b[c6_i103] = c6_K[c6_i103];
        }

        for (c6_i104 = 0; c6_i104 < 18; c6_i104++) {
          c6_b_a[c6_i104] = c6_C[c6_i104];
        }

        c6_j_eml_scalar_eg(chartInstance);
        c6_j_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i105 = 0; c6_i105 < 6; c6_i105++) {
          c6_i106 = 0;
          c6_i107 = 0;
          for (c6_i108 = 0; c6_i108 < 6; c6_i108++) {
            c6_bb_y[c6_i106 + c6_i105] = 0.0;
            c6_i109 = 0;
            for (c6_i110 = 0; c6_i110 < 3; c6_i110++) {
              c6_bb_y[c6_i106 + c6_i105] += c6_b_b[c6_i109 + c6_i105] *
                c6_b_a[c6_i110 + c6_i107];
              c6_i109 += 6;
            }

            c6_i106 += 6;
            c6_i107 += 3;
          }
        }

        for (c6_i111 = 0; c6_i111 < 36; c6_i111++) {
          c6_j_hoistedGlobal[c6_i111] = chartInstance->c6_P[c6_i111];
        }

        c6_eye(chartInstance, c6_a);
        for (c6_i112 = 0; c6_i112 < 36; c6_i112++) {
          c6_a[c6_i112] -= c6_bb_y[c6_i112];
        }

        c6_d_eml_scalar_eg(chartInstance);
        c6_d_eml_scalar_eg(chartInstance);
        c6_threshold(chartInstance);
        for (c6_i113 = 0; c6_i113 < 6; c6_i113++) {
          c6_i114 = 0;
          for (c6_i115 = 0; c6_i115 < 6; c6_i115++) {
            c6_bb_y[c6_i114 + c6_i113] = 0.0;
            c6_i116 = 0;
            for (c6_i117 = 0; c6_i117 < 6; c6_i117++) {
              c6_bb_y[c6_i114 + c6_i113] += c6_a[c6_i116 + c6_i113] *
                c6_j_hoistedGlobal[c6_i117 + c6_i114];
              c6_i116 += 6;
            }

            c6_i114 += 6;
          }
        }

        for (c6_i118 = 0; c6_i118 < 36; c6_i118++) {
          chartInstance->c6_P[c6_i118] = c6_bb_y[c6_i118];
        }
      } else {
        guard1 = true;
      }
    } else {
      guard2 = true;
    }
  } else {
    guard2 = true;
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 1, false);
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 84);
  for (c6_i119 = 0; c6_i119 < 3; c6_i119++) {
    chartInstance->c6_prevMeas[c6_i119] = c6_meas[c6_i119];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 87);
  for (c6_i120 = 0; c6_i120 < 6; c6_i120++) {
    c6_xhatOut[c6_i120] = chartInstance->c6_xhat[c6_i120];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -87);
  _SFD_SYMBOL_SCOPE_POP();
  for (c6_i121 = 0; c6_i121 < 6; c6_i121++) {
    (*c6_b_xhatOut)[c6_i121] = c6_xhatOut[c6_i121];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
}

static void initSimStructsc6_AutoFollow_Simulation
  (SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber)
{
  (void)c6_machineNumber;
  (void)c6_chartNumber;
  (void)c6_instanceNumber;
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i122;
  int32_T c6_i123;
  int32_T c6_i124;
  real_T c6_b_inData[9];
  int32_T c6_i125;
  int32_T c6_i126;
  int32_T c6_i127;
  real_T c6_u[9];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i122 = 0;
  for (c6_i123 = 0; c6_i123 < 3; c6_i123++) {
    for (c6_i124 = 0; c6_i124 < 3; c6_i124++) {
      c6_b_inData[c6_i124 + c6_i122] = (*(real_T (*)[9])c6_inData)[c6_i124 +
        c6_i122];
    }

    c6_i122 += 3;
  }

  c6_i125 = 0;
  for (c6_i126 = 0; c6_i126 < 3; c6_i126++) {
    for (c6_i127 = 0; c6_i127 < 3; c6_i127++) {
      c6_u[c6_i127 + c6_i125] = c6_b_inData[c6_i127 + c6_i125];
    }

    c6_i125 += 3;
  }

  c6_y = NULL;
  if (!chartInstance->c6_R_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_R, const char_T *c6_identifier, real_T
  c6_y[9])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_R), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_R);
}

static void c6_b_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9])
{
  real_T c6_dv17[9];
  int32_T c6_i128;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_R_not_empty = false;
  } else {
    chartInstance->c6_R_not_empty = true;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv17, 1, 0, 0U, 1, 0U, 2, 3,
                  3);
    for (c6_i128 = 0; c6_i128 < 9; c6_i128++) {
      c6_y[c6_i128] = c6_dv17[c6_i128];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_R;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[9];
  int32_T c6_i129;
  int32_T c6_i130;
  int32_T c6_i131;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_R = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_R), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_R);
  c6_i129 = 0;
  for (c6_i130 = 0; c6_i130 < 3; c6_i130++) {
    for (c6_i131 = 0; c6_i131 < 3; c6_i131++) {
      (*(real_T (*)[9])c6_outData)[c6_i131 + c6_i129] = c6_y[c6_i131 + c6_i129];
    }

    c6_i129 += 3;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i132;
  int32_T c6_i133;
  int32_T c6_i134;
  real_T c6_b_inData[36];
  int32_T c6_i135;
  int32_T c6_i136;
  int32_T c6_i137;
  real_T c6_u[36];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i132 = 0;
  for (c6_i133 = 0; c6_i133 < 6; c6_i133++) {
    for (c6_i134 = 0; c6_i134 < 6; c6_i134++) {
      c6_b_inData[c6_i134 + c6_i132] = (*(real_T (*)[36])c6_inData)[c6_i134 +
        c6_i132];
    }

    c6_i132 += 6;
  }

  c6_i135 = 0;
  for (c6_i136 = 0; c6_i136 < 6; c6_i136++) {
    for (c6_i137 = 0; c6_i137 < 6; c6_i137++) {
      c6_u[c6_i137 + c6_i135] = c6_b_inData[c6_i137 + c6_i135];
    }

    c6_i135 += 6;
  }

  c6_y = NULL;
  if (!chartInstance->c6_Q_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_Q, const char_T *c6_identifier, real_T
  c6_y[36])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_Q), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_Q);
}

static void c6_d_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[36])
{
  real_T c6_dv18[36];
  int32_T c6_i138;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_Q_not_empty = false;
  } else {
    chartInstance->c6_Q_not_empty = true;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv18, 1, 0, 0U, 1, 0U, 2, 6,
                  6);
    for (c6_i138 = 0; c6_i138 < 36; c6_i138++) {
      c6_y[c6_i138] = c6_dv18[c6_i138];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_Q;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[36];
  int32_T c6_i139;
  int32_T c6_i140;
  int32_T c6_i141;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_Q = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_Q), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_Q);
  c6_i139 = 0;
  for (c6_i140 = 0; c6_i140 < 6; c6_i140++) {
    for (c6_i141 = 0; c6_i141 < 6; c6_i141++) {
      (*(real_T (*)[36])c6_outData)[c6_i141 + c6_i139] = c6_y[c6_i141 + c6_i139];
    }

    c6_i139 += 6;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i142;
  real_T c6_b_inData[6];
  int32_T c6_i143;
  real_T c6_u[6];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i142 = 0; c6_i142 < 6; c6_i142++) {
    c6_b_inData[c6_i142] = (*(real_T (*)[6])c6_inData)[c6_i142];
  }

  for (c6_i143 = 0; c6_i143 < 6; c6_i143++) {
    c6_u[c6_i143] = c6_b_inData[c6_i143];
  }

  c6_y = NULL;
  if (!chartInstance->c6_xhat_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), false);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_e_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_xhat, const char_T *c6_identifier, real_T
  c6_y[6])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_xhat), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_xhat);
}

static void c6_f_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6])
{
  real_T c6_dv19[6];
  int32_T c6_i144;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_xhat_not_empty = false;
  } else {
    chartInstance->c6_xhat_not_empty = true;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv19, 1, 0, 0U, 1, 0U, 1, 6);
    for (c6_i144 = 0; c6_i144 < 6; c6_i144++) {
      c6_y[c6_i144] = c6_dv19[c6_i144];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_xhat;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[6];
  int32_T c6_i145;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_xhat = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_xhat), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_xhat);
  for (c6_i145 = 0; c6_i145 < 6; c6_i145++) {
    (*(real_T (*)[6])c6_outData)[c6_i145] = c6_y[c6_i145];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i146;
  int32_T c6_i147;
  int32_T c6_i148;
  real_T c6_b_inData[36];
  int32_T c6_i149;
  int32_T c6_i150;
  int32_T c6_i151;
  real_T c6_u[36];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i146 = 0;
  for (c6_i147 = 0; c6_i147 < 6; c6_i147++) {
    for (c6_i148 = 0; c6_i148 < 6; c6_i148++) {
      c6_b_inData[c6_i148 + c6_i146] = (*(real_T (*)[36])c6_inData)[c6_i148 +
        c6_i146];
    }

    c6_i146 += 6;
  }

  c6_i149 = 0;
  for (c6_i150 = 0; c6_i150 < 6; c6_i150++) {
    for (c6_i151 = 0; c6_i151 < 6; c6_i151++) {
      c6_u[c6_i151 + c6_i149] = c6_b_inData[c6_i151 + c6_i149];
    }

    c6_i149 += 6;
  }

  c6_y = NULL;
  if (!chartInstance->c6_P_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_g_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_P, const char_T *c6_identifier, real_T
  c6_y[36])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_P), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_P);
}

static void c6_h_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[36])
{
  real_T c6_dv20[36];
  int32_T c6_i152;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_P_not_empty = false;
  } else {
    chartInstance->c6_P_not_empty = true;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv20, 1, 0, 0U, 1, 0U, 2, 6,
                  6);
    for (c6_i152 = 0; c6_i152 < 36; c6_i152++) {
      c6_y[c6_i152] = c6_dv20[c6_i152];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_P;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[36];
  int32_T c6_i153;
  int32_T c6_i154;
  int32_T c6_i155;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_P = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_P), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_P);
  c6_i153 = 0;
  for (c6_i154 = 0; c6_i154 < 6; c6_i154++) {
    for (c6_i155 = 0; c6_i155 < 6; c6_i155++) {
      (*(real_T (*)[36])c6_outData)[c6_i155 + c6_i153] = c6_y[c6_i155 + c6_i153];
    }

    c6_i153 += 6;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i156;
  real_T c6_b_inData[3];
  int32_T c6_i157;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i156 = 0; c6_i156 < 3; c6_i156++) {
    c6_b_inData[c6_i156] = (*(real_T (*)[3])c6_inData)[c6_i156];
  }

  for (c6_i157 = 0; c6_i157 < 3; c6_i157++) {
    c6_u[c6_i157] = c6_b_inData[c6_i157];
  }

  c6_y = NULL;
  if (!chartInstance->c6_prevMeas_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), false);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_i_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_prevMeas, const char_T *c6_identifier,
  real_T c6_y[3])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_prevMeas), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_b_prevMeas);
}

static void c6_j_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv21[3];
  int32_T c6_i158;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_prevMeas_not_empty = false;
  } else {
    chartInstance->c6_prevMeas_not_empty = true;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv21, 1, 0, 0U, 1, 0U, 1, 3);
    for (c6_i158 = 0; c6_i158 < 3; c6_i158++) {
      c6_y[c6_i158] = c6_dv21[c6_i158];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_prevMeas;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i159;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_prevMeas = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_prevMeas), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_b_prevMeas);
  for (c6_i159 = 0; c6_i159 < 3; c6_i159++) {
    (*(real_T (*)[3])c6_outData)[c6_i159] = c6_y[c6_i159];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  if (!chartInstance->c6_prevT_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static real_T c6_k_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_prevT, const char_T *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_prevT), &c6_thisId);
  sf_mex_destroy(&c6_b_prevT);
  return c6_y;
}

static real_T c6_l_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d1;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_prevT_not_empty = false;
  } else {
    chartInstance->c6_prevT_not_empty = true;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d1, 1, 0, 0U, 0, 0U, 0);
    c6_y = c6_d1;
  }

  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_prevT;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_prevT = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_prevT), &c6_thisId);
  sf_mex_destroy(&c6_b_prevT);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i160;
  real_T c6_b_inData[6];
  int32_T c6_i161;
  real_T c6_u[6];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i160 = 0; c6_i160 < 6; c6_i160++) {
    c6_b_inData[c6_i160] = (*(real_T (*)[6])c6_inData)[c6_i160];
  }

  for (c6_i161 = 0; c6_i161 < 6; c6_i161++) {
    c6_u[c6_i161] = c6_b_inData[c6_i161];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_m_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_xhatOut, const char_T *c6_identifier, real_T
  c6_y[6])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_xhatOut), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_xhatOut);
}

static void c6_n_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6])
{
  real_T c6_dv22[6];
  int32_T c6_i162;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv22, 1, 0, 0U, 1, 0U, 1, 6);
  for (c6_i162 = 0; c6_i162 < 6; c6_i162++) {
    c6_y[c6_i162] = c6_dv22[c6_i162];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_xhatOut;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[6];
  int32_T c6_i163;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_xhatOut = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_xhatOut), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_xhatOut);
  for (c6_i163 = 0; c6_i163 < 6; c6_i163++) {
    (*(real_T (*)[6])c6_outData)[c6_i163] = c6_y[c6_i163];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static real_T c6_o_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d2;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d2, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d2;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i164;
  int32_T c6_i165;
  int32_T c6_i166;
  real_T c6_b_inData[18];
  int32_T c6_i167;
  int32_T c6_i168;
  int32_T c6_i169;
  real_T c6_u[18];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i164 = 0;
  for (c6_i165 = 0; c6_i165 < 3; c6_i165++) {
    for (c6_i166 = 0; c6_i166 < 6; c6_i166++) {
      c6_b_inData[c6_i166 + c6_i164] = (*(real_T (*)[18])c6_inData)[c6_i166 +
        c6_i164];
    }

    c6_i164 += 6;
  }

  c6_i167 = 0;
  for (c6_i168 = 0; c6_i168 < 3; c6_i168++) {
    for (c6_i169 = 0; c6_i169 < 6; c6_i169++) {
      c6_u[c6_i169 + c6_i167] = c6_b_inData[c6_i169 + c6_i167];
    }

    c6_i167 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 3), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_p_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[18])
{
  real_T c6_dv23[18];
  int32_T c6_i170;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv23, 1, 0, 0U, 1, 0U, 2, 6, 3);
  for (c6_i170 = 0; c6_i170 < 18; c6_i170++) {
    c6_y[c6_i170] = c6_dv23[c6_i170];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_K;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[18];
  int32_T c6_i171;
  int32_T c6_i172;
  int32_T c6_i173;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_K = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_K), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_K);
  c6_i171 = 0;
  for (c6_i172 = 0; c6_i172 < 3; c6_i172++) {
    for (c6_i173 = 0; c6_i173 < 6; c6_i173++) {
      (*(real_T (*)[18])c6_outData)[c6_i173 + c6_i171] = c6_y[c6_i173 + c6_i171];
    }

    c6_i171 += 6;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i174;
  real_T c6_b_inData[3];
  int32_T c6_i175;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i174 = 0; c6_i174 < 3; c6_i174++) {
    c6_b_inData[c6_i174] = (*(real_T (*)[3])c6_inData)[c6_i174];
  }

  for (c6_i175 = 0; c6_i175 < 3; c6_i175++) {
    c6_u[c6_i175] = c6_b_inData[c6_i175];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_q_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv24[3];
  int32_T c6_i176;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv24, 1, 0, 0U, 1, 0U, 1, 3);
  for (c6_i176 = 0; c6_i176 < 3; c6_i176++) {
    c6_y[c6_i176] = c6_dv24[c6_i176];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_meas;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i177;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_meas = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_meas), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_meas);
  for (c6_i177 = 0; c6_i177 < 3; c6_i177++) {
    (*(real_T (*)[3])c6_outData)[c6_i177] = c6_y[c6_i177];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_k_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i178;
  int32_T c6_i179;
  int32_T c6_i180;
  real_T c6_b_inData[18];
  int32_T c6_i181;
  int32_T c6_i182;
  int32_T c6_i183;
  real_T c6_u[18];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i178 = 0;
  for (c6_i179 = 0; c6_i179 < 6; c6_i179++) {
    for (c6_i180 = 0; c6_i180 < 3; c6_i180++) {
      c6_b_inData[c6_i180 + c6_i178] = (*(real_T (*)[18])c6_inData)[c6_i180 +
        c6_i178];
    }

    c6_i178 += 3;
  }

  c6_i181 = 0;
  for (c6_i182 = 0; c6_i182 < 6; c6_i182++) {
    for (c6_i183 = 0; c6_i183 < 3; c6_i183++) {
      c6_u[c6_i183 + c6_i181] = c6_b_inData[c6_i183 + c6_i181];
    }

    c6_i181 += 3;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 3, 6), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_r_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[18])
{
  real_T c6_dv25[18];
  int32_T c6_i184;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv25, 1, 0, 0U, 1, 0U, 2, 3, 6);
  for (c6_i184 = 0; c6_i184 < 18; c6_i184++) {
    c6_y[c6_i184] = c6_dv25[c6_i184];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_C;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[18];
  int32_T c6_i185;
  int32_T c6_i186;
  int32_T c6_i187;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_C = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_C), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_C);
  c6_i185 = 0;
  for (c6_i186 = 0; c6_i186 < 6; c6_i186++) {
    for (c6_i187 = 0; c6_i187 < 3; c6_i187++) {
      (*(real_T (*)[18])c6_outData)[c6_i187 + c6_i185] = c6_y[c6_i187 + c6_i185];
    }

    c6_i185 += 3;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_l_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i188;
  int32_T c6_i189;
  int32_T c6_i190;
  real_T c6_b_inData[36];
  int32_T c6_i191;
  int32_T c6_i192;
  int32_T c6_i193;
  real_T c6_u[36];
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i188 = 0;
  for (c6_i189 = 0; c6_i189 < 6; c6_i189++) {
    for (c6_i190 = 0; c6_i190 < 6; c6_i190++) {
      c6_b_inData[c6_i190 + c6_i188] = (*(real_T (*)[36])c6_inData)[c6_i190 +
        c6_i188];
    }

    c6_i188 += 6;
  }

  c6_i191 = 0;
  for (c6_i192 = 0; c6_i192 < 6; c6_i192++) {
    for (c6_i193 = 0; c6_i193 < 6; c6_i193++) {
      c6_u[c6_i193 + c6_i191] = c6_b_inData[c6_i193 + c6_i191];
    }

    c6_i191 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_s_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[36])
{
  real_T c6_dv26[36];
  int32_T c6_i194;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv26, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c6_i194 = 0; c6_i194 < 36; c6_i194++) {
    c6_y[c6_i194] = c6_dv26[c6_i194];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_A;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[36];
  int32_T c6_i195;
  int32_T c6_i196;
  int32_T c6_i197;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_A = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_A), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_A);
  c6_i195 = 0;
  for (c6_i196 = 0; c6_i196 < 6; c6_i196++) {
    for (c6_i197 = 0; c6_i197 < 6; c6_i197++) {
      (*(real_T (*)[36])c6_outData)[c6_i197 + c6_i195] = c6_y[c6_i197 + c6_i195];
    }

    c6_i195 += 6;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_AutoFollow_Simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  sf_mex_assign(&c6_nameCaptureInfo, sf_mex_createstruct("structure", 2, 77, 1),
                false);
  c6_info_helper(&c6_nameCaptureInfo);
  c6_b_info_helper(&c6_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs0 = NULL;
  const mxArray *c6_lhs0 = NULL;
  const mxArray *c6_rhs1 = NULL;
  const mxArray *c6_lhs1 = NULL;
  const mxArray *c6_rhs2 = NULL;
  const mxArray *c6_lhs2 = NULL;
  const mxArray *c6_rhs3 = NULL;
  const mxArray *c6_lhs3 = NULL;
  const mxArray *c6_rhs4 = NULL;
  const mxArray *c6_lhs4 = NULL;
  const mxArray *c6_rhs5 = NULL;
  const mxArray *c6_lhs5 = NULL;
  const mxArray *c6_rhs6 = NULL;
  const mxArray *c6_lhs6 = NULL;
  const mxArray *c6_rhs7 = NULL;
  const mxArray *c6_lhs7 = NULL;
  const mxArray *c6_rhs8 = NULL;
  const mxArray *c6_lhs8 = NULL;
  const mxArray *c6_rhs9 = NULL;
  const mxArray *c6_lhs9 = NULL;
  const mxArray *c6_rhs10 = NULL;
  const mxArray *c6_lhs10 = NULL;
  const mxArray *c6_rhs11 = NULL;
  const mxArray *c6_lhs11 = NULL;
  const mxArray *c6_rhs12 = NULL;
  const mxArray *c6_lhs12 = NULL;
  const mxArray *c6_rhs13 = NULL;
  const mxArray *c6_lhs13 = NULL;
  const mxArray *c6_rhs14 = NULL;
  const mxArray *c6_lhs14 = NULL;
  const mxArray *c6_rhs15 = NULL;
  const mxArray *c6_lhs15 = NULL;
  const mxArray *c6_rhs16 = NULL;
  const mxArray *c6_lhs16 = NULL;
  const mxArray *c6_rhs17 = NULL;
  const mxArray *c6_lhs17 = NULL;
  const mxArray *c6_rhs18 = NULL;
  const mxArray *c6_lhs18 = NULL;
  const mxArray *c6_rhs19 = NULL;
  const mxArray *c6_lhs19 = NULL;
  const mxArray *c6_rhs20 = NULL;
  const mxArray *c6_lhs20 = NULL;
  const mxArray *c6_rhs21 = NULL;
  const mxArray *c6_lhs21 = NULL;
  const mxArray *c6_rhs22 = NULL;
  const mxArray *c6_lhs22 = NULL;
  const mxArray *c6_rhs23 = NULL;
  const mxArray *c6_lhs23 = NULL;
  const mxArray *c6_rhs24 = NULL;
  const mxArray *c6_lhs24 = NULL;
  const mxArray *c6_rhs25 = NULL;
  const mxArray *c6_lhs25 = NULL;
  const mxArray *c6_rhs26 = NULL;
  const mxArray *c6_lhs26 = NULL;
  const mxArray *c6_rhs27 = NULL;
  const mxArray *c6_lhs27 = NULL;
  const mxArray *c6_rhs28 = NULL;
  const mxArray *c6_lhs28 = NULL;
  const mxArray *c6_rhs29 = NULL;
  const mxArray *c6_lhs29 = NULL;
  const mxArray *c6_rhs30 = NULL;
  const mxArray *c6_lhs30 = NULL;
  const mxArray *c6_rhs31 = NULL;
  const mxArray *c6_lhs31 = NULL;
  const mxArray *c6_rhs32 = NULL;
  const mxArray *c6_lhs32 = NULL;
  const mxArray *c6_rhs33 = NULL;
  const mxArray *c6_lhs33 = NULL;
  const mxArray *c6_rhs34 = NULL;
  const mxArray *c6_lhs34 = NULL;
  const mxArray *c6_rhs35 = NULL;
  const mxArray *c6_lhs35 = NULL;
  const mxArray *c6_rhs36 = NULL;
  const mxArray *c6_lhs36 = NULL;
  const mxArray *c6_rhs37 = NULL;
  const mxArray *c6_lhs37 = NULL;
  const mxArray *c6_rhs38 = NULL;
  const mxArray *c6_lhs38 = NULL;
  const mxArray *c6_rhs39 = NULL;
  const mxArray *c6_lhs39 = NULL;
  const mxArray *c6_rhs40 = NULL;
  const mxArray *c6_lhs40 = NULL;
  const mxArray *c6_rhs41 = NULL;
  const mxArray *c6_lhs41 = NULL;
  const mxArray *c6_rhs42 = NULL;
  const mxArray *c6_lhs42 = NULL;
  const mxArray *c6_rhs43 = NULL;
  const mxArray *c6_lhs43 = NULL;
  const mxArray *c6_rhs44 = NULL;
  const mxArray *c6_lhs44 = NULL;
  const mxArray *c6_rhs45 = NULL;
  const mxArray *c6_lhs45 = NULL;
  const mxArray *c6_rhs46 = NULL;
  const mxArray *c6_lhs46 = NULL;
  const mxArray *c6_rhs47 = NULL;
  const mxArray *c6_lhs47 = NULL;
  const mxArray *c6_rhs48 = NULL;
  const mxArray *c6_lhs48 = NULL;
  const mxArray *c6_rhs49 = NULL;
  const mxArray *c6_lhs49 = NULL;
  const mxArray *c6_rhs50 = NULL;
  const mxArray *c6_lhs50 = NULL;
  const mxArray *c6_rhs51 = NULL;
  const mxArray *c6_lhs51 = NULL;
  const mxArray *c6_rhs52 = NULL;
  const mxArray *c6_lhs52 = NULL;
  const mxArray *c6_rhs53 = NULL;
  const mxArray *c6_lhs53 = NULL;
  const mxArray *c6_rhs54 = NULL;
  const mxArray *c6_lhs54 = NULL;
  const mxArray *c6_rhs55 = NULL;
  const mxArray *c6_lhs55 = NULL;
  const mxArray *c6_rhs56 = NULL;
  const mxArray *c6_lhs56 = NULL;
  const mxArray *c6_rhs57 = NULL;
  const mxArray *c6_lhs57 = NULL;
  const mxArray *c6_rhs58 = NULL;
  const mxArray *c6_lhs58 = NULL;
  const mxArray *c6_rhs59 = NULL;
  const mxArray *c6_lhs59 = NULL;
  const mxArray *c6_rhs60 = NULL;
  const mxArray *c6_lhs60 = NULL;
  const mxArray *c6_rhs61 = NULL;
  const mxArray *c6_lhs61 = NULL;
  const mxArray *c6_rhs62 = NULL;
  const mxArray *c6_lhs62 = NULL;
  const mxArray *c6_rhs63 = NULL;
  const mxArray *c6_lhs63 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("diag"), "name", "name", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c6_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("ismatrix"), "name", "name", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c6_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c6_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c6_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c6_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c6_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmax"), "name", "name", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c6_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c6_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mpower"), "name", "name", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717478U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c6_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c6_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("ismatrix"), "name", "name", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c6_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("power"), "name", "name", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c6_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c6_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c6_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c6_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c6_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("floor"), "name", "name", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c6_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c6_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c6_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c6_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("sin"), "name", "name", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c6_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c6_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("cos"), "name", "name", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c6_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c6_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c6_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c6_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c6_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c6_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c6_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c6_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c6_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c6_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c6_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c6_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c6_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c6_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("sqrt"), "name", "name", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c6_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_error"), "name", "name",
                  37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837558U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c6_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825938U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c6_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("atan2"), "name", "name", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c6_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c6_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c6_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825920U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c6_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mrdivide"), "name", "name", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c6_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c6_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("rdivide"), "name", "name", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c6_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c6_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c6_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_div"), "name", "name", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c6_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c6_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("ismatrix"), "name", "name", 50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c6_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_lusolve"), "name", "name",
                  51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m"), "resolved",
                  "resolved", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1370017086U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c6_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xcabs1"), "name", "name",
                  52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c6_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "context", "context", 53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c6_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "context", "context", 54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717452U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c6_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c6_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825912U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c6_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("rdivide"), "name", "name", 57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c6_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular"),
                  "context", "context", 58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_warning"), "name", "name",
                  58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286826002U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c6_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c6_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c6_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eye"), "name", "name", 61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381857498U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c6_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1368190230U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c6_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 63);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 63);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c6_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c6_rhs0);
  sf_mex_destroy(&c6_lhs0);
  sf_mex_destroy(&c6_rhs1);
  sf_mex_destroy(&c6_lhs1);
  sf_mex_destroy(&c6_rhs2);
  sf_mex_destroy(&c6_lhs2);
  sf_mex_destroy(&c6_rhs3);
  sf_mex_destroy(&c6_lhs3);
  sf_mex_destroy(&c6_rhs4);
  sf_mex_destroy(&c6_lhs4);
  sf_mex_destroy(&c6_rhs5);
  sf_mex_destroy(&c6_lhs5);
  sf_mex_destroy(&c6_rhs6);
  sf_mex_destroy(&c6_lhs6);
  sf_mex_destroy(&c6_rhs7);
  sf_mex_destroy(&c6_lhs7);
  sf_mex_destroy(&c6_rhs8);
  sf_mex_destroy(&c6_lhs8);
  sf_mex_destroy(&c6_rhs9);
  sf_mex_destroy(&c6_lhs9);
  sf_mex_destroy(&c6_rhs10);
  sf_mex_destroy(&c6_lhs10);
  sf_mex_destroy(&c6_rhs11);
  sf_mex_destroy(&c6_lhs11);
  sf_mex_destroy(&c6_rhs12);
  sf_mex_destroy(&c6_lhs12);
  sf_mex_destroy(&c6_rhs13);
  sf_mex_destroy(&c6_lhs13);
  sf_mex_destroy(&c6_rhs14);
  sf_mex_destroy(&c6_lhs14);
  sf_mex_destroy(&c6_rhs15);
  sf_mex_destroy(&c6_lhs15);
  sf_mex_destroy(&c6_rhs16);
  sf_mex_destroy(&c6_lhs16);
  sf_mex_destroy(&c6_rhs17);
  sf_mex_destroy(&c6_lhs17);
  sf_mex_destroy(&c6_rhs18);
  sf_mex_destroy(&c6_lhs18);
  sf_mex_destroy(&c6_rhs19);
  sf_mex_destroy(&c6_lhs19);
  sf_mex_destroy(&c6_rhs20);
  sf_mex_destroy(&c6_lhs20);
  sf_mex_destroy(&c6_rhs21);
  sf_mex_destroy(&c6_lhs21);
  sf_mex_destroy(&c6_rhs22);
  sf_mex_destroy(&c6_lhs22);
  sf_mex_destroy(&c6_rhs23);
  sf_mex_destroy(&c6_lhs23);
  sf_mex_destroy(&c6_rhs24);
  sf_mex_destroy(&c6_lhs24);
  sf_mex_destroy(&c6_rhs25);
  sf_mex_destroy(&c6_lhs25);
  sf_mex_destroy(&c6_rhs26);
  sf_mex_destroy(&c6_lhs26);
  sf_mex_destroy(&c6_rhs27);
  sf_mex_destroy(&c6_lhs27);
  sf_mex_destroy(&c6_rhs28);
  sf_mex_destroy(&c6_lhs28);
  sf_mex_destroy(&c6_rhs29);
  sf_mex_destroy(&c6_lhs29);
  sf_mex_destroy(&c6_rhs30);
  sf_mex_destroy(&c6_lhs30);
  sf_mex_destroy(&c6_rhs31);
  sf_mex_destroy(&c6_lhs31);
  sf_mex_destroy(&c6_rhs32);
  sf_mex_destroy(&c6_lhs32);
  sf_mex_destroy(&c6_rhs33);
  sf_mex_destroy(&c6_lhs33);
  sf_mex_destroy(&c6_rhs34);
  sf_mex_destroy(&c6_lhs34);
  sf_mex_destroy(&c6_rhs35);
  sf_mex_destroy(&c6_lhs35);
  sf_mex_destroy(&c6_rhs36);
  sf_mex_destroy(&c6_lhs36);
  sf_mex_destroy(&c6_rhs37);
  sf_mex_destroy(&c6_lhs37);
  sf_mex_destroy(&c6_rhs38);
  sf_mex_destroy(&c6_lhs38);
  sf_mex_destroy(&c6_rhs39);
  sf_mex_destroy(&c6_lhs39);
  sf_mex_destroy(&c6_rhs40);
  sf_mex_destroy(&c6_lhs40);
  sf_mex_destroy(&c6_rhs41);
  sf_mex_destroy(&c6_lhs41);
  sf_mex_destroy(&c6_rhs42);
  sf_mex_destroy(&c6_lhs42);
  sf_mex_destroy(&c6_rhs43);
  sf_mex_destroy(&c6_lhs43);
  sf_mex_destroy(&c6_rhs44);
  sf_mex_destroy(&c6_lhs44);
  sf_mex_destroy(&c6_rhs45);
  sf_mex_destroy(&c6_lhs45);
  sf_mex_destroy(&c6_rhs46);
  sf_mex_destroy(&c6_lhs46);
  sf_mex_destroy(&c6_rhs47);
  sf_mex_destroy(&c6_lhs47);
  sf_mex_destroy(&c6_rhs48);
  sf_mex_destroy(&c6_lhs48);
  sf_mex_destroy(&c6_rhs49);
  sf_mex_destroy(&c6_lhs49);
  sf_mex_destroy(&c6_rhs50);
  sf_mex_destroy(&c6_lhs50);
  sf_mex_destroy(&c6_rhs51);
  sf_mex_destroy(&c6_lhs51);
  sf_mex_destroy(&c6_rhs52);
  sf_mex_destroy(&c6_lhs52);
  sf_mex_destroy(&c6_rhs53);
  sf_mex_destroy(&c6_lhs53);
  sf_mex_destroy(&c6_rhs54);
  sf_mex_destroy(&c6_lhs54);
  sf_mex_destroy(&c6_rhs55);
  sf_mex_destroy(&c6_lhs55);
  sf_mex_destroy(&c6_rhs56);
  sf_mex_destroy(&c6_lhs56);
  sf_mex_destroy(&c6_rhs57);
  sf_mex_destroy(&c6_lhs57);
  sf_mex_destroy(&c6_rhs58);
  sf_mex_destroy(&c6_lhs58);
  sf_mex_destroy(&c6_rhs59);
  sf_mex_destroy(&c6_lhs59);
  sf_mex_destroy(&c6_rhs60);
  sf_mex_destroy(&c6_lhs60);
  sf_mex_destroy(&c6_rhs61);
  sf_mex_destroy(&c6_lhs61);
  sf_mex_destroy(&c6_rhs62);
  sf_mex_destroy(&c6_lhs62);
  sf_mex_destroy(&c6_rhs63);
  sf_mex_destroy(&c6_lhs63);
}

static const mxArray *c6_emlrt_marshallOut(const char * c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c6_u)), false);
  return c6_y;
}

static const mxArray *c6_b_emlrt_marshallOut(const uint32_T c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 7, 0U, 0U, 0U, 0), false);
  return c6_y;
}

static void c6_b_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs64 = NULL;
  const mxArray *c6_lhs64 = NULL;
  const mxArray *c6_rhs65 = NULL;
  const mxArray *c6_lhs65 = NULL;
  const mxArray *c6_rhs66 = NULL;
  const mxArray *c6_lhs66 = NULL;
  const mxArray *c6_rhs67 = NULL;
  const mxArray *c6_lhs67 = NULL;
  const mxArray *c6_rhs68 = NULL;
  const mxArray *c6_lhs68 = NULL;
  const mxArray *c6_rhs69 = NULL;
  const mxArray *c6_lhs69 = NULL;
  const mxArray *c6_rhs70 = NULL;
  const mxArray *c6_lhs70 = NULL;
  const mxArray *c6_rhs71 = NULL;
  const mxArray *c6_lhs71 = NULL;
  const mxArray *c6_rhs72 = NULL;
  const mxArray *c6_lhs72 = NULL;
  const mxArray *c6_rhs73 = NULL;
  const mxArray *c6_lhs73 = NULL;
  const mxArray *c6_rhs74 = NULL;
  const mxArray *c6_lhs74 = NULL;
  const mxArray *c6_rhs75 = NULL;
  const mxArray *c6_lhs75 = NULL;
  const mxArray *c6_rhs76 = NULL;
  const mxArray *c6_lhs76 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isinf"), "name", "name", 64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c6_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c6_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286825982U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c6_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmax"), "name", "name", 67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c6_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmin"), "name", "name", 68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c6_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c6_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326731922U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c6_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c6_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c6_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmin"), "name", "name", 73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c6_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c6_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmax"), "name", "name", 75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c6_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c6_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs76), "lhs", "lhs",
                  76);
  sf_mex_destroy(&c6_rhs64);
  sf_mex_destroy(&c6_lhs64);
  sf_mex_destroy(&c6_rhs65);
  sf_mex_destroy(&c6_lhs65);
  sf_mex_destroy(&c6_rhs66);
  sf_mex_destroy(&c6_lhs66);
  sf_mex_destroy(&c6_rhs67);
  sf_mex_destroy(&c6_lhs67);
  sf_mex_destroy(&c6_rhs68);
  sf_mex_destroy(&c6_lhs68);
  sf_mex_destroy(&c6_rhs69);
  sf_mex_destroy(&c6_lhs69);
  sf_mex_destroy(&c6_rhs70);
  sf_mex_destroy(&c6_lhs70);
  sf_mex_destroy(&c6_rhs71);
  sf_mex_destroy(&c6_lhs71);
  sf_mex_destroy(&c6_rhs72);
  sf_mex_destroy(&c6_lhs72);
  sf_mex_destroy(&c6_rhs73);
  sf_mex_destroy(&c6_lhs73);
  sf_mex_destroy(&c6_rhs74);
  sf_mex_destroy(&c6_lhs74);
  sf_mex_destroy(&c6_rhs75);
  sf_mex_destroy(&c6_lhs75);
  sf_mex_destroy(&c6_rhs76);
  sf_mex_destroy(&c6_lhs76);
}

static void c6_diag(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                    real_T c6_v[6], real_T c6_d[36])
{
  int32_T c6_i198;
  int32_T c6_j;
  int32_T c6_b_j;
  (void)chartInstance;
  for (c6_i198 = 0; c6_i198 < 36; c6_i198++) {
    c6_d[c6_i198] = 0.0;
  }

  for (c6_j = 1; c6_j < 7; c6_j++) {
    c6_b_j = c6_j;
    c6_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_j), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 6, 2, 0) - 1)) -
      1] = c6_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_b_j), 1, 6, 1, 0) - 1];
  }
}

static real_T c6_mpower(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c6_a)
{
  real_T c6_b_a;
  real_T c6_c_a;
  real_T c6_ak;
  real_T c6_d_a;
  c6_b_a = c6_a;
  c6_c_a = c6_b_a;
  c6_eml_scalar_eg(chartInstance);
  c6_ak = c6_c_a;
  c6_d_a = c6_ak;
  c6_eml_scalar_eg(chartInstance);
  return c6_d_a * c6_d_a;
}

static void c6_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_b_diag(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c6_v[3], real_T c6_d[9])
{
  int32_T c6_i199;
  int32_T c6_j;
  int32_T c6_b_j;
  (void)chartInstance;
  for (c6_i199 = 0; c6_i199 < 9; c6_i199++) {
    c6_d[c6_i199] = 0.0;
  }

  for (c6_j = 1; c6_j < 4; c6_j++) {
    c6_b_j = c6_j;
    c6_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_j), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 3, 2, 0) - 1)) -
      1] = c6_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_b_j), 1, 3, 1, 0) - 1];
  }
}

static void c6_b_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_threshold(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c6_sqrt(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c6_x)
{
  real_T c6_b_x;
  c6_b_x = c6_x;
  c6_b_sqrt(chartInstance, &c6_b_x);
  return c6_b_x;
}

static void c6_eml_error(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  int32_T c6_i200;
  static char_T c6_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c6_u[30];
  const mxArray *c6_y = NULL;
  int32_T c6_i201;
  static char_T c6_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c6_b_u[4];
  const mxArray *c6_b_y = NULL;
  (void)chartInstance;
  for (c6_i200 = 0; c6_i200 < 30; c6_i200++) {
    c6_u[c6_i200] = c6_cv0[c6_i200];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c6_i201 = 0; c6_i201 < 4; c6_i201++) {
    c6_b_u[c6_i201] = c6_cv1[c6_i201];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c6_y, 14, c6_b_y));
}

static real_T c6_atan2(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c6_y, real_T c6_x)
{
  real_T c6_b_y;
  real_T c6_b_x;
  c6_eml_scalar_eg(chartInstance);
  c6_b_y = c6_y;
  c6_b_x = c6_x;
  return muDoubleScalarAtan2(c6_b_y, c6_b_x);
}

static real_T c6_b_mpower(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c6_a)
{
  real_T c6_b_a;
  real_T c6_c_a;
  real_T c6_ak;
  real_T c6_d_a;
  real_T c6_ar;
  c6_b_a = c6_a;
  c6_c_a = c6_b_a;
  c6_eml_scalar_eg(chartInstance);
  c6_ak = c6_c_a;
  c6_d_a = c6_ak;
  c6_eml_scalar_eg(chartInstance);
  c6_ar = c6_d_a;
  return muDoubleScalarPower(c6_ar, 3.0);
}

static void c6_c_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_d_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_e_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_f_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_g_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_mrdivide(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c6_A[18], real_T c6_B[9], real_T c6_y[18])
{
  int32_T c6_i202;
  real_T c6_b_B[9];
  int32_T c6_i203;
  real_T c6_b_A[18];
  for (c6_i202 = 0; c6_i202 < 9; c6_i202++) {
    c6_b_B[c6_i202] = c6_B[c6_i202];
  }

  for (c6_i203 = 0; c6_i203 < 18; c6_i203++) {
    c6_b_A[c6_i203] = c6_A[c6_i203];
  }

  c6_eml_lusolve(chartInstance, c6_b_B, c6_b_A, c6_y);
}

static void c6_eml_lusolve(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c6_A[9], real_T c6_B[18], real_T c6_X[18])
{
  int32_T c6_i204;
  real_T c6_b_A[9];
  int32_T c6_r1;
  int32_T c6_r2;
  int32_T c6_r3;
  real_T c6_x;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_y;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_b_y;
  real_T c6_maxval;
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_j_x;
  real_T c6_c_y;
  real_T c6_k_x;
  real_T c6_l_x;
  real_T c6_d_y;
  real_T c6_a21;
  real_T c6_m_x;
  real_T c6_n_x;
  real_T c6_o_x;
  real_T c6_p_x;
  real_T c6_e_y;
  real_T c6_q_x;
  real_T c6_r_x;
  real_T c6_f_y;
  real_T c6_d;
  real_T c6_s_x;
  real_T c6_g_y;
  real_T c6_t_x;
  real_T c6_h_y;
  real_T c6_u_x;
  real_T c6_i_y;
  real_T c6_z;
  real_T c6_v_x;
  real_T c6_j_y;
  real_T c6_w_x;
  real_T c6_k_y;
  real_T c6_x_x;
  real_T c6_l_y;
  real_T c6_b_z;
  real_T c6_y_x;
  real_T c6_ab_x;
  real_T c6_bb_x;
  real_T c6_cb_x;
  real_T c6_m_y;
  real_T c6_db_x;
  real_T c6_eb_x;
  real_T c6_n_y;
  real_T c6_b_d;
  real_T c6_fb_x;
  real_T c6_gb_x;
  real_T c6_hb_x;
  real_T c6_ib_x;
  real_T c6_o_y;
  real_T c6_jb_x;
  real_T c6_kb_x;
  real_T c6_p_y;
  real_T c6_c_d;
  int32_T c6_rtemp;
  real_T c6_lb_x;
  real_T c6_q_y;
  real_T c6_mb_x;
  real_T c6_r_y;
  real_T c6_nb_x;
  real_T c6_s_y;
  real_T c6_c_z;
  int32_T c6_k;
  int32_T c6_b_k;
  real_T c6_ob_x;
  real_T c6_t_y;
  real_T c6_pb_x;
  real_T c6_u_y;
  real_T c6_qb_x;
  real_T c6_v_y;
  real_T c6_d_z;
  real_T c6_rb_x;
  real_T c6_w_y;
  real_T c6_sb_x;
  real_T c6_x_y;
  real_T c6_tb_x;
  real_T c6_y_y;
  real_T c6_e_z;
  real_T c6_ub_x;
  real_T c6_ab_y;
  real_T c6_vb_x;
  real_T c6_bb_y;
  real_T c6_wb_x;
  real_T c6_cb_y;
  real_T c6_f_z;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  for (c6_i204 = 0; c6_i204 < 9; c6_i204++) {
    c6_b_A[c6_i204] = c6_A[c6_i204];
  }

  c6_r1 = 1;
  c6_r2 = 2;
  c6_r3 = 3;
  c6_x = c6_b_A[0];
  c6_b_x = c6_x;
  c6_c_x = c6_b_x;
  c6_d_x = c6_c_x;
  c6_y = muDoubleScalarAbs(c6_d_x);
  c6_e_x = 0.0;
  c6_f_x = c6_e_x;
  c6_b_y = muDoubleScalarAbs(c6_f_x);
  c6_maxval = c6_y + c6_b_y;
  c6_g_x = c6_b_A[1];
  c6_h_x = c6_g_x;
  c6_i_x = c6_h_x;
  c6_j_x = c6_i_x;
  c6_c_y = muDoubleScalarAbs(c6_j_x);
  c6_k_x = 0.0;
  c6_l_x = c6_k_x;
  c6_d_y = muDoubleScalarAbs(c6_l_x);
  c6_a21 = c6_c_y + c6_d_y;
  if (c6_a21 > c6_maxval) {
    c6_maxval = c6_a21;
    c6_r1 = 2;
    c6_r2 = 1;
  }

  c6_m_x = c6_b_A[2];
  c6_n_x = c6_m_x;
  c6_o_x = c6_n_x;
  c6_p_x = c6_o_x;
  c6_e_y = muDoubleScalarAbs(c6_p_x);
  c6_q_x = 0.0;
  c6_r_x = c6_q_x;
  c6_f_y = muDoubleScalarAbs(c6_r_x);
  c6_d = c6_e_y + c6_f_y;
  if (c6_d > c6_maxval) {
    c6_r1 = 3;
    c6_r2 = 2;
    c6_r3 = 1;
  }

  c6_s_x = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r2), 1, 3, 1, 0) - 1];
  c6_g_y = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r1), 1, 3, 1, 0) - 1];
  c6_t_x = c6_s_x;
  c6_h_y = c6_g_y;
  c6_u_x = c6_t_x;
  c6_i_y = c6_h_y;
  c6_z = c6_u_x / c6_i_y;
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r2), 1, 3, 1, 0) - 1] = c6_z;
  c6_v_x = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r3), 1, 3, 1, 0) - 1];
  c6_j_y = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r1), 1, 3, 1, 0) - 1];
  c6_w_x = c6_v_x;
  c6_k_y = c6_j_y;
  c6_x_x = c6_w_x;
  c6_l_y = c6_k_y;
  c6_b_z = c6_x_x / c6_l_y;
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r3), 1, 3, 1, 0) - 1] = c6_b_z;
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r2), 1, 3, 1, 0) + 2] = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 1, 0) + 2] -
    c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r2), 1, 3, 1, 0) - 1] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 1, 0) + 2];
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r3), 1, 3, 1, 0) + 2] = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 1, 0) + 2] -
    c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r3), 1, 3, 1, 0) - 1] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 1, 0) + 2];
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r2), 1, 3, 1, 0) + 5] = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 1, 0) + 5] -
    c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r2), 1, 3, 1, 0) - 1] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 1, 0) + 5];
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r3), 1, 3, 1, 0) + 5] = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 1, 0) + 5] -
    c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r3), 1, 3, 1, 0) - 1] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 1, 0) + 5];
  c6_y_x = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r3), 1, 3, 1, 0) + 2];
  c6_ab_x = c6_y_x;
  c6_bb_x = c6_ab_x;
  c6_cb_x = c6_bb_x;
  c6_m_y = muDoubleScalarAbs(c6_cb_x);
  c6_db_x = 0.0;
  c6_eb_x = c6_db_x;
  c6_n_y = muDoubleScalarAbs(c6_eb_x);
  c6_b_d = c6_m_y + c6_n_y;
  c6_fb_x = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c6_r2), 1, 3, 1, 0) + 2];
  c6_gb_x = c6_fb_x;
  c6_hb_x = c6_gb_x;
  c6_ib_x = c6_hb_x;
  c6_o_y = muDoubleScalarAbs(c6_ib_x);
  c6_jb_x = 0.0;
  c6_kb_x = c6_jb_x;
  c6_p_y = muDoubleScalarAbs(c6_kb_x);
  c6_c_d = c6_o_y + c6_p_y;
  if (c6_b_d > c6_c_d) {
    c6_rtemp = c6_r2;
    c6_r2 = c6_r3;
    c6_r3 = c6_rtemp;
  }

  c6_lb_x = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c6_r3), 1, 3, 1, 0) + 2];
  c6_q_y = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r2), 1, 3, 1, 0) + 2];
  c6_mb_x = c6_lb_x;
  c6_r_y = c6_q_y;
  c6_nb_x = c6_mb_x;
  c6_s_y = c6_r_y;
  c6_c_z = c6_nb_x / c6_s_y;
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r3), 1, 3, 1, 0) + 2] = c6_c_z;
  c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_r3), 1, 3, 1, 0) + 5] = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 1, 0) + 5] -
    c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c6_r3), 1, 3, 1, 0) + 2] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 1, 0) + 5];
  guard1 = false;
  guard2 = false;
  if (c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_r1), 1, 3, 1, 0) - 1] == 0.0) {
    guard2 = true;
  } else if (c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
               "", (real_T)c6_r2), 1, 3, 1, 0) + 2] == 0.0) {
    guard2 = true;
  } else {
    if (c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_r3), 1, 3, 1, 0) + 5] == 0.0) {
      guard1 = true;
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c6_eml_warning(chartInstance);
  }

  for (c6_k = 1; c6_k < 7; c6_k++) {
    c6_b_k = c6_k;
    c6_ob_x = c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
    c6_t_y = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_r1), 1, 3, 1, 0) - 1];
    c6_pb_x = c6_ob_x;
    c6_u_y = c6_t_y;
    c6_qb_x = c6_pb_x;
    c6_v_y = c6_u_y;
    c6_d_z = c6_qb_x / c6_v_y;
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 2, 0) - 1)) -
      1] = c6_d_z;
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 2, 0) - 1)) -
      1] = c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_b_k), 1, 6, 1, 0) + 5] - c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) + 6 *
      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_r1), 1, 3, 2, 0) - 1)) - 1] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 1, 0) + 2];
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 2, 0) - 1)) -
      1] = c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_b_k), 1, 6, 1, 0) + 11] - c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) + 6 *
      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_r1), 1, 3, 2, 0) - 1)) - 1] * c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 1, 0) + 5];
    c6_rb_x = c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 2, 0) - 1)) - 1];
    c6_w_y = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_r2), 1, 3, 1, 0) + 2];
    c6_sb_x = c6_rb_x;
    c6_x_y = c6_w_y;
    c6_tb_x = c6_sb_x;
    c6_y_y = c6_x_y;
    c6_e_z = c6_tb_x / c6_y_y;
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 2, 0) - 1)) -
      1] = c6_e_z;
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 2, 0) - 1)) -
      1] = c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c6_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c6_r3), 1, 3, 2, 0) - 1)) - 1] - c6_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 2, 0) - 1)) - 1] *
      c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_r2), 1, 3, 1, 0) + 5];
    c6_ub_x = c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 2, 0) - 1)) - 1];
    c6_ab_y = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c6_r3), 1, 3, 1, 0) + 5];
    c6_vb_x = c6_ub_x;
    c6_bb_y = c6_ab_y;
    c6_wb_x = c6_vb_x;
    c6_cb_y = c6_bb_y;
    c6_f_z = c6_wb_x / c6_cb_y;
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 2, 0) - 1)) -
      1] = c6_f_z;
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 2, 0) - 1)) -
      1] = c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c6_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c6_r2), 1, 3, 2, 0) - 1)) - 1] - c6_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 2, 0) - 1)) - 1] *
      c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_r3), 1, 3, 1, 0) + 2];
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 2, 0) - 1)) -
      1] = c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c6_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c6_r1), 1, 3, 2, 0) - 1)) - 1] - c6_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_r3), 1, 3, 2, 0) - 1)) - 1] *
      c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_r3), 1, 3, 1, 0) - 1];
    c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_r1), 1, 3, 2, 0) - 1)) -
      1] = c6_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c6_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c6_r1), 1, 3, 2, 0) - 1)) - 1] - c6_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_r2), 1, 3, 2, 0) - 1)) - 1] *
      c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_r2), 1, 3, 1, 0) - 1];
  }
}

static void c6_eml_warning(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  int32_T c6_i205;
  static char_T c6_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c6_u[27];
  const mxArray *c6_y = NULL;
  (void)chartInstance;
  for (c6_i205 = 0; c6_i205 < 27; c6_i205++) {
    c6_u[c6_i205] = c6_varargin_1[c6_i205];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c6_y));
}

static void c6_h_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_i_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_eye(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c6_I[36])
{
  int32_T c6_i206;
  int32_T c6_k;
  int32_T c6_b_k;
  (void)chartInstance;
  for (c6_i206 = 0; c6_i206 < 36; c6_i206++) {
    c6_I[c6_i206] = 0.0;
  }

  for (c6_k = 1; c6_k < 7; c6_k++) {
    c6_b_k = c6_k;
    c6_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 2, 0) - 1)) -
      1] = 1.0;
  }
}

static void c6_j_eml_scalar_eg(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c6_m_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static int32_T c6_t_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i207;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i207, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i207;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_u_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_AutoFollow_Simulation, const
  char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_v_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_AutoFollow_Simulation), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_AutoFollow_Simulation);
  return c6_y;
}

static uint8_T c6_v_emlrt_marshallIn(SFc6_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_sqrt(SFc6_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T *c6_x)
{
  if (*c6_x < 0.0) {
    c6_eml_error(chartInstance);
  }

  *c6_x = muDoubleScalarSqrt(*c6_x);
}

static void init_dsm_address_info(SFc6_AutoFollow_SimulationInstanceStruct
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

void sf_c6_AutoFollow_Simulation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2358405197U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3634011401U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1044684090U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(553701235U);
}

mxArray *sf_c6_AutoFollow_Simulation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("fM5HFctc9srUlvE1jTp8tD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_AutoFollow_Simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c6_AutoFollow_Simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c6_AutoFollow_Simulation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x8'type','srcId','name','auxInfo'{{M[1],M[18],T\"xhatOut\",},{M[4],M[0],T\"P\",S'l','i','p'{{M1x2[446 447],M[0],}}},{M[4],M[0],T\"Q\",S'l','i','p'{{M1x2[453 454],M[0],}}},{M[4],M[0],T\"R\",S'l','i','p'{{M1x2[455 456],M[0],}}},{M[4],M[0],T\"prevMeas\",S'l','i','p'{{M1x2[437 445],M[0],}}},{M[4],M[0],T\"prevT\",S'l','i','p'{{M1x2[431 436],M[0],}}},{M[4],M[0],T\"xhat\",S'l','i','p'{{M1x2[448 452],M[0],}}},{M[8],M[0],T\"is_active_c6_AutoFollow_Simulation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 8, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_AutoFollow_Simulation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _AutoFollow_SimulationMachineNumber_,
           6,
           1,
           1,
           0,
           8,
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
          _SFD_SET_DATA_PROPS(1,1,1,0,"phiq");
          _SFD_SET_DATA_PROPS(2,1,1,0,"psiq");
          _SFD_SET_DATA_PROPS(3,1,1,0,"hr");
          _SFD_SET_DATA_PROPS(4,1,1,0,"htheta");
          _SFD_SET_DATA_PROPS(5,1,1,0,"hpsi");
          _SFD_SET_DATA_PROPS(6,2,0,1,"xhatOut");
          _SFD_SET_DATA_PROPS(7,1,1,0,"t");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,3,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2386);
        _SFD_CV_INIT_EML_IF(0,1,0,514,527,-1,890);
        _SFD_CV_INIT_EML_IF(0,1,1,2151,2234,-1,2336);

        {
          static int condStart[] = { 2155, 2183, 2211 };

          static int condEnd[] = { 2177, 2205, 2233 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,2154,2234,3,0,&(condStart[0]),&(condEnd[0]),
                                5,&(pfixExpr[0]));
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_g_sf_marshallOut,(MexInFcnForType)
            c6_g_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c6_thetaq;
          real_T *c6_phiq;
          real_T *c6_psiq;
          real_T *c6_hr;
          real_T *c6_htheta;
          real_T *c6_hpsi;
          real_T *c6_t;
          real_T (*c6_xhatOut)[6];
          c6_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c6_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_hpsi = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c6_htheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c6_hr = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c6_psiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c6_phiq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c6_thetaq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c6_thetaq);
          _SFD_SET_DATA_VALUE_PTR(1U, c6_phiq);
          _SFD_SET_DATA_VALUE_PTR(2U, c6_psiq);
          _SFD_SET_DATA_VALUE_PTR(3U, c6_hr);
          _SFD_SET_DATA_VALUE_PTR(4U, c6_htheta);
          _SFD_SET_DATA_VALUE_PTR(5U, c6_hpsi);
          _SFD_SET_DATA_VALUE_PTR(6U, *c6_xhatOut);
          _SFD_SET_DATA_VALUE_PTR(7U, c6_t);
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
  return "6qQ1mOSRx4m5QZJvQwNJg";
}

static void sf_opaque_initialize_c6_AutoFollow_Simulation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_AutoFollow_Simulation
    ((SFc6_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
  initialize_c6_AutoFollow_Simulation((SFc6_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c6_AutoFollow_Simulation(void *chartInstanceVar)
{
  enable_c6_AutoFollow_Simulation((SFc6_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_AutoFollow_Simulation(void *chartInstanceVar)
{
  disable_c6_AutoFollow_Simulation((SFc6_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_AutoFollow_Simulation(void *chartInstanceVar)
{
  sf_gateway_c6_AutoFollow_Simulation((SFc6_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_AutoFollow_Simulation
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_AutoFollow_Simulation
    ((SFc6_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_AutoFollow_Simulation();/* state var info */
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

extern void sf_internal_set_sim_state_c6_AutoFollow_Simulation(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c6_AutoFollow_Simulation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_AutoFollow_Simulation
    ((SFc6_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_AutoFollow_Simulation(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c6_AutoFollow_Simulation(S);
}

static void sf_opaque_set_sim_state_c6_AutoFollow_Simulation(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c6_AutoFollow_Simulation(S, st);
}

static void sf_opaque_terminate_c6_AutoFollow_Simulation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_AutoFollow_SimulationInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_AutoFollow_Simulation_optimization_info();
    }

    finalize_c6_AutoFollow_Simulation((SFc6_AutoFollow_SimulationInstanceStruct*)
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
  initSimStructsc6_AutoFollow_Simulation
    ((SFc6_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_AutoFollow_Simulation(SimStruct *S)
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
    initialize_params_c6_AutoFollow_Simulation
      ((SFc6_AutoFollow_SimulationInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_AutoFollow_Simulation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,6,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,7);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 7; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1776657116U));
  ssSetChecksum1(S,(252769471U));
  ssSetChecksum2(S,(4146263919U));
  ssSetChecksum3(S,(2568406699U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_AutoFollow_Simulation(SimStruct *S)
{
  SFc6_AutoFollow_SimulationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc6_AutoFollow_SimulationInstanceStruct *)utMalloc(sizeof
    (SFc6_AutoFollow_SimulationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_AutoFollow_SimulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_AutoFollow_Simulation;
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

void c6_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_AutoFollow_Simulation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_AutoFollow_Simulation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
