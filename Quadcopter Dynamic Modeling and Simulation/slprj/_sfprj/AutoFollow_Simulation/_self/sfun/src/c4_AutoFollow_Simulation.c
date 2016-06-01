/* Include files */

#include <stddef.h>
#include "blas.h"
#include "AutoFollow_Simulation_sfun.h"
#include "c4_AutoFollow_Simulation.h"
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
static const char * c4_debug_family_names[24] = { "dt", "velSlowDown", "A", "B",
  "C", "y", "K", "u", "nargin", "nargout", "hx_cam", "hy_cam", "hz_cam", "v",
  "w", "newMeas", "t", "xhatOut", "prevT", "prevY", "P", "xhat", "Q", "R" };

/* Function Declarations */
static void initialize_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initialize_params_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void enable_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void disable_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void set_sim_state_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c4_st);
static void finalize_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void sf_gateway_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c4_chartstep_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initSimStructsc4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static void c4_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_R, const char_T *c4_identifier, real_T
  c4_y[9]);
static void c4_b_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[9]);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_c_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_Q, const char_T *c4_identifier, real_T
  c4_y[36]);
static void c4_d_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36]);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_e_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_xhat, const char_T *c4_identifier, real_T
  c4_y[6]);
static void c4_f_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[6]);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_g_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_P, const char_T *c4_identifier, real_T
  c4_y[36]);
static void c4_h_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36]);
static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_i_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_prevY, const char_T *c4_identifier, real_T
  c4_y[3]);
static void c4_j_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3]);
static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_k_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_prevT, const char_T *c4_identifier);
static real_T c4_l_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_m_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_xhatOut, const char_T *c4_identifier, real_T
  c4_y[6]);
static void c4_n_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[6]);
static void c4_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_h_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_o_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_i_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_p_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3]);
static void c4_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_j_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_q_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[18]);
static void c4_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_k_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_l_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_r_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36]);
static void c4_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_info_helper(const mxArray **c4_info);
static const mxArray *c4_emlrt_marshallOut(const char * c4_u);
static const mxArray *c4_b_emlrt_marshallOut(const uint32_T c4_u);
static void c4_diag(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                    real_T c4_v[6], real_T c4_d[36]);
static void c4_b_diag(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c4_v[3], real_T c4_d[9]);
static void c4_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_threshold(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c4_b_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static real_T c4_abs(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c4_x);
static void c4_c_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_d_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_e_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_mrdivide(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c4_A[18], real_T c4_B[9], real_T c4_y[18]);
static void c4_eml_lusolve(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c4_A[9], real_T c4_B[18], real_T c4_X[18]);
static void c4_eml_warning(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_f_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_g_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c4_eye(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c4_I[36]);
static void c4_h_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static const mxArray *c4_m_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_s_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_t_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_AutoFollow_Simulation, const
  char_T *c4_identifier);
static uint8_T c4_u_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void init_dsm_address_info(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c4_prevT_not_empty = false;
  chartInstance->c4_prevY_not_empty = false;
  chartInstance->c4_P_not_empty = false;
  chartInstance->c4_xhat_not_empty = false;
  chartInstance->c4_Q_not_empty = false;
  chartInstance->c4_R_not_empty = false;
  chartInstance->c4_is_active_c4_AutoFollow_Simulation = 0U;
}

static void initialize_params_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c4_update_debugger_state_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  int32_T c4_i0;
  real_T c4_u[6];
  const mxArray *c4_b_y = NULL;
  int32_T c4_i1;
  real_T c4_b_u[36];
  const mxArray *c4_c_y = NULL;
  int32_T c4_i2;
  real_T c4_c_u[36];
  const mxArray *c4_d_y = NULL;
  int32_T c4_i3;
  real_T c4_d_u[9];
  const mxArray *c4_e_y = NULL;
  real_T c4_hoistedGlobal;
  real_T c4_e_u;
  const mxArray *c4_f_y = NULL;
  int32_T c4_i4;
  real_T c4_f_u[3];
  const mxArray *c4_g_y = NULL;
  int32_T c4_i5;
  real_T c4_g_u[6];
  const mxArray *c4_h_y = NULL;
  uint8_T c4_b_hoistedGlobal;
  uint8_T c4_h_u;
  const mxArray *c4_i_y = NULL;
  real_T (*c4_xhatOut)[6];
  c4_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellmatrix(8, 1), false);
  for (c4_i0 = 0; c4_i0 < 6; c4_i0++) {
    c4_u[c4_i0] = (*c4_xhatOut)[c4_i0];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  for (c4_i1 = 0; c4_i1 < 36; c4_i1++) {
    c4_b_u[c4_i1] = chartInstance->c4_P[c4_i1];
  }

  c4_c_y = NULL;
  if (!chartInstance->c4_P_not_empty) {
    sf_mex_assign(&c4_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c4_c_y, sf_mex_create("y", c4_b_u, 0, 0U, 1U, 0U, 2, 6, 6),
                  false);
  }

  sf_mex_setcell(c4_y, 1, c4_c_y);
  for (c4_i2 = 0; c4_i2 < 36; c4_i2++) {
    c4_c_u[c4_i2] = chartInstance->c4_Q[c4_i2];
  }

  c4_d_y = NULL;
  if (!chartInstance->c4_Q_not_empty) {
    sf_mex_assign(&c4_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c4_d_y, sf_mex_create("y", c4_c_u, 0, 0U, 1U, 0U, 2, 6, 6),
                  false);
  }

  sf_mex_setcell(c4_y, 2, c4_d_y);
  for (c4_i3 = 0; c4_i3 < 9; c4_i3++) {
    c4_d_u[c4_i3] = chartInstance->c4_R[c4_i3];
  }

  c4_e_y = NULL;
  if (!chartInstance->c4_R_not_empty) {
    sf_mex_assign(&c4_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c4_e_y, sf_mex_create("y", c4_d_u, 0, 0U, 1U, 0U, 2, 3, 3),
                  false);
  }

  sf_mex_setcell(c4_y, 3, c4_e_y);
  c4_hoistedGlobal = chartInstance->c4_prevT;
  c4_e_u = c4_hoistedGlobal;
  c4_f_y = NULL;
  if (!chartInstance->c4_prevT_not_empty) {
    sf_mex_assign(&c4_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c4_f_y, sf_mex_create("y", &c4_e_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c4_y, 4, c4_f_y);
  for (c4_i4 = 0; c4_i4 < 3; c4_i4++) {
    c4_f_u[c4_i4] = chartInstance->c4_prevY[c4_i4];
  }

  c4_g_y = NULL;
  if (!chartInstance->c4_prevY_not_empty) {
    sf_mex_assign(&c4_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c4_g_y, sf_mex_create("y", c4_f_u, 0, 0U, 1U, 0U, 1, 3),
                  false);
  }

  sf_mex_setcell(c4_y, 5, c4_g_y);
  for (c4_i5 = 0; c4_i5 < 6; c4_i5++) {
    c4_g_u[c4_i5] = chartInstance->c4_xhat[c4_i5];
  }

  c4_h_y = NULL;
  if (!chartInstance->c4_xhat_not_empty) {
    sf_mex_assign(&c4_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c4_h_y, sf_mex_create("y", c4_g_u, 0, 0U, 1U, 0U, 1, 6),
                  false);
  }

  sf_mex_setcell(c4_y, 6, c4_h_y);
  c4_b_hoistedGlobal = chartInstance->c4_is_active_c4_AutoFollow_Simulation;
  c4_h_u = c4_b_hoistedGlobal;
  c4_i_y = NULL;
  sf_mex_assign(&c4_i_y, sf_mex_create("y", &c4_h_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c4_y, 7, c4_i_y);
  sf_mex_assign(&c4_st, c4_y, false);
  return c4_st;
}

static void set_sim_state_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[6];
  int32_T c4_i6;
  real_T c4_dv1[36];
  int32_T c4_i7;
  real_T c4_dv2[36];
  int32_T c4_i8;
  real_T c4_dv3[9];
  int32_T c4_i9;
  real_T c4_dv4[3];
  int32_T c4_i10;
  real_T c4_dv5[6];
  int32_T c4_i11;
  real_T (*c4_xhatOut)[6];
  c4_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = true;
  c4_u = sf_mex_dup(c4_st);
  c4_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)),
                        "xhatOut", c4_dv0);
  for (c4_i6 = 0; c4_i6 < 6; c4_i6++) {
    (*c4_xhatOut)[c4_i6] = c4_dv0[c4_i6];
  }

  c4_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)), "P",
                        c4_dv1);
  for (c4_i7 = 0; c4_i7 < 36; c4_i7++) {
    chartInstance->c4_P[c4_i7] = c4_dv1[c4_i7];
  }

  c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 2)), "Q",
                        c4_dv2);
  for (c4_i8 = 0; c4_i8 < 36; c4_i8++) {
    chartInstance->c4_Q[c4_i8] = c4_dv2[c4_i8];
  }

  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 3)), "R",
                      c4_dv3);
  for (c4_i9 = 0; c4_i9 < 9; c4_i9++) {
    chartInstance->c4_R[c4_i9] = c4_dv3[c4_i9];
  }

  chartInstance->c4_prevT = c4_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c4_u, 4)), "prevT");
  c4_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 5)),
                        "prevY", c4_dv4);
  for (c4_i10 = 0; c4_i10 < 3; c4_i10++) {
    chartInstance->c4_prevY[c4_i10] = c4_dv4[c4_i10];
  }

  c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 6)),
                        "xhat", c4_dv5);
  for (c4_i11 = 0; c4_i11 < 6; c4_i11++) {
    chartInstance->c4_xhat[c4_i11] = c4_dv5[c4_i11];
  }

  chartInstance->c4_is_active_c4_AutoFollow_Simulation = c4_t_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 7)),
     "is_active_c4_AutoFollow_Simulation");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_AutoFollow_Simulation(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  int32_T c4_i12;
  real_T *c4_hx_cam;
  real_T *c4_hy_cam;
  real_T *c4_hz_cam;
  real_T *c4_u;
  real_T *c4_v;
  real_T *c4_w;
  real_T *c4_newMeas;
  real_T *c4_t;
  real_T (*c4_xhatOut)[6];
  c4_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c4_newMeas = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c4_w = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c4_v = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c4_u = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_hz_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c4_hy_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c4_hx_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c4_hx_cam, 0U);
  _SFD_DATA_RANGE_CHECK(*c4_hy_cam, 1U);
  _SFD_DATA_RANGE_CHECK(*c4_hz_cam, 2U);
  _SFD_DATA_RANGE_CHECK(*c4_u, 3U);
  _SFD_DATA_RANGE_CHECK(*c4_v, 4U);
  _SFD_DATA_RANGE_CHECK(*c4_w, 5U);
  _SFD_DATA_RANGE_CHECK(*c4_newMeas, 6U);
  _SFD_DATA_RANGE_CHECK(*c4_t, 7U);
  chartInstance->c4_sfEvent = CALL_EVENT;
  c4_chartstep_c4_AutoFollow_Simulation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_AutoFollow_SimulationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c4_i12 = 0; c4_i12 < 6; c4_i12++) {
    _SFD_DATA_RANGE_CHECK((*c4_xhatOut)[c4_i12], 8U);
  }
}

static void c4_chartstep_c4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  real_T c4_hoistedGlobal;
  real_T c4_b_hoistedGlobal;
  real_T c4_c_hoistedGlobal;
  real_T c4_d_hoistedGlobal;
  real_T c4_e_hoistedGlobal;
  real_T c4_f_hoistedGlobal;
  real_T c4_g_hoistedGlobal;
  real_T c4_h_hoistedGlobal;
  real_T c4_hx_cam;
  real_T c4_hy_cam;
  real_T c4_hz_cam;
  real_T c4_u;
  real_T c4_v;
  real_T c4_w;
  real_T c4_newMeas;
  real_T c4_t;
  uint32_T c4_debug_family_var_map[24];
  real_T c4_dt;
  real_T c4_velSlowDown;
  real_T c4_A[36];
  real_T c4_B[18];
  real_T c4_C[18];
  real_T c4_y[3];
  real_T c4_K[18];
  real_T c4_b_u[3];
  real_T c4_nargin = 8.0;
  real_T c4_nargout = 1.0;
  real_T c4_xhatOut[6];
  int32_T c4_i13;
  static real_T c4_dv6[6] = { 0.0, 0.0, 10.0, 0.0, 0.0, 0.0 };

  int32_T c4_i14;
  int32_T c4_i15;
  static real_T c4_dv7[6] = { 100.0, 100.0, 40000.0, 160000.0, 160000.0, 1.96E+6
  };

  real_T c4_dv8[6];
  real_T c4_dv9[36];
  int32_T c4_i16;
  int32_T c4_i17;
  static real_T c4_dv10[3] = { 100.0, 100.0, 40000.0 };

  real_T c4_dv11[3];
  real_T c4_dv12[9];
  int32_T c4_i18;
  int32_T c4_i19;
  real_T c4_b_A;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_b_y;
  real_T c4_c_A;
  real_T c4_d_x;
  real_T c4_e_x;
  real_T c4_f_x;
  real_T c4_c_y;
  real_T c4_d_A;
  real_T c4_g_x;
  real_T c4_h_x;
  real_T c4_i_x;
  real_T c4_d_y;
  int32_T c4_i20;
  int32_T c4_i21;
  static real_T c4_dv13[6] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 };

  int32_T c4_i22;
  int32_T c4_i23;
  static real_T c4_dv14[6] = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 };

  int32_T c4_i24;
  int32_T c4_i25;
  static real_T c4_dv15[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c4_i26;
  static real_T c4_dv16[18] = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0 };

  int32_T c4_i27;
  static real_T c4_a[18] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c4_i28;
  real_T c4_i_hoistedGlobal[6];
  int32_T c4_i29;
  real_T c4_b_a[36];
  int32_T c4_i30;
  real_T c4_e_y[6];
  int32_T c4_i31;
  int32_T c4_i32;
  int32_T c4_i33;
  int32_T c4_i34;
  real_T c4_j_hoistedGlobal[36];
  int32_T c4_i35;
  int32_T c4_i36;
  int32_T c4_i37;
  int32_T c4_i38;
  real_T c4_f_y[36];
  int32_T c4_i39;
  int32_T c4_i40;
  int32_T c4_i41;
  int32_T c4_i42;
  int32_T c4_i43;
  int32_T c4_i44;
  int32_T c4_i45;
  int32_T c4_i46;
  int32_T c4_i47;
  int32_T c4_i48;
  int32_T c4_i49;
  int32_T c4_i50;
  int32_T c4_i51;
  int32_T c4_i52;
  int32_T c4_i53;
  int32_T c4_i54;
  real_T c4_g_y[18];
  int32_T c4_i55;
  int32_T c4_i56;
  static real_T c4_b[18] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };

  int32_T c4_i57;
  int32_T c4_i58;
  int32_T c4_i59;
  int32_T c4_i60;
  int32_T c4_i61;
  real_T c4_h_y[18];
  int32_T c4_i62;
  int32_T c4_i63;
  int32_T c4_i64;
  int32_T c4_i65;
  int32_T c4_i66;
  int32_T c4_i67;
  real_T c4_i_y[9];
  int32_T c4_i68;
  int32_T c4_i69;
  int32_T c4_i70;
  real_T c4_j_y[18];
  int32_T c4_i71;
  real_T c4_k_y[9];
  real_T c4_dv17[18];
  int32_T c4_i72;
  int32_T c4_i73;
  int32_T c4_i74;
  int32_T c4_i75;
  real_T c4_l_y[3];
  int32_T c4_i76;
  int32_T c4_i77;
  int32_T c4_i78;
  int32_T c4_i79;
  int32_T c4_i80;
  int32_T c4_i81;
  int32_T c4_i82;
  int32_T c4_i83;
  int32_T c4_i84;
  int32_T c4_i85;
  int32_T c4_i86;
  int32_T c4_i87;
  int32_T c4_i88;
  int32_T c4_i89;
  int32_T c4_i90;
  int32_T c4_i91;
  int32_T c4_i92;
  int32_T c4_i93;
  int32_T c4_i94;
  int32_T c4_i95;
  int32_T c4_i96;
  int32_T c4_i97;
  int32_T c4_i98;
  int32_T c4_i99;
  int32_T c4_i100;
  int32_T c4_i101;
  real_T *c4_b_hx_cam;
  real_T *c4_b_hy_cam;
  real_T *c4_b_hz_cam;
  real_T *c4_c_u;
  real_T *c4_b_v;
  real_T *c4_b_w;
  real_T *c4_b_newMeas;
  real_T *c4_b_t;
  real_T (*c4_b_xhatOut)[6];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  c4_b_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c4_b_newMeas = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c4_b_w = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c4_b_v = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c4_c_u = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_b_hz_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c4_b_hy_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c4_b_hx_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *c4_b_hx_cam;
  c4_b_hoistedGlobal = *c4_b_hy_cam;
  c4_c_hoistedGlobal = *c4_b_hz_cam;
  c4_d_hoistedGlobal = *c4_c_u;
  c4_e_hoistedGlobal = *c4_b_v;
  c4_f_hoistedGlobal = *c4_b_w;
  c4_g_hoistedGlobal = *c4_b_newMeas;
  c4_h_hoistedGlobal = *c4_b_t;
  c4_hx_cam = c4_hoistedGlobal;
  c4_hy_cam = c4_b_hoistedGlobal;
  c4_hz_cam = c4_c_hoistedGlobal;
  c4_u = c4_d_hoistedGlobal;
  c4_v = c4_e_hoistedGlobal;
  c4_w = c4_f_hoistedGlobal;
  c4_newMeas = c4_g_hoistedGlobal;
  c4_t = c4_h_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 24U, 25U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_dt, 0U, c4_h_sf_marshallOut,
    c4_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_velSlowDown, 1U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_A, 2U, c4_l_sf_marshallOut,
    c4_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_B, 3U, c4_j_sf_marshallOut,
    c4_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_C, 4U, c4_k_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_y, 5U, c4_i_sf_marshallOut,
    c4_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_K, 6U, c4_j_sf_marshallOut,
    c4_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_u, MAX_uint32_T, c4_i_sf_marshallOut,
    c4_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 8U, c4_h_sf_marshallOut,
    c4_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 9U, c4_h_sf_marshallOut,
    c4_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_hx_cam, 10U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_hy_cam, 11U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_hz_cam, 12U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_u, 7U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_v, 13U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_w, 14U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_newMeas, 15U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_t, 16U, c4_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_xhatOut, 17U, c4_g_sf_marshallOut,
    c4_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c4_prevT, 18U,
    c4_f_sf_marshallOut, c4_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c4_prevY, 19U,
    c4_e_sf_marshallOut, c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c4_P, 20U,
    c4_d_sf_marshallOut, c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c4_xhat, 21U,
    c4_c_sf_marshallOut, c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c4_Q, 22U,
    c4_b_sf_marshallOut, c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c4_R, 23U,
    c4_sf_marshallOut, c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 10);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 12);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c4_P_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 14);
    for (c4_i13 = 0; c4_i13 < 6; c4_i13++) {
      chartInstance->c4_xhat[c4_i13] = c4_dv6[c4_i13];
    }

    chartInstance->c4_xhat_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 17);
    for (c4_i14 = 0; c4_i14 < 36; c4_i14++) {
      chartInstance->c4_P[c4_i14] = 0.0;
    }

    chartInstance->c4_P_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 20);
    for (c4_i15 = 0; c4_i15 < 6; c4_i15++) {
      c4_dv8[c4_i15] = c4_dv7[c4_i15];
    }

    c4_diag(chartInstance, c4_dv8, c4_dv9);
    for (c4_i16 = 0; c4_i16 < 36; c4_i16++) {
      chartInstance->c4_Q[c4_i16] = c4_dv9[c4_i16];
    }

    chartInstance->c4_Q_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 23);
    for (c4_i17 = 0; c4_i17 < 3; c4_i17++) {
      c4_dv11[c4_i17] = c4_dv10[c4_i17];
    }

    c4_b_diag(chartInstance, c4_dv11, c4_dv12);
    for (c4_i18 = 0; c4_i18 < 9; c4_i18++) {
      chartInstance->c4_R[c4_i18] = c4_dv12[c4_i18];
    }

    chartInstance->c4_R_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 26);
    chartInstance->c4_prevT = 0.0;
    chartInstance->c4_prevT_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 29);
    for (c4_i19 = 0; c4_i19 < 3; c4_i19++) {
      chartInstance->c4_prevY[c4_i19] = 0.0;
    }

    chartInstance->c4_prevY_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 31);
  c4_dt = c4_t - chartInstance->c4_prevT;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 34);
  c4_velSlowDown = 20.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 35);
  c4_b_A = c4_dt;
  c4_x = c4_b_A;
  c4_b_x = c4_x;
  c4_c_x = c4_b_x;
  c4_b_y = c4_c_x / 20.0;
  c4_c_A = c4_dt;
  c4_d_x = c4_c_A;
  c4_e_x = c4_d_x;
  c4_f_x = c4_e_x;
  c4_c_y = c4_f_x / 20.0;
  c4_d_A = c4_dt;
  c4_g_x = c4_d_A;
  c4_h_x = c4_g_x;
  c4_i_x = c4_h_x;
  c4_d_y = c4_i_x / 20.0;
  c4_A[0] = 1.0;
  c4_A[6] = 0.0;
  c4_A[12] = 0.0;
  c4_A[18] = c4_b_y;
  c4_A[24] = 0.0;
  c4_A[30] = 0.0;
  c4_A[1] = 0.0;
  c4_A[7] = 1.0;
  c4_A[13] = 0.0;
  c4_A[19] = 0.0;
  c4_A[25] = c4_c_y;
  c4_A[31] = 0.0;
  c4_A[2] = 0.0;
  c4_A[8] = 0.0;
  c4_A[14] = 1.0;
  c4_A[20] = 0.0;
  c4_A[26] = 0.0;
  c4_A[32] = c4_d_y;
  c4_i20 = 0;
  for (c4_i21 = 0; c4_i21 < 6; c4_i21++) {
    c4_A[c4_i20 + 3] = c4_dv13[c4_i21];
    c4_i20 += 6;
  }

  c4_i22 = 0;
  for (c4_i23 = 0; c4_i23 < 6; c4_i23++) {
    c4_A[c4_i22 + 4] = c4_dv14[c4_i23];
    c4_i22 += 6;
  }

  c4_i24 = 0;
  for (c4_i25 = 0; c4_i25 < 6; c4_i25++) {
    c4_A[c4_i24 + 5] = c4_dv15[c4_i25];
    c4_i24 += 6;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 42);
  c4_b_u[0] = c4_u;
  c4_b_u[1] = c4_v;
  c4_b_u[2] = c4_w;
  _SFD_SYMBOL_SWITCH(7U, 7U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 43);
  for (c4_i26 = 0; c4_i26 < 18; c4_i26++) {
    c4_B[c4_i26] = c4_dv16[c4_i26];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 50);
  for (c4_i27 = 0; c4_i27 < 18; c4_i27++) {
    c4_C[c4_i27] = c4_a[c4_i27];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 55);
  for (c4_i28 = 0; c4_i28 < 6; c4_i28++) {
    c4_i_hoistedGlobal[c4_i28] = chartInstance->c4_xhat[c4_i28];
  }

  for (c4_i29 = 0; c4_i29 < 36; c4_i29++) {
    c4_b_a[c4_i29] = c4_A[c4_i29];
  }

  c4_eml_scalar_eg(chartInstance);
  c4_eml_scalar_eg(chartInstance);
  c4_threshold(chartInstance);
  for (c4_i30 = 0; c4_i30 < 6; c4_i30++) {
    c4_e_y[c4_i30] = 0.0;
    c4_i31 = 0;
    for (c4_i32 = 0; c4_i32 < 6; c4_i32++) {
      c4_e_y[c4_i30] += c4_b_a[c4_i31 + c4_i30] * c4_i_hoistedGlobal[c4_i32];
      c4_i31 += 6;
    }
  }

  for (c4_i33 = 0; c4_i33 < 6; c4_i33++) {
    chartInstance->c4_xhat[c4_i33] = c4_e_y[c4_i33];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 59);
  for (c4_i34 = 0; c4_i34 < 36; c4_i34++) {
    c4_j_hoistedGlobal[c4_i34] = chartInstance->c4_P[c4_i34];
  }

  for (c4_i35 = 0; c4_i35 < 36; c4_i35++) {
    c4_b_a[c4_i35] = c4_A[c4_i35];
  }

  c4_b_eml_scalar_eg(chartInstance);
  c4_b_eml_scalar_eg(chartInstance);
  c4_threshold(chartInstance);
  for (c4_i36 = 0; c4_i36 < 6; c4_i36++) {
    c4_i37 = 0;
    for (c4_i38 = 0; c4_i38 < 6; c4_i38++) {
      c4_f_y[c4_i37 + c4_i36] = 0.0;
      c4_i39 = 0;
      for (c4_i40 = 0; c4_i40 < 6; c4_i40++) {
        c4_f_y[c4_i37 + c4_i36] += c4_b_a[c4_i39 + c4_i36] *
          c4_j_hoistedGlobal[c4_i40 + c4_i37];
        c4_i39 += 6;
      }

      c4_i37 += 6;
    }
  }

  c4_i41 = 0;
  for (c4_i42 = 0; c4_i42 < 6; c4_i42++) {
    c4_i43 = 0;
    for (c4_i44 = 0; c4_i44 < 6; c4_i44++) {
      c4_b_a[c4_i44 + c4_i41] = c4_A[c4_i43 + c4_i42];
      c4_i43 += 6;
    }

    c4_i41 += 6;
  }

  c4_b_eml_scalar_eg(chartInstance);
  c4_b_eml_scalar_eg(chartInstance);
  c4_threshold(chartInstance);
  for (c4_i45 = 0; c4_i45 < 6; c4_i45++) {
    c4_i46 = 0;
    for (c4_i47 = 0; c4_i47 < 6; c4_i47++) {
      c4_j_hoistedGlobal[c4_i46 + c4_i45] = 0.0;
      c4_i48 = 0;
      for (c4_i49 = 0; c4_i49 < 6; c4_i49++) {
        c4_j_hoistedGlobal[c4_i46 + c4_i45] += c4_f_y[c4_i48 + c4_i45] *
          c4_b_a[c4_i49 + c4_i46];
        c4_i48 += 6;
      }

      c4_i46 += 6;
    }
  }

  for (c4_i50 = 0; c4_i50 < 36; c4_i50++) {
    chartInstance->c4_P[c4_i50] = c4_j_hoistedGlobal[c4_i50] +
      chartInstance->c4_Q[c4_i50];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 62);
  c4_y[0] = c4_hx_cam;
  c4_y[1] = c4_hy_cam;
  c4_y[2] = c4_hz_cam;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 65);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (CV_EML_COND(0, 1, 0, c4_abs(chartInstance, c4_y[0] -
        chartInstance->c4_prevY[0]) > 0.005)) {
    if (CV_EML_COND(0, 1, 1, c4_abs(chartInstance, c4_y[1] -
          chartInstance->c4_prevY[1]) > 0.005)) {
      if (CV_EML_COND(0, 1, 2, c4_abs(chartInstance, c4_y[2] -
            chartInstance->c4_prevY[2]) > 0.005)) {
        if (CV_EML_COND(0, 1, 3, c4_newMeas != 0.0)) {
          CV_EML_MCDC(0, 1, 0, true);
          CV_EML_IF(0, 1, 1, true);
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 70);
          for (c4_i51 = 0; c4_i51 < 36; c4_i51++) {
            c4_j_hoistedGlobal[c4_i51] = chartInstance->c4_P[c4_i51];
          }

          c4_c_eml_scalar_eg(chartInstance);
          c4_c_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i52 = 0; c4_i52 < 6; c4_i52++) {
            c4_i53 = 0;
            for (c4_i54 = 0; c4_i54 < 3; c4_i54++) {
              c4_g_y[c4_i53 + c4_i52] = 0.0;
              c4_i55 = 0;
              for (c4_i56 = 0; c4_i56 < 6; c4_i56++) {
                c4_g_y[c4_i53 + c4_i52] += c4_j_hoistedGlobal[c4_i55 + c4_i52] *
                  c4_b[c4_i56 + c4_i53];
                c4_i55 += 6;
              }

              c4_i53 += 6;
            }
          }

          for (c4_i57 = 0; c4_i57 < 36; c4_i57++) {
            c4_j_hoistedGlobal[c4_i57] = chartInstance->c4_P[c4_i57];
          }

          c4_d_eml_scalar_eg(chartInstance);
          c4_d_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i58 = 0; c4_i58 < 3; c4_i58++) {
            c4_i59 = 0;
            c4_i60 = 0;
            for (c4_i61 = 0; c4_i61 < 6; c4_i61++) {
              c4_h_y[c4_i59 + c4_i58] = 0.0;
              c4_i62 = 0;
              for (c4_i63 = 0; c4_i63 < 6; c4_i63++) {
                c4_h_y[c4_i59 + c4_i58] += c4_a[c4_i62 + c4_i58] *
                  c4_j_hoistedGlobal[c4_i63 + c4_i60];
                c4_i62 += 3;
              }

              c4_i59 += 3;
              c4_i60 += 6;
            }
          }

          c4_e_eml_scalar_eg(chartInstance);
          c4_e_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i64 = 0; c4_i64 < 3; c4_i64++) {
            c4_i65 = 0;
            c4_i66 = 0;
            for (c4_i67 = 0; c4_i67 < 3; c4_i67++) {
              c4_i_y[c4_i65 + c4_i64] = 0.0;
              c4_i68 = 0;
              for (c4_i69 = 0; c4_i69 < 6; c4_i69++) {
                c4_i_y[c4_i65 + c4_i64] += c4_h_y[c4_i68 + c4_i64] * c4_b[c4_i69
                  + c4_i66];
                c4_i68 += 3;
              }

              c4_i65 += 3;
              c4_i66 += 6;
            }
          }

          for (c4_i70 = 0; c4_i70 < 18; c4_i70++) {
            c4_j_y[c4_i70] = c4_g_y[c4_i70];
          }

          for (c4_i71 = 0; c4_i71 < 9; c4_i71++) {
            c4_k_y[c4_i71] = c4_i_y[c4_i71] + chartInstance->c4_R[c4_i71];
          }

          c4_mrdivide(chartInstance, c4_j_y, c4_k_y, c4_dv17);
          for (c4_i72 = 0; c4_i72 < 18; c4_i72++) {
            c4_K[c4_i72] = c4_dv17[c4_i72];
          }

          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 71);
          for (c4_i73 = 0; c4_i73 < 6; c4_i73++) {
            c4_i_hoistedGlobal[c4_i73] = chartInstance->c4_xhat[c4_i73];
          }

          for (c4_i74 = 0; c4_i74 < 6; c4_i74++) {
            c4_e_y[c4_i74] = chartInstance->c4_xhat[c4_i74];
          }

          c4_f_eml_scalar_eg(chartInstance);
          c4_f_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i75 = 0; c4_i75 < 3; c4_i75++) {
            c4_l_y[c4_i75] = 0.0;
            c4_i76 = 0;
            for (c4_i77 = 0; c4_i77 < 6; c4_i77++) {
              c4_l_y[c4_i75] += c4_a[c4_i76 + c4_i75] * c4_e_y[c4_i77];
              c4_i76 += 3;
            }
          }

          for (c4_i78 = 0; c4_i78 < 18; c4_i78++) {
            c4_g_y[c4_i78] = c4_K[c4_i78];
          }

          for (c4_i79 = 0; c4_i79 < 3; c4_i79++) {
            c4_l_y[c4_i79] = c4_y[c4_i79] - c4_l_y[c4_i79];
          }

          c4_g_eml_scalar_eg(chartInstance);
          c4_g_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i80 = 0; c4_i80 < 6; c4_i80++) {
            c4_e_y[c4_i80] = 0.0;
            c4_i81 = 0;
            for (c4_i82 = 0; c4_i82 < 3; c4_i82++) {
              c4_e_y[c4_i80] += c4_g_y[c4_i81 + c4_i80] * c4_l_y[c4_i82];
              c4_i81 += 6;
            }
          }

          for (c4_i83 = 0; c4_i83 < 6; c4_i83++) {
            chartInstance->c4_xhat[c4_i83] = c4_i_hoistedGlobal[c4_i83] +
              c4_e_y[c4_i83];
          }

          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 72);
          for (c4_i84 = 0; c4_i84 < 18; c4_i84++) {
            c4_g_y[c4_i84] = c4_K[c4_i84];
          }

          c4_h_eml_scalar_eg(chartInstance);
          c4_h_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i85 = 0; c4_i85 < 6; c4_i85++) {
            c4_i86 = 0;
            c4_i87 = 0;
            for (c4_i88 = 0; c4_i88 < 6; c4_i88++) {
              c4_f_y[c4_i86 + c4_i85] = 0.0;
              c4_i89 = 0;
              for (c4_i90 = 0; c4_i90 < 3; c4_i90++) {
                c4_f_y[c4_i86 + c4_i85] += c4_g_y[c4_i89 + c4_i85] * c4_a[c4_i90
                  + c4_i87];
                c4_i89 += 6;
              }

              c4_i86 += 6;
              c4_i87 += 3;
            }
          }

          for (c4_i91 = 0; c4_i91 < 36; c4_i91++) {
            c4_j_hoistedGlobal[c4_i91] = chartInstance->c4_P[c4_i91];
          }

          c4_eye(chartInstance, c4_b_a);
          for (c4_i92 = 0; c4_i92 < 36; c4_i92++) {
            c4_b_a[c4_i92] -= c4_f_y[c4_i92];
          }

          c4_b_eml_scalar_eg(chartInstance);
          c4_b_eml_scalar_eg(chartInstance);
          c4_threshold(chartInstance);
          for (c4_i93 = 0; c4_i93 < 6; c4_i93++) {
            c4_i94 = 0;
            for (c4_i95 = 0; c4_i95 < 6; c4_i95++) {
              c4_f_y[c4_i94 + c4_i93] = 0.0;
              c4_i96 = 0;
              for (c4_i97 = 0; c4_i97 < 6; c4_i97++) {
                c4_f_y[c4_i94 + c4_i93] += c4_b_a[c4_i96 + c4_i93] *
                  c4_j_hoistedGlobal[c4_i97 + c4_i94];
                c4_i96 += 6;
              }

              c4_i94 += 6;
            }
          }

          for (c4_i98 = 0; c4_i98 < 36; c4_i98++) {
            chartInstance->c4_P[c4_i98] = c4_f_y[c4_i98];
          }

          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 73);
          for (c4_i99 = 0; c4_i99 < 3; c4_i99++) {
            chartInstance->c4_prevY[c4_i99] = c4_y[c4_i99];
          }
        } else {
          guard1 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3 == true) {
    guard2 = true;
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 1, false);
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 78);
  for (c4_i100 = 0; c4_i100 < 6; c4_i100++) {
    c4_xhatOut[c4_i100] = chartInstance->c4_xhat[c4_i100];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -78);
  _SFD_SYMBOL_SCOPE_POP();
  for (c4_i101 = 0; c4_i101 < 6; c4_i101++) {
    (*c4_b_xhatOut)[c4_i101] = c4_xhatOut[c4_i101];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
}

static void initSimStructsc4_AutoFollow_Simulation
  (SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber)
{
  (void)c4_machineNumber;
  (void)c4_chartNumber;
  (void)c4_instanceNumber;
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i102;
  int32_T c4_i103;
  int32_T c4_i104;
  real_T c4_b_inData[9];
  int32_T c4_i105;
  int32_T c4_i106;
  int32_T c4_i107;
  real_T c4_u[9];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i102 = 0;
  for (c4_i103 = 0; c4_i103 < 3; c4_i103++) {
    for (c4_i104 = 0; c4_i104 < 3; c4_i104++) {
      c4_b_inData[c4_i104 + c4_i102] = (*(real_T (*)[9])c4_inData)[c4_i104 +
        c4_i102];
    }

    c4_i102 += 3;
  }

  c4_i105 = 0;
  for (c4_i106 = 0; c4_i106 < 3; c4_i106++) {
    for (c4_i107 = 0; c4_i107 < 3; c4_i107++) {
      c4_u[c4_i107 + c4_i105] = c4_b_inData[c4_i107 + c4_i105];
    }

    c4_i105 += 3;
  }

  c4_y = NULL;
  if (!chartInstance->c4_R_not_empty) {
    sf_mex_assign(&c4_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  }

  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_R, const char_T *c4_identifier, real_T
  c4_y[9])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_R), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_R);
}

static void c4_b_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[9])
{
  real_T c4_dv18[9];
  int32_T c4_i108;
  if (mxIsEmpty(c4_u)) {
    chartInstance->c4_R_not_empty = false;
  } else {
    chartInstance->c4_R_not_empty = true;
    sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv18, 1, 0, 0U, 1, 0U, 2, 3,
                  3);
    for (c4_i108 = 0; c4_i108 < 9; c4_i108++) {
      c4_y[c4_i108] = c4_dv18[c4_i108];
    }
  }

  sf_mex_destroy(&c4_u);
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_R;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[9];
  int32_T c4_i109;
  int32_T c4_i110;
  int32_T c4_i111;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_R = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_R), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_R);
  c4_i109 = 0;
  for (c4_i110 = 0; c4_i110 < 3; c4_i110++) {
    for (c4_i111 = 0; c4_i111 < 3; c4_i111++) {
      (*(real_T (*)[9])c4_outData)[c4_i111 + c4_i109] = c4_y[c4_i111 + c4_i109];
    }

    c4_i109 += 3;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i112;
  int32_T c4_i113;
  int32_T c4_i114;
  real_T c4_b_inData[36];
  int32_T c4_i115;
  int32_T c4_i116;
  int32_T c4_i117;
  real_T c4_u[36];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i112 = 0;
  for (c4_i113 = 0; c4_i113 < 6; c4_i113++) {
    for (c4_i114 = 0; c4_i114 < 6; c4_i114++) {
      c4_b_inData[c4_i114 + c4_i112] = (*(real_T (*)[36])c4_inData)[c4_i114 +
        c4_i112];
    }

    c4_i112 += 6;
  }

  c4_i115 = 0;
  for (c4_i116 = 0; c4_i116 < 6; c4_i116++) {
    for (c4_i117 = 0; c4_i117 < 6; c4_i117++) {
      c4_u[c4_i117 + c4_i115] = c4_b_inData[c4_i117 + c4_i115];
    }

    c4_i115 += 6;
  }

  c4_y = NULL;
  if (!chartInstance->c4_Q_not_empty) {
    sf_mex_assign(&c4_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  }

  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_c_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_Q, const char_T *c4_identifier, real_T
  c4_y[36])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_Q), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_Q);
}

static void c4_d_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36])
{
  real_T c4_dv19[36];
  int32_T c4_i118;
  if (mxIsEmpty(c4_u)) {
    chartInstance->c4_Q_not_empty = false;
  } else {
    chartInstance->c4_Q_not_empty = true;
    sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv19, 1, 0, 0U, 1, 0U, 2, 6,
                  6);
    for (c4_i118 = 0; c4_i118 < 36; c4_i118++) {
      c4_y[c4_i118] = c4_dv19[c4_i118];
    }
  }

  sf_mex_destroy(&c4_u);
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_Q;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[36];
  int32_T c4_i119;
  int32_T c4_i120;
  int32_T c4_i121;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_Q = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_Q), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_Q);
  c4_i119 = 0;
  for (c4_i120 = 0; c4_i120 < 6; c4_i120++) {
    for (c4_i121 = 0; c4_i121 < 6; c4_i121++) {
      (*(real_T (*)[36])c4_outData)[c4_i121 + c4_i119] = c4_y[c4_i121 + c4_i119];
    }

    c4_i119 += 6;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i122;
  real_T c4_b_inData[6];
  int32_T c4_i123;
  real_T c4_u[6];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i122 = 0; c4_i122 < 6; c4_i122++) {
    c4_b_inData[c4_i122] = (*(real_T (*)[6])c4_inData)[c4_i122];
  }

  for (c4_i123 = 0; c4_i123 < 6; c4_i123++) {
    c4_u[c4_i123] = c4_b_inData[c4_i123];
  }

  c4_y = NULL;
  if (!chartInstance->c4_xhat_not_empty) {
    sf_mex_assign(&c4_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 6), false);
  }

  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_e_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_xhat, const char_T *c4_identifier, real_T
  c4_y[6])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_xhat), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_xhat);
}

static void c4_f_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[6])
{
  real_T c4_dv20[6];
  int32_T c4_i124;
  if (mxIsEmpty(c4_u)) {
    chartInstance->c4_xhat_not_empty = false;
  } else {
    chartInstance->c4_xhat_not_empty = true;
    sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv20, 1, 0, 0U, 1, 0U, 1, 6);
    for (c4_i124 = 0; c4_i124 < 6; c4_i124++) {
      c4_y[c4_i124] = c4_dv20[c4_i124];
    }
  }

  sf_mex_destroy(&c4_u);
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_xhat;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[6];
  int32_T c4_i125;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_xhat = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_xhat), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_xhat);
  for (c4_i125 = 0; c4_i125 < 6; c4_i125++) {
    (*(real_T (*)[6])c4_outData)[c4_i125] = c4_y[c4_i125];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i126;
  int32_T c4_i127;
  int32_T c4_i128;
  real_T c4_b_inData[36];
  int32_T c4_i129;
  int32_T c4_i130;
  int32_T c4_i131;
  real_T c4_u[36];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i126 = 0;
  for (c4_i127 = 0; c4_i127 < 6; c4_i127++) {
    for (c4_i128 = 0; c4_i128 < 6; c4_i128++) {
      c4_b_inData[c4_i128 + c4_i126] = (*(real_T (*)[36])c4_inData)[c4_i128 +
        c4_i126];
    }

    c4_i126 += 6;
  }

  c4_i129 = 0;
  for (c4_i130 = 0; c4_i130 < 6; c4_i130++) {
    for (c4_i131 = 0; c4_i131 < 6; c4_i131++) {
      c4_u[c4_i131 + c4_i129] = c4_b_inData[c4_i131 + c4_i129];
    }

    c4_i129 += 6;
  }

  c4_y = NULL;
  if (!chartInstance->c4_P_not_empty) {
    sf_mex_assign(&c4_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  }

  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_g_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_P, const char_T *c4_identifier, real_T
  c4_y[36])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_P), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_P);
}

static void c4_h_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36])
{
  real_T c4_dv21[36];
  int32_T c4_i132;
  if (mxIsEmpty(c4_u)) {
    chartInstance->c4_P_not_empty = false;
  } else {
    chartInstance->c4_P_not_empty = true;
    sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv21, 1, 0, 0U, 1, 0U, 2, 6,
                  6);
    for (c4_i132 = 0; c4_i132 < 36; c4_i132++) {
      c4_y[c4_i132] = c4_dv21[c4_i132];
    }
  }

  sf_mex_destroy(&c4_u);
}

static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_P;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[36];
  int32_T c4_i133;
  int32_T c4_i134;
  int32_T c4_i135;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_P = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_P), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_P);
  c4_i133 = 0;
  for (c4_i134 = 0; c4_i134 < 6; c4_i134++) {
    for (c4_i135 = 0; c4_i135 < 6; c4_i135++) {
      (*(real_T (*)[36])c4_outData)[c4_i135 + c4_i133] = c4_y[c4_i135 + c4_i133];
    }

    c4_i133 += 6;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i136;
  real_T c4_b_inData[3];
  int32_T c4_i137;
  real_T c4_u[3];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i136 = 0; c4_i136 < 3; c4_i136++) {
    c4_b_inData[c4_i136] = (*(real_T (*)[3])c4_inData)[c4_i136];
  }

  for (c4_i137 = 0; c4_i137 < 3; c4_i137++) {
    c4_u[c4_i137] = c4_b_inData[c4_i137];
  }

  c4_y = NULL;
  if (!chartInstance->c4_prevY_not_empty) {
    sf_mex_assign(&c4_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 3), false);
  }

  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_i_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_prevY, const char_T *c4_identifier, real_T
  c4_y[3])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_prevY), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_prevY);
}

static void c4_j_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3])
{
  real_T c4_dv22[3];
  int32_T c4_i138;
  if (mxIsEmpty(c4_u)) {
    chartInstance->c4_prevY_not_empty = false;
  } else {
    chartInstance->c4_prevY_not_empty = true;
    sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv22, 1, 0, 0U, 1, 0U, 1, 3);
    for (c4_i138 = 0; c4_i138 < 3; c4_i138++) {
      c4_y[c4_i138] = c4_dv22[c4_i138];
    }
  }

  sf_mex_destroy(&c4_u);
}

static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_prevY;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[3];
  int32_T c4_i139;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_prevY = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_prevY), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_b_prevY);
  for (c4_i139 = 0; c4_i139 < 3; c4_i139++) {
    (*(real_T (*)[3])c4_outData)[c4_i139] = c4_y[c4_i139];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  if (!chartInstance->c4_prevT_not_empty) {
    sf_mex_assign(&c4_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static real_T c4_k_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_prevT, const char_T *c4_identifier)
{
  real_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_prevT), &c4_thisId);
  sf_mex_destroy(&c4_b_prevT);
  return c4_y;
}

static real_T c4_l_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  if (mxIsEmpty(c4_u)) {
    chartInstance->c4_prevT_not_empty = false;
  } else {
    chartInstance->c4_prevT_not_empty = true;
    sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
    c4_y = c4_d0;
  }

  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_prevT;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_prevT = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_prevT), &c4_thisId);
  sf_mex_destroy(&c4_b_prevT);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i140;
  real_T c4_b_inData[6];
  int32_T c4_i141;
  real_T c4_u[6];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i140 = 0; c4_i140 < 6; c4_i140++) {
    c4_b_inData[c4_i140] = (*(real_T (*)[6])c4_inData)[c4_i140];
  }

  for (c4_i141 = 0; c4_i141 < 6; c4_i141++) {
    c4_u[c4_i141] = c4_b_inData[c4_i141];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_m_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_xhatOut, const char_T *c4_identifier, real_T
  c4_y[6])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_xhatOut), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_xhatOut);
}

static void c4_n_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[6])
{
  real_T c4_dv23[6];
  int32_T c4_i142;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv23, 1, 0, 0U, 1, 0U, 1, 6);
  for (c4_i142 = 0; c4_i142 < 6; c4_i142++) {
    c4_y[c4_i142] = c4_dv23[c4_i142];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_xhatOut;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[6];
  int32_T c4_i143;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_xhatOut = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_xhatOut), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_xhatOut);
  for (c4_i143 = 0; c4_i143 < 6; c4_i143++) {
    (*(real_T (*)[6])c4_outData)[c4_i143] = c4_y[c4_i143];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_h_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static real_T c4_o_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d1;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d1, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d1;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_nargout;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_nargout = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_nargout), &c4_thisId);
  sf_mex_destroy(&c4_nargout);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_i_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i144;
  real_T c4_b_inData[3];
  int32_T c4_i145;
  real_T c4_u[3];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i144 = 0; c4_i144 < 3; c4_i144++) {
    c4_b_inData[c4_i144] = (*(real_T (*)[3])c4_inData)[c4_i144];
  }

  for (c4_i145 = 0; c4_i145 < 3; c4_i145++) {
    c4_u[c4_i145] = c4_b_inData[c4_i145];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_p_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[3])
{
  real_T c4_dv24[3];
  int32_T c4_i146;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv24, 1, 0, 0U, 1, 0U, 1, 3);
  for (c4_i146 = 0; c4_i146 < 3; c4_i146++) {
    c4_y[c4_i146] = c4_dv24[c4_i146];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_u;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[3];
  int32_T c4_i147;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_u = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_u), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_u);
  for (c4_i147 = 0; c4_i147 < 3; c4_i147++) {
    (*(real_T (*)[3])c4_outData)[c4_i147] = c4_y[c4_i147];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_j_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i148;
  int32_T c4_i149;
  int32_T c4_i150;
  real_T c4_b_inData[18];
  int32_T c4_i151;
  int32_T c4_i152;
  int32_T c4_i153;
  real_T c4_u[18];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i148 = 0;
  for (c4_i149 = 0; c4_i149 < 3; c4_i149++) {
    for (c4_i150 = 0; c4_i150 < 6; c4_i150++) {
      c4_b_inData[c4_i150 + c4_i148] = (*(real_T (*)[18])c4_inData)[c4_i150 +
        c4_i148];
    }

    c4_i148 += 6;
  }

  c4_i151 = 0;
  for (c4_i152 = 0; c4_i152 < 3; c4_i152++) {
    for (c4_i153 = 0; c4_i153 < 6; c4_i153++) {
      c4_u[c4_i153 + c4_i151] = c4_b_inData[c4_i153 + c4_i151];
    }

    c4_i151 += 6;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 3), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_q_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[18])
{
  real_T c4_dv25[18];
  int32_T c4_i154;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv25, 1, 0, 0U, 1, 0U, 2, 6, 3);
  for (c4_i154 = 0; c4_i154 < 18; c4_i154++) {
    c4_y[c4_i154] = c4_dv25[c4_i154];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_K;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[18];
  int32_T c4_i155;
  int32_T c4_i156;
  int32_T c4_i157;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_K = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_K), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_K);
  c4_i155 = 0;
  for (c4_i156 = 0; c4_i156 < 3; c4_i156++) {
    for (c4_i157 = 0; c4_i157 < 6; c4_i157++) {
      (*(real_T (*)[18])c4_outData)[c4_i157 + c4_i155] = c4_y[c4_i157 + c4_i155];
    }

    c4_i155 += 6;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_k_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i158;
  int32_T c4_i159;
  int32_T c4_i160;
  real_T c4_b_inData[18];
  int32_T c4_i161;
  int32_T c4_i162;
  int32_T c4_i163;
  real_T c4_u[18];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i158 = 0;
  for (c4_i159 = 0; c4_i159 < 6; c4_i159++) {
    for (c4_i160 = 0; c4_i160 < 3; c4_i160++) {
      c4_b_inData[c4_i160 + c4_i158] = (*(real_T (*)[18])c4_inData)[c4_i160 +
        c4_i158];
    }

    c4_i158 += 3;
  }

  c4_i161 = 0;
  for (c4_i162 = 0; c4_i162 < 6; c4_i162++) {
    for (c4_i163 = 0; c4_i163 < 3; c4_i163++) {
      c4_u[c4_i163 + c4_i161] = c4_b_inData[c4_i163 + c4_i161];
    }

    c4_i161 += 3;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 3, 6), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static const mxArray *c4_l_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i164;
  int32_T c4_i165;
  int32_T c4_i166;
  real_T c4_b_inData[36];
  int32_T c4_i167;
  int32_T c4_i168;
  int32_T c4_i169;
  real_T c4_u[36];
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i164 = 0;
  for (c4_i165 = 0; c4_i165 < 6; c4_i165++) {
    for (c4_i166 = 0; c4_i166 < 6; c4_i166++) {
      c4_b_inData[c4_i166 + c4_i164] = (*(real_T (*)[36])c4_inData)[c4_i166 +
        c4_i164];
    }

    c4_i164 += 6;
  }

  c4_i167 = 0;
  for (c4_i168 = 0; c4_i168 < 6; c4_i168++) {
    for (c4_i169 = 0; c4_i169 < 6; c4_i169++) {
      c4_u[c4_i169 + c4_i167] = c4_b_inData[c4_i169 + c4_i167];
    }

    c4_i167 += 6;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static void c4_r_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36])
{
  real_T c4_dv26[36];
  int32_T c4_i170;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv26, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c4_i170 = 0; c4_i170 < 36; c4_i170++) {
    c4_y[c4_i170] = c4_dv26[c4_i170];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_A;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[36];
  int32_T c4_i171;
  int32_T c4_i172;
  int32_T c4_i173;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_A = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_A), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_A);
  c4_i171 = 0;
  for (c4_i172 = 0; c4_i172 < 6; c4_i172++) {
    for (c4_i173 = 0; c4_i173 < 6; c4_i173++) {
      (*(real_T (*)[36])c4_outData)[c4_i173 + c4_i171] = c4_y[c4_i173 + c4_i171];
    }

    c4_i171 += 6;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_AutoFollow_Simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  sf_mex_assign(&c4_nameCaptureInfo, sf_mex_createstruct("structure", 2, 64, 1),
                false);
  c4_info_helper(&c4_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(const mxArray **c4_info)
{
  const mxArray *c4_rhs0 = NULL;
  const mxArray *c4_lhs0 = NULL;
  const mxArray *c4_rhs1 = NULL;
  const mxArray *c4_lhs1 = NULL;
  const mxArray *c4_rhs2 = NULL;
  const mxArray *c4_lhs2 = NULL;
  const mxArray *c4_rhs3 = NULL;
  const mxArray *c4_lhs3 = NULL;
  const mxArray *c4_rhs4 = NULL;
  const mxArray *c4_lhs4 = NULL;
  const mxArray *c4_rhs5 = NULL;
  const mxArray *c4_lhs5 = NULL;
  const mxArray *c4_rhs6 = NULL;
  const mxArray *c4_lhs6 = NULL;
  const mxArray *c4_rhs7 = NULL;
  const mxArray *c4_lhs7 = NULL;
  const mxArray *c4_rhs8 = NULL;
  const mxArray *c4_lhs8 = NULL;
  const mxArray *c4_rhs9 = NULL;
  const mxArray *c4_lhs9 = NULL;
  const mxArray *c4_rhs10 = NULL;
  const mxArray *c4_lhs10 = NULL;
  const mxArray *c4_rhs11 = NULL;
  const mxArray *c4_lhs11 = NULL;
  const mxArray *c4_rhs12 = NULL;
  const mxArray *c4_lhs12 = NULL;
  const mxArray *c4_rhs13 = NULL;
  const mxArray *c4_lhs13 = NULL;
  const mxArray *c4_rhs14 = NULL;
  const mxArray *c4_lhs14 = NULL;
  const mxArray *c4_rhs15 = NULL;
  const mxArray *c4_lhs15 = NULL;
  const mxArray *c4_rhs16 = NULL;
  const mxArray *c4_lhs16 = NULL;
  const mxArray *c4_rhs17 = NULL;
  const mxArray *c4_lhs17 = NULL;
  const mxArray *c4_rhs18 = NULL;
  const mxArray *c4_lhs18 = NULL;
  const mxArray *c4_rhs19 = NULL;
  const mxArray *c4_lhs19 = NULL;
  const mxArray *c4_rhs20 = NULL;
  const mxArray *c4_lhs20 = NULL;
  const mxArray *c4_rhs21 = NULL;
  const mxArray *c4_lhs21 = NULL;
  const mxArray *c4_rhs22 = NULL;
  const mxArray *c4_lhs22 = NULL;
  const mxArray *c4_rhs23 = NULL;
  const mxArray *c4_lhs23 = NULL;
  const mxArray *c4_rhs24 = NULL;
  const mxArray *c4_lhs24 = NULL;
  const mxArray *c4_rhs25 = NULL;
  const mxArray *c4_lhs25 = NULL;
  const mxArray *c4_rhs26 = NULL;
  const mxArray *c4_lhs26 = NULL;
  const mxArray *c4_rhs27 = NULL;
  const mxArray *c4_lhs27 = NULL;
  const mxArray *c4_rhs28 = NULL;
  const mxArray *c4_lhs28 = NULL;
  const mxArray *c4_rhs29 = NULL;
  const mxArray *c4_lhs29 = NULL;
  const mxArray *c4_rhs30 = NULL;
  const mxArray *c4_lhs30 = NULL;
  const mxArray *c4_rhs31 = NULL;
  const mxArray *c4_lhs31 = NULL;
  const mxArray *c4_rhs32 = NULL;
  const mxArray *c4_lhs32 = NULL;
  const mxArray *c4_rhs33 = NULL;
  const mxArray *c4_lhs33 = NULL;
  const mxArray *c4_rhs34 = NULL;
  const mxArray *c4_lhs34 = NULL;
  const mxArray *c4_rhs35 = NULL;
  const mxArray *c4_lhs35 = NULL;
  const mxArray *c4_rhs36 = NULL;
  const mxArray *c4_lhs36 = NULL;
  const mxArray *c4_rhs37 = NULL;
  const mxArray *c4_lhs37 = NULL;
  const mxArray *c4_rhs38 = NULL;
  const mxArray *c4_lhs38 = NULL;
  const mxArray *c4_rhs39 = NULL;
  const mxArray *c4_lhs39 = NULL;
  const mxArray *c4_rhs40 = NULL;
  const mxArray *c4_lhs40 = NULL;
  const mxArray *c4_rhs41 = NULL;
  const mxArray *c4_lhs41 = NULL;
  const mxArray *c4_rhs42 = NULL;
  const mxArray *c4_lhs42 = NULL;
  const mxArray *c4_rhs43 = NULL;
  const mxArray *c4_lhs43 = NULL;
  const mxArray *c4_rhs44 = NULL;
  const mxArray *c4_lhs44 = NULL;
  const mxArray *c4_rhs45 = NULL;
  const mxArray *c4_lhs45 = NULL;
  const mxArray *c4_rhs46 = NULL;
  const mxArray *c4_lhs46 = NULL;
  const mxArray *c4_rhs47 = NULL;
  const mxArray *c4_lhs47 = NULL;
  const mxArray *c4_rhs48 = NULL;
  const mxArray *c4_lhs48 = NULL;
  const mxArray *c4_rhs49 = NULL;
  const mxArray *c4_lhs49 = NULL;
  const mxArray *c4_rhs50 = NULL;
  const mxArray *c4_lhs50 = NULL;
  const mxArray *c4_rhs51 = NULL;
  const mxArray *c4_lhs51 = NULL;
  const mxArray *c4_rhs52 = NULL;
  const mxArray *c4_lhs52 = NULL;
  const mxArray *c4_rhs53 = NULL;
  const mxArray *c4_lhs53 = NULL;
  const mxArray *c4_rhs54 = NULL;
  const mxArray *c4_lhs54 = NULL;
  const mxArray *c4_rhs55 = NULL;
  const mxArray *c4_lhs55 = NULL;
  const mxArray *c4_rhs56 = NULL;
  const mxArray *c4_lhs56 = NULL;
  const mxArray *c4_rhs57 = NULL;
  const mxArray *c4_lhs57 = NULL;
  const mxArray *c4_rhs58 = NULL;
  const mxArray *c4_lhs58 = NULL;
  const mxArray *c4_rhs59 = NULL;
  const mxArray *c4_lhs59 = NULL;
  const mxArray *c4_rhs60 = NULL;
  const mxArray *c4_lhs60 = NULL;
  const mxArray *c4_rhs61 = NULL;
  const mxArray *c4_lhs61 = NULL;
  const mxArray *c4_rhs62 = NULL;
  const mxArray *c4_lhs62 = NULL;
  const mxArray *c4_rhs63 = NULL;
  const mxArray *c4_lhs63 = NULL;
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("power"), "name", "name", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742680U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c4_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c4_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c4_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340320U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c4_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c4_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340320U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c4_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("floor"), "name", "name", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742654U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c4_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c4_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851126U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c4_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c4_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("diag"), "name", "name", 10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742654U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c4_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("ismatrix"), "name", "name", 11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1331337258U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c4_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c4_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c4_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c4_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("intmax"), "name", "name", 15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c4_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1381882700U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c4_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("mrdivide"), "name", "name", 17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1388492496U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1370042286U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c4_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c4_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("rdivide"), "name", "name", 19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742680U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c4_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c4_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851196U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c4_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_div"), "name", "name", 22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c4_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340320U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c4_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1383909694U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c4_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c4_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 26);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c4_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 27);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c4_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  28);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013090U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c4_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 29);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340322U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c4_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 30);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 30);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340322U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c4_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 31);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 31);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340322U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c4_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 32);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 32);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340322U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c4_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 33);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 33);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1381882700U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c4_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 34);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 34);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340320U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c4_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 35);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 35);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340322U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c4_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 36);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("abs"), "name", "name", 36);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742652U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c4_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 37);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c4_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 38);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851112U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c4_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 39);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("ismatrix"), "name", "name", 39);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1331337258U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c4_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 40);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_lusolve"), "name", "name",
                  40);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1370042286U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c4_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 41);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_xcabs1"), "name", "name",
                  41);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c4_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "context", "context", 42);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 42);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1389340322U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c4_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "context", "context", 43);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("abs"), "name", "name", 43);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742652U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c4_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 44);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("rdivide"), "name", "name", 44);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742680U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c4_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular"),
                  "context", "context", 45);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_warning"), "name", "name",
                  45);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851202U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c4_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 46);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 46);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 46);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c4_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3"),
                  "context", "context", 47);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 47);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c4_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "context", "context", 48);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eye"), "name", "name", 48);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 48);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1381882698U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c4_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 49);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 49);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1368215430U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c4_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 50);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 50);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c4_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 51);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("isinf"), "name", "name", 51);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 51);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363742656U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c4_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 52);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 52);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1363743356U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c4_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 53);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 53);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1286851182U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c4_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 54);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("intmax"), "name", "name", 54);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c4_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 55);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("intmin"), "name", "name", 55);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 55);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c4_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 56);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1381882700U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c4_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 57);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 57);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1326760722U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c4_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 58);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 58);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1381882700U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c4_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 59);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 59);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1326760396U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c4_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 60);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("intmin"), "name", "name", 60);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 60);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c4_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 61);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 61);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1323202978U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c4_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 62);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("intmax"), "name", "name", 62);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 62);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1362294282U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c4_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 63);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 63);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c4_info, c4_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(1376013088U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c4_info, c4_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c4_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c4_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c4_info, sf_mex_duplicatearraysafe(&c4_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c4_rhs0);
  sf_mex_destroy(&c4_lhs0);
  sf_mex_destroy(&c4_rhs1);
  sf_mex_destroy(&c4_lhs1);
  sf_mex_destroy(&c4_rhs2);
  sf_mex_destroy(&c4_lhs2);
  sf_mex_destroy(&c4_rhs3);
  sf_mex_destroy(&c4_lhs3);
  sf_mex_destroy(&c4_rhs4);
  sf_mex_destroy(&c4_lhs4);
  sf_mex_destroy(&c4_rhs5);
  sf_mex_destroy(&c4_lhs5);
  sf_mex_destroy(&c4_rhs6);
  sf_mex_destroy(&c4_lhs6);
  sf_mex_destroy(&c4_rhs7);
  sf_mex_destroy(&c4_lhs7);
  sf_mex_destroy(&c4_rhs8);
  sf_mex_destroy(&c4_lhs8);
  sf_mex_destroy(&c4_rhs9);
  sf_mex_destroy(&c4_lhs9);
  sf_mex_destroy(&c4_rhs10);
  sf_mex_destroy(&c4_lhs10);
  sf_mex_destroy(&c4_rhs11);
  sf_mex_destroy(&c4_lhs11);
  sf_mex_destroy(&c4_rhs12);
  sf_mex_destroy(&c4_lhs12);
  sf_mex_destroy(&c4_rhs13);
  sf_mex_destroy(&c4_lhs13);
  sf_mex_destroy(&c4_rhs14);
  sf_mex_destroy(&c4_lhs14);
  sf_mex_destroy(&c4_rhs15);
  sf_mex_destroy(&c4_lhs15);
  sf_mex_destroy(&c4_rhs16);
  sf_mex_destroy(&c4_lhs16);
  sf_mex_destroy(&c4_rhs17);
  sf_mex_destroy(&c4_lhs17);
  sf_mex_destroy(&c4_rhs18);
  sf_mex_destroy(&c4_lhs18);
  sf_mex_destroy(&c4_rhs19);
  sf_mex_destroy(&c4_lhs19);
  sf_mex_destroy(&c4_rhs20);
  sf_mex_destroy(&c4_lhs20);
  sf_mex_destroy(&c4_rhs21);
  sf_mex_destroy(&c4_lhs21);
  sf_mex_destroy(&c4_rhs22);
  sf_mex_destroy(&c4_lhs22);
  sf_mex_destroy(&c4_rhs23);
  sf_mex_destroy(&c4_lhs23);
  sf_mex_destroy(&c4_rhs24);
  sf_mex_destroy(&c4_lhs24);
  sf_mex_destroy(&c4_rhs25);
  sf_mex_destroy(&c4_lhs25);
  sf_mex_destroy(&c4_rhs26);
  sf_mex_destroy(&c4_lhs26);
  sf_mex_destroy(&c4_rhs27);
  sf_mex_destroy(&c4_lhs27);
  sf_mex_destroy(&c4_rhs28);
  sf_mex_destroy(&c4_lhs28);
  sf_mex_destroy(&c4_rhs29);
  sf_mex_destroy(&c4_lhs29);
  sf_mex_destroy(&c4_rhs30);
  sf_mex_destroy(&c4_lhs30);
  sf_mex_destroy(&c4_rhs31);
  sf_mex_destroy(&c4_lhs31);
  sf_mex_destroy(&c4_rhs32);
  sf_mex_destroy(&c4_lhs32);
  sf_mex_destroy(&c4_rhs33);
  sf_mex_destroy(&c4_lhs33);
  sf_mex_destroy(&c4_rhs34);
  sf_mex_destroy(&c4_lhs34);
  sf_mex_destroy(&c4_rhs35);
  sf_mex_destroy(&c4_lhs35);
  sf_mex_destroy(&c4_rhs36);
  sf_mex_destroy(&c4_lhs36);
  sf_mex_destroy(&c4_rhs37);
  sf_mex_destroy(&c4_lhs37);
  sf_mex_destroy(&c4_rhs38);
  sf_mex_destroy(&c4_lhs38);
  sf_mex_destroy(&c4_rhs39);
  sf_mex_destroy(&c4_lhs39);
  sf_mex_destroy(&c4_rhs40);
  sf_mex_destroy(&c4_lhs40);
  sf_mex_destroy(&c4_rhs41);
  sf_mex_destroy(&c4_lhs41);
  sf_mex_destroy(&c4_rhs42);
  sf_mex_destroy(&c4_lhs42);
  sf_mex_destroy(&c4_rhs43);
  sf_mex_destroy(&c4_lhs43);
  sf_mex_destroy(&c4_rhs44);
  sf_mex_destroy(&c4_lhs44);
  sf_mex_destroy(&c4_rhs45);
  sf_mex_destroy(&c4_lhs45);
  sf_mex_destroy(&c4_rhs46);
  sf_mex_destroy(&c4_lhs46);
  sf_mex_destroy(&c4_rhs47);
  sf_mex_destroy(&c4_lhs47);
  sf_mex_destroy(&c4_rhs48);
  sf_mex_destroy(&c4_lhs48);
  sf_mex_destroy(&c4_rhs49);
  sf_mex_destroy(&c4_lhs49);
  sf_mex_destroy(&c4_rhs50);
  sf_mex_destroy(&c4_lhs50);
  sf_mex_destroy(&c4_rhs51);
  sf_mex_destroy(&c4_lhs51);
  sf_mex_destroy(&c4_rhs52);
  sf_mex_destroy(&c4_lhs52);
  sf_mex_destroy(&c4_rhs53);
  sf_mex_destroy(&c4_lhs53);
  sf_mex_destroy(&c4_rhs54);
  sf_mex_destroy(&c4_lhs54);
  sf_mex_destroy(&c4_rhs55);
  sf_mex_destroy(&c4_lhs55);
  sf_mex_destroy(&c4_rhs56);
  sf_mex_destroy(&c4_lhs56);
  sf_mex_destroy(&c4_rhs57);
  sf_mex_destroy(&c4_lhs57);
  sf_mex_destroy(&c4_rhs58);
  sf_mex_destroy(&c4_lhs58);
  sf_mex_destroy(&c4_rhs59);
  sf_mex_destroy(&c4_lhs59);
  sf_mex_destroy(&c4_rhs60);
  sf_mex_destroy(&c4_lhs60);
  sf_mex_destroy(&c4_rhs61);
  sf_mex_destroy(&c4_lhs61);
  sf_mex_destroy(&c4_rhs62);
  sf_mex_destroy(&c4_lhs62);
  sf_mex_destroy(&c4_rhs63);
  sf_mex_destroy(&c4_lhs63);
}

static const mxArray *c4_emlrt_marshallOut(const char * c4_u)
{
  const mxArray *c4_y = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c4_u)), false);
  return c4_y;
}

static const mxArray *c4_b_emlrt_marshallOut(const uint32_T c4_u)
{
  const mxArray *c4_y = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 7, 0U, 0U, 0U, 0), false);
  return c4_y;
}

static void c4_diag(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                    real_T c4_v[6], real_T c4_d[36])
{
  int32_T c4_i174;
  int32_T c4_j;
  int32_T c4_b_j;
  (void)chartInstance;
  for (c4_i174 = 0; c4_i174 < 36; c4_i174++) {
    c4_d[c4_i174] = 0.0;
  }

  for (c4_j = 1; c4_j < 7; c4_j++) {
    c4_b_j = c4_j;
    c4_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_j), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0) - 1)) -
      1] = c4_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_b_j), 1, 6, 1, 0) - 1];
  }
}

static void c4_b_diag(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c4_v[3], real_T c4_d[9])
{
  int32_T c4_i175;
  int32_T c4_j;
  int32_T c4_b_j;
  (void)chartInstance;
  for (c4_i175 = 0; c4_i175 < 9; c4_i175++) {
    c4_d[c4_i175] = 0.0;
  }

  for (c4_j = 1; c4_j < 4; c4_j++) {
    c4_b_j = c4_j;
    c4_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_j), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 3, 2, 0) - 1)) -
      1] = c4_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_b_j), 1, 3, 1, 0) - 1];
  }
}

static void c4_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_threshold(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c4_b_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c4_abs(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c4_x)
{
  real_T c4_b_x;
  (void)chartInstance;
  c4_b_x = c4_x;
  return muDoubleScalarAbs(c4_b_x);
}

static void c4_c_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_d_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_e_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_mrdivide(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c4_A[18], real_T c4_B[9], real_T c4_y[18])
{
  int32_T c4_i176;
  real_T c4_b_B[9];
  int32_T c4_i177;
  real_T c4_b_A[18];
  for (c4_i176 = 0; c4_i176 < 9; c4_i176++) {
    c4_b_B[c4_i176] = c4_B[c4_i176];
  }

  for (c4_i177 = 0; c4_i177 < 18; c4_i177++) {
    c4_b_A[c4_i177] = c4_A[c4_i177];
  }

  c4_eml_lusolve(chartInstance, c4_b_B, c4_b_A, c4_y);
}

static void c4_eml_lusolve(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c4_A[9], real_T c4_B[18], real_T c4_X[18])
{
  int32_T c4_i178;
  real_T c4_b_A[9];
  int32_T c4_r1;
  int32_T c4_r2;
  int32_T c4_r3;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_d_x;
  real_T c4_y;
  real_T c4_e_x;
  real_T c4_f_x;
  real_T c4_b_y;
  real_T c4_maxval;
  real_T c4_g_x;
  real_T c4_h_x;
  real_T c4_i_x;
  real_T c4_j_x;
  real_T c4_c_y;
  real_T c4_k_x;
  real_T c4_l_x;
  real_T c4_d_y;
  real_T c4_a21;
  real_T c4_m_x;
  real_T c4_n_x;
  real_T c4_o_x;
  real_T c4_p_x;
  real_T c4_e_y;
  real_T c4_q_x;
  real_T c4_r_x;
  real_T c4_f_y;
  real_T c4_d;
  real_T c4_s_x;
  real_T c4_g_y;
  real_T c4_t_x;
  real_T c4_h_y;
  real_T c4_u_x;
  real_T c4_i_y;
  real_T c4_z;
  real_T c4_v_x;
  real_T c4_j_y;
  real_T c4_w_x;
  real_T c4_k_y;
  real_T c4_x_x;
  real_T c4_l_y;
  real_T c4_b_z;
  real_T c4_y_x;
  real_T c4_ab_x;
  real_T c4_bb_x;
  real_T c4_cb_x;
  real_T c4_m_y;
  real_T c4_db_x;
  real_T c4_eb_x;
  real_T c4_n_y;
  real_T c4_b_d;
  real_T c4_fb_x;
  real_T c4_gb_x;
  real_T c4_hb_x;
  real_T c4_ib_x;
  real_T c4_o_y;
  real_T c4_jb_x;
  real_T c4_kb_x;
  real_T c4_p_y;
  real_T c4_c_d;
  int32_T c4_rtemp;
  real_T c4_lb_x;
  real_T c4_q_y;
  real_T c4_mb_x;
  real_T c4_r_y;
  real_T c4_nb_x;
  real_T c4_s_y;
  real_T c4_c_z;
  int32_T c4_k;
  int32_T c4_b_k;
  real_T c4_ob_x;
  real_T c4_t_y;
  real_T c4_pb_x;
  real_T c4_u_y;
  real_T c4_qb_x;
  real_T c4_v_y;
  real_T c4_d_z;
  real_T c4_rb_x;
  real_T c4_w_y;
  real_T c4_sb_x;
  real_T c4_x_y;
  real_T c4_tb_x;
  real_T c4_y_y;
  real_T c4_e_z;
  real_T c4_ub_x;
  real_T c4_ab_y;
  real_T c4_vb_x;
  real_T c4_bb_y;
  real_T c4_wb_x;
  real_T c4_cb_y;
  real_T c4_f_z;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  for (c4_i178 = 0; c4_i178 < 9; c4_i178++) {
    c4_b_A[c4_i178] = c4_A[c4_i178];
  }

  c4_r1 = 1;
  c4_r2 = 2;
  c4_r3 = 3;
  c4_x = c4_b_A[0];
  c4_b_x = c4_x;
  c4_c_x = c4_b_x;
  c4_d_x = c4_c_x;
  c4_y = muDoubleScalarAbs(c4_d_x);
  c4_e_x = 0.0;
  c4_f_x = c4_e_x;
  c4_b_y = muDoubleScalarAbs(c4_f_x);
  c4_maxval = c4_y + c4_b_y;
  c4_g_x = c4_b_A[1];
  c4_h_x = c4_g_x;
  c4_i_x = c4_h_x;
  c4_j_x = c4_i_x;
  c4_c_y = muDoubleScalarAbs(c4_j_x);
  c4_k_x = 0.0;
  c4_l_x = c4_k_x;
  c4_d_y = muDoubleScalarAbs(c4_l_x);
  c4_a21 = c4_c_y + c4_d_y;
  if (c4_a21 > c4_maxval) {
    c4_maxval = c4_a21;
    c4_r1 = 2;
    c4_r2 = 1;
  }

  c4_m_x = c4_b_A[2];
  c4_n_x = c4_m_x;
  c4_o_x = c4_n_x;
  c4_p_x = c4_o_x;
  c4_e_y = muDoubleScalarAbs(c4_p_x);
  c4_q_x = 0.0;
  c4_r_x = c4_q_x;
  c4_f_y = muDoubleScalarAbs(c4_r_x);
  c4_d = c4_e_y + c4_f_y;
  if (c4_d > c4_maxval) {
    c4_r1 = 3;
    c4_r2 = 2;
    c4_r3 = 1;
  }

  c4_s_x = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r2), 1, 3, 1, 0) - 1];
  c4_g_y = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r1), 1, 3, 1, 0) - 1];
  c4_t_x = c4_s_x;
  c4_h_y = c4_g_y;
  c4_u_x = c4_t_x;
  c4_i_y = c4_h_y;
  c4_z = c4_u_x / c4_i_y;
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r2), 1, 3, 1, 0) - 1] = c4_z;
  c4_v_x = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r3), 1, 3, 1, 0) - 1];
  c4_j_y = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r1), 1, 3, 1, 0) - 1];
  c4_w_x = c4_v_x;
  c4_k_y = c4_j_y;
  c4_x_x = c4_w_x;
  c4_l_y = c4_k_y;
  c4_b_z = c4_x_x / c4_l_y;
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r3), 1, 3, 1, 0) - 1] = c4_b_z;
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r2), 1, 3, 1, 0) + 2] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 1, 0) + 2] -
    c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r2), 1, 3, 1, 0) - 1] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 1, 0) + 2];
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r3), 1, 3, 1, 0) + 2] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 1, 0) + 2] -
    c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r3), 1, 3, 1, 0) - 1] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 1, 0) + 2];
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r2), 1, 3, 1, 0) + 5] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 1, 0) + 5] -
    c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r2), 1, 3, 1, 0) - 1] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 1, 0) + 5];
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r3), 1, 3, 1, 0) + 5] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 1, 0) + 5] -
    c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r3), 1, 3, 1, 0) - 1] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 1, 0) + 5];
  c4_y_x = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r3), 1, 3, 1, 0) + 2];
  c4_ab_x = c4_y_x;
  c4_bb_x = c4_ab_x;
  c4_cb_x = c4_bb_x;
  c4_m_y = muDoubleScalarAbs(c4_cb_x);
  c4_db_x = 0.0;
  c4_eb_x = c4_db_x;
  c4_n_y = muDoubleScalarAbs(c4_eb_x);
  c4_b_d = c4_m_y + c4_n_y;
  c4_fb_x = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c4_r2), 1, 3, 1, 0) + 2];
  c4_gb_x = c4_fb_x;
  c4_hb_x = c4_gb_x;
  c4_ib_x = c4_hb_x;
  c4_o_y = muDoubleScalarAbs(c4_ib_x);
  c4_jb_x = 0.0;
  c4_kb_x = c4_jb_x;
  c4_p_y = muDoubleScalarAbs(c4_kb_x);
  c4_c_d = c4_o_y + c4_p_y;
  if (c4_b_d > c4_c_d) {
    c4_rtemp = c4_r2;
    c4_r2 = c4_r3;
    c4_r3 = c4_rtemp;
  }

  c4_lb_x = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c4_r3), 1, 3, 1, 0) + 2];
  c4_q_y = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r2), 1, 3, 1, 0) + 2];
  c4_mb_x = c4_lb_x;
  c4_r_y = c4_q_y;
  c4_nb_x = c4_mb_x;
  c4_s_y = c4_r_y;
  c4_c_z = c4_nb_x / c4_s_y;
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r3), 1, 3, 1, 0) + 2] = c4_c_z;
  c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c4_r3), 1, 3, 1, 0) + 5] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 1, 0) + 5] -
    c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c4_r3), 1, 3, 1, 0) + 2] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 1, 0) + 5];
  guard1 = false;
  guard2 = false;
  if (c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_r1), 1, 3, 1, 0) - 1] == 0.0) {
    guard2 = true;
  } else if (c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
               "", (real_T)c4_r2), 1, 3, 1, 0) + 2] == 0.0) {
    guard2 = true;
  } else {
    if (c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_r3), 1, 3, 1, 0) + 5] == 0.0) {
      guard1 = true;
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c4_eml_warning(chartInstance);
  }

  for (c4_k = 1; c4_k < 7; c4_k++) {
    c4_b_k = c4_k;
    c4_ob_x = c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
    c4_t_y = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_r1), 1, 3, 1, 0) - 1];
    c4_pb_x = c4_ob_x;
    c4_u_y = c4_t_y;
    c4_qb_x = c4_pb_x;
    c4_v_y = c4_u_y;
    c4_d_z = c4_qb_x / c4_v_y;
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 2, 0) - 1)) -
      1] = c4_d_z;
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 2, 0) - 1)) -
      1] = c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_b_k), 1, 6, 1, 0) + 5] - c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) + 6 *
      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_r1), 1, 3, 2, 0) - 1)) - 1] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 1, 0) + 2];
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 2, 0) - 1)) -
      1] = c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_b_k), 1, 6, 1, 0) + 11] - c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) + 6 *
      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_r1), 1, 3, 2, 0) - 1)) - 1] * c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 1, 0) + 5];
    c4_rb_x = c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 2, 0) - 1)) - 1];
    c4_w_y = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_r2), 1, 3, 1, 0) + 2];
    c4_sb_x = c4_rb_x;
    c4_x_y = c4_w_y;
    c4_tb_x = c4_sb_x;
    c4_y_y = c4_x_y;
    c4_e_z = c4_tb_x / c4_y_y;
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 2, 0) - 1)) -
      1] = c4_e_z;
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 2, 0) - 1)) -
      1] = c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c4_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c4_r3), 1, 3, 2, 0) - 1)) - 1] - c4_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 2, 0) - 1)) - 1] *
      c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_r2), 1, 3, 1, 0) + 5];
    c4_ub_x = c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 2, 0) - 1)) - 1];
    c4_ab_y = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c4_r3), 1, 3, 1, 0) + 5];
    c4_vb_x = c4_ub_x;
    c4_bb_y = c4_ab_y;
    c4_wb_x = c4_vb_x;
    c4_cb_y = c4_bb_y;
    c4_f_z = c4_wb_x / c4_cb_y;
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 2, 0) - 1)) -
      1] = c4_f_z;
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 2, 0) - 1)) -
      1] = c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c4_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c4_r2), 1, 3, 2, 0) - 1)) - 1] - c4_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 2, 0) - 1)) - 1] *
      c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_r3), 1, 3, 1, 0) + 2];
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 2, 0) - 1)) -
      1] = c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c4_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c4_r1), 1, 3, 2, 0) - 1)) - 1] - c4_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_r3), 1, 3, 2, 0) - 1)) - 1] *
      c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_r3), 1, 3, 1, 0) - 1];
    c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_r1), 1, 3, 2, 0) - 1)) -
      1] = c4_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c4_b_k), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c4_r1), 1, 3, 2, 0) - 1)) - 1] - c4_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_r2), 1, 3, 2, 0) - 1)) - 1] *
      c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_r2), 1, 3, 1, 0) - 1];
  }
}

static void c4_eml_warning(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  int32_T c4_i179;
  static char_T c4_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c4_u[27];
  const mxArray *c4_y = NULL;
  (void)chartInstance;
  for (c4_i179 = 0; c4_i179 < 27; c4_i179++) {
    c4_u[c4_i179] = c4_varargin_1[c4_i179];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c4_y));
}

static void c4_f_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_g_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_eye(SFc4_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c4_I[36])
{
  int32_T c4_i180;
  int32_T c4_k;
  int32_T c4_b_k;
  (void)chartInstance;
  for (c4_i180 = 0; c4_i180 < 36; c4_i180++) {
    c4_I[c4_i180] = 0.0;
  }

  for (c4_k = 1; c4_k < 7; c4_k++) {
    c4_b_k = c4_k;
    c4_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 2, 0) - 1)) -
      1] = 1.0;
  }
}

static void c4_h_eml_scalar_eg(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c4_m_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, false);
  return c4_mxArrayOutData;
}

static int32_T c4_s_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i181;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i181, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i181;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_t_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_AutoFollow_Simulation, const
  char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_u_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_AutoFollow_Simulation), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_AutoFollow_Simulation);
  return c4_y;
}

static uint8_T c4_u_emlrt_marshallIn(SFc4_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void init_dsm_address_info(SFc4_AutoFollow_SimulationInstanceStruct
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

void sf_c4_AutoFollow_Simulation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3765175796U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1523762742U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(93704847U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(63064860U);
}

mxArray *sf_c4_AutoFollow_Simulation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("7YxpVBvsFrkjC5Dn0pFPYD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,8,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));
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

mxArray *sf_c4_AutoFollow_Simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c4_AutoFollow_Simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c4_AutoFollow_Simulation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x8'type','srcId','name','auxInfo'{{M[1],M[18],T\"xhatOut\",},{M[4],M[0],T\"P\",S'l','i','p'{{M1x2[471 472],M[0],}}},{M[4],M[0],T\"Q\",S'l','i','p'{{M1x2[478 479],M[0],}}},{M[4],M[0],T\"R\",S'l','i','p'{{M1x2[480 481],M[0],}}},{M[4],M[0],T\"prevT\",S'l','i','p'{{M1x2[459 464],M[0],}}},{M[4],M[0],T\"prevY\",S'l','i','p'{{M1x2[465 470],M[0],}}},{M[4],M[0],T\"xhat\",S'l','i','p'{{M1x2[473 477],M[0],}}},{M[8],M[0],T\"is_active_c4_AutoFollow_Simulation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 8, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_AutoFollow_Simulation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _AutoFollow_SimulationMachineNumber_,
           4,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"hx_cam");
          _SFD_SET_DATA_PROPS(1,1,1,0,"hy_cam");
          _SFD_SET_DATA_PROPS(2,1,1,0,"hz_cam");
          _SFD_SET_DATA_PROPS(3,1,1,0,"u");
          _SFD_SET_DATA_PROPS(4,1,1,0,"v");
          _SFD_SET_DATA_PROPS(5,1,1,0,"w");
          _SFD_SET_DATA_PROPS(6,1,1,0,"newMeas");
          _SFD_SET_DATA_PROPS(7,1,1,0,"t");
          _SFD_SET_DATA_PROPS(8,2,0,1,"xhatOut");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,4,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1828);
        _SFD_CV_INIT_EML_IF(0,1,0,539,552,-1,922);
        _SFD_CV_INIT_EML_IF(0,1,1,1484,1616,-1,1794);

        {
          static int condStart[] = { 1488, 1528, 1568, 1609 };

          static int condEnd[] = { 1516, 1556, 1596, 1616 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3, 3, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,1487,1616,4,0,&(condStart[0]),&(condEnd[0]),
                                7,&(pfixExpr[0]));
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_h_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_g_sf_marshallOut,(MexInFcnForType)
            c4_g_sf_marshallIn);
        }

        {
          real_T *c4_hx_cam;
          real_T *c4_hy_cam;
          real_T *c4_hz_cam;
          real_T *c4_u;
          real_T *c4_v;
          real_T *c4_w;
          real_T *c4_newMeas;
          real_T *c4_t;
          real_T (*c4_xhatOut)[6];
          c4_xhatOut = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c4_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c4_newMeas = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c4_w = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c4_v = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c4_u = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c4_hz_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c4_hy_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c4_hx_cam = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c4_hx_cam);
          _SFD_SET_DATA_VALUE_PTR(1U, c4_hy_cam);
          _SFD_SET_DATA_VALUE_PTR(2U, c4_hz_cam);
          _SFD_SET_DATA_VALUE_PTR(3U, c4_u);
          _SFD_SET_DATA_VALUE_PTR(4U, c4_v);
          _SFD_SET_DATA_VALUE_PTR(5U, c4_w);
          _SFD_SET_DATA_VALUE_PTR(6U, c4_newMeas);
          _SFD_SET_DATA_VALUE_PTR(7U, c4_t);
          _SFD_SET_DATA_VALUE_PTR(8U, *c4_xhatOut);
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
  return "ICNT7MWBqGGYeYfCuZ304E";
}

static void sf_opaque_initialize_c4_AutoFollow_Simulation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c4_AutoFollow_Simulation
    ((SFc4_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
  initialize_c4_AutoFollow_Simulation((SFc4_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c4_AutoFollow_Simulation(void *chartInstanceVar)
{
  enable_c4_AutoFollow_Simulation((SFc4_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c4_AutoFollow_Simulation(void *chartInstanceVar)
{
  disable_c4_AutoFollow_Simulation((SFc4_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c4_AutoFollow_Simulation(void *chartInstanceVar)
{
  sf_gateway_c4_AutoFollow_Simulation((SFc4_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_AutoFollow_Simulation
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_AutoFollow_Simulation
    ((SFc4_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_AutoFollow_Simulation();/* state var info */
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

extern void sf_internal_set_sim_state_c4_AutoFollow_Simulation(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c4_AutoFollow_Simulation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_AutoFollow_Simulation
    ((SFc4_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_AutoFollow_Simulation(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c4_AutoFollow_Simulation(S);
}

static void sf_opaque_set_sim_state_c4_AutoFollow_Simulation(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c4_AutoFollow_Simulation(S, st);
}

static void sf_opaque_terminate_c4_AutoFollow_Simulation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_AutoFollow_SimulationInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_AutoFollow_Simulation_optimization_info();
    }

    finalize_c4_AutoFollow_Simulation((SFc4_AutoFollow_SimulationInstanceStruct*)
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
  initSimStructsc4_AutoFollow_Simulation
    ((SFc4_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_AutoFollow_Simulation(SimStruct *S)
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
    initialize_params_c4_AutoFollow_Simulation
      ((SFc4_AutoFollow_SimulationInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_AutoFollow_Simulation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,4,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,4);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,8);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 8; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1640302650U));
  ssSetChecksum1(S,(69152686U));
  ssSetChecksum2(S,(108300981U));
  ssSetChecksum3(S,(1580109002U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c4_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_AutoFollow_Simulation(SimStruct *S)
{
  SFc4_AutoFollow_SimulationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc4_AutoFollow_SimulationInstanceStruct *)utMalloc(sizeof
    (SFc4_AutoFollow_SimulationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_AutoFollow_SimulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_AutoFollow_Simulation;
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

void c4_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_AutoFollow_Simulation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_AutoFollow_Simulation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
