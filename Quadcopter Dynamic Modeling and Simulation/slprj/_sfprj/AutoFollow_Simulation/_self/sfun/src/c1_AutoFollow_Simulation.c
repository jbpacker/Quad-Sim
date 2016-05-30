/* Include files */

#include "AutoFollow_Simulation_sfun.h"
#include "c1_AutoFollow_Simulation.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "AutoFollow_Simulation_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[33] = { "Xmax", "Ymax", "map", "R",
  "C", "Iq", "Jq", "Ih", "Jh", "Pxel", "xCheck", "yCheck", "distCheck",
  "distHuman", "p_not_rot", "p", "nargin", "nargout", "xq", "yq", "zq", "thetaq",
  "psiq", "phiq", "hx", "hy", "hz", "r", "theta", "psi", "r_prev", "theta_prev",
  "psi_prev" };

static const char * c1_b_debug_family_names[10] = { "nargin", "nargout", "x",
  "y", "Xmax", "Ymax", "R", "C", "i", "j" };

static const char * c1_c_debug_family_names[21] = { "R", "C", "x1", "y1", "x2",
  "y2", "x", "xd", "dx", "y", "yd", "dy", "a", "b", "c", "nargin", "nargout",
  "p1", "p2", "map", "p" };

static const char * c1_d_debug_family_names[10] = { "nargin", "nargout", "i",
  "j", "Xmax", "Ymax", "R", "C", "x", "y" };

/* Function Declarations */
static void initialize_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initialize_params_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void enable_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void disable_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void set_sim_state_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c1_st);
static void finalize_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void sf_gateway_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void mdl_start_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c1_chartstep_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initSimStructsc1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c1_XYtoIJ(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x, real_T c1_y, real_T c1_Xmax, real_T c1_Ymax,
                      real_T c1_R, real_T c1_C, real_T *c1_i, real_T *c1_j);
static void c1_laserRange(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c1_p1[2], real_T c1_p2[2], real_T c1_c_map[10000],
  real_T c1_p_data[], int32_T c1_p_sizes[2]);
static void c1_IJtoXY(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_i, real_T c1_j, real_T c1_Xmax, real_T c1_Ymax,
                      real_T c1_R, real_T c1_C, real_T *c1_x, real_T *c1_y);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_b_psi_prev, const char_T *c1_identifier,
  boolean_T *c1_svPtr);
static real_T c1_b_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  boolean_T *c1_svPtr);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_b_psi, const char_T *c1_identifier);
static real_T c1_d_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_e_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[9]);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_f_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[3]);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[], int32_T c1_inData_sizes[2]);
static void c1_g_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[], int32_T c1_y_sizes[2]);
static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[], int32_T
  c1_outData_sizes[2]);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[], int32_T c1_inData_sizes[2]);
static void c1_h_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[], int32_T c1_y_sizes[2]);
static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[], int32_T
  c1_outData_sizes[2]);
static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_i_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[2]);
static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static boolean_T c1_all(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  boolean_T c1_x[2]);
static real_T c1_mpower(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c1_a);
static void c1_scalarEg(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c1_dimagree(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c1_error(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static real_T c1_sqrt(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x);
static void c1_b_error(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static real_T c1_cos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x);
static real_T c1_sin(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x);
static void c1_b_mpower(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c1_a[9], real_T c1_c[9]);
static void c1_matrix_to_integer_power(SFc1_AutoFollow_SimulationInstanceStruct *
  chartInstance, real_T c1_a[9], real_T c1_c[9]);
static real_T c1_abs(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x);
static void c1_inv3x3(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x[9], real_T c1_y[9]);
static real_T c1_norm(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x[9]);
static void c1_warning(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c1_b_warning(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  char_T c1_varargin_1[14]);
static void c1_b_scalarEg(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c1_c_scalarEg(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c1_power(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_a[3], real_T c1_y[3]);
static void c1_b_dimagree(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static real_T c1_sum(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x[3]);
static real_T c1_atan2(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c1_y, real_T c1_x);
static real_T c1_acos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x);
static void c1_c_error(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *c1_emlrt_marshallOut
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance, const char * c1_u);
static void c1_j_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_load, const char_T *c1_identifier,
  c1_s2aqkGCuE38RBomNVWBcX1B *c1_y);
static void c1_k_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  c1_s2aqkGCuE38RBomNVWBcX1B *c1_y);
static void c1_l_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[10000]);
static void c1_m_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_sprintf, const char_T *c1_identifier, char_T
  c1_y[14]);
static void c1_n_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  char_T c1_y[14]);
static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_o_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_p_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_AutoFollow_Simulation, const
  char_T *c1_identifier);
static uint8_T c1_q_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sqrt(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T *c1_x);
static void c1_b_cos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c1_x);
static void c1_b_sin(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c1_x);
static void c1_b_acos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T *c1_x);
static void init_dsm_address_info(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_AutoFollow_Simulation(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_r_prev_not_empty = false;
  chartInstance->c1_theta_prev_not_empty = false;
  chartInstance->c1_psi_prev_not_empty = false;
  chartInstance->c1_is_active_c1_AutoFollow_Simulation = 0U;
}

static void initialize_params_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  uint8_T c1_g_hoistedGlobal;
  uint8_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(7, 1), false);
  c1_hoistedGlobal = *chartInstance->c1_psi;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *chartInstance->c1_r;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *chartInstance->c1_theta;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = chartInstance->c1_psi_prev;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  if (!chartInstance->c1_psi_prev_not_empty) {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = chartInstance->c1_r_prev;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  if (!chartInstance->c1_psi_prev_not_empty) {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = chartInstance->c1_theta_prev;
  c1_f_u = c1_f_hoistedGlobal;
  c1_g_y = NULL;
  if (!chartInstance->c1_psi_prev_not_empty) {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = chartInstance->c1_is_active_c1_AutoFollow_Simulation;
  c1_g_u = c1_g_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  *chartInstance->c1_psi = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("psi", c1_u, 0)), "psi");
  *chartInstance->c1_r = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("r", c1_u, 1)), "r");
  *chartInstance->c1_theta = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("theta", c1_u, 2)), "theta");
  chartInstance->c1_psi_prev = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("psi_prev", c1_u, 3)), "psi_prev",
    &chartInstance->c1_psi_prev_not_empty);
  chartInstance->c1_r_prev = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("r_prev", c1_u, 4)), "r_prev",
    &chartInstance->c1_r_prev_not_empty);
  chartInstance->c1_theta_prev = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("theta_prev", c1_u, 5)), "theta_prev",
    &chartInstance->c1_theta_prev_not_empty);
  chartInstance->c1_is_active_c1_AutoFollow_Simulation = c1_p_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(
       "is_active_c1_AutoFollow_Simulation", c1_u, 6)),
     "is_active_c1_AutoFollow_Simulation");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_AutoFollow_Simulation(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_hz, 8U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_hy, 7U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_hx, 6U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_phiq, 5U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_psiq, 4U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_thetaq, 3U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_zq, 2U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_yq, 1U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_xq, 0U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_AutoFollow_Simulation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_AutoFollow_SimulationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_r, 9U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_theta, 10U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_psi, 11U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
}

static void mdl_start_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_chartstep_c1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_e_hoistedGlobal;
  real_T c1_f_hoistedGlobal;
  real_T c1_g_hoistedGlobal;
  real_T c1_h_hoistedGlobal;
  real_T c1_i_hoistedGlobal;
  real_T c1_b_xq;
  real_T c1_b_yq;
  real_T c1_b_zq;
  real_T c1_b_thetaq;
  real_T c1_b_psiq;
  real_T c1_b_phiq;
  real_T c1_b_hx;
  real_T c1_b_hy;
  real_T c1_b_hz;
  uint32_T c1_debug_family_var_map[33];
  real_T c1_Xmax;
  real_T c1_Ymax;
  real_T c1_R;
  real_T c1_C;
  real_T c1_Iq;
  real_T c1_Jq;
  real_T c1_Ih;
  real_T c1_Jh;
  real_T c1_Pxel_data[2];
  int32_T c1_Pxel_sizes[2];
  real_T c1_xCheck;
  real_T c1_yCheck;
  real_T c1_distCheck;
  real_T c1_distHuman;
  real_T c1_p_not_rot[3];
  real_T c1_p[3];
  real_T c1_b_R[9];
  real_T c1_nargin = 9.0;
  real_T c1_nargout = 3.0;
  real_T c1_b_r;
  real_T c1_b_theta;
  real_T c1_b_psi;
  int32_T c1_i0;
  real_T c1_b_Iq;
  real_T c1_b_Jq;
  real_T c1_b_Ih;
  real_T c1_b_Jh;
  real_T c1_c_Iq[2];
  real_T c1_c_Ih[2];
  int32_T c1_i1;
  real_T c1_c_map[10000];
  real_T c1_tmp_data[2];
  int32_T c1_tmp_sizes[2];
  int32_T c1_Pxel;
  int32_T c1_b_Pxel;
  int32_T c1_loop_ub;
  int32_T c1_i2;
  real_T c1_b_xCheck;
  real_T c1_b_yCheck;
  real_T c1_d0;
  real_T c1_d1;
  real_T c1_d2;
  real_T c1_d3;
  real_T c1_d4;
  real_T c1_d5;
  real_T c1_d6;
  real_T c1_d7;
  real_T c1_d8;
  real_T c1_d9;
  real_T c1_d10;
  real_T c1_d11;
  real_T c1_d12;
  real_T c1_d13;
  real_T c1_d14;
  real_T c1_d15;
  real_T c1_d16;
  real_T c1_d17;
  real_T c1_d18;
  real_T c1_d19;
  real_T c1_d20;
  real_T c1_d21;
  real_T c1_d22;
  real_T c1_d23;
  real_T c1_d24;
  real_T c1_d25;
  real_T c1_d26;
  real_T c1_d27;
  real_T c1_d28;
  real_T c1_d29;
  int32_T c1_i3;
  real_T c1_c_R[9];
  real_T c1_a[9];
  int32_T c1_i4;
  int32_T c1_i5;
  real_T c1_b[3];
  int32_T c1_i6;
  int32_T c1_i7;
  int32_T c1_i8;
  real_T c1_b_C[3];
  int32_T c1_i9;
  int32_T c1_i10;
  int32_T c1_i11;
  int32_T c1_i12;
  real_T c1_b_p[3];
  real_T c1_A;
  real_T c1_B;
  real_T c1_x;
  real_T c1_y;
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_c_y;
  boolean_T guard1 = false;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *chartInstance->c1_xq;
  c1_b_hoistedGlobal = *chartInstance->c1_yq;
  c1_c_hoistedGlobal = *chartInstance->c1_zq;
  c1_d_hoistedGlobal = *chartInstance->c1_thetaq;
  c1_e_hoistedGlobal = *chartInstance->c1_psiq;
  c1_f_hoistedGlobal = *chartInstance->c1_phiq;
  c1_g_hoistedGlobal = *chartInstance->c1_hx;
  c1_h_hoistedGlobal = *chartInstance->c1_hy;
  c1_i_hoistedGlobal = *chartInstance->c1_hz;
  c1_b_xq = c1_hoistedGlobal;
  c1_b_yq = c1_b_hoistedGlobal;
  c1_b_zq = c1_c_hoistedGlobal;
  c1_b_thetaq = c1_d_hoistedGlobal;
  c1_b_psiq = c1_e_hoistedGlobal;
  c1_b_phiq = c1_f_hoistedGlobal;
  c1_b_hx = c1_g_hoistedGlobal;
  c1_b_hy = c1_h_hoistedGlobal;
  c1_b_hz = c1_i_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 33U, 35U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Xmax, 0U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Ymax, 1U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_map, MAX_uint32_T,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_R, MAX_uint32_T, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_C, 4U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Iq, 5U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Jq, 6U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Ih, 7U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Jh, 8U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c1_Pxel_data, (const int32_T *)
    &c1_Pxel_sizes, NULL, 0, 9, (void *)c1_f_sf_marshallOut, (void *)
    c1_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_xCheck, 10U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_yCheck, 11U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_distCheck, 12U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_distHuman, 13U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_p_not_rot, 14U, c1_e_sf_marshallOut,
    c1_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_p, 15U, c1_e_sf_marshallOut,
    c1_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_b_map, MAX_uint32_T,
    c1_d_sf_marshallOut, c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_R, MAX_uint32_T, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 16U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 17U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_xq, 18U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_yq, 19U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_zq, 20U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_thetaq, 21U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_psiq, 22U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_phiq, 23U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_hx, 24U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_hy, 25U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_hz, 26U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_r, 27U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_theta, 28U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_psi, 29U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_r_prev, 30U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_theta_prev, 31U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_psi_prev, 32U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 2);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  c1_Xmax = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  c1_Ymax = 25.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
  _SFD_SYMBOL_SWITCH(2U, 2U);
  c1_j_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                        (sfGlobalDebugInstanceStruct, "load", 1U, 1U, 14,
    c1_emlrt_marshallOut(chartInstance, "./Lidar/ObstacleMap/map.mat")), "load",
                        &chartInstance->c1_r0);
  chartInstance->c1_map = chartInstance->c1_r0;
  _SFD_SYMBOL_SWITCH(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
  for (c1_i0 = 0; c1_i0 < 10000; c1_i0++) {
    chartInstance->c1_b_map[c1_i0] = chartInstance->c1_map.map[c1_i0];
  }

  _SFD_SYMBOL_SWITCH(2U, 16U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
  c1_R = 100.0;
  _SFD_SYMBOL_SWITCH(3U, 3U);
  c1_C = 100.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
  c1_XYtoIJ(chartInstance, c1_b_xq * 3.28, c1_b_yq * 3.28, 25.0, 25.0, 100.0,
            100.0, &c1_b_Iq, &c1_b_Jq);
  c1_Iq = c1_b_Iq;
  c1_Jq = c1_b_Jq;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  c1_XYtoIJ(chartInstance, c1_b_hx, c1_b_hy, 25.0, 25.0, 100.0, 100.0, &c1_b_Ih,
            &c1_b_Jh);
  c1_Ih = c1_b_Ih;
  c1_Jh = c1_b_Jh;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
  c1_c_Iq[0] = c1_Iq;
  c1_c_Iq[1] = c1_Jq;
  c1_c_Ih[0] = c1_Ih;
  c1_c_Ih[1] = c1_Jh;
  for (c1_i1 = 0; c1_i1 < 10000; c1_i1++) {
    c1_c_map[c1_i1] = chartInstance->c1_b_map[c1_i1];
  }

  c1_laserRange(chartInstance, c1_c_Iq, c1_c_Ih, c1_c_map, c1_tmp_data,
                c1_tmp_sizes);
  c1_Pxel_sizes[0] = c1_tmp_sizes[0];
  c1_Pxel_sizes[1] = c1_tmp_sizes[1];
  c1_Pxel = c1_Pxel_sizes[0];
  c1_b_Pxel = c1_Pxel_sizes[1];
  c1_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i2 = 0; c1_i2 <= c1_loop_ub; c1_i2++) {
    c1_Pxel_data[c1_i2] = c1_tmp_data[c1_i2];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 14);
  c1_IJtoXY(chartInstance, c1_Pxel_data[0], c1_Pxel_data[1], 25.0, 25.0, 100.0,
            100.0, &c1_b_xCheck, &c1_b_yCheck);
  c1_xCheck = c1_b_xCheck;
  c1_yCheck = c1_b_yCheck;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  c1_d0 = c1_mpower(chartInstance, c1_b_xq * 3.28 - c1_xCheck) + c1_mpower
    (chartInstance, c1_b_yq * 3.28 - c1_yCheck);
  c1_b_sqrt(chartInstance, &c1_d0);
  c1_distCheck = c1_d0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 16);
  c1_d0 = c1_mpower(chartInstance, c1_b_xq * 3.28 - c1_b_hx) + c1_mpower
    (chartInstance, c1_b_yq * 3.28 - c1_b_hy);
  c1_b_sqrt(chartInstance, &c1_d0);
  c1_distHuman = c1_d0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
  guard1 = false;
  if (CV_EML_COND(0, 1, 0, !chartInstance->c1_r_prev_not_empty)) {
    guard1 = true;
  } else if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 0, c1_distHuman,
               c1_distCheck, -1, 2U, c1_distHuman < c1_distCheck))) {
    guard1 = true;
  } else {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 0, false);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 61);
    c1_b_r = chartInstance->c1_r_prev;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 62);
    c1_b_theta = chartInstance->c1_theta_prev;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 63);
    c1_b_psi = chartInstance->c1_psi_prev;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, true);
    CV_EML_IF(0, 1, 0, true);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
    c1_p_not_rot[0] = c1_b_xq * 3.28 - c1_b_hx;
    c1_p_not_rot[1] = c1_b_yq * 3.28 - c1_b_hy;
    c1_p_not_rot[2] = c1_b_zq * 3.28 - c1_b_hz;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
    c1_d1 = c1_b_psiq;
    c1_b_cos(chartInstance, &c1_d1);
    c1_d2 = c1_b_thetaq;
    c1_b_cos(chartInstance, &c1_d2);
    c1_d3 = c1_b_psiq;
    c1_b_cos(chartInstance, &c1_d3);
    c1_d4 = c1_b_thetaq;
    c1_b_sin(chartInstance, &c1_d4);
    c1_d5 = c1_b_phiq;
    c1_b_sin(chartInstance, &c1_d5);
    c1_d6 = c1_b_psiq;
    c1_b_sin(chartInstance, &c1_d6);
    c1_d7 = c1_b_phiq;
    c1_b_cos(chartInstance, &c1_d7);
    c1_d8 = c1_b_psiq;
    c1_b_cos(chartInstance, &c1_d8);
    c1_d9 = c1_b_thetaq;
    c1_b_sin(chartInstance, &c1_d9);
    c1_d10 = c1_b_phiq;
    c1_b_cos(chartInstance, &c1_d10);
    c1_d11 = c1_b_psiq;
    c1_b_sin(chartInstance, &c1_d11);
    c1_d12 = c1_b_phiq;
    c1_b_sin(chartInstance, &c1_d12);
    c1_d13 = c1_b_psiq;
    c1_b_sin(chartInstance, &c1_d13);
    c1_d14 = c1_b_thetaq;
    c1_b_cos(chartInstance, &c1_d14);
    c1_d15 = c1_b_psiq;
    c1_b_sin(chartInstance, &c1_d15);
    c1_d16 = c1_b_thetaq;
    c1_b_sin(chartInstance, &c1_d16);
    c1_d17 = c1_b_phiq;
    c1_b_sin(chartInstance, &c1_d17);
    c1_d18 = c1_b_psiq;
    c1_b_cos(chartInstance, &c1_d18);
    c1_d19 = c1_b_phiq;
    c1_b_cos(chartInstance, &c1_d19);
    c1_d20 = c1_b_psiq;
    c1_b_sin(chartInstance, &c1_d20);
    c1_d21 = c1_b_thetaq;
    c1_b_sin(chartInstance, &c1_d21);
    c1_d22 = c1_b_phiq;
    c1_b_cos(chartInstance, &c1_d22);
    c1_d23 = c1_b_psiq;
    c1_b_cos(chartInstance, &c1_d23);
    c1_d24 = c1_b_phiq;
    c1_b_sin(chartInstance, &c1_d24);
    c1_d25 = c1_b_thetaq;
    c1_b_sin(chartInstance, &c1_d25);
    c1_d26 = c1_b_thetaq;
    c1_b_cos(chartInstance, &c1_d26);
    c1_d27 = c1_b_phiq;
    c1_b_sin(chartInstance, &c1_d27);
    c1_d28 = c1_b_thetaq;
    c1_b_cos(chartInstance, &c1_d28);
    c1_d29 = c1_b_phiq;
    c1_b_cos(chartInstance, &c1_d29);
    c1_b_R[0] = c1_d1 * c1_d2;
    c1_b_R[3] = c1_d3 * c1_d4 * c1_d5 - c1_d6 * c1_d7;
    c1_b_R[6] = c1_d8 * c1_d9 * c1_d10 + c1_d11 * c1_d12;
    c1_b_R[1] = c1_d13 * c1_d14;
    c1_b_R[4] = c1_d15 * c1_d16 * c1_d17 + c1_d18 * c1_d19;
    c1_b_R[7] = c1_d20 * c1_d21 * c1_d22 - c1_d23 * c1_d24;
    c1_b_R[2] = -c1_d25;
    c1_b_R[5] = c1_d26 * c1_d27;
    c1_b_R[8] = c1_d28 * c1_d29;
    _SFD_SYMBOL_SWITCH(3U, 17U);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
    for (c1_i3 = 0; c1_i3 < 9; c1_i3++) {
      c1_c_R[c1_i3] = c1_b_R[c1_i3];
    }

    c1_b_mpower(chartInstance, c1_c_R, c1_a);
    for (c1_i4 = 0; c1_i4 < 3; c1_i4++) {
      c1_b[c1_i4] = c1_p_not_rot[c1_i4];
    }

    for (c1_i5 = 0; c1_i5 < 3; c1_i5++) {
      c1_p[c1_i5] = 0.0;
    }

    for (c1_i6 = 0; c1_i6 < 3; c1_i6++) {
      c1_p[c1_i6] = 0.0;
    }

    for (c1_i7 = 0; c1_i7 < 3; c1_i7++) {
      c1_b_C[c1_i7] = c1_p[c1_i7];
    }

    for (c1_i8 = 0; c1_i8 < 3; c1_i8++) {
      c1_p[c1_i8] = c1_b_C[c1_i8];
    }

    for (c1_i9 = 0; c1_i9 < 3; c1_i9++) {
      c1_p[c1_i9] = 0.0;
      c1_i11 = 0;
      for (c1_i12 = 0; c1_i12 < 3; c1_i12++) {
        c1_p[c1_i9] += c1_a[c1_i11 + c1_i9] * c1_b[c1_i12];
        c1_i11 += 3;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
    for (c1_i10 = 0; c1_i10 < 3; c1_i10++) {
      c1_b_p[c1_i10] = c1_p[c1_i10];
    }

    c1_power(chartInstance, c1_b_p, c1_b);
    c1_d0 = c1_sum(chartInstance, c1_b);
    c1_b_sqrt(chartInstance, &c1_d0);
    c1_b_r = c1_d0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
    c1_b_theta = c1_atan2(chartInstance, c1_p[1], c1_p[0]);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 40);
    c1_A = c1_p[2];
    c1_B = c1_b_r;
    c1_x = c1_A;
    c1_y = c1_B;
    c1_b_x = c1_x;
    c1_b_y = c1_y;
    c1_c_y = c1_b_x / c1_b_y;
    c1_d0 = c1_c_y;
    c1_b_acos(chartInstance, &c1_d0);
    c1_b_psi = c1_d0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
    chartInstance->c1_r_prev = c1_b_r;
    chartInstance->c1_r_prev_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
    chartInstance->c1_theta_prev = c1_b_theta;
    chartInstance->c1_theta_prev_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
    chartInstance->c1_psi_prev = c1_b_psi;
    chartInstance->c1_psi_prev_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -63);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c1_r = c1_b_r;
  *chartInstance->c1_theta = c1_b_theta;
  *chartInstance->c1_psi = c1_b_psi;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_AutoFollow_Simulation
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_XYtoIJ(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x, real_T c1_y, real_T c1_Xmax, real_T c1_Ymax,
                      real_T c1_R, real_T c1_C, real_T *c1_i, real_T *c1_j)
{
  uint32_T c1_debug_family_var_map[10];
  real_T c1_nargin = 6.0;
  real_T c1_nargout = 2.0;
  real_T c1_A;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T c1_b_y;
  real_T c1_d_x;
  real_T c1_e_x;
  real_T c1_b_A;
  real_T c1_f_x;
  real_T c1_g_x;
  real_T c1_c_y;
  real_T c1_h_x;
  real_T c1_i_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y, 3U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Xmax, 4U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Ymax, 5U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_R, 6U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_C, 7U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_i, 8U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_j, 9U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 4);
  c1_A = 25.0 - c1_y;
  c1_b_x = c1_A;
  c1_c_x = c1_b_x;
  c1_b_y = c1_c_x / 25.0;
  c1_d_x = c1_b_y * 99.0;
  c1_e_x = c1_d_x;
  c1_e_x = muDoubleScalarRound(c1_e_x);
  *c1_i = c1_e_x + 1.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, 5);
  c1_b_A = c1_x;
  c1_f_x = c1_b_A;
  c1_g_x = c1_f_x;
  c1_c_y = c1_g_x / 25.0;
  c1_h_x = c1_c_y * 99.0;
  c1_i_x = c1_h_x;
  c1_i_x = muDoubleScalarRound(c1_i_x);
  *c1_j = c1_i_x + 1.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c1_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c1_laserRange(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c1_p1[2], real_T c1_p2[2], real_T c1_c_map[10000],
  real_T c1_p_data[], int32_T c1_p_sizes[2])
{
  uint32_T c1_debug_family_var_map[21];
  real_T c1_R;
  real_T c1_C;
  real_T c1_x1;
  real_T c1_y1;
  real_T c1_x2;
  real_T c1_y2;
  real_T c1_x;
  real_T c1_xd;
  real_T c1_dx;
  real_T c1_y;
  real_T c1_yd;
  real_T c1_dy;
  real_T c1_a;
  real_T c1_b;
  real_T c1_c;
  real_T c1_nargin = 3.0;
  real_T c1_nargout = 1.0;
  int32_T c1_p;
  int32_T c1_b_p;
  real_T c1_d30;
  real_T c1_d31;
  int32_T c1_c_p;
  int32_T c1_d_p;
  real_T c1_b_x[2];
  real_T c1_c_x[2];
  int32_T c1_e_p;
  real_T c1_d_x[2];
  int32_T c1_f_p;
  int32_T c1_i13;
  int32_T c1_i14;
  int32_T c1_i15;
  int32_T c1_i16;
  int32_T c1_g_p;
  int32_T c1_h_p;
  boolean_T c1_e_x[2];
  int32_T c1_i_p;
  boolean_T c1_f_x[2];
  int32_T c1_j_p;
  int32_T c1_i17;
  int32_T c1_i18;
  int32_T c1_k_p;
  int32_T c1_l_p;
  int32_T c1_m_p;
  int32_T c1_n_p;
  int32_T c1_i19;
  int32_T c1_i20;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  int32_T exitg1;
  int32_T exitg2;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 21U, 21U, c1_c_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_R, 0U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_C, 1U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x1, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y1, 3U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x2, 4U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y2, 5U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x, 6U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_xd, 7U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_dx, 8U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y, 9U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_yd, 10U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_dy, 11U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_a, 12U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b, 13U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c, 14U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 15U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 16U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_p1, 17U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_p2, 18U, c1_i_sf_marshallOut,
    c1_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_c_map, 19U, c1_d_sf_marshallOut,
    c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c1_p_data, (const int32_T *)
    c1_p_sizes, NULL, 0, 20, (void *)c1_h_sf_marshallOut, (void *)
    c1_h_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 24);
  c1_R = 100.0;
  c1_C = 100.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 26);
  c1_x1 = c1_p1[0];
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 26);
  c1_y1 = c1_p1[1];
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 27);
  c1_x2 = c1_p2[0];
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 27);
  c1_y2 = c1_p2[1];
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 30);
  c1_x = c1_x1;
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 31);
  if (CV_SCRIPT_IF(1, 0, CV_RELATIONAL_EVAL(14U, 1U, 0, c1_x2, c1_x1, -1, 4U,
        c1_x2 > c1_x1))) {
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 32);
    c1_xd = c1_x2 - c1_x1;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 33);
    c1_dx = 1.0;
  } else {
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 35);
    c1_xd = c1_x1 - c1_x2;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 36);
    c1_dx = -1.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 39);
  c1_y = c1_y1;
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 40);
  if (CV_SCRIPT_IF(1, 1, CV_RELATIONAL_EVAL(14U, 1U, 1, c1_y2, c1_y1, -1, 4U,
        c1_y2 > c1_y1))) {
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 41);
    c1_yd = c1_y2 - c1_y1;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 42);
    c1_dy = 1.0;
  } else {
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 44);
    c1_yd = c1_y1 - c1_y2;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 45);
    c1_dy = -1.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 48);
  c1_p_sizes[0] = 0;
  c1_p_sizes[1] = 0;
  c1_p = c1_p_sizes[0];
  c1_b_p = c1_p_sizes[1];
  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 50);
  if (CV_SCRIPT_IF(1, 2, CV_RELATIONAL_EVAL(14U, 1U, 2, c1_xd, c1_yd, -1, 4U,
        c1_xd > c1_yd))) {
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 51);
    c1_a = 2.0 * c1_yd;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 52);
    c1_b = c1_a - c1_xd;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 53);
    c1_c = c1_b - c1_xd;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 55);
    guard4 = false;
    guard5 = false;
    guard6 = false;
    do {
      exitg2 = 0;
      CV_SCRIPT_WHILE(1, 0, true);
      _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 57);
      if (CV_SCRIPT_COND(1, 0, CV_RELATIONAL_EVAL(14U, 1U, 3, c1_x, 0.0, -1, 2U,
            c1_x < 0.0))) {
        guard6 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(1, 1, CV_RELATIONAL_EVAL(14U, 1U, 4, c1_y, 0.0,
                   -1, 2U, c1_y < 0.0))) {
        guard6 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(1, 2, CV_RELATIONAL_EVAL(14U, 1U, 5, c1_x, 100.0,
        -1, 4U, c1_x > 100.0))) {
        guard5 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(1, 3, CV_RELATIONAL_EVAL(14U, 1U, 6, c1_y, 100.0,
        -1, 4U, c1_y > 100.0))) {
        guard4 = true;
        exitg2 = 1;
      } else {
        CV_SCRIPT_MCDC(1, 0, false);
        CV_SCRIPT_IF(1, 3, false);
        _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 61);
        c1_d31 = c1_c_map[(sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 0U, 0U, 0U, c1_x), 1, 100) + 100 *
                           (sf_eml_array_bounds_check
                            (sfGlobalDebugInstanceStruct, chartInstance->S, 0U,
                             0, 0, MAX_uint32_T, (int32_T)sf_integer_check
                             (chartInstance->S, 0U, 0U, 0U, c1_y), 1, 100) - 1))
          - 1];
        if (CV_SCRIPT_IF(1, 4, CV_RELATIONAL_EVAL(14U, 1U, 7, c1_d31, 1.0, -1,
              0U, c1_d31 == 1.0))) {
          _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 62);
          c1_c_x[0] = c1_x;
          c1_c_x[1] = c1_y;
          c1_p_sizes[0] = 1;
          c1_p_sizes[1] = 2;
          c1_h_p = c1_p_sizes[0];
          c1_j_p = c1_p_sizes[1];
          for (c1_i18 = 0; c1_i18 < 2; c1_i18++) {
            c1_p_data[c1_i18] = c1_c_x[c1_i18];
          }

          _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 63);
          exitg2 = 1;
        } else {
          _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 65);
          c1_d_x[0] = c1_x - c1_x2;
          c1_d_x[1] = c1_y - c1_y2;
          for (c1_i16 = 0; c1_i16 < 2; c1_i16++) {
            c1_f_x[c1_i16] = (c1_d_x[c1_i16] == 0.0);
          }

          if (CV_SCRIPT_IF(1, 5, c1_all(chartInstance, c1_f_x))) {
            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 66);
            c1_p_sizes[0] = 1;
            c1_p_sizes[1] = 2;
            c1_l_p = c1_p_sizes[0];
            c1_n_p = c1_p_sizes[1];
            for (c1_i20 = 0; c1_i20 < 2; c1_i20++) {
              c1_p_data[c1_i20] = rtInf;
            }

            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 67);
            exitg2 = 1;
          } else {
            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 69);
            if (CV_SCRIPT_IF(1, 6, CV_RELATIONAL_EVAL(14U, 1U, 8, c1_b, 0.0, -1,
                  2U, c1_b < 0.0))) {
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 70);
              c1_b += c1_a;
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 71);
              c1_x += c1_dx;
            } else {
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 73);
              c1_b += c1_c;
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 74);
              c1_x += c1_dx;
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 74);
              c1_y += c1_dy;
            }

            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 55);
            _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
            guard4 = false;
            guard5 = false;
            guard6 = false;
          }
        }
      }
    } while (exitg2 == 0);

    if (guard6 == true) {
      guard5 = true;
    }

    if (guard5 == true) {
      guard4 = true;
    }

    if (guard4 == true) {
      CV_SCRIPT_MCDC(1, 0, true);
      CV_SCRIPT_IF(1, 3, true);
      _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 58);
      c1_p_sizes[0] = 1;
      c1_p_sizes[1] = 2;
      c1_d_p = c1_p_sizes[0];
      c1_f_p = c1_p_sizes[1];
      for (c1_i14 = 0; c1_i14 < 2; c1_i14++) {
        c1_p_data[c1_i14] = rtInf;
      }

      _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 59);
    }
  } else {
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 78);
    c1_a = 2.0 * c1_xd;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 79);
    c1_b = c1_a - c1_yd;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 80);
    c1_c = c1_b - c1_yd;
    _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 82);
    guard1 = false;
    guard2 = false;
    guard3 = false;
    do {
      exitg1 = 0;
      CV_SCRIPT_WHILE(1, 1, true);
      _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 84);
      if (CV_SCRIPT_COND(1, 4, CV_RELATIONAL_EVAL(14U, 1U, 9, c1_x, 0.0, -1, 2U,
            c1_x < 0.0))) {
        guard3 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(1, 5, CV_RELATIONAL_EVAL(14U, 1U, 10, c1_y, 0.0,
        -1, 2U, c1_y < 0.0))) {
        guard3 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(1, 6, CV_RELATIONAL_EVAL(14U, 1U, 11, c1_x,
                   100.0, -1, 4U, c1_x > 100.0))) {
        guard2 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(1, 7, CV_RELATIONAL_EVAL(14U, 1U, 12, c1_y,
                   100.0, -1, 4U, c1_y > 100.0))) {
        guard1 = true;
        exitg1 = 1;
      } else {
        CV_SCRIPT_MCDC(1, 1, false);
        CV_SCRIPT_IF(1, 7, false);
        _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 88);
        c1_d30 = c1_c_map[(sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 0U, 0U, 0U, c1_x), 1, 100) + 100 *
                           (sf_eml_array_bounds_check
                            (sfGlobalDebugInstanceStruct, chartInstance->S, 0U,
                             0, 0, MAX_uint32_T, (int32_T)sf_integer_check
                             (chartInstance->S, 0U, 0U, 0U, c1_y), 1, 100) - 1))
          - 1];
        if (CV_SCRIPT_IF(1, 8, CV_RELATIONAL_EVAL(14U, 1U, 13, c1_d30, 1.0, -1,
              0U, c1_d30 == 1.0))) {
          _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 89);
          c1_c_x[0] = c1_x;
          c1_c_x[1] = c1_y;
          c1_p_sizes[0] = 1;
          c1_p_sizes[1] = 2;
          c1_g_p = c1_p_sizes[0];
          c1_i_p = c1_p_sizes[1];
          for (c1_i17 = 0; c1_i17 < 2; c1_i17++) {
            c1_p_data[c1_i17] = c1_c_x[c1_i17];
          }

          _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 90);
          exitg1 = 1;
        } else {
          _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 92);
          c1_b_x[0] = c1_x - c1_x2;
          c1_b_x[1] = c1_y - c1_y2;
          for (c1_i15 = 0; c1_i15 < 2; c1_i15++) {
            c1_e_x[c1_i15] = (c1_b_x[c1_i15] == 0.0);
          }

          if (CV_SCRIPT_IF(1, 9, c1_all(chartInstance, c1_e_x))) {
            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 93);
            c1_p_sizes[0] = 1;
            c1_p_sizes[1] = 2;
            c1_k_p = c1_p_sizes[0];
            c1_m_p = c1_p_sizes[1];
            for (c1_i19 = 0; c1_i19 < 2; c1_i19++) {
              c1_p_data[c1_i19] = rtInf;
            }

            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 94);
            exitg1 = 1;
          } else {
            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 96);
            if (CV_SCRIPT_IF(1, 10, CV_RELATIONAL_EVAL(14U, 1U, 14, c1_b, 0.0,
                  -1, 2U, c1_b < 0.0))) {
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 97);
              c1_b += c1_a;
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 98);
              c1_y += c1_dy;
            } else {
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 100);
              c1_b += c1_c;
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 101);
              c1_x += c1_dx;
              _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 101);
              c1_y += c1_dy;
            }

            _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 82);
            _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
            guard1 = false;
            guard2 = false;
            guard3 = false;
          }
        }
      }
    } while (exitg1 == 0);

    if (guard3 == true) {
      guard2 = true;
    }

    if (guard2 == true) {
      guard1 = true;
    }

    if (guard1 == true) {
      CV_SCRIPT_MCDC(1, 1, true);
      CV_SCRIPT_IF(1, 7, true);
      _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 85);
      c1_p_sizes[0] = 1;
      c1_p_sizes[1] = 2;
      c1_c_p = c1_p_sizes[0];
      c1_e_p = c1_p_sizes[1];
      for (c1_i13 = 0; c1_i13 < 2; c1_i13++) {
        c1_p_data[c1_i13] = rtInf;
      }

      _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, 86);
    }
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c1_sfEvent, -101);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c1_IJtoXY(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_i, real_T c1_j, real_T c1_Xmax, real_T c1_Ymax,
                      real_T c1_R, real_T c1_C, real_T *c1_x, real_T *c1_y)
{
  uint32_T c1_debug_family_var_map[10];
  real_T c1_nargin = 6.0;
  real_T c1_nargout = 2.0;
  real_T c1_A;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T c1_b_y;
  real_T c1_b_A;
  real_T c1_d_x;
  real_T c1_e_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c1_d_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j, 3U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Xmax, 4U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Ymax, 5U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_R, 6U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_C, 7U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_x, 8U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_y, 9U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c1_sfEvent, 3);
  c1_A = 25.0 * (c1_i - 1.0);
  c1_b_x = c1_A;
  c1_c_x = c1_b_x;
  c1_b_y = c1_c_x / 99.0;
  *c1_y = 25.0 - c1_b_y;
  _SFD_SCRIPT_CALL(2U, chartInstance->c1_sfEvent, 4);
  c1_b_A = 25.0 * (c1_j - 1.0);
  c1_d_x = c1_b_A;
  c1_e_x = c1_d_x;
  *c1_x = c1_e_x / 99.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c1_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c1_chartNumber, c1_instanceNumber, 0U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/XYtoIJ.m"));
  _SFD_SCRIPT_TRANSLATION(c1_chartNumber, c1_instanceNumber, 1U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/common/laserRange.m"));
  _SFD_SCRIPT_TRANSLATION(c1_chartNumber, c1_instanceNumber, 2U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/IJtoXY.m"));
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_psi_prev_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_b_psi_prev, const char_T *c1_identifier,
  boolean_T *c1_svPtr)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_psi_prev),
    &c1_thisId, c1_svPtr);
  sf_mex_destroy(&c1_b_psi_prev);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  boolean_T *c1_svPtr)
{
  real_T c1_y;
  real_T c1_d32;
  (void)chartInstance;
  if (mxIsEmpty(c1_u)) {
    *c1_svPtr = false;
  } else {
    *c1_svPtr = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d32, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d32;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_psi_prev;
  const char_T *c1_identifier;
  boolean_T *c1_svPtr;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_b_psi_prev = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_svPtr = &chartInstance->c1_psi_prev_not_empty;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_psi_prev),
    &c1_thisId, c1_svPtr);
  sf_mex_destroy(&c1_b_psi_prev);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_b_psi, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_psi), &c1_thisId);
  sf_mex_destroy(&c1_b_psi);
  return c1_y;
}

static real_T c1_d_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d33;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d33, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d33;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_psi;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_b_psi = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_psi), &c1_thisId);
  sf_mex_destroy(&c1_b_psi);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i21;
  int32_T c1_i22;
  const mxArray *c1_y = NULL;
  int32_T c1_i23;
  real_T c1_u[9];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i21 = 0;
  for (c1_i22 = 0; c1_i22 < 3; c1_i22++) {
    for (c1_i23 = 0; c1_i23 < 3; c1_i23++) {
      c1_u[c1_i23 + c1_i21] = (*(real_T (*)[9])c1_inData)[c1_i23 + c1_i21];
    }

    c1_i21 += 3;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_e_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[9])
{
  real_T c1_dv0[9];
  int32_T c1_i24;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv0, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c1_i24 = 0; c1_i24 < 9; c1_i24++) {
    c1_y[c1_i24] = c1_dv0[c1_i24];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_R;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[9];
  int32_T c1_i25;
  int32_T c1_i26;
  int32_T c1_i27;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_R = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_R), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_R);
  c1_i25 = 0;
  for (c1_i26 = 0; c1_i26 < 3; c1_i26++) {
    for (c1_i27 = 0; c1_i27 < 3; c1_i27++) {
      (*(real_T (*)[9])c1_outData)[c1_i27 + c1_i25] = c1_y[c1_i27 + c1_i25];
    }

    c1_i25 += 3;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i28;
  int32_T c1_i29;
  const mxArray *c1_y = NULL;
  int32_T c1_i30;
  real_T c1_u[10000];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i28 = 0;
  for (c1_i29 = 0; c1_i29 < 100; c1_i29++) {
    for (c1_i30 = 0; c1_i30 < 100; c1_i30++) {
      c1_u[c1_i30 + c1_i28] = (*(real_T (*)[10000])c1_inData)[c1_i30 + c1_i28];
    }

    c1_i28 += 100;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 100, 100),
                false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_c_map;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[10000];
  int32_T c1_i31;
  int32_T c1_i32;
  int32_T c1_i33;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_c_map = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_c_map), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_c_map);
  c1_i31 = 0;
  for (c1_i32 = 0; c1_i32 < 100; c1_i32++) {
    for (c1_i33 = 0; c1_i33 < 100; c1_i33++) {
      (*(real_T (*)[10000])c1_outData)[c1_i33 + c1_i31] = c1_y[c1_i33 + c1_i31];
    }

    c1_i31 += 100;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i34;
  const mxArray *c1_y = NULL;
  real_T c1_u[3];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i34 = 0; c1_i34 < 3; c1_i34++) {
    c1_u[c1_i34] = (*(real_T (*)[3])c1_inData)[c1_i34];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_f_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[3])
{
  real_T c1_dv1[3];
  int32_T c1_i35;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv1, 1, 0, 0U, 1, 0U, 1, 3);
  for (c1_i35 = 0; c1_i35 < 3; c1_i35++) {
    c1_y[c1_i35] = c1_dv1[c1_i35];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_p;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[3];
  int32_T c1_i36;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_p = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_p), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_p);
  for (c1_i36 = 0; c1_i36 < 3; c1_i36++) {
    (*(real_T (*)[3])c1_outData)[c1_i36] = c1_y[c1_i36];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[], int32_T c1_inData_sizes[2])
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u_sizes[2];
  int32_T c1_u;
  int32_T c1_b_u;
  int32_T c1_i37;
  const mxArray *c1_y = NULL;
  real_T c1_u_data[2];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  (void)c1_inData_sizes;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u_sizes[0] = 1;
  c1_u_sizes[1] = 2;
  c1_u = c1_u_sizes[0];
  c1_b_u = c1_u_sizes[1];
  for (c1_i37 = 0; c1_i37 < 2; c1_i37++) {
    c1_u_data[c1_i37] = c1_inData_data[c1_i37];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u_data, 0, 0U, 1U, 0U, 2,
    c1_u_sizes[0], c1_u_sizes[1]), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_g_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[], int32_T c1_y_sizes[2])
{
  int32_T c1_i38;
  int32_T c1_i39;
  uint32_T c1_uv0[2];
  real_T c1_tmp_data[2];
  boolean_T c1_bv0[2];
  int32_T c1_tmp_sizes[2];
  int32_T c1_y;
  int32_T c1_b_y;
  int32_T c1_i40;
  (void)chartInstance;
  for (c1_i38 = 0; c1_i38 < 2; c1_i38++) {
    c1_uv0[c1_i38] = 1U + (uint32_T)c1_i38;
  }

  for (c1_i39 = 0; c1_i39 < 2; c1_i39++) {
    c1_bv0[c1_i39] = false;
  }

  sf_mex_import_vs(c1_parentId, sf_mex_dup(c1_u), c1_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c1_bv0, c1_uv0, c1_tmp_sizes);
  c1_y_sizes[0] = 1;
  c1_y_sizes[1] = 2;
  c1_y = c1_y_sizes[0];
  c1_b_y = c1_y_sizes[1];
  for (c1_i40 = 0; c1_i40 < 2; c1_i40++) {
    c1_y_data[c1_i40] = c1_tmp_data[c1_i40];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[], int32_T
  c1_outData_sizes[2])
{
  const mxArray *c1_Pxel;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y_data[2];
  int32_T c1_y_sizes[2];
  int32_T c1_i41;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_Pxel = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_Pxel), &c1_thisId,
                        c1_y_data, c1_y_sizes);
  sf_mex_destroy(&c1_Pxel);
  c1_outData_sizes[0] = 1;
  c1_outData_sizes[1] = 2;
  for (c1_i41 = 0; c1_i41 < 2; c1_i41++) {
    c1_outData_data[c1_outData_sizes[0] * c1_i41] = c1_y_data[c1_y_sizes[0] *
      c1_i41];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  c1_s2aqkGCuE38RBomNVWBcX1B c1_u;
  const mxArray *c1_y = NULL;
  int32_T c1_i42;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_u[10000];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(c1_s2aqkGCuE38RBomNVWBcX1B *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c1_i42 = 0; c1_i42 < 10000; c1_i42++) {
    c1_b_u[c1_i42] = c1_u.map[c1_i42];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 0, 0U, 1U, 0U, 2, 100, 100),
                false);
  sf_mex_addfield(c1_y, c1_b_y, "map", "map", 0);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_load;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  c1_s2aqkGCuE38RBomNVWBcX1B c1_y;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_load = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_load), &c1_thisId, &c1_y);
  sf_mex_destroy(&c1_load);
  *(c1_s2aqkGCuE38RBomNVWBcX1B *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[], int32_T c1_inData_sizes[2])
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u_sizes[2];
  int32_T c1_u;
  int32_T c1_b_u;
  int32_T c1_loop_ub;
  int32_T c1_i43;
  const mxArray *c1_y = NULL;
  real_T c1_u_data[2];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u_sizes[0] = c1_inData_sizes[0];
  c1_u_sizes[1] = c1_inData_sizes[1];
  c1_u = c1_u_sizes[0];
  c1_b_u = c1_u_sizes[1];
  c1_loop_ub = c1_inData_sizes[0] * c1_inData_sizes[1] - 1;
  for (c1_i43 = 0; c1_i43 <= c1_loop_ub; c1_i43++) {
    c1_u_data[c1_i43] = c1_inData_data[c1_i43];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u_data, 0, 0U, 1U, 0U, 2,
    c1_u_sizes[0], c1_u_sizes[1]), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_h_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[], int32_T c1_y_sizes[2])
{
  int32_T c1_i44;
  int32_T c1_i45;
  uint32_T c1_uv1[2];
  real_T c1_tmp_data[2];
  boolean_T c1_bv1[2];
  int32_T c1_tmp_sizes[2];
  int32_T c1_y;
  int32_T c1_b_y;
  int32_T c1_loop_ub;
  int32_T c1_i46;
  (void)chartInstance;
  for (c1_i44 = 0; c1_i44 < 2; c1_i44++) {
    c1_uv1[c1_i44] = 1U + (uint32_T)c1_i44;
  }

  for (c1_i45 = 0; c1_i45 < 2; c1_i45++) {
    c1_bv1[c1_i45] = true;
  }

  sf_mex_import_vs(c1_parentId, sf_mex_dup(c1_u), c1_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c1_bv1, c1_uv1, c1_tmp_sizes);
  c1_y_sizes[0] = c1_tmp_sizes[0];
  c1_y_sizes[1] = c1_tmp_sizes[1];
  c1_y = c1_y_sizes[0];
  c1_b_y = c1_y_sizes[1];
  c1_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i46 = 0; c1_i46 <= c1_loop_ub; c1_i46++) {
    c1_y_data[c1_i46] = c1_tmp_data[c1_i46];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[], int32_T
  c1_outData_sizes[2])
{
  const mxArray *c1_p;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y_data[2];
  int32_T c1_y_sizes[2];
  int32_T c1_loop_ub;
  int32_T c1_i47;
  int32_T c1_b_loop_ub;
  int32_T c1_i48;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_p = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_p), &c1_thisId, c1_y_data,
                        c1_y_sizes);
  sf_mex_destroy(&c1_p);
  c1_outData_sizes[0] = c1_y_sizes[0];
  c1_outData_sizes[1] = c1_y_sizes[1];
  c1_loop_ub = c1_y_sizes[1] - 1;
  for (c1_i47 = 0; c1_i47 <= c1_loop_ub; c1_i47++) {
    c1_b_loop_ub = c1_y_sizes[0] - 1;
    for (c1_i48 = 0; c1_i48 <= c1_b_loop_ub; c1_i48++) {
      c1_outData_data[c1_i48 + c1_outData_sizes[0] * c1_i47] = c1_y_data[c1_i48
        + c1_y_sizes[0] * c1_i47];
    }
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i49;
  const mxArray *c1_y = NULL;
  real_T c1_u[2];
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i49 = 0; c1_i49 < 2; c1_i49++) {
    c1_u[c1_i49] = (*(real_T (*)[2])c1_inData)[c1_i49];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_i_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[2])
{
  real_T c1_dv2[2];
  int32_T c1_i50;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv2, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c1_i50 = 0; c1_i50 < 2; c1_i50++) {
    c1_y[c1_i50] = c1_dv2[c1_i50];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_p2;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[2];
  int32_T c1_i51;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_p2 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_p2), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_p2);
  for (c1_i51 = 0; c1_i51 < 2; c1_i51++) {
    (*(real_T (*)[2])c1_outData)[c1_i51] = c1_y[c1_i51];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_AutoFollow_Simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  const char * c1_data[4] = {
    "789ced94c14a03311086b35a8b0a8a2f20ecd18bc693785450b1a53da8155aa4877433b681245376d36aaf7d0a8f3e93e0bb7834dbddb661595a5815453a1066"
    "ff4cbe2593fc8478953ab1b16bc7c72621659b6d226b24898d547b76aca739992f919d548fed08501b783649513305641a1c95d04c9bc6a80f248408e510f8a4",
    "f228243484821a3ae25a58a1ae9cd24cc4a5b017cdfe4ca42b9288fb3826f33e4a993ea631ed63cfe14e17705e5a73b958375b062bd5e23cc7414742c2bf2ce1"
    "21c3c7fae1b24def230823aa44d063203b213e697a3360fcf04ea8c947807d03a17f31b2f72202bf8e1ca4d05d9f69eedb3503c98c404d6b82b390b228125dad",
    "409b139a3477a472cea79cb33fcfd9dfd6fcbcdbfb6fefe75fe073efe737f80387f77278e2e422eb8bfaf66c0117afdfce70b196cc9ae696e92ee49f4f11ffbe"
    "2ee155868ff5b7fb371c060651463440a5ececbcd1958fc9dff67111ff55aa069bad7ffafe26cdad7c4b7edeb79f909abd76",
    "" };

  c1_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(c1_data, 2312U, &c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static boolean_T c1_all(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  boolean_T c1_x[2])
{
  boolean_T c1_y;
  int32_T c1_k;
  real_T c1_b_k;
  boolean_T exitg1;
  (void)chartInstance;
  c1_y = true;
  c1_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c1_k < 2)) {
    c1_b_k = 1.0 + (real_T)c1_k;
    if (!c1_x[(int32_T)c1_b_k - 1]) {
      c1_y = false;
      exitg1 = true;
    } else {
      c1_k++;
    }
  }

  return c1_y;
}

static real_T c1_mpower(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c1_a)
{
  real_T c1_c;
  real_T c1_b_a;
  real_T c1_c_a;
  real_T c1_x;
  real_T c1_d_a;
  boolean_T c1_p;
  c1_b_a = c1_a;
  c1_c_a = c1_b_a;
  c1_x = c1_c_a;
  c1_d_a = c1_x;
  c1_c = c1_d_a * c1_d_a;
  c1_p = false;
  if (c1_p) {
    c1_error(chartInstance);
  }

  return c1_c;
}

static void c1_scalarEg(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_dimagree(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_error(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 31), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c1_y));
}

static real_T c1_sqrt(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_sqrt(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_b_error(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c1_b_y = NULL;
  static char_T c1_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static real_T c1_cos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_cos(chartInstance, &c1_b_x);
  return c1_b_x;
}

static real_T c1_sin(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_sin(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_b_mpower(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c1_a[9], real_T c1_c[9])
{
  int32_T c1_i52;
  real_T c1_b_a[9];
  for (c1_i52 = 0; c1_i52 < 9; c1_i52++) {
    c1_b_a[c1_i52] = c1_a[c1_i52];
  }

  c1_matrix_to_integer_power(chartInstance, c1_b_a, c1_c);
}

static void c1_matrix_to_integer_power(SFc1_AutoFollow_SimulationInstanceStruct *
  chartInstance, real_T c1_a[9], real_T c1_c[9])
{
  int32_T c1_i53;
  real_T c1_b_a[9];
  int32_T c1_i54;
  real_T c1_n1x;
  real_T c1_c_a[9];
  int32_T c1_i55;
  real_T c1_n1xinv;
  real_T c1_b_c[9];
  real_T c1_rc;
  real_T c1_x;
  boolean_T c1_b;
  real_T c1_b_x;
  const mxArray *c1_y = NULL;
  static char_T c1_rfmt[6] = { '%', '1', '4', '.', '6', 'e' };

  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  char_T c1_str[14];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  for (c1_i53 = 0; c1_i53 < 9; c1_i53++) {
    c1_b_a[c1_i53] = c1_a[c1_i53];
  }

  c1_inv3x3(chartInstance, c1_b_a, c1_c);
  for (c1_i54 = 0; c1_i54 < 9; c1_i54++) {
    c1_c_a[c1_i54] = c1_a[c1_i54];
  }

  c1_n1x = c1_norm(chartInstance, c1_c_a);
  for (c1_i55 = 0; c1_i55 < 9; c1_i55++) {
    c1_b_c[c1_i55] = c1_c[c1_i55];
  }

  c1_n1xinv = c1_norm(chartInstance, c1_b_c);
  c1_rc = 1.0 / (c1_n1x * c1_n1xinv);
  guard1 = false;
  guard2 = false;
  if (c1_n1x == 0.0) {
    guard2 = true;
  } else if (c1_n1xinv == 0.0) {
    guard2 = true;
  } else if (c1_rc == 0.0) {
    guard1 = true;
  } else {
    c1_x = c1_rc;
    c1_b = muDoubleScalarIsNaN(c1_x);
    guard3 = false;
    if (c1_b) {
      guard3 = true;
    } else {
      if (c1_rc < 2.2204460492503131E-16) {
        guard3 = true;
      }
    }

    if (guard3 == true) {
      c1_b_x = c1_rc;
      c1_y = NULL;
      sf_mex_assign(&c1_y, sf_mex_create("y", c1_rfmt, 10, 0U, 1U, 0U, 2, 1, 6),
                    false);
      c1_u = c1_b_x;
      c1_b_y = NULL;
      sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
      c1_m_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                            (sfGlobalDebugInstanceStruct, "sprintf", 1U, 2U, 14,
        c1_y, 14, c1_b_y), "sprintf", c1_str);
      c1_b_warning(chartInstance, c1_str);
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c1_warning(chartInstance);
  }
}

static real_T c1_abs(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x)
{
  real_T c1_b_x;
  real_T c1_c_x;
  (void)chartInstance;
  c1_b_x = c1_x;
  c1_c_x = c1_b_x;
  return muDoubleScalarAbs(c1_c_x);
}

static void c1_inv3x3(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x[9], real_T c1_y[9])
{
  int32_T c1_p1;
  int32_T c1_p2;
  int32_T c1_p3;
  real_T c1_absx11;
  real_T c1_absx21;
  real_T c1_absx31;
  real_T c1_t1;
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_z;
  real_T c1_c_x;
  real_T c1_c_y;
  real_T c1_b_z;
  int32_T c1_itmp;
  real_T c1_d_x;
  real_T c1_d_y;
  real_T c1_c_z;
  real_T c1_e_x;
  real_T c1_e_y;
  real_T c1_t3;
  real_T c1_f_x;
  real_T c1_f_y;
  real_T c1_t2;
  int32_T c1_a;
  int32_T c1_c;
  real_T c1_g_x;
  real_T c1_g_y;
  real_T c1_d_z;
  int32_T c1_b_a;
  int32_T c1_b_c;
  int32_T c1_c_a;
  int32_T c1_c_c;
  real_T c1_h_x;
  real_T c1_h_y;
  real_T c1_i_x;
  real_T c1_i_y;
  int32_T c1_d_a;
  int32_T c1_d_c;
  real_T c1_j_x;
  real_T c1_j_y;
  real_T c1_e_z;
  int32_T c1_e_a;
  int32_T c1_e_c;
  int32_T c1_f_a;
  int32_T c1_f_c;
  real_T c1_k_y;
  real_T c1_k_x;
  real_T c1_l_y;
  int32_T c1_g_a;
  int32_T c1_g_c;
  real_T c1_l_x;
  real_T c1_m_y;
  real_T c1_f_z;
  int32_T c1_h_a;
  int32_T c1_h_c;
  int32_T c1_i_a;
  int32_T c1_i_c;
  boolean_T guard1 = false;
  c1_p1 = 0;
  c1_p2 = 3;
  c1_p3 = 6;
  c1_absx11 = c1_abs(chartInstance, c1_x[0]);
  c1_absx21 = c1_abs(chartInstance, c1_x[1]);
  c1_absx31 = c1_abs(chartInstance, c1_x[2]);
  guard1 = false;
  if (c1_absx21 > c1_absx11) {
    if (c1_absx21 > c1_absx31) {
      c1_p1 = 3;
      c1_p2 = 0;
      c1_t1 = c1_x[0];
      c1_x[0] = c1_x[1];
      c1_x[1] = c1_t1;
      c1_t1 = c1_x[3];
      c1_x[3] = c1_x[4];
      c1_x[4] = c1_t1;
      c1_t1 = c1_x[6];
      c1_x[6] = c1_x[7];
      c1_x[7] = c1_t1;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    if (c1_absx31 > c1_absx11) {
      c1_p1 = 6;
      c1_p3 = 0;
      c1_t1 = c1_x[0];
      c1_x[0] = c1_x[2];
      c1_x[2] = c1_t1;
      c1_t1 = c1_x[3];
      c1_x[3] = c1_x[5];
      c1_x[5] = c1_t1;
      c1_t1 = c1_x[6];
      c1_x[6] = c1_x[8];
      c1_x[8] = c1_t1;
    }
  }

  c1_b_x = c1_x[1];
  c1_b_y = c1_x[0];
  c1_z = c1_b_x / c1_b_y;
  c1_x[1] = c1_z;
  c1_c_x = c1_x[2];
  c1_c_y = c1_x[0];
  c1_b_z = c1_c_x / c1_c_y;
  c1_x[2] = c1_b_z;
  c1_x[4] -= c1_x[1] * c1_x[3];
  c1_x[5] -= c1_x[2] * c1_x[3];
  c1_x[7] -= c1_x[1] * c1_x[6];
  c1_x[8] -= c1_x[2] * c1_x[6];
  if (c1_abs(chartInstance, c1_x[5]) > c1_abs(chartInstance, c1_x[4])) {
    c1_itmp = c1_p2;
    c1_p2 = c1_p3;
    c1_p3 = c1_itmp;
    c1_t1 = c1_x[1];
    c1_x[1] = c1_x[2];
    c1_x[2] = c1_t1;
    c1_t1 = c1_x[4];
    c1_x[4] = c1_x[5];
    c1_x[5] = c1_t1;
    c1_t1 = c1_x[7];
    c1_x[7] = c1_x[8];
    c1_x[8] = c1_t1;
  }

  c1_d_x = c1_x[5];
  c1_d_y = c1_x[4];
  c1_c_z = c1_d_x / c1_d_y;
  c1_x[5] = c1_c_z;
  c1_x[8] -= c1_x[5] * c1_x[7];
  c1_e_x = c1_x[5] * c1_x[1] - c1_x[2];
  c1_e_y = c1_x[8];
  c1_t3 = c1_e_x / c1_e_y;
  c1_f_x = -(c1_x[1] + c1_x[7] * c1_t3);
  c1_f_y = c1_x[4];
  c1_t2 = c1_f_x / c1_f_y;
  c1_a = c1_p1 + 1;
  c1_c = c1_a - 1;
  c1_g_x = (1.0 - c1_x[3] * c1_t2) - c1_x[6] * c1_t3;
  c1_g_y = c1_x[0];
  c1_d_z = c1_g_x / c1_g_y;
  c1_y[c1_c] = c1_d_z;
  c1_b_a = c1_p1 + 2;
  c1_b_c = c1_b_a - 1;
  c1_y[c1_b_c] = c1_t2;
  c1_c_a = c1_p1 + 3;
  c1_c_c = c1_c_a - 1;
  c1_y[c1_c_c] = c1_t3;
  c1_h_x = -c1_x[5];
  c1_h_y = c1_x[8];
  c1_t3 = c1_h_x / c1_h_y;
  c1_i_x = 1.0 - c1_x[7] * c1_t3;
  c1_i_y = c1_x[4];
  c1_t2 = c1_i_x / c1_i_y;
  c1_d_a = c1_p2 + 1;
  c1_d_c = c1_d_a - 1;
  c1_j_x = -(c1_x[3] * c1_t2 + c1_x[6] * c1_t3);
  c1_j_y = c1_x[0];
  c1_e_z = c1_j_x / c1_j_y;
  c1_y[c1_d_c] = c1_e_z;
  c1_e_a = c1_p2 + 2;
  c1_e_c = c1_e_a - 1;
  c1_y[c1_e_c] = c1_t2;
  c1_f_a = c1_p2 + 3;
  c1_f_c = c1_f_a - 1;
  c1_y[c1_f_c] = c1_t3;
  c1_k_y = c1_x[8];
  c1_t3 = 1.0 / c1_k_y;
  c1_k_x = -c1_x[7] * c1_t3;
  c1_l_y = c1_x[4];
  c1_t2 = c1_k_x / c1_l_y;
  c1_g_a = c1_p3 + 1;
  c1_g_c = c1_g_a - 1;
  c1_l_x = -(c1_x[3] * c1_t2 + c1_x[6] * c1_t3);
  c1_m_y = c1_x[0];
  c1_f_z = c1_l_x / c1_m_y;
  c1_y[c1_g_c] = c1_f_z;
  c1_h_a = c1_p3 + 2;
  c1_h_c = c1_h_a - 1;
  c1_y[c1_h_c] = c1_t2;
  c1_i_a = c1_p3 + 3;
  c1_i_c = c1_i_a - 1;
  c1_y[c1_i_c] = c1_t3;
}

static real_T c1_norm(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x[9])
{
  real_T c1_y;
  int32_T c1_j;
  real_T c1_b_j;
  real_T c1_s;
  int32_T c1_i;
  real_T c1_b_x;
  real_T c1_b_i;
  boolean_T c1_b;
  boolean_T exitg1;
  c1_y = 0.0;
  c1_j = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c1_j < 3)) {
    c1_b_j = 1.0 + (real_T)c1_j;
    c1_s = 0.0;
    for (c1_i = 0; c1_i < 3; c1_i++) {
      c1_b_i = 1.0 + (real_T)c1_i;
      c1_s += c1_abs(chartInstance, c1_x[((int32_T)c1_b_i + 3 * ((int32_T)c1_b_j
        - 1)) - 1]);
    }

    c1_b_x = c1_s;
    c1_b = muDoubleScalarIsNaN(c1_b_x);
    if (c1_b) {
      c1_y = rtNaN;
      exitg1 = true;
    } else {
      if (c1_s > c1_y) {
        c1_y = c1_s;
      }

      c1_j++;
    }
  }

  return c1_y;
}

static void c1_warning(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[7] = { 'w', 'a', 'r', 'n', 'i', 'n', 'g' };

  const mxArray *c1_b_y = NULL;
  static char_T c1_b_u[7] = { 'm', 'e', 's', 's', 'a', 'g', 'e' };

  const mxArray *c1_c_y = NULL;
  static char_T c1_msgID[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T',
    'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a', 't',
    'r', 'i', 'x' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 7), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 7),
                false);
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", c1_msgID, 10, 0U, 1U, 0U, 2, 1, 27),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "feval", 0U, 2U, 14, c1_y, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "feval", 1U,
    2U, 14, c1_b_y, 14, c1_c_y));
}

static void c1_b_warning(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  char_T c1_varargin_1[14])
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[7] = { 'w', 'a', 'r', 'n', 'i', 'n', 'g' };

  const mxArray *c1_b_y = NULL;
  static char_T c1_b_u[7] = { 'm', 'e', 's', 's', 'a', 'g', 'e' };

  const mxArray *c1_c_y = NULL;
  static char_T c1_msgID[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T',
    'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i', 'o',
    'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  const mxArray *c1_d_y = NULL;
  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 7), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 7),
                false);
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", c1_msgID, 10, 0U, 1U, 0U, 2, 1, 33),
                false);
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", c1_varargin_1, 10, 0U, 1U, 0U, 2, 1,
    14), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "feval", 0U, 2U, 14, c1_y, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "feval", 1U,
    3U, 14, c1_b_y, 14, c1_c_y, 14, c1_d_y));
}

static void c1_b_scalarEg(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_c_scalarEg(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_power(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_a[3], real_T c1_y[3])
{
  int32_T c1_k;
  int32_T c1_b_k;
  real_T c1_b_a;
  real_T c1_b_y;
  (void)chartInstance;
  for (c1_k = 1; c1_k < 4; c1_k++) {
    c1_b_k = c1_k - 1;
    c1_b_a = c1_a[c1_b_k];
    c1_b_y = c1_b_a * c1_b_a;
    c1_y[c1_b_k] = c1_b_y;
  }
}

static void c1_b_dimagree(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c1_sum(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c1_x[3])
{
  real_T c1_y;
  int32_T c1_k;
  int32_T c1_b_k;
  (void)chartInstance;
  c1_y = c1_x[0];
  for (c1_k = 2; c1_k < 4; c1_k++) {
    c1_b_k = c1_k - 1;
    c1_y += c1_x[c1_b_k];
  }

  return c1_y;
}

static real_T c1_atan2(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c1_y, real_T c1_x)
{
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_c_y;
  real_T c1_c_x;
  (void)chartInstance;
  c1_b_x = c1_y;
  c1_b_y = c1_x;
  c1_c_y = c1_b_x;
  c1_c_x = c1_b_y;
  return muDoubleScalarAtan2(c1_c_y, c1_c_x);
}

static real_T c1_acos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_acos(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_c_error(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c1_b_y = NULL;
  static char_T c1_b_u[4] = { 'a', 'c', 'o', 's' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static const mxArray *c1_emlrt_marshallOut
  (SFc1_AutoFollow_SimulationInstanceStruct *chartInstance, const char * c1_u)
{
  const mxArray *c1_y = NULL;
  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_u)), false);
  return c1_y;
}

static void c1_j_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_load, const char_T *c1_identifier,
  c1_s2aqkGCuE38RBomNVWBcX1B *c1_y)
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_load), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_load);
}

static void c1_k_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  c1_s2aqkGCuE38RBomNVWBcX1B *c1_y)
{
  emlrtMsgIdentifier c1_thisId;
  static const char * c1_fieldNames[1] = { "map" };

  c1_thisId.fParent = c1_parentId;
  c1_thisId.bParentIsCell = false;
  sf_mex_check_struct(c1_parentId, c1_u, 1, c1_fieldNames, 0U, NULL);
  c1_thisId.fIdentifier = "map";
  c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c1_u, "map",
    "map", 0)), &c1_thisId, c1_y->map);
  sf_mex_destroy(&c1_u);
}

static void c1_l_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[10000])
{
  real_T c1_dv3[10000];
  int32_T c1_i56;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv3, 1, 0, 0U, 1, 0U, 2, 100,
                100);
  for (c1_i56 = 0; c1_i56 < 10000; c1_i56++) {
    c1_y[c1_i56] = c1_dv3[c1_i56];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_m_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_sprintf, const char_T *c1_identifier, char_T
  c1_y[14])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_sprintf), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_sprintf);
}

static void c1_n_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  char_T c1_y[14])
{
  char_T c1_cv0[14];
  int32_T c1_i57;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_cv0, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c1_i57 = 0; c1_i57 < 14; c1_i57++) {
    c1_y[c1_i57] = c1_cv0[c1_i57];
  }

  sf_mex_destroy(&c1_u);
}

static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_o_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i58;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i58, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i58;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_p_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_AutoFollow_Simulation, const
  char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_AutoFollow_Simulation), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_AutoFollow_Simulation);
  return c1_y;
}

static uint8_T c1_q_emlrt_marshallIn(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sqrt(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T *c1_x)
{
  real_T c1_b_x;
  boolean_T c1_b0;
  boolean_T c1_p;
  c1_b_x = *c1_x;
  c1_b0 = (c1_b_x < 0.0);
  c1_p = c1_b0;
  if (c1_p) {
    c1_b_error(chartInstance);
  }

  *c1_x = muDoubleScalarSqrt(*c1_x);
}

static void c1_b_cos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c1_x)
{
  (void)chartInstance;
  *c1_x = muDoubleScalarCos(*c1_x);
}

static void c1_b_sin(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c1_x)
{
  (void)chartInstance;
  *c1_x = muDoubleScalarSin(*c1_x);
}

static void c1_b_acos(SFc1_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T *c1_x)
{
  real_T c1_b_x;
  boolean_T c1_b1;
  boolean_T c1_b2;
  boolean_T c1_b3;
  boolean_T c1_p;
  c1_b_x = *c1_x;
  c1_b1 = (c1_b_x < -1.0);
  c1_b2 = (c1_b_x > 1.0);
  c1_b3 = (c1_b1 || c1_b2);
  c1_p = c1_b3;
  if (c1_p) {
    c1_c_error(chartInstance);
  }

  *c1_x = muDoubleScalarAcos(*c1_x);
}

static void init_dsm_address_info(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  chartInstance->c1_xq = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c1_yq = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c1_zq = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c1_thetaq = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_psiq = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_phiq = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_hx = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    6);
  chartInstance->c1_hy = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    7);
  chartInstance->c1_hz = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    8);
  chartInstance->c1_r = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c1_theta = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_psi = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
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

void sf_c1_AutoFollow_Simulation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3429122700U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2086225913U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(376030330U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2575498434U);
}

mxArray* sf_c1_AutoFollow_Simulation_get_post_codegen_info(void);
mxArray *sf_c1_AutoFollow_Simulation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("PFMMxU5QwqQnkpqefcMz8E");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,9,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));
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
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c1_AutoFollow_Simulation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_AutoFollow_Simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_AutoFollow_Simulation_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_AutoFollow_Simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_AutoFollow_Simulation_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_AutoFollow_Simulation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[8],T\"psi\",},{M[1],M[13],T\"r\",},{M[1],M[7],T\"theta\",},{M[4],M[0],T\"psi_prev\",S'l','i','p'{{M1x2[152 160],M[0],}}},{M[4],M[0],T\"r_prev\",S'l','i','p'{{M1x2[103 109],M[0],}}},{M[4],M[0],T\"theta_prev\",S'l','i','p'{{M1x2[126 136],M[0],}}},{M[8],M[0],T\"is_active_c1_AutoFollow_Simulation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_AutoFollow_Simulation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_AutoFollow_SimulationInstanceStruct *chartInstance =
      (SFc1_AutoFollow_SimulationInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _AutoFollow_SimulationMachineNumber_,
           1,
           1,
           1,
           0,
           12,
           0,
           0,
           0,
           0,
           3,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"xq");
          _SFD_SET_DATA_PROPS(1,1,1,0,"yq");
          _SFD_SET_DATA_PROPS(2,1,1,0,"zq");
          _SFD_SET_DATA_PROPS(3,1,1,0,"thetaq");
          _SFD_SET_DATA_PROPS(4,1,1,0,"psiq");
          _SFD_SET_DATA_PROPS(5,1,1,0,"phiq");
          _SFD_SET_DATA_PROPS(6,1,1,0,"hx");
          _SFD_SET_DATA_PROPS(7,1,1,0,"hy");
          _SFD_SET_DATA_PROPS(8,1,1,0,"hz");
          _SFD_SET_DATA_PROPS(9,2,0,1,"r");
          _SFD_SET_DATA_PROPS(10,2,0,1,"theta");
          _SFD_SET_DATA_PROPS(11,2,0,1,"psi");
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
        _SFD_CV_INIT_EML(0,1,1,0,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2866);
        _SFD_CV_INIT_EML_IF(0,1,0,769,814,2756,2862);

        {
          static int condStart[] = { 772, 792 };

          static int condEnd[] = { 787, 813 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,772,814,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,792,813,-1,2);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"XYtoIJ",0,-1,222);
        _SFD_CV_INIT_SCRIPT(1,1,0,11,0,0,0,0,2,8,2);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"laserRange",1033,-1,2550);
        _SFD_CV_INIT_SCRIPT_IF(1,0,1175,1185,1226,1275);
        _SFD_CV_INIT_SCRIPT_IF(1,1,1293,1303,1344,1393);
        _SFD_CV_INIT_SCRIPT_IF(1,2,1412,1422,1965,2546);
        _SFD_CV_INIT_SCRIPT_IF(1,3,1543,1580,1652,1721);
        _SFD_CV_INIT_SCRIPT_IF(1,4,1652,1669,1730,1811);
        _SFD_CV_INIT_SCRIPT_IF(1,5,1730,1754,1820,1950);
        _SFD_CV_INIT_SCRIPT_IF(1,6,1820,1829,1881,1950);
        _SFD_CV_INIT_SCRIPT_IF(1,7,2090,2127,2199,2268);
        _SFD_CV_INIT_SCRIPT_IF(1,8,2199,2216,2277,2389);
        _SFD_CV_INIT_SCRIPT_IF(1,9,2277,2301,2398,2528);
        _SFD_CV_INIT_SCRIPT_IF(1,10,2398,2407,2459,2528);
        _SFD_CV_INIT_SCRIPT_WHILE(1,0,1482,1490,1960);
        _SFD_CV_INIT_SCRIPT_WHILE(1,1,2029,2037,2538);

        {
          static int condStart[] = { 1547, 1556, 1565, 1574 };

          static int condEnd[] = { 1552, 1561, 1570, 1579 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2, 3, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(1,0,1547,1579,4,0,&(condStart[0]),&(condEnd[0]),
            7,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2094, 2103, 2112, 2121 };

          static int condEnd[] = { 2099, 2108, 2117, 2126 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2, 3, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(1,1,2094,2126,4,4,&(condStart[0]),&(condEnd[0]),
            7,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,0,1178,1185,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,1,1296,1303,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,2,1415,1422,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,3,1547,1552,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,4,1556,1561,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,5,1565,1570,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,6,1574,1579,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,7,1655,1669,-1,0);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,8,1824,1829,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,9,2094,2099,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,10,2103,2108,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,11,2112,2117,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,12,2121,2126,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,13,2202,2216,-1,0);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,14,2402,2407,-1,2);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"IJtoXY",0,-1,175);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)c1_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)c1_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)c1_b_sf_marshallIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _AutoFollow_SimulationMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_AutoFollow_SimulationInstanceStruct *chartInstance =
      (SFc1_AutoFollow_SimulationInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c1_xq);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c1_yq);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c1_zq);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_thetaq);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c1_psiq);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c1_phiq);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c1_hx);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c1_hy);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c1_hz);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c1_r);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c1_theta);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c1_psi);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sUSFZ7x8dw6CUCJMyUq6gT";
}

static void sf_opaque_initialize_c1_AutoFollow_Simulation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_AutoFollow_Simulation
    ((SFc1_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
  initialize_c1_AutoFollow_Simulation((SFc1_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_AutoFollow_Simulation(void *chartInstanceVar)
{
  enable_c1_AutoFollow_Simulation((SFc1_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c1_AutoFollow_Simulation(void *chartInstanceVar)
{
  disable_c1_AutoFollow_Simulation((SFc1_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_AutoFollow_Simulation(void *chartInstanceVar)
{
  sf_gateway_c1_AutoFollow_Simulation((SFc1_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_AutoFollow_Simulation(SimStruct*
  S)
{
  return get_sim_state_c1_AutoFollow_Simulation
    ((SFc1_AutoFollow_SimulationInstanceStruct *)sf_get_chart_instance_ptr(S));/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_AutoFollow_Simulation(SimStruct* S, const
  mxArray *st)
{
  set_sim_state_c1_AutoFollow_Simulation
    ((SFc1_AutoFollow_SimulationInstanceStruct*)sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c1_AutoFollow_Simulation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_AutoFollow_SimulationInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_AutoFollow_Simulation_optimization_info();
    }

    finalize_c1_AutoFollow_Simulation((SFc1_AutoFollow_SimulationInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_AutoFollow_Simulation
    ((SFc1_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_AutoFollow_Simulation(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_AutoFollow_Simulation
      ((SFc1_AutoFollow_SimulationInstanceStruct*)sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c1_AutoFollow_Simulation(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_AutoFollow_Simulation_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,9);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 9; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3651377185U));
  ssSetChecksum1(S,(420214777U));
  ssSetChecksum2(S,(2459098389U));
  ssSetChecksum3(S,(2553527040U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_AutoFollow_Simulation(SimStruct *S)
{
  SFc1_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc1_AutoFollow_SimulationInstanceStruct *)utMalloc(sizeof
    (SFc1_AutoFollow_SimulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_AutoFollow_SimulationInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_AutoFollow_Simulation;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_AutoFollow_Simulation(chartInstance);
}

void c1_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_AutoFollow_Simulation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_AutoFollow_Simulation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
