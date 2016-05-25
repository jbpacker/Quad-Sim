/* Include files */

#include <stddef.h>
#include "blas.h"
#include "AutoFollow_Simulation_sfun.h"
#include "c3_AutoFollow_Simulation.h"
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
static const char * c3_debug_family_names[19] = { "lidarUpdateTime", "Xmax",
  "Ymax", "map", "angleSpan", "angleStep", "rangeMax", "Tl", "p", "nargin",
  "nargout", "xq", "yq", "t", "r", "theta", "prev_t", "prev_r", "prev_theta" };

static const char * c3_b_debug_family_names[10] = { "x", "y", "th", "cth", "sth",
  "R", "nargin", "nargout", "a", "t" };

static const char * c3_c_debug_family_names[10] = { "nargin", "nargout", "x",
  "y", "Xmax", "Ymax", "R", "C", "i", "j" };

static const char * c3_d_debug_family_names[26] = { "varargin", "tol", "N1",
  "N2", "N", "dx", "dy", "denom", "par", "col", "x0", "y0", "inds", "x1", "y1",
  "dx1", "dy1", "x2", "y2", "dx2", "dy2", "nargin", "nargout", "line1", "line2",
  "point" };

static const char * c3_e_debug_family_names[12] = { "varargin", "vx", "vy", "dx",
  "dy", "delta", "invalidLine", "nargin", "nargout", "point", "line", "pos" };

static const char * c3_f_debug_family_names[28] = { "varargin", "nLines",
  "nBoxes", "i", "xmin", "xmax", "ymin", "ymax", "delta", "px1", "px2", "py1",
  "py2", "points", "pos", "inds", "ind", "inter1", "inter2", "midX", "xOk",
  "midY", "yOk", "box", "nargin", "nargout", "line", "edge" };

static const char * c3_g_debug_family_names[21] = { "R", "C", "x1", "y1", "x2",
  "y2", "x", "xd", "dx", "y", "yd", "dy", "a", "b", "c", "nargin", "nargout",
  "p1", "p2", "map", "p" };

static const char * c3_h_debug_family_names[10] = { "nargin", "nargout", "i",
  "j", "Xmax", "Ymax", "R", "C", "x", "y" };

static const char * c3_i_debug_family_names[34] = { "N", "R", "C", "i", "P1",
  "x1", "y1", "I1", "J1", "a", "Xl", "Yl", "P2", "x2", "y2", "edge", "I2", "J2",
  "Pxel", "Io", "Jo", "xTarget", "yTarget", "r", "angleSpan", "angleStep",
  "rangeMax", "Xmax", "Ymax", "nargin", "nargout", "Tl", "map", "p" };

/* Function Declarations */
static void initialize_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initialize_params_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void enable_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void disable_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void set_sim_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c3_st);
static void finalize_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void sf_gateway_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_chartstep_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initSimStructsc3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_se2(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c3_a[3], real_T c3_t[9]);
static void c3_laserScanner(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_Tl[9], real_T c3_map[10000], real_T c3_p[722]);
static void c3_XYtoIJ(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x, real_T c3_y, real_T c3_Xmax, real_T c3_Ymax,
                      real_T c3_R, real_T c3_C, real_T *c3_i, real_T *c3_j);
static void c3_clipLine(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_line[4], real_T c3_edge[4]);
static void c3_intersectLines(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_line1[4], real_T c3_line2[4], real_T c3_point[2]);
static void c3_laserRange(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_p1[2], real_T c3_p2[2], real_T c3_map[10000], real_T
  c3_p_data[], int32_T c3_p_sizes[2]);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static void c3_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_load, const char_T *c3_identifier,
  c3_s2aqkGCuE38RBomNVWBcX1B *c3_y);
static void c3_b_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_s2aqkGCuE38RBomNVWBcX1B *c3_y);
static void c3_c_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[10000]);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static void c3_d_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_theta, const char_T *c3_identifier,
  real_T c3_y[361]);
static void c3_e_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_f_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_r, const char_T *c3_identifier,
  real_T c3_y[361]);
static void c3_g_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361]);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_h_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_t, const char_T *c3_identifier);
static real_T c3_i_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_j_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_theta, const char_T *c3_identifier, real_T
  c3_y[361]);
static void c3_k_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361]);
static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_l_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_m_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[722]);
static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_n_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[9]);
static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_o_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_k_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_p_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4]);
static void c3_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_l_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_q_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[2]);
static void c3_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_m_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_r_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4]);
static void c3_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_n_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_s_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_o_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static boolean_T c3_t_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_p_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_q_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T *c3_inData_sizes);
static void c3_u_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T *c3_y_sizes);
static void c3_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  *c3_outData_sizes);
static const mxArray *c3_r_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_v_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_s_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_t_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_w_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8]);
static void c3_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_u_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_v_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_x_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_w_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_y_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_x_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_ab_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(const mxArray **c3_info);
static const mxArray *c3_emlrt_marshallOut(const char * c3_u);
static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u);
static void c3_b_info_helper(const mxArray **c3_info);
static void c3_c_info_helper(const mxArray **c3_info);
static void c3_eml_scalar_eg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_eml_xgemm(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_A[9], real_T c3_B[3], real_T c3_C[3], real_T c3_b_C[3]);
static real_T c3_hypot(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x, real_T c3_y);
static void c3_b_eml_scalar_eg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static real_T c3_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a, real_T c3_b);
static void c3_c_eml_scalar_eg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_eml_li_find(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, boolean_T c3_x, int32_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_isfinite(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x[4], boolean_T c3_b[4]);
static void c3_eml_switch_helper(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_b_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes);
static void c3_check_forloop_overflow_error
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, boolean_T
   c3_overflow);
static void c3_c_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes);
static void c3_d_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes);
static void c3_eml_sort(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x_data[], int32_T c3_x_sizes, real_T c3_y_data[], int32_T
  *c3_y_sizes, int32_T c3_idx_data[], int32_T *c3_idx_sizes);
static void c3_eml_sort_idx(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_x_data[], int32_T c3_x_sizes, int32_T c3_idx_data[],
  int32_T *c3_idx_sizes);
static real_T c3_mean(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x[2]);
static boolean_T c3_all(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  boolean_T c3_x[2]);
static real_T c3_mpower(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a);
static void c3_eml_error(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *c3_y_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_bb_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_cb_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_AutoFollow_Simulation, const
  char_T *c3_identifier);
static uint8_T c3_db_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_eml_xgemm(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_A[9], real_T c3_B[3], real_T c3_C[3]);
static void init_dsm_address_info(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_prev_t_not_empty = false;
  chartInstance->c3_prev_r_not_empty = false;
  chartInstance->c3_prev_theta_not_empty = false;
  chartInstance->c3_is_active_c3_AutoFollow_Simulation = 0U;
}

static void initialize_params_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_update_debugger_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_u[361];
  const mxArray *c3_b_y = NULL;
  int32_T c3_i1;
  real_T c3_b_u[361];
  const mxArray *c3_c_y = NULL;
  int32_T c3_i2;
  real_T c3_c_u[361];
  const mxArray *c3_d_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_d_u;
  const mxArray *c3_e_y = NULL;
  int32_T c3_i3;
  real_T c3_e_u[361];
  const mxArray *c3_f_y = NULL;
  uint8_T c3_b_hoistedGlobal;
  uint8_T c3_f_u;
  const mxArray *c3_g_y = NULL;
  real_T (*c3_theta)[361];
  real_T (*c3_r)[361];
  c3_theta = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_r = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(6, 1), false);
  for (c3_i0 = 0; c3_i0 < 361; c3_i0++) {
    c3_u[c3_i0] = (*c3_r)[c3_i0];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 361), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  for (c3_i1 = 0; c3_i1 < 361; c3_i1++) {
    c3_b_u[c3_i1] = (*c3_theta)[c3_i1];
  }

  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", c3_b_u, 0, 0U, 1U, 0U, 1, 361),
                false);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  for (c3_i2 = 0; c3_i2 < 361; c3_i2++) {
    c3_c_u[c3_i2] = chartInstance->c3_prev_r[c3_i2];
  }

  c3_d_y = NULL;
  if (!chartInstance->c3_prev_r_not_empty) {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", c3_c_u, 0, 0U, 1U, 0U, 1, 361),
                  false);
  }

  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_hoistedGlobal = chartInstance->c3_prev_t;
  c3_d_u = c3_hoistedGlobal;
  c3_e_y = NULL;
  if (!chartInstance->c3_prev_t_not_empty) {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c3_y, 3, c3_e_y);
  for (c3_i3 = 0; c3_i3 < 361; c3_i3++) {
    c3_e_u[c3_i3] = chartInstance->c3_prev_theta[c3_i3];
  }

  c3_f_y = NULL;
  if (!chartInstance->c3_prev_theta_not_empty) {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", c3_e_u, 0, 0U, 1U, 0U, 1, 361),
                  false);
  }

  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_b_hoistedGlobal = chartInstance->c3_is_active_c3_AutoFollow_Simulation;
  c3_f_u = c3_b_hoistedGlobal;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_f_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 5, c3_g_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[361];
  int32_T c3_i4;
  real_T c3_dv1[361];
  int32_T c3_i5;
  real_T c3_dv2[361];
  int32_T c3_i6;
  real_T c3_dv3[361];
  int32_T c3_i7;
  real_T (*c3_r)[361];
  real_T (*c3_theta)[361];
  c3_theta = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_r = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)), "r",
                        c3_dv0);
  for (c3_i4 = 0; c3_i4 < 361; c3_i4++) {
    (*c3_r)[c3_i4] = c3_dv0[c3_i4];
  }

  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
                        "theta", c3_dv1);
  for (c3_i5 = 0; c3_i5 < 361; c3_i5++) {
    (*c3_theta)[c3_i5] = c3_dv1[c3_i5];
  }

  c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 2)),
                        "prev_r", c3_dv2);
  for (c3_i6 = 0; c3_i6 < 361; c3_i6++) {
    chartInstance->c3_prev_r[c3_i6] = c3_dv2[c3_i6];
  }

  chartInstance->c3_prev_t = c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 3)), "prev_t");
  c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 4)),
                        "prev_theta", c3_dv3);
  for (c3_i7 = 0; c3_i7 < 361; c3_i7++) {
    chartInstance->c3_prev_theta[c3_i7] = c3_dv3[c3_i7];
  }

  chartInstance->c3_is_active_c3_AutoFollow_Simulation = c3_cb_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 5)),
     "is_active_c3_AutoFollow_Simulation");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_AutoFollow_Simulation(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  int32_T c3_i8;
  int32_T c3_i9;
  real_T *c3_xq;
  real_T *c3_yq;
  real_T *c3_t;
  real_T (*c3_theta)[361];
  real_T (*c3_r)[361];
  c3_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_theta = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_yq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c3_r = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_xq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c3_xq, 0U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_AutoFollow_Simulation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_AutoFollow_SimulationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c3_i8 = 0; c3_i8 < 361; c3_i8++) {
    _SFD_DATA_RANGE_CHECK((*c3_r)[c3_i8], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c3_yq, 2U);
  for (c3_i9 = 0; c3_i9 < 361; c3_i9++) {
    _SFD_DATA_RANGE_CHECK((*c3_theta)[c3_i9], 3U);
  }

  _SFD_DATA_RANGE_CHECK(*c3_t, 4U);
}

static void c3_chartstep_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_c_hoistedGlobal;
  real_T c3_xq;
  real_T c3_yq;
  real_T c3_t;
  uint32_T c3_debug_family_var_map[19];
  real_T c3_lidarUpdateTime;
  real_T c3_Xmax;
  real_T c3_Ymax;
  static c3_s2aqkGCuE38RBomNVWBcX1B c3_map;
  real_T c3_angleSpan;
  real_T c3_angleStep;
  real_T c3_rangeMax;
  real_T c3_Tl[9];
  real_T c3_p[722];
  static real_T c3_b_map[10000];
  real_T c3_nargin = 3.0;
  real_T c3_nargout = 2.0;
  real_T c3_r[361];
  real_T c3_theta[361];
  const mxArray *c3_y = NULL;
  static c3_s2aqkGCuE38RBomNVWBcX1B c3_r0;
  int32_T c3_i10;
  real_T c3_b_xq[3];
  real_T c3_dv4[9];
  int32_T c3_i11;
  int32_T c3_i12;
  real_T c3_b_Tl[9];
  int32_T c3_i13;
  real_T c3_c_map[10000];
  real_T c3_dv5[722];
  int32_T c3_i14;
  int32_T c3_i15;
  int32_T c3_i16;
  int32_T c3_i17;
  int32_T c3_i18;
  int32_T c3_i19;
  int32_T c3_i20;
  int32_T c3_i21;
  int32_T c3_i22;
  real_T *c3_c_xq;
  real_T *c3_b_yq;
  real_T *c3_b_t;
  real_T (*c3_b_r)[361];
  real_T (*c3_b_theta)[361];
  boolean_T guard1 = false;
  c3_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_b_theta = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 2);
  c3_b_yq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c3_b_r = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_c_xq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *c3_c_xq;
  c3_b_hoistedGlobal = *c3_b_yq;
  c3_c_hoistedGlobal = *c3_b_t;
  c3_xq = c3_hoistedGlobal;
  c3_yq = c3_b_hoistedGlobal;
  c3_t = c3_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 19U, 20U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_lidarUpdateTime, 0U,
    c3_e_sf_marshallOut, c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Xmax, 1U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Ymax, 2U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_map, MAX_uint32_T,
    c3_i_sf_marshallOut, c3_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleSpan, 4U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleStep, 5U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_rangeMax, 6U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Tl, 7U, c3_h_sf_marshallOut,
    c3_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p, 8U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_map, MAX_uint32_T,
    c3_f_sf_marshallOut, c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 9U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 10U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_xq, 11U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_yq, 12U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_t, 13U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_r, 14U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_theta, 15U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c3_prev_t, 16U,
    c3_c_sf_marshallOut, c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c3_prev_r, 17U,
    c3_b_sf_marshallOut, c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c3_prev_theta, 18U,
    c3_sf_marshallOut, c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 2);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c3_prev_t_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 7);
    chartInstance->c3_prev_t = 0.0;
    chartInstance->c3_prev_t_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
  c3_lidarUpdateTime = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 12);
  guard1 = false;
  if (CV_EML_COND(0, 1, 0, !chartInstance->c3_prev_r_not_empty)) {
    guard1 = true;
  } else if (CV_EML_COND(0, 1, 1, c3_t - chartInstance->c3_prev_t > 0.1)) {
    guard1 = true;
  } else {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 1, false);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 37);
    for (c3_i19 = 0; c3_i19 < 361; c3_i19++) {
      c3_r[c3_i19] = chartInstance->c3_prev_r[c3_i19];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 38);
    for (c3_i20 = 0; c3_i20 < 361; c3_i20++) {
      c3_theta[c3_i20] = chartInstance->c3_prev_theta[c3_i20];
    }
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, true);
    CV_EML_IF(0, 1, 1, true);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
    c3_Xmax = 25.0;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
    c3_Ymax = 25.0;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 15);
    _SFD_SYMBOL_SWITCH(3U, 3U);
    c3_y = NULL;
    sf_mex_assign(&c3_y, sf_mex_create("y", "./Lidar/ObstacleMap/map.mat", 15,
      0U, 0U, 0U, 2, 1, strlen("./Lidar/ObstacleMap/map.mat")), false);
    c3_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                        (sfGlobalDebugInstanceStruct, "load", 1U, 1U, 14, c3_y),
                        "load", &c3_r0);
    c3_map = c3_r0;
    _SFD_SYMBOL_SWITCH(3U, 3U);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 16);
    for (c3_i10 = 0; c3_i10 < 10000; c3_i10++) {
      c3_b_map[c3_i10] = c3_map.map[c3_i10];
    }

    _SFD_SYMBOL_SWITCH(3U, 9U);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
    c3_angleSpan = 6.2831853071795862;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
    c3_angleStep = 0.017453292519943295;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 20);
    c3_rangeMax = 50.0;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
    c3_b_xq[0] = c3_xq * 3.28;
    c3_b_xq[1] = c3_yq * 3.28;
    c3_b_xq[2] = 0.0;
    c3_se2(chartInstance, c3_b_xq, c3_dv4);
    for (c3_i11 = 0; c3_i11 < 9; c3_i11++) {
      c3_Tl[c3_i11] = c3_dv4[c3_i11];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 27);
    for (c3_i12 = 0; c3_i12 < 9; c3_i12++) {
      c3_b_Tl[c3_i12] = c3_Tl[c3_i12];
    }

    for (c3_i13 = 0; c3_i13 < 10000; c3_i13++) {
      c3_c_map[c3_i13] = c3_b_map[c3_i13];
    }

    c3_laserScanner(chartInstance, c3_b_Tl, c3_c_map, c3_dv5);
    for (c3_i14 = 0; c3_i14 < 722; c3_i14++) {
      c3_p[c3_i14] = c3_dv5[c3_i14];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 32);
    for (c3_i15 = 0; c3_i15 < 361; c3_i15++) {
      c3_r[c3_i15] = c3_p[c3_i15 + 361];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 32);
    for (c3_i16 = 0; c3_i16 < 361; c3_i16++) {
      c3_theta[c3_i16] = c3_p[c3_i16];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 33);
    for (c3_i17 = 0; c3_i17 < 361; c3_i17++) {
      chartInstance->c3_prev_r[c3_i17] = c3_r[c3_i17];
    }

    chartInstance->c3_prev_r_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 34);
    for (c3_i18 = 0; c3_i18 < 361; c3_i18++) {
      chartInstance->c3_prev_theta[c3_i18] = c3_theta[c3_i18];
    }

    chartInstance->c3_prev_theta_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 35);
    chartInstance->c3_prev_t = c3_t;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -38);
  _SFD_SYMBOL_SCOPE_POP();
  for (c3_i21 = 0; c3_i21 < 361; c3_i21++) {
    (*c3_b_r)[c3_i21] = c3_r[c3_i21];
  }

  for (c3_i22 = 0; c3_i22 < 361; c3_i22++) {
    (*c3_b_theta)[c3_i22] = c3_theta[c3_i22];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_se2(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c3_a[3], real_T c3_t[9])
{
  uint32_T c3_debug_family_var_map[10];
  real_T c3_x;
  real_T c3_y;
  real_T c3_th;
  real_T c3_cth;
  real_T c3_sth;
  real_T c3_R[4];
  real_T c3_nargin = 1.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i23;
  static real_T c3_dv6[4] = { 1.0, 0.0, -0.0, 1.0 };

  real_T c3_b_x[2];
  int32_T c3_i24;
  int32_T c3_i25;
  int32_T c3_i26;
  int32_T c3_i27;
  int32_T c3_i28;
  int32_T c3_i29;
  int32_T c3_i30;
  static real_T c3_dv7[3] = { 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_b_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x, 0U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y, 1U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_th, 2U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_cth, 3U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_sth, 4U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R, 5U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 6U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_a, 8U, c3_j_sf_marshallOut,
    c3_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_t, 9U, c3_h_sf_marshallOut,
    c3_h_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 36);
  CV_SCRIPT_IF(0, 0, true);
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 37);
  c3_x = c3_a[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 38);
  c3_y = c3_a[1];
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 39);
  c3_th = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 58);
  c3_cth = 1.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 59);
  c3_sth = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 60);
  for (c3_i23 = 0; c3_i23 < 4; c3_i23++) {
    c3_R[c3_i23] = c3_dv6[c3_i23];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 62);
  c3_b_x[0] = c3_x;
  c3_b_x[1] = c3_y;
  c3_i24 = 0;
  c3_i25 = 0;
  for (c3_i26 = 0; c3_i26 < 2; c3_i26++) {
    for (c3_i27 = 0; c3_i27 < 2; c3_i27++) {
      c3_t[c3_i27 + c3_i24] = c3_R[c3_i27 + c3_i25];
    }

    c3_i24 += 3;
    c3_i25 += 2;
  }

  for (c3_i28 = 0; c3_i28 < 2; c3_i28++) {
    c3_t[c3_i28 + 6] = c3_b_x[c3_i28];
  }

  c3_i29 = 0;
  for (c3_i30 = 0; c3_i30 < 3; c3_i30++) {
    c3_t[c3_i29 + 2] = c3_dv7[c3_i30];
    c3_i29 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, -62);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_laserScanner(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_Tl[9], real_T c3_map[10000], real_T c3_p[722])
{
  uint32_T c3_debug_family_var_map[34];
  real_T c3_N;
  real_T c3_R;
  real_T c3_C;
  real_T c3_i;
  real_T c3_P1[3];
  real_T c3_x1;
  real_T c3_y1;
  real_T c3_I1;
  real_T c3_J1;
  real_T c3_a;
  real_T c3_Xl;
  real_T c3_Yl;
  real_T c3_P2[3];
  real_T c3_x2;
  real_T c3_y2;
  real_T c3_edge[4];
  real_T c3_I2;
  real_T c3_J2;
  int32_T c3_Pxel_sizes[2];
  real_T c3_Pxel_data[2];
  real_T c3_Io;
  real_T c3_Jo;
  real_T c3_xTarget;
  real_T c3_yTarget;
  real_T c3_r;
  real_T c3_angleSpan;
  real_T c3_angleStep;
  real_T c3_rangeMax;
  real_T c3_Xmax;
  real_T c3_Ymax;
  real_T c3_nargin = 7.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i31;
  int32_T c3_i32;
  real_T c3_b_a[9];
  int32_T c3_i33;
  int32_T c3_i34;
  int32_T c3_i35;
  real_T c3_dv8[9];
  int32_T c3_i36;
  static real_T c3_b[3] = { 0.0, 0.0, 1.0 };

  real_T c3_dv9[3];
  int32_T c3_i37;
  real_T c3_dv10[9];
  int32_T c3_i38;
  real_T c3_dv11[3];
  real_T c3_b_J1;
  real_T c3_b_I1;
  int32_T c3_c_a;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  int32_T c3_i39;
  real_T c3_b_Xl[3];
  int32_T c3_i40;
  real_T c3_b_b[3];
  int32_T c3_i41;
  int32_T c3_i42;
  int32_T c3_i43;
  real_T c3_dv12[9];
  int32_T c3_i44;
  real_T c3_dv13[3];
  int32_T c3_i45;
  real_T c3_dv14[9];
  int32_T c3_i46;
  real_T c3_dv15[3];
  real_T c3_b_x1[4];
  real_T c3_dv16[4];
  int32_T c3_i47;
  real_T c3_b_J2;
  real_T c3_b_I2;
  real_T c3_c_I1[2];
  real_T c3_c_I2[2];
  int32_T c3_i48;
  real_T c3_b_map[10000];
  int32_T c3_tmp_sizes[2];
  real_T c3_tmp_data[2];
  int32_T c3_Pxel;
  int32_T c3_b_Pxel;
  int32_T c3_loop_ub;
  int32_T c3_i49;
  real_T c3_e_x;
  boolean_T c3_c_b;
  real_T c3_f_x;
  boolean_T c3_d_b;
  real_T c3_b_i;
  real_T c3_j;
  real_T c3_b_Xmax;
  real_T c3_b_Ymax;
  real_T c3_b_R;
  real_T c3_b_C;
  uint32_T c3_b_debug_family_var_map[10];
  real_T c3_b_nargin = 6.0;
  real_T c3_b_nargout = 2.0;
  real_T c3_b_xTarget;
  real_T c3_b_yTarget;
  real_T c3_A;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_i_x;
  real_T c3_y;
  real_T c3_b_A;
  real_T c3_j_x;
  real_T c3_k_x;
  real_T c3_l_x;
  real_T c3_m_x;
  real_T c3_n_x;
  boolean_T guard1 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 34U, 34U, c3_i_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N, 0U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_R, 1U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_C, 2U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_i, 3U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_P1, 4U, c3_x_sf_marshallOut,
    c3_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x1, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y1, 6U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_I1, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_J1, 8U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_a, 9U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Xl, 10U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Yl, 11U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_P2, 12U, c3_x_sf_marshallOut,
    c3_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x2, 13U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y2, 14U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_edge, 15U, c3_m_sf_marshallOut,
    c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_I2, 16U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_J2, 17U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_Pxel_data, (const int32_T *)
    &c3_Pxel_sizes, NULL, 0, 18, (void *)c3_w_sf_marshallOut, (void *)
    c3_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Io, 19U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Jo, 20U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xTarget, 21U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_yTarget, 22U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_r, 23U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleSpan, 24U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleStep, 25U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_rangeMax, 26U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Xmax, 27U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Ymax, 28U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 29U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 30U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Tl, 31U, c3_h_sf_marshallOut,
    c3_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_map, 32U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p, 33U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  c3_Ymax = 25.0;
  c3_Xmax = 25.0;
  c3_rangeMax = 50.0;
  c3_angleStep = 0.017453292519943295;
  c3_angleSpan = 6.2831853071795862;
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 6);
  c3_N = 361.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 7);
  for (c3_i31 = 0; c3_i31 < 722; c3_i31++) {
    c3_p[c3_i31] = 0.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 8);
  c3_R = 100.0;
  c3_C = 100.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 9);
  c3_i = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 11);
  for (c3_i32 = 0; c3_i32 < 9; c3_i32++) {
    c3_b_a[c3_i32] = c3_Tl[c3_i32];
  }

  c3_eml_scalar_eg(chartInstance);
  c3_eml_scalar_eg(chartInstance);
  for (c3_i33 = 0; c3_i33 < 3; c3_i33++) {
    c3_P1[c3_i33] = 0.0;
  }

  for (c3_i34 = 0; c3_i34 < 3; c3_i34++) {
    c3_P1[c3_i34] = 0.0;
  }

  for (c3_i35 = 0; c3_i35 < 9; c3_i35++) {
    c3_dv8[c3_i35] = c3_b_a[c3_i35];
  }

  for (c3_i36 = 0; c3_i36 < 3; c3_i36++) {
    c3_dv9[c3_i36] = c3_b[c3_i36];
  }

  for (c3_i37 = 0; c3_i37 < 9; c3_i37++) {
    c3_dv10[c3_i37] = c3_dv8[c3_i37];
  }

  for (c3_i38 = 0; c3_i38 < 3; c3_i38++) {
    c3_dv11[c3_i38] = c3_dv9[c3_i38];
  }

  c3_b_eml_xgemm(chartInstance, c3_dv10, c3_dv11, c3_P1);
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 12);
  c3_x1 = c3_P1[0];
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 12);
  c3_y1 = c3_P1[1];
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 13);
  c3_XYtoIJ(chartInstance, c3_x1, c3_y1, 25.0, 25.0, 100.0, 100.0, &c3_b_I1,
            &c3_b_J1);
  c3_I1 = c3_b_I1;
  c3_J1 = c3_b_J1;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 14);
  c3_a = 0.0;
  c3_c_a = 0;
  while (c3_c_a < 361) {
    c3_a = (real_T)c3_c_a * 0.017453292519943295;
    CV_SCRIPT_FOR(1, 0, 1);
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 16);
    c3_x = c3_a;
    c3_b_x = c3_x;
    c3_b_x = muDoubleScalarCos(c3_b_x);
    c3_Xl = 50.0 * c3_b_x;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 17);
    c3_c_x = c3_a;
    c3_d_x = c3_c_x;
    c3_d_x = muDoubleScalarSin(c3_d_x);
    c3_Yl = 50.0 * c3_d_x;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 20);
    for (c3_i39 = 0; c3_i39 < 9; c3_i39++) {
      c3_b_a[c3_i39] = c3_Tl[c3_i39];
    }

    c3_b_Xl[0] = c3_Xl;
    c3_b_Xl[1] = c3_Yl;
    c3_b_Xl[2] = 1.0;
    for (c3_i40 = 0; c3_i40 < 3; c3_i40++) {
      c3_b_b[c3_i40] = c3_b_Xl[c3_i40];
    }

    c3_eml_scalar_eg(chartInstance);
    c3_eml_scalar_eg(chartInstance);
    for (c3_i41 = 0; c3_i41 < 3; c3_i41++) {
      c3_P2[c3_i41] = 0.0;
    }

    for (c3_i42 = 0; c3_i42 < 3; c3_i42++) {
      c3_P2[c3_i42] = 0.0;
    }

    for (c3_i43 = 0; c3_i43 < 9; c3_i43++) {
      c3_dv12[c3_i43] = c3_b_a[c3_i43];
    }

    for (c3_i44 = 0; c3_i44 < 3; c3_i44++) {
      c3_dv13[c3_i44] = c3_b_b[c3_i44];
    }

    for (c3_i45 = 0; c3_i45 < 9; c3_i45++) {
      c3_dv14[c3_i45] = c3_dv12[c3_i45];
    }

    for (c3_i46 = 0; c3_i46 < 3; c3_i46++) {
      c3_dv15[c3_i46] = c3_dv13[c3_i46];
    }

    c3_b_eml_xgemm(chartInstance, c3_dv14, c3_dv15, c3_P2);
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 21);
    c3_x2 = c3_P2[0];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 21);
    c3_y2 = c3_P2[1];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 24);
    c3_b_x1[0] = c3_x1;
    c3_b_x1[1] = c3_y1;
    c3_b_x1[2] = c3_x2 - c3_x1;
    c3_b_x1[3] = c3_y2 - c3_y1;
    c3_clipLine(chartInstance, c3_b_x1, c3_dv16);
    for (c3_i47 = 0; c3_i47 < 4; c3_i47++) {
      c3_edge[c3_i47] = c3_dv16[c3_i47];
    }

    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 26);
    c3_x2 = c3_edge[2];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 26);
    c3_y2 = c3_edge[3];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 32);
    c3_XYtoIJ(chartInstance, c3_x2, c3_y2, 25.0, 25.0, 100.0, 100.0, &c3_b_I2,
              &c3_b_J2);
    c3_I2 = c3_b_I2;
    c3_J2 = c3_b_J2;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 34);
    c3_p[_SFD_EML_ARRAY_BOUNDS_CHECK("p", (int32_T)c3_i, 1, 361, 1, 0) - 1] =
      c3_a;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 36);
    c3_c_I1[0] = c3_I1;
    c3_c_I1[1] = c3_J1;
    c3_c_I2[0] = c3_I2;
    c3_c_I2[1] = c3_J2;
    for (c3_i48 = 0; c3_i48 < 10000; c3_i48++) {
      c3_b_map[c3_i48] = c3_map[c3_i48];
    }

    c3_laserRange(chartInstance, c3_c_I1, c3_c_I2, c3_b_map, c3_tmp_data,
                  c3_tmp_sizes);
    c3_Pxel_sizes[0] = c3_tmp_sizes[0];
    c3_Pxel_sizes[1] = c3_tmp_sizes[1];
    c3_Pxel = c3_Pxel_sizes[0];
    c3_b_Pxel = c3_Pxel_sizes[1];
    c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
    for (c3_i49 = 0; c3_i49 <= c3_loop_ub; c3_i49++) {
      c3_Pxel_data[c3_i49] = c3_tmp_data[c3_i49];
    }

    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 37);
    c3_Io = c3_Pxel_data[0];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 37);
    c3_Jo = c3_Pxel_data[1];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 39);
    c3_e_x = c3_Io;
    c3_c_b = muDoubleScalarIsInf(c3_e_x);
    guard1 = false;
    if (CV_SCRIPT_COND(1, 0, c3_c_b)) {
      guard1 = true;
    } else {
      c3_f_x = c3_Jo;
      c3_d_b = muDoubleScalarIsInf(c3_f_x);
      if (CV_SCRIPT_COND(1, 1, c3_d_b)) {
        guard1 = true;
      } else {
        CV_SCRIPT_MCDC(1, 0, false);
        CV_SCRIPT_IF(1, 0, false);
        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 43);
        c3_b_i = c3_Io;
        c3_j = c3_Jo;
        c3_b_Xmax = 25.0;
        c3_b_Ymax = 25.0;
        c3_b_R = 100.0;
        c3_b_C = 100.0;
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_h_debug_family_names,
          c3_b_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U,
          c3_e_sf_marshallOut, c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U,
          c3_e_sf_marshallOut, c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_i, 2U, c3_e_sf_marshallOut,
          c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_j, 3U, c3_e_sf_marshallOut,
          c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_Xmax, 4U, c3_e_sf_marshallOut,
          c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_Ymax, 5U, c3_e_sf_marshallOut,
          c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_R, 6U, c3_e_sf_marshallOut,
          c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_C, 7U, c3_e_sf_marshallOut,
          c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_xTarget, 8U,
          c3_e_sf_marshallOut, c3_e_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_yTarget, 9U,
          c3_e_sf_marshallOut, c3_e_sf_marshallIn);
        CV_SCRIPT_FCN(7, 0);
        _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 3);
        c3_A = 25.0 * (c3_b_i - 1.0);
        c3_g_x = c3_A;
        c3_h_x = c3_g_x;
        c3_i_x = c3_h_x;
        c3_y = c3_i_x / 99.0;
        c3_b_yTarget = 25.0 - c3_y;
        _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 4);
        c3_b_A = 25.0 * (c3_j - 1.0);
        c3_j_x = c3_b_A;
        c3_k_x = c3_j_x;
        c3_l_x = c3_k_x;
        c3_b_xTarget = c3_l_x / 99.0;
        _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, -4);
        _SFD_SYMBOL_SCOPE_POP();
        c3_xTarget = c3_b_xTarget;
        c3_yTarget = c3_b_yTarget;
        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 45);
        c3_m_x = c3_mpower(chartInstance, c3_x1 - c3_xTarget) + c3_mpower
          (chartInstance, c3_y1 - c3_yTarget);
        c3_r = c3_m_x;
        if (c3_r < 0.0) {
          c3_eml_error(chartInstance);
        }

        c3_n_x = c3_r;
        c3_r = c3_n_x;
        c3_r = muDoubleScalarSqrt(c3_r);
        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 46);
        if (CV_SCRIPT_IF(1, 1, c3_r > 50.0)) {
          _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 47);
          c3_r = 50.0;
        }

        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 49);
        c3_p[(int32_T)c3_i + 360] = c3_r;
      }
    }

    if (guard1 == true) {
      CV_SCRIPT_MCDC(1, 0, true);
      CV_SCRIPT_IF(1, 0, true);
      _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 40);
      c3_p[(int32_T)c3_i + 360] = 50.0;
    }

    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 65);
    c3_i++;
    c3_c_a++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_SCRIPT_FOR(1, 0, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, -65);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_XYtoIJ(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x, real_T c3_y, real_T c3_Xmax, real_T c3_Ymax,
                      real_T c3_R, real_T c3_C, real_T *c3_i, real_T *c3_j)
{
  uint32_T c3_debug_family_var_map[10];
  real_T c3_nargin = 6.0;
  real_T c3_nargout = 2.0;
  real_T c3_A;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_b_y;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_b_A;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_i_x;
  real_T c3_c_y;
  real_T c3_j_x;
  real_T c3_k_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_c_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x, 2U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y, 3U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Xmax, 4U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Ymax, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R, 6U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_i, 8U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_j, 9U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 4);
  c3_A = 25.0 - c3_y;
  c3_b_x = c3_A;
  c3_c_x = c3_b_x;
  c3_d_x = c3_c_x;
  c3_b_y = c3_d_x / 25.0;
  c3_e_x = c3_b_y * 99.0;
  c3_f_x = c3_e_x;
  c3_f_x = muDoubleScalarRound(c3_f_x);
  *c3_i = c3_f_x + 1.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 5);
  c3_b_A = c3_x;
  c3_g_x = c3_b_A;
  c3_h_x = c3_g_x;
  c3_i_x = c3_h_x;
  c3_c_y = c3_i_x / 25.0;
  c3_j_x = c3_c_y * 99.0;
  c3_k_x = c3_j_x;
  c3_k_x = muDoubleScalarRound(c3_k_x);
  *c3_j = c3_k_x + 1.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_clipLine(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_line[4], real_T c3_edge[4])
{
  uint32_T c3_debug_family_var_map[28];
  real_T c3_nLines;
  real_T c3_nBoxes;
  real_T c3_i;
  real_T c3_xmin;
  real_T c3_xmax;
  real_T c3_ymin;
  real_T c3_ymax;
  real_T c3_delta;
  real_T c3_px1[2];
  real_T c3_px2[2];
  real_T c3_py1[2];
  real_T c3_py2[2];
  real_T c3_points[8];
  int32_T c3_pos_sizes;
  real_T c3_pos_data[4];
  int32_T c3_inds_sizes;
  real_T c3_inds_data[4];
  real_T c3_ind;
  real_T c3_inter1[2];
  real_T c3_inter2[2];
  real_T c3_midX;
  boolean_T c3_xOk;
  real_T c3_midY;
  boolean_T c3_yOk;
  int32_T c3_points_sizes[2];
  real_T c3_points_data[8];
  real_T c3_box[4];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i50;
  static real_T c3_dv17[4] = { 0.0, 25.0, 0.0, 25.0 };

  int32_T c3_i51;
  int32_T c3_i52;
  real_T c3_b_line[4];
  real_T c3_dv18[4];
  real_T c3_dv19[2];
  int32_T c3_i53;
  int32_T c3_i54;
  real_T c3_c_line[4];
  real_T c3_dv20[4];
  real_T c3_dv21[2];
  int32_T c3_i55;
  int32_T c3_i56;
  real_T c3_d_line[4];
  real_T c3_dv22[4];
  real_T c3_dv23[2];
  int32_T c3_i57;
  int32_T c3_i58;
  real_T c3_e_line[4];
  real_T c3_dv24[4];
  real_T c3_dv25[2];
  int32_T c3_i59;
  int32_T c3_i60;
  int32_T c3_i61;
  int32_T c3_i62;
  int32_T c3_i63;
  int32_T c3_i64;
  int32_T c3_i65;
  int32_T c3_i66;
  int32_T c3_i67;
  int32_T c3_i68;
  real_T c3_b_points[4];
  boolean_T c3_x[4];
  int32_T c3_k;
  int32_T c3_b_i;
  int32_T c3_c_i;
  int32_T c3_a;
  int32_T c3_b_a;
  const mxArray *c3_y = NULL;
  int32_T c3_iidx_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i69;
  int32_T c3_iidx_data[4];
  int32_T c3_y_sizes;
  int32_T c3_j;
  int32_T c3_d_i;
  int32_T c3_e_i;
  int32_T c3_y_data[4];
  int32_T c3_c_a;
  int32_T c3_d_a;
  int32_T c3_i70;
  int32_T c3_b_loop_ub;
  int32_T c3_i71;
  int32_T c3_point_sizes[2];
  int32_T c3_point;
  int32_T c3_b_point;
  int32_T c3_c_loop_ub;
  int32_T c3_i72;
  real_T c3_point_data[8];
  int32_T c3_i73;
  real_T c3_f_line[4];
  uint32_T c3_b_debug_family_var_map[12];
  real_T c3_vx;
  real_T c3_vy;
  int32_T c3_dx_sizes;
  real_T c3_dx_data[4];
  int32_T c3_dy_sizes;
  real_T c3_dy_data[4];
  real_T c3_b_delta;
  boolean_T c3_invalidLine;
  real_T c3_b_nargin = 2.0;
  real_T c3_b_nargout = 1.0;
  int32_T c3_i74;
  int32_T c3_b_point_sizes;
  int32_T c3_d_loop_ub;
  int32_T c3_i75;
  real_T c3_b_point_data[4];
  int32_T c3_tmp_sizes;
  real_T c3_tmp_data[4];
  int32_T c3_e_loop_ub;
  int32_T c3_i76;
  int32_T c3_i77;
  int32_T c3_c_point_sizes;
  int32_T c3_f_loop_ub;
  int32_T c3_i78;
  real_T c3_c_point_data[4];
  int32_T c3_b_tmp_sizes;
  real_T c3_b_tmp_data[4];
  int32_T c3_g_loop_ub;
  int32_T c3_i79;
  real_T c3_dv26[1];
  int32_T c3_c_tmp_sizes[2];
  int32_T c3_c_tmp_data[1];
  int32_T c3_h_loop_ub;
  int32_T c3_i80;
  int32_T c3_b_dx_sizes;
  int32_T c3_i_loop_ub;
  int32_T c3_i81;
  real_T c3_b_dx_data[4];
  int32_T c3_x_sizes;
  real_T c3_x_data[4];
  int32_T c3_b_dy_sizes;
  int32_T c3_j_loop_ub;
  int32_T c3_i82;
  real_T c3_b_dy_data[4];
  int32_T c3_b_inds_sizes;
  real_T c3_b_inds_data[4];
  int32_T c3_b_x_sizes;
  int32_T c3_k_loop_ub;
  int32_T c3_i83;
  real_T c3_b_x_data[4];
  int32_T c3_d_tmp_sizes;
  real_T c3_d_tmp_data[4];
  int32_T c3_l_loop_ub;
  int32_T c3_i84;
  int32_T c3_i85;
  int32_T c3_m_loop_ub;
  int32_T c3_i86;
  int32_T c3_pos;
  int32_T c3_b_pos[2];
  int32_T c3_iidx[2];
  int32_T c3_n_loop_ub;
  int32_T c3_i87;
  int32_T c3_o_loop_ub;
  int32_T c3_i88;
  int32_T c3_e_tmp_sizes[2];
  int32_T c3_p_loop_ub;
  int32_T c3_i89;
  int32_T c3_c_x_sizes;
  int32_T c3_q_loop_ub;
  int32_T c3_i90;
  real_T c3_c_x_data[4];
  int32_T c3_r_loop_ub;
  int32_T c3_i91;
  int32_T c3_s_loop_ub;
  int32_T c3_i92;
  int32_T c3_t_loop_ub;
  int32_T c3_i93;
  int32_T c3_c_points;
  int32_T c3_b_points_sizes[2];
  int32_T c3_i94;
  int32_T c3_u_loop_ub;
  int32_T c3_i95;
  real_T c3_b_points_data[8];
  int32_T c3_i96;
  int32_T c3_v_loop_ub;
  int32_T c3_i97;
  real_T c3_A;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  int32_T c3_b_ind;
  int32_T c3_i98;
  int32_T c3_c_ind;
  int32_T c3_i99;
  int32_T c3_i100;
  int32_T c3_i101;
  int32_T c3_i102;
  int32_T c3_i103;
  real_T c3_b_edge[2];
  boolean_T c3_b0;
  int32_T c3_i104;
  int32_T c3_i105;
  real_T c3_c_edge[2];
  boolean_T c3_b1;
  boolean_T c3_b2;
  int32_T c3_i106;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 28U, 29U, c3_f_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c3_u_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_nLines, 1U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_nBoxes, 2U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_i, 3U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xmin, 4U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xmax, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_ymin, 6U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_ymax, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_delta, 8U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_px1, 9U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_px2, 10U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_py1, 11U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_py2, 12U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_points, MAX_uint32_T,
    c3_t_sf_marshallOut, c3_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_pos_data, (const int32_T *)
    &c3_pos_sizes, NULL, 0, 14, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_inds_data, (const int32_T *)
    &c3_inds_sizes, NULL, 0, 15, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_ind, 16U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_inter1, 17U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_inter2, 18U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_midX, 19U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xOk, 20U, c3_o_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_midY, 21U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_yOk, 22U, c3_o_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_points_data, (const int32_T *)
    &c3_points_sizes, NULL, 0, -1, (void *)c3_r_sf_marshallOut, (void *)
    c3_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_box, 23U, c3_m_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 24U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 25U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line, 26U, c3_m_sf_marshallOut,
    c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_edge, 27U, c3_m_sf_marshallOut,
    c3_m_sf_marshallIn);
  for (c3_i50 = 0; c3_i50 < 4; c3_i50++) {
    c3_box[c3_i50] = c3_dv17[c3_i50];
  }

  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 42);
  c3_nLines = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 43);
  c3_nBoxes = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 44);
  CV_SCRIPT_COND(3, 0, true);
  CV_SCRIPT_COND(3, 1, false);
  CV_SCRIPT_MCDC(3, 0, false);
  CV_SCRIPT_IF(3, 0, false);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 46);
  CV_SCRIPT_COND(3, 2, true);
  CV_SCRIPT_COND(3, 3, false);
  CV_SCRIPT_MCDC(3, 1, false);
  CV_SCRIPT_IF(3, 1, false);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 48);
  CV_SCRIPT_IF(3, 2, false);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 53);
  c3_nLines = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 54);
  for (c3_i51 = 0; c3_i51 < 4; c3_i51++) {
    c3_edge[c3_i51] = 0.0;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 57);
  c3_i = 1.0;
  CV_SCRIPT_FOR(3, 0, 1);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 59);
  c3_xmin = 0.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 60);
  c3_xmax = 25.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 61);
  c3_ymin = 0.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 62);
  c3_ymax = 25.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 66);
  c3_delta = c3_hypot(chartInstance, c3_line[2], c3_line[3]);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 69);
  for (c3_i52 = 0; c3_i52 < 4; c3_i52++) {
    c3_b_line[c3_i52] = c3_line[c3_i52];
  }

  c3_dv18[0] = 0.0;
  c3_dv18[1] = 0.0;
  c3_dv18[2] = c3_delta;
  c3_dv18[3] = 0.0;
  c3_intersectLines(chartInstance, c3_b_line, c3_dv18, c3_dv19);
  for (c3_i53 = 0; c3_i53 < 2; c3_i53++) {
    c3_px1[c3_i53] = c3_dv19[c3_i53];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 70);
  for (c3_i54 = 0; c3_i54 < 4; c3_i54++) {
    c3_c_line[c3_i54] = c3_line[c3_i54];
  }

  c3_dv20[0] = 25.0;
  c3_dv20[1] = 0.0;
  c3_dv20[2] = 0.0;
  c3_dv20[3] = c3_delta;
  c3_intersectLines(chartInstance, c3_c_line, c3_dv20, c3_dv21);
  for (c3_i55 = 0; c3_i55 < 2; c3_i55++) {
    c3_px2[c3_i55] = c3_dv21[c3_i55];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 71);
  for (c3_i56 = 0; c3_i56 < 4; c3_i56++) {
    c3_d_line[c3_i56] = c3_line[c3_i56];
  }

  c3_dv22[0] = 25.0;
  c3_dv22[1] = 25.0;
  c3_dv22[2] = -c3_delta;
  c3_dv22[3] = 0.0;
  c3_intersectLines(chartInstance, c3_d_line, c3_dv22, c3_dv23);
  for (c3_i57 = 0; c3_i57 < 2; c3_i57++) {
    c3_py1[c3_i57] = c3_dv23[c3_i57];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 72);
  for (c3_i58 = 0; c3_i58 < 4; c3_i58++) {
    c3_e_line[c3_i58] = c3_line[c3_i58];
  }

  c3_dv24[0] = 0.0;
  c3_dv24[1] = 25.0;
  c3_dv24[2] = 0.0;
  c3_dv24[3] = -c3_delta;
  c3_intersectLines(chartInstance, c3_e_line, c3_dv24, c3_dv25);
  for (c3_i59 = 0; c3_i59 < 2; c3_i59++) {
    c3_py2[c3_i59] = c3_dv25[c3_i59];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 75);
  c3_i60 = 0;
  for (c3_i61 = 0; c3_i61 < 2; c3_i61++) {
    c3_points[c3_i60] = c3_px1[c3_i61];
    c3_i60 += 4;
  }

  c3_i62 = 0;
  for (c3_i63 = 0; c3_i63 < 2; c3_i63++) {
    c3_points[c3_i62 + 1] = c3_px2[c3_i63];
    c3_i62 += 4;
  }

  c3_i64 = 0;
  for (c3_i65 = 0; c3_i65 < 2; c3_i65++) {
    c3_points[c3_i64 + 2] = c3_py1[c3_i65];
    c3_i64 += 4;
  }

  c3_i66 = 0;
  for (c3_i67 = 0; c3_i67 < 2; c3_i67++) {
    c3_points[c3_i66 + 3] = c3_py2[c3_i67];
    c3_i66 += 4;
  }

  _SFD_SYMBOL_SWITCH(13U, 13U);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 76);
  for (c3_i68 = 0; c3_i68 < 4; c3_i68++) {
    c3_b_points[c3_i68] = c3_points[c3_i68];
  }

  c3_isfinite(chartInstance, c3_b_points, c3_x);
  c3_k = 0;
  for (c3_b_i = 1; c3_b_i < 5; c3_b_i++) {
    c3_c_i = c3_b_i - 1;
    if (c3_x[c3_c_i]) {
      c3_a = c3_k;
      c3_b_a = c3_a + 1;
      c3_k = c3_b_a;
    }
  }

  if (c3_k <= 4) {
  } else {
    c3_y = NULL;
    sf_mex_assign(&c3_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U, 0U,
      2, 1, strlen("Assertion failed.")), false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14, c3_y);
  }

  c3_iidx_sizes = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c3_k);
  c3_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c3_k) - 1;
  for (c3_i69 = 0; c3_i69 <= c3_loop_ub; c3_i69++) {
    c3_iidx_data[c3_i69] = 0;
  }

  c3_y_sizes = c3_iidx_sizes;
  c3_j = 1;
  for (c3_d_i = 1; c3_d_i < 5; c3_d_i++) {
    c3_e_i = c3_d_i;
    if (c3_x[c3_e_i - 1]) {
      c3_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_j, 1, c3_y_sizes, 1, 0) - 1] =
        c3_e_i;
      c3_c_a = c3_j;
      c3_d_a = c3_c_a + 1;
      c3_j = c3_d_a;
    }
  }

  c3_points_sizes[0] = c3_y_sizes;
  c3_points_sizes[1] = 2;
  for (c3_i70 = 0; c3_i70 < 2; c3_i70++) {
    c3_b_loop_ub = c3_y_sizes - 1;
    for (c3_i71 = 0; c3_i71 <= c3_b_loop_ub; c3_i71++) {
      c3_points_data[c3_i71 + c3_points_sizes[0] * c3_i70] = c3_points
        [(c3_y_data[c3_i71] + (c3_i70 << 2)) - 1];
    }
  }

  _SFD_SYMBOL_SWITCH(13U, 23U);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 79);
  c3_point_sizes[0] = c3_points_sizes[0];
  c3_point_sizes[1] = 2;
  c3_point = c3_point_sizes[0];
  c3_b_point = c3_point_sizes[1];
  c3_c_loop_ub = c3_points_sizes[0] * c3_points_sizes[1] - 1;
  for (c3_i72 = 0; c3_i72 <= c3_c_loop_ub; c3_i72++) {
    c3_point_data[c3_i72] = c3_points_data[c3_i72];
  }

  for (c3_i73 = 0; c3_i73 < 4; c3_i73++) {
    c3_f_line[c3_i73] = c3_line[c3_i73];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c3_e_debug_family_names,
    c3_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c3_s_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_vx, 1U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_vy, 2U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_dx_data, (const int32_T *)
    &c3_dx_sizes, NULL, 0, 3, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_dy_data, (const int32_T *)
    &c3_dy_sizes, NULL, 0, 4, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_delta, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_invalidLine, 6U, c3_o_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 8U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_point_data, (const int32_T *)
    &c3_point_sizes, NULL, 1, 9, (void *)c3_r_sf_marshallOut, (void *)
    c3_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_f_line, 10U, c3_m_sf_marshallOut,
    c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_pos_data, (const int32_T *)
    &c3_pos_sizes, NULL, 0, 11, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 52);
  CV_SCRIPT_COND(5, 0, true);
  CV_SCRIPT_MCDC(5, 0, false);
  CV_SCRIPT_IF(5, 0, false);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 76);
  c3_vx = c3_f_line[2];
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 77);
  c3_vy = c3_f_line[3];
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 80);
  c3_i74 = c3_point_sizes[0];
  c3_b_point_sizes = c3_i74;
  c3_d_loop_ub = c3_i74 - 1;
  for (c3_i75 = 0; c3_i75 <= c3_d_loop_ub; c3_i75++) {
    c3_b_point_data[c3_i75] = c3_point_data[c3_i75];
  }

  c3_b_bsxfun(chartInstance, c3_b_point_data, c3_b_point_sizes, c3_f_line[0],
              c3_tmp_data, &c3_tmp_sizes);
  c3_dx_sizes = c3_tmp_sizes;
  c3_e_loop_ub = c3_tmp_sizes - 1;
  for (c3_i76 = 0; c3_i76 <= c3_e_loop_ub; c3_i76++) {
    c3_dx_data[c3_i76] = c3_tmp_data[c3_i76];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 81);
  c3_i77 = c3_point_sizes[0];
  c3_c_point_sizes = c3_i77;
  c3_f_loop_ub = c3_i77 - 1;
  for (c3_i78 = 0; c3_i78 <= c3_f_loop_ub; c3_i78++) {
    c3_c_point_data[c3_i78] = c3_point_data[c3_i78 + c3_point_sizes[0]];
  }

  c3_b_bsxfun(chartInstance, c3_c_point_data, c3_c_point_sizes, c3_f_line[1],
              c3_b_tmp_data, &c3_b_tmp_sizes);
  c3_dy_sizes = c3_b_tmp_sizes;
  c3_g_loop_ub = c3_b_tmp_sizes - 1;
  for (c3_i79 = 0; c3_i79 <= c3_g_loop_ub; c3_i79++) {
    c3_dy_data[c3_i79] = c3_b_tmp_data[c3_i79];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 86);
  c3_b_delta = c3_vx * c3_vx + c3_vy * c3_vy;
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 87);
  c3_invalidLine = (c3_b_delta < 2.2204460492503131E-16);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 88);
  c3_dv26[0] = c3_b_delta;
  c3_eml_li_find(chartInstance, c3_invalidLine, c3_c_tmp_data, c3_c_tmp_sizes);
  c3_h_loop_ub = c3_c_tmp_sizes[0] * c3_c_tmp_sizes[1] - 1;
  for (c3_i80 = 0; c3_i80 <= c3_h_loop_ub; c3_i80++) {
    c3_dv26[c3_c_tmp_data[c3_i80] - 1] = 1.0;
  }

  c3_b_delta = c3_dv26[0];
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 92);
  c3_b_dx_sizes = c3_dx_sizes;
  c3_i_loop_ub = c3_dx_sizes - 1;
  for (c3_i81 = 0; c3_i81 <= c3_i_loop_ub; c3_i81++) {
    c3_b_dx_data[c3_i81] = c3_dx_data[c3_i81];
  }

  c3_c_bsxfun(chartInstance, c3_b_dx_data, c3_b_dx_sizes, c3_vx, c3_x_data,
              &c3_x_sizes);
  c3_b_dy_sizes = c3_dy_sizes;
  c3_j_loop_ub = c3_dy_sizes - 1;
  for (c3_i82 = 0; c3_i82 <= c3_j_loop_ub; c3_i82++) {
    c3_b_dy_data[c3_i82] = c3_dy_data[c3_i82];
  }

  c3_c_bsxfun(chartInstance, c3_b_dy_data, c3_b_dy_sizes, c3_vy, c3_b_inds_data,
              &c3_b_inds_sizes);
  _SFD_SIZE_EQ_CHECK_1D(c3_x_sizes, c3_b_inds_sizes);
  c3_b_x_sizes = c3_x_sizes;
  c3_k_loop_ub = c3_x_sizes - 1;
  for (c3_i83 = 0; c3_i83 <= c3_k_loop_ub; c3_i83++) {
    c3_b_x_data[c3_i83] = c3_x_data[c3_i83] + c3_b_inds_data[c3_i83];
  }

  c3_d_bsxfun(chartInstance, c3_b_x_data, c3_b_x_sizes, c3_b_delta,
              c3_d_tmp_data, &c3_d_tmp_sizes);
  c3_pos_sizes = c3_d_tmp_sizes;
  c3_l_loop_ub = c3_d_tmp_sizes - 1;
  for (c3_i84 = 0; c3_i84 <= c3_l_loop_ub; c3_i84++) {
    c3_pos_data[c3_i84] = c3_d_tmp_data[c3_i84];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 96);
  c3_i85 = c3_pos_sizes;
  c3_iidx_sizes = c3_i85;
  c3_m_loop_ub = c3_i85 - 1;
  for (c3_i86 = 0; c3_i86 <= c3_m_loop_ub; c3_i86++) {
    c3_iidx_data[c3_i86] = 1 + c3_i86;
  }

  c3_eml_li_find(chartInstance, c3_invalidLine, c3_c_tmp_data, c3_c_tmp_sizes);
  c3_pos = c3_pos_sizes;
  c3_b_pos[0] = c3_pos;
  c3_b_pos[1] = 1;
  c3_iidx[0] = c3_iidx_sizes;
  c3_iidx[1] = c3_c_tmp_sizes[1];
  c3_n_loop_ub = c3_iidx[1] - 1;
  for (c3_i87 = 0; c3_i87 <= c3_n_loop_ub; c3_i87++) {
    c3_o_loop_ub = c3_iidx[0] - 1;
    for (c3_i88 = 0; c3_i88 <= c3_o_loop_ub; c3_i88++) {
      c3_e_tmp_sizes[0] = c3_b_pos[0];
      c3_e_tmp_sizes[1] = c3_b_pos[1];
      c3_pos_data[(c3_iidx_data[c3_i88] + c3_e_tmp_sizes[0] *
                   (c3_c_tmp_data[c3_c_tmp_sizes[0] * c3_i87] - 1)) - 1] = 0.0;
    }
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, -96);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 80);
  c3_x_sizes = c3_pos_sizes;
  c3_p_loop_ub = c3_pos_sizes - 1;
  for (c3_i89 = 0; c3_i89 <= c3_p_loop_ub; c3_i89++) {
    c3_x_data[c3_i89] = c3_pos_data[c3_i89];
  }

  c3_c_x_sizes = c3_x_sizes;
  c3_q_loop_ub = c3_x_sizes - 1;
  for (c3_i90 = 0; c3_i90 <= c3_q_loop_ub; c3_i90++) {
    c3_c_x_data[c3_i90] = c3_x_data[c3_i90];
  }

  c3_eml_sort(chartInstance, c3_c_x_data, c3_c_x_sizes, c3_x_data, &c3_x_sizes,
              c3_iidx_data, &c3_iidx_sizes);
  c3_b_inds_sizes = c3_iidx_sizes;
  c3_r_loop_ub = c3_iidx_sizes - 1;
  for (c3_i91 = 0; c3_i91 <= c3_r_loop_ub; c3_i91++) {
    c3_b_inds_data[c3_i91] = (real_T)c3_iidx_data[c3_i91];
  }

  c3_pos_sizes = c3_x_sizes;
  c3_s_loop_ub = c3_x_sizes - 1;
  for (c3_i92 = 0; c3_i92 <= c3_s_loop_ub; c3_i92++) {
    c3_pos_data[c3_i92] = c3_x_data[c3_i92];
  }

  c3_inds_sizes = c3_b_inds_sizes;
  c3_t_loop_ub = c3_b_inds_sizes - 1;
  for (c3_i93 = 0; c3_i93 <= c3_t_loop_ub; c3_i93++) {
    c3_inds_data[c3_i93] = c3_b_inds_data[c3_i93];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 81);
  c3_c_points = c3_points_sizes[0];
  c3_b_points_sizes[0] = c3_inds_sizes;
  c3_b_points_sizes[1] = 2;
  for (c3_i94 = 0; c3_i94 < 2; c3_i94++) {
    c3_u_loop_ub = c3_inds_sizes - 1;
    for (c3_i95 = 0; c3_i95 <= c3_u_loop_ub; c3_i95++) {
      c3_b_points_data[c3_i95 + c3_b_points_sizes[0] * c3_i94] = c3_points_data
        [(_SFD_EML_ARRAY_BOUNDS_CHECK("points", (int32_T)c3_inds_data[c3_i95], 1,
           c3_c_points, 1, 0) + c3_points_sizes[0] * c3_i94) - 1];
    }
  }

  c3_points_sizes[0] = c3_b_points_sizes[0];
  c3_points_sizes[1] = 2;
  for (c3_i96 = 0; c3_i96 < 2; c3_i96++) {
    c3_v_loop_ub = c3_b_points_sizes[0] - 1;
    for (c3_i97 = 0; c3_i97 <= c3_v_loop_ub; c3_i97++) {
      c3_points_data[c3_i97 + c3_points_sizes[0] * c3_i96] =
        c3_b_points_data[c3_i97 + c3_b_points_sizes[0] * c3_i96];
    }
  }

  _SFD_SYMBOL_SWITCH(13U, 23U);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 84);
  c3_A = (real_T)c3_points_sizes[0];
  c3_b_x = c3_A;
  c3_c_x = c3_b_x;
  c3_d_x = c3_c_x;
  c3_ind = c3_d_x / 2.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 85);
  c3_b_ind = _SFD_EML_ARRAY_BOUNDS_CHECK("points", (int32_T)_SFD_INTEGER_CHECK(
    "ind", c3_ind), 1, c3_points_sizes[0], 1, 0) - 1;
  for (c3_i98 = 0; c3_i98 < 2; c3_i98++) {
    c3_inter1[c3_i98] = c3_points_data[c3_b_ind + c3_points_sizes[0] * c3_i98];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 86);
  c3_c_ind = _SFD_EML_ARRAY_BOUNDS_CHECK("points", (int32_T)(c3_ind + 1.0), 1,
    c3_points_sizes[0], 1, 0) - 1;
  for (c3_i99 = 0; c3_i99 < 2; c3_i99++) {
    c3_inter2[c3_i99] = c3_points_data[c3_c_ind + c3_points_sizes[0] * c3_i99];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 87);
  for (c3_i100 = 0; c3_i100 < 2; c3_i100++) {
    c3_edge[c3_i100] = c3_inter1[c3_i100];
  }

  for (c3_i101 = 0; c3_i101 < 2; c3_i101++) {
    c3_edge[c3_i101 + 2] = c3_inter2[c3_i101];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 90);
  c3_i102 = 0;
  for (c3_i103 = 0; c3_i103 < 2; c3_i103++) {
    c3_b_edge[c3_i103] = c3_edge[c3_i102];
    c3_i102 += 2;
  }

  c3_midX = c3_mean(chartInstance, c3_b_edge);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 91);
  guard3 = false;
  if (0.0 <= c3_midX) {
    if (c3_midX <= 25.0) {
      c3_b0 = true;
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3 == true) {
    c3_b0 = false;
  }

  c3_xOk = c3_b0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 92);
  c3_i104 = 0;
  for (c3_i105 = 0; c3_i105 < 2; c3_i105++) {
    c3_c_edge[c3_i105] = c3_edge[c3_i104 + 1];
    c3_i104 += 2;
  }

  c3_midY = c3_mean(chartInstance, c3_c_edge);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 93);
  guard2 = false;
  if (0.0 <= c3_midY) {
    if (c3_midY <= 25.0) {
      c3_b1 = true;
    } else {
      guard2 = true;
    }
  } else {
    guard2 = true;
  }

  if (guard2 == true) {
    c3_b1 = false;
  }

  c3_yOk = c3_b1;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 96);
  guard1 = false;
  if (CV_SCRIPT_COND(3, 4, c3_xOk)) {
    if (CV_SCRIPT_COND(3, 5, c3_yOk)) {
      c3_b2 = true;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    c3_b2 = false;
  }

  if (CV_SCRIPT_IF(3, 3, CV_SCRIPT_MCDC(3, 2, !c3_b2))) {
    _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 97);
    for (c3_i106 = 0; c3_i106 < 4; c3_i106++) {
      c3_edge[c3_i106] = rtNaN;
    }
  }

  CV_SCRIPT_FOR(3, 0, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, -97);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_intersectLines(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_line1[4], real_T c3_line2[4], real_T c3_point[2])
{
  uint32_T c3_debug_family_var_map[26];
  real_T c3_tol;
  real_T c3_N1;
  real_T c3_N2;
  real_T c3_N;
  real_T c3_dx;
  real_T c3_dy;
  real_T c3_denom;
  boolean_T c3_par;
  boolean_T c3_col;
  real_T c3_x0;
  real_T c3_y0;
  boolean_T c3_inds;
  real_T c3_x1;
  real_T c3_y1;
  real_T c3_dx1;
  real_T c3_dy1;
  real_T c3_x2;
  real_T c3_y2;
  real_T c3_dx2;
  real_T c3_dy2;
  int32_T c3_denom_sizes[2];
  real_T c3_denom_data[1];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_y;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_b_y;
  boolean_T c3_b3;
  boolean_T c3_b_par;
  real_T c3_dv27[1];
  int32_T c3_tmp_sizes[2];
  int32_T c3_tmp_data[1];
  int32_T c3_loop_ub;
  int32_T c3_i107;
  real_T c3_dv28[1];
  int32_T c3_b_loop_ub;
  int32_T c3_i108;
  real_T c3_dv29[1];
  boolean_T c3_c_par;
  boolean_T c3_b4;
  int32_T c3_c_loop_ub;
  int32_T c3_i109;
  real_T c3_dv30[1];
  boolean_T c3_d_par;
  boolean_T c3_b5;
  int32_T c3_d_loop_ub;
  int32_T c3_i110;
  boolean_T c3_e_x;
  boolean_T c3_f_x;
  boolean_T c3_c_y;
  int32_T c3_i111;
  int32_T c3_i112;
  int32_T c3_i113;
  int32_T c3_i114;
  int32_T c3_e_loop_ub;
  int32_T c3_i115;
  int32_T c3_b_denom;
  int32_T c3_c_denom;
  int32_T c3_f_loop_ub;
  int32_T c3_i116;
  real_T c3_g_x;
  int32_T c3_y_sizes[2];
  int32_T c3_d_y;
  int32_T c3_e_y;
  int32_T c3_g_loop_ub;
  int32_T c3_i117;
  real_T c3_y_data[1];
  real_T c3_h_x;
  real_T c3_i_x;
  int32_T c3_f_y;
  int32_T c3_g_y;
  int32_T c3_h_y;
  int32_T c3_i_y;
  int32_T c3_h_loop_ub;
  int32_T c3_i118;
  real_T c3_dv31[1];
  int32_T c3_i_loop_ub;
  int32_T c3_i119;
  real_T c3_j_x;
  int32_T c3_j_y;
  int32_T c3_k_y;
  int32_T c3_j_loop_ub;
  int32_T c3_i120;
  real_T c3_k_x;
  real_T c3_l_x;
  int32_T c3_l_y;
  int32_T c3_m_y;
  int32_T c3_n_y;
  int32_T c3_o_y;
  int32_T c3_k_loop_ub;
  int32_T c3_i121;
  real_T c3_dv32[1];
  int32_T c3_l_loop_ub;
  int32_T c3_i122;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 26U, 27U, c3_d_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c3_p_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tol, 1U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N1, 2U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N2, 3U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N, 4U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy, 6U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_denom, MAX_uint32_T,
    c3_e_sf_marshallOut, c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_par, 8U, c3_o_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_col, 9U, c3_o_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x0, 10U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y0, 11U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_inds, 12U, c3_o_sf_marshallOut,
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x1, 13U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y1, 14U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx1, 15U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy1, 16U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x2, 17U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y2, 18U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx2, 19U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy2, 20U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_denom_data, (const int32_T *)
    &c3_denom_sizes, NULL, 0, -1, (void *)c3_n_sf_marshallOut, (void *)
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 21U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 22U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line1, 23U, c3_m_sf_marshallOut,
    c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line2, 24U, c3_m_sf_marshallOut,
    c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_point, 25U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  CV_SCRIPT_FCN(4, 0);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 46);
  c3_tol = 1.0E-14;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 47);
  CV_SCRIPT_COND(4, 0, true);
  CV_SCRIPT_MCDC(4, 0, false);
  CV_SCRIPT_IF(4, 0, false);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 52);
  c3_N1 = 1.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 53);
  c3_N2 = 1.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 54);
  c3_N = 1.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 55);
  CV_SCRIPT_COND(4, 1, false);
  CV_SCRIPT_MCDC(4, 1, false);
  CV_SCRIPT_IF(4, 1, false);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 64);
  c3_dx = c3_bsxfun(chartInstance, c3_line2[0], c3_line1[0]);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 65);
  c3_dy = c3_bsxfun(chartInstance, c3_line2[1], c3_line1[1]);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 68);
  c3_denom = c3_line1[2] * c3_line2[3] - c3_line2[2] * c3_line1[3];
  _SFD_SYMBOL_SWITCH(7U, 7U);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 69);
  c3_x = c3_denom;
  c3_b_x = c3_x;
  c3_y = muDoubleScalarAbs(c3_b_x);
  c3_par = (c3_y < 1.0E-14);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 72);
  c3_c_x = c3_dx * c3_line1[3] - c3_dy * c3_line1[2];
  c3_d_x = c3_c_x;
  c3_b_y = muDoubleScalarAbs(c3_d_x);
  c3_b3 = (c3_b_y < 1.0E-14);
  c3_b_par = c3_par;
  c3_col = (c3_b3 && c3_b_par);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 75);
  c3_x0 = 0.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 76);
  c3_y0 = 0.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 79);
  c3_dv27[0] = 0.0;
  c3_eml_li_find(chartInstance, c3_col, c3_tmp_data, c3_tmp_sizes);
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i107 = 0; c3_i107 <= c3_loop_ub; c3_i107++) {
    c3_dv27[c3_tmp_data[c3_i107] - 1] = rtInf;
  }

  c3_x0 = c3_dv27[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 80);
  c3_dv28[0] = 0.0;
  c3_eml_li_find(chartInstance, c3_col, c3_tmp_data, c3_tmp_sizes);
  c3_b_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i108 = 0; c3_i108 <= c3_b_loop_ub; c3_i108++) {
    c3_dv28[c3_tmp_data[c3_i108] - 1] = rtInf;
  }

  c3_y0 = c3_dv28[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 81);
  c3_dv29[0] = c3_x0;
  c3_c_par = c3_par;
  c3_b4 = !c3_col;
  c3_eml_li_find(chartInstance, c3_c_par && c3_b4, c3_tmp_data, c3_tmp_sizes);
  c3_c_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i109 = 0; c3_i109 <= c3_c_loop_ub; c3_i109++) {
    c3_dv29[c3_tmp_data[c3_i109] - 1] = rtNaN;
  }

  c3_x0 = c3_dv29[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 82);
  c3_dv30[0] = c3_y0;
  c3_d_par = c3_par;
  c3_b5 = !c3_col;
  c3_eml_li_find(chartInstance, c3_d_par && c3_b5, c3_tmp_data, c3_tmp_sizes);
  c3_d_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i110 = 0; c3_i110 <= c3_d_loop_ub; c3_i110++) {
    c3_dv30[c3_tmp_data[c3_i110] - 1] = rtNaN;
  }

  c3_y0 = c3_dv30[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 85);
  c3_e_x = c3_par;
  c3_f_x = c3_e_x;
  c3_c_y = ((real_T)c3_f_x != 0.0);
  if (CV_SCRIPT_IF(4, 2, c3_c_y)) {
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 86);
    c3_point[0] = c3_x0;
    c3_point[1] = c3_y0;
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 87);
  } else {
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 94);
    c3_inds = !c3_par;
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 97);
    CV_SCRIPT_IF(4, 3, false);
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 100);
    c3_x1 = c3_line1[0];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 101);
    c3_y1 = c3_line1[1];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 102);
    c3_dx1 = c3_line1[2];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 103);
    c3_dy1 = c3_line1[3];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 106);
    CV_SCRIPT_IF(4, 4, false);
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 109);
    c3_x2 = c3_line2[0];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 110);
    c3_y2 = c3_line2[1];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 111);
    c3_dx2 = c3_line2[2];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 112);
    c3_dy2 = c3_line2[3];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 115);
    c3_dx = c3_bsxfun(chartInstance, c3_line2[0], c3_line1[0]);
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 116);
    c3_dy = c3_bsxfun(chartInstance, c3_line2[1], c3_line1[1]);
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 121);
    c3_eml_li_find(chartInstance, c3_inds, c3_tmp_data, c3_tmp_sizes);
    c3_tmp_sizes[0] = 1;
    c3_tmp_sizes[1];
    c3_i111 = c3_tmp_sizes[0];
    c3_i112 = c3_tmp_sizes[1];
    c3_i113 = c3_tmp_sizes[0];
    c3_i114 = c3_tmp_sizes[1];
    c3_e_loop_ub = c3_i113 * c3_i114 - 1;
    for (c3_i115 = 0; c3_i115 <= c3_e_loop_ub; c3_i115++) {
      c3_tmp_data[c3_i115]--;
    }

    c3_denom_sizes[0] = 1;
    c3_denom_sizes[1] = c3_tmp_sizes[1];
    c3_b_denom = c3_denom_sizes[0];
    c3_c_denom = c3_denom_sizes[1];
    c3_f_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
    for (c3_i116 = 0; c3_i116 <= c3_f_loop_ub; c3_i116++) {
      c3_denom_data[c3_i116] = c3_denom;
    }

    _SFD_SYMBOL_SWITCH(7U, 21U);
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 122);
    c3_eml_li_find(chartInstance, c3_inds, c3_tmp_data, c3_tmp_sizes);
    c3_g_x = (c3_x2 * c3_dy2 * c3_dx1 - c3_dy * c3_dx1 * c3_dx2) - c3_x1 *
      c3_dy1 * c3_dx2;
    c3_y_sizes[0] = 1;
    c3_y_sizes[1] = c3_denom_sizes[1];
    c3_d_y = c3_y_sizes[0];
    c3_e_y = c3_y_sizes[1];
    c3_g_loop_ub = c3_denom_sizes[0] * c3_denom_sizes[1] - 1;
    for (c3_i117 = 0; c3_i117 <= c3_g_loop_ub; c3_i117++) {
      c3_y_data[c3_i117] = c3_denom_data[c3_i117];
    }

    c3_h_x = c3_g_x;
    c3_i_x = c3_h_x;
    c3_y_sizes[0] = 1;
    c3_y_sizes[1];
    c3_f_y = c3_y_sizes[0];
    c3_g_y = c3_y_sizes[1];
    c3_h_y = c3_y_sizes[0];
    c3_i_y = c3_y_sizes[1];
    c3_h_loop_ub = c3_h_y * c3_i_y - 1;
    for (c3_i118 = 0; c3_i118 <= c3_h_loop_ub; c3_i118++) {
      c3_y_data[c3_i118] = c3_i_x / c3_y_data[c3_i118];
    }

    _SFD_SIZE_EQ_CHECK_1D(c3_tmp_sizes[1], c3_y_sizes[1]);
    c3_dv31[0] = c3_x0;
    c3_i_loop_ub = c3_y_sizes[0] * c3_y_sizes[1] - 1;
    for (c3_i119 = 0; c3_i119 <= c3_i_loop_ub; c3_i119++) {
      c3_dv31[c3_tmp_data[c3_i119] - 1] = c3_y_data[c3_i119];
    }

    c3_x0 = c3_dv31[0];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 123);
    c3_eml_li_find(chartInstance, c3_inds, c3_tmp_data, c3_tmp_sizes);
    c3_j_x = (c3_dx * c3_dy1 * c3_dy2 + c3_y1 * c3_dx1 * c3_dy2) - c3_y2 *
      c3_dx2 * c3_dy1;
    c3_y_sizes[0] = 1;
    c3_y_sizes[1] = c3_denom_sizes[1];
    c3_j_y = c3_y_sizes[0];
    c3_k_y = c3_y_sizes[1];
    c3_j_loop_ub = c3_denom_sizes[0] * c3_denom_sizes[1] - 1;
    for (c3_i120 = 0; c3_i120 <= c3_j_loop_ub; c3_i120++) {
      c3_y_data[c3_i120] = c3_denom_data[c3_i120];
    }

    c3_k_x = c3_j_x;
    c3_l_x = c3_k_x;
    c3_y_sizes[0] = 1;
    c3_y_sizes[1];
    c3_l_y = c3_y_sizes[0];
    c3_m_y = c3_y_sizes[1];
    c3_n_y = c3_y_sizes[0];
    c3_o_y = c3_y_sizes[1];
    c3_k_loop_ub = c3_n_y * c3_o_y - 1;
    for (c3_i121 = 0; c3_i121 <= c3_k_loop_ub; c3_i121++) {
      c3_y_data[c3_i121] = c3_l_x / c3_y_data[c3_i121];
    }

    _SFD_SIZE_EQ_CHECK_1D(c3_tmp_sizes[1], c3_y_sizes[1]);
    c3_dv32[0] = c3_y0;
    c3_l_loop_ub = c3_y_sizes[0] * c3_y_sizes[1] - 1;
    for (c3_i122 = 0; c3_i122 <= c3_l_loop_ub; c3_i122++) {
      c3_dv32[c3_tmp_data[c3_i122] - 1] = c3_y_data[c3_i122];
    }

    c3_y0 = c3_dv32[0];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 126);
    c3_point[0] = c3_x0;
    c3_point[1] = c3_y0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, -126);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_laserRange(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_p1[2], real_T c3_p2[2], real_T c3_map[10000], real_T
  c3_p_data[], int32_T c3_p_sizes[2])
{
  uint32_T c3_debug_family_var_map[21];
  real_T c3_R;
  real_T c3_C;
  real_T c3_x1;
  real_T c3_y1;
  real_T c3_x2;
  real_T c3_y2;
  real_T c3_x;
  real_T c3_xd;
  real_T c3_dx;
  real_T c3_y;
  real_T c3_yd;
  real_T c3_dy;
  real_T c3_a;
  real_T c3_b;
  real_T c3_c;
  real_T c3_nargin = 3.0;
  real_T c3_nargout = 1.0;
  int32_T c3_p;
  int32_T c3_b_p;
  int32_T c3_c_p;
  int32_T c3_d_p;
  int32_T c3_i123;
  real_T c3_b_x[2];
  int32_T c3_e_p;
  int32_T c3_f_p;
  int32_T c3_i124;
  real_T c3_c_x[2];
  int32_T c3_i125;
  boolean_T c3_d_x[2];
  int32_T c3_g_p;
  int32_T c3_h_p;
  int32_T c3_i126;
  int32_T c3_i_p;
  int32_T c3_j_p;
  int32_T c3_i127;
  int32_T c3_k_p;
  int32_T c3_l_p;
  int32_T c3_i128;
  real_T c3_e_x[2];
  int32_T c3_i129;
  boolean_T c3_f_x[2];
  int32_T c3_m_p;
  int32_T c3_n_p;
  int32_T c3_i130;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  int32_T exitg1;
  int32_T exitg2;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 21U, 21U, c3_g_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R, 0U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C, 1U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x1, 2U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y1, 3U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x2, 4U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y2, 5U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x, 6U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xd, 7U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx, 8U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y, 9U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_yd, 10U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy, 11U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_a, 12U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b, 13U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c, 14U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 15U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 16U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p1, 17U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p2, 18U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_map, 19U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_p_data, (const int32_T *)
    c3_p_sizes, NULL, 0, 20, (void *)c3_v_sf_marshallOut, (void *)
    c3_s_sf_marshallIn);
  CV_SCRIPT_FCN(6, 0);
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 24);
  c3_R = 100.0;
  c3_C = 100.0;
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 26);
  c3_x1 = c3_p1[0];
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 26);
  c3_y1 = c3_p1[1];
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 27);
  c3_x2 = c3_p2[0];
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 27);
  c3_y2 = c3_p2[1];
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 30);
  c3_x = c3_x1;
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 31);
  if (CV_SCRIPT_IF(6, 0, c3_x2 > c3_x1)) {
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 32);
    c3_xd = c3_x2 - c3_x1;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 33);
    c3_dx = 1.0;
  } else {
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 35);
    c3_xd = c3_x1 - c3_x2;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 36);
    c3_dx = -1.0;
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 39);
  c3_y = c3_y1;
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 40);
  if (CV_SCRIPT_IF(6, 1, c3_y2 > c3_y1)) {
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 41);
    c3_yd = c3_y2 - c3_y1;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 42);
    c3_dy = 1.0;
  } else {
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 44);
    c3_yd = c3_y1 - c3_y2;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 45);
    c3_dy = -1.0;
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 48);
  c3_p_sizes[0] = 0;
  c3_p_sizes[1] = 0;
  c3_p = c3_p_sizes[0];
  c3_b_p = c3_p_sizes[1];
  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 50);
  if (CV_SCRIPT_IF(6, 2, c3_xd > c3_yd)) {
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 51);
    c3_a = 2.0 * c3_yd;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 52);
    c3_b = c3_a - c3_xd;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 53);
    c3_c = c3_b - c3_xd;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 55);
    guard4 = false;
    guard5 = false;
    guard6 = false;
    do {
      exitg2 = 0;
      CV_SCRIPT_WHILE(6, 0, true);
      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 57);
      if (CV_SCRIPT_COND(6, 0, c3_x < 0.0)) {
        guard6 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(6, 1, c3_y < 0.0)) {
        guard6 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(6, 2, c3_x > 100.0)) {
        guard5 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(6, 3, c3_y > 100.0)) {
        guard4 = true;
        exitg2 = 1;
      } else {
        CV_SCRIPT_MCDC(6, 0, false);
        CV_SCRIPT_IF(6, 3, false);
        _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 61);
        if (CV_SCRIPT_IF(6, 4, c3_map[(_SFD_EML_ARRAY_BOUNDS_CHECK("map",
               (int32_T)_SFD_INTEGER_CHECK("x", c3_x), 1, 100, 1, 0) + 100 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("map", (int32_T)_SFD_INTEGER_CHECK(
                 "y", c3_y), 1, 100, 2, 0) - 1)) - 1] == 1.0)) {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 62);
          c3_b_x[0] = c3_x;
          c3_b_x[1] = c3_y;
          c3_p_sizes[0] = 1;
          c3_p_sizes[1] = 2;
          c3_e_p = c3_p_sizes[0];
          c3_f_p = c3_p_sizes[1];
          for (c3_i124 = 0; c3_i124 < 2; c3_i124++) {
            c3_p_data[c3_i124] = c3_b_x[c3_i124];
          }

          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 63);
          exitg2 = 1;
        } else {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 65);
          c3_c_x[0] = c3_x - c3_x2;
          c3_c_x[1] = c3_y - c3_y2;
          for (c3_i125 = 0; c3_i125 < 2; c3_i125++) {
            c3_d_x[c3_i125] = (c3_c_x[c3_i125] == 0.0);
          }

          if (CV_SCRIPT_IF(6, 5, c3_all(chartInstance, c3_d_x))) {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 66);
            c3_p_sizes[0] = 1;
            c3_p_sizes[1] = 2;
            c3_g_p = c3_p_sizes[0];
            c3_h_p = c3_p_sizes[1];
            for (c3_i126 = 0; c3_i126 < 2; c3_i126++) {
              c3_p_data[c3_i126] = rtInf;
            }

            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 67);
            exitg2 = 1;
          } else {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 69);
            if (CV_SCRIPT_IF(6, 6, c3_b < 0.0)) {
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 70);
              c3_b += c3_a;
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 71);
              c3_x += c3_dx;
            } else {
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 73);
              c3_b += c3_c;
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 74);
              c3_x += c3_dx;
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 74);
              c3_y += c3_dy;
            }

            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 55);
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
      CV_SCRIPT_MCDC(6, 0, true);
      CV_SCRIPT_IF(6, 3, true);
      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 58);
      c3_p_sizes[0] = 1;
      c3_p_sizes[1] = 2;
      c3_c_p = c3_p_sizes[0];
      c3_d_p = c3_p_sizes[1];
      for (c3_i123 = 0; c3_i123 < 2; c3_i123++) {
        c3_p_data[c3_i123] = rtInf;
      }

      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 59);
    }
  } else {
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 78);
    c3_a = 2.0 * c3_xd;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 79);
    c3_b = c3_a - c3_yd;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 80);
    c3_c = c3_b - c3_yd;
    _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 82);
    guard1 = false;
    guard2 = false;
    guard3 = false;
    do {
      exitg1 = 0;
      CV_SCRIPT_WHILE(6, 1, true);
      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 84);
      if (CV_SCRIPT_COND(6, 4, c3_x < 0.0)) {
        guard3 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(6, 5, c3_y < 0.0)) {
        guard3 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(6, 6, c3_x > 100.0)) {
        guard2 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(6, 7, c3_y > 100.0)) {
        guard1 = true;
        exitg1 = 1;
      } else {
        CV_SCRIPT_MCDC(6, 1, false);
        CV_SCRIPT_IF(6, 7, false);
        _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 88);
        if (CV_SCRIPT_IF(6, 8, c3_map[(_SFD_EML_ARRAY_BOUNDS_CHECK("map",
               (int32_T)_SFD_INTEGER_CHECK("x", c3_x), 1, 100, 1, 0) + 100 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("map", (int32_T)_SFD_INTEGER_CHECK(
                 "y", c3_y), 1, 100, 2, 0) - 1)) - 1] == 1.0)) {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 89);
          c3_b_x[0] = c3_x;
          c3_b_x[1] = c3_y;
          c3_p_sizes[0] = 1;
          c3_p_sizes[1] = 2;
          c3_k_p = c3_p_sizes[0];
          c3_l_p = c3_p_sizes[1];
          for (c3_i128 = 0; c3_i128 < 2; c3_i128++) {
            c3_p_data[c3_i128] = c3_b_x[c3_i128];
          }

          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 90);
          exitg1 = 1;
        } else {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 92);
          c3_e_x[0] = c3_x - c3_x2;
          c3_e_x[1] = c3_y - c3_y2;
          for (c3_i129 = 0; c3_i129 < 2; c3_i129++) {
            c3_f_x[c3_i129] = (c3_e_x[c3_i129] == 0.0);
          }

          if (CV_SCRIPT_IF(6, 9, c3_all(chartInstance, c3_f_x))) {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 93);
            c3_p_sizes[0] = 1;
            c3_p_sizes[1] = 2;
            c3_m_p = c3_p_sizes[0];
            c3_n_p = c3_p_sizes[1];
            for (c3_i130 = 0; c3_i130 < 2; c3_i130++) {
              c3_p_data[c3_i130] = rtInf;
            }

            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 94);
            exitg1 = 1;
          } else {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 96);
            if (CV_SCRIPT_IF(6, 10, c3_b < 0.0)) {
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 97);
              c3_b += c3_a;
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 98);
              c3_y += c3_dy;
            } else {
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 100);
              c3_b += c3_c;
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 101);
              c3_x += c3_dx;
              _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 101);
              c3_y += c3_dy;
            }

            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 82);
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
      CV_SCRIPT_MCDC(6, 1, true);
      CV_SCRIPT_IF(6, 7, true);
      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 85);
      c3_p_sizes[0] = 1;
      c3_p_sizes[1] = 2;
      c3_i_p = c3_p_sizes[0];
      c3_j_p = c3_p_sizes[1];
      for (c3_i127 = 0; c3_i127 < 2; c3_i127++) {
        c3_p_data[c3_i127] = rtInf;
      }

      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 86);
    }
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, -101);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 0U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\rvctools\\robot\\se2.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 1U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\assignment6\\laserScanner.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 2U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\assignment6\\XYtoIJ.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 3U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\geom2d\\geom2d\\geom2d\\clipLine.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 4U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\geom2d\\geom2d\\geom2d\\intersectLines.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 5U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\geom2d\\geom2d\\geom2d\\linePosition.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 6U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\rvctools\\common\\laserRange.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 7U,
    sf_debug_get_script_id(
    "C:\\Users\\jpacker\\stash\\Quad-Sim\\Quadcopter Dynamic Modeling and Simulation\\Lidar\\assignment6\\IJtoXY.m"));
}

static void c3_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_load, const char_T *c3_identifier,
  c3_s2aqkGCuE38RBomNVWBcX1B *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_load), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_load);
}

static void c3_b_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_s2aqkGCuE38RBomNVWBcX1B *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[1] = { "map" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 1, c3_fieldNames, 0U, NULL);
  c3_thisId.fIdentifier = "map";
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "map",
    "map", 0)), &c3_thisId, c3_y->map);
  sf_mex_destroy(&c3_u);
}

static void c3_c_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[10000])
{
  real_T c3_dv33[10000];
  int32_T c3_i131;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv33, 1, 0, 0U, 1, 0U, 2, 100,
                100);
  for (c3_i131 = 0; c3_i131 < 10000; c3_i131++) {
    c3_y[c3_i131] = c3_dv33[c3_i131];
  }

  sf_mex_destroy(&c3_u);
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i132;
  real_T c3_b_inData[361];
  int32_T c3_i133;
  real_T c3_u[361];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i132 = 0; c3_i132 < 361; c3_i132++) {
    c3_b_inData[c3_i132] = (*(real_T (*)[361])c3_inData)[c3_i132];
  }

  for (c3_i133 = 0; c3_i133 < 361; c3_i133++) {
    c3_u[c3_i133] = c3_b_inData[c3_i133];
  }

  c3_y = NULL;
  if (!chartInstance->c3_prev_theta_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 361), false);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_d_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_theta, const char_T *c3_identifier,
  real_T c3_y[361])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_theta), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_b_prev_theta);
}

static void c3_e_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361])
{
  real_T c3_dv34[361];
  int32_T c3_i134;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_prev_theta_not_empty = false;
  } else {
    chartInstance->c3_prev_theta_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv34, 1, 0, 0U, 1, 0U, 1,
                  361);
    for (c3_i134 = 0; c3_i134 < 361; c3_i134++) {
      c3_y[c3_i134] = c3_dv34[c3_i134];
    }
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_prev_theta;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[361];
  int32_T c3_i135;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_prev_theta = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_theta), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_b_prev_theta);
  for (c3_i135 = 0; c3_i135 < 361; c3_i135++) {
    (*(real_T (*)[361])c3_outData)[c3_i135] = c3_y[c3_i135];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i136;
  real_T c3_b_inData[361];
  int32_T c3_i137;
  real_T c3_u[361];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i136 = 0; c3_i136 < 361; c3_i136++) {
    c3_b_inData[c3_i136] = (*(real_T (*)[361])c3_inData)[c3_i136];
  }

  for (c3_i137 = 0; c3_i137 < 361; c3_i137++) {
    c3_u[c3_i137] = c3_b_inData[c3_i137];
  }

  c3_y = NULL;
  if (!chartInstance->c3_prev_r_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 361), false);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_f_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_r, const char_T *c3_identifier,
  real_T c3_y[361])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_r), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_prev_r);
}

static void c3_g_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361])
{
  real_T c3_dv35[361];
  int32_T c3_i138;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_prev_r_not_empty = false;
  } else {
    chartInstance->c3_prev_r_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv35, 1, 0, 0U, 1, 0U, 1,
                  361);
    for (c3_i138 = 0; c3_i138 < 361; c3_i138++) {
      c3_y[c3_i138] = c3_dv35[c3_i138];
    }
  }

  sf_mex_destroy(&c3_u);
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_prev_r;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[361];
  int32_T c3_i139;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_prev_r = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_r), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_prev_r);
  for (c3_i139 = 0; c3_i139 < 361; c3_i139++) {
    (*(real_T (*)[361])c3_outData)[c3_i139] = c3_y[c3_i139];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  if (!chartInstance->c3_prev_t_not_empty) {
    sf_mex_assign(&c3_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_h_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_t, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_t),
    &c3_thisId);
  sf_mex_destroy(&c3_b_prev_t);
  return c3_y;
}

static real_T c3_i_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  if (mxIsEmpty(c3_u)) {
    chartInstance->c3_prev_t_not_empty = false;
  } else {
    chartInstance->c3_prev_t_not_empty = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d0;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_prev_t;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_prev_t = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_t),
    &c3_thisId);
  sf_mex_destroy(&c3_b_prev_t);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i140;
  real_T c3_b_inData[361];
  int32_T c3_i141;
  real_T c3_u[361];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i140 = 0; c3_i140 < 361; c3_i140++) {
    c3_b_inData[c3_i140] = (*(real_T (*)[361])c3_inData)[c3_i140];
  }

  for (c3_i141 = 0; c3_i141 < 361; c3_i141++) {
    c3_u[c3_i141] = c3_b_inData[c3_i141];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 361), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_j_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_theta, const char_T *c3_identifier, real_T
  c3_y[361])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_theta), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_theta);
}

static void c3_k_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361])
{
  real_T c3_dv36[361];
  int32_T c3_i142;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv36, 1, 0, 0U, 1, 0U, 1, 361);
  for (c3_i142 = 0; c3_i142 < 361; c3_i142++) {
    c3_y[c3_i142] = c3_dv36[c3_i142];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_theta;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[361];
  int32_T c3_i143;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_theta = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_theta), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_theta);
  for (c3_i143 = 0; c3_i143 < 361; c3_i143++) {
    (*(real_T (*)[361])c3_outData)[c3_i143] = c3_y[c3_i143];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_l_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d1;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d1, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d1;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_nargout;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_nargout = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i144;
  int32_T c3_i145;
  int32_T c3_i146;
  real_T c3_b_inData[10000];
  int32_T c3_i147;
  int32_T c3_i148;
  int32_T c3_i149;
  real_T c3_u[10000];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i144 = 0;
  for (c3_i145 = 0; c3_i145 < 100; c3_i145++) {
    for (c3_i146 = 0; c3_i146 < 100; c3_i146++) {
      c3_b_inData[c3_i146 + c3_i144] = (*(real_T (*)[10000])c3_inData)[c3_i146 +
        c3_i144];
    }

    c3_i144 += 100;
  }

  c3_i147 = 0;
  for (c3_i148 = 0; c3_i148 < 100; c3_i148++) {
    for (c3_i149 = 0; c3_i149 < 100; c3_i149++) {
      c3_u[c3_i149 + c3_i147] = c3_b_inData[c3_i149 + c3_i147];
    }

    c3_i147 += 100;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 100, 100),
                false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_map;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[10000];
  int32_T c3_i150;
  int32_T c3_i151;
  int32_T c3_i152;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_map = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_map), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_map);
  c3_i150 = 0;
  for (c3_i151 = 0; c3_i151 < 100; c3_i151++) {
    for (c3_i152 = 0; c3_i152 < 100; c3_i152++) {
      (*(real_T (*)[10000])c3_outData)[c3_i152 + c3_i150] = c3_y[c3_i152 +
        c3_i150];
    }

    c3_i150 += 100;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i153;
  int32_T c3_i154;
  int32_T c3_i155;
  real_T c3_b_inData[722];
  int32_T c3_i156;
  int32_T c3_i157;
  int32_T c3_i158;
  real_T c3_u[722];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i153 = 0;
  for (c3_i154 = 0; c3_i154 < 2; c3_i154++) {
    for (c3_i155 = 0; c3_i155 < 361; c3_i155++) {
      c3_b_inData[c3_i155 + c3_i153] = (*(real_T (*)[722])c3_inData)[c3_i155 +
        c3_i153];
    }

    c3_i153 += 361;
  }

  c3_i156 = 0;
  for (c3_i157 = 0; c3_i157 < 2; c3_i157++) {
    for (c3_i158 = 0; c3_i158 < 361; c3_i158++) {
      c3_u[c3_i158 + c3_i156] = c3_b_inData[c3_i158 + c3_i156];
    }

    c3_i156 += 361;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 361, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_m_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[722])
{
  real_T c3_dv37[722];
  int32_T c3_i159;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv37, 1, 0, 0U, 1, 0U, 2, 361,
                2);
  for (c3_i159 = 0; c3_i159 < 722; c3_i159++) {
    c3_y[c3_i159] = c3_dv37[c3_i159];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_p;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[722];
  int32_T c3_i160;
  int32_T c3_i161;
  int32_T c3_i162;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_p = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_p), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_p);
  c3_i160 = 0;
  for (c3_i161 = 0; c3_i161 < 2; c3_i161++) {
    for (c3_i162 = 0; c3_i162 < 361; c3_i162++) {
      (*(real_T (*)[722])c3_outData)[c3_i162 + c3_i160] = c3_y[c3_i162 + c3_i160];
    }

    c3_i160 += 361;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i163;
  int32_T c3_i164;
  int32_T c3_i165;
  real_T c3_b_inData[9];
  int32_T c3_i166;
  int32_T c3_i167;
  int32_T c3_i168;
  real_T c3_u[9];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i163 = 0;
  for (c3_i164 = 0; c3_i164 < 3; c3_i164++) {
    for (c3_i165 = 0; c3_i165 < 3; c3_i165++) {
      c3_b_inData[c3_i165 + c3_i163] = (*(real_T (*)[9])c3_inData)[c3_i165 +
        c3_i163];
    }

    c3_i163 += 3;
  }

  c3_i166 = 0;
  for (c3_i167 = 0; c3_i167 < 3; c3_i167++) {
    for (c3_i168 = 0; c3_i168 < 3; c3_i168++) {
      c3_u[c3_i168 + c3_i166] = c3_b_inData[c3_i168 + c3_i166];
    }

    c3_i166 += 3;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_n_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[9])
{
  real_T c3_dv38[9];
  int32_T c3_i169;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv38, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c3_i169 = 0; c3_i169 < 9; c3_i169++) {
    c3_y[c3_i169] = c3_dv38[c3_i169];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_Tl;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[9];
  int32_T c3_i170;
  int32_T c3_i171;
  int32_T c3_i172;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_Tl = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Tl), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_Tl);
  c3_i170 = 0;
  for (c3_i171 = 0; c3_i171 < 3; c3_i171++) {
    for (c3_i172 = 0; c3_i172 < 3; c3_i172++) {
      (*(real_T (*)[9])c3_outData)[c3_i172 + c3_i170] = c3_y[c3_i172 + c3_i170];
    }

    c3_i170 += 3;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  c3_s2aqkGCuE38RBomNVWBcX1B c3_u;
  const mxArray *c3_y = NULL;
  int32_T c3_i173;
  real_T c3_b_u[10000];
  const mxArray *c3_b_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_s2aqkGCuE38RBomNVWBcX1B *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c3_i173 = 0; c3_i173 < 10000; c3_i173++) {
    c3_b_u[c3_i173] = c3_u.map[c3_i173];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 0, 0U, 1U, 0U, 2, 100, 100),
                false);
  sf_mex_addfield(c3_y, c3_b_y, "map", "map", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_load;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_s2aqkGCuE38RBomNVWBcX1B c3_y;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_load = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_load), &c3_thisId, &c3_y);
  sf_mex_destroy(&c3_load);
  *(c3_s2aqkGCuE38RBomNVWBcX1B *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i174;
  real_T c3_b_inData[3];
  int32_T c3_i175;
  real_T c3_u[3];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i174 = 0; c3_i174 < 3; c3_i174++) {
    c3_b_inData[c3_i174] = (*(real_T (*)[3])c3_inData)[c3_i174];
  }

  for (c3_i175 = 0; c3_i175 < 3; c3_i175++) {
    c3_u[c3_i175] = c3_b_inData[c3_i175];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_o_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv39[3];
  int32_T c3_i176;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv39, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c3_i176 = 0; c3_i176 < 3; c3_i176++) {
    c3_y[c3_i176] = c3_dv39[c3_i176];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_a;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i177;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_a = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_a), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_a);
  for (c3_i177 = 0; c3_i177 < 3; c3_i177++) {
    (*(real_T (*)[3])c3_outData)[c3_i177] = c3_y[c3_i177];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_k_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i178;
  int32_T c3_i179;
  int32_T c3_i180;
  real_T c3_b_inData[4];
  int32_T c3_i181;
  int32_T c3_i182;
  int32_T c3_i183;
  real_T c3_u[4];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i178 = 0;
  for (c3_i179 = 0; c3_i179 < 2; c3_i179++) {
    for (c3_i180 = 0; c3_i180 < 2; c3_i180++) {
      c3_b_inData[c3_i180 + c3_i178] = (*(real_T (*)[4])c3_inData)[c3_i180 +
        c3_i178];
    }

    c3_i178 += 2;
  }

  c3_i181 = 0;
  for (c3_i182 = 0; c3_i182 < 2; c3_i182++) {
    for (c3_i183 = 0; c3_i183 < 2; c3_i183++) {
      c3_u[c3_i183 + c3_i181] = c3_b_inData[c3_i183 + c3_i181];
    }

    c3_i181 += 2;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_p_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4])
{
  real_T c3_dv40[4];
  int32_T c3_i184;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv40, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c3_i184 = 0; c3_i184 < 4; c3_i184++) {
    c3_y[c3_i184] = c3_dv40[c3_i184];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_R;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[4];
  int32_T c3_i185;
  int32_T c3_i186;
  int32_T c3_i187;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_R = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_R), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_R);
  c3_i185 = 0;
  for (c3_i186 = 0; c3_i186 < 2; c3_i186++) {
    for (c3_i187 = 0; c3_i187 < 2; c3_i187++) {
      (*(real_T (*)[4])c3_outData)[c3_i187 + c3_i185] = c3_y[c3_i187 + c3_i185];
    }

    c3_i185 += 2;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_l_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i188;
  real_T c3_b_inData[2];
  int32_T c3_i189;
  real_T c3_u[2];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i188 = 0; c3_i188 < 2; c3_i188++) {
    c3_b_inData[c3_i188] = (*(real_T (*)[2])c3_inData)[c3_i188];
  }

  for (c3_i189 = 0; c3_i189 < 2; c3_i189++) {
    c3_u[c3_i189] = c3_b_inData[c3_i189];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_q_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[2])
{
  real_T c3_dv41[2];
  int32_T c3_i190;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv41, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c3_i190 = 0; c3_i190 < 2; c3_i190++) {
    c3_y[c3_i190] = c3_dv41[c3_i190];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_point;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[2];
  int32_T c3_i191;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_point = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_point), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_point);
  for (c3_i191 = 0; c3_i191 < 2; c3_i191++) {
    (*(real_T (*)[2])c3_outData)[c3_i191] = c3_y[c3_i191];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_m_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i192;
  real_T c3_b_inData[4];
  int32_T c3_i193;
  real_T c3_u[4];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i192 = 0; c3_i192 < 4; c3_i192++) {
    c3_b_inData[c3_i192] = (*(real_T (*)[4])c3_inData)[c3_i192];
  }

  for (c3_i193 = 0; c3_i193 < 4; c3_i193++) {
    c3_u[c3_i193] = c3_b_inData[c3_i193];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_r_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4])
{
  real_T c3_dv42[4];
  int32_T c3_i194;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv42, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c3_i194 = 0; c3_i194 < 4; c3_i194++) {
    c3_y[c3_i194] = c3_dv42[c3_i194];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_line2;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[4];
  int32_T c3_i195;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_line2 = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_line2), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_line2);
  for (c3_i195 = 0; c3_i195 < 4; c3_i195++) {
    (*(real_T (*)[4])c3_outData)[c3_i195] = c3_y[c3_i195];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_n_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_b_inData_sizes[2];
  int32_T c3_loop_ub;
  int32_T c3_i196;
  real_T c3_b_inData_data[1];
  int32_T c3_u_sizes[2];
  int32_T c3_b_loop_ub;
  int32_T c3_i197;
  real_T c3_u_data[1];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_b_inData_sizes[0] = 1;
  c3_b_inData_sizes[1] = c3_inData_sizes[1];
  c3_loop_ub = c3_inData_sizes[1] - 1;
  for (c3_i196 = 0; c3_i196 <= c3_loop_ub; c3_i196++) {
    c3_b_inData_data[c3_b_inData_sizes[0] * c3_i196] =
      c3_inData_data[c3_inData_sizes[0] * c3_i196];
  }

  c3_u_sizes[0] = 1;
  c3_u_sizes[1] = c3_b_inData_sizes[1];
  c3_b_loop_ub = c3_b_inData_sizes[1] - 1;
  for (c3_i197 = 0; c3_i197 <= c3_b_loop_ub; c3_i197++) {
    c3_u_data[c3_u_sizes[0] * c3_i197] = c3_b_inData_data[c3_b_inData_sizes[0] *
      c3_i197];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_s_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i198;
  uint32_T c3_uv0[2];
  int32_T c3_i199;
  static boolean_T c3_bv0[2] = { false, true };

  boolean_T c3_bv1[2];
  int32_T c3_tmp_sizes[2];
  real_T c3_tmp_data[1];
  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_loop_ub;
  int32_T c3_i200;
  (void)chartInstance;
  for (c3_i198 = 0; c3_i198 < 2; c3_i198++) {
    c3_uv0[c3_i198] = 1U;
  }

  for (c3_i199 = 0; c3_i199 < 2; c3_i199++) {
    c3_bv1[c3_i199] = c3_bv0[c3_i199];
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv1, c3_uv0, c3_tmp_sizes);
  c3_y_sizes[0] = 1;
  c3_y_sizes[1] = c3_tmp_sizes[1];
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i200 = 0; c3_i200 <= c3_loop_ub; c3_i200++) {
    c3_y_data[c3_i200] = c3_tmp_data[c3_i200];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_denom;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y_sizes[2];
  real_T c3_y_data[1];
  int32_T c3_loop_ub;
  int32_T c3_i201;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_denom = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_denom), &c3_thisId,
                        c3_y_data, c3_y_sizes);
  sf_mex_destroy(&c3_denom);
  c3_outData_sizes[0] = 1;
  c3_outData_sizes[1] = c3_y_sizes[1];
  c3_loop_ub = c3_y_sizes[1] - 1;
  for (c3_i201 = 0; c3_i201 <= c3_loop_ub; c3_i201++) {
    c3_outData_data[c3_outData_sizes[0] * c3_i201] = c3_y_data[c3_y_sizes[0] *
      c3_i201];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_o_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  boolean_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(boolean_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static boolean_T c3_t_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  boolean_T c3_y;
  boolean_T c3_b6;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_b6, 1, 11, 0U, 0, 0U, 0);
  c3_y = c3_b6;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_inds;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  boolean_T c3_y;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_inds = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_inds), &c3_thisId);
  sf_mex_destroy(&c3_inds);
  *(boolean_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_p_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  (void)c3_inData;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static const mxArray *c3_q_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T *c3_inData_sizes)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_b_inData_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i202;
  real_T c3_b_inData_data[4];
  int32_T c3_u_sizes;
  int32_T c3_b_loop_ub;
  int32_T c3_i203;
  real_T c3_u_data[4];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_b_inData_sizes = *c3_inData_sizes;
  c3_loop_ub = *c3_inData_sizes - 1;
  for (c3_i202 = 0; c3_i202 <= c3_loop_ub; c3_i202++) {
    c3_b_inData_data[c3_i202] = c3_inData_data[c3_i202];
  }

  c3_u_sizes = c3_b_inData_sizes;
  c3_b_loop_ub = c3_b_inData_sizes - 1;
  for (c3_i203 = 0; c3_i203 <= c3_b_loop_ub; c3_i203++) {
    c3_u_data[c3_i203] = c3_b_inData_data[c3_i203];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 1,
    c3_u_sizes), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_u_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T *c3_y_sizes)
{
  static uint32_T c3_uv1[1] = { 4U };

  uint32_T c3_uv2[1];
  static boolean_T c3_bv2[1] = { true };

  boolean_T c3_bv3[1];
  int32_T c3_tmp_sizes;
  real_T c3_tmp_data[4];
  int32_T c3_loop_ub;
  int32_T c3_i204;
  (void)chartInstance;
  c3_uv2[0] = c3_uv1[0];
  c3_bv3[0] = c3_bv2[0];
  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c3_bv3, c3_uv2, &c3_tmp_sizes);
  *c3_y_sizes = c3_tmp_sizes;
  c3_loop_ub = c3_tmp_sizes - 1;
  for (c3_i204 = 0; c3_i204 <= c3_loop_ub; c3_i204++) {
    c3_y_data[c3_i204] = c3_tmp_data[c3_i204];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  *c3_outData_sizes)
{
  const mxArray *c3_pos;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y_sizes;
  real_T c3_y_data[4];
  int32_T c3_loop_ub;
  int32_T c3_i205;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_pos = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_pos), &c3_thisId, c3_y_data,
                        &c3_y_sizes);
  sf_mex_destroy(&c3_pos);
  *c3_outData_sizes = c3_y_sizes;
  c3_loop_ub = c3_y_sizes - 1;
  for (c3_i205 = 0; c3_i205 <= c3_loop_ub; c3_i205++) {
    c3_outData_data[c3_i205] = c3_y_data[c3_i205];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_r_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_b_inData_sizes[2];
  int32_T c3_i206;
  int32_T c3_loop_ub;
  int32_T c3_i207;
  real_T c3_b_inData_data[8];
  int32_T c3_u_sizes[2];
  int32_T c3_i208;
  int32_T c3_b_loop_ub;
  int32_T c3_i209;
  real_T c3_u_data[8];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_b_inData_sizes[0] = c3_inData_sizes[0];
  c3_b_inData_sizes[1] = 2;
  for (c3_i206 = 0; c3_i206 < 2; c3_i206++) {
    c3_loop_ub = c3_inData_sizes[0] - 1;
    for (c3_i207 = 0; c3_i207 <= c3_loop_ub; c3_i207++) {
      c3_b_inData_data[c3_i207 + c3_b_inData_sizes[0] * c3_i206] =
        c3_inData_data[c3_i207 + c3_inData_sizes[0] * c3_i206];
    }
  }

  c3_u_sizes[0] = c3_b_inData_sizes[0];
  c3_u_sizes[1] = 2;
  for (c3_i208 = 0; c3_i208 < 2; c3_i208++) {
    c3_b_loop_ub = c3_b_inData_sizes[0] - 1;
    for (c3_i209 = 0; c3_i209 <= c3_b_loop_ub; c3_i209++) {
      c3_u_data[c3_i209 + c3_u_sizes[0] * c3_i208] = c3_b_inData_data[c3_i209 +
        c3_b_inData_sizes[0] * c3_i208];
    }
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_v_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i210;
  uint32_T c3_uv3[2];
  int32_T c3_i211;
  static boolean_T c3_bv4[2] = { true, false };

  boolean_T c3_bv5[2];
  int32_T c3_tmp_sizes[2];
  real_T c3_tmp_data[8];
  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_loop_ub;
  int32_T c3_i212;
  (void)chartInstance;
  for (c3_i210 = 0; c3_i210 < 2; c3_i210++) {
    c3_uv3[c3_i210] = 4U + (uint32_T)(-2 * (int32_T)(uint32_T)c3_i210);
  }

  for (c3_i211 = 0; c3_i211 < 2; c3_i211++) {
    c3_bv5[c3_i211] = c3_bv4[c3_i211];
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv5, c3_uv3, c3_tmp_sizes);
  c3_y_sizes[0] = c3_tmp_sizes[0];
  c3_y_sizes[1] = 2;
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i212 = 0; c3_i212 <= c3_loop_ub; c3_i212++) {
    c3_y_data[c3_i212] = c3_tmp_data[c3_i212];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_point;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y_sizes[2];
  real_T c3_y_data[8];
  int32_T c3_i213;
  int32_T c3_loop_ub;
  int32_T c3_i214;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_point = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_point), &c3_thisId,
                        c3_y_data, c3_y_sizes);
  sf_mex_destroy(&c3_point);
  c3_outData_sizes[0] = c3_y_sizes[0];
  c3_outData_sizes[1] = 2;
  for (c3_i213 = 0; c3_i213 < 2; c3_i213++) {
    c3_loop_ub = c3_y_sizes[0] - 1;
    for (c3_i214 = 0; c3_i214 <= c3_loop_ub; c3_i214++) {
      c3_outData_data[c3_i214 + c3_outData_sizes[0] * c3_i213] =
        c3_y_data[c3_i214 + c3_y_sizes[0] * c3_i213];
    }
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_s_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  (void)c3_inData;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static const mxArray *c3_t_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i215;
  int32_T c3_i216;
  int32_T c3_i217;
  real_T c3_b_inData[8];
  int32_T c3_i218;
  int32_T c3_i219;
  int32_T c3_i220;
  real_T c3_u[8];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i215 = 0;
  for (c3_i216 = 0; c3_i216 < 2; c3_i216++) {
    for (c3_i217 = 0; c3_i217 < 4; c3_i217++) {
      c3_b_inData[c3_i217 + c3_i215] = (*(real_T (*)[8])c3_inData)[c3_i217 +
        c3_i215];
    }

    c3_i215 += 4;
  }

  c3_i218 = 0;
  for (c3_i219 = 0; c3_i219 < 2; c3_i219++) {
    for (c3_i220 = 0; c3_i220 < 4; c3_i220++) {
      c3_u[c3_i220 + c3_i218] = c3_b_inData[c3_i220 + c3_i218];
    }

    c3_i218 += 4;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 4, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_w_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8])
{
  real_T c3_dv43[8];
  int32_T c3_i221;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv43, 1, 0, 0U, 1, 0U, 2, 4, 2);
  for (c3_i221 = 0; c3_i221 < 8; c3_i221++) {
    c3_y[c3_i221] = c3_dv43[c3_i221];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_points;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[8];
  int32_T c3_i222;
  int32_T c3_i223;
  int32_T c3_i224;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_points = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_w_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_points), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_points);
  c3_i222 = 0;
  for (c3_i223 = 0; c3_i223 < 2; c3_i223++) {
    for (c3_i224 = 0; c3_i224 < 4; c3_i224++) {
      (*(real_T (*)[8])c3_outData)[c3_i224 + c3_i222] = c3_y[c3_i224 + c3_i222];
    }

    c3_i222 += 4;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_u_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  (void)c3_inData;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static const mxArray *c3_v_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_b_inData_sizes[2];
  int32_T c3_loop_ub;
  int32_T c3_i225;
  int32_T c3_b_loop_ub;
  int32_T c3_i226;
  real_T c3_b_inData_data[2];
  int32_T c3_u_sizes[2];
  int32_T c3_c_loop_ub;
  int32_T c3_i227;
  int32_T c3_d_loop_ub;
  int32_T c3_i228;
  real_T c3_u_data[2];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_b_inData_sizes[0] = c3_inData_sizes[0];
  c3_b_inData_sizes[1] = c3_inData_sizes[1];
  c3_loop_ub = c3_inData_sizes[1] - 1;
  for (c3_i225 = 0; c3_i225 <= c3_loop_ub; c3_i225++) {
    c3_b_loop_ub = c3_inData_sizes[0] - 1;
    for (c3_i226 = 0; c3_i226 <= c3_b_loop_ub; c3_i226++) {
      c3_b_inData_data[c3_i226 + c3_b_inData_sizes[0] * c3_i225] =
        c3_inData_data[c3_i226 + c3_inData_sizes[0] * c3_i225];
    }
  }

  c3_u_sizes[0] = c3_b_inData_sizes[0];
  c3_u_sizes[1] = c3_b_inData_sizes[1];
  c3_c_loop_ub = c3_b_inData_sizes[1] - 1;
  for (c3_i227 = 0; c3_i227 <= c3_c_loop_ub; c3_i227++) {
    c3_d_loop_ub = c3_b_inData_sizes[0] - 1;
    for (c3_i228 = 0; c3_i228 <= c3_d_loop_ub; c3_i228++) {
      c3_u_data[c3_i228 + c3_u_sizes[0] * c3_i227] = c3_b_inData_data[c3_i228 +
        c3_b_inData_sizes[0] * c3_i227];
    }
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_x_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i229;
  uint32_T c3_uv4[2];
  int32_T c3_i230;
  boolean_T c3_bv6[2];
  int32_T c3_tmp_sizes[2];
  real_T c3_tmp_data[2];
  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_loop_ub;
  int32_T c3_i231;
  (void)chartInstance;
  for (c3_i229 = 0; c3_i229 < 2; c3_i229++) {
    c3_uv4[c3_i229] = 1U + (uint32_T)c3_i229;
  }

  for (c3_i230 = 0; c3_i230 < 2; c3_i230++) {
    c3_bv6[c3_i230] = true;
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv6, c3_uv4, c3_tmp_sizes);
  c3_y_sizes[0] = c3_tmp_sizes[0];
  c3_y_sizes[1] = c3_tmp_sizes[1];
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i231 = 0; c3_i231 <= c3_loop_ub; c3_i231++) {
    c3_y_data[c3_i231] = c3_tmp_data[c3_i231];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_p;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y_sizes[2];
  real_T c3_y_data[2];
  int32_T c3_loop_ub;
  int32_T c3_i232;
  int32_T c3_b_loop_ub;
  int32_T c3_i233;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_p = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_p), &c3_thisId, c3_y_data,
                        c3_y_sizes);
  sf_mex_destroy(&c3_p);
  c3_outData_sizes[0] = c3_y_sizes[0];
  c3_outData_sizes[1] = c3_y_sizes[1];
  c3_loop_ub = c3_y_sizes[1] - 1;
  for (c3_i232 = 0; c3_i232 <= c3_loop_ub; c3_i232++) {
    c3_b_loop_ub = c3_y_sizes[0] - 1;
    for (c3_i233 = 0; c3_i233 <= c3_b_loop_ub; c3_i233++) {
      c3_outData_data[c3_i233 + c3_outData_sizes[0] * c3_i232] =
        c3_y_data[c3_i233 + c3_y_sizes[0] * c3_i232];
    }
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_w_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_b_inData_sizes[2];
  int32_T c3_i234;
  real_T c3_b_inData_data[2];
  int32_T c3_u_sizes[2];
  int32_T c3_i235;
  real_T c3_u_data[2];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_b_inData_sizes[0] = 1;
  c3_b_inData_sizes[1] = 2;
  for (c3_i234 = 0; c3_i234 < 2; c3_i234++) {
    c3_b_inData_data[c3_b_inData_sizes[0] * c3_i234] =
      c3_inData_data[c3_inData_sizes[0] * c3_i234];
  }

  c3_u_sizes[0] = 1;
  c3_u_sizes[1] = 2;
  for (c3_i235 = 0; c3_i235 < 2; c3_i235++) {
    c3_u_data[c3_u_sizes[0] * c3_i235] = c3_b_inData_data[c3_b_inData_sizes[0] *
      c3_i235];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_y_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i236;
  uint32_T c3_uv5[2];
  int32_T c3_i237;
  boolean_T c3_bv7[2];
  int32_T c3_tmp_sizes[2];
  real_T c3_tmp_data[2];
  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_i238;
  (void)chartInstance;
  for (c3_i236 = 0; c3_i236 < 2; c3_i236++) {
    c3_uv5[c3_i236] = 1U + (uint32_T)c3_i236;
  }

  for (c3_i237 = 0; c3_i237 < 2; c3_i237++) {
    c3_bv7[c3_i237] = false;
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv7, c3_uv5, c3_tmp_sizes);
  c3_y_sizes[0] = 1;
  c3_y_sizes[1] = 2;
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  for (c3_i238 = 0; c3_i238 < 2; c3_i238++) {
    c3_y_data[c3_i238] = c3_tmp_data[c3_i238];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_Pxel;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y_sizes[2];
  real_T c3_y_data[2];
  int32_T c3_i239;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_Pxel = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Pxel), &c3_thisId,
                        c3_y_data, c3_y_sizes);
  sf_mex_destroy(&c3_Pxel);
  c3_outData_sizes[0] = 1;
  c3_outData_sizes[1] = 2;
  for (c3_i239 = 0; c3_i239 < 2; c3_i239++) {
    c3_outData_data[c3_outData_sizes[0] * c3_i239] = c3_y_data[c3_y_sizes[0] *
      c3_i239];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_x_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i240;
  real_T c3_b_inData[3];
  int32_T c3_i241;
  real_T c3_u[3];
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i240 = 0; c3_i240 < 3; c3_i240++) {
    c3_b_inData[c3_i240] = (*(real_T (*)[3])c3_inData)[c3_i240];
  }

  for (c3_i241 = 0; c3_i241 < 3; c3_i241++) {
    c3_u[c3_i241] = c3_b_inData[c3_i241];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_ab_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv44[3];
  int32_T c3_i242;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv44, 1, 0, 0U, 1, 0U, 1, 3);
  for (c3_i242 = 0; c3_i242 < 3; c3_i242++) {
    c3_y[c3_i242] = c3_dv44[c3_i242];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_P2;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i243;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_P2 = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_P2), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_P2);
  for (c3_i243 = 0; c3_i243 < 3; c3_i243++) {
    (*(real_T (*)[3])c3_outData)[c3_i243] = c3_y[c3_i243];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_AutoFollow_Simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_createstruct("structure", 2, 180, 1),
                false);
  c3_info_helper(&c3_nameCaptureInfo);
  c3_b_info_helper(&c3_nameCaptureInfo);
  c3_c_info_helper(&c3_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs0 = NULL;
  const mxArray *c3_lhs0 = NULL;
  const mxArray *c3_rhs1 = NULL;
  const mxArray *c3_lhs1 = NULL;
  const mxArray *c3_rhs2 = NULL;
  const mxArray *c3_lhs2 = NULL;
  const mxArray *c3_rhs3 = NULL;
  const mxArray *c3_lhs3 = NULL;
  const mxArray *c3_rhs4 = NULL;
  const mxArray *c3_lhs4 = NULL;
  const mxArray *c3_rhs5 = NULL;
  const mxArray *c3_lhs5 = NULL;
  const mxArray *c3_rhs6 = NULL;
  const mxArray *c3_lhs6 = NULL;
  const mxArray *c3_rhs7 = NULL;
  const mxArray *c3_lhs7 = NULL;
  const mxArray *c3_rhs8 = NULL;
  const mxArray *c3_lhs8 = NULL;
  const mxArray *c3_rhs9 = NULL;
  const mxArray *c3_lhs9 = NULL;
  const mxArray *c3_rhs10 = NULL;
  const mxArray *c3_lhs10 = NULL;
  const mxArray *c3_rhs11 = NULL;
  const mxArray *c3_lhs11 = NULL;
  const mxArray *c3_rhs12 = NULL;
  const mxArray *c3_lhs12 = NULL;
  const mxArray *c3_rhs13 = NULL;
  const mxArray *c3_lhs13 = NULL;
  const mxArray *c3_rhs14 = NULL;
  const mxArray *c3_lhs14 = NULL;
  const mxArray *c3_rhs15 = NULL;
  const mxArray *c3_lhs15 = NULL;
  const mxArray *c3_rhs16 = NULL;
  const mxArray *c3_lhs16 = NULL;
  const mxArray *c3_rhs17 = NULL;
  const mxArray *c3_lhs17 = NULL;
  const mxArray *c3_rhs18 = NULL;
  const mxArray *c3_lhs18 = NULL;
  const mxArray *c3_rhs19 = NULL;
  const mxArray *c3_lhs19 = NULL;
  const mxArray *c3_rhs20 = NULL;
  const mxArray *c3_lhs20 = NULL;
  const mxArray *c3_rhs21 = NULL;
  const mxArray *c3_lhs21 = NULL;
  const mxArray *c3_rhs22 = NULL;
  const mxArray *c3_lhs22 = NULL;
  const mxArray *c3_rhs23 = NULL;
  const mxArray *c3_lhs23 = NULL;
  const mxArray *c3_rhs24 = NULL;
  const mxArray *c3_lhs24 = NULL;
  const mxArray *c3_rhs25 = NULL;
  const mxArray *c3_lhs25 = NULL;
  const mxArray *c3_rhs26 = NULL;
  const mxArray *c3_lhs26 = NULL;
  const mxArray *c3_rhs27 = NULL;
  const mxArray *c3_lhs27 = NULL;
  const mxArray *c3_rhs28 = NULL;
  const mxArray *c3_lhs28 = NULL;
  const mxArray *c3_rhs29 = NULL;
  const mxArray *c3_lhs29 = NULL;
  const mxArray *c3_rhs30 = NULL;
  const mxArray *c3_lhs30 = NULL;
  const mxArray *c3_rhs31 = NULL;
  const mxArray *c3_lhs31 = NULL;
  const mxArray *c3_rhs32 = NULL;
  const mxArray *c3_lhs32 = NULL;
  const mxArray *c3_rhs33 = NULL;
  const mxArray *c3_lhs33 = NULL;
  const mxArray *c3_rhs34 = NULL;
  const mxArray *c3_lhs34 = NULL;
  const mxArray *c3_rhs35 = NULL;
  const mxArray *c3_lhs35 = NULL;
  const mxArray *c3_rhs36 = NULL;
  const mxArray *c3_lhs36 = NULL;
  const mxArray *c3_rhs37 = NULL;
  const mxArray *c3_lhs37 = NULL;
  const mxArray *c3_rhs38 = NULL;
  const mxArray *c3_lhs38 = NULL;
  const mxArray *c3_rhs39 = NULL;
  const mxArray *c3_lhs39 = NULL;
  const mxArray *c3_rhs40 = NULL;
  const mxArray *c3_lhs40 = NULL;
  const mxArray *c3_rhs41 = NULL;
  const mxArray *c3_lhs41 = NULL;
  const mxArray *c3_rhs42 = NULL;
  const mxArray *c3_lhs42 = NULL;
  const mxArray *c3_rhs43 = NULL;
  const mxArray *c3_lhs43 = NULL;
  const mxArray *c3_rhs44 = NULL;
  const mxArray *c3_lhs44 = NULL;
  const mxArray *c3_rhs45 = NULL;
  const mxArray *c3_lhs45 = NULL;
  const mxArray *c3_rhs46 = NULL;
  const mxArray *c3_lhs46 = NULL;
  const mxArray *c3_rhs47 = NULL;
  const mxArray *c3_lhs47 = NULL;
  const mxArray *c3_rhs48 = NULL;
  const mxArray *c3_lhs48 = NULL;
  const mxArray *c3_rhs49 = NULL;
  const mxArray *c3_lhs49 = NULL;
  const mxArray *c3_rhs50 = NULL;
  const mxArray *c3_lhs50 = NULL;
  const mxArray *c3_rhs51 = NULL;
  const mxArray *c3_lhs51 = NULL;
  const mxArray *c3_rhs52 = NULL;
  const mxArray *c3_lhs52 = NULL;
  const mxArray *c3_rhs53 = NULL;
  const mxArray *c3_lhs53 = NULL;
  const mxArray *c3_rhs54 = NULL;
  const mxArray *c3_lhs54 = NULL;
  const mxArray *c3_rhs55 = NULL;
  const mxArray *c3_lhs55 = NULL;
  const mxArray *c3_rhs56 = NULL;
  const mxArray *c3_lhs56 = NULL;
  const mxArray *c3_rhs57 = NULL;
  const mxArray *c3_lhs57 = NULL;
  const mxArray *c3_rhs58 = NULL;
  const mxArray *c3_lhs58 = NULL;
  const mxArray *c3_rhs59 = NULL;
  const mxArray *c3_lhs59 = NULL;
  const mxArray *c3_rhs60 = NULL;
  const mxArray *c3_lhs60 = NULL;
  const mxArray *c3_rhs61 = NULL;
  const mxArray *c3_lhs61 = NULL;
  const mxArray *c3_rhs62 = NULL;
  const mxArray *c3_lhs62 = NULL;
  const mxArray *c3_rhs63 = NULL;
  const mxArray *c3_lhs63 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c3_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c3_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c3_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c3_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c3_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c3_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c3_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("se2"), "name", "name", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/robot/se2.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140128U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c3_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/robot/se2.m"),
                  "context", "context", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("length"), "name", "name", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1303153406U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c3_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/robot/se2.m"),
                  "context", "context", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("cos"), "name", "name", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c3_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c3_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/robot/se2.m"),
                  "context", "context", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sin"), "name", "name", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c3_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c3_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("laserScanner"), "name", "name",
                  13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140122U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c3_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c3_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("round"), "name", "name", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c3_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c3_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_round"), "name",
                  "name", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1307658438U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c3_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c3_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c3_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c3_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c3_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c3_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c3_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c3_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c3_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c3_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c3_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c3_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c3_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c3_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("XYtoIJ"), "name", "name", 31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/XYtoIJ.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140122U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c3_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/XYtoIJ.m"),
                  "context", "context", 32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name", 32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c3_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/XYtoIJ.m"),
                  "context", "context", 33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("round"), "name", "name", 33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c3_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("cos"), "name", "name", 34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c3_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sin"), "name", "name", 35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c3_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("clipLine"), "name", "name", 36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140123U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c3_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("hypot"), "name", "name", 37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/hypot.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837580U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c3_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/hypot.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c3_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/hypot.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c3_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c3_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/hypot.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_hypot"), "name",
                  "name", 41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_hypot.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c3_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_hypot.m"),
                  "context", "context", 42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c3_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_hypot.m"),
                  "context", "context", 43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_dlapy2"), "name", "name",
                  43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_dlapy2.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1350417854U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c3_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intersectLines"), "name",
                  "name", 44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140123U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c3_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "context", "context", 45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("max"), "name", "name", 45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311262516U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c3_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378303184U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c3_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c3_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c3_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c3_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c3_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c3_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("bsxfun"), "name", "name", 52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "resolved",
                  "resolved", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1356545094U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c3_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c3_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m!bsxfun_compatible"),
                  "context", "context", 54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311262518U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c3_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1378303184U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c3_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("max"), "name", "name", 56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311262516U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c3_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c3_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c3_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c3_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m!calc_page_back_vector"),
                  "context", "context", 60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c3_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c3_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c3_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 63);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 63);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c3_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c3_rhs0);
  sf_mex_destroy(&c3_lhs0);
  sf_mex_destroy(&c3_rhs1);
  sf_mex_destroy(&c3_lhs1);
  sf_mex_destroy(&c3_rhs2);
  sf_mex_destroy(&c3_lhs2);
  sf_mex_destroy(&c3_rhs3);
  sf_mex_destroy(&c3_lhs3);
  sf_mex_destroy(&c3_rhs4);
  sf_mex_destroy(&c3_lhs4);
  sf_mex_destroy(&c3_rhs5);
  sf_mex_destroy(&c3_lhs5);
  sf_mex_destroy(&c3_rhs6);
  sf_mex_destroy(&c3_lhs6);
  sf_mex_destroy(&c3_rhs7);
  sf_mex_destroy(&c3_lhs7);
  sf_mex_destroy(&c3_rhs8);
  sf_mex_destroy(&c3_lhs8);
  sf_mex_destroy(&c3_rhs9);
  sf_mex_destroy(&c3_lhs9);
  sf_mex_destroy(&c3_rhs10);
  sf_mex_destroy(&c3_lhs10);
  sf_mex_destroy(&c3_rhs11);
  sf_mex_destroy(&c3_lhs11);
  sf_mex_destroy(&c3_rhs12);
  sf_mex_destroy(&c3_lhs12);
  sf_mex_destroy(&c3_rhs13);
  sf_mex_destroy(&c3_lhs13);
  sf_mex_destroy(&c3_rhs14);
  sf_mex_destroy(&c3_lhs14);
  sf_mex_destroy(&c3_rhs15);
  sf_mex_destroy(&c3_lhs15);
  sf_mex_destroy(&c3_rhs16);
  sf_mex_destroy(&c3_lhs16);
  sf_mex_destroy(&c3_rhs17);
  sf_mex_destroy(&c3_lhs17);
  sf_mex_destroy(&c3_rhs18);
  sf_mex_destroy(&c3_lhs18);
  sf_mex_destroy(&c3_rhs19);
  sf_mex_destroy(&c3_lhs19);
  sf_mex_destroy(&c3_rhs20);
  sf_mex_destroy(&c3_lhs20);
  sf_mex_destroy(&c3_rhs21);
  sf_mex_destroy(&c3_lhs21);
  sf_mex_destroy(&c3_rhs22);
  sf_mex_destroy(&c3_lhs22);
  sf_mex_destroy(&c3_rhs23);
  sf_mex_destroy(&c3_lhs23);
  sf_mex_destroy(&c3_rhs24);
  sf_mex_destroy(&c3_lhs24);
  sf_mex_destroy(&c3_rhs25);
  sf_mex_destroy(&c3_lhs25);
  sf_mex_destroy(&c3_rhs26);
  sf_mex_destroy(&c3_lhs26);
  sf_mex_destroy(&c3_rhs27);
  sf_mex_destroy(&c3_lhs27);
  sf_mex_destroy(&c3_rhs28);
  sf_mex_destroy(&c3_lhs28);
  sf_mex_destroy(&c3_rhs29);
  sf_mex_destroy(&c3_lhs29);
  sf_mex_destroy(&c3_rhs30);
  sf_mex_destroy(&c3_lhs30);
  sf_mex_destroy(&c3_rhs31);
  sf_mex_destroy(&c3_lhs31);
  sf_mex_destroy(&c3_rhs32);
  sf_mex_destroy(&c3_lhs32);
  sf_mex_destroy(&c3_rhs33);
  sf_mex_destroy(&c3_lhs33);
  sf_mex_destroy(&c3_rhs34);
  sf_mex_destroy(&c3_lhs34);
  sf_mex_destroy(&c3_rhs35);
  sf_mex_destroy(&c3_lhs35);
  sf_mex_destroy(&c3_rhs36);
  sf_mex_destroy(&c3_lhs36);
  sf_mex_destroy(&c3_rhs37);
  sf_mex_destroy(&c3_lhs37);
  sf_mex_destroy(&c3_rhs38);
  sf_mex_destroy(&c3_lhs38);
  sf_mex_destroy(&c3_rhs39);
  sf_mex_destroy(&c3_lhs39);
  sf_mex_destroy(&c3_rhs40);
  sf_mex_destroy(&c3_lhs40);
  sf_mex_destroy(&c3_rhs41);
  sf_mex_destroy(&c3_lhs41);
  sf_mex_destroy(&c3_rhs42);
  sf_mex_destroy(&c3_lhs42);
  sf_mex_destroy(&c3_rhs43);
  sf_mex_destroy(&c3_lhs43);
  sf_mex_destroy(&c3_rhs44);
  sf_mex_destroy(&c3_lhs44);
  sf_mex_destroy(&c3_rhs45);
  sf_mex_destroy(&c3_lhs45);
  sf_mex_destroy(&c3_rhs46);
  sf_mex_destroy(&c3_lhs46);
  sf_mex_destroy(&c3_rhs47);
  sf_mex_destroy(&c3_lhs47);
  sf_mex_destroy(&c3_rhs48);
  sf_mex_destroy(&c3_lhs48);
  sf_mex_destroy(&c3_rhs49);
  sf_mex_destroy(&c3_lhs49);
  sf_mex_destroy(&c3_rhs50);
  sf_mex_destroy(&c3_lhs50);
  sf_mex_destroy(&c3_rhs51);
  sf_mex_destroy(&c3_lhs51);
  sf_mex_destroy(&c3_rhs52);
  sf_mex_destroy(&c3_lhs52);
  sf_mex_destroy(&c3_rhs53);
  sf_mex_destroy(&c3_lhs53);
  sf_mex_destroy(&c3_rhs54);
  sf_mex_destroy(&c3_lhs54);
  sf_mex_destroy(&c3_rhs55);
  sf_mex_destroy(&c3_lhs55);
  sf_mex_destroy(&c3_rhs56);
  sf_mex_destroy(&c3_lhs56);
  sf_mex_destroy(&c3_rhs57);
  sf_mex_destroy(&c3_lhs57);
  sf_mex_destroy(&c3_rhs58);
  sf_mex_destroy(&c3_lhs58);
  sf_mex_destroy(&c3_rhs59);
  sf_mex_destroy(&c3_lhs59);
  sf_mex_destroy(&c3_rhs60);
  sf_mex_destroy(&c3_lhs60);
  sf_mex_destroy(&c3_rhs61);
  sf_mex_destroy(&c3_lhs61);
  sf_mex_destroy(&c3_rhs62);
  sf_mex_destroy(&c3_lhs62);
  sf_mex_destroy(&c3_rhs63);
  sf_mex_destroy(&c3_lhs63);
}

static const mxArray *c3_emlrt_marshallOut(const char * c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c3_u)), false);
  return c3_y;
}

static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 7, 0U, 0U, 0U, 0), false);
  return c3_y;
}

static void c3_b_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs64 = NULL;
  const mxArray *c3_lhs64 = NULL;
  const mxArray *c3_rhs65 = NULL;
  const mxArray *c3_lhs65 = NULL;
  const mxArray *c3_rhs66 = NULL;
  const mxArray *c3_lhs66 = NULL;
  const mxArray *c3_rhs67 = NULL;
  const mxArray *c3_lhs67 = NULL;
  const mxArray *c3_rhs68 = NULL;
  const mxArray *c3_lhs68 = NULL;
  const mxArray *c3_rhs69 = NULL;
  const mxArray *c3_lhs69 = NULL;
  const mxArray *c3_rhs70 = NULL;
  const mxArray *c3_lhs70 = NULL;
  const mxArray *c3_rhs71 = NULL;
  const mxArray *c3_lhs71 = NULL;
  const mxArray *c3_rhs72 = NULL;
  const mxArray *c3_lhs72 = NULL;
  const mxArray *c3_rhs73 = NULL;
  const mxArray *c3_lhs73 = NULL;
  const mxArray *c3_rhs74 = NULL;
  const mxArray *c3_lhs74 = NULL;
  const mxArray *c3_rhs75 = NULL;
  const mxArray *c3_lhs75 = NULL;
  const mxArray *c3_rhs76 = NULL;
  const mxArray *c3_lhs76 = NULL;
  const mxArray *c3_rhs77 = NULL;
  const mxArray *c3_lhs77 = NULL;
  const mxArray *c3_rhs78 = NULL;
  const mxArray *c3_lhs78 = NULL;
  const mxArray *c3_rhs79 = NULL;
  const mxArray *c3_lhs79 = NULL;
  const mxArray *c3_rhs80 = NULL;
  const mxArray *c3_lhs80 = NULL;
  const mxArray *c3_rhs81 = NULL;
  const mxArray *c3_lhs81 = NULL;
  const mxArray *c3_rhs82 = NULL;
  const mxArray *c3_lhs82 = NULL;
  const mxArray *c3_rhs83 = NULL;
  const mxArray *c3_lhs83 = NULL;
  const mxArray *c3_rhs84 = NULL;
  const mxArray *c3_lhs84 = NULL;
  const mxArray *c3_rhs85 = NULL;
  const mxArray *c3_lhs85 = NULL;
  const mxArray *c3_rhs86 = NULL;
  const mxArray *c3_lhs86 = NULL;
  const mxArray *c3_rhs87 = NULL;
  const mxArray *c3_lhs87 = NULL;
  const mxArray *c3_rhs88 = NULL;
  const mxArray *c3_lhs88 = NULL;
  const mxArray *c3_rhs89 = NULL;
  const mxArray *c3_lhs89 = NULL;
  const mxArray *c3_rhs90 = NULL;
  const mxArray *c3_lhs90 = NULL;
  const mxArray *c3_rhs91 = NULL;
  const mxArray *c3_lhs91 = NULL;
  const mxArray *c3_rhs92 = NULL;
  const mxArray *c3_lhs92 = NULL;
  const mxArray *c3_rhs93 = NULL;
  const mxArray *c3_lhs93 = NULL;
  const mxArray *c3_rhs94 = NULL;
  const mxArray *c3_lhs94 = NULL;
  const mxArray *c3_rhs95 = NULL;
  const mxArray *c3_lhs95 = NULL;
  const mxArray *c3_rhs96 = NULL;
  const mxArray *c3_lhs96 = NULL;
  const mxArray *c3_rhs97 = NULL;
  const mxArray *c3_lhs97 = NULL;
  const mxArray *c3_rhs98 = NULL;
  const mxArray *c3_lhs98 = NULL;
  const mxArray *c3_rhs99 = NULL;
  const mxArray *c3_lhs99 = NULL;
  const mxArray *c3_rhs100 = NULL;
  const mxArray *c3_lhs100 = NULL;
  const mxArray *c3_rhs101 = NULL;
  const mxArray *c3_lhs101 = NULL;
  const mxArray *c3_rhs102 = NULL;
  const mxArray *c3_lhs102 = NULL;
  const mxArray *c3_rhs103 = NULL;
  const mxArray *c3_lhs103 = NULL;
  const mxArray *c3_rhs104 = NULL;
  const mxArray *c3_lhs104 = NULL;
  const mxArray *c3_rhs105 = NULL;
  const mxArray *c3_lhs105 = NULL;
  const mxArray *c3_rhs106 = NULL;
  const mxArray *c3_lhs106 = NULL;
  const mxArray *c3_rhs107 = NULL;
  const mxArray *c3_lhs107 = NULL;
  const mxArray *c3_rhs108 = NULL;
  const mxArray *c3_lhs108 = NULL;
  const mxArray *c3_rhs109 = NULL;
  const mxArray *c3_lhs109 = NULL;
  const mxArray *c3_rhs110 = NULL;
  const mxArray *c3_lhs110 = NULL;
  const mxArray *c3_rhs111 = NULL;
  const mxArray *c3_lhs111 = NULL;
  const mxArray *c3_rhs112 = NULL;
  const mxArray *c3_lhs112 = NULL;
  const mxArray *c3_rhs113 = NULL;
  const mxArray *c3_lhs113 = NULL;
  const mxArray *c3_rhs114 = NULL;
  const mxArray *c3_lhs114 = NULL;
  const mxArray *c3_rhs115 = NULL;
  const mxArray *c3_lhs115 = NULL;
  const mxArray *c3_rhs116 = NULL;
  const mxArray *c3_lhs116 = NULL;
  const mxArray *c3_rhs117 = NULL;
  const mxArray *c3_lhs117 = NULL;
  const mxArray *c3_rhs118 = NULL;
  const mxArray *c3_lhs118 = NULL;
  const mxArray *c3_rhs119 = NULL;
  const mxArray *c3_lhs119 = NULL;
  const mxArray *c3_rhs120 = NULL;
  const mxArray *c3_lhs120 = NULL;
  const mxArray *c3_rhs121 = NULL;
  const mxArray *c3_lhs121 = NULL;
  const mxArray *c3_rhs122 = NULL;
  const mxArray *c3_lhs122 = NULL;
  const mxArray *c3_rhs123 = NULL;
  const mxArray *c3_lhs123 = NULL;
  const mxArray *c3_rhs124 = NULL;
  const mxArray *c3_lhs124 = NULL;
  const mxArray *c3_rhs125 = NULL;
  const mxArray *c3_lhs125 = NULL;
  const mxArray *c3_rhs126 = NULL;
  const mxArray *c3_lhs126 = NULL;
  const mxArray *c3_rhs127 = NULL;
  const mxArray *c3_lhs127 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c3_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "context", "context", 65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("abs"), "name", "name", 65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717452U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c3_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c3_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825912U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c3_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "context", "context", 68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("all"), "name", "name", 68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589614U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c3_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c3_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c3_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.allOrAny"),
                  "name", "name", 71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590358U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c3_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c3_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825958U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c3_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825986U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c3_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c3_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "context", "context", 76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_li_find"), "name", "name",
                  76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825986U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c3_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c3_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c3_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c3_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c3_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"),
                  "context", "context", 81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c3_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isfinite"), "name", "name", 82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c3_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c3_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isinf"), "name", "name", 84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c3_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c3_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c3_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c3_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_li_find"), "name", "name",
                  88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "resolved",
                  "resolved", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825986U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c3_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c3_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c3_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c3_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c3_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("linePosition"), "name", "name",
                  93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/linePosition.m"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140123U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c3_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/linePosition.m"),
                  "context", "context", 94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("bsxfun"), "name", "name", 94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "resolved",
                  "resolved", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1356545094U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c3_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m!no_dynamic_expansion"),
                  "context", "context", 95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("min"), "name", "name", 95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1311262518U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c3_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c3_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmin"), "name", "name", 97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c3_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c3_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c3_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c3_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/linePosition.m"),
                  "context", "context", 101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eps"), "name", "name", 101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c3_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_eps"), "name", "name", 102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c3_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c3_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/linePosition.m"),
                  "context", "context", 104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_li_find"), "name", "name",
                  104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "resolved",
                  "resolved", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825986U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c3_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/linePosition.m"),
                  "context", "context", 105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c3_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c3_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "context", "context", 107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825958U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c3_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m!isequal_scalar"),
                  "context", "context", 108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c3_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sort"), "name", "name", 109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "resolved",
                  "resolved", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c3_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "context",
                  "context", 110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c3_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "context",
                  "context", 111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_sort"), "name", "name",
                  111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "resolved",
                  "resolved", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1314743812U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c3_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_nonsingleton_dim"), "name",
                  "name", 112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_nonsingleton_dim.m"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1307658442U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c3_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_nonsingleton_dim.m"),
                  "context", "context", 113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c3_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c3_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assertValidDim"),
                  "name", "name", 115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c3_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c3_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("floor"), "name", "name", 117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c3_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c3_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c3_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("intmax"), "name", "name", 120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c3_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c3_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c3_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_matrix_vstride"), "name",
                  "name", 123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360285950U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c3_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "context", "context", 124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360286188U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c3_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "context", "context", 125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c3_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c3_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 127);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 127);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 127);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c3_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c3_rhs64);
  sf_mex_destroy(&c3_lhs64);
  sf_mex_destroy(&c3_rhs65);
  sf_mex_destroy(&c3_lhs65);
  sf_mex_destroy(&c3_rhs66);
  sf_mex_destroy(&c3_lhs66);
  sf_mex_destroy(&c3_rhs67);
  sf_mex_destroy(&c3_lhs67);
  sf_mex_destroy(&c3_rhs68);
  sf_mex_destroy(&c3_lhs68);
  sf_mex_destroy(&c3_rhs69);
  sf_mex_destroy(&c3_lhs69);
  sf_mex_destroy(&c3_rhs70);
  sf_mex_destroy(&c3_lhs70);
  sf_mex_destroy(&c3_rhs71);
  sf_mex_destroy(&c3_lhs71);
  sf_mex_destroy(&c3_rhs72);
  sf_mex_destroy(&c3_lhs72);
  sf_mex_destroy(&c3_rhs73);
  sf_mex_destroy(&c3_lhs73);
  sf_mex_destroy(&c3_rhs74);
  sf_mex_destroy(&c3_lhs74);
  sf_mex_destroy(&c3_rhs75);
  sf_mex_destroy(&c3_lhs75);
  sf_mex_destroy(&c3_rhs76);
  sf_mex_destroy(&c3_lhs76);
  sf_mex_destroy(&c3_rhs77);
  sf_mex_destroy(&c3_lhs77);
  sf_mex_destroy(&c3_rhs78);
  sf_mex_destroy(&c3_lhs78);
  sf_mex_destroy(&c3_rhs79);
  sf_mex_destroy(&c3_lhs79);
  sf_mex_destroy(&c3_rhs80);
  sf_mex_destroy(&c3_lhs80);
  sf_mex_destroy(&c3_rhs81);
  sf_mex_destroy(&c3_lhs81);
  sf_mex_destroy(&c3_rhs82);
  sf_mex_destroy(&c3_lhs82);
  sf_mex_destroy(&c3_rhs83);
  sf_mex_destroy(&c3_lhs83);
  sf_mex_destroy(&c3_rhs84);
  sf_mex_destroy(&c3_lhs84);
  sf_mex_destroy(&c3_rhs85);
  sf_mex_destroy(&c3_lhs85);
  sf_mex_destroy(&c3_rhs86);
  sf_mex_destroy(&c3_lhs86);
  sf_mex_destroy(&c3_rhs87);
  sf_mex_destroy(&c3_lhs87);
  sf_mex_destroy(&c3_rhs88);
  sf_mex_destroy(&c3_lhs88);
  sf_mex_destroy(&c3_rhs89);
  sf_mex_destroy(&c3_lhs89);
  sf_mex_destroy(&c3_rhs90);
  sf_mex_destroy(&c3_lhs90);
  sf_mex_destroy(&c3_rhs91);
  sf_mex_destroy(&c3_lhs91);
  sf_mex_destroy(&c3_rhs92);
  sf_mex_destroy(&c3_lhs92);
  sf_mex_destroy(&c3_rhs93);
  sf_mex_destroy(&c3_lhs93);
  sf_mex_destroy(&c3_rhs94);
  sf_mex_destroy(&c3_lhs94);
  sf_mex_destroy(&c3_rhs95);
  sf_mex_destroy(&c3_lhs95);
  sf_mex_destroy(&c3_rhs96);
  sf_mex_destroy(&c3_lhs96);
  sf_mex_destroy(&c3_rhs97);
  sf_mex_destroy(&c3_lhs97);
  sf_mex_destroy(&c3_rhs98);
  sf_mex_destroy(&c3_lhs98);
  sf_mex_destroy(&c3_rhs99);
  sf_mex_destroy(&c3_lhs99);
  sf_mex_destroy(&c3_rhs100);
  sf_mex_destroy(&c3_lhs100);
  sf_mex_destroy(&c3_rhs101);
  sf_mex_destroy(&c3_lhs101);
  sf_mex_destroy(&c3_rhs102);
  sf_mex_destroy(&c3_lhs102);
  sf_mex_destroy(&c3_rhs103);
  sf_mex_destroy(&c3_lhs103);
  sf_mex_destroy(&c3_rhs104);
  sf_mex_destroy(&c3_lhs104);
  sf_mex_destroy(&c3_rhs105);
  sf_mex_destroy(&c3_lhs105);
  sf_mex_destroy(&c3_rhs106);
  sf_mex_destroy(&c3_lhs106);
  sf_mex_destroy(&c3_rhs107);
  sf_mex_destroy(&c3_lhs107);
  sf_mex_destroy(&c3_rhs108);
  sf_mex_destroy(&c3_lhs108);
  sf_mex_destroy(&c3_rhs109);
  sf_mex_destroy(&c3_lhs109);
  sf_mex_destroy(&c3_rhs110);
  sf_mex_destroy(&c3_lhs110);
  sf_mex_destroy(&c3_rhs111);
  sf_mex_destroy(&c3_lhs111);
  sf_mex_destroy(&c3_rhs112);
  sf_mex_destroy(&c3_lhs112);
  sf_mex_destroy(&c3_rhs113);
  sf_mex_destroy(&c3_lhs113);
  sf_mex_destroy(&c3_rhs114);
  sf_mex_destroy(&c3_lhs114);
  sf_mex_destroy(&c3_rhs115);
  sf_mex_destroy(&c3_lhs115);
  sf_mex_destroy(&c3_rhs116);
  sf_mex_destroy(&c3_lhs116);
  sf_mex_destroy(&c3_rhs117);
  sf_mex_destroy(&c3_lhs117);
  sf_mex_destroy(&c3_rhs118);
  sf_mex_destroy(&c3_lhs118);
  sf_mex_destroy(&c3_rhs119);
  sf_mex_destroy(&c3_lhs119);
  sf_mex_destroy(&c3_rhs120);
  sf_mex_destroy(&c3_lhs120);
  sf_mex_destroy(&c3_rhs121);
  sf_mex_destroy(&c3_lhs121);
  sf_mex_destroy(&c3_rhs122);
  sf_mex_destroy(&c3_lhs122);
  sf_mex_destroy(&c3_rhs123);
  sf_mex_destroy(&c3_lhs123);
  sf_mex_destroy(&c3_rhs124);
  sf_mex_destroy(&c3_lhs124);
  sf_mex_destroy(&c3_rhs125);
  sf_mex_destroy(&c3_lhs125);
  sf_mex_destroy(&c3_rhs126);
  sf_mex_destroy(&c3_lhs126);
  sf_mex_destroy(&c3_rhs127);
  sf_mex_destroy(&c3_lhs127);
}

static void c3_c_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs128 = NULL;
  const mxArray *c3_lhs128 = NULL;
  const mxArray *c3_rhs129 = NULL;
  const mxArray *c3_lhs129 = NULL;
  const mxArray *c3_rhs130 = NULL;
  const mxArray *c3_lhs130 = NULL;
  const mxArray *c3_rhs131 = NULL;
  const mxArray *c3_lhs131 = NULL;
  const mxArray *c3_rhs132 = NULL;
  const mxArray *c3_lhs132 = NULL;
  const mxArray *c3_rhs133 = NULL;
  const mxArray *c3_lhs133 = NULL;
  const mxArray *c3_rhs134 = NULL;
  const mxArray *c3_lhs134 = NULL;
  const mxArray *c3_rhs135 = NULL;
  const mxArray *c3_lhs135 = NULL;
  const mxArray *c3_rhs136 = NULL;
  const mxArray *c3_lhs136 = NULL;
  const mxArray *c3_rhs137 = NULL;
  const mxArray *c3_lhs137 = NULL;
  const mxArray *c3_rhs138 = NULL;
  const mxArray *c3_lhs138 = NULL;
  const mxArray *c3_rhs139 = NULL;
  const mxArray *c3_lhs139 = NULL;
  const mxArray *c3_rhs140 = NULL;
  const mxArray *c3_lhs140 = NULL;
  const mxArray *c3_rhs141 = NULL;
  const mxArray *c3_lhs141 = NULL;
  const mxArray *c3_rhs142 = NULL;
  const mxArray *c3_lhs142 = NULL;
  const mxArray *c3_rhs143 = NULL;
  const mxArray *c3_lhs143 = NULL;
  const mxArray *c3_rhs144 = NULL;
  const mxArray *c3_lhs144 = NULL;
  const mxArray *c3_rhs145 = NULL;
  const mxArray *c3_lhs145 = NULL;
  const mxArray *c3_rhs146 = NULL;
  const mxArray *c3_lhs146 = NULL;
  const mxArray *c3_rhs147 = NULL;
  const mxArray *c3_lhs147 = NULL;
  const mxArray *c3_rhs148 = NULL;
  const mxArray *c3_lhs148 = NULL;
  const mxArray *c3_rhs149 = NULL;
  const mxArray *c3_lhs149 = NULL;
  const mxArray *c3_rhs150 = NULL;
  const mxArray *c3_lhs150 = NULL;
  const mxArray *c3_rhs151 = NULL;
  const mxArray *c3_lhs151 = NULL;
  const mxArray *c3_rhs152 = NULL;
  const mxArray *c3_lhs152 = NULL;
  const mxArray *c3_rhs153 = NULL;
  const mxArray *c3_lhs153 = NULL;
  const mxArray *c3_rhs154 = NULL;
  const mxArray *c3_lhs154 = NULL;
  const mxArray *c3_rhs155 = NULL;
  const mxArray *c3_lhs155 = NULL;
  const mxArray *c3_rhs156 = NULL;
  const mxArray *c3_lhs156 = NULL;
  const mxArray *c3_rhs157 = NULL;
  const mxArray *c3_lhs157 = NULL;
  const mxArray *c3_rhs158 = NULL;
  const mxArray *c3_lhs158 = NULL;
  const mxArray *c3_rhs159 = NULL;
  const mxArray *c3_lhs159 = NULL;
  const mxArray *c3_rhs160 = NULL;
  const mxArray *c3_lhs160 = NULL;
  const mxArray *c3_rhs161 = NULL;
  const mxArray *c3_lhs161 = NULL;
  const mxArray *c3_rhs162 = NULL;
  const mxArray *c3_lhs162 = NULL;
  const mxArray *c3_rhs163 = NULL;
  const mxArray *c3_lhs163 = NULL;
  const mxArray *c3_rhs164 = NULL;
  const mxArray *c3_lhs164 = NULL;
  const mxArray *c3_rhs165 = NULL;
  const mxArray *c3_lhs165 = NULL;
  const mxArray *c3_rhs166 = NULL;
  const mxArray *c3_lhs166 = NULL;
  const mxArray *c3_rhs167 = NULL;
  const mxArray *c3_lhs167 = NULL;
  const mxArray *c3_rhs168 = NULL;
  const mxArray *c3_lhs168 = NULL;
  const mxArray *c3_rhs169 = NULL;
  const mxArray *c3_lhs169 = NULL;
  const mxArray *c3_rhs170 = NULL;
  const mxArray *c3_lhs170 = NULL;
  const mxArray *c3_rhs171 = NULL;
  const mxArray *c3_lhs171 = NULL;
  const mxArray *c3_rhs172 = NULL;
  const mxArray *c3_lhs172 = NULL;
  const mxArray *c3_rhs173 = NULL;
  const mxArray *c3_lhs173 = NULL;
  const mxArray *c3_rhs174 = NULL;
  const mxArray *c3_lhs174 = NULL;
  const mxArray *c3_rhs175 = NULL;
  const mxArray *c3_lhs175 = NULL;
  const mxArray *c3_rhs176 = NULL;
  const mxArray *c3_lhs176 = NULL;
  const mxArray *c3_rhs177 = NULL;
  const mxArray *c3_lhs177 = NULL;
  const mxArray *c3_rhs178 = NULL;
  const mxArray *c3_lhs178 = NULL;
  const mxArray *c3_rhs179 = NULL;
  const mxArray *c3_lhs179 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c3_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_matrix_npages"), "name",
                  "name", 129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360285950U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c3_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "context", "context", 130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1360286188U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c3_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c3_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c3_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c3_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_sort_idx"), "name", "name",
                  134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "resolved",
                  "resolved", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1305325204U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c3_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c3_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c3_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c3_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c3_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c3_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_sort_le"), "name", "name",
                  140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m"), "resolved",
                  "resolved", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1292194110U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c3_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m!eml_sort_ascending_le"),
                  "context", "context", 141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_relop"), "name", "name",
                  141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1342458382U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c3_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m!eml_sort_ascending_le"),
                  "context", "context", 142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c3_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c3_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c3_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c3_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name",
                  146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c3_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"),
                  "context", "context", 147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mean"), "name", "name", 147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "resolved",
                  "resolved", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1383880884U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c3_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "context",
                  "context", 148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c3_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "context",
                  "context", 149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c3_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "context",
                  "context", 150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825958U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c3_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "context",
                  "context", 151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c3_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "context",
                  "context", 152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sum"), "name", "name", 152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c3_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c3_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c3_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assertValidDim"),
                  "name", "name", 155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c3_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c3_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("floor"), "name", "name", 157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c3_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c3_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c3_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c3_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c3_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/mean.m"), "context",
                  "context", 162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c3_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("laserRange"), "name", "name",
                  163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/common/laserRange.m"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140125U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c3_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/common/laserRange.m"),
                  "context", "context", 164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("all"), "name", "name", 164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "resolved",
                  "resolved", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589614U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c3_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isinf"), "name", "name", 165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c3_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("IJtoXY"), "name", "name", 166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/IJtoXY.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1464140122U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c3_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/IJtoXY.m"),
                  "context", "context", 167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name",
                  167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c3_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mpower"), "name", "name", 168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717478U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c3_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c3_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("ismatrix"), "name", "name",
                  170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c3_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("power"), "name", "name", 171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c3_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c3_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c3_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c3_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("floor"), "name", "name", 175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c3_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c3_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[E]C:/Users/jpacker/stash/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"),
                  "context", "context", 177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("sqrt"), "name", "name", 177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c3_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_error"), "name", "name",
                  178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1343837558U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c3_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825938U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c3_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs179), "lhs", "lhs",
                  179);
  sf_mex_destroy(&c3_rhs128);
  sf_mex_destroy(&c3_lhs128);
  sf_mex_destroy(&c3_rhs129);
  sf_mex_destroy(&c3_lhs129);
  sf_mex_destroy(&c3_rhs130);
  sf_mex_destroy(&c3_lhs130);
  sf_mex_destroy(&c3_rhs131);
  sf_mex_destroy(&c3_lhs131);
  sf_mex_destroy(&c3_rhs132);
  sf_mex_destroy(&c3_lhs132);
  sf_mex_destroy(&c3_rhs133);
  sf_mex_destroy(&c3_lhs133);
  sf_mex_destroy(&c3_rhs134);
  sf_mex_destroy(&c3_lhs134);
  sf_mex_destroy(&c3_rhs135);
  sf_mex_destroy(&c3_lhs135);
  sf_mex_destroy(&c3_rhs136);
  sf_mex_destroy(&c3_lhs136);
  sf_mex_destroy(&c3_rhs137);
  sf_mex_destroy(&c3_lhs137);
  sf_mex_destroy(&c3_rhs138);
  sf_mex_destroy(&c3_lhs138);
  sf_mex_destroy(&c3_rhs139);
  sf_mex_destroy(&c3_lhs139);
  sf_mex_destroy(&c3_rhs140);
  sf_mex_destroy(&c3_lhs140);
  sf_mex_destroy(&c3_rhs141);
  sf_mex_destroy(&c3_lhs141);
  sf_mex_destroy(&c3_rhs142);
  sf_mex_destroy(&c3_lhs142);
  sf_mex_destroy(&c3_rhs143);
  sf_mex_destroy(&c3_lhs143);
  sf_mex_destroy(&c3_rhs144);
  sf_mex_destroy(&c3_lhs144);
  sf_mex_destroy(&c3_rhs145);
  sf_mex_destroy(&c3_lhs145);
  sf_mex_destroy(&c3_rhs146);
  sf_mex_destroy(&c3_lhs146);
  sf_mex_destroy(&c3_rhs147);
  sf_mex_destroy(&c3_lhs147);
  sf_mex_destroy(&c3_rhs148);
  sf_mex_destroy(&c3_lhs148);
  sf_mex_destroy(&c3_rhs149);
  sf_mex_destroy(&c3_lhs149);
  sf_mex_destroy(&c3_rhs150);
  sf_mex_destroy(&c3_lhs150);
  sf_mex_destroy(&c3_rhs151);
  sf_mex_destroy(&c3_lhs151);
  sf_mex_destroy(&c3_rhs152);
  sf_mex_destroy(&c3_lhs152);
  sf_mex_destroy(&c3_rhs153);
  sf_mex_destroy(&c3_lhs153);
  sf_mex_destroy(&c3_rhs154);
  sf_mex_destroy(&c3_lhs154);
  sf_mex_destroy(&c3_rhs155);
  sf_mex_destroy(&c3_lhs155);
  sf_mex_destroy(&c3_rhs156);
  sf_mex_destroy(&c3_lhs156);
  sf_mex_destroy(&c3_rhs157);
  sf_mex_destroy(&c3_lhs157);
  sf_mex_destroy(&c3_rhs158);
  sf_mex_destroy(&c3_lhs158);
  sf_mex_destroy(&c3_rhs159);
  sf_mex_destroy(&c3_lhs159);
  sf_mex_destroy(&c3_rhs160);
  sf_mex_destroy(&c3_lhs160);
  sf_mex_destroy(&c3_rhs161);
  sf_mex_destroy(&c3_lhs161);
  sf_mex_destroy(&c3_rhs162);
  sf_mex_destroy(&c3_lhs162);
  sf_mex_destroy(&c3_rhs163);
  sf_mex_destroy(&c3_lhs163);
  sf_mex_destroy(&c3_rhs164);
  sf_mex_destroy(&c3_lhs164);
  sf_mex_destroy(&c3_rhs165);
  sf_mex_destroy(&c3_lhs165);
  sf_mex_destroy(&c3_rhs166);
  sf_mex_destroy(&c3_lhs166);
  sf_mex_destroy(&c3_rhs167);
  sf_mex_destroy(&c3_lhs167);
  sf_mex_destroy(&c3_rhs168);
  sf_mex_destroy(&c3_lhs168);
  sf_mex_destroy(&c3_rhs169);
  sf_mex_destroy(&c3_lhs169);
  sf_mex_destroy(&c3_rhs170);
  sf_mex_destroy(&c3_lhs170);
  sf_mex_destroy(&c3_rhs171);
  sf_mex_destroy(&c3_lhs171);
  sf_mex_destroy(&c3_rhs172);
  sf_mex_destroy(&c3_lhs172);
  sf_mex_destroy(&c3_rhs173);
  sf_mex_destroy(&c3_lhs173);
  sf_mex_destroy(&c3_rhs174);
  sf_mex_destroy(&c3_lhs174);
  sf_mex_destroy(&c3_rhs175);
  sf_mex_destroy(&c3_lhs175);
  sf_mex_destroy(&c3_rhs176);
  sf_mex_destroy(&c3_lhs176);
  sf_mex_destroy(&c3_rhs177);
  sf_mex_destroy(&c3_lhs177);
  sf_mex_destroy(&c3_rhs178);
  sf_mex_destroy(&c3_lhs178);
  sf_mex_destroy(&c3_rhs179);
  sf_mex_destroy(&c3_lhs179);
}

static void c3_eml_scalar_eg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_xgemm(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_A[9], real_T c3_B[3], real_T c3_C[3], real_T c3_b_C[3])
{
  int32_T c3_i244;
  int32_T c3_i245;
  real_T c3_b_A[9];
  int32_T c3_i246;
  real_T c3_b_B[3];
  for (c3_i244 = 0; c3_i244 < 3; c3_i244++) {
    c3_b_C[c3_i244] = c3_C[c3_i244];
  }

  for (c3_i245 = 0; c3_i245 < 9; c3_i245++) {
    c3_b_A[c3_i245] = c3_A[c3_i245];
  }

  for (c3_i246 = 0; c3_i246 < 3; c3_i246++) {
    c3_b_B[c3_i246] = c3_B[c3_i246];
  }

  c3_b_eml_xgemm(chartInstance, c3_b_A, c3_b_B, c3_b_C);
}

static real_T c3_hypot(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x, real_T c3_y)
{
  real_T c3_b_x;
  real_T c3_b_y;
  real_T c3_x1;
  real_T c3_x2;
  real_T c3_a;
  real_T c3_b;
  c3_b_eml_scalar_eg(chartInstance);
  c3_b_x = c3_x;
  c3_b_y = c3_y;
  c3_b_eml_scalar_eg(chartInstance);
  c3_x1 = c3_b_x;
  c3_x2 = c3_b_y;
  c3_a = c3_x1;
  c3_b = c3_x2;
  return muDoubleScalarHypot(c3_a, c3_b);
}

static void c3_b_eml_scalar_eg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c3_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a, real_T c3_b)
{
  real_T c3_av;
  real_T c3_bv;
  real_T c3_cv;
  c3_c_eml_scalar_eg(chartInstance);
  c3_av = c3_a;
  c3_bv = c3_b;
  c3_cv = c3_av - c3_bv;
  return c3_cv;
}

static void c3_c_eml_scalar_eg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_eml_li_find(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, boolean_T c3_x, int32_T c3_y_data[], int32_T c3_y_sizes[2])
{
  boolean_T c3_b_x;
  int32_T c3_k;
  int32_T c3_tmp_sizes[2];
  int32_T c3_iv0[2];
  int32_T c3_i247;
  int32_T c3_i248;
  int32_T c3_loop_ub;
  int32_T c3_i249;
  int32_T c3_tmp_data[1];
  int32_T c3_i250;
  (void)chartInstance;
  c3_b_x = c3_x;
  c3_k = 0;
  if (c3_b_x) {
    c3_k = 1;
  }

  c3_tmp_sizes[0] = 1;
  c3_iv0[0] = 1;
  c3_iv0[1] = c3_k;
  c3_tmp_sizes[1] = c3_iv0[1];
  c3_i247 = c3_tmp_sizes[0];
  c3_i248 = c3_tmp_sizes[1];
  c3_loop_ub = c3_k - 1;
  for (c3_i249 = 0; c3_i249 <= c3_loop_ub; c3_i249++) {
    c3_tmp_data[c3_i249] = 0;
  }

  for (c3_i250 = 0; c3_i250 < 2; c3_i250++) {
    c3_y_sizes[c3_i250] = c3_tmp_sizes[c3_i250];
  }

  if (c3_x) {
    _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c3_y_sizes[1], 1, 0);
    c3_y_data[0] = 1;
  }
}

static void c3_isfinite(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x[4], boolean_T c3_b[4])
{
  int32_T c3_i251;
  int32_T c3_i252;
  int32_T c3_i253;
  boolean_T c3_b_b[4];
  int32_T c3_i254;
  int32_T c3_i255;
  (void)chartInstance;
  for (c3_i251 = 0; c3_i251 < 4; c3_i251++) {
    c3_b[c3_i251] = muDoubleScalarIsInf(c3_x[c3_i251]);
  }

  for (c3_i252 = 0; c3_i252 < 4; c3_i252++) {
    c3_b[c3_i252] = !c3_b[c3_i252];
  }

  for (c3_i253 = 0; c3_i253 < 4; c3_i253++) {
    c3_b_b[c3_i253] = muDoubleScalarIsNaN(c3_x[c3_i253]);
  }

  for (c3_i254 = 0; c3_i254 < 4; c3_i254++) {
    c3_b_b[c3_i254] = !c3_b_b[c3_i254];
  }

  for (c3_i255 = 0; c3_i255 < 4; c3_i255++) {
    c3_b[c3_i255] = (c3_b[c3_i255] && c3_b_b[c3_i255]);
  }
}

static void c3_eml_switch_helper(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_b_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes)
{
  int32_T c3_na1;
  int32_T c3_csz[2];
  int32_T c3_cv_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i256;
  real_T c3_cv_data[4];
  int32_T c3_b_loop_ub;
  int32_T c3_i257;
  int32_T c3_av_sizes;
  int32_T c3_nc1;
  int32_T c3_b_nc1;
  real_T c3_a;
  int32_T c3_b_b;
  real_T c3_b_a;
  int32_T c3_c_b;
  int32_T c3_i258;
  int32_T c3_d_b;
  int32_T c3_e_b;
  int32_T c3_ck;
  int32_T c3_b_ck;
  int32_T c3_b_na1;
  int32_T c3_f_b;
  int32_T c3_g_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_h_b;
  int32_T c3_i_b;
  int32_T c3_c;
  real_T c3_av_data[4];
  real_T c3_bv;
  int32_T c3_c_loop_ub;
  int32_T c3_i259;
  int32_T c3_c_nc1;
  int32_T c3_j_b;
  int32_T c3_k_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  int32_T c3_c_a;
  int32_T c3_l_b;
  int32_T c3_d_a;
  int32_T c3_m_b;
  int32_T c3_b_c;
  c3_na1 = c3_a_sizes;
  c3_csz[0] = c3_a_sizes;
  c3_c_eml_scalar_eg(chartInstance);
  c3_cv_sizes = c3_csz[0];
  c3_loop_ub = c3_csz[0] - 1;
  for (c3_i256 = 0; c3_i256 <= c3_loop_ub; c3_i256++) {
    c3_cv_data[c3_i256] = 0.0;
  }

  *c3_c_sizes = c3_cv_sizes;
  if (*c3_c_sizes == 0) {
  } else {
    c3_csz[0] = c3_na1;
    c3_csz[1] = 1;
    c3_cv_sizes = c3_csz[0];
    c3_b_loop_ub = c3_csz[0] - 1;
    for (c3_i257 = 0; c3_i257 <= c3_b_loop_ub; c3_i257++) {
      c3_cv_data[c3_i257] = 0.0;
    }

    c3_av_sizes = c3_cv_sizes;
    c3_nc1 = *c3_c_sizes;
    c3_b_nc1 = c3_nc1;
    c3_a = (real_T)*c3_c_sizes;
    c3_b_b = c3_nc1;
    c3_b_a = c3_a;
    c3_c_b = c3_b_b;
    c3_i258 = (int32_T)c3_b_a - c3_c_b;
    c3_d_b = c3_i258;
    c3_e_b = c3_d_b;
    if (0 > c3_e_b) {
    } else {
      c3_eml_switch_helper(chartInstance);
    }

    for (c3_ck = 0; c3_ck <= c3_i258; c3_ck += c3_b_nc1) {
      c3_b_ck = c3_ck;
      c3_b_na1 = c3_na1;
      c3_f_b = c3_b_na1;
      c3_g_b = c3_f_b;
      if (1 > c3_g_b) {
        c3_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_overflow = (c3_g_b > 2147483646);
      }

      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_k = 1; c3_k <= c3_b_na1; c3_k++) {
        c3_b_k = c3_k;
        c3_h_b = c3_b_k;
        c3_i_b = c3_h_b;
        c3_c = c3_i_b;
        c3_av_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1, c3_av_sizes, 1, 0)
          - 1] = c3_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c, 1, c3_a_sizes,
          1, 0) - 1];
      }

      c3_bv = c3_b;
      c3_cv_sizes = c3_av_sizes;
      c3_c_loop_ub = c3_av_sizes - 1;
      for (c3_i259 = 0; c3_i259 <= c3_c_loop_ub; c3_i259++) {
        c3_cv_data[c3_i259] = c3_av_data[c3_i259] - c3_bv;
      }

      c3_c_nc1 = c3_nc1;
      c3_j_b = c3_c_nc1;
      c3_k_b = c3_j_b;
      if (1 > c3_k_b) {
        c3_b_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_b_overflow = (c3_k_b > 2147483646);
      }

      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_c_k = 1; c3_c_k <= c3_c_nc1; c3_c_k++) {
        c3_b_k = c3_c_k;
        c3_c_a = c3_b_ck;
        c3_l_b = c3_b_k;
        c3_d_a = c3_c_a;
        c3_m_b = c3_l_b;
        c3_b_c = c3_d_a + c3_m_b;
        c3_c_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_c, 1, *c3_c_sizes, 1, 0)
          - 1] = c3_cv_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
          c3_cv_sizes, 1, 0) - 1];
      }
    }
  }
}

static void c3_check_forloop_overflow_error
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, boolean_T
   c3_overflow)
{
  int32_T c3_i260;
  static char_T c3_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c3_u[34];
  const mxArray *c3_y = NULL;
  int32_T c3_i261;
  static char_T c3_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c3_b_u[23];
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  (void)c3_overflow;
  for (c3_i260 = 0; c3_i260 < 34; c3_i260++) {
    c3_u[c3_i260] = c3_cv0[c3_i260];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 34), false);
  for (c3_i261 = 0; c3_i261 < 23; c3_i261++) {
    c3_b_u[c3_i261] = c3_cv1[c3_i261];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c3_y, 14, c3_b_y));
}

static void c3_c_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes)
{
  int32_T c3_na1;
  int32_T c3_csz[2];
  int32_T c3_cv_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i262;
  real_T c3_cv_data[4];
  int32_T c3_b_loop_ub;
  int32_T c3_i263;
  int32_T c3_av_sizes;
  int32_T c3_nc1;
  int32_T c3_b_nc1;
  real_T c3_a;
  int32_T c3_b_b;
  real_T c3_b_a;
  int32_T c3_c_b;
  int32_T c3_i264;
  int32_T c3_d_b;
  int32_T c3_e_b;
  int32_T c3_ck;
  int32_T c3_b_ck;
  int32_T c3_b_na1;
  int32_T c3_f_b;
  int32_T c3_g_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_h_b;
  int32_T c3_i_b;
  int32_T c3_c;
  real_T c3_av_data[4];
  real_T c3_bv;
  int32_T c3_c_loop_ub;
  int32_T c3_i265;
  int32_T c3_c_nc1;
  int32_T c3_j_b;
  int32_T c3_k_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  int32_T c3_c_a;
  int32_T c3_l_b;
  int32_T c3_d_a;
  int32_T c3_m_b;
  int32_T c3_b_c;
  c3_na1 = c3_a_sizes;
  c3_csz[0] = c3_a_sizes;
  c3_c_eml_scalar_eg(chartInstance);
  c3_cv_sizes = c3_csz[0];
  c3_loop_ub = c3_csz[0] - 1;
  for (c3_i262 = 0; c3_i262 <= c3_loop_ub; c3_i262++) {
    c3_cv_data[c3_i262] = 0.0;
  }

  *c3_c_sizes = c3_cv_sizes;
  if (*c3_c_sizes == 0) {
  } else {
    c3_csz[0] = c3_na1;
    c3_csz[1] = 1;
    c3_cv_sizes = c3_csz[0];
    c3_b_loop_ub = c3_csz[0] - 1;
    for (c3_i263 = 0; c3_i263 <= c3_b_loop_ub; c3_i263++) {
      c3_cv_data[c3_i263] = 0.0;
    }

    c3_av_sizes = c3_cv_sizes;
    c3_nc1 = *c3_c_sizes;
    c3_b_nc1 = c3_nc1;
    c3_a = (real_T)*c3_c_sizes;
    c3_b_b = c3_nc1;
    c3_b_a = c3_a;
    c3_c_b = c3_b_b;
    c3_i264 = (int32_T)c3_b_a - c3_c_b;
    c3_d_b = c3_i264;
    c3_e_b = c3_d_b;
    if (0 > c3_e_b) {
    } else {
      c3_eml_switch_helper(chartInstance);
    }

    for (c3_ck = 0; c3_ck <= c3_i264; c3_ck += c3_b_nc1) {
      c3_b_ck = c3_ck;
      c3_b_na1 = c3_na1;
      c3_f_b = c3_b_na1;
      c3_g_b = c3_f_b;
      if (1 > c3_g_b) {
        c3_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_overflow = (c3_g_b > 2147483646);
      }

      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_k = 1; c3_k <= c3_b_na1; c3_k++) {
        c3_b_k = c3_k;
        c3_h_b = c3_b_k;
        c3_i_b = c3_h_b;
        c3_c = c3_i_b;
        c3_av_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1, c3_av_sizes, 1, 0)
          - 1] = c3_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c, 1, c3_a_sizes,
          1, 0) - 1];
      }

      c3_bv = c3_b;
      c3_cv_sizes = c3_av_sizes;
      c3_c_loop_ub = c3_av_sizes - 1;
      for (c3_i265 = 0; c3_i265 <= c3_c_loop_ub; c3_i265++) {
        c3_cv_data[c3_i265] = c3_av_data[c3_i265] * c3_bv;
      }

      c3_c_nc1 = c3_nc1;
      c3_j_b = c3_c_nc1;
      c3_k_b = c3_j_b;
      if (1 > c3_k_b) {
        c3_b_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_b_overflow = (c3_k_b > 2147483646);
      }

      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_c_k = 1; c3_c_k <= c3_c_nc1; c3_c_k++) {
        c3_b_k = c3_c_k;
        c3_c_a = c3_b_ck;
        c3_l_b = c3_b_k;
        c3_d_a = c3_c_a;
        c3_m_b = c3_l_b;
        c3_b_c = c3_d_a + c3_m_b;
        c3_c_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_c, 1, *c3_c_sizes, 1, 0)
          - 1] = c3_cv_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
          c3_cv_sizes, 1, 0) - 1];
      }
    }
  }
}

static void c3_d_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes)
{
  int32_T c3_na1;
  int32_T c3_csz[2];
  int32_T c3_cv_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i266;
  real_T c3_cv_data[4];
  int32_T c3_b_loop_ub;
  int32_T c3_i267;
  int32_T c3_av_sizes;
  int32_T c3_nc1;
  int32_T c3_b_nc1;
  real_T c3_a;
  int32_T c3_b_b;
  real_T c3_b_a;
  int32_T c3_c_b;
  int32_T c3_i268;
  int32_T c3_d_b;
  int32_T c3_e_b;
  int32_T c3_ck;
  int32_T c3_b_ck;
  int32_T c3_b_na1;
  int32_T c3_f_b;
  int32_T c3_g_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_h_b;
  int32_T c3_i_b;
  int32_T c3_c;
  real_T c3_av_data[4];
  real_T c3_bv;
  real_T c3_y;
  real_T c3_b_y;
  real_T c3_c_y;
  int32_T c3_c_loop_ub;
  int32_T c3_i269;
  int32_T c3_c_nc1;
  int32_T c3_j_b;
  int32_T c3_k_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  int32_T c3_c_a;
  int32_T c3_l_b;
  int32_T c3_d_a;
  int32_T c3_m_b;
  int32_T c3_b_c;
  c3_na1 = c3_a_sizes;
  c3_csz[0] = c3_a_sizes;
  c3_c_eml_scalar_eg(chartInstance);
  c3_cv_sizes = c3_csz[0];
  c3_loop_ub = c3_csz[0] - 1;
  for (c3_i266 = 0; c3_i266 <= c3_loop_ub; c3_i266++) {
    c3_cv_data[c3_i266] = 0.0;
  }

  *c3_c_sizes = c3_cv_sizes;
  if (*c3_c_sizes == 0) {
  } else {
    c3_csz[0] = c3_na1;
    c3_csz[1] = 1;
    c3_cv_sizes = c3_csz[0];
    c3_b_loop_ub = c3_csz[0] - 1;
    for (c3_i267 = 0; c3_i267 <= c3_b_loop_ub; c3_i267++) {
      c3_cv_data[c3_i267] = 0.0;
    }

    c3_av_sizes = c3_cv_sizes;
    c3_nc1 = *c3_c_sizes;
    c3_b_nc1 = c3_nc1;
    c3_a = (real_T)*c3_c_sizes;
    c3_b_b = c3_nc1;
    c3_b_a = c3_a;
    c3_c_b = c3_b_b;
    c3_i268 = (int32_T)c3_b_a - c3_c_b;
    c3_d_b = c3_i268;
    c3_e_b = c3_d_b;
    if (0 > c3_e_b) {
    } else {
      c3_eml_switch_helper(chartInstance);
    }

    for (c3_ck = 0; c3_ck <= c3_i268; c3_ck += c3_b_nc1) {
      c3_b_ck = c3_ck;
      c3_b_na1 = c3_na1;
      c3_f_b = c3_b_na1;
      c3_g_b = c3_f_b;
      if (1 > c3_g_b) {
        c3_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_overflow = (c3_g_b > 2147483646);
      }

      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_k = 1; c3_k <= c3_b_na1; c3_k++) {
        c3_b_k = c3_k;
        c3_h_b = c3_b_k;
        c3_i_b = c3_h_b;
        c3_c = c3_i_b;
        c3_av_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1, c3_av_sizes, 1, 0)
          - 1] = c3_a_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c, 1, c3_a_sizes,
          1, 0) - 1];
      }

      c3_bv = c3_b;
      c3_y = c3_bv;
      c3_b_y = c3_y;
      c3_c_y = c3_b_y;
      c3_cv_sizes = c3_av_sizes;
      c3_c_loop_ub = c3_av_sizes - 1;
      for (c3_i269 = 0; c3_i269 <= c3_c_loop_ub; c3_i269++) {
        c3_cv_data[c3_i269] = c3_av_data[c3_i269] / c3_c_y;
      }

      c3_c_nc1 = c3_nc1;
      c3_j_b = c3_c_nc1;
      c3_k_b = c3_j_b;
      if (1 > c3_k_b) {
        c3_b_overflow = false;
      } else {
        c3_eml_switch_helper(chartInstance);
        c3_b_overflow = (c3_k_b > 2147483646);
      }

      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_c_k = 1; c3_c_k <= c3_c_nc1; c3_c_k++) {
        c3_b_k = c3_c_k;
        c3_c_a = c3_b_ck;
        c3_l_b = c3_b_k;
        c3_d_a = c3_c_a;
        c3_m_b = c3_l_b;
        c3_b_c = c3_d_a + c3_m_b;
        c3_c_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_c, 1, *c3_c_sizes, 1, 0)
          - 1] = c3_cv_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
          c3_cv_sizes, 1, 0) - 1];
      }
    }
  }
}

static void c3_eml_sort(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x_data[], int32_T c3_x_sizes, real_T c3_y_data[], int32_T
  *c3_y_sizes, int32_T c3_idx_data[], int32_T *c3_idx_sizes)
{
  int32_T c3_dim;
  int32_T c3_b_dim;
  int32_T c3_c_dim;
  int32_T c3_x;
  int32_T c3_b_x;
  boolean_T c3_b7;
  int32_T c3_i270;
  static char_T c3_cv2[53] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'g', 'e', 't', 'd', 'i', 'm', 'a', 'r', 'g', '_', 'd', 'i',
    'm', 'e', 'n', 's', 'i', 'o', 'n', 'M', 'u', 's', 't', 'B', 'e', 'P', 'o',
    's', 'i', 't', 'i', 'v', 'e', 'I', 'n', 't', 'e', 'g', 'e', 'r' };

  char_T c3_u[53];
  const mxArray *c3_y = NULL;
  int32_T c3_i271;
  real_T c3_d2;
  real_T c3_vlen;
  real_T c3_dv45[2];
  int32_T c3_tmp_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i272;
  real_T c3_tmp_data[4];
  int32_T c3_vwork_sizes;
  int32_T c3_iidx_sizes;
  int32_T c3_b_loop_ub;
  int32_T c3_i273;
  int32_T c3_iidx_data[4];
  int32_T c3_d_dim;
  int32_T c3_e_dim;
  int32_T c3_vstride;
  int32_T c3_i274;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  real_T c3_d3;
  real_T c3_a;
  real_T c3_b_a;
  int32_T c3_c;
  int32_T c3_c_a;
  int32_T c3_c_b;
  int32_T c3_d_a;
  int32_T c3_d_b;
  int32_T c3_vspread;
  int32_T c3_f_dim;
  int32_T c3_g_dim;
  int32_T c3_i275;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_i2;
  int32_T c3_i;
  int32_T c3_i1;
  int32_T c3_g_a;
  int32_T c3_e_b;
  int32_T c3_h_a;
  int32_T c3_f_b;
  int32_T c3_b_vstride;
  int32_T c3_g_b;
  int32_T c3_h_b;
  boolean_T c3_b_overflow;
  int32_T c3_j;
  int32_T c3_i_a;
  int32_T c3_j_a;
  int32_T c3_k_a;
  int32_T c3_l_a;
  int32_T c3_ix;
  real_T c3_b_vlen;
  int32_T c3_i276;
  int32_T c3_b_k;
  real_T c3_c_k;
  real_T c3_vwork_data[4];
  int32_T c3_m_a;
  int32_T c3_i_b;
  int32_T c3_n_a;
  int32_T c3_j_b;
  int32_T c3_b_vwork_sizes;
  int32_T c3_c_loop_ub;
  int32_T c3_i277;
  real_T c3_b_vwork_data[4];
  real_T c3_c_vlen;
  int32_T c3_i278;
  int32_T c3_d_k;
  int32_T c3_o_a;
  int32_T c3_k_b;
  int32_T c3_p_a;
  int32_T c3_l_b;
  c3_dim = 2;
  if ((real_T)c3_x_sizes != 1.0) {
    c3_dim = 1;
  }

  c3_b_dim = c3_dim;
  c3_c_dim = c3_b_dim;
  c3_x = c3_c_dim;
  c3_b_x = c3_x;
  c3_b7 = (c3_c_dim == c3_b_x);
  if (c3_b7) {
  } else {
    for (c3_i270 = 0; c3_i270 < 53; c3_i270++) {
      c3_u[c3_i270] = c3_cv2[c3_i270];
    }

    c3_y = NULL;
    sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 53),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c3_y));
  }

  c3_eml_switch_helper(chartInstance);
  c3_i271 = c3_dim;
  if (c3_i271 <= 1) {
    c3_d2 = (real_T)c3_x_sizes;
  } else {
    c3_d2 = 1.0;
  }

  c3_vlen = c3_d2;
  c3_dv45[0] = c3_vlen;
  c3_dv45[1] = 1.0;
  c3_tmp_sizes = (int32_T)c3_dv45[0];
  c3_loop_ub = (int32_T)c3_dv45[0] - 1;
  for (c3_i272 = 0; c3_i272 <= c3_loop_ub; c3_i272++) {
    c3_tmp_data[c3_i272] = 0.0;
  }

  c3_vwork_sizes = c3_tmp_sizes;
  *c3_y_sizes = c3_x_sizes;
  c3_dv45[0] = (real_T)c3_x_sizes;
  c3_dv45[1] = 1.0;
  c3_iidx_sizes = (int32_T)c3_dv45[0];
  c3_b_loop_ub = (int32_T)c3_dv45[0] - 1;
  for (c3_i273 = 0; c3_i273 <= c3_b_loop_ub; c3_i273++) {
    c3_iidx_data[c3_i273] = 0;
  }

  *c3_idx_sizes = c3_iidx_sizes;
  c3_d_dim = c3_dim;
  c3_e_dim = c3_d_dim - 1;
  c3_vstride = 1;
  c3_i274 = c3_e_dim;
  c3_b = c3_i274;
  c3_b_b = c3_b;
  if (1 > c3_b_b) {
    c3_overflow = false;
  } else {
    c3_eml_switch_helper(chartInstance);
    c3_overflow = (c3_b_b > 2147483646);
  }

  if (c3_overflow) {
    c3_check_forloop_overflow_error(chartInstance, true);
  }

  c3_k = 1;
  while (c3_k <= c3_i274) {
    c3_d3 = (real_T)c3_x_sizes;
    c3_vstride *= (int32_T)c3_d3;
    c3_k = 2;
  }

  c3_a = c3_vlen;
  c3_b_a = c3_a;
  c3_c = (int32_T)c3_b_a;
  c3_c_a = c3_c - 1;
  c3_c_b = c3_vstride;
  c3_d_a = c3_c_a;
  c3_d_b = c3_c_b;
  c3_vspread = c3_d_a * c3_d_b;
  c3_f_dim = c3_dim;
  c3_g_dim = c3_f_dim + 1;
  c3_i275 = c3_g_dim;
  c3_e_a = c3_i275;
  c3_f_a = c3_e_a;
  if (c3_f_a > 2) {
  } else {
    c3_eml_switch_helper(chartInstance);
  }

  c3_i2 = 0;
  c3_eml_switch_helper(chartInstance);
  c3_i = 1;
  while (c3_i <= 1) {
    c3_i1 = c3_i2;
    c3_g_a = c3_i2;
    c3_e_b = c3_vspread;
    c3_h_a = c3_g_a;
    c3_f_b = c3_e_b;
    c3_i2 = c3_h_a + c3_f_b;
    c3_b_vstride = c3_vstride;
    c3_g_b = c3_b_vstride;
    c3_h_b = c3_g_b;
    if (1 > c3_h_b) {
      c3_b_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_b_overflow = (c3_h_b > 2147483646);
    }

    if (c3_b_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_j = 1; c3_j <= c3_b_vstride; c3_j++) {
      c3_i_a = c3_i1;
      c3_j_a = c3_i_a + 1;
      c3_i1 = c3_j_a;
      c3_k_a = c3_i2;
      c3_l_a = c3_k_a + 1;
      c3_i2 = c3_l_a;
      c3_ix = c3_i1;
      c3_b_vlen = c3_vlen;
      c3_i276 = (int32_T)c3_b_vlen - 1;
      for (c3_b_k = 0; c3_b_k <= c3_i276; c3_b_k++) {
        c3_c_k = 1.0 + (real_T)c3_b_k;
        c3_vwork_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c3_c_k, 1,
          c3_vwork_sizes, 1, 0) - 1] = c3_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c3_ix, 1, c3_x_sizes, 1, 0) - 1];
        c3_m_a = c3_ix;
        c3_i_b = c3_vstride;
        c3_n_a = c3_m_a;
        c3_j_b = c3_i_b;
        c3_ix = c3_n_a + c3_j_b;
      }

      c3_b_vwork_sizes = c3_vwork_sizes;
      c3_c_loop_ub = c3_vwork_sizes - 1;
      for (c3_i277 = 0; c3_i277 <= c3_c_loop_ub; c3_i277++) {
        c3_b_vwork_data[c3_i277] = c3_vwork_data[c3_i277];
      }

      c3_eml_sort_idx(chartInstance, c3_b_vwork_data, c3_b_vwork_sizes,
                      c3_iidx_data, &c3_iidx_sizes);
      c3_ix = c3_i1;
      c3_c_vlen = c3_vlen;
      c3_i278 = (int32_T)c3_c_vlen - 1;
      for (c3_d_k = 0; c3_d_k <= c3_i278; c3_d_k++) {
        c3_c_k = 1.0 + (real_T)c3_d_k;
        c3_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_ix, 1, *c3_y_sizes, 1, 0) -
          1] = c3_vwork_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c3_iidx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c3_c_k, 1,
          c3_iidx_sizes, 1, 0) - 1], 1, c3_vwork_sizes, 1, 0) - 1];
        c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_ix, 1, *c3_idx_sizes, 1,
          0) - 1] = c3_iidx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c3_c_k,
          1, c3_iidx_sizes, 1, 0) - 1];
        c3_o_a = c3_ix;
        c3_k_b = c3_vstride;
        c3_p_a = c3_o_a;
        c3_l_b = c3_k_b;
        c3_ix = c3_p_a + c3_l_b;
      }
    }

    c3_i = 2;
  }
}

static void c3_eml_sort_idx(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_x_data[], int32_T c3_x_sizes, int32_T c3_idx_data[],
  int32_T *c3_idx_sizes)
{
  int32_T c3_n;
  real_T c3_dv46[2];
  int32_T c3_idx0_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i279;
  int32_T c3_idx0_data[4];
  int32_T c3_a;
  int32_T c3_b_a;
  int32_T c3_np1;
  int32_T c3_b_n;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  int32_T c3_c_n;
  int32_T c3_c_b;
  int32_T c3_d_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  int32_T c3_c_a;
  int32_T c3_d_a;
  int32_T c3_i280;
  int32_T c3_e_b;
  int32_T c3_f_b;
  int32_T c3_d_k;
  int32_T c3_e_a;
  int32_T c3_f_a;
  int32_T c3_c;
  int32_T c3_irow1;
  int32_T c3_irow2;
  real_T c3_g_a;
  real_T c3_g_b;
  real_T c3_h_a;
  real_T c3_h_b;
  boolean_T c3_p;
  real_T c3_x;
  boolean_T c3_i_b;
  boolean_T c3_b8;
  boolean_T c3_b_p;
  int32_T c3_i_a;
  int32_T c3_j_a;
  int32_T c3_b_c;
  int32_T c3_k_a;
  int32_T c3_l_a;
  int32_T c3_c_c;
  int32_T c3_b_loop_ub;
  int32_T c3_i281;
  int32_T c3_i;
  int32_T c3_m_a;
  int32_T c3_n_a;
  int32_T c3_i2;
  int32_T c3_j;
  int32_T c3_j_b;
  int32_T c3_k_b;
  int32_T c3_pEnd;
  int32_T c3_c_p;
  int32_T c3_q;
  int32_T c3_o_a;
  int32_T c3_l_b;
  int32_T c3_p_a;
  int32_T c3_m_b;
  int32_T c3_qEnd;
  int32_T c3_q_a;
  int32_T c3_n_b;
  int32_T c3_r_a;
  int32_T c3_o_b;
  int32_T c3_kEnd;
  int32_T c3_b_irow1;
  int32_T c3_b_irow2;
  real_T c3_s_a;
  real_T c3_p_b;
  real_T c3_t_a;
  real_T c3_q_b;
  boolean_T c3_d_p;
  real_T c3_b_x;
  boolean_T c3_r_b;
  boolean_T c3_b9;
  boolean_T c3_e_p;
  int32_T c3_u_a;
  int32_T c3_v_a;
  int32_T c3_w_a;
  int32_T c3_x_a;
  int32_T c3_y_a;
  int32_T c3_ab_a;
  int32_T c3_bb_a;
  int32_T c3_cb_a;
  int32_T c3_db_a;
  int32_T c3_eb_a;
  int32_T c3_fb_a;
  int32_T c3_gb_a;
  int32_T c3_hb_a;
  int32_T c3_ib_a;
  int32_T c3_jb_a;
  int32_T c3_kb_a;
  int32_T c3_b_kEnd;
  int32_T c3_s_b;
  int32_T c3_t_b;
  boolean_T c3_c_overflow;
  int32_T c3_e_k;
  int32_T c3_lb_a;
  int32_T c3_u_b;
  int32_T c3_mb_a;
  int32_T c3_v_b;
  int32_T c3_d_c;
  int32_T c3_nb_a;
  int32_T c3_w_b;
  int32_T c3_ob_a;
  int32_T c3_x_b;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  c3_n = c3_x_sizes;
  c3_dv46[0] = (real_T)c3_x_sizes;
  c3_dv46[1] = 1.0;
  c3_idx0_sizes = (int32_T)c3_dv46[0];
  c3_loop_ub = (int32_T)c3_dv46[0] - 1;
  for (c3_i279 = 0; c3_i279 <= c3_loop_ub; c3_i279++) {
    c3_idx0_data[c3_i279] = 0;
  }

  *c3_idx_sizes = c3_idx0_sizes;
  c3_a = c3_n;
  c3_b_a = c3_a + 1;
  c3_np1 = c3_b_a;
  if (c3_x_sizes == 0) {
    c3_b_n = c3_n;
    c3_b = c3_b_n;
    c3_b_b = c3_b;
    if (1 > c3_b_b) {
      c3_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_overflow = (c3_b_b > 2147483646);
    }

    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_k = 1; c3_k <= c3_b_n; c3_k++) {
      c3_b_k = c3_k;
      c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1, *c3_idx_sizes, 1, 0)
        - 1] = c3_b_k;
    }
  } else {
    c3_c_n = c3_n;
    c3_c_b = c3_c_n;
    c3_d_b = c3_c_b;
    if (1 > c3_d_b) {
      c3_b_overflow = false;
    } else {
      c3_eml_switch_helper(chartInstance);
      c3_b_overflow = (c3_d_b > 2147483646);
    }

    if (c3_b_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_c_k = 1; c3_c_k <= c3_c_n; c3_c_k++) {
      c3_b_k = c3_c_k;
      c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1, *c3_idx_sizes, 1, 0)
        - 1] = c3_b_k;
    }

    c3_c_a = c3_n;
    c3_d_a = c3_c_a - 1;
    c3_i280 = c3_d_a;
    c3_e_b = c3_i280;
    c3_f_b = c3_e_b;
    if (1 > c3_f_b) {
    } else {
      c3_eml_switch_helper(chartInstance);
    }

    for (c3_d_k = 1; c3_d_k <= c3_i280; c3_d_k += 2) {
      c3_b_k = c3_d_k;
      c3_e_a = c3_b_k;
      c3_f_a = c3_e_a;
      c3_c = c3_f_a;
      c3_irow1 = c3_b_k;
      c3_irow2 = c3_c + 1;
      c3_g_a = c3_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_irow1, 1, c3_x_sizes,
        1, 0) - 1];
      c3_g_b = c3_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_irow2, 1, c3_x_sizes,
        1, 0) - 1];
      c3_h_a = c3_g_a;
      c3_h_b = c3_g_b;
      c3_p = (c3_h_a <= c3_h_b);
      guard2 = false;
      if (c3_p) {
        guard2 = true;
      } else {
        c3_x = c3_g_b;
        c3_i_b = muDoubleScalarIsNaN(c3_x);
        if (c3_i_b) {
          guard2 = true;
        } else {
          c3_b8 = false;
        }
      }

      if (guard2 == true) {
        c3_b8 = true;
      }

      c3_b_p = c3_b8;
      if (c3_b_p) {
      } else {
        c3_i_a = c3_b_k;
        c3_j_a = c3_i_a;
        c3_b_c = c3_j_a;
        c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1, *c3_idx_sizes, 1,
          0) - 1] = c3_b_c + 1;
        c3_k_a = c3_b_k;
        c3_l_a = c3_k_a;
        c3_c_c = c3_l_a;
        c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c_c + 1, 1, *c3_idx_sizes,
          1, 0) - 1] = c3_b_k;
      }
    }

    c3_idx0_sizes = c3_n;
    c3_b_loop_ub = c3_n - 1;
    for (c3_i281 = 0; c3_i281 <= c3_b_loop_ub; c3_i281++) {
      c3_idx0_data[c3_i281] = 1;
    }

    c3_i = 2;
    while (c3_i < c3_n) {
      c3_m_a = c3_i;
      c3_n_a = c3_m_a;
      c3_i2 = c3_n_a << 1;
      c3_j = 1;
      c3_j_b = c3_i;
      c3_k_b = c3_j_b + 1;
      for (c3_pEnd = c3_k_b; c3_pEnd < c3_np1; c3_pEnd = c3_ob_a + c3_x_b) {
        c3_c_p = c3_j;
        c3_q = c3_pEnd;
        c3_o_a = c3_j;
        c3_l_b = c3_i2;
        c3_p_a = c3_o_a;
        c3_m_b = c3_l_b;
        c3_qEnd = c3_p_a + c3_m_b;
        if (c3_qEnd > c3_np1) {
          c3_qEnd = c3_np1;
        }

        c3_b_k = 1;
        c3_q_a = c3_qEnd;
        c3_n_b = c3_j;
        c3_r_a = c3_q_a;
        c3_o_b = c3_n_b;
        c3_kEnd = c3_r_a - c3_o_b;
        while (c3_b_k <= c3_kEnd) {
          c3_b_irow1 = c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c_p, 1,
            *c3_idx_sizes, 1, 0) - 1];
          c3_b_irow2 = c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_q, 1,
            *c3_idx_sizes, 1, 0) - 1];
          c3_s_a = c3_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_irow1, 1,
            c3_x_sizes, 1, 0) - 1];
          c3_p_b = c3_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_irow2, 1,
            c3_x_sizes, 1, 0) - 1];
          c3_t_a = c3_s_a;
          c3_q_b = c3_p_b;
          c3_d_p = (c3_t_a <= c3_q_b);
          guard1 = false;
          if (c3_d_p) {
            guard1 = true;
          } else {
            c3_b_x = c3_p_b;
            c3_r_b = muDoubleScalarIsNaN(c3_b_x);
            if (c3_r_b) {
              guard1 = true;
            } else {
              c3_b9 = false;
            }
          }

          if (guard1 == true) {
            c3_b9 = true;
          }

          c3_e_p = c3_b9;
          if (c3_e_p) {
            c3_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
              c3_idx0_sizes, 1, 0) - 1] =
              c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c_p, 1,
              *c3_idx_sizes, 1, 0) - 1];
            c3_u_a = c3_c_p;
            c3_v_a = c3_u_a + 1;
            c3_c_p = c3_v_a;
            if (c3_c_p == c3_pEnd) {
              while (c3_q < c3_qEnd) {
                c3_w_a = c3_b_k;
                c3_x_a = c3_w_a + 1;
                c3_b_k = c3_x_a;
                c3_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
                  c3_idx0_sizes, 1, 0) - 1] =
                  c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_q, 1,
                  *c3_idx_sizes, 1, 0) - 1];
                c3_y_a = c3_q;
                c3_ab_a = c3_y_a + 1;
                c3_q = c3_ab_a;
              }
            }
          } else {
            c3_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
              c3_idx0_sizes, 1, 0) - 1] =
              c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_q, 1, *c3_idx_sizes,
              1, 0) - 1];
            c3_bb_a = c3_q;
            c3_cb_a = c3_bb_a + 1;
            c3_q = c3_cb_a;
            if (c3_q == c3_qEnd) {
              while (c3_c_p < c3_pEnd) {
                c3_db_a = c3_b_k;
                c3_eb_a = c3_db_a + 1;
                c3_b_k = c3_eb_a;
                c3_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
                  c3_idx0_sizes, 1, 0) - 1] =
                  c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_c_p, 1,
                  *c3_idx_sizes, 1, 0) - 1];
                c3_fb_a = c3_c_p;
                c3_gb_a = c3_fb_a + 1;
                c3_c_p = c3_gb_a;
              }
            }
          }

          c3_hb_a = c3_b_k;
          c3_ib_a = c3_hb_a + 1;
          c3_b_k = c3_ib_a;
        }

        c3_jb_a = c3_j;
        c3_kb_a = c3_jb_a;
        c3_c_p = c3_kb_a;
        c3_b_kEnd = c3_kEnd;
        c3_s_b = c3_b_kEnd;
        c3_t_b = c3_s_b;
        if (1 > c3_t_b) {
          c3_c_overflow = false;
        } else {
          c3_eml_switch_helper(chartInstance);
          c3_c_overflow = (c3_t_b > 2147483646);
        }

        if (c3_c_overflow) {
          c3_check_forloop_overflow_error(chartInstance, true);
        }

        for (c3_e_k = 1; c3_e_k <= c3_b_kEnd; c3_e_k++) {
          c3_b_k = c3_e_k;
          c3_lb_a = c3_c_p - 1;
          c3_u_b = c3_b_k;
          c3_mb_a = c3_lb_a;
          c3_v_b = c3_u_b;
          c3_d_c = c3_mb_a + c3_v_b;
          c3_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_d_c, 1, *c3_idx_sizes,
            1, 0) - 1] = c3_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c3_b_k, 1,
            c3_idx0_sizes, 1, 0) - 1];
        }

        c3_j = c3_qEnd;
        c3_nb_a = c3_j;
        c3_w_b = c3_i;
        c3_ob_a = c3_nb_a;
        c3_x_b = c3_w_b;
      }

      c3_i = c3_i2;
    }
  }
}

static real_T c3_mean(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x[2])
{
  real_T c3_b_y;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_d_x;
  (void)chartInstance;
  c3_b_y = c3_x[0];
  c3_b_y += c3_x[1];
  c3_b_x = c3_b_y;
  c3_c_x = c3_b_x;
  c3_d_x = c3_c_x;
  return c3_d_x / 2.0;
}

static boolean_T c3_all(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  boolean_T c3_x[2])
{
  boolean_T c3_y;
  int32_T c3_k;
  real_T c3_b_k;
  boolean_T exitg1;
  (void)chartInstance;
  c3_y = true;
  c3_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c3_k < 2)) {
    c3_b_k = 1.0 + (real_T)c3_k;
    if ((real_T)c3_x[(int32_T)c3_b_k - 1] == 0.0) {
      c3_y = false;
      exitg1 = true;
    } else {
      c3_k++;
    }
  }

  return c3_y;
}

static real_T c3_mpower(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a)
{
  real_T c3_b_a;
  real_T c3_c_a;
  real_T c3_ak;
  real_T c3_d_a;
  c3_b_a = c3_a;
  c3_c_a = c3_b_a;
  c3_b_eml_scalar_eg(chartInstance);
  c3_ak = c3_c_a;
  c3_d_a = c3_ak;
  c3_b_eml_scalar_eg(chartInstance);
  return c3_d_a * c3_d_a;
}

static void c3_eml_error(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  int32_T c3_i282;
  static char_T c3_cv3[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[30];
  const mxArray *c3_y = NULL;
  int32_T c3_i283;
  static char_T c3_cv4[4] = { 's', 'q', 'r', 't' };

  char_T c3_b_u[4];
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  for (c3_i282 = 0; c3_i282 < 30; c3_i282++) {
    c3_u[c3_i282] = c3_cv3[c3_i282];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c3_i283 = 0; c3_i283 < 4; c3_i283++) {
    c3_b_u[c3_i283] = c3_cv4[c3_i283];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c3_y, 14, c3_b_y));
}

static const mxArray *c3_y_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static int32_T c3_bb_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i284;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i284, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i284;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_cb_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_AutoFollow_Simulation, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_db_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_AutoFollow_Simulation), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_AutoFollow_Simulation);
  return c3_y;
}

static uint8_T c3_db_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_eml_xgemm(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_A[9], real_T c3_B[3], real_T c3_C[3])
{
  int32_T c3_i285;
  int32_T c3_i286;
  int32_T c3_i287;
  (void)chartInstance;
  for (c3_i285 = 0; c3_i285 < 3; c3_i285++) {
    c3_C[c3_i285] = 0.0;
    c3_i286 = 0;
    for (c3_i287 = 0; c3_i287 < 3; c3_i287++) {
      c3_C[c3_i285] += c3_A[c3_i286 + c3_i285] * c3_B[c3_i287];
      c3_i286 += 3;
    }
  }
}

static void init_dsm_address_info(SFc3_AutoFollow_SimulationInstanceStruct
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

void sf_c3_AutoFollow_Simulation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4029390308U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2579763474U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(592107315U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3913247837U);
}

mxArray *sf_c3_AutoFollow_Simulation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("u67pBcag2DP2kgykYX4eRG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(361);
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
      pr[0] = (double)(361);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_AutoFollow_Simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_AutoFollow_Simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c3_AutoFollow_Simulation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x6'type','srcId','name','auxInfo'{{M[1],M[5],T\"r\",},{M[1],M[8],T\"theta\",},{M[4],M[0],T\"prev_r\",S'l','i','p'{{M1x2[74 80],M[0],}}},{M[4],M[0],T\"prev_t\",S'l','i','p'{{M1x2[51 57],M[0],}}},{M[4],M[0],T\"prev_theta\",S'l','i','p'{{M1x2[97 107],M[0],}}},{M[8],M[0],T\"is_active_c3_AutoFollow_Simulation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 6, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_AutoFollow_Simulation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _AutoFollow_SimulationMachineNumber_,
           3,
           1,
           1,
           0,
           5,
           0,
           0,
           0,
           0,
           8,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"xq");
          _SFD_SET_DATA_PROPS(1,2,0,1,"r");
          _SFD_SET_DATA_PROPS(2,1,1,0,"yq");
          _SFD_SET_DATA_PROPS(3,2,0,1,"theta");
          _SFD_SET_DATA_PROPS(4,1,1,0,"t");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1344);
        _SFD_CV_INIT_EML_IF(0,1,0,114,132,-1,160);
        _SFD_CV_INIT_EML_IF(0,1,1,257,311,1280,1340);

        {
          static int condStart[] = { 261, 280 };

          static int condEnd[] = { 276, 310 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,261,310,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(0,1,4,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"se2",1148,-1,1631);
        _SFD_CV_INIT_SCRIPT_IF(0,0,1179,1196,1256,1534);
        _SFD_CV_INIT_SCRIPT_IF(0,1,1256,1277,1405,1534);
        _SFD_CV_INIT_SCRIPT_IF(0,2,1322,1335,1364,1400);
        _SFD_CV_INIT_SCRIPT_IF(0,3,1448,1461,1490,1526);
        _SFD_CV_INIT_SCRIPT(1,1,2,0,0,0,1,0,2,1);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"laserScanner",301,-1,2323);
        _SFD_CV_INIT_SCRIPT_IF(1,0,1556,1583,1647,2257);
        _SFD_CV_INIT_SCRIPT_IF(1,1,1914,1931,-1,1969);
        _SFD_CV_INIT_SCRIPT_FOR(1,0,707,741,2318);

        {
          static int condStart[] = { 1560, 1573 };

          static int condEnd[] = { 1569, 1582 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(1,0,1560,1582,2,0,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"XYtoIJ",0,-1,222);
        _SFD_CV_INIT_SCRIPT(3,1,4,0,0,0,1,0,6,3);
        _SFD_CV_INIT_SCRIPT_FCN(3,0,"clipLine",0,-1,3199);
        _SFD_CV_INIT_SCRIPT_IF(3,0,1380,1408,1445,1477);
        _SFD_CV_INIT_SCRIPT_IF(3,1,1445,1477,1512,1573);
        _SFD_CV_INIT_SCRIPT_IF(3,2,1512,1535,-1,1535);
        _SFD_CV_INIT_SCRIPT_IF(3,3,3144,3160,-1,3194);
        _SFD_CV_INIT_SCRIPT_FOR(3,0,1666,1683,3198);

        {
          static int condStart[] = { 1383, 1398 };

          static int condEnd[] = { 1394, 1408 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(3,0,1383,1408,2,0,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1452, 1467 };

          static int condEnd[] = { 1463, 1477 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(3,1,1452,1477,2,2,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 3149, 3156 };

          static int condEnd[] = { 3152, 3159 };

          static int pfixExpr[] = { 0, 1, -3, -1 };

          _SFD_CV_INIT_SCRIPT_MCDC(3,2,3147,3160,2,4,&(condStart[0]),&(condEnd[0]),
            4,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(4,1,5,0,0,0,0,0,3,2);
        _SFD_CV_INIT_SCRIPT_FCN(4,0,"intersectLines",0,-1,3089);
        _SFD_CV_INIT_SCRIPT_IF(4,0,1347,1368,-1,1395);
        _SFD_CV_INIT_SCRIPT_IF(4,1,1483,1508,-1,1635);
        _SFD_CV_INIT_SCRIPT_IF(4,2,2238,2249,2423,2463);
        _SFD_CV_INIT_SCRIPT_IF(4,3,2423,2432,-1,2463);
        _SFD_CV_INIT_SCRIPT_IF(4,4,2580,2589,-1,2620);

        {
          static int condStart[] = { 1351 };

          static int condEnd[] = { 1368 };

          static int pfixExpr[] = { 0, -1 };

          _SFD_CV_INIT_SCRIPT_MCDC(4,0,1350,1368,1,0,&(condStart[0]),&(condEnd[0]),
            2,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1486, 1498 };

          static int condEnd[] = { 1494, 1508 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(4,1,1486,1508,2,1,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(5,1,2,0,0,0,0,0,3,1);
        _SFD_CV_INIT_SCRIPT_FCN(5,0,"linePosition",0,-1,2949);
        _SFD_CV_INIT_SCRIPT_IF(5,0,1559,1635,2189,2497);
        _SFD_CV_INIT_SCRIPT_IF(5,1,1820,1831,-1,1962);

        {
          static int condStart[] = { 1563, 1584, 1607 };

          static int condEnd[] = { 1580, 1603, 1635 };

          static int pfixExpr[] = { 0, -1, 1, -3, 2, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(5,0,1562,1635,3,0,&(condStart[0]),&(condEnd[0]),
            6,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(6,1,11,0,0,0,0,2,8,2);
        _SFD_CV_INIT_SCRIPT_FCN(6,0,"laserRange",1033,-1,2550);
        _SFD_CV_INIT_SCRIPT_IF(6,0,1175,1185,1226,1275);
        _SFD_CV_INIT_SCRIPT_IF(6,1,1293,1303,1344,1393);
        _SFD_CV_INIT_SCRIPT_IF(6,2,1412,1422,1965,2546);
        _SFD_CV_INIT_SCRIPT_IF(6,3,1543,1580,1652,1721);
        _SFD_CV_INIT_SCRIPT_IF(6,4,1652,1669,1730,1811);
        _SFD_CV_INIT_SCRIPT_IF(6,5,1730,1754,1820,1950);
        _SFD_CV_INIT_SCRIPT_IF(6,6,1820,1829,1881,1950);
        _SFD_CV_INIT_SCRIPT_IF(6,7,2090,2127,2199,2268);
        _SFD_CV_INIT_SCRIPT_IF(6,8,2199,2216,2277,2389);
        _SFD_CV_INIT_SCRIPT_IF(6,9,2277,2301,2398,2528);
        _SFD_CV_INIT_SCRIPT_IF(6,10,2398,2407,2459,2528);
        _SFD_CV_INIT_SCRIPT_WHILE(6,0,1482,1490,1960);
        _SFD_CV_INIT_SCRIPT_WHILE(6,1,2029,2037,2538);

        {
          static int condStart[] = { 1547, 1556, 1565, 1574 };

          static int condEnd[] = { 1552, 1561, 1570, 1579 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2, 3, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(6,0,1547,1579,4,0,&(condStart[0]),&(condEnd[0]),
            7,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2094, 2103, 2112, 2121 };

          static int condEnd[] = { 2099, 2108, 2117, 2126 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2, 3, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(6,1,2094,2126,4,4,&(condStart[0]),&(condEnd[0]),
            7,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(7,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(7,0,"IJtoXY",0,-1,175);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_e_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 361;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_d_sf_marshallOut,(MexInFcnForType)
            c3_d_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_e_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 361;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_d_sf_marshallOut,(MexInFcnForType)
            c3_d_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_e_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c3_xq;
          real_T *c3_yq;
          real_T *c3_t;
          real_T (*c3_r)[361];
          real_T (*c3_theta)[361];
          c3_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c3_theta = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 2);
          c3_yq = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c3_r = (real_T (*)[361])ssGetOutputPortSignal(chartInstance->S, 1);
          c3_xq = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_xq);
          _SFD_SET_DATA_VALUE_PTR(1U, *c3_r);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_yq);
          _SFD_SET_DATA_VALUE_PTR(3U, *c3_theta);
          _SFD_SET_DATA_VALUE_PTR(4U, c3_t);
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
  return "VXUkA6vcKhFDyZYfTRNdZG";
}

static void sf_opaque_initialize_c3_AutoFollow_Simulation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_AutoFollow_Simulation
    ((SFc3_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
  initialize_c3_AutoFollow_Simulation((SFc3_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_AutoFollow_Simulation(void *chartInstanceVar)
{
  enable_c3_AutoFollow_Simulation((SFc3_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_AutoFollow_Simulation(void *chartInstanceVar)
{
  disable_c3_AutoFollow_Simulation((SFc3_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_AutoFollow_Simulation(void *chartInstanceVar)
{
  sf_gateway_c3_AutoFollow_Simulation((SFc3_AutoFollow_SimulationInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_AutoFollow_Simulation
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_AutoFollow_Simulation
    ((SFc3_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_AutoFollow_Simulation();/* state var info */
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

extern void sf_internal_set_sim_state_c3_AutoFollow_Simulation(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c3_AutoFollow_Simulation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_AutoFollow_Simulation
    ((SFc3_AutoFollow_SimulationInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_AutoFollow_Simulation(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c3_AutoFollow_Simulation(S);
}

static void sf_opaque_set_sim_state_c3_AutoFollow_Simulation(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c3_AutoFollow_Simulation(S, st);
}

static void sf_opaque_terminate_c3_AutoFollow_Simulation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_AutoFollow_SimulationInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_AutoFollow_Simulation_optimization_info();
    }

    finalize_c3_AutoFollow_Simulation((SFc3_AutoFollow_SimulationInstanceStruct*)
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
  initSimStructsc3_AutoFollow_Simulation
    ((SFc3_AutoFollow_SimulationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_AutoFollow_Simulation(SimStruct *S)
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
    initialize_params_c3_AutoFollow_Simulation
      ((SFc3_AutoFollow_SimulationInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_AutoFollow_Simulation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2841112238U));
  ssSetChecksum1(S,(1027083490U));
  ssSetChecksum2(S,(625915600U));
  ssSetChecksum3(S,(2587841982U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_AutoFollow_Simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_AutoFollow_Simulation(SimStruct *S)
{
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)utMalloc(sizeof
    (SFc3_AutoFollow_SimulationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_AutoFollow_SimulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_AutoFollow_Simulation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_AutoFollow_Simulation;
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

void c3_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_AutoFollow_Simulation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_AutoFollow_Simulation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_AutoFollow_Simulation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
