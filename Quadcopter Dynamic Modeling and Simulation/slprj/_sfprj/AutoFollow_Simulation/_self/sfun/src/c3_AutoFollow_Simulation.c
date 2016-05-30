/* Include files */

#include "AutoFollow_Simulation_sfun.h"
#include "c3_AutoFollow_Simulation.h"
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

static const char * c3_j_debug_family_names[4] = { "nargin", "nargout", "deg",
  "rad" };

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
static void mdl_start_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_chartstep_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void initSimStructsc3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_se2(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c3_a[3], real_T c3_b_t[9]);
static void c3_laserScanner(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_Tl[9], real_T c3_c_map[10000], real_T c3_p[722]);
static void c3_XYtoIJ(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x, real_T c3_y, real_T c3_Xmax, real_T c3_Ymax,
                      real_T c3_R, real_T c3_C, real_T *c3_i, real_T *c3_j);
static void c3_clipLine(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_line[4], real_T c3_edge[4]);
static void c3_intersectLines(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_line1[4], real_T c3_line2[4], real_T c3_point[2]);
static void c3_linePosition(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_point_data[], int32_T c3_point_sizes[2], real_T
  c3_line[4], real_T c3_pos_data[], int32_T *c3_pos_sizes);
static void c3_laserRange(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_p1[2], real_T c3_p2[2], real_T c3_c_map[10000],
  real_T c3_p_data[], int32_T c3_p_sizes[2]);
static real_T c3_deg2rad(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_deg);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static const mxArray *c3_emlrt_marshallOut
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, const char * c3_u);
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
  boolean_T *c3_svPtr, real_T c3_y[361]);
static void c3_e_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr, real_T c3_y[361]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_f_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_t, const char_T *c3_identifier,
  boolean_T *c3_svPtr);
static real_T c3_g_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_h_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_theta, const char_T *c3_identifier, real_T
  c3_y[361]);
static void c3_i_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361]);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_j_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_k_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[722]);
static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_l_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[9]);
static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_m_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_n_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4]);
static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_k_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_o_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[2]);
static void c3_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_l_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_p_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4]);
static void c3_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_m_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_q_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_n_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static boolean_T c3_r_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_o_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_p_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T *c3_inData_sizes);
static void c3_s_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T *c3_y_sizes);
static void c3_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  *c3_outData_sizes);
static const mxArray *c3_q_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_t_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_r_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_u_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8]);
static void c3_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_s_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_v_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_t_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2]);
static void c3_w_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2]);
static void c3_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2]);
static const mxArray *c3_u_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_x_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static real_T c3_cos(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c3_x);
static real_T c3_sin(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c3_x);
static void c3_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_b_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_c_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_dimagree(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static real_T c3_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a, real_T c3_b);
static void c3_d_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_e_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static real_T c3_abs(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c3_x);
static boolean_T c3_all(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  boolean_T c3_x);
static void c3_indexShapeCheck(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_rdivide(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x, real_T c3_y_data[], int32_T c3_y_sizes[2], real_T c3_z_data[],
  int32_T c3_z_sizes[2]);
static void c3_bsxfun_compatible(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_no_dynamic_expansion(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void c3_f_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
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
static void c3_sort(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                    real_T c3_x_data[], int32_T c3_x_sizes, real_T c3_b_x_data[],
                    int32_T *c3_b_x_sizes, int32_T c3_idx_data[], int32_T
                    *c3_idx_sizes);
static int32_T c3_nonSingletonDim(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_x_data[], int32_T c3_x_sizes);
static void c3_sortIdx(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x_data[], int32_T c3_x_sizes, int32_T c3_idx_data[], int32_T
  *c3_idx_sizes, real_T c3_b_x_data[], int32_T *c3_b_x_sizes);
static void c3_merge_block(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, int32_T c3_idx_data[], int32_T c3_idx_sizes, real_T c3_x_data[],
  int32_T c3_x_sizes, int32_T c3_offset, int32_T c3_n, int32_T c3_preSortLevel,
  int32_T c3_iwork_data[], int32_T c3_iwork_sizes, real_T c3_xwork_data[],
  int32_T c3_xwork_sizes, int32_T c3_b_idx_data[], int32_T *c3_b_idx_sizes,
  real_T c3_b_x_data[], int32_T *c3_b_x_sizes, int32_T c3_b_iwork_data[],
  int32_T *c3_b_iwork_sizes, real_T c3_b_xwork_data[], int32_T *c3_b_xwork_sizes);
static void c3_merge(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     int32_T c3_idx_data[], int32_T c3_idx_sizes, real_T
                     c3_x_data[], int32_T c3_x_sizes, int32_T c3_offset, int32_T
                     c3_np, int32_T c3_nq, int32_T c3_iwork_data[], int32_T
                     c3_iwork_sizes, real_T c3_xwork_data[], int32_T
                     c3_xwork_sizes, int32_T c3_b_idx_data[], int32_T
                     *c3_b_idx_sizes, real_T c3_b_x_data[], int32_T
                     *c3_b_x_sizes, int32_T c3_b_iwork_data[], int32_T
                     *c3_b_iwork_sizes, real_T c3_b_xwork_data[], int32_T
                     *c3_b_xwork_sizes);
static real_T c3_mean(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x[2]);
static boolean_T c3_b_all(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, boolean_T c3_x[2]);
static real_T c3_mpower(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a);
static void c3_error(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static void c3_b_error(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance);
static const mxArray *c3_v_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_y_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_ab_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_AutoFollow_Simulation, const
  char_T *c3_identifier);
static uint8_T c3_bb_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_cos(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c3_x);
static void c3_b_sin(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c3_x);
static void c3_b_sort(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x_data[], int32_T *c3_x_sizes, int32_T
                      c3_idx_data[], int32_T *c3_idx_sizes);
static void c3_b_sortIdx(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x_data[], int32_T *c3_x_sizes, int32_T c3_idx_data[], int32_T
  *c3_idx_sizes);
static void c3_b_merge_block(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, int32_T c3_idx_data[], int32_T *c3_idx_sizes, real_T
  c3_x_data[], int32_T *c3_x_sizes, int32_T c3_offset, int32_T c3_n, int32_T
  c3_preSortLevel, int32_T c3_iwork_data[], int32_T *c3_iwork_sizes, real_T
  c3_xwork_data[], int32_T *c3_xwork_sizes);
static void c3_b_merge(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  int32_T c3_idx_data[], int32_T *c3_idx_sizes, real_T c3_x_data[], int32_T
  *c3_x_sizes, int32_T c3_offset, int32_T c3_np, int32_T c3_nq, int32_T
  c3_iwork_data[], int32_T *c3_iwork_sizes, real_T c3_xwork_data[], int32_T
  *c3_xwork_sizes);
static void init_dsm_address_info(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc3_AutoFollow_Simulation(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

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
  const mxArray *c3_b_y = NULL;
  const mxArray *c3_c_y = NULL;
  const mxArray *c3_d_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_e_y = NULL;
  const mxArray *c3_f_y = NULL;
  uint8_T c3_b_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_g_y = NULL;
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(6, 1), false);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", *chartInstance->c3_r, 0, 0U, 1U, 0U,
    1, 361), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", *chartInstance->c3_theta, 0, 0U, 1U,
    0U, 1, 361), false);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_d_y = NULL;
  if (!chartInstance->c3_prev_theta_not_empty) {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_d_y, sf_mex_create("y", chartInstance->c3_prev_r, 0, 0U,
      1U, 0U, 1, 361), false);
  }

  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_hoistedGlobal = chartInstance->c3_prev_t;
  c3_u = c3_hoistedGlobal;
  c3_e_y = NULL;
  if (!chartInstance->c3_prev_t_not_empty) {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c3_y, 3, c3_e_y);
  c3_f_y = NULL;
  if (!chartInstance->c3_prev_theta_not_empty) {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", chartInstance->c3_prev_theta, 0,
      0U, 1U, 0U, 1, 361), false);
  }

  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_b_hoistedGlobal = chartInstance->c3_is_active_c3_AutoFollow_Simulation;
  c3_b_u = c3_b_hoistedGlobal;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 5, c3_g_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[361];
  int32_T c3_i0;
  real_T c3_dv1[361];
  int32_T c3_i1;
  real_T c3_dv2[361];
  int32_T c3_i2;
  real_T c3_dv3[361];
  int32_T c3_i3;
  chartInstance->c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("r", c3_u, 0)),
                        "r", c3_dv0);
  for (c3_i0 = 0; c3_i0 < 361; c3_i0++) {
    (*chartInstance->c3_r)[c3_i0] = c3_dv0[c3_i0];
  }

  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("theta", c3_u,
    1)), "theta", c3_dv1);
  for (c3_i1 = 0; c3_i1 < 361; c3_i1++) {
    (*chartInstance->c3_theta)[c3_i1] = c3_dv1[c3_i1];
  }

  c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("prev_r", c3_u,
    2)), "prev_r", &chartInstance->c3_prev_r_not_empty, c3_dv2);
  for (c3_i2 = 0; c3_i2 < 361; c3_i2++) {
    chartInstance->c3_prev_r[c3_i2] = c3_dv2[c3_i2];
  }

  chartInstance->c3_prev_t = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("prev_t", c3_u, 3)), "prev_t",
    &chartInstance->c3_prev_t_not_empty);
  c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("prev_theta",
    c3_u, 4)), "prev_theta", &chartInstance->c3_prev_theta_not_empty, c3_dv3);
  for (c3_i3 = 0; c3_i3 < 361; c3_i3++) {
    chartInstance->c3_prev_theta[c3_i3] = c3_dv3[c3_i3];
  }

  chartInstance->c3_is_active_c3_AutoFollow_Simulation = c3_ab_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(
       "is_active_c3_AutoFollow_Simulation", c3_u, 5)),
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
  int32_T c3_i4;
  int32_T c3_i5;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_t, 2U, 1U, 0U,
                        chartInstance->c3_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_yq, 1U, 1U, 0U,
                        chartInstance->c3_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c3_xq, 0U, 1U, 0U,
                        chartInstance->c3_sfEvent, false);
  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_AutoFollow_Simulation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_AutoFollow_SimulationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c3_i4 = 0; c3_i4 < 361; c3_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c3_r)[c3_i4], 3U, 1U, 0U,
                          chartInstance->c3_sfEvent, false);
  }

  for (c3_i5 = 0; c3_i5 < 361; c3_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c3_theta)[c3_i5], 4U, 1U, 0U,
                          chartInstance->c3_sfEvent, false);
  }
}

static void mdl_start_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_chartstep_c3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_c_hoistedGlobal;
  real_T c3_b_xq;
  real_T c3_b_yq;
  real_T c3_b_t;
  uint32_T c3_debug_family_var_map[19];
  real_T c3_lidarUpdateTime;
  real_T c3_Xmax;
  real_T c3_Ymax;
  real_T c3_angleSpan;
  real_T c3_angleStep;
  real_T c3_rangeMax;
  real_T c3_Tl[9];
  real_T c3_p[722];
  real_T c3_nargin = 3.0;
  real_T c3_nargout = 2.0;
  real_T c3_b_r[361];
  real_T c3_b_theta[361];
  int32_T c3_i6;
  int32_T c3_i7;
  int32_T c3_i8;
  int32_T c3_i9;
  int32_T c3_i10;
  real_T c3_c_xq[3];
  real_T c3_dv4[9];
  int32_T c3_i11;
  int32_T c3_i12;
  int32_T c3_i13;
  real_T c3_b_Tl[9];
  real_T c3_c_map[10000];
  real_T c3_dv5[722];
  int32_T c3_i14;
  int32_T c3_i15;
  real_T c3_d0;
  int32_T c3_i16;
  int32_T c3_i17;
  int32_T c3_i18;
  boolean_T guard1 = false;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *chartInstance->c3_xq;
  c3_b_hoistedGlobal = *chartInstance->c3_yq;
  c3_c_hoistedGlobal = *chartInstance->c3_t;
  c3_b_xq = c3_hoistedGlobal;
  c3_b_yq = c3_b_hoistedGlobal;
  c3_b_t = c3_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 19U, 20U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_lidarUpdateTime, 0U,
    c3_d_sf_marshallOut, c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Xmax, 1U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Ymax, 2U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c3_map, MAX_uint32_T,
    c3_h_sf_marshallOut, c3_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleSpan, 4U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleStep, 5U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_rangeMax, 6U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Tl, 7U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p, 8U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c3_b_map, MAX_uint32_T,
    c3_e_sf_marshallOut, c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 9U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 10U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_xq, 11U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_yq, 12U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_b_t, 13U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_r, 14U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_theta, 15U, c3_c_sf_marshallOut,
    c3_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c3_prev_t, 16U,
    c3_b_sf_marshallOut, c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c3_prev_r, 17U,
    c3_sf_marshallOut, c3_sf_marshallIn);
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
  } else if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 0, c3_b_t -
               chartInstance->c3_prev_t, 0.1, -1, 4U, c3_b_t -
               chartInstance->c3_prev_t > 0.1))) {
    guard1 = true;
  } else {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 1, false);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 37);
    for (c3_i6 = 0; c3_i6 < 361; c3_i6++) {
      c3_b_r[c3_i6] = chartInstance->c3_prev_r[c3_i6];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 38);
    for (c3_i7 = 0; c3_i7 < 361; c3_i7++) {
      c3_b_theta[c3_i7] = chartInstance->c3_prev_theta[c3_i7];
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
    c3_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                        (sfGlobalDebugInstanceStruct, "load", 1U, 1U, 14,
                         c3_emlrt_marshallOut(chartInstance,
      "./Lidar/ObstacleMap/map.mat")), "load", &chartInstance->c3_r0);
    chartInstance->c3_map = chartInstance->c3_r0;
    _SFD_SYMBOL_SWITCH(3U, 3U);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 16);
    for (c3_i8 = 0; c3_i8 < 10000; c3_i8++) {
      chartInstance->c3_b_map[c3_i8] = chartInstance->c3_map.map[c3_i8];
    }

    _SFD_SYMBOL_SWITCH(3U, 9U);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
    c3_angleSpan = 6.2831853071795862;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
    c3_angleStep = 0.017453292519943295;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 20);
    c3_rangeMax = 50.0;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
    c3_c_xq[0] = c3_b_xq * 3.28;
    c3_c_xq[1] = c3_b_yq * 3.28;
    c3_c_xq[2] = 0.0;
    c3_se2(chartInstance, c3_c_xq, c3_dv4);
    for (c3_i11 = 0; c3_i11 < 9; c3_i11++) {
      c3_Tl[c3_i11] = c3_dv4[c3_i11];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 27);
    for (c3_i12 = 0; c3_i12 < 9; c3_i12++) {
      c3_b_Tl[c3_i12] = c3_Tl[c3_i12];
    }

    for (c3_i13 = 0; c3_i13 < 10000; c3_i13++) {
      c3_c_map[c3_i13] = chartInstance->c3_b_map[c3_i13];
    }

    c3_laserScanner(chartInstance, c3_b_Tl, c3_c_map, c3_dv5);
    for (c3_i14 = 0; c3_i14 < 722; c3_i14++) {
      c3_p[c3_i14] = c3_dv5[c3_i14];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 32);
    for (c3_i15 = 0; c3_i15 < 361; c3_i15++) {
      c3_b_r[c3_i15] = c3_p[c3_i15 + 361];
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 32);
    c3_d0 = c3_deg2rad(chartInstance, 180.0);
    for (c3_i16 = 0; c3_i16 < 361; c3_i16++) {
      c3_b_theta[c3_i16] = c3_p[c3_i16] + c3_d0;
    }

    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 33);
    for (c3_i17 = 0; c3_i17 < 361; c3_i17++) {
      chartInstance->c3_prev_r[c3_i17] = c3_b_r[c3_i17];
    }

    chartInstance->c3_prev_r_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 34);
    for (c3_i18 = 0; c3_i18 < 361; c3_i18++) {
      chartInstance->c3_prev_theta[c3_i18] = c3_b_theta[c3_i18];
    }

    chartInstance->c3_prev_theta_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 35);
    chartInstance->c3_prev_t = c3_b_t;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -38);
  _SFD_SYMBOL_SCOPE_POP();
  for (c3_i9 = 0; c3_i9 < 361; c3_i9++) {
    (*chartInstance->c3_r)[c3_i9] = c3_b_r[c3_i9];
  }

  for (c3_i10 = 0; c3_i10 < 361; c3_i10++) {
    (*chartInstance->c3_theta)[c3_i10] = c3_b_theta[c3_i10];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_AutoFollow_Simulation
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_se2(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                   real_T c3_a[3], real_T c3_b_t[9])
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
  int32_T c3_i19;
  static real_T c3_dv6[4] = { 1.0, 0.0, -0.0, 1.0 };

  int32_T c3_i20;
  int32_T c3_i21;
  int32_T c3_i22;
  int32_T c3_i23;
  int32_T c3_i24;
  int32_T c3_i25;
  static real_T c3_dv7[3] = { 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_b_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x, 0U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y, 1U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_th, 2U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_cth, 3U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_sth, 4U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_R, 5U, c3_j_sf_marshallOut,
    c3_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 6U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 7U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_a, 8U, c3_i_sf_marshallOut,
    c3_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_b_t, 9U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 36);
  CV_RELATIONAL_EVAL(14U, 0U, 0, 3.0, 3.0, -1, 0U, 1);
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
  for (c3_i19 = 0; c3_i19 < 4; c3_i19++) {
    c3_R[c3_i19] = c3_dv6[c3_i19];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, 62);
  c3_i20 = 0;
  c3_i21 = 0;
  for (c3_i22 = 0; c3_i22 < 2; c3_i22++) {
    for (c3_i23 = 0; c3_i23 < 2; c3_i23++) {
      c3_b_t[c3_i23 + c3_i20] = c3_R[c3_i23 + c3_i21];
    }

    c3_i20 += 3;
    c3_i21 += 2;
  }

  c3_b_t[6] = c3_x;
  c3_b_t[7] = c3_y;
  c3_i24 = 0;
  for (c3_i25 = 0; c3_i25 < 3; c3_i25++) {
    c3_b_t[c3_i24 + 2] = c3_dv7[c3_i25];
    c3_i24 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c3_sfEvent, -62);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_laserScanner(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_Tl[9], real_T c3_c_map[10000], real_T c3_p[722])
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
  real_T c3_Pxel_data[2];
  int32_T c3_Pxel_sizes[2];
  real_T c3_Io;
  real_T c3_Jo;
  real_T c3_xTarget;
  real_T c3_yTarget;
  real_T c3_b_r;
  real_T c3_angleSpan;
  real_T c3_angleStep;
  real_T c3_rangeMax;
  real_T c3_Xmax;
  real_T c3_Ymax;
  real_T c3_nargin = 7.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i26;
  int32_T c3_i27;
  int32_T c3_i28;
  real_T c3_b_a[9];
  int32_T c3_i29;
  int32_T c3_i30;
  int32_T c3_i31;
  real_T c3_b_C[3];
  int32_T c3_i32;
  int32_T c3_i33;
  int32_T c3_i34;
  real_T c3_b_I1;
  real_T c3_b_J1;
  static real_T c3_b[3] = { 0.0, 0.0, 1.0 };

  int32_T c3_c_a;
  real_T c3_d1;
  real_T c3_d2;
  int32_T c3_i35;
  real_T c3_b_Xl[3];
  int32_T c3_i36;
  int32_T c3_i37;
  real_T c3_b_b[3];
  int32_T c3_i38;
  int32_T c3_i39;
  int32_T c3_i40;
  int32_T c3_i41;
  int32_T c3_i42;
  int32_T c3_i43;
  real_T c3_b_x1[4];
  real_T c3_dv8[4];
  int32_T c3_i44;
  real_T c3_b_I2;
  real_T c3_b_J2;
  real_T c3_c_I1[2];
  real_T c3_c_I2[2];
  int32_T c3_i45;
  real_T c3_d_map[10000];
  real_T c3_tmp_data[2];
  int32_T c3_tmp_sizes[2];
  int32_T c3_Pxel;
  int32_T c3_b_Pxel;
  int32_T c3_loop_ub;
  int32_T c3_i46;
  real_T c3_x;
  boolean_T c3_c_b;
  real_T c3_b_x;
  boolean_T c3_d_b;
  real_T c3_b_i;
  real_T c3_j;
  real_T c3_b_Xmax;
  real_T c3_b_Ymax;
  real_T c3_b_R;
  real_T c3_c_C;
  uint32_T c3_b_debug_family_var_map[10];
  real_T c3_b_nargin = 6.0;
  real_T c3_b_nargout = 2.0;
  real_T c3_b_xTarget;
  real_T c3_b_yTarget;
  real_T c3_A;
  real_T c3_c_x;
  real_T c3_d_x;
  real_T c3_y;
  real_T c3_b_A;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_h_x;
  boolean_T c3_b0;
  boolean_T c3_b_p;
  real_T c3_i_x;
  real_T c3_j_x;
  boolean_T guard1 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 34U, 34U, c3_i_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N, 0U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_R, 1U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_C, 2U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_i, 3U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_P1, 4U, c3_u_sf_marshallOut,
    c3_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x1, 5U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y1, 6U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_I1, 7U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_J1, 8U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_a, 9U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Xl, 10U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Yl, 11U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_P2, 12U, c3_u_sf_marshallOut,
    c3_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x2, 13U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y2, 14U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_edge, 15U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_I2, 16U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_J2, 17U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_Pxel_data, (const int32_T *)
    &c3_Pxel_sizes, NULL, 0, 18, (void *)c3_t_sf_marshallOut, (void *)
    c3_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Io, 19U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Jo, 20U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xTarget, 21U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_yTarget, 22U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_r, 23U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleSpan, 24U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_angleStep, 25U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_rangeMax, 26U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Xmax, 27U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_Ymax, 28U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 29U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 30U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_Tl, 31U, c3_g_sf_marshallOut,
    c3_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_c_map, 32U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p, 33U, c3_f_sf_marshallOut,
    c3_f_sf_marshallIn);
  c3_Ymax = 25.0;
  c3_Xmax = 25.0;
  c3_rangeMax = 50.0;
  c3_angleStep = 0.017453292519943295;
  c3_angleSpan = 6.2831853071795862;
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 6);
  c3_N = 361.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 7);
  for (c3_i26 = 0; c3_i26 < 722; c3_i26++) {
    c3_p[c3_i26] = 0.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 8);
  c3_R = 100.0;
  c3_C = 100.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 9);
  c3_i = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 11);
  for (c3_i27 = 0; c3_i27 < 9; c3_i27++) {
    c3_b_a[c3_i27] = c3_Tl[c3_i27];
  }

  for (c3_i28 = 0; c3_i28 < 3; c3_i28++) {
    c3_P1[c3_i28] = 0.0;
  }

  for (c3_i29 = 0; c3_i29 < 3; c3_i29++) {
    c3_P1[c3_i29] = 0.0;
  }

  for (c3_i30 = 0; c3_i30 < 3; c3_i30++) {
    c3_b_C[c3_i30] = c3_P1[c3_i30];
  }

  for (c3_i31 = 0; c3_i31 < 3; c3_i31++) {
    c3_P1[c3_i31] = c3_b_C[c3_i31];
  }

  for (c3_i32 = 0; c3_i32 < 3; c3_i32++) {
    c3_P1[c3_i32] = 0.0;
    c3_i33 = 0;
    for (c3_i34 = 0; c3_i34 < 3; c3_i34++) {
      c3_P1[c3_i32] += c3_b_a[c3_i33 + c3_i32] * c3_b[c3_i34];
      c3_i33 += 3;
    }
  }

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
    c3_d1 = c3_a;
    c3_b_cos(chartInstance, &c3_d1);
    c3_Xl = 50.0 * c3_d1;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 17);
    c3_d2 = c3_a;
    c3_b_sin(chartInstance, &c3_d2);
    c3_Yl = 50.0 * c3_d2;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 20);
    for (c3_i35 = 0; c3_i35 < 9; c3_i35++) {
      c3_b_a[c3_i35] = c3_Tl[c3_i35];
    }

    c3_b_Xl[0] = c3_Xl;
    c3_b_Xl[1] = c3_Yl;
    c3_b_Xl[2] = 1.0;
    for (c3_i36 = 0; c3_i36 < 3; c3_i36++) {
      c3_b_b[c3_i36] = c3_b_Xl[c3_i36];
    }

    for (c3_i37 = 0; c3_i37 < 3; c3_i37++) {
      c3_P2[c3_i37] = 0.0;
    }

    for (c3_i38 = 0; c3_i38 < 3; c3_i38++) {
      c3_P2[c3_i38] = 0.0;
    }

    for (c3_i39 = 0; c3_i39 < 3; c3_i39++) {
      c3_b_C[c3_i39] = c3_P2[c3_i39];
    }

    for (c3_i40 = 0; c3_i40 < 3; c3_i40++) {
      c3_P2[c3_i40] = c3_b_C[c3_i40];
    }

    for (c3_i41 = 0; c3_i41 < 3; c3_i41++) {
      c3_P2[c3_i41] = 0.0;
      c3_i42 = 0;
      for (c3_i43 = 0; c3_i43 < 3; c3_i43++) {
        c3_P2[c3_i41] += c3_b_a[c3_i42 + c3_i41] * c3_b_b[c3_i43];
        c3_i42 += 3;
      }
    }

    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 21);
    c3_x2 = c3_P2[0];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 21);
    c3_y2 = c3_P2[1];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 24);
    c3_b_x1[0] = c3_x1;
    c3_b_x1[1] = c3_y1;
    c3_b_x1[2] = c3_x2 - c3_x1;
    c3_b_x1[3] = c3_y2 - c3_y1;
    c3_clipLine(chartInstance, c3_b_x1, c3_dv8);
    for (c3_i44 = 0; c3_i44 < 4; c3_i44++) {
      c3_edge[c3_i44] = c3_dv8[c3_i44];
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
    c3_p[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct, chartInstance->S,
      0U, 0, 0, MAX_uint32_T, (int32_T)c3_i, 1, 361) - 1] = c3_a;
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 36);
    c3_c_I1[0] = c3_I1;
    c3_c_I1[1] = c3_J1;
    c3_c_I2[0] = c3_I2;
    c3_c_I2[1] = c3_J2;
    for (c3_i45 = 0; c3_i45 < 10000; c3_i45++) {
      c3_d_map[c3_i45] = c3_c_map[c3_i45];
    }

    c3_laserRange(chartInstance, c3_c_I1, c3_c_I2, c3_d_map, c3_tmp_data,
                  c3_tmp_sizes);
    c3_Pxel_sizes[0] = c3_tmp_sizes[0];
    c3_Pxel_sizes[1] = c3_tmp_sizes[1];
    c3_Pxel = c3_Pxel_sizes[0];
    c3_b_Pxel = c3_Pxel_sizes[1];
    c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
    for (c3_i46 = 0; c3_i46 <= c3_loop_ub; c3_i46++) {
      c3_Pxel_data[c3_i46] = c3_tmp_data[c3_i46];
    }

    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 37);
    c3_Io = c3_Pxel_data[0];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 37);
    c3_Jo = c3_Pxel_data[1];
    _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 39);
    c3_x = c3_Io;
    c3_c_b = muDoubleScalarIsInf(c3_x);
    guard1 = false;
    if (CV_SCRIPT_COND(1, 0, c3_c_b)) {
      guard1 = true;
    } else {
      c3_b_x = c3_Jo;
      c3_d_b = muDoubleScalarIsInf(c3_b_x);
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
        c3_c_C = 100.0;
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_h_debug_family_names,
          c3_b_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U,
          c3_d_sf_marshallOut, c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U,
          c3_d_sf_marshallOut, c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_i, 2U, c3_d_sf_marshallOut,
          c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_j, 3U, c3_d_sf_marshallOut,
          c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_Xmax, 4U, c3_d_sf_marshallOut,
          c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_Ymax, 5U, c3_d_sf_marshallOut,
          c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_R, 6U, c3_d_sf_marshallOut,
          c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_C, 7U, c3_d_sf_marshallOut,
          c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_xTarget, 8U,
          c3_d_sf_marshallOut, c3_d_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_yTarget, 9U,
          c3_d_sf_marshallOut, c3_d_sf_marshallIn);
        CV_SCRIPT_FCN(7, 0);
        _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 3);
        c3_A = 25.0 * (c3_b_i - 1.0);
        c3_c_x = c3_A;
        c3_d_x = c3_c_x;
        c3_y = c3_d_x / 99.0;
        c3_b_yTarget = 25.0 - c3_y;
        _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, 4);
        c3_b_A = 25.0 * (c3_j - 1.0);
        c3_e_x = c3_b_A;
        c3_f_x = c3_e_x;
        c3_b_xTarget = c3_f_x / 99.0;
        _SFD_SCRIPT_CALL(7U, chartInstance->c3_sfEvent, -4);
        _SFD_SYMBOL_SCOPE_POP();
        c3_xTarget = c3_b_xTarget;
        c3_yTarget = c3_b_yTarget;
        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 45);
        c3_g_x = c3_mpower(chartInstance, c3_x1 - c3_xTarget) + c3_mpower
          (chartInstance, c3_y1 - c3_yTarget);
        c3_b_r = c3_g_x;
        c3_h_x = c3_b_r;
        c3_b0 = (c3_h_x < 0.0);
        c3_b_p = c3_b0;
        if (c3_b_p) {
          c3_b_error(chartInstance);
        }

        c3_i_x = c3_b_r;
        c3_b_r = c3_i_x;
        c3_j_x = c3_b_r;
        c3_b_r = c3_j_x;
        c3_b_r = muDoubleScalarSqrt(c3_b_r);
        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 46);
        if (CV_SCRIPT_IF(1, 1, CV_RELATIONAL_EVAL(14U, 1U, 0, c3_b_r, 50.0, -1,
              4U, c3_b_r > 50.0))) {
          _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 47);
          c3_b_r = 50.0;
        }

        _SFD_SCRIPT_CALL(1U, chartInstance->c3_sfEvent, 49);
        c3_p[(int32_T)c3_i + 360] = c3_b_r;
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
  real_T c3_b_y;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_b_A;
  real_T c3_f_x;
  real_T c3_g_x;
  real_T c3_c_y;
  real_T c3_h_x;
  real_T c3_i_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c3_c_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x, 2U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y, 3U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Xmax, 4U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Ymax, 5U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R, 6U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C, 7U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_i, 8U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_j, 9U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 4);
  c3_A = 25.0 - c3_y;
  c3_b_x = c3_A;
  c3_c_x = c3_b_x;
  c3_b_y = c3_c_x / 25.0;
  c3_d_x = c3_b_y * 99.0;
  c3_e_x = c3_d_x;
  c3_e_x = muDoubleScalarRound(c3_e_x);
  *c3_i = c3_e_x + 1.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c3_sfEvent, 5);
  c3_b_A = c3_x;
  c3_f_x = c3_b_A;
  c3_g_x = c3_f_x;
  c3_c_y = c3_g_x / 25.0;
  c3_h_x = c3_c_y * 99.0;
  c3_i_x = c3_h_x;
  c3_i_x = muDoubleScalarRound(c3_i_x);
  *c3_j = c3_i_x + 1.0;
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
  real_T c3_pos_data[4];
  int32_T c3_pos_sizes;
  real_T c3_inds_data[4];
  int32_T c3_inds_sizes;
  real_T c3_ind;
  real_T c3_inter1[2];
  real_T c3_inter2[2];
  real_T c3_midX;
  boolean_T c3_xOk;
  real_T c3_midY;
  boolean_T c3_yOk;
  real_T c3_points_data[8];
  int32_T c3_points_sizes[2];
  real_T c3_box[4];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i47;
  static real_T c3_dv9[4] = { 0.0, 25.0, 0.0, 25.0 };

  int32_T c3_i48;
  real_T c3_x;
  real_T c3_y;
  real_T c3_b_x;
  real_T c3_b_y;
  real_T c3_c_x;
  real_T c3_c_y;
  real_T c3_x1;
  real_T c3_x2;
  real_T c3_a;
  real_T c3_b;
  int32_T c3_i49;
  real_T c3_dv10[4];
  real_T c3_b_line[4];
  real_T c3_dv11[2];
  int32_T c3_i50;
  int32_T c3_i51;
  real_T c3_dv12[4];
  real_T c3_c_line[4];
  real_T c3_dv13[2];
  int32_T c3_i52;
  int32_T c3_i53;
  real_T c3_dv14[4];
  real_T c3_d_line[4];
  real_T c3_dv15[2];
  int32_T c3_i54;
  int32_T c3_i55;
  real_T c3_dv16[4];
  real_T c3_e_line[4];
  real_T c3_dv17[2];
  int32_T c3_i56;
  int32_T c3_i57;
  int32_T c3_i58;
  int32_T c3_i59;
  int32_T c3_i60;
  int32_T c3_i61;
  int32_T c3_i62;
  int32_T c3_i63;
  int32_T c3_i64;
  int32_T c3_i65;
  int32_T c3_i66;
  real_T c3_d_x[4];
  int32_T c3_i67;
  boolean_T c3_b_b[4];
  int32_T c3_i68;
  int32_T c3_i69;
  boolean_T c3_c_b[4];
  int32_T c3_i70;
  int32_T c3_trueCount;
  int32_T c3_b_i;
  int32_T c3_iidx_sizes;
  int32_T c3_partialTrueCount;
  int32_T c3_c_i;
  int32_T c3_iidx_data[4];
  int32_T c3_i71;
  int32_T c3_loop_ub;
  int32_T c3_i72;
  int32_T c3_b_points_sizes[2];
  int32_T c3_b_points;
  int32_T c3_c_points;
  int32_T c3_b_loop_ub;
  int32_T c3_i73;
  int32_T c3_i74;
  real_T c3_b_points_data[8];
  real_T c3_f_line[4];
  real_T c3_tmp_data[4];
  int32_T c3_tmp_sizes;
  int32_T c3_c_loop_ub;
  int32_T c3_i75;
  int32_T c3_x_sizes;
  int32_T c3_d_loop_ub;
  int32_T c3_i76;
  real_T c3_x_data[4];
  int32_T c3_b_inds_sizes;
  int32_T c3_e_loop_ub;
  int32_T c3_i77;
  real_T c3_b_inds_data[4];
  int32_T c3_f_loop_ub;
  int32_T c3_i78;
  int32_T c3_g_loop_ub;
  int32_T c3_i79;
  int32_T c3_d_points;
  int32_T c3_c_points_sizes[2];
  int32_T c3_i80;
  int32_T c3_h_loop_ub;
  int32_T c3_i81;
  int32_T c3_i82;
  real_T c3_c_points_data[8];
  int32_T c3_i_loop_ub;
  int32_T c3_i83;
  real_T c3_A;
  real_T c3_e_x;
  real_T c3_f_x;
  int32_T c3_b_ind;
  int32_T c3_i84;
  int32_T c3_c_ind;
  int32_T c3_i85;
  int32_T c3_i86;
  int32_T c3_i87;
  int32_T c3_i88;
  int32_T c3_i89;
  real_T c3_b_edge[2];
  boolean_T c3_b1;
  int32_T c3_i90;
  int32_T c3_i91;
  real_T c3_c_edge[2];
  boolean_T c3_b2;
  boolean_T c3_b3;
  int32_T c3_i92;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 28U, 29U, c3_f_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c3_o_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_nLines, 1U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_nBoxes, 2U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_i, 3U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_xmin, 4U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_xmax, 5U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_ymin, 6U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_ymax, 7U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_delta, 8U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_px1, 9U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_px2, 10U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_py1, 11U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_py2, 12U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_points, MAX_uint32_T,
    c3_r_sf_marshallOut, c3_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_pos_data, (const int32_T *)
    &c3_pos_sizes, NULL, 0, 14, (void *)c3_p_sf_marshallOut, (void *)
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_inds_data, (const int32_T *)
    &c3_inds_sizes, NULL, 0, 15, (void *)c3_p_sf_marshallOut, (void *)
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_ind, 16U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_inter1, 17U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_inter2, 18U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_midX, 19U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xOk, 20U, c3_n_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_midY, 21U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_yOk, 22U, c3_n_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_points_data, (const int32_T *)
    &c3_points_sizes, NULL, 0, -1, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_box, 23U, c3_l_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 24U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 25U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line, 26U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_edge, 27U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  for (c3_i47 = 0; c3_i47 < 4; c3_i47++) {
    c3_box[c3_i47] = c3_dv9[c3_i47];
  }

  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 42);
  c3_nLines = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 43);
  c3_nBoxes = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 44);
  CV_RELATIONAL_EVAL(14U, 3U, 0, c3_nLines, 1.0, -1, 0U, c3_nLines == 1.0);
  CV_SCRIPT_COND(3, 0, true);
  CV_RELATIONAL_EVAL(14U, 3U, 1, c3_nBoxes, 1.0, -1, 4U, c3_nBoxes > 1.0);
  CV_SCRIPT_COND(3, 1, false);
  CV_SCRIPT_MCDC(3, 0, false);
  CV_SCRIPT_IF(3, 0, false);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 46);
  CV_RELATIONAL_EVAL(14U, 3U, 2, c3_nBoxes, 1.0, -1, 0U, c3_nBoxes == 1.0);
  CV_SCRIPT_COND(3, 2, true);
  CV_RELATIONAL_EVAL(14U, 3U, 3, c3_nLines, 1.0, -1, 4U, c3_nLines > 1.0);
  CV_SCRIPT_COND(3, 3, false);
  CV_SCRIPT_MCDC(3, 1, false);
  CV_SCRIPT_IF(3, 1, false);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 48);
  CV_RELATIONAL_EVAL(14U, 3U, 4, c3_nLines, c3_nBoxes, -1, 1U, c3_nLines !=
                     c3_nBoxes);
  CV_SCRIPT_IF(3, 2, false);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 53);
  c3_nLines = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 54);
  for (c3_i48 = 0; c3_i48 < 4; c3_i48++) {
    c3_edge[c3_i48] = 0.0;
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
  c3_x = c3_line[2];
  c3_y = c3_line[3];
  c3_b_x = c3_x;
  c3_b_y = c3_y;
  c3_c_x = c3_b_x;
  c3_c_y = c3_b_y;
  c3_x1 = c3_c_x;
  c3_x2 = c3_c_y;
  c3_a = c3_x1;
  c3_b = c3_x2;
  c3_delta = muDoubleScalarHypot(c3_a, c3_b);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 69);
  for (c3_i49 = 0; c3_i49 < 4; c3_i49++) {
    c3_b_line[c3_i49] = c3_line[c3_i49];
  }

  c3_dv10[0] = 0.0;
  c3_dv10[1] = 0.0;
  c3_dv10[2] = c3_delta;
  c3_dv10[3] = 0.0;
  c3_intersectLines(chartInstance, c3_b_line, c3_dv10, c3_dv11);
  for (c3_i50 = 0; c3_i50 < 2; c3_i50++) {
    c3_px1[c3_i50] = c3_dv11[c3_i50];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 70);
  for (c3_i51 = 0; c3_i51 < 4; c3_i51++) {
    c3_c_line[c3_i51] = c3_line[c3_i51];
  }

  c3_dv12[0] = 25.0;
  c3_dv12[1] = 0.0;
  c3_dv12[2] = 0.0;
  c3_dv12[3] = c3_delta;
  c3_intersectLines(chartInstance, c3_c_line, c3_dv12, c3_dv13);
  for (c3_i52 = 0; c3_i52 < 2; c3_i52++) {
    c3_px2[c3_i52] = c3_dv13[c3_i52];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 71);
  for (c3_i53 = 0; c3_i53 < 4; c3_i53++) {
    c3_d_line[c3_i53] = c3_line[c3_i53];
  }

  c3_dv14[0] = 25.0;
  c3_dv14[1] = 25.0;
  c3_dv14[2] = -c3_delta;
  c3_dv14[3] = 0.0;
  c3_intersectLines(chartInstance, c3_d_line, c3_dv14, c3_dv15);
  for (c3_i54 = 0; c3_i54 < 2; c3_i54++) {
    c3_py1[c3_i54] = c3_dv15[c3_i54];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 72);
  for (c3_i55 = 0; c3_i55 < 4; c3_i55++) {
    c3_e_line[c3_i55] = c3_line[c3_i55];
  }

  c3_dv16[0] = 0.0;
  c3_dv16[1] = 25.0;
  c3_dv16[2] = 0.0;
  c3_dv16[3] = -c3_delta;
  c3_intersectLines(chartInstance, c3_e_line, c3_dv16, c3_dv17);
  for (c3_i56 = 0; c3_i56 < 2; c3_i56++) {
    c3_py2[c3_i56] = c3_dv17[c3_i56];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 75);
  c3_i57 = 0;
  for (c3_i58 = 0; c3_i58 < 2; c3_i58++) {
    c3_points[c3_i57] = c3_px1[c3_i58];
    c3_i57 += 4;
  }

  c3_i59 = 0;
  for (c3_i60 = 0; c3_i60 < 2; c3_i60++) {
    c3_points[c3_i59 + 1] = c3_px2[c3_i60];
    c3_i59 += 4;
  }

  c3_i61 = 0;
  for (c3_i62 = 0; c3_i62 < 2; c3_i62++) {
    c3_points[c3_i61 + 2] = c3_py1[c3_i62];
    c3_i61 += 4;
  }

  c3_i63 = 0;
  for (c3_i64 = 0; c3_i64 < 2; c3_i64++) {
    c3_points[c3_i63 + 3] = c3_py2[c3_i64];
    c3_i63 += 4;
  }

  _SFD_SYMBOL_SWITCH(13U, 13U);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 76);
  for (c3_i65 = 0; c3_i65 < 4; c3_i65++) {
    c3_d_x[c3_i65] = c3_points[c3_i65];
  }

  for (c3_i66 = 0; c3_i66 < 4; c3_i66++) {
    c3_b_b[c3_i66] = muDoubleScalarIsInf(c3_d_x[c3_i66]);
  }

  for (c3_i67 = 0; c3_i67 < 4; c3_i67++) {
    c3_b_b[c3_i67] = !c3_b_b[c3_i67];
  }

  for (c3_i68 = 0; c3_i68 < 4; c3_i68++) {
    c3_c_b[c3_i68] = muDoubleScalarIsNaN(c3_d_x[c3_i68]);
  }

  for (c3_i69 = 0; c3_i69 < 4; c3_i69++) {
    c3_c_b[c3_i69] = !c3_c_b[c3_i69];
  }

  for (c3_i70 = 0; c3_i70 < 4; c3_i70++) {
    c3_b_b[c3_i70] = (c3_b_b[c3_i70] && c3_c_b[c3_i70]);
  }

  c3_trueCount = 0;
  c3_b_i = 0;
  while (c3_b_i <= 3) {
    if (c3_b_b[c3_b_i]) {
      c3_trueCount++;
    }

    c3_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_iidx_sizes = c3_trueCount;
  c3_partialTrueCount = 0;
  c3_c_i = 0;
  while (c3_c_i <= 3) {
    if (c3_b_b[c3_c_i]) {
      c3_iidx_data[c3_partialTrueCount] = c3_c_i + 1;
      c3_partialTrueCount++;
    }

    c3_c_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_points_sizes[0] = c3_iidx_sizes;
  c3_points_sizes[1] = 2;
  for (c3_i71 = 0; c3_i71 < 2; c3_i71++) {
    c3_loop_ub = c3_iidx_sizes - 1;
    for (c3_i72 = 0; c3_i72 <= c3_loop_ub; c3_i72++) {
      c3_points_data[c3_i72 + c3_points_sizes[0] * c3_i71] = c3_points
        [(c3_iidx_data[c3_i72] + (c3_i71 << 2)) - 1];
    }
  }

  _SFD_SYMBOL_SWITCH(13U, 23U);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 79);
  c3_b_points_sizes[0] = c3_points_sizes[0];
  c3_b_points_sizes[1] = 2;
  c3_b_points = c3_b_points_sizes[0];
  c3_c_points = c3_b_points_sizes[1];
  c3_b_loop_ub = c3_points_sizes[0] * c3_points_sizes[1] - 1;
  for (c3_i73 = 0; c3_i73 <= c3_b_loop_ub; c3_i73++) {
    c3_b_points_data[c3_i73] = c3_points_data[c3_i73];
  }

  for (c3_i74 = 0; c3_i74 < 4; c3_i74++) {
    c3_f_line[c3_i74] = c3_line[c3_i74];
  }

  c3_linePosition(chartInstance, c3_b_points_data, c3_b_points_sizes, c3_f_line,
                  c3_tmp_data, &c3_tmp_sizes);
  c3_pos_sizes = c3_tmp_sizes;
  c3_c_loop_ub = c3_tmp_sizes - 1;
  for (c3_i75 = 0; c3_i75 <= c3_c_loop_ub; c3_i75++) {
    c3_pos_data[c3_i75] = c3_tmp_data[c3_i75];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 80);
  c3_x_sizes = c3_pos_sizes;
  c3_d_loop_ub = c3_pos_sizes - 1;
  for (c3_i76 = 0; c3_i76 <= c3_d_loop_ub; c3_i76++) {
    c3_x_data[c3_i76] = c3_pos_data[c3_i76];
  }

  c3_b_sort(chartInstance, c3_x_data, &c3_x_sizes, c3_iidx_data, &c3_iidx_sizes);
  c3_b_inds_sizes = c3_iidx_sizes;
  c3_e_loop_ub = c3_iidx_sizes - 1;
  for (c3_i77 = 0; c3_i77 <= c3_e_loop_ub; c3_i77++) {
    c3_b_inds_data[c3_i77] = (real_T)c3_iidx_data[c3_i77];
  }

  c3_pos_sizes = c3_x_sizes;
  c3_f_loop_ub = c3_x_sizes - 1;
  for (c3_i78 = 0; c3_i78 <= c3_f_loop_ub; c3_i78++) {
    c3_pos_data[c3_i78] = c3_x_data[c3_i78];
  }

  c3_inds_sizes = c3_b_inds_sizes;
  c3_g_loop_ub = c3_b_inds_sizes - 1;
  for (c3_i79 = 0; c3_i79 <= c3_g_loop_ub; c3_i79++) {
    c3_inds_data[c3_i79] = c3_b_inds_data[c3_i79];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 81);
  c3_d_points = c3_points_sizes[0];
  c3_c_points_sizes[0] = c3_inds_sizes;
  c3_c_points_sizes[1] = 2;
  for (c3_i80 = 0; c3_i80 < 2; c3_i80++) {
    c3_h_loop_ub = c3_inds_sizes - 1;
    for (c3_i81 = 0; c3_i81 <= c3_h_loop_ub; c3_i81++) {
      c3_c_points_data[c3_i81 + c3_c_points_sizes[0] * c3_i80] = c3_points_data
        [(sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
           chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)
           c3_inds_data[c3_i81], 1, c3_d_points) + c3_points_sizes[0] * c3_i80)
        - 1];
    }
  }

  c3_points_sizes[0] = c3_c_points_sizes[0];
  c3_points_sizes[1] = 2;
  for (c3_i82 = 0; c3_i82 < 2; c3_i82++) {
    c3_i_loop_ub = c3_c_points_sizes[0] - 1;
    for (c3_i83 = 0; c3_i83 <= c3_i_loop_ub; c3_i83++) {
      c3_points_data[c3_i83 + c3_points_sizes[0] * c3_i82] =
        c3_c_points_data[c3_i83 + c3_c_points_sizes[0] * c3_i82];
    }
  }

  _SFD_SYMBOL_SWITCH(13U, 23U);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 84);
  c3_A = (real_T)c3_points_sizes[0];
  c3_e_x = c3_A;
  c3_f_x = c3_e_x;
  c3_ind = c3_f_x / 2.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 85);
  c3_b_ind = sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
    chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
    (chartInstance->S, 0U, 0U, 0U, c3_ind), 1, c3_points_sizes[0]) - 1;
  for (c3_i84 = 0; c3_i84 < 2; c3_i84++) {
    c3_inter1[c3_i84] = c3_points_data[c3_b_ind + c3_points_sizes[0] * c3_i84];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 86);
  c3_c_ind = sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
    chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)(c3_ind + 1.0), 1,
    c3_points_sizes[0]) - 1;
  for (c3_i85 = 0; c3_i85 < 2; c3_i85++) {
    c3_inter2[c3_i85] = c3_points_data[c3_c_ind + c3_points_sizes[0] * c3_i85];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 87);
  for (c3_i86 = 0; c3_i86 < 2; c3_i86++) {
    c3_edge[c3_i86] = c3_inter1[c3_i86];
  }

  for (c3_i87 = 0; c3_i87 < 2; c3_i87++) {
    c3_edge[c3_i87 + 2] = c3_inter2[c3_i87];
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 90);
  c3_i88 = 0;
  for (c3_i89 = 0; c3_i89 < 2; c3_i89++) {
    c3_b_edge[c3_i89] = c3_edge[c3_i88];
    c3_i88 += 2;
  }

  c3_midX = c3_mean(chartInstance, c3_b_edge);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 91);
  guard3 = false;
  if (0.0 <= c3_midX) {
    if (c3_midX <= 25.0) {
      c3_b1 = true;
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3 == true) {
    c3_b1 = false;
  }

  c3_xOk = c3_b1;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 92);
  c3_i90 = 0;
  for (c3_i91 = 0; c3_i91 < 2; c3_i91++) {
    c3_c_edge[c3_i91] = c3_edge[c3_i90 + 1];
    c3_i90 += 2;
  }

  c3_midY = c3_mean(chartInstance, c3_c_edge);
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 93);
  guard2 = false;
  if (0.0 <= c3_midY) {
    if (c3_midY <= 25.0) {
      c3_b2 = true;
    } else {
      guard2 = true;
    }
  } else {
    guard2 = true;
  }

  if (guard2 == true) {
    c3_b2 = false;
  }

  c3_yOk = c3_b2;
  _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 96);
  guard1 = false;
  if (CV_SCRIPT_COND(3, 4, c3_xOk)) {
    if (CV_SCRIPT_COND(3, 5, c3_yOk)) {
      c3_b3 = true;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    c3_b3 = false;
  }

  if (CV_SCRIPT_IF(3, 3, CV_SCRIPT_MCDC(3, 2, !c3_b3))) {
    _SFD_SCRIPT_CALL(3U, chartInstance->c3_sfEvent, 97);
    for (c3_i92 = 0; c3_i92 < 4; c3_i92++) {
      c3_edge[c3_i92] = rtNaN;
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
  real_T c3_denom_data[1];
  int32_T c3_denom_sizes[2];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  boolean_T c3_b4;
  boolean_T c3_b_par;
  real_T c3_dv18[1];
  int32_T c3_trueCount;
  int32_T c3_i;
  int32_T c3_partialTrueCount;
  int32_T c3_b_i;
  int32_T c3_iv0[2];
  real_T c3_dv19[1];
  int32_T c3_b_trueCount;
  int32_T c3_tmp_sizes[2];
  int32_T c3_c_i;
  int32_T c3_i93;
  int32_T c3_i94;
  int32_T c3_b_partialTrueCount;
  int32_T c3_loop_ub;
  int32_T c3_d_i;
  int32_T c3_i95;
  real_T c3_tmp_data[1];
  real_T c3_dv20[1];
  boolean_T c3_b5;
  int32_T c3_c_trueCount;
  int32_T c3_e_i;
  int32_T c3_i96;
  int32_T c3_i97;
  int32_T c3_b_loop_ub;
  int32_T c3_c_partialTrueCount;
  int32_T c3_i98;
  int32_T c3_f_i;
  real_T c3_dv21[1];
  boolean_T c3_b6;
  int32_T c3_d_trueCount;
  int32_T c3_g_i;
  int32_T c3_i99;
  int32_T c3_i100;
  int32_T c3_c_loop_ub;
  int32_T c3_d_partialTrueCount;
  int32_T c3_i101;
  int32_T c3_h_i;
  int32_T c3_i102;
  int32_T c3_i103;
  int32_T c3_d_loop_ub;
  int32_T c3_i104;
  int32_T c3_e_trueCount;
  int32_T c3_i_i;
  int32_T c3_b_denom;
  int32_T c3_c_denom;
  int32_T c3_e_loop_ub;
  int32_T c3_i105;
  int32_T c3_b_denom_sizes[2];
  int32_T c3_d_denom;
  int32_T c3_e_denom;
  int32_T c3_f_loop_ub;
  int32_T c3_i106;
  real_T c3_b_denom_data[1];
  int32_T c3_f_trueCount;
  int32_T c3_j_i;
  real_T c3_dv22[1];
  int32_T c3_e_partialTrueCount;
  int32_T c3_k_i;
  int32_T c3_c_denom_sizes[2];
  int32_T c3_f_denom;
  int32_T c3_g_denom;
  int32_T c3_g_loop_ub;
  int32_T c3_i107;
  real_T c3_c_denom_data[1];
  int32_T c3_g_trueCount;
  int32_T c3_l_i;
  real_T c3_dv23[1];
  int32_T c3_f_partialTrueCount;
  int32_T c3_m_i;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 26U, 27U, c3_d_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c3_o_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tol, 1U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N1, 2U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N2, 3U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_N, 4U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx, 5U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy, 6U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_denom, MAX_uint32_T,
    c3_d_sf_marshallOut, c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_par, 8U, c3_n_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_col, 9U, c3_n_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x0, 10U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y0, 11U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_inds, 12U, c3_n_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x1, 13U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y1, 14U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx1, 15U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy1, 16U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x2, 17U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y2, 18U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx2, 19U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy2, 20U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_denom_data, (const int32_T *)
    &c3_denom_sizes, NULL, 0, -1, (void *)c3_m_sf_marshallOut, (void *)
    c3_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 21U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 22U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line1, 23U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line2, 24U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_point, 25U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
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
  CV_RELATIONAL_EVAL(14U, 4U, 0, c3_N1, c3_N2, -1, 1U, c3_N1 != c3_N2);
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
  c3_par = (c3_abs(chartInstance, c3_denom) < 1.0E-14);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 72);
  c3_b4 = (c3_abs(chartInstance, c3_dx * c3_line1[3] - c3_dy * c3_line1[2]) <
           1.0E-14);
  c3_b_par = c3_par;
  c3_col = (c3_b4 && c3_b_par);
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 75);
  c3_x0 = 0.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 76);
  c3_y0 = 0.0;
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 79);
  c3_dv18[0] = 0.0;
  c3_trueCount = 0;
  c3_i = 0;
  while (c3_i <= 0) {
    if (c3_col) {
      c3_trueCount++;
    }

    c3_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_partialTrueCount = 0;
  c3_b_i = 0;
  while (c3_b_i <= 0) {
    if (c3_col) {
      c3_iv0[0] = 1;
      c3_iv0[1] = c3_trueCount;
      c3_tmp_sizes[0] = 1;
      c3_tmp_sizes[1] = c3_iv0[1];
      c3_i93 = c3_tmp_sizes[0];
      c3_i94 = c3_tmp_sizes[1];
      c3_loop_ub = c3_iv0[0] * c3_iv0[1] - 1;
      for (c3_i95 = 0; c3_i95 <= c3_loop_ub; c3_i95++) {
        c3_tmp_data[c3_i95] = rtInf;
      }

      c3_dv18[c3_b_i] = c3_tmp_data[c3_partialTrueCount];
      c3_partialTrueCount++;
    }

    c3_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_x0 = c3_dv18[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 80);
  c3_dv19[0] = 0.0;
  c3_b_trueCount = 0;
  c3_c_i = 0;
  while (c3_c_i <= 0) {
    if (c3_col) {
      c3_b_trueCount++;
    }

    c3_c_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_b_partialTrueCount = 0;
  c3_d_i = 0;
  while (c3_d_i <= 0) {
    if (c3_col) {
      c3_iv0[0] = 1;
      c3_iv0[1] = c3_b_trueCount;
      c3_tmp_sizes[0] = 1;
      c3_tmp_sizes[1] = c3_iv0[1];
      c3_i96 = c3_tmp_sizes[0];
      c3_i97 = c3_tmp_sizes[1];
      c3_b_loop_ub = c3_iv0[0] * c3_iv0[1] - 1;
      for (c3_i98 = 0; c3_i98 <= c3_b_loop_ub; c3_i98++) {
        c3_tmp_data[c3_i98] = rtInf;
      }

      c3_dv19[c3_d_i] = c3_tmp_data[c3_b_partialTrueCount];
      c3_b_partialTrueCount++;
    }

    c3_d_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_y0 = c3_dv19[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 81);
  c3_dv20[0] = c3_x0;
  c3_b5 = !c3_col;
  c3_c_trueCount = 0;
  c3_e_i = 0;
  while (c3_e_i <= 0) {
    if (c3_par && c3_b5) {
      c3_c_trueCount++;
    }

    c3_e_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_c_partialTrueCount = 0;
  c3_f_i = 0;
  while (c3_f_i <= 0) {
    if (c3_par && c3_b5) {
      c3_iv0[0] = 1;
      c3_iv0[1] = c3_c_trueCount;
      c3_tmp_sizes[0] = 1;
      c3_tmp_sizes[1] = c3_iv0[1];
      c3_i99 = c3_tmp_sizes[0];
      c3_i100 = c3_tmp_sizes[1];
      c3_c_loop_ub = c3_iv0[0] * c3_iv0[1] - 1;
      for (c3_i101 = 0; c3_i101 <= c3_c_loop_ub; c3_i101++) {
        c3_tmp_data[c3_i101] = rtNaN;
      }

      c3_dv20[c3_f_i] = c3_tmp_data[c3_c_partialTrueCount];
      c3_c_partialTrueCount++;
    }

    c3_f_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_x0 = c3_dv20[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 82);
  c3_dv21[0] = c3_y0;
  c3_b6 = !c3_col;
  c3_d_trueCount = 0;
  c3_g_i = 0;
  while (c3_g_i <= 0) {
    if (c3_par && c3_b6) {
      c3_d_trueCount++;
    }

    c3_g_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_d_partialTrueCount = 0;
  c3_h_i = 0;
  while (c3_h_i <= 0) {
    if (c3_par && c3_b6) {
      c3_iv0[0] = 1;
      c3_iv0[1] = c3_d_trueCount;
      c3_tmp_sizes[0] = 1;
      c3_tmp_sizes[1] = c3_iv0[1];
      c3_i102 = c3_tmp_sizes[0];
      c3_i103 = c3_tmp_sizes[1];
      c3_d_loop_ub = c3_iv0[0] * c3_iv0[1] - 1;
      for (c3_i104 = 0; c3_i104 <= c3_d_loop_ub; c3_i104++) {
        c3_tmp_data[c3_i104] = rtNaN;
      }

      c3_dv21[c3_h_i] = c3_tmp_data[c3_d_partialTrueCount];
      c3_d_partialTrueCount++;
    }

    c3_h_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_y0 = c3_dv21[0];
  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 85);
  if (CV_SCRIPT_IF(4, 2, c3_all(chartInstance, c3_par))) {
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 86);
    c3_point[0] = c3_x0;
    c3_point[1] = c3_y0;
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 87);
  } else {
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 94);
    c3_inds = !c3_par;
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 97);
    CV_RELATIONAL_EVAL(14U, 4U, 2, c3_N1, 1.0, -1, 4U, c3_N1 > 1.0);
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
    CV_RELATIONAL_EVAL(14U, 4U, 3, c3_N2, 1.0, -1, 4U, c3_N2 > 1.0);
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
    c3_e_trueCount = 0;
    c3_i_i = 0;
    while (c3_i_i <= 0) {
      if (c3_inds) {
        c3_e_trueCount++;
      }

      c3_i_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    c3_denom_sizes[0] = 1;
    c3_denom_sizes[1] = c3_e_trueCount;
    c3_b_denom = c3_denom_sizes[0];
    c3_c_denom = c3_denom_sizes[1];
    c3_e_loop_ub = c3_e_trueCount - 1;
    for (c3_i105 = 0; c3_i105 <= c3_e_loop_ub; c3_i105++) {
      c3_denom_data[c3_i105] = c3_denom;
    }

    _SFD_SYMBOL_SWITCH(7U, 21U);
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 122);
    c3_b_denom_sizes[0] = 1;
    c3_b_denom_sizes[1] = c3_denom_sizes[1];
    c3_d_denom = c3_b_denom_sizes[0];
    c3_e_denom = c3_b_denom_sizes[1];
    c3_f_loop_ub = c3_denom_sizes[0] * c3_denom_sizes[1] - 1;
    for (c3_i106 = 0; c3_i106 <= c3_f_loop_ub; c3_i106++) {
      c3_b_denom_data[c3_i106] = c3_denom_data[c3_i106];
    }

    c3_rdivide(chartInstance, (c3_x2 * c3_dy2 * c3_dx1 - c3_dy * c3_dx1 * c3_dx2)
               - c3_x1 * c3_dy1 * c3_dx2, c3_b_denom_data, c3_b_denom_sizes,
               c3_tmp_data, c3_tmp_sizes);
    c3_f_trueCount = 0;
    c3_j_i = 0;
    while (c3_j_i <= 0) {
      if (c3_inds) {
        c3_f_trueCount++;
      }

      c3_j_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    c3_iv0[0] = 1;
    c3_iv0[1] = c3_f_trueCount;
    _SFD_SIZE_EQ_CHECK_1D(c3_iv0[1], c3_tmp_sizes[1]);
    c3_dv22[0] = c3_x0;
    c3_e_partialTrueCount = 0;
    c3_k_i = 0;
    while (c3_k_i <= 0) {
      if (c3_inds) {
        c3_dv22[c3_k_i] = c3_tmp_data[c3_e_partialTrueCount];
        c3_e_partialTrueCount++;
      }

      c3_k_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    c3_x0 = c3_dv22[0];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 123);
    c3_c_denom_sizes[0] = 1;
    c3_c_denom_sizes[1] = c3_denom_sizes[1];
    c3_f_denom = c3_c_denom_sizes[0];
    c3_g_denom = c3_c_denom_sizes[1];
    c3_g_loop_ub = c3_denom_sizes[0] * c3_denom_sizes[1] - 1;
    for (c3_i107 = 0; c3_i107 <= c3_g_loop_ub; c3_i107++) {
      c3_c_denom_data[c3_i107] = c3_denom_data[c3_i107];
    }

    c3_rdivide(chartInstance, (c3_dx * c3_dy1 * c3_dy2 + c3_y1 * c3_dx1 * c3_dy2)
               - c3_y2 * c3_dx2 * c3_dy1, c3_c_denom_data, c3_c_denom_sizes,
               c3_tmp_data, c3_tmp_sizes);
    c3_g_trueCount = 0;
    c3_l_i = 0;
    while (c3_l_i <= 0) {
      if (c3_inds) {
        c3_g_trueCount++;
      }

      c3_l_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    c3_iv0[0] = 1;
    c3_iv0[1] = c3_g_trueCount;
    _SFD_SIZE_EQ_CHECK_1D(c3_iv0[1], c3_tmp_sizes[1]);
    c3_dv23[0] = c3_y0;
    c3_f_partialTrueCount = 0;
    c3_m_i = 0;
    while (c3_m_i <= 0) {
      if (c3_inds) {
        c3_dv23[c3_m_i] = c3_tmp_data[c3_f_partialTrueCount];
        c3_f_partialTrueCount++;
      }

      c3_m_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    c3_y0 = c3_dv23[0];
    _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, 126);
    c3_point[0] = c3_x0;
    c3_point[1] = c3_y0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c3_sfEvent, -126);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_linePosition(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_point_data[], int32_T c3_point_sizes[2], real_T
  c3_line[4], real_T c3_pos_data[], int32_T *c3_pos_sizes)
{
  uint32_T c3_debug_family_var_map[12];
  real_T c3_vx;
  real_T c3_vy;
  real_T c3_dx_data[4];
  int32_T c3_dx_sizes;
  real_T c3_dy_data[4];
  int32_T c3_dy_sizes;
  real_T c3_delta;
  boolean_T c3_invalidLine;
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  int32_T c3_i108;
  int32_T c3_b_point_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i109;
  real_T c3_b_point_data[4];
  real_T c3_tmp_data[4];
  int32_T c3_tmp_sizes;
  int32_T c3_b_loop_ub;
  int32_T c3_i110;
  int32_T c3_i111;
  int32_T c3_c_point_sizes;
  int32_T c3_c_loop_ub;
  int32_T c3_i112;
  real_T c3_c_point_data[4];
  real_T c3_b_tmp_data[4];
  int32_T c3_b_tmp_sizes;
  int32_T c3_d_loop_ub;
  int32_T c3_i113;
  real_T c3_dv24[1];
  int32_T c3_trueCount;
  int32_T c3_i;
  int32_T c3_partialTrueCount;
  int32_T c3_b_i;
  int32_T c3_iv1[2];
  int32_T c3_b_dx_sizes;
  int32_T c3_e_loop_ub;
  int32_T c3_c_tmp_sizes[2];
  int32_T c3_i114;
  int32_T c3_i115;
  int32_T c3_i116;
  real_T c3_b_dx_data[4];
  real_T c3_c_tmp_data[4];
  int32_T c3_d_tmp_sizes;
  int32_T c3_f_loop_ub;
  int32_T c3_b_dy_sizes;
  int32_T c3_i117;
  int32_T c3_g_loop_ub;
  int32_T c3_i118;
  real_T c3_d_tmp_data[1];
  real_T c3_b_dy_data[4];
  real_T c3_e_tmp_data[4];
  int32_T c3_e_tmp_sizes;
  int32_T c3_f_tmp_sizes;
  int32_T c3_h_loop_ub;
  int32_T c3_i119;
  real_T c3_f_tmp_data[4];
  real_T c3_g_tmp_data[4];
  int32_T c3_g_tmp_sizes;
  int32_T c3_i_loop_ub;
  int32_T c3_i120;
  int32_T c3_i121;
  int32_T c3_h_tmp_sizes;
  int32_T c3_j_loop_ub;
  int32_T c3_i122;
  int32_T c3_b_trueCount;
  int32_T c3_h_tmp_data[4];
  int32_T c3_c_i;
  int32_T c3_i_tmp_sizes[2];
  int32_T c3_b_partialTrueCount;
  int32_T c3_d_i;
  int32_T c3_iv2[2];
  int32_T c3_i_tmp_data[1];
  int32_T c3_pos;
  int32_T c3_b_pos[2];
  int32_T c3_k_loop_ub;
  int32_T c3_i123;
  int32_T c3_l_loop_ub;
  int32_T c3_i124;
  int32_T c3_j_tmp_sizes[2];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c3_e_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c3_o_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_vx, 1U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_vy, 2U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_dx_data, (const int32_T *)
    &c3_dx_sizes, NULL, 0, 3, (void *)c3_p_sf_marshallOut, (void *)
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_dy_data, (const int32_T *)
    &c3_dy_sizes, NULL, 0, 4, (void *)c3_p_sf_marshallOut, (void *)
    c3_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_delta, 5U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_invalidLine, 6U, c3_n_sf_marshallOut,
    c3_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 7U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 8U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_point_data, (const int32_T *)
    c3_point_sizes, NULL, 1, 9, (void *)c3_q_sf_marshallOut, (void *)
    c3_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_line, 10U, c3_l_sf_marshallOut,
    c3_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_pos_data, (const int32_T *)
    c3_pos_sizes, NULL, 0, 11, (void *)c3_p_sf_marshallOut, (void *)
    c3_o_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 52);
  CV_SCRIPT_COND(5, 0, true);
  CV_SCRIPT_MCDC(5, 0, false);
  CV_SCRIPT_IF(5, 0, false);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 76);
  c3_vx = c3_line[2];
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 77);
  c3_vy = c3_line[3];
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 80);
  c3_i108 = c3_point_sizes[0];
  c3_b_point_sizes = c3_i108;
  c3_loop_ub = c3_i108 - 1;
  for (c3_i109 = 0; c3_i109 <= c3_loop_ub; c3_i109++) {
    c3_b_point_data[c3_i109] = c3_point_data[c3_i109];
  }

  c3_b_bsxfun(chartInstance, c3_b_point_data, c3_b_point_sizes, c3_line[0],
              c3_tmp_data, &c3_tmp_sizes);
  c3_dx_sizes = c3_tmp_sizes;
  c3_b_loop_ub = c3_tmp_sizes - 1;
  for (c3_i110 = 0; c3_i110 <= c3_b_loop_ub; c3_i110++) {
    c3_dx_data[c3_i110] = c3_tmp_data[c3_i110];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 81);
  c3_i111 = c3_point_sizes[0];
  c3_c_point_sizes = c3_i111;
  c3_c_loop_ub = c3_i111 - 1;
  for (c3_i112 = 0; c3_i112 <= c3_c_loop_ub; c3_i112++) {
    c3_c_point_data[c3_i112] = c3_point_data[c3_i112 + c3_point_sizes[0]];
  }

  c3_b_bsxfun(chartInstance, c3_c_point_data, c3_c_point_sizes, c3_line[1],
              c3_b_tmp_data, &c3_b_tmp_sizes);
  c3_dy_sizes = c3_b_tmp_sizes;
  c3_d_loop_ub = c3_b_tmp_sizes - 1;
  for (c3_i113 = 0; c3_i113 <= c3_d_loop_ub; c3_i113++) {
    c3_dy_data[c3_i113] = c3_b_tmp_data[c3_i113];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 86);
  c3_delta = c3_vx * c3_vx + c3_vy * c3_vy;
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 87);
  c3_invalidLine = (c3_delta < 2.2204460492503131E-16);
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 88);
  c3_dv24[0] = c3_delta;
  c3_trueCount = 0;
  c3_i = 0;
  while (c3_i <= 0) {
    if (c3_invalidLine) {
      c3_trueCount++;
    }

    c3_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_partialTrueCount = 0;
  c3_b_i = 0;
  while (c3_b_i <= 0) {
    if (c3_invalidLine) {
      c3_iv1[0] = 1;
      c3_iv1[1] = c3_trueCount;
      c3_c_tmp_sizes[0] = 1;
      c3_c_tmp_sizes[1] = c3_iv1[1];
      c3_i115 = c3_c_tmp_sizes[0];
      c3_i116 = c3_c_tmp_sizes[1];
      c3_f_loop_ub = c3_iv1[0] * c3_iv1[1] - 1;
      for (c3_i117 = 0; c3_i117 <= c3_f_loop_ub; c3_i117++) {
        c3_d_tmp_data[c3_i117] = 1.0;
      }

      c3_dv24[c3_b_i] = c3_d_tmp_data[c3_partialTrueCount];
      c3_partialTrueCount++;
    }

    c3_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_delta = c3_dv24[0];
  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 92);
  c3_b_dx_sizes = c3_dx_sizes;
  c3_e_loop_ub = c3_dx_sizes - 1;
  for (c3_i114 = 0; c3_i114 <= c3_e_loop_ub; c3_i114++) {
    c3_b_dx_data[c3_i114] = c3_dx_data[c3_i114];
  }

  c3_c_bsxfun(chartInstance, c3_b_dx_data, c3_b_dx_sizes, c3_vx, c3_c_tmp_data,
              &c3_d_tmp_sizes);
  c3_b_dy_sizes = c3_dy_sizes;
  c3_g_loop_ub = c3_dy_sizes - 1;
  for (c3_i118 = 0; c3_i118 <= c3_g_loop_ub; c3_i118++) {
    c3_b_dy_data[c3_i118] = c3_dy_data[c3_i118];
  }

  c3_c_bsxfun(chartInstance, c3_b_dy_data, c3_b_dy_sizes, c3_vy, c3_e_tmp_data,
              &c3_e_tmp_sizes);
  _SFD_SIZE_EQ_CHECK_1D(c3_d_tmp_sizes, c3_e_tmp_sizes);
  c3_f_tmp_sizes = c3_d_tmp_sizes;
  c3_h_loop_ub = c3_d_tmp_sizes - 1;
  for (c3_i119 = 0; c3_i119 <= c3_h_loop_ub; c3_i119++) {
    c3_f_tmp_data[c3_i119] = c3_c_tmp_data[c3_i119] + c3_e_tmp_data[c3_i119];
  }

  c3_d_bsxfun(chartInstance, c3_f_tmp_data, c3_f_tmp_sizes, c3_delta,
              c3_g_tmp_data, &c3_g_tmp_sizes);
  *c3_pos_sizes = c3_g_tmp_sizes;
  c3_i_loop_ub = c3_g_tmp_sizes - 1;
  for (c3_i120 = 0; c3_i120 <= c3_i_loop_ub; c3_i120++) {
    c3_pos_data[c3_i120] = c3_g_tmp_data[c3_i120];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, 96);
  c3_i121 = *c3_pos_sizes;
  c3_h_tmp_sizes = c3_i121;
  c3_j_loop_ub = c3_i121 - 1;
  for (c3_i122 = 0; c3_i122 <= c3_j_loop_ub; c3_i122++) {
    c3_h_tmp_data[c3_i122] = 1 + c3_i122;
  }

  c3_b_trueCount = 0;
  c3_c_i = 0;
  while (c3_c_i <= 0) {
    if (c3_invalidLine) {
      c3_b_trueCount++;
    }

    c3_c_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_i_tmp_sizes[0] = 1;
  c3_i_tmp_sizes[1] = c3_b_trueCount;
  c3_b_partialTrueCount = 0;
  c3_d_i = 1;
  while (c3_d_i - 1 <= 0) {
    if (c3_invalidLine) {
      c3_i_tmp_data[c3_b_partialTrueCount] = c3_d_i;
      c3_b_partialTrueCount++;
    }

    c3_d_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c3_iv2[0] = c3_h_tmp_sizes;
  c3_iv2[1] = c3_i_tmp_sizes[1];
  c3_pos = *c3_pos_sizes;
  c3_b_pos[0] = c3_pos;
  c3_b_pos[1] = 1;
  c3_k_loop_ub = c3_iv2[1] - 1;
  for (c3_i123 = 0; c3_i123 <= c3_k_loop_ub; c3_i123++) {
    c3_l_loop_ub = c3_iv2[0] - 1;
    for (c3_i124 = 0; c3_i124 <= c3_l_loop_ub; c3_i124++) {
      c3_j_tmp_sizes[0] = c3_b_pos[0];
      c3_j_tmp_sizes[1] = c3_b_pos[1];
      c3_pos_data[(c3_h_tmp_data[c3_i124] + c3_j_tmp_sizes[0] *
                   (c3_i_tmp_data[c3_i_tmp_sizes[0] * c3_i123] - 1)) - 1] = 0.0;
    }
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c3_sfEvent, -96);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c3_laserRange(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_p1[2], real_T c3_p2[2], real_T c3_c_map[10000],
  real_T c3_p_data[], int32_T c3_p_sizes[2])
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
  real_T c3_d3;
  real_T c3_d4;
  int32_T c3_c_p;
  int32_T c3_d_p;
  real_T c3_b_x[2];
  real_T c3_c_x[2];
  int32_T c3_e_p;
  real_T c3_d_x[2];
  int32_T c3_f_p;
  int32_T c3_i125;
  int32_T c3_i126;
  int32_T c3_i127;
  int32_T c3_i128;
  int32_T c3_g_p;
  int32_T c3_h_p;
  boolean_T c3_e_x[2];
  int32_T c3_i_p;
  boolean_T c3_f_x[2];
  int32_T c3_j_p;
  int32_T c3_i129;
  int32_T c3_i130;
  int32_T c3_k_p;
  int32_T c3_l_p;
  int32_T c3_m_p;
  int32_T c3_n_p;
  int32_T c3_i131;
  int32_T c3_i132;
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
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_R, 0U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_C, 1U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x1, 2U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y1, 3U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x2, 4U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y2, 5U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_x, 6U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_xd, 7U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dx, 8U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_y, 9U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_yd, 10U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_dy, 11U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_a, 12U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b, 13U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c, 14U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 15U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 16U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p1, 17U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_p2, 18U, c3_k_sf_marshallOut,
    c3_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_c_map, 19U, c3_e_sf_marshallOut,
    c3_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c3_p_data, (const int32_T *)
    c3_p_sizes, NULL, 0, 20, (void *)c3_s_sf_marshallOut, (void *)
    c3_r_sf_marshallIn);
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
  if (CV_SCRIPT_IF(6, 0, CV_RELATIONAL_EVAL(14U, 6U, 0, c3_x2, c3_x1, -1, 4U,
        c3_x2 > c3_x1))) {
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
  if (CV_SCRIPT_IF(6, 1, CV_RELATIONAL_EVAL(14U, 6U, 1, c3_y2, c3_y1, -1, 4U,
        c3_y2 > c3_y1))) {
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
  if (CV_SCRIPT_IF(6, 2, CV_RELATIONAL_EVAL(14U, 6U, 2, c3_xd, c3_yd, -1, 4U,
        c3_xd > c3_yd))) {
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
      if (CV_SCRIPT_COND(6, 0, CV_RELATIONAL_EVAL(14U, 6U, 3, c3_x, 0.0, -1, 2U,
            c3_x < 0.0))) {
        guard6 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(6, 1, CV_RELATIONAL_EVAL(14U, 6U, 4, c3_y, 0.0,
                   -1, 2U, c3_y < 0.0))) {
        guard6 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(6, 2, CV_RELATIONAL_EVAL(14U, 6U, 5, c3_x, 100.0,
        -1, 4U, c3_x > 100.0))) {
        guard5 = true;
        exitg2 = 1;
      } else if (CV_SCRIPT_COND(6, 3, CV_RELATIONAL_EVAL(14U, 6U, 6, c3_y, 100.0,
        -1, 4U, c3_y > 100.0))) {
        guard4 = true;
        exitg2 = 1;
      } else {
        CV_SCRIPT_MCDC(6, 0, false);
        CV_SCRIPT_IF(6, 3, false);
        _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 61);
        c3_d4 = c3_c_map[(sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 0U, 0U, 0U, c3_x), 1, 100) + 100 *
                          (sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 0U, 0U, 0U, c3_y), 1, 100) - 1)) - 1];
        if (CV_SCRIPT_IF(6, 4, CV_RELATIONAL_EVAL(14U, 6U, 7, c3_d4, 1.0, -1, 0U,
              c3_d4 == 1.0))) {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 62);
          c3_c_x[0] = c3_x;
          c3_c_x[1] = c3_y;
          c3_p_sizes[0] = 1;
          c3_p_sizes[1] = 2;
          c3_h_p = c3_p_sizes[0];
          c3_j_p = c3_p_sizes[1];
          for (c3_i130 = 0; c3_i130 < 2; c3_i130++) {
            c3_p_data[c3_i130] = c3_c_x[c3_i130];
          }

          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 63);
          exitg2 = 1;
        } else {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 65);
          c3_d_x[0] = c3_x - c3_x2;
          c3_d_x[1] = c3_y - c3_y2;
          for (c3_i128 = 0; c3_i128 < 2; c3_i128++) {
            c3_f_x[c3_i128] = (c3_d_x[c3_i128] == 0.0);
          }

          if (CV_SCRIPT_IF(6, 5, c3_b_all(chartInstance, c3_f_x))) {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 66);
            c3_p_sizes[0] = 1;
            c3_p_sizes[1] = 2;
            c3_l_p = c3_p_sizes[0];
            c3_n_p = c3_p_sizes[1];
            for (c3_i132 = 0; c3_i132 < 2; c3_i132++) {
              c3_p_data[c3_i132] = rtInf;
            }

            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 67);
            exitg2 = 1;
          } else {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 69);
            if (CV_SCRIPT_IF(6, 6, CV_RELATIONAL_EVAL(14U, 6U, 8, c3_b, 0.0, -1,
                  2U, c3_b < 0.0))) {
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
      c3_d_p = c3_p_sizes[0];
      c3_f_p = c3_p_sizes[1];
      for (c3_i126 = 0; c3_i126 < 2; c3_i126++) {
        c3_p_data[c3_i126] = rtInf;
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
      if (CV_SCRIPT_COND(6, 4, CV_RELATIONAL_EVAL(14U, 6U, 9, c3_x, 0.0, -1, 2U,
            c3_x < 0.0))) {
        guard3 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(6, 5, CV_RELATIONAL_EVAL(14U, 6U, 10, c3_y, 0.0,
        -1, 2U, c3_y < 0.0))) {
        guard3 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(6, 6, CV_RELATIONAL_EVAL(14U, 6U, 11, c3_x,
                   100.0, -1, 4U, c3_x > 100.0))) {
        guard2 = true;
        exitg1 = 1;
      } else if (CV_SCRIPT_COND(6, 7, CV_RELATIONAL_EVAL(14U, 6U, 12, c3_y,
                   100.0, -1, 4U, c3_y > 100.0))) {
        guard1 = true;
        exitg1 = 1;
      } else {
        CV_SCRIPT_MCDC(6, 1, false);
        CV_SCRIPT_IF(6, 7, false);
        _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 88);
        c3_d3 = c3_c_map[(sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 0U, 0U, 0U, c3_x), 1, 100) + 100 *
                          (sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 0U, 0, 0, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 0U, 0U, 0U, c3_y), 1, 100) - 1)) - 1];
        if (CV_SCRIPT_IF(6, 8, CV_RELATIONAL_EVAL(14U, 6U, 13, c3_d3, 1.0, -1,
              0U, c3_d3 == 1.0))) {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 89);
          c3_c_x[0] = c3_x;
          c3_c_x[1] = c3_y;
          c3_p_sizes[0] = 1;
          c3_p_sizes[1] = 2;
          c3_g_p = c3_p_sizes[0];
          c3_i_p = c3_p_sizes[1];
          for (c3_i129 = 0; c3_i129 < 2; c3_i129++) {
            c3_p_data[c3_i129] = c3_c_x[c3_i129];
          }

          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 90);
          exitg1 = 1;
        } else {
          _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 92);
          c3_b_x[0] = c3_x - c3_x2;
          c3_b_x[1] = c3_y - c3_y2;
          for (c3_i127 = 0; c3_i127 < 2; c3_i127++) {
            c3_e_x[c3_i127] = (c3_b_x[c3_i127] == 0.0);
          }

          if (CV_SCRIPT_IF(6, 9, c3_b_all(chartInstance, c3_e_x))) {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 93);
            c3_p_sizes[0] = 1;
            c3_p_sizes[1] = 2;
            c3_k_p = c3_p_sizes[0];
            c3_m_p = c3_p_sizes[1];
            for (c3_i131 = 0; c3_i131 < 2; c3_i131++) {
              c3_p_data[c3_i131] = rtInf;
            }

            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 94);
            exitg1 = 1;
          } else {
            _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 96);
            if (CV_SCRIPT_IF(6, 10, CV_RELATIONAL_EVAL(14U, 6U, 14, c3_b, 0.0,
                  -1, 2U, c3_b < 0.0))) {
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
      c3_c_p = c3_p_sizes[0];
      c3_e_p = c3_p_sizes[1];
      for (c3_i125 = 0; c3_i125 < 2; c3_i125++) {
        c3_p_data[c3_i125] = rtInf;
      }

      _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, 86);
    }
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c3_sfEvent, -101);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c3_deg2rad(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_deg)
{
  real_T c3_rad;
  uint32_T c3_debug_family_var_map[4];
  real_T c3_nargin = 1.0;
  real_T c3_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c3_j_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_deg, 2U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_rad, 3U, c3_d_sf_marshallOut,
    c3_d_sf_marshallIn);
  CV_SCRIPT_FCN(8, 0);
  _SFD_SCRIPT_CALL(8U, chartInstance->c3_sfEvent, 25);
  c3_rad = 3.1415926535897931;
  _SFD_SCRIPT_CALL(8U, chartInstance->c3_sfEvent, -25);
  _SFD_SYMBOL_SCOPE_POP();
  return c3_rad;
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 0U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/robot/se2.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 1U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/laserScanner.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 2U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/XYtoIJ.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 3U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/clipLine.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 4U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/intersectLines.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 5U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/linePosition.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 6U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/rvctools/common/laserRange.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 7U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/assignment6/IJtoXY.m"));
  _SFD_SCRIPT_TRANSLATION(c3_chartNumber, c3_instanceNumber, 8U,
    sf_debug_get_script_id(
    "/Users/michaelbrown/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/geom2d/geom2d/geom2d/deg2rad.m"));
}

static const mxArray *c3_emlrt_marshallOut
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, const char * c3_u)
{
  const mxArray *c3_y = NULL;
  (void)chartInstance;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c3_u)), false);
  return c3_y;
}

static void c3_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_load, const char_T *c3_identifier,
  c3_s2aqkGCuE38RBomNVWBcX1B *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
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
  c3_thisId.bParentIsCell = false;
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
  real_T c3_dv25[10000];
  int32_T c3_i133;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv25, 1, 0, 0U, 1, 0U, 2, 100,
                100);
  for (c3_i133 = 0; c3_i133 < 10000; c3_i133++) {
    c3_y[c3_i133] = c3_dv25[c3_i133];
  }

  sf_mex_destroy(&c3_u);
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  int32_T c3_i134;
  const mxArray *c3_y = NULL;
  real_T c3_u[361];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_mxArrayOutData = NULL;
  for (c3_i134 = 0; c3_i134 < 361; c3_i134++) {
    c3_u[c3_i134] = (*(real_T (*)[361])c3_inData)[c3_i134];
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
  boolean_T *c3_svPtr, real_T c3_y[361])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_theta), &c3_thisId,
                        c3_svPtr, c3_y);
  sf_mex_destroy(&c3_b_prev_theta);
}

static void c3_e_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr, real_T c3_y[361])
{
  real_T c3_dv26[361];
  int32_T c3_i135;
  (void)chartInstance;
  if (mxIsEmpty(c3_u)) {
    *c3_svPtr = false;
  } else {
    *c3_svPtr = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv26, 1, 0, 0U, 1, 0U, 1,
                  361);
    for (c3_i135 = 0; c3_i135 < 361; c3_i135++) {
      c3_y[c3_i135] = c3_dv26[c3_i135];
    }
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_prev_theta;
  const char_T *c3_identifier;
  boolean_T *c3_svPtr;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[361];
  int32_T c3_i136;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_prev_theta = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_svPtr = &chartInstance->c3_prev_theta_not_empty;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_theta), &c3_thisId,
                        c3_svPtr, c3_y);
  sf_mex_destroy(&c3_b_prev_theta);
  for (c3_i136 = 0; c3_i136 < 361; c3_i136++) {
    (*(real_T (*)[361])c3_outData)[c3_i136] = c3_y[c3_i136];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
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

static real_T c3_f_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_prev_t, const char_T *c3_identifier,
  boolean_T *c3_svPtr)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_y = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_t),
    &c3_thisId, c3_svPtr);
  sf_mex_destroy(&c3_b_prev_t);
  return c3_y;
}

static real_T c3_g_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr)
{
  real_T c3_y;
  real_T c3_d5;
  (void)chartInstance;
  if (mxIsEmpty(c3_u)) {
    *c3_svPtr = false;
  } else {
    *c3_svPtr = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d5, 1, 0, 0U, 0, 0U, 0);
    c3_y = c3_d5;
  }

  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_prev_t;
  const char_T *c3_identifier;
  boolean_T *c3_svPtr;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_prev_t = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_svPtr = &chartInstance->c3_prev_t_not_empty;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_y = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_prev_t),
    &c3_thisId, c3_svPtr);
  sf_mex_destroy(&c3_b_prev_t);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i137;
  const mxArray *c3_y = NULL;
  real_T c3_u[361];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i137 = 0; c3_i137 < 361; c3_i137++) {
    c3_u[c3_i137] = (*(real_T (*)[361])c3_inData)[c3_i137];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 361), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_h_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_theta, const char_T *c3_identifier, real_T
  c3_y[361])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_theta), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_theta);
}

static void c3_i_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[361])
{
  real_T c3_dv27[361];
  int32_T c3_i138;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv27, 1, 0, 0U, 1, 0U, 1, 361);
  for (c3_i138 = 0; c3_i138 < 361; c3_i138++) {
    c3_y[c3_i138] = c3_dv27[c3_i138];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_theta;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[361];
  int32_T c3_i139;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_b_theta = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_theta), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_theta);
  for (c3_i139 = 0; c3_i139 < 361; c3_i139++) {
    (*(real_T (*)[361])c3_outData)[c3_i139] = c3_y[c3_i139];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
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

static real_T c3_j_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d6;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d6, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d6;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c3_thisId.bParentIsCell = false;
  c3_y = c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i140;
  int32_T c3_i141;
  const mxArray *c3_y = NULL;
  int32_T c3_i142;
  real_T c3_u[10000];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i140 = 0;
  for (c3_i141 = 0; c3_i141 < 100; c3_i141++) {
    for (c3_i142 = 0; c3_i142 < 100; c3_i142++) {
      c3_u[c3_i142 + c3_i140] = (*(real_T (*)[10000])c3_inData)[c3_i142 +
        c3_i140];
    }

    c3_i140 += 100;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 100, 100),
                false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_c_map;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[10000];
  int32_T c3_i143;
  int32_T c3_i144;
  int32_T c3_i145;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_c_map = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_c_map), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_c_map);
  c3_i143 = 0;
  for (c3_i144 = 0; c3_i144 < 100; c3_i144++) {
    for (c3_i145 = 0; c3_i145 < 100; c3_i145++) {
      (*(real_T (*)[10000])c3_outData)[c3_i145 + c3_i143] = c3_y[c3_i145 +
        c3_i143];
    }

    c3_i143 += 100;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i146;
  int32_T c3_i147;
  const mxArray *c3_y = NULL;
  int32_T c3_i148;
  real_T c3_u[722];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i146 = 0;
  for (c3_i147 = 0; c3_i147 < 2; c3_i147++) {
    for (c3_i148 = 0; c3_i148 < 361; c3_i148++) {
      c3_u[c3_i148 + c3_i146] = (*(real_T (*)[722])c3_inData)[c3_i148 + c3_i146];
    }

    c3_i146 += 361;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 361, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_k_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[722])
{
  real_T c3_dv28[722];
  int32_T c3_i149;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv28, 1, 0, 0U, 1, 0U, 2, 361,
                2);
  for (c3_i149 = 0; c3_i149 < 722; c3_i149++) {
    c3_y[c3_i149] = c3_dv28[c3_i149];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_p;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[722];
  int32_T c3_i150;
  int32_T c3_i151;
  int32_T c3_i152;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_p = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_p), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_p);
  c3_i150 = 0;
  for (c3_i151 = 0; c3_i151 < 2; c3_i151++) {
    for (c3_i152 = 0; c3_i152 < 361; c3_i152++) {
      (*(real_T (*)[722])c3_outData)[c3_i152 + c3_i150] = c3_y[c3_i152 + c3_i150];
    }

    c3_i150 += 361;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_g_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i153;
  int32_T c3_i154;
  const mxArray *c3_y = NULL;
  int32_T c3_i155;
  real_T c3_u[9];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i153 = 0;
  for (c3_i154 = 0; c3_i154 < 3; c3_i154++) {
    for (c3_i155 = 0; c3_i155 < 3; c3_i155++) {
      c3_u[c3_i155 + c3_i153] = (*(real_T (*)[9])c3_inData)[c3_i155 + c3_i153];
    }

    c3_i153 += 3;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_l_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[9])
{
  real_T c3_dv29[9];
  int32_T c3_i156;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv29, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c3_i156 = 0; c3_i156 < 9; c3_i156++) {
    c3_y[c3_i156] = c3_dv29[c3_i156];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_Tl;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[9];
  int32_T c3_i157;
  int32_T c3_i158;
  int32_T c3_i159;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_Tl = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Tl), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_Tl);
  c3_i157 = 0;
  for (c3_i158 = 0; c3_i158 < 3; c3_i158++) {
    for (c3_i159 = 0; c3_i159 < 3; c3_i159++) {
      (*(real_T (*)[9])c3_outData)[c3_i159 + c3_i157] = c3_y[c3_i159 + c3_i157];
    }

    c3_i157 += 3;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_h_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  c3_s2aqkGCuE38RBomNVWBcX1B c3_u;
  const mxArray *c3_y = NULL;
  int32_T c3_i160;
  const mxArray *c3_b_y = NULL;
  real_T c3_b_u[10000];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_s2aqkGCuE38RBomNVWBcX1B *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c3_i160 = 0; c3_i160 < 10000; c3_i160++) {
    c3_b_u[c3_i160] = c3_u.map[c3_i160];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 0, 0U, 1U, 0U, 2, 100, 100),
                false);
  sf_mex_addfield(c3_y, c3_b_y, "map", "map", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c3_thisId.bParentIsCell = false;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_load), &c3_thisId, &c3_y);
  sf_mex_destroy(&c3_load);
  *(c3_s2aqkGCuE38RBomNVWBcX1B *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_i_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i161;
  const mxArray *c3_y = NULL;
  real_T c3_u[3];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i161 = 0; c3_i161 < 3; c3_i161++) {
    c3_u[c3_i161] = (*(real_T (*)[3])c3_inData)[c3_i161];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_m_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv30[3];
  int32_T c3_i162;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv30, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c3_i162 = 0; c3_i162 < 3; c3_i162++) {
    c3_y[c3_i162] = c3_dv30[c3_i162];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_a;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i163;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_a = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_a), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_a);
  for (c3_i163 = 0; c3_i163 < 3; c3_i163++) {
    (*(real_T (*)[3])c3_outData)[c3_i163] = c3_y[c3_i163];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_j_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i164;
  int32_T c3_i165;
  const mxArray *c3_y = NULL;
  int32_T c3_i166;
  real_T c3_u[4];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i164 = 0;
  for (c3_i165 = 0; c3_i165 < 2; c3_i165++) {
    for (c3_i166 = 0; c3_i166 < 2; c3_i166++) {
      c3_u[c3_i166 + c3_i164] = (*(real_T (*)[4])c3_inData)[c3_i166 + c3_i164];
    }

    c3_i164 += 2;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_n_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4])
{
  real_T c3_dv31[4];
  int32_T c3_i167;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv31, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c3_i167 = 0; c3_i167 < 4; c3_i167++) {
    c3_y[c3_i167] = c3_dv31[c3_i167];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_R;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[4];
  int32_T c3_i168;
  int32_T c3_i169;
  int32_T c3_i170;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_R = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_R), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_R);
  c3_i168 = 0;
  for (c3_i169 = 0; c3_i169 < 2; c3_i169++) {
    for (c3_i170 = 0; c3_i170 < 2; c3_i170++) {
      (*(real_T (*)[4])c3_outData)[c3_i170 + c3_i168] = c3_y[c3_i170 + c3_i168];
    }

    c3_i168 += 2;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_k_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i171;
  const mxArray *c3_y = NULL;
  real_T c3_u[2];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i171 = 0; c3_i171 < 2; c3_i171++) {
    c3_u[c3_i171] = (*(real_T (*)[2])c3_inData)[c3_i171];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_o_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[2])
{
  real_T c3_dv32[2];
  int32_T c3_i172;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv32, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c3_i172 = 0; c3_i172 < 2; c3_i172++) {
    c3_y[c3_i172] = c3_dv32[c3_i172];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_point;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[2];
  int32_T c3_i173;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_point = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_point), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_point);
  for (c3_i173 = 0; c3_i173 < 2; c3_i173++) {
    (*(real_T (*)[2])c3_outData)[c3_i173] = c3_y[c3_i173];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_l_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i174;
  const mxArray *c3_y = NULL;
  real_T c3_u[4];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i174 = 0; c3_i174 < 4; c3_i174++) {
    c3_u[c3_i174] = (*(real_T (*)[4])c3_inData)[c3_i174];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_p_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4])
{
  real_T c3_dv33[4];
  int32_T c3_i175;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv33, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c3_i175 = 0; c3_i175 < 4; c3_i175++) {
    c3_y[c3_i175] = c3_dv33[c3_i175];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_line2;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[4];
  int32_T c3_i176;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_line2 = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_line2), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_line2);
  for (c3_i176 = 0; c3_i176 < 4; c3_i176++) {
    (*(real_T (*)[4])c3_outData)[c3_i176] = c3_y[c3_i176];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_m_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u_sizes[2];
  int32_T c3_u;
  int32_T c3_b_u;
  int32_T c3_loop_ub;
  int32_T c3_i177;
  const mxArray *c3_y = NULL;
  real_T c3_u_data[1];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u_sizes[0] = 1;
  c3_u_sizes[1] = c3_inData_sizes[1];
  c3_u = c3_u_sizes[0];
  c3_b_u = c3_u_sizes[1];
  c3_loop_ub = c3_inData_sizes[0] * c3_inData_sizes[1] - 1;
  for (c3_i177 = 0; c3_i177 <= c3_loop_ub; c3_i177++) {
    c3_u_data[c3_i177] = c3_inData_data[c3_i177];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_q_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i178;
  int32_T c3_i179;
  uint32_T c3_uv0[2];
  real_T c3_tmp_data[1];
  boolean_T c3_bv0[2];
  int32_T c3_tmp_sizes[2];
  static boolean_T c3_bv1[2] = { false, true };

  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_loop_ub;
  int32_T c3_i180;
  (void)chartInstance;
  for (c3_i178 = 0; c3_i178 < 2; c3_i178++) {
    c3_uv0[c3_i178] = 1U;
  }

  for (c3_i179 = 0; c3_i179 < 2; c3_i179++) {
    c3_bv0[c3_i179] = c3_bv1[c3_i179];
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv0, c3_uv0, c3_tmp_sizes);
  c3_y_sizes[0] = 1;
  c3_y_sizes[1] = c3_tmp_sizes[1];
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i180 = 0; c3_i180 <= c3_loop_ub; c3_i180++) {
    c3_y_data[c3_i180] = c3_tmp_data[c3_i180];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_denom;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y_data[1];
  int32_T c3_y_sizes[2];
  int32_T c3_loop_ub;
  int32_T c3_i181;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_denom = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_denom), &c3_thisId,
                        c3_y_data, c3_y_sizes);
  sf_mex_destroy(&c3_denom);
  c3_outData_sizes[0] = 1;
  c3_outData_sizes[1] = c3_y_sizes[1];
  c3_loop_ub = c3_y_sizes[1] - 1;
  for (c3_i181 = 0; c3_i181 <= c3_loop_ub; c3_i181++) {
    c3_outData_data[c3_outData_sizes[0] * c3_i181] = c3_y_data[c3_y_sizes[0] *
      c3_i181];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_n_sf_marshallOut(void *chartInstanceVoid, void
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

static boolean_T c3_r_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  boolean_T c3_y;
  boolean_T c3_b7;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_b7, 1, 11, 0U, 0, 0U, 0);
  c3_y = c3_b7;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c3_thisId.bParentIsCell = false;
  c3_y = c3_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_inds), &c3_thisId);
  sf_mex_destroy(&c3_inds);
  *(boolean_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_o_sf_marshallOut(void *chartInstanceVoid, void
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

static const mxArray *c3_p_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T *c3_inData_sizes)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i182;
  const mxArray *c3_y = NULL;
  real_T c3_u_data[4];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u_sizes = *c3_inData_sizes;
  c3_loop_ub = *c3_inData_sizes - 1;
  for (c3_i182 = 0; c3_i182 <= c3_loop_ub; c3_i182++) {
    c3_u_data[c3_i182] = c3_inData_data[c3_i182];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 1,
    c3_u_sizes), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_s_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T *c3_y_sizes)
{
  uint32_T c3_uv1[1];
  boolean_T c3_bv2[1];
  real_T c3_tmp_data[4];
  int32_T c3_tmp_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i183;
  (void)chartInstance;
  c3_uv1[0] = 4U;
  c3_bv2[0] = true;
  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c3_bv2, c3_uv1, &c3_tmp_sizes);
  *c3_y_sizes = c3_tmp_sizes;
  c3_loop_ub = c3_tmp_sizes - 1;
  for (c3_i183 = 0; c3_i183 <= c3_loop_ub; c3_i183++) {
    c3_y_data[c3_i183] = c3_tmp_data[c3_i183];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  *c3_outData_sizes)
{
  const mxArray *c3_pos;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y_data[4];
  int32_T c3_y_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i184;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_pos = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_pos), &c3_thisId, c3_y_data,
                        &c3_y_sizes);
  sf_mex_destroy(&c3_pos);
  *c3_outData_sizes = c3_y_sizes;
  c3_loop_ub = c3_y_sizes - 1;
  for (c3_i184 = 0; c3_i184 <= c3_loop_ub; c3_i184++) {
    c3_outData_data[c3_i184] = c3_y_data[c3_i184];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_q_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u_sizes[2];
  int32_T c3_u;
  int32_T c3_b_u;
  int32_T c3_loop_ub;
  int32_T c3_i185;
  const mxArray *c3_y = NULL;
  real_T c3_u_data[8];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u_sizes[0] = c3_inData_sizes[0];
  c3_u_sizes[1] = 2;
  c3_u = c3_u_sizes[0];
  c3_b_u = c3_u_sizes[1];
  c3_loop_ub = c3_inData_sizes[0] * c3_inData_sizes[1] - 1;
  for (c3_i185 = 0; c3_i185 <= c3_loop_ub; c3_i185++) {
    c3_u_data[c3_i185] = c3_inData_data[c3_i185];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_t_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i186;
  int32_T c3_i187;
  uint32_T c3_uv2[2];
  real_T c3_tmp_data[8];
  boolean_T c3_bv3[2];
  int32_T c3_tmp_sizes[2];
  static boolean_T c3_bv4[2] = { true, false };

  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_loop_ub;
  int32_T c3_i188;
  (void)chartInstance;
  for (c3_i186 = 0; c3_i186 < 2; c3_i186++) {
    c3_uv2[c3_i186] = 4U + (uint32_T)(-2 * c3_i186);
  }

  for (c3_i187 = 0; c3_i187 < 2; c3_i187++) {
    c3_bv3[c3_i187] = c3_bv4[c3_i187];
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv3, c3_uv2, c3_tmp_sizes);
  c3_y_sizes[0] = c3_tmp_sizes[0];
  c3_y_sizes[1] = 2;
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i188 = 0; c3_i188 <= c3_loop_ub; c3_i188++) {
    c3_y_data[c3_i188] = c3_tmp_data[c3_i188];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_point;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y_data[8];
  int32_T c3_y_sizes[2];
  int32_T c3_i189;
  int32_T c3_loop_ub;
  int32_T c3_i190;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_point = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_point), &c3_thisId,
                        c3_y_data, c3_y_sizes);
  sf_mex_destroy(&c3_point);
  c3_outData_sizes[0] = c3_y_sizes[0];
  c3_outData_sizes[1] = 2;
  for (c3_i189 = 0; c3_i189 < 2; c3_i189++) {
    c3_loop_ub = c3_y_sizes[0] - 1;
    for (c3_i190 = 0; c3_i190 <= c3_loop_ub; c3_i190++) {
      c3_outData_data[c3_i190 + c3_outData_sizes[0] * c3_i189] =
        c3_y_data[c3_i190 + c3_y_sizes[0] * c3_i189];
    }
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_r_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i191;
  int32_T c3_i192;
  const mxArray *c3_y = NULL;
  int32_T c3_i193;
  real_T c3_u[8];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i191 = 0;
  for (c3_i192 = 0; c3_i192 < 2; c3_i192++) {
    for (c3_i193 = 0; c3_i193 < 4; c3_i193++) {
      c3_u[c3_i193 + c3_i191] = (*(real_T (*)[8])c3_inData)[c3_i193 + c3_i191];
    }

    c3_i191 += 4;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 4, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_u_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8])
{
  real_T c3_dv34[8];
  int32_T c3_i194;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv34, 1, 0, 0U, 1, 0U, 2, 4, 2);
  for (c3_i194 = 0; c3_i194 < 8; c3_i194++) {
    c3_y[c3_i194] = c3_dv34[c3_i194];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_points;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[8];
  int32_T c3_i195;
  int32_T c3_i196;
  int32_T c3_i197;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_points = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_points), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_points);
  c3_i195 = 0;
  for (c3_i196 = 0; c3_i196 < 2; c3_i196++) {
    for (c3_i197 = 0; c3_i197 < 4; c3_i197++) {
      (*(real_T (*)[8])c3_outData)[c3_i197 + c3_i195] = c3_y[c3_i197 + c3_i195];
    }

    c3_i195 += 4;
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_s_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u_sizes[2];
  int32_T c3_u;
  int32_T c3_b_u;
  int32_T c3_loop_ub;
  int32_T c3_i198;
  const mxArray *c3_y = NULL;
  real_T c3_u_data[2];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u_sizes[0] = c3_inData_sizes[0];
  c3_u_sizes[1] = c3_inData_sizes[1];
  c3_u = c3_u_sizes[0];
  c3_b_u = c3_u_sizes[1];
  c3_loop_ub = c3_inData_sizes[0] * c3_inData_sizes[1] - 1;
  for (c3_i198 = 0; c3_i198 <= c3_loop_ub; c3_i198++) {
    c3_u_data[c3_i198] = c3_inData_data[c3_i198];
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
  int32_T c3_i199;
  int32_T c3_i200;
  uint32_T c3_uv3[2];
  real_T c3_tmp_data[2];
  boolean_T c3_bv5[2];
  int32_T c3_tmp_sizes[2];
  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_loop_ub;
  int32_T c3_i201;
  (void)chartInstance;
  for (c3_i199 = 0; c3_i199 < 2; c3_i199++) {
    c3_uv3[c3_i199] = 1U + (uint32_T)c3_i199;
  }

  for (c3_i200 = 0; c3_i200 < 2; c3_i200++) {
    c3_bv5[c3_i200] = true;
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv5, c3_uv3, c3_tmp_sizes);
  c3_y_sizes[0] = c3_tmp_sizes[0];
  c3_y_sizes[1] = c3_tmp_sizes[1];
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  c3_loop_ub = c3_tmp_sizes[0] * c3_tmp_sizes[1] - 1;
  for (c3_i201 = 0; c3_i201 <= c3_loop_ub; c3_i201++) {
    c3_y_data[c3_i201] = c3_tmp_data[c3_i201];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_p;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y_data[2];
  int32_T c3_y_sizes[2];
  int32_T c3_loop_ub;
  int32_T c3_i202;
  int32_T c3_b_loop_ub;
  int32_T c3_i203;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_p = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_p), &c3_thisId, c3_y_data,
                        c3_y_sizes);
  sf_mex_destroy(&c3_p);
  c3_outData_sizes[0] = c3_y_sizes[0];
  c3_outData_sizes[1] = c3_y_sizes[1];
  c3_loop_ub = c3_y_sizes[1] - 1;
  for (c3_i202 = 0; c3_i202 <= c3_loop_ub; c3_i202++) {
    c3_b_loop_ub = c3_y_sizes[0] - 1;
    for (c3_i203 = 0; c3_i203 <= c3_b_loop_ub; c3_i203++) {
      c3_outData_data[c3_i203 + c3_outData_sizes[0] * c3_i202] =
        c3_y_data[c3_i203 + c3_y_sizes[0] * c3_i202];
    }
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_t_sf_marshallOut(void *chartInstanceVoid, real_T
  c3_inData_data[], int32_T c3_inData_sizes[2])
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u_sizes[2];
  int32_T c3_u;
  int32_T c3_b_u;
  int32_T c3_i204;
  const mxArray *c3_y = NULL;
  real_T c3_u_data[2];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  (void)c3_inData_sizes;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u_sizes[0] = 1;
  c3_u_sizes[1] = 2;
  c3_u = c3_u_sizes[0];
  c3_b_u = c3_u_sizes[1];
  for (c3_i204 = 0; c3_i204 < 2; c3_i204++) {
    c3_u_data[c3_i204] = c3_inData_data[c3_i204];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u_data, 0, 0U, 1U, 0U, 2,
    c3_u_sizes[0], c3_u_sizes[1]), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_w_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y_data[], int32_T c3_y_sizes[2])
{
  int32_T c3_i205;
  int32_T c3_i206;
  uint32_T c3_uv4[2];
  real_T c3_tmp_data[2];
  boolean_T c3_bv6[2];
  int32_T c3_tmp_sizes[2];
  int32_T c3_y;
  int32_T c3_b_y;
  int32_T c3_i207;
  (void)chartInstance;
  for (c3_i205 = 0; c3_i205 < 2; c3_i205++) {
    c3_uv4[c3_i205] = 1U + (uint32_T)c3_i205;
  }

  for (c3_i206 = 0; c3_i206 < 2; c3_i206++) {
    c3_bv6[c3_i206] = false;
  }

  sf_mex_import_vs(c3_parentId, sf_mex_dup(c3_u), c3_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c3_bv6, c3_uv4, c3_tmp_sizes);
  c3_y_sizes[0] = 1;
  c3_y_sizes[1] = 2;
  c3_y = c3_y_sizes[0];
  c3_b_y = c3_y_sizes[1];
  for (c3_i207 = 0; c3_i207 < 2; c3_i207++) {
    c3_y_data[c3_i207] = c3_tmp_data[c3_i207];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, real_T c3_outData_data[], int32_T
  c3_outData_sizes[2])
{
  const mxArray *c3_Pxel;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y_data[2];
  int32_T c3_y_sizes[2];
  int32_T c3_i208;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_Pxel = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_w_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_Pxel), &c3_thisId,
                        c3_y_data, c3_y_sizes);
  sf_mex_destroy(&c3_Pxel);
  c3_outData_sizes[0] = 1;
  c3_outData_sizes[1] = 2;
  for (c3_i208 = 0; c3_i208 < 2; c3_i208++) {
    c3_outData_data[c3_outData_sizes[0] * c3_i208] = c3_y_data[c3_y_sizes[0] *
      c3_i208];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_u_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i209;
  const mxArray *c3_y = NULL;
  real_T c3_u[3];
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i209 = 0; c3_i209 < 3; c3_i209++) {
    c3_u[c3_i209] = (*(real_T (*)[3])c3_inData)[c3_i209];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_x_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv35[3];
  int32_T c3_i210;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv35, 1, 0, 0U, 1, 0U, 1, 3);
  for (c3_i210 = 0; c3_i210 < 3; c3_i210++) {
    c3_y[c3_i210] = c3_dv35[c3_i210];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_P2;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i211;
  SFc3_AutoFollow_SimulationInstanceStruct *chartInstance;
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)chartInstanceVoid;
  c3_P2 = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_P2), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_P2);
  for (c3_i211 = 0; c3_i211 < 3; c3_i211++) {
    (*(real_T (*)[3])c3_outData)[c3_i211] = c3_y[c3_i211];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_AutoFollow_Simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  const char * c3_data[6] = {
    "789ced58dd6ed3301875d9e83604130fc0a45e72c3827a81b82c12203a7512b0216d42bb70938fccc23f95ed1676cb5370b94b9e0789f7c14ed2d6b32252ba26"
    "2dc8962cf7d43e89cfe7f3398951ab7f8c4cd937f5e92384daa6dd35f50ecacbdd02b74cdd2bdafcff6df4a0c0df4c8d05d7f055e79d1c3340d392084638e6fa",
    "f46a04488212740249d6f3895038250c06c2016f8801ecb5d33503b64b5eaad9951175415e321d68ae63dbd3312d531d0f17e4d9f15b37785b484137e33dafe0"
    "b5bdfbb5b3a88c8714f2fb7eafe083c7b7f8e3ab8be88302a92246e24b0c7428c5171ebd1be3e4c90961d98f588c34c8cecb2bb31e24ee1c8b0428e16907f3a4",
    "63c68c29d644f06840122c233989b5105445520c858e8cb64356c4c5d5d72e995fcb99dfde3cce17073f7fbdb8057fb62eebe63f76f8ad123e72da65c62febd7"
    "de1f7876fc7d8f6731c5c6342731e61c64597c96f1ef7505ffb3c7b778e5fec54a919433e0fa59e48acc5c1c7cdc848fafd1bfe78345fd7e76ae45ff6835f9b2",
    "11fbbd1ba75c5cd8ef51c893bfc9935d6f9e16c7948c06844313cf9591c71fd511a71404eb265e335579c842beac3f5f36c507bd8a79ee7bf3b49898cf26a920"
    "d6f6326a35cf971f15fc89c79f3416af9b6aed5613f227e4cfa2f953fa3d63d86f8522f64e65eb5147fe688faf1b8b97ab367f5087fc597ffe6ccafb5aaf629e",
    "f7bc795a9c5dea3de62994af471def6dcce3b33ae2353bcf8a0563e6dfb9d0f09d8342dedcf63ca07fa4c5d9f97f7a1e908b0b798236fbfcb7ca773b1ecfe204"
    "d2aec44923fbbcf0f81637f39e54a8340e0efeadd7bfbf017b8f019b", "" };

  c3_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(c3_data, 7232U, &c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static real_T c3_cos(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_b_cos(chartInstance, &c3_b_x);
  return c3_b_x;
}

static real_T c3_sin(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_b_sin(chartInstance, &c3_b_x);
  return c3_b_x;
}

static void c3_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_b_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_c_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_dimagree(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c3_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a, real_T c3_b)
{
  real_T c3_av;
  real_T c3_bv;
  real_T c3_cv;
  (void)chartInstance;
  c3_av = c3_a;
  c3_bv = c3_b;
  c3_cv = c3_av - c3_bv;
  return c3_cv;
}

static void c3_d_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_e_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c3_abs(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T c3_x)
{
  real_T c3_b_x;
  real_T c3_c_x;
  (void)chartInstance;
  c3_b_x = c3_x;
  c3_c_x = c3_b_x;
  return muDoubleScalarAbs(c3_c_x);
}

static boolean_T c3_all(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  boolean_T c3_x)
{
  boolean_T c3_b_x;
  (void)chartInstance;
  c3_b_x = c3_x;
  return c3_b_x;
}

static void c3_indexShapeCheck(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_rdivide(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x, real_T c3_y_data[], int32_T c3_y_sizes[2], real_T c3_z_data[],
  int32_T c3_z_sizes[2])
{
  real_T c3_b_x;
  int32_T c3_z;
  int32_T c3_b_z;
  int32_T c3_loop_ub;
  int32_T c3_i212;
  (void)chartInstance;
  c3_b_x = c3_x;
  c3_z_sizes[0] = 1;
  c3_z_sizes[1] = c3_y_sizes[1];
  c3_z = c3_z_sizes[0];
  c3_b_z = c3_z_sizes[1];
  c3_loop_ub = c3_y_sizes[0] * c3_y_sizes[1] - 1;
  for (c3_i212 = 0; c3_i212 <= c3_loop_ub; c3_i212++) {
    c3_z_data[c3_i212] = c3_b_x / c3_y_data[c3_i212];
  }
}

static void c3_bsxfun_compatible(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_no_dynamic_expansion(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_f_scalarEg(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c3_b_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes)
{
  int32_T c3_na1;
  int32_T c3_sak;
  int32_T c3_csz[2];
  int32_T c3_av_sizes;
  int32_T c3_nc1;
  int32_T c3_b_nc1;
  int32_T c3_i213;
  int32_T c3_ck;
  int32_T c3_b_ck;
  int32_T c3_b_na1;
  int32_T c3_b_b;
  int32_T c3_c_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  real_T c3_bv;
  int32_T c3_b_k;
  int32_T c3_cv_sizes;
  real_T c3_av_data[4];
  int32_T c3_loop_ub;
  int32_T c3_i214;
  int32_T c3_c_nc1;
  real_T c3_cv_data[4];
  int32_T c3_d_b;
  int32_T c3_e_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  c3_na1 = c3_a_sizes;
  c3_sak = c3_a_sizes;
  c3_csz[0] = c3_sak;
  *c3_c_sizes = c3_csz[0];
  if (*c3_c_sizes == 0) {
  } else {
    c3_csz[0] = c3_na1;
    c3_csz[1] = 1;
    c3_av_sizes = c3_csz[0];
    c3_nc1 = *c3_c_sizes;
    c3_b_nc1 = c3_nc1;
    c3_i213 = *c3_c_sizes - c3_nc1;
    for (c3_ck = 0; c3_ck <= c3_i213; c3_ck += c3_b_nc1) {
      c3_b_ck = c3_ck;
      c3_b_na1 = c3_na1;
      c3_b_b = c3_b_na1;
      c3_c_b = c3_b_b;
      c3_overflow = ((!(1 > c3_c_b)) && (c3_c_b > 2147483646));
      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_k = 1; c3_k <= c3_b_na1; c3_k++) {
        c3_b_k = c3_k - 1;
        c3_av_data[c3_b_k] = c3_a_data[c3_b_k];
      }

      c3_bv = c3_b;
      c3_cv_sizes = c3_av_sizes;
      c3_loop_ub = c3_av_sizes - 1;
      for (c3_i214 = 0; c3_i214 <= c3_loop_ub; c3_i214++) {
        c3_cv_data[c3_i214] = c3_av_data[c3_i214] - c3_bv;
      }

      c3_c_nc1 = c3_nc1;
      c3_d_b = c3_c_nc1;
      c3_e_b = c3_d_b;
      c3_b_overflow = ((!(1 > c3_e_b)) && (c3_e_b > 2147483646));
      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_c_k = 1; c3_c_k <= c3_c_nc1; c3_c_k++) {
        c3_b_k = c3_c_k - 1;
        c3_c_data[c3_b_ck + c3_b_k] = c3_cv_data[c3_b_k];
      }
    }
  }
}

static void c3_check_forloop_overflow_error
  (SFc3_AutoFollow_SimulationInstanceStruct *chartInstance, boolean_T
   c3_overflow)
{
  const mxArray *c3_y = NULL;
  static char_T c3_u[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  const mxArray *c3_b_y = NULL;
  static char_T c3_b_u[5] = { 'i', 'n', 't', '3', '2' };

  (void)chartInstance;
  (void)c3_overflow;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 34), false);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
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
  int32_T c3_sak;
  int32_T c3_csz[2];
  int32_T c3_av_sizes;
  int32_T c3_nc1;
  int32_T c3_b_nc1;
  int32_T c3_i215;
  int32_T c3_ck;
  int32_T c3_b_ck;
  int32_T c3_b_na1;
  int32_T c3_b_b;
  int32_T c3_c_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  real_T c3_bv;
  int32_T c3_b_k;
  int32_T c3_cv_sizes;
  real_T c3_av_data[4];
  int32_T c3_loop_ub;
  int32_T c3_i216;
  int32_T c3_c_nc1;
  real_T c3_cv_data[4];
  int32_T c3_d_b;
  int32_T c3_e_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  c3_na1 = c3_a_sizes;
  c3_sak = c3_a_sizes;
  c3_csz[0] = c3_sak;
  *c3_c_sizes = c3_csz[0];
  if (*c3_c_sizes == 0) {
  } else {
    c3_csz[0] = c3_na1;
    c3_csz[1] = 1;
    c3_av_sizes = c3_csz[0];
    c3_nc1 = *c3_c_sizes;
    c3_b_nc1 = c3_nc1;
    c3_i215 = *c3_c_sizes - c3_nc1;
    for (c3_ck = 0; c3_ck <= c3_i215; c3_ck += c3_b_nc1) {
      c3_b_ck = c3_ck;
      c3_b_na1 = c3_na1;
      c3_b_b = c3_b_na1;
      c3_c_b = c3_b_b;
      c3_overflow = ((!(1 > c3_c_b)) && (c3_c_b > 2147483646));
      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_k = 1; c3_k <= c3_b_na1; c3_k++) {
        c3_b_k = c3_k - 1;
        c3_av_data[c3_b_k] = c3_a_data[c3_b_k];
      }

      c3_bv = c3_b;
      c3_cv_sizes = c3_av_sizes;
      c3_loop_ub = c3_av_sizes - 1;
      for (c3_i216 = 0; c3_i216 <= c3_loop_ub; c3_i216++) {
        c3_cv_data[c3_i216] = c3_av_data[c3_i216] * c3_bv;
      }

      c3_c_nc1 = c3_nc1;
      c3_d_b = c3_c_nc1;
      c3_e_b = c3_d_b;
      c3_b_overflow = ((!(1 > c3_e_b)) && (c3_e_b > 2147483646));
      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_c_k = 1; c3_c_k <= c3_c_nc1; c3_c_k++) {
        c3_b_k = c3_c_k - 1;
        c3_c_data[c3_b_ck + c3_b_k] = c3_cv_data[c3_b_k];
      }
    }
  }
}

static void c3_d_bsxfun(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_a_data[], int32_T c3_a_sizes, real_T c3_b, real_T c3_c_data[],
  int32_T *c3_c_sizes)
{
  int32_T c3_na1;
  int32_T c3_sak;
  int32_T c3_csz[2];
  int32_T c3_av_sizes;
  int32_T c3_nc1;
  int32_T c3_b_nc1;
  int32_T c3_i217;
  int32_T c3_ck;
  int32_T c3_b_ck;
  int32_T c3_b_na1;
  int32_T c3_b_b;
  int32_T c3_c_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  real_T c3_bv;
  int32_T c3_b_k;
  real_T c3_y;
  real_T c3_av_data[4];
  real_T c3_b_y;
  int32_T c3_cv_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i218;
  int32_T c3_c_nc1;
  real_T c3_cv_data[4];
  int32_T c3_d_b;
  int32_T c3_e_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_k;
  c3_na1 = c3_a_sizes;
  c3_sak = c3_a_sizes;
  c3_csz[0] = c3_sak;
  *c3_c_sizes = c3_csz[0];
  if (*c3_c_sizes == 0) {
  } else {
    c3_csz[0] = c3_na1;
    c3_csz[1] = 1;
    c3_av_sizes = c3_csz[0];
    c3_nc1 = *c3_c_sizes;
    c3_b_nc1 = c3_nc1;
    c3_i217 = *c3_c_sizes - c3_nc1;
    for (c3_ck = 0; c3_ck <= c3_i217; c3_ck += c3_b_nc1) {
      c3_b_ck = c3_ck - 1;
      c3_b_na1 = c3_na1;
      c3_b_b = c3_b_na1;
      c3_c_b = c3_b_b;
      c3_overflow = ((!(1 > c3_c_b)) && (c3_c_b > 2147483646));
      if (c3_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_k = 1; c3_k <= c3_b_na1; c3_k++) {
        c3_b_k = c3_k - 1;
        c3_av_data[c3_b_k] = c3_a_data[c3_b_k];
      }

      c3_bv = c3_b;
      c3_y = c3_bv;
      c3_b_y = c3_y;
      c3_cv_sizes = c3_av_sizes;
      c3_loop_ub = c3_av_sizes - 1;
      for (c3_i218 = 0; c3_i218 <= c3_loop_ub; c3_i218++) {
        c3_cv_data[c3_i218] = c3_av_data[c3_i218] / c3_b_y;
      }

      c3_c_nc1 = c3_nc1;
      c3_d_b = c3_c_nc1;
      c3_e_b = c3_d_b;
      c3_b_overflow = ((!(1 > c3_e_b)) && (c3_e_b > 2147483646));
      if (c3_b_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_c_k = 1; c3_c_k <= c3_c_nc1; c3_c_k++) {
        c3_b_k = c3_c_k;
        c3_c_data[c3_b_ck + c3_b_k] = c3_cv_data[c3_b_k - 1];
      }
    }
  }
}

static void c3_sort(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                    real_T c3_x_data[], int32_T c3_x_sizes, real_T c3_b_x_data[],
                    int32_T *c3_b_x_sizes, int32_T c3_idx_data[], int32_T
                    *c3_idx_sizes)
{
  int32_T c3_loop_ub;
  int32_T c3_i219;
  *c3_b_x_sizes = c3_x_sizes;
  c3_loop_ub = c3_x_sizes - 1;
  for (c3_i219 = 0; c3_i219 <= c3_loop_ub; c3_i219++) {
    c3_b_x_data[c3_i219] = c3_x_data[c3_i219];
  }

  c3_b_sort(chartInstance, c3_b_x_data, c3_b_x_sizes, c3_idx_data, c3_idx_sizes);
}

static int32_T c3_nonSingletonDim(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, real_T c3_x_data[], int32_T c3_x_sizes)
{
  int32_T c3_dim;
  (void)chartInstance;
  (void)c3_x_data;
  c3_dim = 2;
  if ((real_T)c3_x_sizes != 1.0) {
    c3_dim = 1;
  }

  return c3_dim;
}

static void c3_sortIdx(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x_data[], int32_T c3_x_sizes, int32_T c3_idx_data[], int32_T
  *c3_idx_sizes, real_T c3_b_x_data[], int32_T *c3_b_x_sizes)
{
  int32_T c3_loop_ub;
  int32_T c3_i220;
  *c3_b_x_sizes = c3_x_sizes;
  c3_loop_ub = c3_x_sizes - 1;
  for (c3_i220 = 0; c3_i220 <= c3_loop_ub; c3_i220++) {
    c3_b_x_data[c3_i220] = c3_x_data[c3_i220];
  }

  c3_b_sortIdx(chartInstance, c3_b_x_data, c3_b_x_sizes, c3_idx_data,
               c3_idx_sizes);
}

static void c3_merge_block(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, int32_T c3_idx_data[], int32_T c3_idx_sizes, real_T c3_x_data[],
  int32_T c3_x_sizes, int32_T c3_offset, int32_T c3_n, int32_T c3_preSortLevel,
  int32_T c3_iwork_data[], int32_T c3_iwork_sizes, real_T c3_xwork_data[],
  int32_T c3_xwork_sizes, int32_T c3_b_idx_data[], int32_T *c3_b_idx_sizes,
  real_T c3_b_x_data[], int32_T *c3_b_x_sizes, int32_T c3_b_iwork_data[],
  int32_T *c3_b_iwork_sizes, real_T c3_b_xwork_data[], int32_T *c3_b_xwork_sizes)
{
  int32_T c3_loop_ub;
  int32_T c3_i221;
  int32_T c3_b_loop_ub;
  int32_T c3_i222;
  int32_T c3_c_loop_ub;
  int32_T c3_i223;
  int32_T c3_d_loop_ub;
  int32_T c3_i224;
  *c3_b_idx_sizes = c3_idx_sizes;
  c3_loop_ub = c3_idx_sizes - 1;
  for (c3_i221 = 0; c3_i221 <= c3_loop_ub; c3_i221++) {
    c3_b_idx_data[c3_i221] = c3_idx_data[c3_i221];
  }

  *c3_b_x_sizes = c3_x_sizes;
  c3_b_loop_ub = c3_x_sizes - 1;
  for (c3_i222 = 0; c3_i222 <= c3_b_loop_ub; c3_i222++) {
    c3_b_x_data[c3_i222] = c3_x_data[c3_i222];
  }

  *c3_b_iwork_sizes = c3_iwork_sizes;
  c3_c_loop_ub = c3_iwork_sizes - 1;
  for (c3_i223 = 0; c3_i223 <= c3_c_loop_ub; c3_i223++) {
    c3_b_iwork_data[c3_i223] = c3_iwork_data[c3_i223];
  }

  *c3_b_xwork_sizes = c3_xwork_sizes;
  c3_d_loop_ub = c3_xwork_sizes - 1;
  for (c3_i224 = 0; c3_i224 <= c3_d_loop_ub; c3_i224++) {
    c3_b_xwork_data[c3_i224] = c3_xwork_data[c3_i224];
  }

  c3_b_merge_block(chartInstance, c3_b_idx_data, c3_b_idx_sizes, c3_b_x_data,
                   c3_b_x_sizes, c3_offset, c3_n, c3_preSortLevel,
                   c3_b_iwork_data, c3_b_iwork_sizes, c3_b_xwork_data,
                   c3_b_xwork_sizes);
}

static void c3_merge(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     int32_T c3_idx_data[], int32_T c3_idx_sizes, real_T
                     c3_x_data[], int32_T c3_x_sizes, int32_T c3_offset, int32_T
                     c3_np, int32_T c3_nq, int32_T c3_iwork_data[], int32_T
                     c3_iwork_sizes, real_T c3_xwork_data[], int32_T
                     c3_xwork_sizes, int32_T c3_b_idx_data[], int32_T
                     *c3_b_idx_sizes, real_T c3_b_x_data[], int32_T
                     *c3_b_x_sizes, int32_T c3_b_iwork_data[], int32_T
                     *c3_b_iwork_sizes, real_T c3_b_xwork_data[], int32_T
                     *c3_b_xwork_sizes)
{
  int32_T c3_loop_ub;
  int32_T c3_i225;
  int32_T c3_b_loop_ub;
  int32_T c3_i226;
  int32_T c3_c_loop_ub;
  int32_T c3_i227;
  int32_T c3_d_loop_ub;
  int32_T c3_i228;
  *c3_b_idx_sizes = c3_idx_sizes;
  c3_loop_ub = c3_idx_sizes - 1;
  for (c3_i225 = 0; c3_i225 <= c3_loop_ub; c3_i225++) {
    c3_b_idx_data[c3_i225] = c3_idx_data[c3_i225];
  }

  *c3_b_x_sizes = c3_x_sizes;
  c3_b_loop_ub = c3_x_sizes - 1;
  for (c3_i226 = 0; c3_i226 <= c3_b_loop_ub; c3_i226++) {
    c3_b_x_data[c3_i226] = c3_x_data[c3_i226];
  }

  *c3_b_iwork_sizes = c3_iwork_sizes;
  c3_c_loop_ub = c3_iwork_sizes - 1;
  for (c3_i227 = 0; c3_i227 <= c3_c_loop_ub; c3_i227++) {
    c3_b_iwork_data[c3_i227] = c3_iwork_data[c3_i227];
  }

  *c3_b_xwork_sizes = c3_xwork_sizes;
  c3_d_loop_ub = c3_xwork_sizes - 1;
  for (c3_i228 = 0; c3_i228 <= c3_d_loop_ub; c3_i228++) {
    c3_b_xwork_data[c3_i228] = c3_xwork_data[c3_i228];
  }

  c3_b_merge(chartInstance, c3_b_idx_data, c3_b_idx_sizes, c3_b_x_data,
             c3_b_x_sizes, c3_offset, c3_np, c3_nq, c3_b_iwork_data,
             c3_b_iwork_sizes, c3_b_xwork_data, c3_b_xwork_sizes);
}

static real_T c3_mean(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x[2])
{
  real_T c3_b_y;
  real_T c3_A;
  real_T c3_b_x;
  real_T c3_c_x;
  (void)chartInstance;
  c3_b_y = c3_x[0];
  c3_b_y += c3_x[1];
  c3_A = c3_b_y;
  c3_b_x = c3_A;
  c3_c_x = c3_b_x;
  return c3_c_x / 2.0;
}

static boolean_T c3_b_all(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, boolean_T c3_x[2])
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
    if (!c3_x[(int32_T)c3_b_k - 1]) {
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
  real_T c3_c;
  real_T c3_b_a;
  real_T c3_c_a;
  real_T c3_x;
  real_T c3_d_a;
  boolean_T c3_p;
  c3_b_a = c3_a;
  c3_c_a = c3_b_a;
  c3_x = c3_c_a;
  c3_d_a = c3_x;
  c3_c = c3_d_a * c3_d_a;
  c3_p = false;
  if (c3_p) {
    c3_error(chartInstance);
  }

  return c3_c;
}

static void c3_error(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c3_y = NULL;
  static char_T c3_u[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 31), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c3_y));
}

static void c3_b_error(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance)
{
  const mxArray *c3_y = NULL;
  static char_T c3_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c3_b_y = NULL;
  static char_T c3_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c3_y, 14, c3_b_y));
}

static const mxArray *c3_v_sf_marshallOut(void *chartInstanceVoid, void
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

static int32_T c3_y_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i229;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i229, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i229;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c3_thisId.bParentIsCell = false;
  c3_y = c3_y_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_ab_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_AutoFollow_Simulation, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_y = c3_bb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_AutoFollow_Simulation), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_AutoFollow_Simulation);
  return c3_y;
}

static uint8_T c3_bb_emlrt_marshallIn(SFc3_AutoFollow_SimulationInstanceStruct
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

static void c3_b_cos(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c3_x)
{
  (void)chartInstance;
  *c3_x = muDoubleScalarCos(*c3_x);
}

static void c3_b_sin(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                     real_T *c3_x)
{
  (void)chartInstance;
  *c3_x = muDoubleScalarSin(*c3_x);
}

static void c3_b_sort(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
                      real_T c3_x_data[], int32_T *c3_x_sizes, int32_T
                      c3_idx_data[], int32_T *c3_idx_sizes)
{
  int32_T c3_b_x_sizes;
  int32_T c3_loop_ub;
  int32_T c3_i230;
  int32_T c3_dim;
  real_T c3_b_x_data[4];
  int32_T c3_b_dim;
  int32_T c3_x;
  int32_T c3_b_x;
  boolean_T c3_b8;
  const mxArray *c3_y = NULL;
  static char_T c3_u[53] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'g', 'e', 't', 'd', 'i', 'm', 'a', 'r', 'g', '_', 'd', 'i',
    'm', 'e', 'n', 's', 'i', 'o', 'n', 'M', 'u', 's', 't', 'B', 'e', 'P', 'o',
    's', 'i', 't', 'i', 'v', 'e', 'I', 'n', 't', 'e', 'g', 'e', 'r' };

  int32_T c3_i231;
  real_T c3_d7;
  int32_T c3_vlen;
  int32_T c3_iv3[2];
  int32_T c3_vwork_sizes;
  real_T c3_c_x[2];
  int32_T c3_i232;
  real_T c3_dv36[2];
  int32_T c3_c_dim;
  int32_T c3_vstride;
  int32_T c3_i233;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_i;
  real_T c3_d8;
  int32_T c3_b_vstride;
  int32_T c3_c_b;
  int32_T c3_d_b;
  boolean_T c3_b_overflow;
  int32_T c3_j;
  int32_T c3_b_j;
  int32_T c3_idx0;
  int32_T c3_b_vlen;
  int32_T c3_e_b;
  int32_T c3_f_b;
  boolean_T c3_c_overflow;
  int32_T c3_b_k;
  real_T c3_vwork_data[4];
  int32_T c3_iidx_data[4];
  int32_T c3_iidx_sizes;
  int32_T c3_c_k;
  int32_T c3_c_vlen;
  int32_T c3_g_b;
  int32_T c3_h_b;
  boolean_T c3_d_overflow;
  int32_T c3_d_k;
  c3_b_x_sizes = *c3_x_sizes;
  c3_loop_ub = *c3_x_sizes - 1;
  for (c3_i230 = 0; c3_i230 <= c3_loop_ub; c3_i230++) {
    c3_b_x_data[c3_i230] = c3_x_data[c3_i230];
  }

  c3_dim = c3_nonSingletonDim(chartInstance, c3_b_x_data, c3_b_x_sizes);
  c3_b_dim = c3_dim;
  c3_x = c3_b_dim;
  c3_b_x = c3_x;
  c3_b8 = (c3_b_dim == c3_b_x);
  if (c3_b8) {
  } else {
    c3_y = NULL;
    sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 53),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c3_y));
  }

  c3_i231 = c3_dim;
  if (c3_i231 <= 1) {
    c3_d7 = (real_T)*c3_x_sizes;
  } else {
    c3_d7 = 1.0;
  }

  c3_vlen = (int32_T)c3_d7;
  c3_iv3[0] = c3_vlen;
  c3_iv3[1] = 1;
  c3_vwork_sizes = c3_iv3[0];
  c3_c_x[0] = (real_T)*c3_x_sizes;
  c3_c_x[1] = 1.0;
  for (c3_i232 = 0; c3_i232 < 2; c3_i232++) {
    c3_dv36[c3_i232] = c3_c_x[c3_i232];
  }

  *c3_idx_sizes = (int32_T)c3_dv36[0];
  c3_c_dim = c3_dim - 1;
  c3_vstride = 1;
  c3_i233 = c3_c_dim;
  c3_b = c3_i233;
  c3_b_b = c3_b;
  c3_overflow = ((!(1 > c3_b_b)) && (c3_b_b > 2147483646));
  if (c3_overflow) {
    c3_check_forloop_overflow_error(chartInstance, true);
  }

  c3_k = 1;
  while (c3_k <= c3_i233) {
    c3_d8 = (real_T)*c3_x_sizes;
    c3_vstride *= (int32_T)c3_d8;
    c3_k = 2;
  }

  c3_i = 1;
  while (c3_i <= 1) {
    c3_b_vstride = c3_vstride;
    c3_c_b = c3_b_vstride;
    c3_d_b = c3_c_b;
    c3_b_overflow = ((!(1 > c3_d_b)) && (c3_d_b > 2147483646));
    if (c3_b_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_j = 1; c3_j <= c3_b_vstride; c3_j++) {
      c3_b_j = c3_j;
      c3_idx0 = c3_b_j - 1;
      c3_b_vlen = c3_vlen;
      c3_e_b = c3_b_vlen;
      c3_f_b = c3_e_b;
      c3_c_overflow = ((!(1 > c3_f_b)) && (c3_f_b > 2147483646));
      if (c3_c_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_b_k = 1; c3_b_k <= c3_b_vlen; c3_b_k++) {
        c3_c_k = c3_b_k - 1;
        c3_vwork_data[c3_c_k] = c3_x_data[c3_idx0 + c3_c_k * c3_vstride];
      }

      c3_b_sortIdx(chartInstance, c3_vwork_data, &c3_vwork_sizes, c3_iidx_data,
                   &c3_iidx_sizes);
      c3_c_vlen = c3_vlen;
      c3_g_b = c3_c_vlen;
      c3_h_b = c3_g_b;
      c3_d_overflow = ((!(1 > c3_h_b)) && (c3_h_b > 2147483646));
      if (c3_d_overflow) {
        c3_check_forloop_overflow_error(chartInstance, true);
      }

      for (c3_d_k = 1; c3_d_k <= c3_c_vlen; c3_d_k++) {
        c3_c_k = c3_d_k - 1;
        c3_x_data[c3_idx0 + c3_c_k * c3_vstride] = c3_vwork_data[c3_c_k];
        c3_idx_data[c3_idx0 + c3_c_k * c3_vstride] = c3_iidx_data[c3_c_k];
      }
    }

    c3_i = 2;
  }
}

static void c3_b_sortIdx(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  real_T c3_x_data[], int32_T *c3_x_sizes, int32_T c3_idx_data[], int32_T
  *c3_idx_sizes)
{
  real_T c3_x[2];
  int32_T c3_i234;
  real_T c3_dv37[2];
  int32_T c3_loop_ub;
  int32_T c3_i235;
  int32_T c3_n;
  int32_T c3_b_n;
  int32_T c3_i236;
  int32_T c3_i237;
  real_T c3_x4[4];
  int32_T c3_iwork_sizes;
  int32_T c3_idx4[4];
  int32_T c3_iwork;
  int32_T c3_b_loop_ub;
  int32_T c3_i238;
  int32_T c3_iwork_data[4];
  int32_T c3_xwork_sizes;
  int32_T c3_xwork;
  int32_T c3_c_loop_ub;
  int32_T c3_i239;
  int32_T c3_nNaNs;
  real_T c3_xwork_data[4];
  int32_T c3_ib;
  int32_T c3_c_n;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_wOffset;
  int32_T c3_b_k;
  int32_T c3_tOffset;
  real_T c3_b_x;
  boolean_T c3_c_b;
  int32_T c3_d_n;
  int32_T c3_m;
  int32_T c3_i240;
  int32_T c3_b_m;
  int32_T c3_d_b;
  int32_T c3_e_b;
  int32_T c3_perm[4];
  boolean_T c3_b_overflow;
  int32_T c3_quartetOffset;
  int32_T c3_b_ib;
  int32_T c3_i1;
  int32_T c3_c_k;
  int32_T c3_f_b;
  int32_T c3_i2;
  int32_T c3_g_b;
  boolean_T c3_c_overflow;
  int32_T c3_i3;
  int32_T c3_itmp;
  int32_T c3_i4;
  int32_T c3_b_nNaNs;
  int32_T c3_d_k;
  int32_T c3_nNonNaN;
  c3_x[0] = (real_T)*c3_x_sizes;
  c3_x[1] = 1.0;
  for (c3_i234 = 0; c3_i234 < 2; c3_i234++) {
    c3_dv37[c3_i234] = c3_x[c3_i234];
  }

  *c3_idx_sizes = (int32_T)c3_dv37[0];
  c3_loop_ub = (int32_T)c3_dv37[0] - 1;
  for (c3_i235 = 0; c3_i235 <= c3_loop_ub; c3_i235++) {
    c3_idx_data[c3_i235] = 0;
  }

  c3_n = *c3_x_sizes;
  c3_b_n = *c3_x_sizes;
  for (c3_i236 = 0; c3_i236 < 4; c3_i236++) {
    c3_x4[c3_i236] = 0.0;
  }

  for (c3_i237 = 0; c3_i237 < 4; c3_i237++) {
    c3_idx4[c3_i237] = 0;
  }

  c3_iwork_sizes = *c3_idx_sizes;
  c3_iwork = c3_iwork_sizes;
  c3_iwork_sizes = c3_iwork;
  c3_b_loop_ub = c3_iwork - 1;
  for (c3_i238 = 0; c3_i238 <= c3_b_loop_ub; c3_i238++) {
    c3_iwork_data[c3_i238] = 0;
  }

  c3_dv37[0] = (real_T)*c3_x_sizes;
  c3_dv37[1] = 1.0;
  c3_xwork_sizes = (int32_T)c3_dv37[0];
  c3_xwork = c3_xwork_sizes;
  c3_xwork_sizes = c3_xwork;
  c3_c_loop_ub = c3_xwork - 1;
  for (c3_i239 = 0; c3_i239 <= c3_c_loop_ub; c3_i239++) {
    c3_xwork_data[c3_i239] = 0.0;
  }

  c3_nNaNs = 0;
  c3_ib = 0;
  c3_c_n = c3_b_n;
  c3_b = c3_c_n;
  c3_b_b = c3_b;
  c3_overflow = ((!(1 > c3_b_b)) && (c3_b_b > 2147483646));
  if (c3_overflow) {
    c3_check_forloop_overflow_error(chartInstance, true);
  }

  for (c3_k = 1; c3_k <= c3_c_n; c3_k++) {
    c3_b_k = c3_k - 1;
    c3_b_x = c3_x_data[c3_b_k];
    c3_c_b = muDoubleScalarIsNaN(c3_b_x);
    if (c3_c_b) {
      c3_idx_data[(c3_b_n - c3_nNaNs) - 1] = c3_b_k + 1;
      c3_xwork_data[(c3_b_n - c3_nNaNs) - 1] = c3_x_data[c3_b_k];
      c3_nNaNs++;
    } else {
      c3_ib++;
      c3_idx4[c3_ib - 1] = c3_b_k + 1;
      c3_x4[c3_ib - 1] = c3_x_data[c3_b_k];
      if (c3_ib == 4) {
        c3_quartetOffset = c3_b_k - c3_nNaNs;
        if (c3_x4[0] <= c3_x4[1]) {
          c3_i1 = 1;
          c3_i2 = 2;
        } else {
          c3_i1 = 2;
          c3_i2 = 1;
        }

        if (c3_x4[2] <= c3_x4[3]) {
          c3_i3 = 3;
          c3_i4 = 4;
        } else {
          c3_i3 = 4;
          c3_i4 = 3;
        }

        if (c3_x4[c3_i1 - 1] <= c3_x4[c3_i3 - 1]) {
          if (c3_x4[c3_i2 - 1] <= c3_x4[c3_i3 - 1]) {
            c3_perm[0] = c3_i1;
            c3_perm[1] = c3_i2;
            c3_perm[2] = c3_i3;
            c3_perm[3] = c3_i4;
          } else if (c3_x4[c3_i2 - 1] <= c3_x4[c3_i4 - 1]) {
            c3_perm[0] = c3_i1;
            c3_perm[1] = c3_i3;
            c3_perm[2] = c3_i2;
            c3_perm[3] = c3_i4;
          } else {
            c3_perm[0] = c3_i1;
            c3_perm[1] = c3_i3;
            c3_perm[2] = c3_i4;
            c3_perm[3] = c3_i2;
          }
        } else if (c3_x4[c3_i1 - 1] <= c3_x4[c3_i4 - 1]) {
          if (c3_x4[c3_i2 - 1] <= c3_x4[c3_i4 - 1]) {
            c3_perm[0] = c3_i3;
            c3_perm[1] = c3_i1;
            c3_perm[2] = c3_i2;
            c3_perm[3] = c3_i4;
          } else {
            c3_perm[0] = c3_i3;
            c3_perm[1] = c3_i1;
            c3_perm[2] = c3_i4;
            c3_perm[3] = c3_i2;
          }
        } else {
          c3_perm[0] = c3_i3;
          c3_perm[1] = c3_i4;
          c3_perm[2] = c3_i1;
          c3_perm[3] = c3_i2;
        }

        c3_idx_data[c3_quartetOffset - 3] = c3_idx4[c3_perm[0] - 1];
        c3_idx_data[c3_quartetOffset - 2] = c3_idx4[c3_perm[1] - 1];
        c3_idx_data[c3_quartetOffset - 1] = c3_idx4[c3_perm[2] - 1];
        c3_idx_data[c3_quartetOffset] = c3_idx4[c3_perm[3] - 1];
        c3_x_data[c3_quartetOffset - 3] = c3_x4[c3_perm[0] - 1];
        c3_x_data[c3_quartetOffset - 2] = c3_x4[c3_perm[1] - 1];
        c3_x_data[c3_quartetOffset - 1] = c3_x4[c3_perm[2] - 1];
        c3_x_data[c3_quartetOffset] = c3_x4[c3_perm[3] - 1];
        c3_ib = 0;
      }
    }
  }

  c3_wOffset = c3_b_n - c3_nNaNs;
  c3_tOffset = c3_wOffset - 1;
  if (c3_ib > 0) {
    c3_d_n = c3_ib;
    for (c3_i240 = 0; c3_i240 < 4; c3_i240++) {
      c3_perm[c3_i240] = 0;
    }

    if (c3_d_n == 1) {
      c3_perm[0] = 1;
    } else if (c3_d_n == 2) {
      if (c3_x4[0] <= c3_x4[1]) {
        c3_perm[0] = 1;
        c3_perm[1] = 2;
      } else {
        c3_perm[0] = 2;
        c3_perm[1] = 1;
      }
    } else if (c3_x4[0] <= c3_x4[1]) {
      if (c3_x4[1] <= c3_x4[2]) {
        c3_perm[0] = 1;
        c3_perm[1] = 2;
        c3_perm[2] = 3;
      } else if (c3_x4[0] <= c3_x4[2]) {
        c3_perm[0] = 1;
        c3_perm[1] = 3;
        c3_perm[2] = 2;
      } else {
        c3_perm[0] = 3;
        c3_perm[1] = 1;
        c3_perm[2] = 2;
      }
    } else if (c3_x4[0] <= c3_x4[2]) {
      c3_perm[0] = 2;
      c3_perm[1] = 1;
      c3_perm[2] = 3;
    } else if (c3_x4[1] <= c3_x4[2]) {
      c3_perm[0] = 2;
      c3_perm[1] = 3;
      c3_perm[2] = 1;
    } else {
      c3_perm[0] = 3;
      c3_perm[1] = 2;
      c3_perm[2] = 1;
    }

    c3_b_ib = c3_ib;
    c3_f_b = c3_b_ib;
    c3_g_b = c3_f_b;
    c3_c_overflow = ((!(1 > c3_g_b)) && (c3_g_b > 2147483646));
    if (c3_c_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_d_k = 1; c3_d_k <= c3_b_ib; c3_d_k++) {
      c3_b_k = c3_d_k;
      c3_idx_data[(c3_tOffset - c3_ib) + c3_b_k] = c3_idx4[c3_perm[c3_b_k - 1] -
        1];
      c3_x_data[(c3_tOffset - c3_ib) + c3_b_k] = c3_x4[c3_perm[c3_b_k - 1] - 1];
    }
  }

  c3_m = c3_nNaNs >> 1;
  c3_b_m = c3_m;
  c3_d_b = c3_b_m;
  c3_e_b = c3_d_b;
  c3_b_overflow = ((!(1 > c3_e_b)) && (c3_e_b > 2147483646));
  if (c3_b_overflow) {
    c3_check_forloop_overflow_error(chartInstance, true);
  }

  for (c3_c_k = 1; c3_c_k <= c3_b_m; c3_c_k++) {
    c3_b_k = c3_c_k;
    c3_itmp = c3_idx_data[c3_tOffset + c3_b_k];
    c3_idx_data[c3_tOffset + c3_b_k] = c3_idx_data[c3_b_n - c3_b_k];
    c3_idx_data[c3_b_n - c3_b_k] = c3_itmp;
    c3_x_data[c3_tOffset + c3_b_k] = c3_xwork_data[c3_b_n - c3_b_k];
    c3_x_data[c3_b_n - c3_b_k] = c3_xwork_data[(c3_wOffset + c3_b_k) - 1];
  }

  if ((c3_nNaNs & 1) != 0) {
    c3_x_data[(c3_tOffset + c3_m) + 1] = c3_xwork_data[c3_wOffset + c3_m];
  }

  c3_b_nNaNs = c3_nNaNs;
  c3_nNonNaN = c3_n - c3_b_nNaNs;
  if (c3_nNonNaN > 1) {
    c3_b_merge_block(chartInstance, c3_idx_data, c3_idx_sizes, c3_x_data,
                     c3_x_sizes, 0, c3_nNonNaN, 2, c3_iwork_data,
                     &c3_iwork_sizes, c3_xwork_data, &c3_xwork_sizes);
  }
}

static void c3_b_merge_block(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance, int32_T c3_idx_data[], int32_T *c3_idx_sizes, real_T
  c3_x_data[], int32_T *c3_x_sizes, int32_T c3_offset, int32_T c3_n, int32_T
  c3_preSortLevel, int32_T c3_iwork_data[], int32_T *c3_iwork_sizes, real_T
  c3_xwork_data[], int32_T *c3_xwork_sizes)
{
  int32_T c3_nBlocks;
  int32_T c3_bLen;
  int32_T c3_bLen2;
  int32_T c3_tailOffset;
  int32_T c3_nPairs;
  int32_T c3_nTail;
  int32_T c3_b_nPairs;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_k;
  int32_T c3_b_k;
  (void)c3_offset;
  (void)c3_preSortLevel;
  c3_nBlocks = c3_n >> 2;
  c3_bLen = 4;
  while (c3_nBlocks > 1) {
    if ((c3_nBlocks & 1) != 0) {
      c3_nBlocks--;
      c3_tailOffset = c3_bLen * c3_nBlocks;
      c3_nTail = c3_n - c3_tailOffset;
      if (c3_nTail > c3_bLen) {
        c3_b_merge(chartInstance, c3_idx_data, c3_idx_sizes, c3_x_data,
                   c3_x_sizes, c3_tailOffset, c3_bLen, c3_nTail - c3_bLen,
                   c3_iwork_data, c3_iwork_sizes, c3_xwork_data, c3_xwork_sizes);
      }
    }

    c3_bLen2 = c3_bLen << 1;
    c3_nPairs = c3_nBlocks >> 1;
    c3_b_nPairs = c3_nPairs;
    c3_b = c3_b_nPairs;
    c3_b_b = c3_b;
    c3_overflow = ((!(1 > c3_b_b)) && (c3_b_b > 2147483646));
    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_k = 1; c3_k <= c3_b_nPairs; c3_k++) {
      c3_b_k = c3_k - 1;
      c3_b_merge(chartInstance, c3_idx_data, c3_idx_sizes, c3_x_data, c3_x_sizes,
                 c3_b_k * c3_bLen2, c3_bLen, c3_bLen, c3_iwork_data,
                 c3_iwork_sizes, c3_xwork_data, c3_xwork_sizes);
    }

    c3_bLen = c3_bLen2;
    c3_nBlocks = c3_nPairs;
  }

  if (c3_n > c3_bLen) {
    c3_b_merge(chartInstance, c3_idx_data, c3_idx_sizes, c3_x_data, c3_x_sizes,
               0, c3_bLen, c3_n - c3_bLen, c3_iwork_data, c3_iwork_sizes,
               c3_xwork_data, c3_xwork_sizes);
  }
}

static void c3_b_merge(SFc3_AutoFollow_SimulationInstanceStruct *chartInstance,
  int32_T c3_idx_data[], int32_T *c3_idx_sizes, real_T c3_x_data[], int32_T
  *c3_x_sizes, int32_T c3_offset, int32_T c3_np, int32_T c3_nq, int32_T
  c3_iwork_data[], int32_T *c3_iwork_sizes, real_T c3_xwork_data[], int32_T
  *c3_xwork_sizes)
{
  int32_T c3_n;
  int32_T c3_b_n;
  int32_T c3_b;
  int32_T c3_b_b;
  boolean_T c3_overflow;
  int32_T c3_j;
  int32_T c3_p;
  int32_T c3_b_j;
  int32_T c3_pend;
  int32_T c3_q;
  int32_T c3_qend;
  int32_T c3_iout;
  int32_T c3_offset1;
  int32_T c3_b_p;
  int32_T c3_b_pend;
  int32_T c3_c_b;
  int32_T c3_d_b;
  boolean_T c3_b_overflow;
  int32_T c3_c_j;
  int32_T exitg1;
  (void)c3_idx_sizes;
  (void)c3_x_sizes;
  (void)c3_iwork_sizes;
  (void)c3_xwork_sizes;
  if (c3_nq == 0) {
  } else {
    c3_n = c3_np + c3_nq;
    c3_b_n = c3_n;
    c3_b = c3_b_n;
    c3_b_b = c3_b;
    c3_overflow = ((!(1 > c3_b_b)) && (c3_b_b > 2147483646));
    if (c3_overflow) {
      c3_check_forloop_overflow_error(chartInstance, true);
    }

    for (c3_j = 1; c3_j <= c3_b_n; c3_j++) {
      c3_b_j = c3_j - 1;
      c3_iwork_data[c3_b_j] = c3_idx_data[c3_offset + c3_b_j];
      c3_xwork_data[c3_b_j] = c3_x_data[c3_offset + c3_b_j];
    }

    c3_p = 0;
    c3_pend = c3_np;
    c3_q = c3_pend;
    c3_qend = c3_pend + c3_nq;
    c3_iout = c3_offset - 1;
    do {
      exitg1 = 0;
      c3_iout++;
      if (c3_xwork_data[c3_p] <= c3_xwork_data[c3_q]) {
        c3_idx_data[c3_iout] = c3_iwork_data[c3_p];
        c3_x_data[c3_iout] = c3_xwork_data[c3_p];
        if (c3_p + 1 < c3_pend) {
          c3_p++;
        } else {
          exitg1 = 1;
        }
      } else {
        c3_idx_data[c3_iout] = c3_iwork_data[c3_q];
        c3_x_data[c3_iout] = c3_xwork_data[c3_q];
        if (c3_q + 1 < c3_qend) {
          c3_q++;
        } else {
          c3_offset1 = (c3_iout - c3_p) + 1;
          c3_b_p = c3_p + 1;
          c3_b_pend = c3_pend;
          c3_c_b = c3_b_pend;
          c3_d_b = c3_c_b;
          c3_b_overflow = (c3_d_b > 2147483646);
          if (c3_b_overflow) {
            c3_check_forloop_overflow_error(chartInstance, true);
          }

          for (c3_c_j = c3_b_p; c3_c_j <= c3_b_pend; c3_c_j++) {
            c3_b_j = c3_c_j - 1;
            c3_idx_data[c3_offset1 + c3_b_j] = c3_iwork_data[c3_b_j];
            c3_x_data[c3_offset1 + c3_b_j] = c3_xwork_data[c3_b_j];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

static void init_dsm_address_info(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc3_AutoFollow_SimulationInstanceStruct
  *chartInstance)
{
  chartInstance->c3_xq = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c3_r = (real_T (*)[361])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c3_yq = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c3_theta = (real_T (*)[361])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c3_t = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
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
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2921287817U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2662999763U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2201845309U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2997600218U);
}

mxArray* sf_c3_AutoFollow_Simulation_get_post_codegen_info(void);
mxArray *sf_c3_AutoFollow_Simulation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("iTPW1SMYa6OGJbfCr32DXF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(361);
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
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(361);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c3_AutoFollow_Simulation_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_AutoFollow_Simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_AutoFollow_Simulation_jit_fallback_info(void)
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

mxArray *sf_c3_AutoFollow_Simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c3_AutoFollow_Simulation_get_post_codegen_info(void)
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
    SFc3_AutoFollow_SimulationInstanceStruct *chartInstance =
      (SFc3_AutoFollow_SimulationInstanceStruct *)sf_get_chart_instance_ptr(S);
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
           9,
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
          _SFD_SET_DATA_PROPS(2,1,1,0,"t");
          _SFD_SET_DATA_PROPS(3,2,0,1,"r");
          _SFD_SET_DATA_PROPS(4,2,0,1,"theta");
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
        _SFD_CV_INIT_EML(0,1,1,0,2,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1359);
        _SFD_CV_INIT_EML_IF(0,1,0,114,132,-1,160);
        _SFD_CV_INIT_EML_IF(0,1,1,257,311,1295,1355);

        {
          static int condStart[] = { 261, 280 };

          static int condEnd[] = { 276, 310 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,261,310,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,280,310,-1,4);
        _SFD_CV_INIT_SCRIPT(0,1,0,4,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"se2",1148,-1,1631);
        _SFD_CV_INIT_SCRIPT_IF(0,0,1179,1196,1256,1534);
        _SFD_CV_INIT_SCRIPT_IF(0,1,1256,1277,1405,1534);
        _SFD_CV_INIT_SCRIPT_IF(0,2,1322,1335,1364,1400);
        _SFD_CV_INIT_SCRIPT_IF(0,3,1448,1461,1490,1526);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(0,0,1182,1196,-1,0);
        _SFD_CV_INIT_SCRIPT(1,1,0,2,0,0,0,1,0,2,1);
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

        _SFD_CV_INIT_SCRIPT_RELATIONAL(1,0,1918,1930,-1,4);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"XYtoIJ",0,-1,222);
        _SFD_CV_INIT_SCRIPT(3,1,0,4,0,0,0,1,0,6,3);
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

        _SFD_CV_INIT_SCRIPT_RELATIONAL(3,0,1383,1394,-1,0);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(3,1,1398,1408,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(3,2,1452,1463,-1,0);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(3,3,1467,1477,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(3,4,1519,1535,-1,1);
        _SFD_CV_INIT_SCRIPT(4,1,0,5,0,0,0,0,0,3,2);
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

        _SFD_CV_INIT_SCRIPT_RELATIONAL(4,0,1486,1494,-1,1);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(4,2,2426,2432,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(4,3,2583,2589,-1,4);
        _SFD_CV_INIT_SCRIPT(5,1,0,2,0,0,0,0,0,3,1);
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

        _SFD_CV_INIT_SCRIPT(6,1,0,11,0,0,0,0,2,8,2);
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

        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,0,1178,1185,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,1,1296,1303,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,2,1415,1422,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,3,1547,1552,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,4,1556,1561,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,5,1565,1570,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,6,1574,1579,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,7,1655,1669,-1,0);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,8,1824,1829,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,9,2094,2099,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,10,2103,2108,-1,2);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,11,2112,2117,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,12,2121,2126,-1,4);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,13,2202,2216,-1,0);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(6,14,2402,2407,-1,2);
        _SFD_CV_INIT_SCRIPT(7,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(7,0,"IJtoXY",0,-1,175);
        _SFD_CV_INIT_SCRIPT(8,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(8,0,"deg2rad",0,-1,459);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_d_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_d_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_d_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 361U;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_c_sf_marshallOut,(MexInFcnForType)
            c3_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 361U;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_c_sf_marshallOut,(MexInFcnForType)
            c3_c_sf_marshallIn);
        }
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
    SFc3_AutoFollow_SimulationInstanceStruct *chartInstance =
      (SFc3_AutoFollow_SimulationInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c3_xq);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c3_r);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c3_yq);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c3_theta);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c3_t);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s7aVIEtt11T3fvhO7WoYKfF";
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

static const mxArray* sf_opaque_get_sim_state_c3_AutoFollow_Simulation(SimStruct*
  S)
{
  return get_sim_state_c3_AutoFollow_Simulation
    ((SFc3_AutoFollow_SimulationInstanceStruct *)sf_get_chart_instance_ptr(S));/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c3_AutoFollow_Simulation(SimStruct* S, const
  mxArray *st)
{
  set_sim_state_c3_AutoFollow_Simulation
    ((SFc3_AutoFollow_SimulationInstanceStruct*)sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c3_AutoFollow_Simulation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_AutoFollow_SimulationInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_AutoFollow_Simulation_optimization_info();
    }

    finalize_c3_AutoFollow_Simulation((SFc3_AutoFollow_SimulationInstanceStruct*)
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
    initialize_params_c3_AutoFollow_Simulation
      ((SFc3_AutoFollow_SimulationInstanceStruct*)sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c3_AutoFollow_Simulation(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_AutoFollow_Simulation_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 3);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,3);
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
  ssSetChecksum0(S,(3146717175U));
  ssSetChecksum1(S,(2777744590U));
  ssSetChecksum2(S,(1698244741U));
  ssSetChecksum3(S,(4047019301U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
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
  chartInstance = (SFc3_AutoFollow_SimulationInstanceStruct *)utMalloc(sizeof
    (SFc3_AutoFollow_SimulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc3_AutoFollow_SimulationInstanceStruct));
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
  mdl_start_c3_AutoFollow_Simulation(chartInstance);
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
