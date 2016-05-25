#ifndef __c6_AutoFollow_Simulation_h__
#define __c6_AutoFollow_Simulation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc6_AutoFollow_SimulationInstanceStruct
#define typedef_SFc6_AutoFollow_SimulationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c6_sfEvent;
  boolean_T c6_isStable;
  boolean_T c6_doneDoubleBufferReInit;
  uint8_T c6_is_active_c6_AutoFollow_Simulation;
  real_T c6_prevT;
  boolean_T c6_prevT_not_empty;
  real_T c6_prevMeas[3];
  boolean_T c6_prevMeas_not_empty;
  real_T c6_P[36];
  boolean_T c6_P_not_empty;
  real_T c6_xhat[6];
  boolean_T c6_xhat_not_empty;
  real_T c6_Q[36];
  boolean_T c6_Q_not_empty;
  real_T c6_R[9];
  boolean_T c6_R_not_empty;
} SFc6_AutoFollow_SimulationInstanceStruct;

#endif                                 /*typedef_SFc6_AutoFollow_SimulationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c6_AutoFollow_Simulation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c6_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
extern void c6_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
