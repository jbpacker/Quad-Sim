#ifndef __c4_AutoFollow_Simulation_h__
#define __c4_AutoFollow_Simulation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc4_AutoFollow_SimulationInstanceStruct
#define typedef_SFc4_AutoFollow_SimulationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c4_sfEvent;
  boolean_T c4_isStable;
  boolean_T c4_doneDoubleBufferReInit;
  uint8_T c4_is_active_c4_AutoFollow_Simulation;
  real_T c4_prevT;
  boolean_T c4_prevT_not_empty;
  real_T c4_prevY[3];
  boolean_T c4_prevY_not_empty;
  real_T c4_P[36];
  boolean_T c4_P_not_empty;
  real_T c4_xhat[6];
  boolean_T c4_xhat_not_empty;
  real_T c4_Q[36];
  boolean_T c4_Q_not_empty;
  real_T c4_R[9];
  boolean_T c4_R_not_empty;
} SFc4_AutoFollow_SimulationInstanceStruct;

#endif                                 /*typedef_SFc4_AutoFollow_SimulationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c4_AutoFollow_Simulation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c4_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
extern void c4_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
