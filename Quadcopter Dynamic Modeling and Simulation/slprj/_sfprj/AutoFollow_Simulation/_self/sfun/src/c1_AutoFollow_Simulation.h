#ifndef __c1_AutoFollow_Simulation_h__
#define __c1_AutoFollow_Simulation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_s2aqkGCuE38RBomNVWBcX1B
#define struct_s2aqkGCuE38RBomNVWBcX1B

struct s2aqkGCuE38RBomNVWBcX1B
{
  real_T map[10000];
};

#endif                                 /*struct_s2aqkGCuE38RBomNVWBcX1B*/

#ifndef typedef_c1_s2aqkGCuE38RBomNVWBcX1B
#define typedef_c1_s2aqkGCuE38RBomNVWBcX1B

typedef struct s2aqkGCuE38RBomNVWBcX1B c1_s2aqkGCuE38RBomNVWBcX1B;

#endif                                 /*typedef_c1_s2aqkGCuE38RBomNVWBcX1B*/

#ifndef typedef_SFc1_AutoFollow_SimulationInstanceStruct
#define typedef_SFc1_AutoFollow_SimulationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_AutoFollow_Simulation;
  real_T c1_r_prev;
  boolean_T c1_r_prev_not_empty;
  real_T c1_theta_prev;
  boolean_T c1_theta_prev_not_empty;
  real_T c1_psi_prev;
  boolean_T c1_psi_prev_not_empty;
} SFc1_AutoFollow_SimulationInstanceStruct;

#endif                                 /*typedef_SFc1_AutoFollow_SimulationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_AutoFollow_Simulation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
extern void c1_AutoFollow_Simulation_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
