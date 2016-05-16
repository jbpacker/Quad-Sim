#ifndef __c1_kalmanTest_h__
#define __c1_kalmanTest_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_kalmanTestInstanceStruct
#define typedef_SFc1_kalmanTestInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_kalmanTest;
  real_T c1_prevT;
  boolean_T c1_prevT_not_empty;
  real_T c1_prevY[3];
  boolean_T c1_prevY_not_empty;
  real_T c1_P[36];
  boolean_T c1_P_not_empty;
  real_T c1_xhat[6];
  boolean_T c1_xhat_not_empty;
  real_T c1_Q[36];
  boolean_T c1_Q_not_empty;
  real_T c1_R[9];
  boolean_T c1_R_not_empty;
} SFc1_kalmanTestInstanceStruct;

#endif                                 /*typedef_SFc1_kalmanTestInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_kalmanTest_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_kalmanTest_get_check_sum(mxArray *plhs[]);
extern void c1_kalmanTest_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
