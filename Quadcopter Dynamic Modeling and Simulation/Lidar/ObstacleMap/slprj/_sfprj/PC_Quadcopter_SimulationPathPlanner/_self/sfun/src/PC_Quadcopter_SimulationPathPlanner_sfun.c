/* Include files */

#include "PC_Quadcopter_SimulationPathPlanner_sfun.h"
#include "PC_Quadcopter_SimulationPathPlanner_sfun_debug_macros.h"
#include "c1_PC_Quadcopter_SimulationPathPlanner.h"
#include "c2_PC_Quadcopter_SimulationPathPlanner.h"
#include "c3_PC_Quadcopter_SimulationPathPlanner.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _PC_Quadcopter_SimulationPathPlannerMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void PC_Quadcopter_SimulationPathPlanner_initializer(void)
{
}

void PC_Quadcopter_SimulationPathPlanner_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_PC_Quadcopter_SimulationPathPlanner_method_dispatcher(SimStruct *
  simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==1) {
    c1_PC_Quadcopter_SimulationPathPlanner_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_PC_Quadcopter_SimulationPathPlanner_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_PC_Quadcopter_SimulationPathPlanner_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_PC_Quadcopter_SimulationPathPlanner_process_check_sum_call( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2073393308U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2948588441U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1122650671U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1519187215U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1091271907U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4007459125U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1421239769U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3645091433U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_PC_Quadcopter_SimulationPathPlanner_get_check_sum
            (mxArray *plhs[]);
          sf_c1_PC_Quadcopter_SimulationPathPlanner_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_PC_Quadcopter_SimulationPathPlanner_get_check_sum
            (mxArray *plhs[]);
          sf_c2_PC_Quadcopter_SimulationPathPlanner_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_PC_Quadcopter_SimulationPathPlanner_get_check_sum
            (mxArray *plhs[]);
          sf_c3_PC_Quadcopter_SimulationPathPlanner_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2083502392U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1110276785U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3258378658U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3926592909U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1083715300U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2371424057U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1131255170U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3474281746U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_PC_Quadcopter_SimulationPathPlanner_autoinheritance_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "DBrhK5NAqyYRxXz6nMg5uB") == 0) {
          extern mxArray
            *sf_c1_PC_Quadcopter_SimulationPathPlanner_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c1_PC_Quadcopter_SimulationPathPlanner_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "1DK66Nj8xbqd2SBiSMfdaF") == 0) {
          extern mxArray
            *sf_c2_PC_Quadcopter_SimulationPathPlanner_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c2_PC_Quadcopter_SimulationPathPlanner_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "MId26Yuz4T6mplnaF74W5F") == 0) {
          extern mxArray
            *sf_c3_PC_Quadcopter_SimulationPathPlanner_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c3_PC_Quadcopter_SimulationPathPlanner_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int
  sf_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_PC_Quadcopter_SimulationPathPlanner_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "N5IrWP5huURc01ljDS4RYE") == 0) {
          extern mxArray
            *sf_c1_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info
            (void);
          plhs[0] =
            sf_c1_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "ArD3gF6mVJDXAZl85LBvoB") == 0) {
          extern mxArray
            *sf_c2_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info
            (void);
          plhs[0] =
            sf_c2_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "TMbW3TT2FI281gZHr2GGzD") == 0) {
          extern mxArray
            *sf_c3_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info
            (void);
          plhs[0] =
            sf_c3_PC_Quadcopter_SimulationPathPlanner_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "N5IrWP5huURc01ljDS4RYE") == 0) {
          extern mxArray
            *sf_c1_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c1_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "ArD3gF6mVJDXAZl85LBvoB") == 0) {
          extern mxArray
            *sf_c2_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c2_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "TMbW3TT2FI281gZHr2GGzD") == 0) {
          extern mxArray
            *sf_c3_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c3_PC_Quadcopter_SimulationPathPlanner_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void PC_Quadcopter_SimulationPathPlanner_debug_initialize(struct
  SfDebugInstanceStruct* debugInstance)
{
  _PC_Quadcopter_SimulationPathPlannerMachineNumber_ =
    sf_debug_initialize_machine(debugInstance,
    "PC_Quadcopter_SimulationPathPlanner","sfun",0,3,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _PC_Quadcopter_SimulationPathPlannerMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _PC_Quadcopter_SimulationPathPlannerMachineNumber_,0);
}

void PC_Quadcopter_SimulationPathPlanner_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_PC_Quadcopter_SimulationPathPlanner_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "PC_Quadcopter_SimulationPathPlanner",
      "PC_Quadcopter_SimulationPathPlanner");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_PC_Quadcopter_SimulationPathPlanner_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
