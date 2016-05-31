/* Include files */

#include "AutoFollow_Simulation_sfun.h"
#include "AutoFollow_Simulation_sfun_debug_macros.h"
#include "c1_AutoFollow_Simulation.h"
#include "c2_AutoFollow_Simulation.h"
#include "c3_AutoFollow_Simulation.h"
#include "c4_AutoFollow_Simulation.h"
#include "c5_AutoFollow_Simulation.h"
#include "c6_AutoFollow_Simulation.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _AutoFollow_SimulationMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void AutoFollow_Simulation_initializer(void)
{
}

void AutoFollow_Simulation_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_AutoFollow_Simulation_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_AutoFollow_Simulation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_AutoFollow_Simulation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_AutoFollow_Simulation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_AutoFollow_Simulation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_AutoFollow_Simulation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_AutoFollow_Simulation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_AutoFollow_Simulation_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2277929674U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2153482220U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1812946908U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1978861714U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(685345678U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(545804772U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2361839259U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1161594813U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
          sf_c1_AutoFollow_Simulation_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
          sf_c2_AutoFollow_Simulation_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
          sf_c3_AutoFollow_Simulation_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
          sf_c4_AutoFollow_Simulation_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
          sf_c5_AutoFollow_Simulation_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_AutoFollow_Simulation_get_check_sum(mxArray *plhs[]);
          sf_c6_AutoFollow_Simulation_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4275001758U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2927837604U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(835195587U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1860256421U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_AutoFollow_Simulation_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        if (strcmp(aiChksum, "boxXMWj1nd84qMyDj1YNZ") == 0) {
          extern mxArray *sf_c1_AutoFollow_Simulation_get_autoinheritance_info
            (void);
          plhs[0] = sf_c1_AutoFollow_Simulation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "sfm7xB1DclXufWrOSjKMZD") == 0) {
          extern mxArray *sf_c2_AutoFollow_Simulation_get_autoinheritance_info
            (void);
          plhs[0] = sf_c2_AutoFollow_Simulation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "MUTgSGXVgQRTz47ojo4BRG") == 0) {
          extern mxArray *sf_c3_AutoFollow_Simulation_get_autoinheritance_info
            (void);
          plhs[0] = sf_c3_AutoFollow_Simulation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "Dm0ImkfMyr1lqH6LSAKJCF") == 0) {
          extern mxArray *sf_c4_AutoFollow_Simulation_get_autoinheritance_info
            (void);
          plhs[0] = sf_c4_AutoFollow_Simulation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "8gjAqZTpYOmyJGOAsoyXrH") == 0) {
          extern mxArray *sf_c5_AutoFollow_Simulation_get_autoinheritance_info
            (void);
          plhs[0] = sf_c5_AutoFollow_Simulation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "fM5HFctc9srUlvE1jTp8tD") == 0) {
          extern mxArray *sf_c6_AutoFollow_Simulation_get_autoinheritance_info
            (void);
          plhs[0] = sf_c6_AutoFollow_Simulation_get_autoinheritance_info();
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

unsigned int sf_AutoFollow_Simulation_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
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
          *sf_c1_AutoFollow_Simulation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_AutoFollow_Simulation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_AutoFollow_Simulation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_AutoFollow_Simulation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_AutoFollow_Simulation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_AutoFollow_Simulation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_AutoFollow_Simulation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_AutoFollow_Simulation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_AutoFollow_Simulation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_AutoFollow_Simulation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_AutoFollow_Simulation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_AutoFollow_Simulation_get_eml_resolved_functions_info();
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

unsigned int sf_AutoFollow_Simulation_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "XfeqXlR09XU765x7fCuYS") == 0) {
          extern mxArray *sf_c1_AutoFollow_Simulation_third_party_uses_info(void);
          plhs[0] = sf_c1_AutoFollow_Simulation_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "YjyGK2g4frignzJ2s5J4XB") == 0) {
          extern mxArray *sf_c2_AutoFollow_Simulation_third_party_uses_info(void);
          plhs[0] = sf_c2_AutoFollow_Simulation_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "bTXtGSNog6ZcIqZotHDAPE") == 0) {
          extern mxArray *sf_c3_AutoFollow_Simulation_third_party_uses_info(void);
          plhs[0] = sf_c3_AutoFollow_Simulation_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "QuBtlAEEQwheLw8n0OTZIC") == 0) {
          extern mxArray *sf_c4_AutoFollow_Simulation_third_party_uses_info(void);
          plhs[0] = sf_c4_AutoFollow_Simulation_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "rqghAeRqUFNXi1Hhi13ACE") == 0) {
          extern mxArray *sf_c5_AutoFollow_Simulation_third_party_uses_info(void);
          plhs[0] = sf_c5_AutoFollow_Simulation_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "6qQ1mOSRx4m5QZJvQwNJg") == 0) {
          extern mxArray *sf_c6_AutoFollow_Simulation_third_party_uses_info(void);
          plhs[0] = sf_c6_AutoFollow_Simulation_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_AutoFollow_Simulation_updateBuildInfo_args_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "XfeqXlR09XU765x7fCuYS") == 0) {
          extern mxArray *sf_c1_AutoFollow_Simulation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c1_AutoFollow_Simulation_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "YjyGK2g4frignzJ2s5J4XB") == 0) {
          extern mxArray *sf_c2_AutoFollow_Simulation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c2_AutoFollow_Simulation_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "bTXtGSNog6ZcIqZotHDAPE") == 0) {
          extern mxArray *sf_c3_AutoFollow_Simulation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c3_AutoFollow_Simulation_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "QuBtlAEEQwheLw8n0OTZIC") == 0) {
          extern mxArray *sf_c4_AutoFollow_Simulation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c4_AutoFollow_Simulation_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "rqghAeRqUFNXi1Hhi13ACE") == 0) {
          extern mxArray *sf_c5_AutoFollow_Simulation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c5_AutoFollow_Simulation_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "6qQ1mOSRx4m5QZJvQwNJg") == 0) {
          extern mxArray *sf_c6_AutoFollow_Simulation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c6_AutoFollow_Simulation_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void AutoFollow_Simulation_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _AutoFollow_SimulationMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"AutoFollow_Simulation","sfun",0,6,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _AutoFollow_SimulationMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _AutoFollow_SimulationMachineNumber_,0);
}

void AutoFollow_Simulation_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_AutoFollow_Simulation_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "AutoFollow_Simulation", "AutoFollow_Simulation");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_AutoFollow_Simulation_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
