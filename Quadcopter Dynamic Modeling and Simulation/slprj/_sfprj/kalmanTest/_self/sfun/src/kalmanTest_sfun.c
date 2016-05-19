/* Include files */

#include "kalmanTest_sfun.h"
#include "kalmanTest_sfun_debug_macros.h"
#include "c1_kalmanTest.h"
#include "c2_kalmanTest.h"
#include "c3_kalmanTest.h"
#include "c4_kalmanTest.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _kalmanTestMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void kalmanTest_initializer(void)
{
}

void kalmanTest_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_kalmanTest_method_dispatcher(SimStruct *simstructPtr, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_kalmanTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_kalmanTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_kalmanTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_kalmanTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_kalmanTest_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(696788221U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2378393985U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2155484143U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3084694242U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(871470012U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(426041252U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2672872198U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(253090707U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_kalmanTest_get_check_sum(mxArray *plhs[]);
          sf_c1_kalmanTest_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_kalmanTest_get_check_sum(mxArray *plhs[]);
          sf_c2_kalmanTest_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_kalmanTest_get_check_sum(mxArray *plhs[]);
          sf_c3_kalmanTest_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_kalmanTest_get_check_sum(mxArray *plhs[]);
          sf_c4_kalmanTest_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1216703283U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1655708648U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2127466952U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1508870030U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_kalmanTest_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
        if (strcmp(aiChksum, "Dm0ImkfMyr1lqH6LSAKJCF") == 0) {
          extern mxArray *sf_c1_kalmanTest_get_autoinheritance_info(void);
          plhs[0] = sf_c1_kalmanTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "ATGgS8pEPCRhk6O3F0jHaG") == 0) {
          extern mxArray *sf_c2_kalmanTest_get_autoinheritance_info(void);
          plhs[0] = sf_c2_kalmanTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "8gjAqZTpYOmyJGOAsoyXrH") == 0) {
          extern mxArray *sf_c3_kalmanTest_get_autoinheritance_info(void);
          plhs[0] = sf_c3_kalmanTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "fM5HFctc9srUlvE1jTp8tD") == 0) {
          extern mxArray *sf_c4_kalmanTest_get_autoinheritance_info(void);
          plhs[0] = sf_c4_kalmanTest_get_autoinheritance_info();
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

unsigned int sf_kalmanTest_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        extern const mxArray *sf_c1_kalmanTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_kalmanTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray *sf_c2_kalmanTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_kalmanTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_kalmanTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_kalmanTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_kalmanTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_kalmanTest_get_eml_resolved_functions_info();
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

unsigned int sf_kalmanTest_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "QuBtlAEEQwheLw8n0OTZIC") == 0) {
          extern mxArray *sf_c1_kalmanTest_third_party_uses_info(void);
          plhs[0] = sf_c1_kalmanTest_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "Tefau5Tuh9V44tzq6TnMfG") == 0) {
          extern mxArray *sf_c2_kalmanTest_third_party_uses_info(void);
          plhs[0] = sf_c2_kalmanTest_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "rqghAeRqUFNXi1Hhi13ACE") == 0) {
          extern mxArray *sf_c3_kalmanTest_third_party_uses_info(void);
          plhs[0] = sf_c3_kalmanTest_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "6qQ1mOSRx4m5QZJvQwNJg") == 0) {
          extern mxArray *sf_c4_kalmanTest_third_party_uses_info(void);
          plhs[0] = sf_c4_kalmanTest_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_kalmanTest_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "QuBtlAEEQwheLw8n0OTZIC") == 0) {
          extern mxArray *sf_c1_kalmanTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_kalmanTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "Tefau5Tuh9V44tzq6TnMfG") == 0) {
          extern mxArray *sf_c2_kalmanTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_kalmanTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "rqghAeRqUFNXi1Hhi13ACE") == 0) {
          extern mxArray *sf_c3_kalmanTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_kalmanTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "6qQ1mOSRx4m5QZJvQwNJg") == 0) {
          extern mxArray *sf_c4_kalmanTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_kalmanTest_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void kalmanTest_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _kalmanTestMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "kalmanTest","sfun",0,4,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_kalmanTestMachineNumber_,
    0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_kalmanTestMachineNumber_,0);
}

void kalmanTest_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_kalmanTest_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("kalmanTest",
      "kalmanTest");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_kalmanTest_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
