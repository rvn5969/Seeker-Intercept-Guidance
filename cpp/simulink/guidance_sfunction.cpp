#define S_FUNCTION_NAME guidance_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "intercept/config.hpp"
#include "intercept/guidance_stack.hpp"
#include <memory>
#include <string>

static std::string configPath(SimStruct* S){
  char* raw = mxArrayToString(ssGetSFcnParam(S,0));
  std::string path = raw ? raw : "config/experiment.json";
  mxFree(raw);
  return path;
}

static void mdlInitializeSizes(SimStruct* S){
  ssSetNumSFcnParams(S,1);
  if(ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

  if(!ssSetNumInputPorts(S,1)) return;
  ssSetInputPortWidth(S,0,7);
  ssSetInputPortDirectFeedThrough(S,0,1);

  if(!ssSetNumOutputPorts(S,1)) return;
  ssSetOutputPortWidth(S,0,2);

  ssSetNumSampleTimes(S,1);
  ssSetNumPWork(S,1);
}

static void mdlInitializeSampleTimes(SimStruct* S){
  try{
    const auto cfg = intercept::loadConfig(configPath(S));
    ssSetSampleTime(S,0,cfg.simulation.guidance_dt);
    ssSetOffsetTime(S,0,0.0);
  } catch(const std::exception& e){
    ssSetErrorStatus(S,e.what());
  }
}

#define MDL_START
static void mdlStart(SimStruct* S){
  try{
    ssSetPWorkValue(
      S,0,
      new intercept::GuidanceStack(intercept::loadConfig(configPath(S)))
    );
  } catch(const std::exception& e){
    ssSetErrorStatus(S,e.what());
  }
}

static void mdlOutputs(SimStruct* S,int_T){
  auto* stack =
    static_cast<intercept::GuidanceStack*>(ssGetPWorkValue(S,0));

  real_T* y = ssGetOutputPortRealSignal(S,0);
  if(!stack){
    y[0] = 0.0;
    y[1] = 0.0;
    return;
  }

  InputRealPtrsType u = ssGetInputPortRealSignalPtrs(S,0);

  const double timestamp  = *u[0];
  const double range      = *u[1];
  const double bearing    = *u[2];
  const double own_speed  = *u[3];
  const double own_heading= *u[4];
  const double own_accel  = *u[5];
  const bool valid        = (*u[6] > 0.5);

  intercept::SeekerMeasurement z{
    timestamp,
    range,
    bearing,
    own_speed,
    own_heading,
    own_accel,
    valid
  };

  const auto c = stack->update(z,ssGetT(S));
  y[0] = c.lateral_accel;
  y[1] = c.valid ? 1.0 : 0.0;
}

static void mdlTerminate(SimStruct* S){
  delete static_cast<intercept::GuidanceStack*>(ssGetPWorkValue(S,0));
  ssSetPWorkValue(S,0,nullptr);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
