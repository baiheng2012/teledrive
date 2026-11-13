//
// File: pid_ctrl.cpp
//
// Code generated for Simulink model 'pid_ctrl'.
//
// Model version                  : 1.2
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Fri Nov 13 00:41:03 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "pid_ctrl.h"
#include <cmath>
#include "rtwtypes.h"

// Model step function
void pid_ctrl::step()
{
  real_T cand;
  real_T dterm;

  // MATLAB Function: '<Root>/PID' incorporates:
  //   Inport: '<Root>/alpha'
  //   Inport: '<Root>/err'
  //   Inport: '<Root>/kd'
  //   Inport: '<Root>/ki'
  //   Inport: '<Root>/kp'
  //   Inport: '<Root>/meas'
  //   Inport: '<Root>/omax'
  //   Inport: '<Root>/omin'

  dterm = 0.0;
  if (!pid_ctrl_DW.first) {
    pid_ctrl_DW.dfilt += (-(pid_ctrl_U.meas - pid_ctrl_DW.prev) / 0.02 -
                          pid_ctrl_DW.dfilt) * pid_ctrl_U.alpha;
    dterm = pid_ctrl_U.kd * pid_ctrl_DW.dfilt;
  }

  pid_ctrl_DW.prev = pid_ctrl_U.meas;
  pid_ctrl_DW.first = false;
  cand = pid_ctrl_U.err * 0.02 + pid_ctrl_DW.integ;
  dterm += pid_ctrl_U.kp * pid_ctrl_U.err + pid_ctrl_U.ki * cand;
  if (((!(dterm > pid_ctrl_U.omax)) || (!(pid_ctrl_U.err > 0.0))) && ((!(dterm <
         pid_ctrl_U.omin)) || (!(pid_ctrl_U.err < 0.0)))) {
    pid_ctrl_DW.integ = cand;
  }

  // Outport: '<Root>/out' incorporates:
  //   Inport: '<Root>/omax'
  //   Inport: '<Root>/omin'
  //   MATLAB Function: '<Root>/PID'

  pid_ctrl_Y.out = std::fmin(std::fmax(dterm, pid_ctrl_U.omin), pid_ctrl_U.omax);
}

// Model initialize function
void pid_ctrl::initialize()
{
  // SystemInitialize for MATLAB Function: '<Root>/PID'
  pid_ctrl_DW.first = true;
}

// Model terminate function
void pid_ctrl::terminate()
{
  // (no terminate code required)
}

// Constructor
pid_ctrl::pid_ctrl() :
  pid_ctrl_U(),
  pid_ctrl_Y(),
  pid_ctrl_DW(),
  pid_ctrl_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
pid_ctrl::~pid_ctrl() = default;

// Real-Time Model get method
pid_ctrl::RT_MODEL_pid_ctrl_T * pid_ctrl::getRTM()
{
  return (&pid_ctrl_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
