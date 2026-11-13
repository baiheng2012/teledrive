//
// File: pid_ctrl.h
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
#ifndef pid_ctrl_h_
#define pid_ctrl_h_
#include <cmath>
#include "rtwtypes.h"
#include "pid_ctrl_types.h"
// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif
#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif
// Class declaration for model pid_ctrl
class pid_ctrl final
{
// public data and function members
public:
// Block states (default storage) for system '<Root>'
struct DW_pid_ctrl_T {
    real_T integ;                      // '<Root>/PID'
    real_T prev;                       // '<Root>/PID'
    real_T dfilt;                      // '<Root>/PID'
    boolean_T first;                   // '<Root>/PID'
  };
// External inputs (root inport signals with default storage)
struct ExtU_pid_ctrl_T {
    real_T err;                        // '<Root>/err'
    real_T meas;                       // '<Root>/meas'
    real_T kp;                         // '<Root>/kp'
    real_T ki;                         // '<Root>/ki'
    real_T kd;                         // '<Root>/kd'
    real_T omin;                       // '<Root>/omin'
    real_T omax;                       // '<Root>/omax'
    real_T alpha;                      // '<Root>/alpha'
  };
// External outputs (root outports fed by signals with default storage)
struct ExtY_pid_ctrl_T {
    real_T out;                        // '<Root>/out'
  };
// Real-time Model Data Structure
struct RT_MODEL_pid_ctrl_T {
const char_T * volatile errorStatus;
  };
// Copy Constructor
  pid_ctrl(pid_ctrl const&) = delete;
// Assignment Operator
  pid_ctrl& operator= (pid_ctrl const&) & = delete;
// Move Constructor
  pid_ctrl(pid_ctrl &&) = delete;
// Move Assignment Operator
  pid_ctrl& operator= (pid_ctrl &&) = delete;
// Real-Time Model get method
  pid_ctrl::RT_MODEL_pid_ctrl_T * getRTM();
// Root inports set method
void setExternalInputs(const ExtU_pid_ctrl_T *pExtU_pid_ctrl_T)
  {
    pid_ctrl_U = *pExtU_pid_ctrl_T;
  }
// Root outports get method
const ExtY_pid_ctrl_T &getExternalOutputs() const
  {
return pid_ctrl_Y;
  }
// model initialize function
void initialize();
// model step function
void step();
// model terminate function
static void terminate();
// Constructor
  pid_ctrl();
// Destructor
  ~pid_ctrl();
// private data and function members
private:
// External inputs
  ExtU_pid_ctrl_T pid_ctrl_U;
// External outputs
  ExtY_pid_ctrl_T pid_ctrl_Y;
// Block states
  DW_pid_ctrl_T pid_ctrl_DW;
// Real-Time Model
  RT_MODEL_pid_ctrl_T pid_ctrl_M;
};
//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'pid_ctrl'
//  '<S1>'   : 'pid_ctrl/PID'
#endif                                 // pid_ctrl_h_
//
// File trailer for generated code.
//
// [EOF]
//
