#ifndef PIDLITE_H
#define PIDLITE_H
/*
Configuration header
*/
#include "config.h"
#include <float.h>
/*
If USE_FLOAT defined in config.h,
use float instead of double
*/
#ifdef USE_FLOAT
  typedef float PID_t;
#else
  typedef double PID_t;
#endif
/*
Constant definitions for math.
Change at your own risk!
*/
#define SCALE_FACTOR 10000.0f
#define SCALE_MIN -0.5f
#define SCALE_MAX 0.5f
#define INTEGRAL_MAX 1000.0f
#define INTEGRAL_MIN -1000.0f
/*
Config struct for pid lib
*/
#ifdef ENABLE_DERIVATIVE
typedef struct {
  // Setpoint and process value
  PID_t SP, PV;
  // Control variable (unscaled output)
  PID_t CV;
  // Proportional and Integral gains
  PID_t PGain, IGain, DGain;
  // Proportional and integral terms
  PID_t PWeight, IWeight, DWeight;
  // Previous error for derivative
  PID_t PreviousError;
} PID_Handle_t;
#else
 typedef struct {
 // Setpoint and process value
 PID_t SP, PV;
 // Control variable (unscaled output)
 PID_t CV;
 // Proportional and Integral gains
 PID_t PGain, IGain;
 // Proportional and integral terms
 PID_t PWeight, IWeight;
 } PID_Handle_t;
#endif
/*
Update SP at tarGet PID
*/
void PID_SetSP(PID_Handle_t *PID, PID_t new_SP);
/*
Update PV at tarGet PID
*/
void PID_SetPV(PID_Handle_t *PID, PID_t new_PV);
/*
Calculate error
*/
PID_t PID_GetError(PID_Handle_t *PID);
/*
Calculate P weight
*/
void PID_GetPWeight(PID_Handle_t *PID, PID_t error);
/*
Calculate I weight
*/
void PID_GetIWeight(PID_Handle_t *PID, PID_t error);
/*
Calculate D weight
*/
void PID_GetDWeight(PID_Handle_t *PID, PID_t new_error);
/*
Update tarGet pid
*/
void PID_Update(PID_Handle_t *PID);
// Input -> PID Control System -> Output
void PID_Process(PID_Handle_t *PID, PID_t adjustment_factor);

void PID_ClearTerms(PID_Handle_t *PID);
#endif
