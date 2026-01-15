#include "cpid.h"
/*
Update SP for tarGet PID
*/
void PID_SetSP(PID_Handle_t *PID, PID_t new_SP)
{
    // Store input value at tarGet PID SP
    PID->SP = new_SP;
}
/*
Update PV for tarGet PID
*/
void PID_SetPV(PID_Handle_t *PID, PID_t new_PV)
{
    // Store input value at tarGet PID PV
    PID->PV = new_PV;
}
/*
Calculate error
*/
PID_t PID_GetError(PID_Handle_t *PID)
{
    return (PID->SP - PID->PV);
}
/*
Calculate P weight
*/
void PID_GetPWeight(PID_Handle_t *PID, PID_t error)
{
    // Multiply defined P_GAIN by most recent error
    PID->PWeight = (PID->PGain * error);
}
/*
Calculate I weight
*/
void PID_GetIWeight(PID_Handle_t *PID, PID_t error)
{
    // Multiply I_GAIN by new error and add to moving sum
    PID->IWeight += (PID->IGain * error);
    if (PID->IWeight > INTEGRAL_MAX) {
        PID->IWeight = INTEGRAL_MAX;
    }
    else if (PID->IWeight < INTEGRAL_MIN) {
        PID->IWeight = INTEGRAL_MIN;
    }
}
#ifdef ENABLE_DERIVATIVE
/*
Calculate D weight
*/
void PID_GetDWeight(PID_Handle_t *PID, PID_t new_error) {
    // Calculate the change in error (Current Error - Previous Error)
    PID_t error_difference = new_error - PID->PreviousError;
    // Multiply D Gain by the difference (slope)
    PID->DWeight = (PID->DGain * error_difference);
}
#endif
/*
Update target PID
*/
#ifdef ENABLE_DERIVATIVE
void PID_Update(PID_Handle_t *PID) {
    // Find new error
    PID_t new_error = PID_GetError(PID);
    // Calculate P weight
    PID_GetPWeight(PID, new_error);
    // Calculate I weight
    PID_GetIWeight(PID, new_error);
    // Calculate D weight
    PID_GetDWeight(PID, new_error);
    // Move new error into previous_error
    PID->PreviousError = new_error;
    // Sum weights and Update output
    PID_t sum = (PID->PWeight + PID->IWeight + PID->DWeight);
    // Normalize sum
    PID_t scaled = (sum / SCALE_FACTOR);
    if (scaled > SCALE_MAX) {
        scaled = SCALE_MAX;
    }
    else if (scaled < SCALE_MIN) {
        scaled = SCALE_MIN;
    }
    // Give scaled value to CV
    PID->CV = scaled;
}
// Alternate definition with no derivative term
#else
void PID_Update(PID_Handle_t *PID) {
    // Find new error
    PID_t new_error = PID_GetError(PID);
    // Calculate P weight
    PID_GetPWeight(PID, new_error);
    // Calculate I weight
    PID_GetIWeight(PID, new_error);
    // Sum weights and Update output
    PID_t sum = (PID->PWeight + PID->IWeight);
    // Normalize sum
    PID_t scaled = (sum / SCALE_FACTOR);
    if (scaled > SCALE_MAX) {
        scaled = SCALE_MAX;
    }
    else if (scaled < SCALE_MIN) {
        scaled = SCALE_MIN;
    }
    // Give scaled value to CV
    PID->CV = scaled;
}
#endif
/*
Adjust PV of target PID
*/
void PID_Process(PID_Handle_t *PID, PID_t adjustment_factor)
{
    PID->PV += PID->CV * adjustment_factor;
}
/*
Zero PWeight, IWeight
*/
void PID_ClearWeights(PID_Handle_t * PID) {
    PID->PWeight = 0.0;
    PID->IWeight = 0.0;
    PID->IWeight = 0.0;
}
