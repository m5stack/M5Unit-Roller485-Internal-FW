//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C -
// Platform Independent
// 
// Revision: 1.1
// 
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
// 
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// 
// Revisions can be found here:
// https://github.com/tcleg
// 
// Modified by: Trent Cleghorn , <trentoncleghorn@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
// 
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// this program.  If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************************

//*********************************************************************************
// Headers
//*********************************************************************************
#include "main.h"
#include "pid_controller.h"

//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

#define FOC_PID_OUTPUT_RAMP 500000

//*********************************************************************************
// Functions
//*********************************************************************************
void PIDInit(PIDControl *pid, float kp, float ki, float kd, 
             float sampleTimeSeconds, float minOutput, float maxOutput, 
             PIDMode mode, PIDDirection controllerDirection)     	
{
    pid->normal_motor_pid_velocity_output_ramp = FOC_PID_OUTPUT_RAMP;
    pid->controllerDirection = controllerDirection;
    pid->mode = mode;
    pid->iTerm = 0.0f;
    pid->input = 0.0f;
    pid->lastInput = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    
    if(sampleTimeSeconds > 0.0f)
    {
        pid->sampleTime = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        pid->sampleTime = 1.0f;
    }
    
    PIDOutputLimitsSet(pid, minOutput, maxOutput);
    PIDTuningsSet(pid, kp, ki, kd);
}
        
bool
PIDCompute(PIDControl *pid) 
{
    float error, dInput;

    pid->my_timestamp_now = micros();
    pid->my_Ts = (pid->my_timestamp_now - pid->my_timestamp_prev) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if(pid->my_Ts <= 0 || pid->my_Ts > 0.5f) pid->my_Ts = 1e-3f;    

    if(pid->mode == MANUAL)
    {
        return false;
    }
    
    // The classic PID error term
    error = (pid->setpoint) - (pid->input);

    pid->point_error = error;
    
    // Compute the integral term separately ahead of time
    pid->iTerm += (pid->alteredKi) * error;
    
    // Constrain the integrator to make sure it does not exceed output bounds
    pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );
    
    // Take the "derivative on measurement" instead of "derivative on error"
    dInput = (pid->input) - (pid->lastInput);
    
    // Run all the terms together to get the overall output
    pid->output = (pid->alteredKp) * error + (pid->iTerm) - (pid->alteredKd) * dInput;
    
    // Bound the output
    pid->output = CONSTRAIN( (pid->output), (pid->outMin), (pid->outMax) );
    
    if(pid->normal_motor_pid_velocity_output_ramp > 0){
        // limit the acceleration by ramping the output
        pid->normal_output_rate = (pid->output - pid->output_prev)/pid->my_Ts;
        if (pid->normal_output_rate > pid->normal_motor_pid_velocity_output_ramp)
            pid->output = pid->output_prev + pid->normal_motor_pid_velocity_output_ramp*pid->my_Ts;
        else if (pid->normal_output_rate < -pid->normal_motor_pid_velocity_output_ramp)
            pid->output = pid->output_prev - pid->normal_motor_pid_velocity_output_ramp*pid->my_Ts;
    }    
    
    // Make the current input the former input
    pid->lastInput = pid->input;
    pid->output_prev = pid->output;
    pid->my_timestamp_prev = pid->my_timestamp_now;
    
    return true;
}
     
void 
PIDModeSet(PIDControl *pid, PIDMode mode)                                                                                                                                       
{
    // If the mode changed from MANUAL to AUTOMATIC
    if(pid->mode != mode && mode == AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        pid->iTerm = pid->output;
        pid->lastInput = pid->input;
        
        // Constrain the integrator to make sure it does not exceed output bounds
        pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );
    }
    
    pid->mode = mode;
}

void 
PIDOutputLimitsSet(PIDControl *pid, float min, float max) 							  							  
{
    // Check if the parameters are valid
    if(min >= max)
    {
        return;
    }
    
    // Save the parameters
    pid->outMin = min;
    pid->outMax = max;
    
    // If in automatic, apply the new constraints
    if(pid->mode == AUTOMATIC)
    {
        pid->output = CONSTRAIN(pid->output, min, max);
        pid->iTerm  = CONSTRAIN(pid->iTerm,  min, max);
    }
}

void 
PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd)         	                                         
{
    // Check if the parameters are valid
    if(kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }
    
    // Save the parameters for displaying purposes
    pid->dispKp = kp;
    pid->dispKi = ki;
    pid->dispKd = kd;
    
    // Alter the parameters for PID
    pid->alteredKp = kp;
    pid->alteredKi = ki;
    pid->alteredKd = kd;
    
    // Apply reverse direction to the altered values if necessary
    if(pid->controllerDirection == REVERSE)
    {
        pid->alteredKp = -(pid->alteredKp);
        pid->alteredKi = -(pid->alteredKi);
        pid->alteredKd = -(pid->alteredKd);
    }
}

void 
PIDTuningKpSet(PIDControl *pid, float kp)
{
    PIDTuningsSet(pid, kp, pid->dispKi, pid->dispKd);
}

void 
PIDTuningKiSet(PIDControl *pid, float ki)
{
    PIDTuningsSet(pid, pid->dispKp, ki, pid->dispKd);
}

void 
PIDTuningKdSet(PIDControl *pid, float kd)
{
    PIDTuningsSet(pid, pid->dispKp, pid->dispKi, kd);
}

void 
PIDControllerDirectionSet(PIDControl *pid, PIDDirection controllerDirection)	  									  									  									  
{
    // If in automatic mode and the controller's sense of direction is reversed
    if(pid->mode == AUTOMATIC && controllerDirection == REVERSE)
    {
        // Reverse sense of direction of PID gain constants
        pid->alteredKp = -(pid->alteredKp);
        pid->alteredKi = -(pid->alteredKi);
        pid->alteredKd = -(pid->alteredKd);
    }
    
    pid->controllerDirection = controllerDirection;
}

void 
PIDSampleTimeSet(PIDControl *pid, float sampleTimeSeconds)                                                       									  									  									   
{
    float ratio;

    if(sampleTimeSeconds > 0.0f)
    {
        // Find the ratio of change and apply to the altered values
        ratio = sampleTimeSeconds / pid->sampleTime;
        pid->alteredKi *= ratio;
        pid->alteredKd /= ratio;
        
        // Save the new sampling time
        pid->sampleTime = sampleTimeSeconds;
    }
}


