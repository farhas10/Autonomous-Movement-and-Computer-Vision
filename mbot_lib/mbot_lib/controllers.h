/**
 * File: controllers.h
 * 
 * Definitions for feedback controllers.
 */

#ifndef MBOT_LIB_CONTROLLERS_H
#define MBOT_LIB_CONTROLLERS_H

/**
 * Computes the output of bang-bang control for a single point in time.
 * @param   current The current value of the controlled variable.
 * @param   setpoint The setpoint value of the controlled variable.
 * @param   scaling The scaling to apply to the output before return.
 * @param   tolerance The error tolerance to apply to the contorlled variable
 * @return  The scaled control to be applied.
 */
float bangBangControl(float current, float setpoint, float scaling, float tolerance = .05);

/**
 * Computes the output of p control for a single point in time.
 * @param  current The current value of the controlled variable.
 * @param  setpoint The setpoint value of the controlled variable.
 * @param  kp The p gain of the controller.
 * @return  The control to be applied.
 */
float pControl(float current, float setpoint, float kp);

#endif  // MBOT_LIB_CONTROLLERS_H