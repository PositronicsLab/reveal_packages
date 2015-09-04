/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/

#ifndef _REVEAL_PACKAGE_PENDULUM_CONTROLLER_H_
#define _REVEAL_PACKAGE_PENDULUM_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include <string>
#include <map>

//-----------------------------------------------------------------------------
/*
extern double position_gain;               // positional PD gain
extern double velocity_gain;               // velocity PD gain

extern double initial_time;                // start time of the simulation
extern double final_time;                  // goal time to complete the control
extern double initial_position;            // start position of the pivot joint
extern double final_position;              // goal position of the pivot joint
*/
//-----------------------------------------------------------------------------
/// returns the initial velocity of all of the joints
/// @param qd the map returned containing the velocities mapped to the name of 
///        the joint
void get_initial_velocity( std::map<std::string,double>& qd ) {

  // the starting velocity for the pivot joint is zero
  qd.insert( std::pair<std::string,double>("pivot_joint", 0.0) );
}

//-----------------------------------------------------------------------------
/// the control function which returns the set of torques computed to move the 
/// joint to the desired position with the desired velocity
/// @param t the time at which to calculate the control
/// @param pos the map of joint positions mapped to the joint name
/// @param vel the map of joint velocities mapped to the joint name
/// @param force the map returned containing the joint torques mapped joint name
void get_control( double t, std::map<std::string,double> pos, std::map<std::string,double> vel, std::map<std::string, double>& force ) {
/*
  // initial and goal parameters (provided externally)
  double q_0 = initial_position;
  double q_f = final_position;
  double t_0 = initial_time;
  double t_f = final_time;

  // gains (provided externally)
  double Kp = position_gain;
  double Kv = velocity_gain;

  // compute time exponentials
  double t3_f = t_f * t_f * t_f;           // final time cubed
  double t2_f = t_f * t_f;                 // final time squared
  double t3 = t * t * t;                   // current time cubed
  double t2 = t * t;                       // current time squared

  // state and control variables
  double q, q_des, q_err;                  // position
  double dq, dq_des, dq_err;               // velocity
  double f;                                // force

  // compute the differential between initial and final state
  double delta_q = q_f - q_0;

  // map in actual state to state parameters
  q = pos.find( "pivot_joint" )->second;   // current position
  dq = vel.find( "pivot_joint" )->second;  // current velocity
 
  // compute desired state
  if( t <= t_f ) {
    // if time is less than final time, use the cubic trajectory equations
    // to compute desired position and velocity
    q_des =(-2.0 * delta_q / t3_f) * t3  +  (3.0 * delta_q / t2_f) * t2  +  q_0;
    dq_des=(-6.0 * delta_q / t3_f) * t2  +  (6.0 * delta_q / t2_f) * t   +  0;
  } else {
    // otherwise, use the goal state as desired position and velocity
    q_des = q_f;
    dq_des = 0.0;                          // to balance must have zero velocity
  }

  // compute error between desired and actual state
  q_err = q_des - q;                       // position error
  dq_err = dq_des - dq;                    // velocity error

  // compute force
  f = Kp * q_err + Kv * dq_err;            // PD force control

  // map out force
  force.insert( std::pair<std::string,double>("pivot_joint", f) );
*/

  force.insert( std::pair<std::string,double>("pivot_joint", 0.0) );  
}

//-----------------------------------------------------------------------------

#endif // _REVEAL_PACKAGE_PENDULUM_CONTROLLER_H_
