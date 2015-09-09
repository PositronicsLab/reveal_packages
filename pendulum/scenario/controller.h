/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/

#ifndef _REVEAL_PACKAGE_PENDULUM_CONTROLLER_H_
#define _REVEAL_PACKAGE_PENDULUM_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include <string>
#include <map>

//-----------------------------------------------------------------------------
/// returns the initial velocity of all of the joints
/// @param[out] qd the map returned containing the velocities mapped to the 
///        name of the joint
void get_initial_velocity( std::map<std::string,double>& velocity ) {

  // the starting velocity for the pivot joint is zero
  velocity.insert( std::pair<std::string,double>("pivot_joint", 0.0) );
}

//-----------------------------------------------------------------------------
/// the control function which returns the set of torques computed to move the 
/// joint to the desired position with the desired velocity
/// @param t[in] the time at which to calculate the control
/// @param position[in] the map of joint positions mapped to the joint name
/// @param velocity[in] the map of joint velocities mapped to the joint name
/// @param force[out] the map returned containing the joint torques mapped 
///        joint name
void get_control( double t, std::map<std::string,double> position, std::map<std::string,double> velocity, std::map<std::string, double>& force ) {

  double q = position.find( "pivot_joint" )->second;
  double dq = velocity.find( "pivot_joint" )->second;

  // insert a null control for the pivot joint
  force.insert( std::pair<std::string,double>( "pivot_joint", 0.0 ) );  
}

//-----------------------------------------------------------------------------

#endif // _REVEAL_PACKAGE_PENDULUM_CONTROLLER_H_
