/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/

#ifndef _REVEAL_PACKAGE_ARM_CONTROLLER_H_
#define _REVEAL_PACKAGE_ARM_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include <string>
#include <map>

//-----------------------------------------------------------------------------
static const double PERIOD = 5.0;
static const double AMP = 0.5;
static const double SMALL_AMP = AMP * 0.1;

//-----------------------------------------------------------------------------
/// returns the initial velocity of all of the arm joints
/// @param qd the map returned containing the velocities mapped to the name of 
///        the joint
void get_initial_velocity( std::map<std::string,double>& qd ) {
  // Note: this is the same formulation for computing desired velocity used
  // in control function.

  double t = 0;
  // compute the starting velocity for the joints
  double sh_pan_qd = std::cos(t) * AMP * PERIOD;
  double sh_lift_qd = std::cos(t*2.0) * SMALL_AMP * PERIOD * 2.0;
  double elbow_qd = std::cos(t*2.0/3.0) * AMP * PERIOD * 2.0/3.0;
  double wrist1_qd = std::cos(t*1.0/7.0) * AMP * PERIOD * 1.0/7.0;
  double wrist2_qd = std::cos(t*2.0/11.0) * AMP * PERIOD * 2.0/11.0;
  double wrist3_qd = std::cos(t*3.0/13.0) * AMP * PERIOD * 3.0/13.0;

  qd.insert(std::pair<std::string,double>("shoulder_pan_joint", sh_pan_qd));
  qd.insert(std::pair<std::string,double>("shoulder_lift_joint", sh_lift_qd));
  qd.insert(std::pair<std::string,double>("elbow_joint", elbow_qd));
  qd.insert(std::pair<std::string,double>("wrist_1_joint", wrist1_qd));
  qd.insert(std::pair<std::string,double>("wrist_2_joint", wrist2_qd));
  qd.insert(std::pair<std::string,double>("wrist_3_joint", wrist3_qd));
}

//-----------------------------------------------------------------------------
/// the control function which returns the set of torques computed to move the 
/// arm to the desired position with the desired velocity
/// @param t the time at which to calculate the control
/// @param pos the map of the joint positions mapped to the name of the joint
/// @param vel the map of the joint velocities mapped to the name of the joint
/// @param force the map returned containing the joint torques mapped to the 
///        name of the joint
void get_control( double t, std::map<std::string,double> pos, std::map<std::string,double> vel, std::map<std::string,double>& force ) {
  // map in the state
  double sh_pan_q = pos.find( "shoulder_pan_joint" )->second;
  double sh_pan_qd = vel.find( "shoulder_pan_joint" )->second;
  double sh_lift_q = pos.find( "shoulder_lift_joint" )->second;
  double sh_lift_qd = vel.find( "shoulder_lift_joint" )->second;
  double elbow_q = pos.find( "elbow_joint" )->second;
  double elbow_qd = vel.find( "elbow_joint" )->second;
  double wrist1_q = pos.find( "wrist_1_joint" )->second;
  double wrist1_qd = vel.find( "wrist_1_joint" )->second;
  double wrist2_q = pos.find( "wrist_2_joint" )->second;
  double wrist2_qd = vel.find( "wrist_2_joint" )->second;
  double wrist3_q = pos.find( "wrist_3_joint" )->second;
  double wrist3_qd = vel.find( "wrist_3_joint" )->second;

  // determine the desired position and velocity for the controller 
  double sh_pan_q_des = std::sin(t) * AMP * PERIOD;
  double sh_pan_qd_des = std::cos(t) * AMP * PERIOD;
  double sh_lift_q_des = std::sin(t*2.0) * SMALL_AMP * PERIOD * 2.0;
  double sh_lift_qd_des = std::cos(t*2.0) * SMALL_AMP * PERIOD * 2.0;
  double elbow_q_des = std::sin(t*2.0/3.0) * AMP * PERIOD * 2.0/3.0;
  double elbow_qd_des = std::cos(t*2.0/3.0) * AMP * PERIOD * 2.0/3.0;
  double wrist1_q_des = std::sin(t*1.0/7.0) * AMP * PERIOD * 1.0/7.0;
  double wrist1_qd_des = std::cos(t*1.0/7.0) * AMP * PERIOD * 1.0/7.0;
  double wrist2_q_des = std::sin(t*2.0/11.0) * AMP * PERIOD * 2.0/11.0;
  double wrist2_qd_des = std::cos(t*2.0/11.0) * AMP * PERIOD * 2.0/11.0;
  double wrist3_q_des = std::sin(t*3.0/13.0) * AMP * PERIOD * 3.0/13.0;
  double wrist3_qd_des = std::cos(t*3.0/13.0) * AMP * PERIOD * 3.0/13.0;

  // compute the errors
  double sh_pan_q_err = sh_pan_q_des - sh_pan_q;
  double sh_pan_qd_err = sh_pan_qd_des - sh_pan_qd;
  double sh_lift_q_err = sh_lift_q_des - sh_lift_q;
  double sh_lift_qd_err = sh_lift_qd_des - sh_lift_qd;
  double elbow_q_err = elbow_q_des - elbow_q;
  double elbow_qd_err = elbow_qd_des - elbow_qd;
  double wrist1_q_err = wrist1_q_des - wrist1_q;
  double wrist1_qd_err = wrist1_qd_des - wrist1_qd;
  double wrist2_q_err = wrist2_q_des - wrist2_q;
  double wrist2_qd_err = wrist2_qd_des - wrist2_qd;
  double wrist3_q_err = wrist3_q_des - wrist3_q;
  double wrist3_qd_err = wrist3_qd_des - wrist3_qd;

  // setup gains
  const double SH_KP = 300.0, SH_KV = 120.0;
  const double EL_KP = 60.0, EL_KV = 24.0;
  const double WR_KP = 15.0, WR_KV = 6.0;

  // compute the actuator forces
  double sh_pan_f = SH_KP * sh_pan_q_err + SH_KV * sh_pan_qd_err;
  double sh_lift_f = SH_KP * sh_lift_q_err + SH_KV * sh_lift_qd_err;
  double elbow_f = EL_KP * elbow_q_err + EL_KV * elbow_qd_err;
  double wrist1_f = WR_KP * wrist1_q_err + WR_KV * wrist1_qd_err;
  double wrist2_f = WR_KP * wrist2_q_err + WR_KV * wrist2_qd_err;
  double wrist3_f = WR_KP * wrist3_q_err + WR_KV * wrist3_qd_err;

  // close fingers forces
  double finger_l_f =  100;
  double finger_r_f = -100;
  
  // map out the forces
  force.insert(std::pair<std::string,double>("shoulder_pan_joint", sh_pan_f));
  force.insert(std::pair<std::string,double>("shoulder_lift_joint", sh_lift_f));
  force.insert(std::pair<std::string,double>("elbow_joint", elbow_f));
  force.insert(std::pair<std::string,double>("wrist_1_joint", wrist1_f));
  force.insert(std::pair<std::string,double>("wrist_2_joint", wrist2_f));
  force.insert(std::pair<std::string,double>("wrist_3_joint", wrist3_f));
  force.insert(std::pair<std::string,double>("l_finger_actuator", finger_l_f));
  force.insert( std::pair<std::string,double>("r_finger_actuator", finger_r_f));
}

//-----------------------------------------------------------------------------

#endif // _REVEAL_PACKAGE_ARM_CONTROLLER_H_
