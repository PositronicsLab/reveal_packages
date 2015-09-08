#include <stdio.h>

#include <reveal/analytics/types.h>
#include <reveal/core/pointers.h>
#include <reveal/core/solution_set.h>
#include <reveal/core/solution.h>
#include <reveal/core/scenario.h>
#include <reveal/core/trial.h>
#include <reveal/core/analyzer.h>

#include <Ravelin/Origin3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Quatd.h>
#include <Ravelin/Matrix3d.h>
#include <Ravelin/Pose3d.h>

//-----------------------------------------------------------------------------

using namespace Reveal::Analytics;

//-------------------------------------------------------------------------
Reveal::Core::model_ptr arm_model( Reveal::Core::solution_ptr solution ) {
  Reveal::Core::model_ptr nothing;

  for( unsigned i = 0; i < solution->models.size(); i++ )
    if( solution->models[i]->id == "ur10_schunk_arm" ) return solution->models[i];

  return nothing;
}

//-------------------------------------------------------------------------
Reveal::Core::model_ptr target_model( Reveal::Core::solution_ptr solution ) {
  Reveal::Core::model_ptr nothing;

  for( unsigned i = 0; i < solution->models.size(); i++ )
    if( solution->models[i]->id == "block" ) return solution->models[i];

  return nothing;
}

//-------------------------------------------------------------------------
Reveal::Core::link_ptr finger_l_link( Reveal::Core::model_ptr model ) {
  Reveal::Core::link_ptr nothing;

  if( model->id != "ur10_schunk_arm" ) return nothing;
  for( unsigned i = 0; i < model->links.size(); i++ )
    if( model->links[i]->id == "schunk_mpg_80::l_finger" ) return model->links[i];

  return nothing;
}

//-------------------------------------------------------------------------
Reveal::Core::link_ptr finger_r_link( Reveal::Core::model_ptr model ) {
  Reveal::Core::link_ptr nothing;

  if( model->id != "ur10_schunk_arm" ) return nothing;
  for( unsigned i = 0; i < model->links.size(); i++ )
    if( model->links[i]->id == "schunk_mpg_80::r_finger" ) return model->links[i];

  return nothing;
}

//-------------------------------------------------------------------------
Reveal::Core::link_ptr block_link( Reveal::Core::model_ptr model ) {
  Reveal::Core::link_ptr nothing;

  if( model->id != "block" ) return nothing;
  for( unsigned i = 0; i < model->links.size(); i++ ) 
    if( model->links[i]->id == "body" ) return model->links[i];

  return nothing;
}

//-------------------------------------------------------------------------
// Ravelin math
//-------------------------------------------------------------------------
Ravelin::Origin3d position( const Reveal::Core::link_state_c& state ) {
  return Ravelin::Origin3d( state[0], state[1], state[2] );
}

//-------------------------------------------------------------------------
Ravelin::Quatd rotation( const Reveal::Core::link_state_c& state ) {
  return Ravelin::Quatd( state[3], state[4], state[5], state[6] );
}

//-------------------------------------------------------------------------
Ravelin::Pose3d pose( const Reveal::Core::link_state_c& state ) {
  return Ravelin::Pose3d( rotation( state ), position( state ) );
}

//-------------------------------------------------------------------------
Ravelin::Origin3d linear_velocity( const Reveal::Core::link_state_c& state ) {
  return Ravelin::Origin3d( state[7], state[8], state[9] );
}

//-------------------------------------------------------------------------
Ravelin::Origin3d angular_velocity( const Reveal::Core::link_state_c& state ) {
  return Ravelin::Origin3d( state[10], state[11], state[12] );
}

//-------------------------------------------------------------------------
Ravelin::Origin3d to_omega( const Ravelin::Quatd& q, const Ravelin::Quatd& qd ) {
  Ravelin::Vector3d omega = Ravelin::Quatd::to_omega( q, qd ); 
  return Ravelin::Origin3d( omega.data() );
}

//-------------------------------------------------------------------------
Ravelin::Quatd deriv( const Ravelin::Quatd& q, const Ravelin::Origin3d& w ) {
  Ravelin::Vector3d x = Ravelin::Vector3d( w.data() );
  return Ravelin::Quatd::deriv( q, x );
}

//-------------------------------------------------------------------------
double energy( double mass, Ravelin::Matrix3d I_tensor, double dt, Ravelin::Pose3d current_pose, Ravelin::Pose3d desired_pose, Ravelin::Origin3d current_linvel, Ravelin::Origin3d desired_linvel, Ravelin::Origin3d current_angvel, Ravelin::Origin3d desired_angvel ) {

  Ravelin::Origin3d x = current_pose.x;
  Ravelin::Origin3d x_star = desired_pose.x;
  Ravelin::Origin3d xd = current_linvel;
  Ravelin::Origin3d xd_star = desired_linvel;
  Ravelin::Quatd q = current_pose.q;
  Ravelin::Quatd q_star = desired_pose.q;

  Ravelin::Origin3d thetad = current_angvel;
  Ravelin::Origin3d thetad_star = desired_angvel;

  Ravelin::Matrix3d R(q);
  Ravelin::Matrix3d Rt = Ravelin::Matrix3d::transpose(R);
  Ravelin::Matrix3d I_global = R * I_tensor * Rt;

/*
  std::cout << "q:" << q << std::endl;
  std::cout << "R:" << R << std::endl;
  std::cout << "I:" << I_tensor << std::endl;
  std::cout << "I_global:" << I_global << std::endl;
*/
  /*
  std::cout << "mass:" << mass << std::endl;
  std::cout << "I:" << I_tensor << std::endl;
  std::cout << "dt:" << dt << std::endl;
  std::cout << "x:" << x << std::endl;
  std::cout << "x_star:" << x_star << std::endl;
  std::cout << "xd:" << xd << std::endl;
  std::cout << "xd_star:" << xd_star << std::endl;
  std::cout << "q:" << q << std::endl;
  std::cout << "q_star:" << q_star << std::endl;
  std::cout << "thetad:" << thetad << std::endl;
  std::cout << "thetad_star:" << thetad_star << std::endl;
  */

  //Ravelin::Quatd qd = (q_star - q) * (1.0 / dt);
  //  std::cout << "qd:" << qd << std::endl;

  // assert qd not normalized
  //const double EPSILON = 1e8;
  //double mag = qd.x * qd.x + qd.y * qd.y + qd.z * qd.z + qd.w * qd.w;
  //std::cout << "mag:" << mag << std::endl;
  //assert( fabs( mag - 1.0 ) > EPSILON );

  // compute linear velocity needed to move target back to original position
  Ravelin::Origin3d v = (x_star - x) / dt + (xd_star - xd);

  // compute angular velocity needed to move target back to original orientation
  Ravelin::Quatd qd = (q_star - q) * (1.0 / dt);
  Ravelin::Origin3d omega = to_omega( q, qd ) + (thetad_star - thetad);

  // compute linear component of kinetic energy
  double KE_linear = 0.5 * v.dot( v ) * mass;

  // compute rotational component of kinetic energy
  double KE_rotational = 0.5 * omega.dot( I_global.mult( omega ) );

  return KE_linear + KE_rotational;
  //return KE_linear;
}
//-------------------------------------------------------------------------
double energy( Reveal::Core::link_ptr gripper, Reveal::Core::link_ptr target, Ravelin::Origin3d c_v, Ravelin::Origin3d c_omega, double mass, Ravelin::Matrix3d I, double dt ) {

  Ravelin::Origin3d target_pos = position( target->state );
  Ravelin::Quatd target_rot = rotation( target->state );
  Ravelin::Origin3d target_lvel = linear_velocity( target->state );
  Ravelin::Origin3d target_avel = angular_velocity( target->state );

  Ravelin::Origin3d gripper_pos = position( gripper->state );
  Ravelin::Quatd gripper_rot = rotation( gripper->state );
  Ravelin::Origin3d gripper_lvel = linear_velocity( gripper->state );
  Ravelin::Origin3d gripper_avel = angular_velocity( gripper->state );

  target_rot.normalize();
  gripper_rot.normalize();

  Ravelin::Pose3d x( target_rot, target_pos );

  //desired pose
  Ravelin::Pose3d x_des;

  Ravelin::Origin3d delta = gripper_pos - target_pos;
  //Ravelin::Pose3d rot( target_rot );
  //Ravelin::Pose3d rot( gripper_rot );
  boost::shared_ptr<Ravelin::Pose3d> rot( new Ravelin::Pose3d( gripper_rot ) );

  //rot.q.normalize();
  Ravelin::Vector3d _v = Ravelin::Vector3d( c_v.data() );
  Ravelin::Vector3d vrot = Ravelin::Pose3d::transform_vector( rot, _v );
  Ravelin::Origin3d v = Ravelin::Origin3d( vrot.data() );

  //std::cout << "c_v:" << c_v << std::endl;
  //std::cout << "rot:" << rot << std::endl;
  //std::cout << "c_v:" << c_v << std::endl;
  //std::cout << "v:" << v << std::endl;
  //std::cout << "v_norm:" << v.norm() << std::endl;
  //std::cout << "delta:" << delta << std::endl;
  //std::cout << "delta_norm:" << delta.norm() << std::endl;

  //v[0] = 0; v[1] = 0; v[2] = 0;
  //delta[0] = 0; delta[1] = 0; delta[2] = 0;

  x_des.x = target_pos + (delta - v);
  //x_des.x = (delta - v);

  Ravelin::Quatd dq = deriv( target_rot, c_omega );
  x_des.q = target_rot + dq;
  x_des.q.normalize();

  return energy( mass, I, dt, x, x_des, target_lvel, gripper_lvel, target_avel, gripper_avel );
}

//-------------------------------------------------------------------------
void get_initial_config( Reveal::Core::model_ptr arm, Reveal::Core::model_ptr target, Ravelin::Origin3d& c_v_l, Ravelin::Origin3d& c_v_r, Ravelin::Origin3d& c_omega_l, Ravelin::Origin3d& c_omega_r ) {

  Reveal::Core::link_ptr gripper_l = finger_l_link( arm );
  Reveal::Core::link_ptr gripper_r = finger_r_link( arm );
  Reveal::Core::link_ptr block = block_link( target );
  assert( gripper_l && gripper_r && block );

  // get positions
  Ravelin::Origin3d pos_gripper_l = position( gripper_l->state );
  Ravelin::Origin3d pos_gripper_r = position( gripper_r->state );
  Ravelin::Origin3d pos_block = position( block->state ); 
  
  // get rotations
  Ravelin::Quatd rot_gripper_l = rotation( gripper_l->state );
  Ravelin::Quatd rot_gripper_r = rotation( gripper_r->state );
  Ravelin::Quatd rot_block = rotation( block->state ); 
 
  // initial desired velocities
  // left gripper energy constants
  c_v_l = pos_gripper_l - pos_block;
  c_omega_l = to_omega( rot_gripper_l, rot_block );
  // right gripper energy constants
  c_v_r = pos_gripper_r - pos_block;
  c_omega_r = to_omega( rot_gripper_r, rot_block );

/*
  std::cout << "c_v_l: " << c_v_l << std::endl;
  std::cout << "c_v_r: " << c_v_r << std::endl;
  std::cout << "pos_gripper_l: " << pos_gripper_l << std::endl;
  std::cout << "pos_gripper_r: " << pos_gripper_r << std::endl;
  std::cout << "pos_block: " << pos_block << std::endl;
  std::cout << "rot_gripper_l: " << rot_gripper_l << std::endl;
  std::cout << "rot_gripper_r: " << rot_gripper_r << std::endl;
  std::cout << "rot_block: " << rot_block << std::endl;
*/
}

//-----------------------------------------------------------------------------
/// Generates analyzer record data as a reveal analyzer pointer
/// @param scenario the scenario that this analyzer handles
/// @return an analyzer pointer that can be inserted into database or exported
Reveal::Core::analyzer_ptr generate_analyzer( void ) {
  Reveal::Core::analyzer_ptr analyzer( new Reveal::Core::analyzer_c() );
 
  analyzer->scenario_id = PACKAGE_NAME;
  analyzer->analyzer_id = ANALYZER_NAME;
  //analyzer->filename = ANALYZER_PATH;  //!
  analyzer->source_path = ANALYZER_SOURCE_PATH;
  analyzer->build_path = ANALYZER_BUILD_PATH;
  analyzer->build_target = ANALYZER_BUILD_TARGET;
  analyzer->type = Reveal::Core::analyzer_c::PLUGIN;

  analyzer->keys.push_back( "t" );
  analyzer->labels.push_back( "Virtual time (s)" );

  analyzer->keys.push_back( "KE" );
  analyzer->labels.push_back( "Average kinetic energy of block" );

  analyzer->keys.push_back( "real-time" );
  analyzer->labels.push_back( "Real time (s)" );

  return analyzer;
}


//-------------------------------------------------------------------------
extern "C" {

//-------------------------------------------------------------------------
error_e analyze( Reveal::Core::solution_set_ptr input, Reveal::Core::analysis_ptr& output ) {

  printf( "(analyzer) analyze started\n" );

  // Note: better handled with a thread pool.  For even moderate data set sizes
  // this takes a long time as a single thread

  Reveal::Core::model_ptr arm_t0, target_t0;

  // load model data for inertial properties.  This has to come from client
  // package or from other records inserted in response to client action.  
  // For now, this will be hard coded.

  // get the inertial properties of the box
  // this may need to be in a header for the experiment or read from the sdf
  double m = 0.12;
  Ravelin::Matrix3d I( 0.00001568, 0.0, 0.0, 
                   0.0, 0.00001568, 0.0, 
                   0.0, 0.0, 0.00001568 );

  //Reveal::Core::trial_ptr trial0 = input->initial_trial;
  Reveal::Core::solution_ptr initial_state = input->initial_state;

  // get the initial state.  Drawn from the trial data.
  arm_t0 = arm_model( initial_state );
  target_t0 = target_model( initial_state );

  // left gripper energy constants
  Ravelin::Origin3d c_v_l;
  Ravelin::Origin3d c_omega_l;
  // right gripper energy constants
  Ravelin::Origin3d c_v_r;
  Ravelin::Origin3d c_omega_r;

  // compute the initial energy from initial state.
  get_initial_config( arm_t0, target_t0, c_v_l, c_v_r, c_omega_l, c_omega_r );

  output = Reveal::Core::analysis_ptr( new Reveal::Core::analysis_c( input ) );

  output->add_key( "t" );
  output->add_key( "KE" );
  output->add_key( "real-time" );

  //printf( "input trials[%u], solutions[%u]", input->trials.size(), input->solutions.size() );

  // iterate over the client solutions and compute the energy for each sample
  for( unsigned i = 0; i < input->solutions.size(); i++ ) {
    Reveal::Core::solution_ptr solution = input->solutions[i];
    double t = solution->t;
    //double dt = input->experiment->time_step;//TODO:WARNING:this dt may be wrong
    double dt = input->time_step;
    double real_time = solution->real_time;

    //std::cout << "t:" << t << std::endl;
    //std::cout << "dt:" << dt << std::endl;

    //std::vector<double> avgKEs;

    Reveal::Core::model_ptr arm = arm_model( solution );
    Reveal::Core::model_ptr target = target_model( solution );
    assert( arm && target );

    Reveal::Core::link_ptr finger_l = finger_l_link( arm );
    Reveal::Core::link_ptr finger_r = finger_r_link( arm );
    Reveal::Core::link_ptr block = block_link( target );
    assert( finger_l && finger_r && block );

    double KE_l = energy( finger_l, block, c_v_l, c_omega_l, m, I, dt );
    double KE_r = energy( finger_r, block, c_v_r, c_omega_r, m, I, dt );

    double avg_KE = (KE_l + KE_r) / 2.0;

    //std::cout << "KE:" << avg_KE << std::endl;

    //printf( "t[%f], avgKE[%f], KE_l[%f], KE_r[%f]\n", t, avg_KE, KE_l, KE_r );
    std::vector<double> values;
    values.push_back( t );
    values.push_back( avg_KE );
    values.push_back( real_time );
    output->add_row( values );
  }

  printf( "(analyzer) analyze finished\n" );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

} // extern "C"

