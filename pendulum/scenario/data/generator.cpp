#define ODEINT_V2                  // odeint versioning
#define ODEINT_RUNGEKUTTA_STEPPER  // which stepper to use
//#define ODEINT_EULER_STEPPER

//----------------------------------------------------------------------------

#include "pendulum.h"

#include <stdio.h>

#include <reveal/core/pointers.h>
#include <reveal/core/system.h>

#include <reveal/core/model.h>
#include <reveal/core/scenario.h>
#include <reveal/core/analyzer.h>
#include <reveal/core/trial.h>
#include <reveal/core/solution.h>

#include <reveal/core/exporter.h>

#ifdef DB_DIRECT_INSERT
#ifndef _REVEAL_SERVER_SERVER_H_
#define _REVEAL_SERVER_SERVER_H_
#endif // _REVEAL_SERVER_SERVER_H_
#include <reveal/db/database.h>
#endif  // DB_DIRECT_INSERT

//----------------------------------------------------------------------------
// Constants
#define PI 3.14159265359

//----------------------------------------------------------------------------
// Scenario Defintion Values
double _sample_rate = 0.001;
double _start_time = 0.0;
double _end_time = 10.0;

//----------------------------------------------------------------------------
// Initial state
#define PENDULUM_LENGTH    1.0
#define INITIAL_q          PI / 2.0
#define INITIAL_dq         0.0

//----------------------------------------------------------------------------
// Variables
pendulum_ptr _pendulum;                //< the pendulum ODEs

#if defined( ODEINT_V1 )
stepper_t _stepper;                    //< odeint version 1 stepper structure
#endif

Reveal::Core::scenario_ptr _scenario;  //< scenario definition
Reveal::Core::analyzer_ptr _analyzer;  //< analyzer definition

/// exporter writes all scenario data to a compatible Reveal import format
Reveal::Core::exporter_c _exporter;    

#ifdef DB_DIRECT_INSERT
boost::shared_ptr<Reveal::DB::database_c> _db;  //< the local reveal database
#endif // DB_DIRECT_INSERT

/// Accurate time spec to correct error from inaccurate floating point addition
long _sec;      //< seconds portion of current time
long _nsec;     //< nanoseconds portion of current time

//----------------------------------------------------------------------------
/// Reads the currently stored time and returns as a floating point value
/// @return the current time in a double format
double get_time( void ) {
  return (double)_sec + ((double)_nsec / (double)1E9);
}

//----------------------------------------------------------------------------
/// Converts the floating point time step into a more accurate representation,
/// updates the stored time based on the time step, returns the current time
/// @param dt the time step to add to the currently stored time
double update_time( double dt ) {
  // split the time differentials into whole seconds and whole nanoseconds
  double dt_sec = floor( dt );
  double dt_nsec = dt - dt_sec;

  // convert the floating point time differentials to long differentials
  long nsec_step = (long) (dt_nsec * 1E9);
  long sec_step = (long) dt_sec;

  // increment the stored time using the long differentials
  _nsec += nsec_step;
  _sec += sec_step;

  // if nanoseconds has accrued a second through increment, correct the values
  if( _nsec > 1E9 ) { 
    _nsec -= 1E9;
    _sec += 1;
  }

  // get the newly updated time and return it as a floating point value
  return get_time();
}

//----------------------------------------------------------------------------
/// Creates a fixed base link reference
/// @return a Reveal link defining the fixed base link state in the world frame
Reveal::Core::link_ptr base_link( void ) {
  // base is positoned at 0,0,0 and rotated 90 degrees to compensate for model
  // definition in sdf.  The link is fixed to the world and cannot move.

  // create the link object
  Reveal::Core::link_ptr base( new Reveal::Core::link_c( "base_link" ) );

  // set the link state relative to the world frame
  base->state.linear_x( 0.0 );
  base->state.linear_y( 0.0 );
  base->state.linear_z( 0.0 );
  base->state.angular_x( PI / 4 );
  base->state.angular_y( 0.0 );
  base->state.angular_z( 0.0 );
  base->state.angular_w( PI / 4 );
  base->state.linear_dx( 0.0 );
  base->state.linear_dy( 0.0 );
  base->state.linear_dz( 0.0 );
  base->state.angular_dx( 0.0 );
  base->state.angular_dy( 0.0 );
  base->state.angular_dz( 0.0 );

  // return the link object
  return base;
}

//----------------------------------------------------------------------------
/// Creates a joint for the pivot
/// @return a Reveal joint defining the current state of the pivot joint
Reveal::Core::joint_ptr pivot_joint( void ) {
  // pivot rotates based on the current ode computation.  reads the ode state
  // and writes that state into a reveal joint

  // create the joint object
  Reveal::Core::joint_ptr pivot( new Reveal::Core::joint_c( "pivot_joint" ) );

  // define the number of state and controls needed for this joint
  pivot->state.resize( 1 );          // joint has 1 DOF
  pivot->control.resize( 0 );        // joint uses no controls

  // Note: the model is rotated 90 degrees in the scenario world frame from 
  // its model frame so the state is adjusted accordingly
  pivot->state.q( 0, _pendulum->q - PI/2.0 );  // set position
  pivot->state.dq( 0, _pendulum->dq );         // set velocity

  // return the joint object
  return pivot;
}

//----------------------------------------------------------------------------
/// Creates a scenario definition
/// @return the scenario definition for the pendulum scenario
Reveal::Core::scenario_ptr define_scenario( void ) {

  // create the scenario object
  Reveal::Core::scenario_ptr scenario( new Reveal::Core::scenario_c() );

  // set the scenario parameters
  scenario->id = generate_uuid();
  scenario->package_id = "pendulum";
  scenario->description = "a pendulum acting under gravity alone";
  scenario->sample_rate = _sample_rate;
  scenario->sample_start_time = _start_time;
  scenario->sample_end_time = _end_time;

  // return the scenario object
  return scenario;
}

//----------------------------------------------------------------------------
/// Creates an analyzer definition
/// @return the analyzer definition for the pendulum scenario
Reveal::Core::analyzer_ptr define_analyzer( Reveal::Core::scenario_ptr scenario ) {

  // create the analyzer object
  Reveal::Core::analyzer_ptr analyzer( new Reveal::Core::analyzer_c() );

  // set the analyzer parameters
  analyzer->scenario_id = scenario->id;
  analyzer->source_path = "pendulum/analyzers/pivot_angle";
  analyzer->build_path = "pendulum/analyzers/pivot_angle/build";
  analyzer->build_target = "libanalyzer.so";
  analyzer->type = Reveal::Core::analyzer_c::PLUGIN;

  // define the keys and user friendly labels for the analysis
  analyzer->keys.push_back( "t" );
  analyzer->labels.push_back( "Virtual time (s)" );

  analyzer->keys.push_back( "delta" );
  analyzer->labels.push_back( "Joint angle error" );

  // return the analyzer object
  return analyzer;
}

//----------------------------------------------------------------------------
/// Creates a trial definition
/// @param scenario the scenario definition who will own this trial
/// @param t the time of the trial
/// @return the trial definition generated for the current time
Reveal::Core::trial_ptr define_trial( Reveal::Core::scenario_ptr scenario, double t ) {

  // create the trial object
  Reveal::Core::trial_ptr trial( new Reveal::Core::trial_c() );

  // set the trial parameters
  trial->scenario_id = scenario->id;
  trial->t = t;

  // define all the models for this trial
  Reveal::Core::model_ptr model( new Reveal::Core::model_c( "pendulum" ) );

  model->insert( base_link() );
  model->insert( pivot_joint() );

  trial->models.push_back( model );

  // return the trial object
  return trial;
}

//----------------------------------------------------------------------------
/// Creates a model solution definition
/// @param scenario the scenario definition who will own this solution
/// @param t the time of the solution
/// @return the model solution definition generated for the current time
Reveal::Core::solution_ptr define_solution( Reveal::Core::scenario_ptr scenario, double t ) {

  // create the model solution object
  Reveal::Core::solution_ptr solution( new Reveal::Core::solution_c( Reveal::Core::solution_c::MODEL ) );

  // set the solution parameters
  solution->scenario_id = scenario->id;
  solution->t = t;

  // define all the models for this solution
  Reveal::Core::model_ptr model( new Reveal::Core::model_c( "pendulum" ) );

  model->insert( base_link() );
  model->insert( pivot_joint() );

  solution->models.push_back( model );

  // return the solution object
  return solution;
}

//----------------------------------------------------------------------------
/// Initializes all objects necessary to generate data
/// @return returns true if data generation is ready to proceed OR false if 
///         the operation fails for any reason
bool init( void ) {

  // create the pendulum
  double l = PENDULUM_LENGTH;
  double q_0 = INITIAL_q;
  double dq_0 = INITIAL_dq;
  _pendulum = pendulum_ptr( new pendulum_c( l, q_0, dq_0 ) );

  // intialize time which internally maintained as longs for accuracy
  _sec = 0;
  _nsec = 0;

#if defined( ODEINT_V1 )
  // initialize integrators
  // ODEINT_V1 requires the integrator to be setup before use
  state_t x( 2 );
  _stepper.adjust_size( x );
#endif

  // get the scenario and analyzer definitions
  _scenario = define_scenario();
  _analyzer = define_analyzer( _scenario );

#ifdef DB_DIRECT_INSERT
  // if the data is being directly inserted into the database, open the db
  _db = boost::shared_ptr<Reveal::DB::database_c>( new Reveal::DB::database_c() );
  if( !_db->open() ) {
    printf( "ERROR: failed to open database\n" );
    return false;
  }

  // then insert the scenario and analyzer records
  if( _db->insert( _scenario ) != Reveal::DB::database_c::ERROR_NONE ) {
    printf( "ERROR: failed to insert scenario into database\n" );
    _db->close();
    return false;
  }
  if( _db->insert( _analyzer ) != Reveal::DB::database_c::ERROR_NONE ) {
    printf( "ERROR: failed to insert analyzer into database\n" );
    _db->close();
    // Note: if this fails, scenario transaction is not rolled back
    return false;
  }
#endif // DB_DIRECT_INSERT

  return true;
}

//----------------------------------------------------------------------------
/// Cleans up all components that expect a clean exit
void shutdown( void ) {
#ifdef DB_DIRECT_INSERT
  // close the database
  _db->close();
#endif // DB_DIRECT_INSERT
}

//----------------------------------------------------------------------------
/// Integrates the pendulum forward and updates subsequent state
/// @param[in] dt the integration step size
/// @param[in] t0 the time preceding integration
/// @param[out] t1 the time after integration has completed
void step( double dt, double t0, double& t1 ) {
  state_t x( 2 );             // odeint compatible state vector

  // update the state vector with current state
  x[0] = _pendulum->q;
  x[1] = _pendulum->dq;

  // compute the end time t1
  t1 = t0 + dt;

  // integrate
#if defined( ODEINT_V1 )
  _stepper.do_step( *_pendulum.get(), x, t0, dt );
#elif defined( ODEINT_V2 )
  boost::numeric::odeint::integrate_adaptive( stepper_t(), *_pendulum.get(), x, t0, t1, dt );
#endif

  // update the pendulum state data structure from the odeint computed state
  _pendulum->q = x[0];
  _pendulum->dq = x[1];
}

//----------------------------------------------------------------------------
/// The generator application produces a set of Reveal import compatible files
/// defining the pendulum scenario for a pendulum acting under gravity with no
/// control inputs.  The pendulum is started at an angle orthogonal to the 
/// equilibrium axis and oscillates back and forth for a number of seconds.
int main( void ) {

  double t, tf, dt;                    // time variables
  Reveal::Core::trial_ptr trial;       // the current trial definition
  Reveal::Core::solution_ptr solution; // the current solution definition

  // initialize all components that support the generator
  if( !init() ) {
    printf( "ERROR: Initialization failed.\nExiting\n" );
    shutdown();
    return 1;
  }

  // initialize local time variables
  t = _start_time;
  dt = _sample_rate;

  // compute initial state; also acts as a solution prototype
  Reveal::Core::solution_ptr initial_state = define_solution( _scenario, t );

  // get a prototype for the trial; solution prototype is initial_state
  trial = define_trial( _scenario, t );
 
  // export the scenario framework.
  // Note: the trial and initial_state content is not actually written, but 
  // the structure of all models contained in them is used to map fields to
  // joint and link ids when actual data is available
  bool result = _exporter.write( _scenario, _analyzer, initial_state, trial );
  if( !result ) {
    printf( "ERROR: Failed to write export framework.\nExiting\n" );
    shutdown();
    return 2; 
  }

  // write initial state
  _exporter.write( initial_state );
#ifdef DB_DIRECT_INSERT
  _db->insert( initial_state );
#endif // DB_DIRECT_INSERT

  // simulate until the end time is reached
  while( t <= _end_time ) {

    // define the trial for the current time
    trial = define_trial( _scenario, t );

    // integrate the pendulum state forward an amount of time dt beginning at
    // time t.  tf is computed (and not necessary to this implementation)
    step( dt, t, tf );

    // update time using a more accurate time method rather than relying on tf
    t = update_time( dt );

    // define the solution for time t (now corresponds to time tf)
    solution = define_solution( _scenario, t );

    // write the trial and solution
    _exporter.write( trial );
    _exporter.write( solution );
#ifdef DB_DIRECT_INSERT
    _db->insert( trial );
    _db->insert( solution );
#endif // DB_DIRECT_INSERT

    // output the state for debugging
    std::cout << t << " " << _pendulum->q << " " << _pendulum->dq << std::endl;
  }

#ifdef DB_DIRECT_INSERT
  /// clean up
  _db->close();
#endif // DB_DIRECT_INSERT

  // report success and exit cleanly
  printf( "Data generation succeeded\n" );
  shutdown();
  return 0;
}

//-----------------------------------------------------------------------------
