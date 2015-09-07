#define ODEINT_V2                  // odeint versioning
#define ODEINT_RUNGEKUTTA_STEPPER  // which stepper to use
//#define ODEINT_EULER_STEPPER

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------
//Constants
#define PI 3.14159265359

double _sample_rate = 0.001;
double _start_time = 0.0;
double _end_time = 10.0;
//-----------------------------------------------------------------------------
// Initial state
#define PENDULUM_LENGTH    1.0
#define INITIAL_q          PI / 2.0
#define INITIAL_dq         0.0

//-----------------------------------------------------------------------------
// Variables
pendulum_ptr _pendulum;

#if defined( ODEINT_V1 )
stepper_t _stepper;
#endif

Reveal::Core::scenario_ptr _scenario;
Reveal::Core::analyzer_ptr _analyzer;

/// the exporter handles exporting all the scenario data to requisite files
Reveal::Core::exporter_c _exporter;

#ifdef DB_DIRECT_INSERT
/// the local reveal database
boost::shared_ptr<Reveal::DB::database_c> _db;
#endif // DB_DIRECT_INSERT

long _sec;
long _nsec;

//-----------------------------------------------------------------------------
double get_time( void ) {
  return (double)_sec + ((double)_nsec / (double)1E9);
}

//-----------------------------------------------------------------------------
double update_time( double dt ) {
  double dt_sec = floor( dt );
  double dt_nsec = dt - dt_sec;

  long nsec_step = (long) (dt_nsec * 1E9);
  long sec_step = (long) dt_sec;

  _nsec += nsec_step;
  _sec += sec_step;
  if( _nsec > 1E9 ) { 
    _nsec -= 1E9;
    _sec += 1;
  }

  return (double)_sec + ((double)_nsec / (double)1E9);
}

//-----------------------------------------------------------------------------
Reveal::Core::link_ptr base_link( void ) {
  Reveal::Core::link_ptr base( new Reveal::Core::link_c( "base_link" ) );
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
  return base;
}

//-----------------------------------------------------------------------------
Reveal::Core::joint_ptr pivot_joint( void ) {
  Reveal::Core::joint_ptr pivot( new Reveal::Core::joint_c( "pivot_joint" ) );

  pivot->state.resize( 1 );          // joint has 1 DOF
  pivot->control.resize( 0 );        // joint uses no controls

  // Note: the model is turned 90 degrees in the scenario world frame from its 
  // model frame
  pivot->state.q( 0, _pendulum->q - PI/2.0 );
  pivot->state.dq( 0, _pendulum->dq );

  return pivot;
}

//-----------------------------------------------------------------------------
Reveal::Core::scenario_ptr define_scenario( void ) {
  Reveal::Core::scenario_ptr scenario( new Reveal::Core::scenario_c() );

  scenario->id = generate_uuid();
  scenario->package_id = "pendulum";
  scenario->description = "a pendulum acting under gravity alone";
  scenario->sample_rate = _sample_rate;
  scenario->sample_start_time = _start_time;
  scenario->sample_end_time = _end_time;

  return scenario;
}

//-----------------------------------------------------------------------------
Reveal::Core::analyzer_ptr define_analyzer( Reveal::Core::scenario_ptr scenario ) {
  Reveal::Core::analyzer_ptr analyzer( new Reveal::Core::analyzer_c() );

  analyzer->scenario_id = scenario->id;
  analyzer->source_path = "pendulum/analyzers/pivot_angle";
  analyzer->build_path = "pendulum/analyzers/pivot_angle/build";
  analyzer->build_target = "libanalyzer.so";
  analyzer->type = Reveal::Core::analyzer_c::PLUGIN;

  analyzer->keys.push_back( "t" );
  analyzer->labels.push_back( "Virtual time (s)" );

  analyzer->keys.push_back( "delta" );
  analyzer->labels.push_back( "Joint angle error" );

  return analyzer;
}

//-----------------------------------------------------------------------------
Reveal::Core::trial_ptr define_trial( Reveal::Core::scenario_ptr scenario, double t ) {
  Reveal::Core::trial_ptr trial( new Reveal::Core::trial_c() );

  trial->scenario_id = scenario->id;
  trial->t = t;

  Reveal::Core::model_ptr model( new Reveal::Core::model_c( "pendulum" ) );

  model->insert( base_link() );
  model->insert( pivot_joint() );

  trial->models.push_back( model );

  return trial;
}

//-----------------------------------------------------------------------------
Reveal::Core::solution_ptr define_solution( Reveal::Core::scenario_ptr scenario, double t ) {
  Reveal::Core::solution_ptr solution( new Reveal::Core::solution_c( Reveal::Core::solution_c::MODEL ) );

  solution->scenario_id = scenario->id;
  solution->t = t;

  Reveal::Core::model_ptr model( new Reveal::Core::model_c( "pendulum" ) );

  model->insert( base_link() );
  model->insert( pivot_joint() );

  solution->models.push_back( model );

  return solution;
}

//-----------------------------------------------------------------------------
bool init( void ) {
  // create the pendulum
  double l = PENDULUM_LENGTH;
  double q_0 = INITIAL_q;
  double dq_0 = INITIAL_dq;
  _pendulum = pendulum_ptr( new pendulum_c( l, q_0, dq_0 ) );

  // time is actually maintained as longs in _sec + _nsec
  _sec = 0;
  _nsec = 0;

  // initialize integrators
#if defined( ODEINT_V1 )
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
    return false;
  }
#endif // DB_DIRECT_INSERT

  return true;
}

//-----------------------------------------------------------------------------
void shutdown( void ) {
#ifdef DB_DIRECT_INSERT
  _db->close();
#endif // DB_DIRECT_INSERT
}

//-----------------------------------------------------------------------------
void step( double dt, double t0, double& t1 ) {
  state_t x( 2 );

  // get the state vector
  x[0] = _pendulum->q;
  x[1] = _pendulum->dq;

  // compute end time t1
  t1 = t0 + dt;

  // integrate
#if defined( ODEINT_V1 )
  _stepper.do_step( *_pendulum.get(), x, t0, dt );
#elif defined( ODEINT_V2 )
  boost::numeric::odeint::integrate_adaptive( stepper_t(), *_pendulum.get(), x, t0, t1, dt );
#endif

  // update the pendulum state data structure
  _pendulum->q = x[0];
  _pendulum->dq = x[1];

}

//-----------------------------------------------------------------------------
int main( void ) {

  double t, tf, dt;
  Reveal::Core::trial_ptr trial;
  Reveal::Core::solution_ptr solution;

  if( !init() ) {
    printf( "ERROR: Initialization failed.\nExiting\n" );
    shutdown();
    return 1;
  }

  t = _start_time;
  dt = _sample_rate;

  // compute initial state
  Reveal::Core::solution_ptr initial_state = define_solution( _scenario, t );

  // get a prototype for the trial;  solution prototype is initial_state
  trial = define_trial( _scenario, t );
 
  // export the scenario framework 
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

  while( t <= _end_time ) {
    // define trial
    trial = define_trial( _scenario, t );

    // integrate
    step( dt, t, tf );

    // update time
    t = update_time( dt );

    // define solution
    solution = define_solution( _scenario, t );

    // write trial and solution
    _exporter.write( trial );
    _exporter.write( solution );
#ifdef DB_DIRECT_INSERT
    _db->insert( trial );
    _db->insert( solution );
#endif // DB_DIRECT_INSERT

    std::cout << t << " " << _pendulum->q << " " << _pendulum->dq << std::endl;
  }

#ifdef DB_DIRECT_INSERT
  /// clean up
  _db->close();
#endif // DB_DIRECT_INSERT

  printf( "Data generation succeeded\n" );
  shutdown();
  return 0;
}

//-----------------------------------------------------------------------------
