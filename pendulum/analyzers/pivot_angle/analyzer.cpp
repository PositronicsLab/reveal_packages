#include <stdio.h>
#include <cmath>

#include <reveal/analytics/types.h>
#include <reveal/core/pointers.h>
#include <reveal/core/solution_set.h>
#include <reveal/core/solution.h>
#include <reveal/core/scenario.h>
#include <reveal/core/trial.h>
#include <reveal/core/analyzer.h>

//-----------------------------------------------------------------------------

using namespace Reveal::Analytics;

//-------------------------------------------------------------------------
Reveal::Core::model_ptr pendulum_model( Reveal::Core::solution_ptr solution ) {
  Reveal::Core::model_ptr nothing;

  for( unsigned i = 0; i < solution->models.size(); i++ )
    if( solution->models[i]->id == "pendulum" ) return solution->models[i];

  return nothing;
}

//-------------------------------------------------------------------------
Reveal::Core::joint_ptr pivot_joint( Reveal::Core::model_ptr model ) {
  Reveal::Core::joint_ptr nothing;

  if( model->id != "pendulum" ) return nothing;
  for( unsigned i = 0; i < model->joints.size(); i++ )
    if( model->joints[i]->id == "pivot_joint" ) return model->joints[i];

  return nothing;
}

//-------------------------------------------------------------------------
extern "C" {

//-------------------------------------------------------------------------
error_e analyze( Reveal::Core::solution_set_ptr input, Reveal::Core::analysis_ptr& output ) {

  printf( "(analyzer) analyze started\n" );

  Reveal::Core::model_ptr initial_state = pendulum_model( input->initial_state );

  output = Reveal::Core::analysis_ptr( new Reveal::Core::analysis_c() );
  output->experiment = input->experiment;

  output->add_key( "t" );
  output->add_key( "delta" );

  for( unsigned i = 0; i < input->solutions.size(); i++ ) {
    Reveal::Core::solution_ptr user_solution = input->solutions[i];
    Reveal::Core::solution_ptr desired_solution = input->models[i];

    Reveal::Core::model_ptr user_state = pendulum_model( input->solutions[i] );
    Reveal::Core::model_ptr desired_state = pendulum_model( input->models[i] );

    double t = user_solution->t;

    Reveal::Core::joint_ptr user_pivot = pivot_joint( user_state );
    Reveal::Core::joint_ptr desired_pivot = pivot_joint( desired_state );

    double delta = fabs( desired_pivot->state.q( 0 ) - user_pivot->state.q( 0 ) );

    std::vector<double> values;
    values.push_back( t );
    values.push_back( delta );
    output->add_row( values );
    
  }  
  printf( "(analyzer) analyze finished\n" );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

} // extern "C"

