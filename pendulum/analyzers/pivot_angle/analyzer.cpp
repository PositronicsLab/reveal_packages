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

//-----------------------------------------------------------------------------
/// Locates the pendulum model in the solution
Reveal::Core::model_ptr pendulum_model( Reveal::Core::solution_ptr solution ) {
  Reveal::Core::model_ptr nothing;

  // iterate through the list and return the pendulum model if found
  for( unsigned i = 0; i < solution->models.size(); i++ )
    if( solution->models[i]->id == "pendulum" ) return solution->models[i];

  // otherwise the pendulum was not found, so return the uninitialized reference
  return nothing;
}

//-----------------------------------------------------------------------------
/// Locates the pivot joint in the pendulum model
Reveal::Core::joint_ptr pivot_joint( Reveal::Core::model_ptr model ) {
  Reveal::Core::joint_ptr nothing;

  // if the model is not the pendulum return the uninitialized reference
  if( model->id != "pendulum" ) return nothing;

  // iterate through the list and return the pivot joint if found
  for( unsigned i = 0; i < model->joints.size(); i++ )
    if( model->joints[i]->id == "pivot_joint" ) return model->joints[i];

  // otherwise the joint was not found, so return the uninitialized reference
  return nothing;
}

//-----------------------------------------------------------------------------
extern "C" {
//-----------------------------------------------------------------------------
/// Produces an analysis from a solution set
/// @param input[in] the solution set to analyze
/// @param output[out] the analysis generated
/// @return returns ERROR_NONE if the operation succeeded OR an error code 
///         indicating the type of failure
error_e analyze( Reveal::Core::solution_set_ptr input, Reveal::Core::analysis_ptr& output ) {

  // create a new analysis reference to contain output
  output = Reveal::Core::analysis_ptr( new Reveal::Core::analysis_c( input ) );

  // add the keys defined for this analysis to the output
  output->add_key( "t" );
  output->add_key( "delta" );

  // iterate over all solutions contained in the solution set and analyze each
  for( unsigned i = 0; i < input->solutions.size(); i++ ) {

    // get references to both the user and desired solutions
    Reveal::Core::solution_ptr user_solution = input->solutions[i];
    Reveal::Core::solution_ptr desired_solution = input->models[i];

    // extract the pendulum state from the user and desired solutions
    Reveal::Core::model_ptr user_state = pendulum_model( input->solutions[i] );
    Reveal::Core::model_ptr desired_state = pendulum_model( input->models[i] );

    // get the time of the solution
    double t = user_solution->t;

    // get the pivot joint state for the user and desired solutions
    Reveal::Core::joint_ptr user_pivot = pivot_joint( user_state );
    Reveal::Core::joint_ptr desired_pivot = pivot_joint( desired_state );

    // compute an analysis.  this case is a simple differential between desired 
    // and experimental values recorded in the pivot joint's joint angle
    double delta = fabs( desired_pivot->state.q( 0 ) - user_pivot->state.q( 0 ) );

    // build a list of the analytical values ordered by the keys definition 
    std::vector<double> values;
    values.push_back( t );
    values.push_back( delta );

    // add the list of analytical values to the output
    output->add_row( values );
  }  

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
} // extern "C"
//-----------------------------------------------------------------------------
