/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <reveal/core/system.h>
#include <reveal/core/ipc.h>
#include <reveal/core/pointers.h>
#include <reveal/core/experiment.h>
#include <reveal/core/scenario.h>
#include <reveal/core/trial.h>
#include <reveal/core/solution.h>
#include <reveal/samples/exchange.h>

//#include <reveal/sim/gazebo/helpers.h>
#include <helpers.h>

//-----------------------------------------------------------------------------
namespace gazebo {
//-----------------------------------------------------------------------------

class world_plugin_c : public WorldPlugin {
private:
  /// gazebo callback connection allows Preupdate to be called before a step so
  /// the simulation environment can be set up
  event::ConnectionPtr     _preupdateConnection;
  /// gazebo callback connection allows Postupdate to be called after a step so
  /// the simulation results can be returned back to reveal
  event::ConnectionPtr     _postupdateConnection;

  /// the gazebo world pointer allows the simulation to get and set world state
  physics::WorldPtr        _world;

  /// the reveal authorization that contains authorization and session data
  Reveal::Core::authorization_ptr   _auth;
  /// the reveal scenario that defines the general parameters of the simulation
  Reveal::Core::scenario_ptr        _scenario;
  /// the reveal experiment that defines user specified parameters of the sim
  Reveal::Core::experiment_ptr      _experiment;
  /// the reveal trial that defines model state and control data to apply to sim
  Reveal::Core::trial_ptr           _trial;
  /// the reveal solution that defines the state of the simulation after a trial
  Reveal::Core::solution_ptr        _solution;

  /// the interprocess communication pipe used to receive command and message
  /// input from the reveal client and to send result message output to the 
  /// reveal client
  Reveal::Core::pipe_ptr            _revealpipe;

  //---------------------------------------------------------------------------
  /// Opens the interprocess communication pipe to the reveal client
  /// @return true if the connection is opened OR false if a connection cannot 
  ///         be made
  bool connect( void ) {
    std::cerr << "Connecting to server..." << std::endl;

    Reveal::Core::system_c system( Reveal::Core::system_c::CLIENT );
    if( !system.open() ) return false;
    unsigned port = system.monitor_port();
    std::string host = "localhost";
    _revealpipe = Reveal::Core::pipe_ptr( new Reveal::Core::pipe_c( host, port ) );
    if( _revealpipe->open() != Reveal::Core::pipe_c::ERROR_NONE ) {
      return false;
    }

    std::cerr << "Connected" << std::endl;
    return true;
  }

public:
  //---------------------------------------------------------------------------
  /// Default constructor
  world_plugin_c( ) {

  }

  //---------------------------------------------------------------------------
  /// Destructor
  ~world_plugin_c( ) {
    // remove the gazebo callbacks
    event::Events::DisconnectWorldUpdateBegin( _preupdateConnection );
    event::Events::DisconnectWorldUpdateBegin( _postupdateConnection );

    // close ipc
    _revealpipe->close();
  }

protected:
  //---------------------------------------------------------------------------
  /// Initializes everything necessary to interconnect Gazebo and Reveal.  
  /// Fulfills the gazebo WorldPlugin interface
  /// @param world the gazebo world object
  /// @param sdf the configuration data
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
    std::cerr << "Starting World Plugin" << std::endl;
    std::cerr << "Connecting to Reveal Client..." << std::endl;

    // connect to the reveal client
    if( !connect() ) {
      std::cerr << "Failed to connect to Reveal Client\nExiting\n" << std::endl;
      exit( 1 );
    }
    std::cerr << "Connected to Reveal Client." << std::endl;

    // read the experiment from the client connection
    Reveal::Samples::exchange_c ex;
    std::string msg;
    // block waiting for a server msg
    //if( _revealpipe->read( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
    if( !_revealpipe->read( msg ) ) {
      // TODO: error recovery
    }

    //Reveal::Samples::exchange_c::error_e ex_err;
    //ex_err = ex.parse_server_experiment( msg, _auth, _scenario, _experiment );
    if( !ex.parse_server_experiment( msg, _auth, _scenario, _experiment ) ) {
      //TODO trap error
    }

    // establish the gazebo callbacks to manage this plugin's activation
    _preupdateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind( &world_plugin_c::Preupdate, this ) );
    _postupdateConnection = event::Events::ConnectWorldUpdateEnd(
        boost::bind( &world_plugin_c::Postupdate, this ) );

    // set up the world
    _world = world;
    Reveal::Sim::Gazebo::helpers_c::sim_time( 0.0, _world );
    Reveal::Sim::Gazebo::helpers_c::reset( _world );
  }

  //---------------------------------------------------------------------------
  /// Registered as the WorldUpdateBegin callback function.  Determines 
  /// appropriate reaction to commands sent by the Reveal client including
  /// setting the state of the simulator from trial data, continuing to step
  /// through the current simulation state, or exiting the simulator
  void Preupdate( ) {

    Reveal::Samples::exchange_c ex;
    //Reveal::Samples::exchange_c::error_e ex_err;
    std::string msg;

    // read the next command message from the reveal client
    if( _revealpipe->read( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
      // TODO: error recovery
    }
    //ex_err = ex.parse( msg );
    //if( ex_err != Reveal::Samples::exchange_c::ERROR_NONE ) {
    if( !ex.parse( msg ) ) {
      //TODO: error handling
    }

    // apply the correct response to the command message
    if( ex.get_type() == Reveal::Samples::exchange_c::TYPE_TRIAL ) {
      // if the message contains trial data, write trial to the simulator
      _trial = ex.get_trial();
      Reveal::Sim::Gazebo::helpers_c::write_trial( _trial, _experiment, _world );
    } else if( ex.get_type() == Reveal::Samples::exchange_c::TYPE_STEP ) {
      // otherwise, if the reveal client sent a step command, let the sim run
    } else if( ex.get_type() == Reveal::Samples::exchange_c::TYPE_EXIT ) {
      // otherwise, if the reveal client sent an exit command, kill gazebo
      exit( 0 );
    }
  }

  //---------------------------------------------------------------------------
  /// Registered as the WorldUpdateEnd callback function.  Publishes final state
  /// to the Reveal client.
  void Postupdate( ) {
    Reveal::Samples::exchange_c ex;
    //Reveal::Samples::exchange_c::error_e ex_err;
    std::string msg;

    // get the solution from the simulator
    std::vector<std::string> model_list;
    model_list.push_back( "ur10_schunk_arm" );
    model_list.push_back( "block" );
    _solution = Reveal::Sim::Gazebo::helpers_c::read_client_solution( _world, model_list, _trial->scenario_id );

    // submit the solution to the reveal client.
    //ex_err = ex.build_client_solution( msg, _auth, _experiment, _solution );
    //if( ex_err != Reveal::Samples::exchange_c::ERROR_NONE ) {
    if( !ex.build_client_solution( msg, _auth, _experiment, _solution ) ) {
      //TODO: error handling
    }
    if( _revealpipe->write( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
      //TODO: error handling
    }
  }
};

GZ_REGISTER_WORLD_PLUGIN( world_plugin_c )

//-----------------------------------------------------------------------------
} // namespace gazebo
//-----------------------------------------------------------------------------
