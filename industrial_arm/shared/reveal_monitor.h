#ifndef _REVEAL_PACKAGE_ARM_MONITOR_H_
#define _REVEAL_PACKAGE_ARM_MONITOR_H_

//-----------------------------------------------------------------------------

class monitor_c {
public:
  Reveal::Core::authorization_ptr auth;
  Reveal::Core::scenario_ptr scenario;
  Reveal::Core::experiment_ptr experiment;
  Reveal::Core::trial_ptr trial;
  Reveal::Core::solution_ptr solution;

  Reveal::Core::pipe_ptr _revealpipe;

  monitor_c( void ) { }
  virtual ~monitor_c( void ) { }

  bool connect( void ) {
    //TODO: correct constructor
    printf( "Connecting to server...\n" );

    // TODO: For now, these hardcoded values are okay, but most likely needs 
    // to be configurable through defines in cmake
    unsigned port = MONITOR_PORT;
    std::string host = "localhost";
    _revealpipe = Reveal::Core::pipe_ptr( new Reveal::Core::pipe_c( host, port ) );
    if( _revealpipe->open() != Reveal::Core::pipe_c::ERROR_NONE ) {
      return false;
    }

    printf( "Connected\n" );
    return true;
  }

  bool read_experiment( void ) {
    Reveal::Core::transport_exchange_c ex;
    Reveal::Core::transport_exchange_c::error_e ex_err;
    std::string msg;

    // block waiting for a server msg
    if( _revealpipe->read( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
      // TODO: error recovery
    }

    ex_err = ex.parse_server_experiment( msg, auth, scenario, experiment );
    if( ex_err != Reveal::Core::transport_exchange_c::ERROR_NONE ) {
      //TODO: error handling
    }
  }

  bool read_trial( void ) {
    Reveal::Core::transport_exchange_c ex;
    Reveal::Core::transport_exchange_c::error_e ex_err;
    std::string msg;

    if( _revealpipe->read( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
      // TODO: error recovery
    }
    ex_err = ex.parse( msg );
    if( ex_err != Reveal::Core::transport_exchange_c::ERROR_NONE ) {
      //TODO: error handling
    }
    trial = ex.get_trial();
  }

  bool write_solution( Reveal::Core::solution_ptr solution ) {
    Reveal::Core::transport_exchange_c ex;
    Reveal::Core::transport_exchange_c::error_e ex_err;
    std::string msg;

    ex_err = ex.build_client_solution( msg, auth, experiment, solution );
    if( ex_err != Reveal::Core::transport_exchange_c::ERROR_NONE ) {
      //TODO: error handling
    }
    if( _revealpipe->write( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
      //TODO: error handling
    }

  }
};
//-----------------------------------------------------------------------------

#endif // _REVEAL_PACKAGE_ARM_MONITOR_H_
