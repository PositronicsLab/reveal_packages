/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <controller.h>

//-----------------------------------------------------------------------------
#define PI 3.14159265358979323846

//-----------------------------------------------------------------------------

// open parameters
double position_gain = 5.0;
double velocity_gain = 1.0;

// experiment parameters
double initial_time = 0.0;
double final_time = 10.0;
double initial_position = 0.0;
double final_position = PI;

//-----------------------------------------------------------------------------
namespace gazebo {
//-----------------------------------------------------------------------------

class plugin_c : public ModelPlugin {
private:
  /// gazebo callback connection allows Preupdate to be called before a step so
  /// controls can be applied
  event::ConnectionPtr _preupdateConnection;
  /// gazebo callback connection allows Postupdate to be called after a step so
  /// the simulation state can be recorded for data generation.
  //event::ConnectionPtr _postupdateConnection;

  /// the gazebo world pointer allows the simulation to get world state 
  physics::WorldPtr _world;
  /// the gazebo model pointer allows the controls to be applied to the arm  
  physics::ModelPtr _model;

  physics::JointPtr _pivot;  

public:
  //---------------------------------------------------------------------------
  /// Default constructor
  plugin_c( void ) { }

  //---------------------------------------------------------------------------
  /// Destructor
  virtual ~plugin_c( void ) {
    // remove the gazebo callbacks
    event::Events::DisconnectWorldUpdateBegin( _preupdateConnection );
    //event::Events::DisconnectWorldUpdateBegin( _postupdateConnection );
  }

  //---------------------------------------------------------------------------
  /// validates that the scenario loaded matches the references expected and
  /// sets all the helper link and joint references for ease of use later
  /// @return true if validation succeeded OR false if validation failed
  bool validate( void ) {
    _pivot = _model->GetJoint( "pivot_joint" );

    // if any of the pointers are nothing then validation fails
    if( !( _world && _model && _pivot ) )
      return false;

    // otherwise the configuration is valid
    return true;
  }

  //---------------------------------------------------------------------------
  /// Initializes the controller.  Fulfills the gazebo ModelPlugin interface
  virtual void Load( physics::ModelPtr model, sdf::ElementPtr sdf ) {

    // get the model and world references
    _model = model;
    _world = model->GetWorld();

    // validate the configuration
    if( !validate( ) ) {
      std::cerr << "Unable to validate model in plugin" << std::endl;
      std::cerr << "ERROR: Plugin failed to load" << std::endl;
      return;
    }

    // get the starting velocity for the joints from the controller
    std::map<std::string, double> qd;
    get_initial_velocity( qd );
  
    // set the starting velocity for the joints from the controller data
    _pivot->SetVelocity(0, qd.find("pivot_joint")->second);

    // register the gazebo callbacks.
    _preupdateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind( &plugin_c::Preupdate, this ) );
    //_postupdateConnection = event::Events::ConnectWorldUpdateEnd(
    //  boost::bind( &plugin_c::Postupdate, this ) );

    // -- FIN --
    std::cerr << "controller has initialized" << std::endl;
  
  }

  //---------------------------------------------------------------------------
  /// Registered as the WorldUpdateBegin callback function for the ModelPlugin.
  /// Computes the controls to be applied to the arm using the arm controller 
  /// and applies those controls to corresponding joints
  virtual void Preupdate( ) {

    // get the current time
    //double t = Reveal::Sim::Gazebo::helpers_c::sim_time( _world );
    double t = _world->GetSimTime().Double();

    std::map<std::string, double> q;
    std::map<std::string, double> qd;
    std::map<std::string, double> u;

    // map in the positions
    q.insert( std::pair<std::string,double>("pivot_joint", _pivot->GetAngle(0).Radian() ) );

    // map in the velocities
    qd.insert( std::pair<std::string,double>("pivot_joint", _pivot->GetVelocity(0) ) );

    // compute controls using the arm controller
    get_control( t, q, qd, u );

    double f = u.find("pivot_joint")->second;

    // apply the forces
    _pivot->SetForce( 0, u.find("pivot_joint")->second );
  }
};

GZ_REGISTER_MODEL_PLUGIN( plugin_c )

//-----------------------------------------------------------------------------
} // namespace gazebo
//-----------------------------------------------------------------------------
