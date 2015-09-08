/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <controller.h>

//-----------------------------------------------------------------------------
namespace gazebo {
//-----------------------------------------------------------------------------

class plugin_c : public ModelPlugin {
private:
  event::ConnectionPtr _preupdateConnection; //< controls applied before update

  physics::WorldPtr _world;  //< to query world state
  physics::ModelPtr _model;  //< to query model state
  physics::JointPtr _pivot;  //< to get state and set controls of pivot joint

public:
  //---------------------------------------------------------------------------
  /// Default constructor
  plugin_c( void ) { }

  //---------------------------------------------------------------------------
  /// Destructor
  virtual ~plugin_c( void ) {
    // unregister the update callback
    event::Events::DisconnectWorldUpdateBegin( _preupdateConnection );
  }

  //---------------------------------------------------------------------------
  /// validates that the scenario loaded matches the references expected and
  /// sets any link and joint references for ease of use later
  /// @return true if validation succeeded OR false if validation failed
  bool validate( void ) {
    // locate the pivot joint in the model
    _pivot = _model->GetJoint( "pivot_joint" );

    // if any of the pointers are nothing then validation fails
    if( !( _world && _model && _pivot ) ) return false;

    // otherwise the configuration is valid
    return true;
  }

  //---------------------------------------------------------------------------
  /// Initializes the controller.  Fulfills the gazebo ModelPlugin interface
  virtual void Load( physics::ModelPtr model, sdf::ElementPtr sdf ) {

    // get model and world references
    _model = model;
    _world = model->GetWorld();

    // validate the configuration
    if( !validate( ) ) {
      std::cerr << "Unable to validate model in plugin" << std::endl;
      std::cerr << "ERROR: Plugin failed to load" << std::endl;
      return;
    }

    // get the starting velocity for the joints from the controller
    std::map<std::string, double> dq;
    get_initial_velocity( dq );
  
    // set the starting velocity for the joints from the controller data
    _pivot->SetVelocity( 0, dq.find("pivot_joint")->second );

    // register the gazebo callback
    _preupdateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind( &plugin_c::Preupdate, this ) );

    // -- FIN --
    std::cerr << "controller has initialized" << std::endl;
  }

  //---------------------------------------------------------------------------
  /// Registered as the Gazebo WorldUpdateBegin function for the ModelPlugin.
  /// Gets current state from the updated model, computes controls by calling
  /// the controller, and applies those controls to corresponding joints.
  virtual void Preupdate( ) {

    double t = _world->GetSimTime().Double(); // get the current virtual time 

    std::map<std::string, double> q;          // map for position state
    std::map<std::string, double> dq;         // map for velocity state
    std::map<std::string, double> u;          // map for controls

    // map in the positions relevant to control code
    q.insert( std::pair<std::string,double>("pivot_joint", _pivot->GetAngle(0).Radian() ) );

    // map in the velocities relevant to control code
    dq.insert( std::pair<std::string,double>("pivot_joint", _pivot->GetVelocity(0) ) );

    // let controller compute controls
    get_control( t, q, dq, u );

    // apply the forces
    _pivot->SetForce( 0, u.find("pivot_joint")->second );
  }
};

GZ_REGISTER_MODEL_PLUGIN( plugin_c )

//-----------------------------------------------------------------------------
} // namespace gazebo
//-----------------------------------------------------------------------------
