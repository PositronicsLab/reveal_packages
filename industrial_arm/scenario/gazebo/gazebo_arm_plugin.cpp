/*------------------------------------------------------------------------------

Note: much of this code is dedicated to generating data.  Paring down and 
removing the data generation code shows that this implementation only handles 
the interface between gazebo and the arm controller code in arm_controller.h
------------------------------------------------------------------------------*/
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

// The DATA_GENERATION define is set by cmake and must be manually turned on
// so that the data is output to file.  This is an example of how the controller
// can be set up to produce data, but also be compatible for Reveal which by
// default has no knowledge of the cmake parameter so the switch is off when
// running a scenario via Reveal 
#ifdef DATA_GENERATION

#include <helpers.h>

// The DB_DIRECT_INSERT define is set by cmake and must be manually turned on so
// that data is directly inserted into the database.  This option can speed up
// development on a fully local platform as it circumvents the need to export
// then import, but it is not compatible to distributed Reveal client/server so
// should not be included in any form in distributed packages
#ifdef DB_DIRECT_INSERT
#ifndef _REVEAL_SERVER_SERVER_H_
#define _REVEAL_SERVER_SERVER_H_
#endif // _REVEAL_SERVER_SERVER_H_
#include <reveal/db/database.h>
#endif  // DB_DIRECT_INSERT

#include <reveal/core/scenario.h>
#include <reveal/core/trial.h>
#include <reveal/core/solution.h>

#include <reveal/core/system.h>
#include <reveal/core/exporter.h>
#include <reveal/core/analyzer.h>
#endif // DATA_GENERATION

#include <arm_controller.h>

#define MAX_TIME 16.000

//-----------------------------------------------------------------------------
namespace gazebo {
//-----------------------------------------------------------------------------

class arm_controller_c : public ModelPlugin {
private:
  /// gazebo callback connection allows Preupdate to be called before a step so
  /// controls can be applied
  event::ConnectionPtr _preupdateConnection;
  /// gazebo callback connection allows Postupdate to be called after a step so
  /// the simulation state can be recorded for data generation.
  event::ConnectionPtr _postupdateConnection;

  /// the gazebo world pointer allows the simulation to get world state 
  physics::WorldPtr _world;
  /// the gazebo model pointer allows the controls to be applied to the arm  
  physics::ModelPtr _model;

  physics::LinkPtr _base;         /// reference to the arm's base link
  physics::LinkPtr _shoulder;     /// reference to the arm's shoulder link
  physics::LinkPtr _upperarm;     /// reference to the arm's upperarm link
  physics::LinkPtr _forearm;      /// reference to the arm's forearm link
  physics::LinkPtr _wrist1;       /// reference to the arm's first wrist link
  physics::LinkPtr _wrist2;       /// reference to the arm's second wrist link
  physics::LinkPtr _wrist3;       /// reference to the arm's third wrist link

  physics::JointPtr _shoulder_pan_actuator;  /// shoulder to base joint
  physics::JointPtr _shoulder_lift_actuator; /// upperarm to shoulder joint
  physics::JointPtr _elbow_actuator;         /// forearm to upperarm joint
  physics::JointPtr _wrist1_actuator;        /// wrist1 to forearm joint
  physics::JointPtr _wrist2_actuator;        /// wrist2 to wrist1 joint
  physics::JointPtr _wrist3_actuator;        /// wrist3 to wrist2 joint

  // Schunk hand
  physics::LinkPtr _hand;         /// reference to hand's base link;
  physics::LinkPtr _finger_l;     /// reference to hand's left finger link
  physics::LinkPtr _finger_r;     /// reference to hand's right finger link

  physics::JointPtr _finger_actuator_l;      /// left finger to hand base joint
  physics::JointPtr _finger_actuator_r;      /// right finger to hand base joint

#ifdef DATA_GENERATION
  /// the reveal scenario data created to generate a record or file
  Reveal::Core::scenario_ptr   _scenario;
  /// the reveal analyzer data created to generate a record of file
  Reveal::Core::analyzer_ptr   _analyzer;
  /// the reveal trial data gathered from sim and written to db or file
  Reveal::Core::trial_ptr      _trial;
  /// the reveal solution data gathered from sim and written to db or file
  Reveal::Core::solution_ptr   _solution;

  Reveal::Core::solution_ptr _initial_state;

  bool _first_iteration;    /// flag to indicate first step.  only for data gen

  std::vector<std::string> _model_list;
  Reveal::Core::model_ptr _arm_filter;
  Reveal::Core::model_ptr _block_filter;
  std::vector<Reveal::Core::model_ptr> _filters;

#ifdef DB_DIRECT_INSERT
  /// the local reveal database
  boost::shared_ptr<Reveal::DB::database_c> _db;
#endif // DB_DIRECT_INSERT

  /// the reveal exporter that handles exporting all the scenario data generated
  /// to requisite files
  Reveal::Core::exporter_c exporter;
#endif // DATA_GENERATION

public:
  //---------------------------------------------------------------------------
  /// Default constructor
  arm_controller_c( void ) { }

  //---------------------------------------------------------------------------
  /// Destructor
  virtual ~arm_controller_c( void ) {
    // remove the gazebo callbacks
    event::Events::DisconnectWorldUpdateBegin( _preupdateConnection );
    event::Events::DisconnectWorldUpdateBegin( _postupdateConnection );

#ifdef DB_DIRECT_INSERT
    /// if direct insertion into the database was enabled, close the database
    _db->close();
#endif // DB_DIRECT_INSERT
  }

  //---------------------------------------------------------------------------
  /// validates that the scenario loaded matches the references expected and
  /// sets all the helper link and joint references for ease of use later
  /// @return true if validation succeeded OR false if validation failed
  bool validate( void ) {
    // ur10 arm references
    _base = _model->GetLink( "ur10::base_link" );
    _shoulder = _model->GetLink( "ur10::shoulder_link" );
    _upperarm = _model->GetLink( "ur10::upper_arm_link" );
    _forearm = _model->GetLink( "ur10::forearm_link" );
    _wrist1 = _model->GetLink( "ur10::wrist_1_link" );
    _wrist2 = _model->GetLink( "ur10::wrist_2_link" );
    _wrist3 = _model->GetLink( "ur10::wrist_3_link" );
    _shoulder_pan_actuator = _model->GetJoint( "ur10::shoulder_pan_joint" );
    _shoulder_lift_actuator = _model->GetJoint( "ur10::shoulder_lift_joint" );
    _elbow_actuator = _model->GetJoint( "ur10::elbow_joint" );
    _wrist1_actuator = _model->GetJoint( "ur10::wrist_1_joint" );
    _wrist2_actuator = _model->GetJoint( "ur10::wrist_2_joint" );
    _wrist3_actuator = _model->GetJoint( "ur10::wrist_3_joint" );

    // schunk hand references
    _hand = _model->GetLink( "schunk_mpg_80::base" );
    _finger_l = _model->GetLink( "schunk_mpg_80::l_finger" );
    _finger_r = _model->GetLink( "schunk_mpg_80::r_finger" );
    _finger_actuator_l = _model->GetJoint( "schunk_mpg_80::l_finger_actuator" );
    _finger_actuator_r = _model->GetJoint( "schunk_mpg_80::r_finger_actuator" );

    // if any of the pointers are nothing then validation fails
    if( !( _world && _model && _base && _shoulder && _upperarm && _forearm &&
           _wrist1 && _wrist2 && _wrist3 && _shoulder_pan_actuator && 
           _shoulder_lift_actuator && _elbow_actuator && _wrist1_actuator && 
           _wrist2_actuator && _wrist3_actuator && _hand && _finger_l && 
           _finger_r && _finger_actuator_l && _finger_actuator_r ) )
      return false;

    // otherwise the configuration is valid
    return true;
  }

  //---------------------------------------------------------------------------
  /// Initializes the arm controller.  Fulfills the gazebo ModelPlugin interface
  /// If data generation is enabled, Load opens any export or database resources
   virtual void Load( physics::ModelPtr model, sdf::ElementPtr sdf ) {

    // get the model and world references
    _model = model;
    _world = model->GetWorld();

    // validate the configuration
    if( !validate( ) ) {
      std::cerr << "Unable to validate arm model in arm_controller" << std::endl;
      std::cerr << "ERROR: Plugin failed to load" << std::endl;
      return;
    }

#ifdef DATA_GENERATION
    // if data generation, initialize relevant variables and create scenario 
    // and analyzer data
    _first_iteration = true;

    _scenario = generate_scenario();
    _analyzer = generate_analyzer( _scenario );

    Reveal::Core::link_ptr link;
    Reveal::Core::joint_ptr joint;

    // build the block filter
    _block_filter = Reveal::Core::model_ptr( new Reveal::Core::model_c( "block" ) );
    link = Reveal::Core::link_ptr( new Reveal::Core::link_c( "body" ) );
    _block_filter->links.push_back( link );

    _filters.push_back( _block_filter );

    // build the arm filter
    _arm_filter = Reveal::Core::model_ptr( new Reveal::Core::model_c( "ur10_schunk_arm" ) );
    link = Reveal::Core::link_ptr( new Reveal::Core::link_c("ur10::base_link") );
    _arm_filter->links.push_back( link );

    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "ur10::shoulder_pan_joint" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "ur10::shoulder_lift_joint" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "ur10::elbow_joint" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "ur10::wrist_1_joint" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "ur10::wrist_2_joint" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "ur10::wrist_3_joint" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "schunk_mpg_80::l_finger_actuator" ) );
    _arm_filter->joints.push_back( joint );
    joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c( "schunk_mpg_80::r_finger_actuator" ) );
    _arm_filter->joints.push_back( joint );

    _filters.push_back( _arm_filter );

    // reset the world state
    Reveal::Sim::Gazebo::helpers_c::reset( _world );
#ifdef DB_DIRECT_INSERT
    // if the data is being directly inserted into the database, open the db
    _db = boost::shared_ptr<Reveal::DB::database_c>( new Reveal::DB::database_c() );
    if( !_db->open() ) {
      printf( "ERROR: failed to open database\n" );
    }

    // then insert the scenario and analyzer records
    if( _db->insert( _scenario ) != Reveal::DB::database_c::ERROR_NONE ) {
      printf( "ERROR: failed to insert scenario into database\n" );
    }
    _db->insert( _analyzer );
#endif // DB_DIRECT_INSERT
#endif // DATA_GENERATION

    // get the starting velocity for the joints from the controller
    std::map<std::string, double> qd;
    get_initial_velocity( qd );
  
    // set the starting velocity for the joints from the controller data
    _shoulder_pan_actuator->SetVelocity(0, qd.find("shoulder_pan_joint")->second);
    _shoulder_lift_actuator->SetVelocity(0, qd.find("shoulder_lift_joint")->second);
    _elbow_actuator->SetVelocity(0, qd.find("elbow_joint")->second);
    _wrist1_actuator->SetVelocity(0, qd.find("wrist_1_joint")->second);
    _wrist2_actuator->SetVelocity(0, qd.find("wrist_2_joint")->second);
    _wrist3_actuator->SetVelocity(0, qd.find("wrist_3_joint")->second);

    // register the gazebo callbacks.
    _preupdateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind( &arm_controller_c::Preupdate, this ) );
    _postupdateConnection = event::Events::ConnectWorldUpdateEnd(
      boost::bind( &arm_controller_c::Postupdate, this ) );

#ifdef DATA_GENERATION
    // TODO: determine best initial case
    // may write the initial trial.  State at t = 0 and no controls
    // but potential solution might not be found until post update and preupdate
    // may interfere

    // build the list of models intended to be recorded
    _model_list.push_back( "ur10_schunk_arm" );
    _model_list.push_back( "block" );

#endif // DATA_GENERATION

    // -- FIN --
    std::cerr << "arm_controller has initialized" << std::endl;
  
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
    q.insert( std::pair<std::string,double>("shoulder_pan_joint", _shoulder_pan_actuator->GetAngle(0).Radian() ) );
    q.insert( std::pair<std::string,double>("shoulder_lift_joint", _shoulder_lift_actuator->GetAngle(0).Radian() ) );
    q.insert( std::pair<std::string,double>("elbow_joint", _elbow_actuator->GetAngle(0).Radian() ) );
    q.insert( std::pair<std::string,double>("wrist_1_joint", _wrist1_actuator->GetAngle(0).Radian() ) );
    q.insert( std::pair<std::string,double>("wrist_2_joint", _wrist2_actuator->GetAngle(0).Radian() ) );
    q.insert( std::pair<std::string,double>("wrist_3_joint", _wrist3_actuator->GetAngle(0).Radian() ) );

    // map in the velocities
    qd.insert( std::pair<std::string,double>("shoulder_pan_joint", _shoulder_pan_actuator->GetVelocity(0) ) );
    qd.insert( std::pair<std::string,double>("shoulder_lift_joint", _shoulder_lift_actuator->GetVelocity(0) ) );
    qd.insert( std::pair<std::string,double>("elbow_joint", _elbow_actuator->GetVelocity(0) ) );
    qd.insert( std::pair<std::string,double>("wrist_1_joint", _wrist1_actuator->GetVelocity(0) ) );
    qd.insert( std::pair<std::string,double>("wrist_2_joint", _wrist2_actuator->GetVelocity(0) ) );
    qd.insert( std::pair<std::string,double>("wrist_3_joint", _wrist3_actuator->GetVelocity(0) ) );

    // compute controls using the arm controller
    get_control( t, q, qd, u );

    // apply the forces
    _shoulder_pan_actuator->SetForce(0, u.find("shoulder_pan_joint")->second);
    _shoulder_lift_actuator->SetForce(0, u.find("shoulder_lift_joint")->second);
    _elbow_actuator->SetForce(0, u.find("elbow_joint")->second);
    _wrist1_actuator->SetForce(0, u.find("wrist_1_joint")->second);
    _wrist2_actuator->SetForce(0, u.find("wrist_2_joint")->second);
    _wrist3_actuator->SetForce(0, u.find("wrist_3_joint")->second);
    _finger_actuator_l->SetForce(0, u.find("l_finger_actuator")->second);
    _finger_actuator_r->SetForce(0, u.find("r_finger_actuator")->second); 

#ifdef DATA_GENERATION
    // if data is being generated, then get the trial information
    _trial = generate_trial( _scenario,  
                         u.find("shoulder_pan_joint")->second, 
                         u.find("shoulder_lift_joint")->second, 
                         u.find("elbow_joint")->second, 
                         u.find("wrist_1_joint")->second,
                         u.find("wrist_2_joint")->second,
                         u.find("wrist_3_joint")->second,
                         u.find("l_finger_actuator")->second,
                         u.find("r_finger_actuator")->second  );

    if( _first_iteration ) {
      // read the initial state as the first model solution
      _initial_state = Reveal::Sim::Gazebo::helpers_c::read_model_solution( _world, _model_list, _scenario->id );
    }

#ifdef DB_DIRECT_INSERT
    // if the data is being directly inserted, insert the trial in the db
    _db->insert( _trial );
#endif // DB_DIRECT_INSERT
#endif // DATA_GENERATION
  }

  //---------------------------------------------------------------------------
  /// Registered as the WorldUpdateEnd callback function for the ModelPlugin.
  /// Only has content if data generation is enabled.  Handles exporting data
  /// to file and if direct insertion enabled inserts solutions into database
  virtual void Postupdate( ) {
#ifdef DATA_GENERATION
    // get the time and time step from the simulator
    double t = Reveal::Sim::Gazebo::helpers_c::sim_time( _world );
    double dt = Reveal::Sim::Gazebo::helpers_c::step_size( _world );

    // create a model solution record by reading it from the sim
    _solution = Reveal::Sim::Gazebo::helpers_c::read_model_solution( _world, _model_list, _scenario->id, _filters );
      
#ifdef DB_DIRECT_INSERT
    // if first iteration, store as the first model solution
    if( _first_iteration ) {
      _db->insert( _initial_state );
    } else {
      // if direct insert, insert the model solution into the db
      _db->insert( _solution ); 
    }
#endif // DB_DIRECT_INSERT

    if( _first_iteration ) {
      // if first iteration, then export the scenario framework
      bool result = exporter.write( _scenario, _analyzer, _solution, _trial );
      exporter.write( _initial_state );

      _first_iteration = false;
    } else {
      exporter.write( _solution );
    }

    // write the trial and solution data for the current iteration
    exporter.write( _trial );

    // exit condition
    // Note: this is arbitrary at this point.  It assumes that the scenario
    // has not been tuned and the simulator is ODE.
    if( t >= MAX_TIME ) exit( 0 );
#endif // DATA_GENERATION
  }

  //---------------------------------------------------------------------------
  // Gazebo callback.  Called whenever the simulation is reset
  //virtual void Reset( ) { }

#ifdef DATA_GENERATION
  //---------------------------------------------------------------------------
  // Data Generation methods.
  // For generating records directly from the simulation.  Not directly
  // used in Reveal implementations, but used to run the scenario and produce
  // data files that are compatible with Reveal import
  //---------------------------------------------------------------------------
  /// Generates scenario record data as a reveal scenario pointer
  /// @return a scenario pointer than can be inserted into database or exported
  Reveal::Core::scenario_ptr generate_scenario( void ) {
    // create and insert the scenario record
    Reveal::Core::scenario_ptr scenario( new Reveal::Core::scenario_c() );

    scenario->id = generate_uuid();
    scenario->package_id = "industrial_arm";
    scenario->description = "grasping a block with an industrial arm";
    scenario->sample_rate = Reveal::Sim::Gazebo::helpers_c::step_size(_world);
    scenario->sample_start_time = scenario->sample_rate;
    scenario->sample_end_time = MAX_TIME + scenario->sample_rate;

    return scenario;
  }

  //---------------------------------------------------------------------------
  /// Generates analyzer record data as a reveal analyzer pointer
  /// @param scenario the scenario that this analyzer handles
  /// @return an analyzer pointer that can be inserted into database or exported
  Reveal::Core::analyzer_ptr generate_analyzer( Reveal::Core::scenario_ptr scenario ) {
    Reveal::Core::analyzer_ptr analyzer( new Reveal::Core::analyzer_c() );
 
    analyzer->scenario_id = scenario->id;
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

  //---------------------------------------------------------------------------
  /// Generates trial record data as a reveal trial pointer
  /// @param scenario the scenario that this trial is generated from
  /// @param sh_pan_f the torque to be applied to the shoulder pan joint
  /// @param sh_lift_f the torque to be applied to the shoulder lift joint
  /// @param elbow_f the torque to be applied to the elbow joint
  /// @param wrist1_f the torque to be applied to the first wrist joint
  /// @param wrist2_f the torque to be applied to the second wrist joint
  /// @param wrist3_f the torque to be applied to the third wrist joint
  /// @param finger_l_f the force to be applied to the left finger joint
  /// @param finger_r_f the force to be applied to the right finger joint
  /// @return a trial pointer that can be inserted into database or exported
  Reveal::Core::trial_ptr generate_trial( Reveal::Core::scenario_ptr scenario, double sh_pan_f, double sh_lift_f, double elbow_f, double wrist1_f, double wrist2_f, double wrist3_f, double finger_l_f, double finger_r_f ) {
    Reveal::Core::trial_ptr trial = Reveal::Core::trial_ptr( new Reveal::Core::trial_c() );

    trial->scenario_id = scenario->id;
    trial->t = Reveal::Sim::Gazebo::helpers_c::sim_time( _world );

    std::map<std::string,double> arm_controls;
    arm_controls.insert( std::pair<std::string,double>( "ur10::shoulder_pan_joint&0", sh_pan_f ) );
    arm_controls.insert( std::pair<std::string,double>( "ur10::shoulder_lift_joint&0", sh_lift_f ) );
    arm_controls.insert( std::pair<std::string,double>( "ur10::elbow_joint&0", elbow_f ) );
    arm_controls.insert( std::pair<std::string,double>( "ur10::wrist_1_joint&0", wrist1_f ) );
    arm_controls.insert( std::pair<std::string,double>( "ur10::wrist2_joint&0", wrist2_f ) );
    arm_controls.insert( std::pair<std::string,double>( "ur10::wrist3_joint&0", wrist3_f ) );
    arm_controls.insert( std::pair<std::string,double>( "schunk_mpg_80::l_finger_actuator&0", finger_l_f ) );
    arm_controls.insert( std::pair<std::string,double>( "schunk_mpg_80::r_finger_actuator&0", finger_r_f ) );

    Reveal::Core::model_ptr arm_model = Reveal::Sim::Gazebo::helpers_c::read_model( _world, "ur10_schunk_arm", arm_controls, _arm_filter );

    trial->models.push_back( arm_model );

    std::map<std::string,double> null_controls;
    Reveal::Core::model_ptr block_model = Reveal::Sim::Gazebo::helpers_c::read_model( _world, "block", null_controls, _block_filter );

    trial->models.push_back( block_model );
    
    return trial;
  }
#endif // DATA_GENERATION
};

GZ_REGISTER_MODEL_PLUGIN( arm_controller_c )

//-----------------------------------------------------------------------------
} // namespace gazebo
//-----------------------------------------------------------------------------
