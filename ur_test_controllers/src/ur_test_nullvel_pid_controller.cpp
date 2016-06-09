#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

#include <control_toolbox/pid.h>

namespace ur_test_controllers
{


class NullVelPidController : public controller_interface::Controller< hardware_interface::VelocityJointInterface >
{

public:

  bool init( hardware_interface::VelocityJointInterface* robot, ros::NodeHandle& n );

  void starting( const ros::Time& time );

  void update( const ros::Time& time, const ros::Duration& period );

private:

  ros::NodeHandle nh_;

  std::vector< hardware_interface::JointHandle > joint_handles_;

  size_t n_joints_;

  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<double> initial_pos_;

};


bool NullVelPidController::init( hardware_interface::VelocityJointInterface* robot, ros::NodeHandle& n )
{

  // Copy the nodehandle
  nh_ = n;

  // Retrieve names of the joints to be controlled
  std::vector< std::string > joint_names;
  if( !nh_.getParam( "joints", joint_names ) )
  {
    ROS_ERROR_STREAM( "No joint list found on controller parameter namespace: " << nh_.getNamespace() );
    return false;
  }

  // Get handles to controlled joints
  joint_handles_.clear();
  for( std::vector< std::string >::const_iterator it = joint_names.begin(); it != joint_names.end(); ++it )
  {
    try
    {
      joint_handles_.push_back( robot->getHandle( *it ) );
    }
    catch( const hardware_interface::HardwareInterfaceException& e )
    {
      ROS_ERROR_STREAM( "Failed to get handle for joint: " << *it );
      return false;
    }
  }

  n_joints_ = joint_handles_.size();

  initial_pos_.resize( n_joints_ );

  pids_.resize( n_joints_ );
  for( unsigned int i = 0; i < n_joints_; ++i )
  {
    pids_[i].reset( new control_toolbox::Pid( 1.0 ) );
  }

  return true;

}


void NullVelPidController::starting( const ros::Time& time )
{

  for( unsigned int i = 0; i < n_joints_; ++i )
  {
    initial_pos_[i] = joint_handles_[i].getPosition();
  }

}


void NullVelPidController::update( const ros::Time& time, const ros::Duration& period )
{

  for( unsigned int i = 0; i < n_joints_; ++i )
  {
    double pos_error = initial_pos_[i] - joint_handles_[i].getPosition();
    double vel_error = -joint_handles_[i].getVelocity();
    double pid_output = pids_[i]->computeCommand( pos_error, vel_error, period );
    joint_handles_[i].setCommand( pid_output );
  }

}


}

PLUGINLIB_EXPORT_CLASS(ur_test_controllers::NullVelPidController, controller_interface::ControllerBase)
