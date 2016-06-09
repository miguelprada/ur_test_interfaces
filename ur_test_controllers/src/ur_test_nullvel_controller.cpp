#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

namespace ur_test_controllers
{


class NullVelController : public controller_interface::Controller< hardware_interface::VelocityJointInterface >
{

public:

  bool init( hardware_interface::VelocityJointInterface* robot, ros::NodeHandle& n );

  void starting( const ros::Time& time );

  void update( const ros::Time& time, const ros::Duration& period );

private:

  ros::NodeHandle nh_;

  std::vector< hardware_interface::JointHandle > joint_handles_;

  size_t n_joints_;

};


bool NullVelController::init( hardware_interface::VelocityJointInterface* robot, ros::NodeHandle& n )
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

  return true;

}


void NullVelController::starting( const ros::Time& time )
{

}


void NullVelController::update( const ros::Time& time, const ros::Duration& period )
{

  for( std::vector< hardware_interface::JointHandle >::iterator it = joint_handles_.begin(); it != joint_handles_.end(); ++it )
  {
    it->setCommand( 0.0 );
  }

}


}

PLUGINLIB_EXPORT_CLASS(ur_test_controllers::NullVelController, controller_interface::ControllerBase)
