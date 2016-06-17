#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

#include <control_toolbox/pid.h>

#include <joint_trajectory_controller/hardware_interface_adapter.h>

namespace ur_test_controllers
{


const double pi = 4*atan2( 1, 1 );


struct State
{
  std::vector<double> position;
  std::vector<double> velocity;
};


template <class HardwareInterface>
class SinusoidController : public controller_interface::Controller< HardwareInterface >
{

public:

  bool init( HardwareInterface* robot, ros::NodeHandle& n );

  void starting( const ros::Time& time );

  void update( const ros::Time& time, const ros::Duration& period );

private:

  ros::NodeHandle nh_;

  std::vector< hardware_interface::JointHandle > joint_handles_;

  size_t n_joints_;

  std::vector<double> initial_pos_;

  HardwareInterfaceAdapter<HardwareInterface,State> hardware_interface_adapter_;

  State desired_state_;
  State state_error_;

  size_t sinusoid_joint_idx_;
  double sinusoid_amplitude_;
  double sinusoid_rate_;

  ros::Time start_time_;

};


template <class HardwareInterface>
bool SinusoidController<HardwareInterface>::init( HardwareInterface* robot, ros::NodeHandle& n )
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

  // Retrieve name and index of the joint to be moved in sinusoidal motion
  std::string sinusoid_joint;
  if( !nh_.getParam( "sinusoid_joint", sinusoid_joint ) )
  {
    ROS_ERROR( "No joint specified to generate sinusoidal motion" );
    return false;
  }

  sinusoid_joint_idx_ = std::distance( joint_names.begin(), std::find( joint_names.begin(), joint_names.end(), sinusoid_joint ) );
  if( sinusoid_joint_idx_ == joint_names.size() )
  {
    ROS_ERROR_STREAM( "Could not find joint named " << sinusoid_joint << " in joints list" );
    return false;
  }

  // Retrieve parameters of sinusoid motion
  if( !nh_.getParam( "sinusoid_amplitude", sinusoid_amplitude_ ) )
  {
    ROS_ERROR( "Could not retrieve sinusoid amplitude" );
    return false;
  }

  if( !nh_.getParam( "sinusoid_rate", sinusoid_rate_ ) )
  {
    ROS_ERROR( "Could not retrieve sinusoid rate" );
    return false;
  }

  // Initialize rest of the variables
  n_joints_ = joint_handles_.size();

  initial_pos_.resize( n_joints_ );

  hardware_interface_adapter_.init( joint_handles_, nh_ );

  desired_state_.position.resize( n_joints_ );
  desired_state_.velocity.resize( n_joints_ );
  state_error_.position.resize( n_joints_ );
  state_error_.velocity.resize( n_joints_ );

  return true;

}


template <class HardwareInterface>
void SinusoidController<HardwareInterface>::starting( const ros::Time& time )
{

  for( unsigned int i = 0; i < n_joints_; ++i )
  {
    initial_pos_[i] = joint_handles_[i].getPosition();
    desired_state_.position[i] = initial_pos_[i];
    desired_state_.velocity[i] = 0.0;
    state_error_.position[i] = 0.0;
    state_error_.velocity[i] = 0.0;
  }

  hardware_interface_adapter_.starting( time );

  start_time_ = time;

}


template <class HardwareInterface>
void SinusoidController<HardwareInterface>::update( const ros::Time& time, const ros::Duration& period )
{

  double uptime = (time - start_time_).toSec();
  double sinusoid_phase = 2*pi*uptime*sinusoid_rate_ + pi;

  for( unsigned int i = 0; i < n_joints_; ++i )
  {

    desired_state_.position[i] = initial_pos_[i];
    desired_state_.velocity[i] = 0.0;

    if( i == sinusoid_joint_idx_ )
    {
      desired_state_.position[i] += sinusoid_amplitude_ * ( 1 + cos( sinusoid_phase ) ) / 2;
      desired_state_.velocity[i] += -sinusoid_amplitude_ * pi * sinusoid_rate_ * sin( sinusoid_phase );
    }

    state_error_.position[i] = desired_state_.position[i] - joint_handles_[i].getPosition();
    state_error_.velocity[i] = desired_state_.velocity[i] - joint_handles_[i].getVelocity();
    
  }
  hardware_interface_adapter_.updateCommand( time, period, desired_state_, state_error_ );

}


typedef SinusoidController< hardware_interface::PositionJointInterface > PositionSinusoidController;
typedef SinusoidController< hardware_interface::VelocityJointInterface > VelocitySinusoidController;

}

PLUGINLIB_EXPORT_CLASS(ur_test_controllers::PositionSinusoidController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(ur_test_controllers::VelocitySinusoidController, controller_interface::ControllerBase)
