#include <sstream>
#include <kuuvef1_hw_interface/kuuvef1_hw_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <serial/serial.h>
#include <string>
//#include <kuuvef1cpp/kuuvef1.h>
//#include <kuuvef1cpp/joint.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using namespace std;

namespace kuuvef1_hw_interface
{
	Kuuvef1HardwareInterface::Kuuvef1HardwareInterface(ros::NodeHandle& nh) \
		: nh_(nh)
	{
		init();
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

		nh_.param("/kuuvef1/hardware_interface/loop_hz", loop_hz_, 0.1);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &Kuuvef1HardwareInterface::update, this);

		ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
	}

	Kuuvef1HardwareInterface::~Kuuvef1HardwareInterface()
	{
	}

	void Kuuvef1HardwareInterface::serialInit()
	{
		string port;
		int baudrate;

		nh_.getParam("/kuuvef1/serial/port", port);
		nh_.getParam("/kuuvef1/serial/baudrate", baudrate);

		try
		{
			kuuvef1_motor_.init(port, baudrate);
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
            return;
        }

	}

	void Kuuvef1HardwareInterface::init()
	{
		//joint_mode_ = 3; // ONLY EFFORT FOR NOW
		// Get joint names
		/*
		nh_.getParam("/kuuvef1/hardware_interface/joints", joint_names_);
		if (joint_names_.size() == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}
		num_joints_ = joint_names_.size();
		*/

		serialInit();

		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		//joint_effort_.resize(num_joints_);
		//joint_position_command_.resize(num_joints_);
		//joint_velocity_command_.resize(num_joints_);
		//joint_effort_command_.resize(num_joints_);


		// Initialize controller
		//for (int i = 0; i < num_joints_; ++i)
		//{
			//kuuvef1cpp::Joint joint = kuuvef1.getJoint(joint_names_[i]);

			//ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint.name);

			//nh_.getParam("/kuuvef1/joint_offsets/" + joint.name, joint.angleOffset);
			//nh_.getParam("/kuuvef1/joint_read_ratio/" + joint.name, joint.readRatio);
			//kuuvef1.setJoint(joint);

		  // Create joint state interface
			JointStateHandle steerStateHandle(kuuvef1_motor_.getSteerName(), &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
			joint_state_interface_.registerHandle(steerStateHandle);

			JointStateHandle driveStateHandle(kuuvef1_motor_.getDriveName(), &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
			joint_state_interface_.registerHandle(driveStateHandle);

		  // Create position joint interface
			JointHandle steerPositionHandle(steerStateHandle, &steer_position_command_);
			/*
			JointLimits limits;
			SoftJointLimits softLimits;
			if (getJointLimits(joint.name, nh_, limits) == false) {
				ROS_ERROR_STREAM("Cannot set joint limits for " << joint.name);
			} else {
				PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
				positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
			}
			*/
			position_joint_interface_.registerHandle(steerPositionHandle);

			// Create velocity joint interface
			JointHandle driveVelocityHandle(driveStateHandle, &drive_velocity_command_);
			velocity_joint_interface_.registerHandle(driveVelocityHandle);

			// Create effort joint interface
			//JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
			//effort_joint_interface_.registerHandle(jointEffortHandle);

		//}

		registerInterface(&joint_state_interface_);
		registerInterface(&position_joint_interface_);
		registerInterface(&velocity_joint_interface_);
		//registerInterface(&effort_joint_interface_);
		//registerInterface(&positionJointSoftLimitsInterface);
	}

	void Kuuvef1HardwareInterface::update(const ros::TimerEvent& e)
	{
		_logInfo = "\n";
		/*
		_logInfo += "Joint Position Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointPositionStr;
			jointPositionStr << joint_position_command_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
		}
		*/

		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

		read();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write(elapsed_time_);

		//ROS_INFO_STREAM(_logInfo);
	}

	void Kuuvef1HardwareInterface::read()
	{
		_logInfo += "Joint State:\n";
		/*
		for (int i = 0; i < num_joints_; i++)
		{
			kuuvef1cpp::Joint joint = kuuvef1.getJoint(joint_names_[i]);

			if (joint.getActuatorType() == ACTUATOR_TYPE_MOTOR)
			{
				joint_position_[i] = joint.readAngle();

				std::ostringstream jointPositionStr;
				jointPositionStr << joint_position_[i];
				_logInfo += "  " + joint.name + ": " + jointPositionStr.str() + "\n";
			}

			kuuvef1.setJoint(joint);
		}
		*/

		kuuvef1_motor_.readSteerAndDrive(joint_position_[0], joint_velocity_[1]);
		std::ostringstream steerPositionStr;
		steerPositionStr << joint_position_[0];
		_logInfo += "  " + kuuvef1_motor_.getSteerName() + ": " + steerPositionStr.str() + "\n";

		std::ostringstream driveVelocityStr;
		driveVelocityStr << joint_velocity_[1];
		_logInfo += "  " + kuuvef1_motor_.getDriveName() + ": " + driveVelocityStr.str() + "\n";
	}

	void Kuuvef1HardwareInterface::write(ros::Duration elapsed_time)
	{
		//positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

		_logInfo += "Joint Command:\n";
		/*
		for (int i = 0; i < num_joints_; i++)
		{
			kuuvef1cpp::Joint joint = kuuvef1.getJoint(joint_names_[i]);
			//if (joint_effort_command_[i] > 1) joint_effort_command_[i] = 1;
			//if (joint_effort_command_[i] < -1) joint_effort_command_[i] = -1;

			double effort = joint_effort_command_[i];
			uint8_t duration = 15;

			if (joint.getActuatorType() == 1) { // servo
				double previousEffort = joint.getPreviousEffort();
				effort += previousEffort;
			}
			
			joint.actuate(effort, duration);

			std::ostringstream jointEffortStr;
			jointEffortStr << joint_effort_command_[i];
			_logInfo += "  " + joint.name + ": " + jointEffortStr.str() + "\n";
		}
		*/

		kuuvef1_motor_.actuate(steer_position_command_, drive_velocity_command_);

		std::ostringstream steerPositionStr;
		steerPositionStr << steer_position_command_;
		_logInfo += "  " + kuuvef1_motor_.getSteerName() + ": " + steerPositionStr.str() + "\n";

		std::ostringstream driveVelocityStr;
		driveVelocityStr << drive_velocity_command_;
		_logInfo += "  " + kuuvef1_motor_.getDriveName() + ": " + driveVelocityStr.str() + "\n";
	}
}

