#include "effector_control/force_application.hh"

namespace gazebo
{

	ForceApplication::ForceApplication() : WorldPlugin(), dataPtr(new ApplyWrenchDialogPrivate)
	{
		ros::init(argc, argv, "force_application");
		ros::NodeHandle n;

		this->force_sub = n.subscribe("/gazebo_force_app", 1000, force_callback);

		this->dataPtr->node = transport::NodePtr(new transport::Node());
		this->dataPtr->node->TryInit(common::Time::Maximum());

		this->dataPtr->userCmdPub =  this->dataPtr->node->Advertise<msgs::UserCmd>("~/user_cmd");

		this->spinner.start();
	}

	ForceApplication::~ForceApplication()
	{
		this->spinner.stop();
	}

	void ForceApplication::force_callback(WrenchStampedConstPtr& data)
	{
		msgs::Wrench msg = this->ros_wrench_msg_to_msg(data);
		std::string linkName = data.header.frame_id;

		// Register user command on server
		// The wrench will be applied from the server
		msgs::UserCmd userCmdMsg;
		userCmdMsg.set_description("Apply force to [" + linkName + "]");
		userCmdMsg.set_entity_name(linkName);
		userCmdMsg.set_type(msgs::UserCmd::WRENCH);
		userCmdMsg.mutable_wrench()->CopyFrom(msg);

		this->dataPtr->userCmdPub->Publish(userCmdMsg);
	}

	msgs::Wrench ForceApplication::ros_wrench_msg_to_msg(WrenchStampedConstPtr& data)
	{
		msgs::Wrench to_ret;

		float x = data.wrench.force.x;
		float y = data.wrench.force.y;
		float z = data.wrench.force.z;

		float phi = data.wrench.torque.x;
		float omega = data.wrench.torque.y;
		float psi = data.wrench.torque.z;

		ignition::math::Vector3d forceVector(x, y, z);
		ignition::math::Vector3d torqueVector(phi, omega, psi);

		msgs::Set(msg.mutable_force(), forceVector);
		msgs::Set(msg.mutable_torque(), torqueVector);
		msgs::Set(msg.mutable_force_offset(), ignition::math::Vector3d::zero);

		return to_ret;
	}
}