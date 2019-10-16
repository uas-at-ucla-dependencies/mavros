/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace mavros {
namespace std_plugins {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public plugin::PluginBase {
public:
	LocalPositionPlugin() : PluginBase(),
		tf_send(false),
		has_local_position_ned(false),
		has_local_position_ned_cov(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		lp_nh = uas_.mavros_node->create_sub_node("local_position"),

		// header frame_id.
		// default to map (world-fixed,ENU as per REP-105).
		frame_id = lp_nh->declare_parameter<std::string>("frame_id", "map");
		// Important tf subsection
		// Report the transform from world to base_link here.
		tf_send = lp_nh->declare_parameter("tf/send", false);
		tf_frame_id = lp_nh->declare_parameter<std::string>("tf/frame_id", "map");
		tf_child_frame_id = lp_nh->declare_parameter<std::string>("tf/child_frame_id", "base_link");

		local_position = lp_nh->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
		local_position_cov = lp_nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_cov", 10);
		local_velocity_local = lp_nh->create_publisher<geometry_msgs::msg::TwistStamped>("velocity_local", 10);
		local_velocity_body = lp_nh->create_publisher<geometry_msgs::msg::TwistStamped>("velocity_body", 10);
		local_velocity_cov = lp_nh->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("velocity_body_cov", 10);
		local_accel = lp_nh->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("accel", 10);
		local_odom = lp_nh->create_publisher<nav_msgs::msg::Odometry>("odom",10);
	}

	Subscriptions get_subscriptions() {
		return {
			       make_handler(&LocalPositionPlugin::handle_local_position_ned),
			       make_handler(&LocalPositionPlugin::handle_local_position_ned_cov)
		};
	}

private:
	rclcpp::Node::SharedPtr lp_nh;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr local_position_cov;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_local;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_body;
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr local_velocity_cov;
	rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr local_accel;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;
	bool has_local_position_ned;
	bool has_local_position_ned_cov;

	void publish_tf(std::shared_ptr<nav_msgs::msg::Odometry> &odom)
	{
		if (tf_send) {
			geometry_msgs::msg::TransformStamped transform;
			transform.header.stamp = odom->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;
			transform.transform.translation.x = odom->pose.pose.position.x;
			transform.transform.translation.y = odom->pose.pose.position.y;
			transform.transform.translation.z = odom->pose.pose.position.z;
			transform.transform.rotation = odom->pose.pose.orientation;
			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	void handle_local_position_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED &pos_ned)
	{
		has_local_position_ned = true;

		//--------------- Transform FCU position and Velocity Data ---------------//
		auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

		//--------------- Get Odom Information ---------------//
		// Note this orientation describes baselink->ENU transform
		auto enu_orientation_msg = m_uas->get_attitude_orientation_enu();
		auto baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
		auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

		auto odom = std::make_shared<nav_msgs::msg::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
		odom->child_frame_id = tf_child_frame_id;

		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		odom->pose.pose.orientation = enu_orientation_msg;
		tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;

		// publish odom if we don't have LOCAL_POSITION_NED_COV
		if (!has_local_position_ned_cov) {
			local_odom->publish(*odom);
		}

		// publish pose always
		auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
		pose->header = odom->header;
		pose->pose = odom->pose.pose;
		local_position->publish(*pose);

		// publish velocity always
		// velocity in the body frame
		auto twist_body = std::make_shared<geometry_msgs::msg::TwistStamped>();
		twist_body->header.stamp = odom->header.stamp;
		twist_body->header.frame_id = tf_child_frame_id;
		twist_body->twist.linear = odom->twist.twist.linear;
		twist_body->twist.angular = baselink_angular_msg;
		local_velocity_body->publish(*twist_body);

		// velocity in the local frame
		auto twist_local = std::make_shared<geometry_msgs::msg::TwistStamped>();
		twist_local->header.stamp = twist_body->header.stamp;
		twist_local->header.frame_id = tf_child_frame_id;
		tf::vectorEigenToMsg(enu_velocity, twist_local->twist.linear);
		tf::vectorEigenToMsg(ftf::transform_frame_baselink_enu(ftf::to_eigen(baselink_angular_msg), enu_orientation),
						twist_body->twist.angular);
		local_velocity_local->publish(*twist_local);

		// publish tf
		publish_tf(odom);
	}

	void handle_local_position_ned_cov(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED_COV &pos_ned)
	{
		has_local_position_ned_cov = true;

		auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

		auto enu_orientation_msg = m_uas->get_attitude_orientation_enu();
		auto baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
		auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

		auto odom = std::make_shared<nav_msgs::msg::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, pos_ned.time_usec);
		odom->child_frame_id = tf_child_frame_id;

		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		odom->pose.pose.orientation = enu_orientation_msg;
		tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;

		odom->pose.covariance[0] = pos_ned.covariance[0];	// x
		odom->pose.covariance[7] = pos_ned.covariance[9];	// y
		odom->pose.covariance[14] = pos_ned.covariance[17];	// z

		odom->twist.covariance[0] = pos_ned.covariance[24];	// vx
		odom->twist.covariance[7] = pos_ned.covariance[30];	// vy
		odom->twist.covariance[14] = pos_ned.covariance[35];	// vz
		// TODO: orientation + angular velocity covariances from ATTITUDE_QUATERION_COV

		// publish odom always
		local_odom->publish(*odom);

		// publish pose_cov always
		auto pose_cov = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
		pose_cov->header = odom->header;
		pose_cov->pose = odom->pose;
		local_position_cov->publish(*pose_cov);

		// publish velocity_cov always
		auto twist_cov = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = odom->header.stamp;
		twist_cov->header.frame_id = odom->child_frame_id;
		twist_cov->twist = odom->twist;
		local_velocity_cov->publish(*twist_cov);

		// publish pose, velocity, tf if we don't have LOCAL_POSITION_NED
		if (!has_local_position_ned) {
			auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
			pose->header = odom->header;
			pose->pose = odom->pose.pose;
			local_position->publish(*pose);

			auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
			twist->header.stamp = odom->header.stamp;
			twist->header.frame_id = odom->child_frame_id;
			twist->twist = odom->twist.twist;
			local_velocity_body->publish(*twist);

			// publish tf
			publish_tf(odom);
		}

		// publish accelerations
		auto accel = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
		accel->header = odom->header;

		auto enu_accel = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.ax, pos_ned.ay, pos_ned.az));
		tf::vectorEigenToMsg(enu_accel, accel->accel.accel.linear);

		accel->accel.covariance[0] = pos_ned.covariance[39];	// ax
		accel->accel.covariance[7] = pos_ned.covariance[42];	// ay
		accel->accel.covariance[14] = pos_ned.covariance[44];	// az

		local_accel->publish(*accel);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LocalPositionPlugin, mavros::plugin::PluginBase)
