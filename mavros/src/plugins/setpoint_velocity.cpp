/**
 * @brief SetpointVelocity plugin
 * @file setpoint_velocity.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <mavros_msgs/msg/set_mav_frame.hpp>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief Setpoint velocity plugin
 *
 * Send setpoint velocities to FCU controller.
 */
class SetpointVelocityPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin> {
public:
	SetpointVelocityPlugin() : PluginBase(),
		sp_nh("~setpoint_velocity")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		//cmd_vel usually is the topic used for velocity control in many controllers / planners
		vel_sub = sp_nh->create_subscription("cmd_vel", 10, std::bind(&SetpointVelocityPlugin, this, std::placeholders::_1));
		vel_unstamped_sub = sp_nh->create_subscription("cmd_vel_unstamped", 10, std::bind(&SetpointVelocityPlugin, this, std::placeholders::_1));
		mav_frame_srv = sp_nh->create_service("mav_frame", std::bind(&SetpointVelocityPlugin, this, std::placeholders::_1));

		// mav_frame
		std::string mav_frame_str;
		if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
			mav_frame = MAV_FRAME::LOCAL_NED;
		} else {
			mav_frame = utils::mav_frame_from_str(mav_frame_str);
		}
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	rclcpp::Node::SharedPtr sp_nh;

	rclcpp::Subscription<>::SharedPtr vel_sub;
	rclcpp::Subscription<>::SharedPtr vel_unstamped_sub;
	rclcpp::Service<>::SharedPtr mav_frame_srv;

	MAV_FRAME mav_frame;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send velocity to FCU velocity controller
	 *
	 * @warning Send only VX VY VZ. ENU frame.
	 */
	void send_setpoint_velocity(const rclcpp::Time &stamp, Eigen::Vector3d &vel_enu, double yaw_rate)
	{
		/**
		 * Documentation start from bit 1 instead 0;
		 * Ignore position and accel vectors, yaw.
		 */
		uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);
		auto vel = [&] () {
			if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
				return ftf::transform_frame_baselink_aircraft(vel_enu);
			} else {
				return ftf::transform_frame_enu_ned(vel_enu);
			}
		} ();

		auto yr = [&] () {
			if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
				return ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(0.0, 0.0, yaw_rate));
			} else {
				return ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate));
			}
		} ();

		set_position_target_local_ned(stamp.toNSec() / 1000000,
					utils::enum_value(mav_frame),
					ignore_all_except_v_xyz_yr,
					Eigen::Vector3d::Zero(),
					vel,
					Eigen::Vector3d::Zero(),
					0.0, yr.z());
	}

	/* -*- callbacks -*- */

	void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr req) {
		Eigen::Vector3d vel_enu;

		tf2::convert(req->twist.linear, vel_enu);
		send_setpoint_velocity(req->header.stamp, vel_enu,
					req->twist.angular.z);
	}

	void vel_unstamped_cb(const geometry_msgs::msg::Twist::SharedPtr req) {
		Eigen::Vector3d vel_enu;

		tf2::convert(req->linear, vel_enu);
		send_setpoint_velocity(rclcpp::Time::now(), vel_enu,
					req->angular.z);
	}

	bool set_mav_frame_cb(mavros_msgs::msg::SetMavFrame::Request::SharedPtr req, mavros_msgs::msg::SetMavFrame::Response::SharedPtr res)
	{
		mav_frame = static_cast<MAV_FRAME>(req.mav_frame);
		const std::string mav_frame_str = utils::to_string(mav_frame);
		sp_nh.setParam("mav_frame", mav_frame_str);
		res.success = true;
		return true;
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointVelocityPlugin, mavros::plugin::PluginBase)
