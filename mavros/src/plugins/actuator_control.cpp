/**
 * @brief ActuatorControl plugin
 * @file actuator_control.cpp
 * @author Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/msg/actuator_control.hpp>

namespace mavros {
namespace std_plugins {
/**
 * @brief ActuatorControl plugin
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorControlPlugin : public plugin::PluginBase {
public:
	ActuatorControlPlugin() : PluginBase()
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		nh = uas_.mavros_node;

		target_actuator_control_pub = nh->create_publisher<mavros_msgs::msg::ActuatorControl>("target_actuator_control", 10);
		actuator_control_sub = nh->create_subscription<mavros_msgs::msg::ActuatorControl>("actuator_control", 10, std::bind(&ActuatorControlPlugin::actuator_control_cb, this, std::placeholders::_1));
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ActuatorControlPlugin::handle_actuator_control_target),
		};
	}

private:
	rclcpp::Node* nh;

	rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr target_actuator_control_pub;
	rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_control_sub;

	/* -*- rx handlers -*- */

	void handle_actuator_control_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ACTUATOR_CONTROL_TARGET &actuator_control_target)
	{
		auto actuator_control_target_msg = std::make_shared<mavros_msgs::msg::ActuatorControl>();
		actuator_control_target_msg->header.stamp = m_uas->synchronise_stamp(actuator_control_target.time_usec);

		actuator_control_target_msg->group_mix = actuator_control_target.group_mlx;
		const auto &arr = actuator_control_target.controls;
		std::copy(arr.cbegin(), arr.cend(), actuator_control_target_msg->controls.begin());

		target_actuator_control_pub->publish(*actuator_control_target_msg);
	}

	/* -*- callbacks -*- */

	void actuator_control_cb(const mavros_msgs::msg::ActuatorControl::UniquePtr req) {
		//! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
		//! message definiton here: @p https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET
		mavlink::common::msg::SET_ACTUATOR_CONTROL_TARGET act{};

		act.time_usec = rclcpp::Time(req->header.stamp).nanoseconds() / 1000;
		act.group_mlx = req->group_mix;
		act.target_system = m_uas->get_tgt_system();
		act.target_component = m_uas->get_tgt_component();
		std::copy(req->controls.begin(), req->controls.end(), act.controls.begin());	// std::array = std::array

		UAS_FCU(m_uas)->send_message_ignore_drop(act);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ActuatorControlPlugin, mavros::plugin::PluginBase)
