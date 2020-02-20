/**
 * @brief HomePosition plugin
 * @file home_position.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 Thomas Stastny, Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <mavros/mavros_plugin.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_srvs/srv/trigger.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/home_position.hpp>


namespace mavros {
namespace std_plugins {
/**
 * @brief home position plugin.
 *
 * Publishes home position.
 */
class HomePositionPlugin : public plugin::PluginBase {
public:
	HomePositionPlugin() :
		hp_nh(rclcpp::Node::make_shared("home_position", "mavros")),
		REQUEST_POLL_TIME_DT(REQUEST_POLL_TIME_MS / 1000.0)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		hp_pub = hp_nh->create_publisher<mavros_msgs::msg::HomePosition>("~/home", 2);
		hp_sub = hp_nh->create_subscription<mavros_msgs::msg::HomePosition>("~/set", 10,
			std::bind(&HomePositionPlugin::home_position_cb, this, std::placeholders::_1));
		update_srv = hp_nh->create_service<std_srvs::srv::Trigger>("~/req_update",
			std::bind(&HomePositionPlugin::req_update_cb, this, std::placeholders::_1, std::placeholders::_2));

		poll_timer = hp_nh->create_wall_timer(REQUEST_POLL_TIME_DT, std::bind(&HomePositionPlugin::timeout_cb, this));
		poll_timer->cancel();
		enable_connection_cb();
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&HomePositionPlugin::handle_home_position),
		};
	}

private:
	rclcpp::Node::SharedPtr hp_nh;

	rclcpp::Publisher<mavros_msgs::msg::HomePosition>::SharedPtr hp_pub;
	rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr hp_sub;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_srv;

	rclcpp::TimerBase::SharedPtr poll_timer;

	static constexpr int REQUEST_POLL_TIME_MS = 10000;	//! position refresh poll interval
	const std::chrono::duration<double> REQUEST_POLL_TIME_DT;

	bool call_get_home_position(void)
	{
		using mavlink::common::MAV_CMD;

		bool ret = false;

		try {
			auto client = hp_nh->create_client<mavros_msgs::srv::CommandLong>("cmd/command");

			auto cmd = std::make_shared<mavros_msgs::srv::CommandLong::Request>();

			cmd->command = utils::enum_value(MAV_CMD::GET_HOME_POSITION);

			auto future = client->async_send_request(cmd);
			auto wait_result = rclcpp::spin_until_future_complete(hp_nh, future);
			ret = wait_result == rclcpp::executor::FutureReturnCode::SUCCESS;
		}
		catch (rclcpp::exceptions::InvalidServiceNameError &ex) {
			RCUTILS_LOG_ERROR_NAMED("home_position", "HP: %s", ex.what());
		}

		return ret;
	}

	void handle_home_position(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HOME_POSITION &home_position)
	{
		poll_timer->cancel();

		auto hp = std::make_shared<mavros_msgs::msg::HomePosition>();

		auto pos = ftf::transform_frame_ned_enu(Eigen::Vector3d(home_position.x, home_position.y, home_position.z));
		auto q = ftf::transform_orientation_ned_enu(ftf::mavlink_to_quaternion(home_position.q));
		auto hp_approach_enu = ftf::transform_frame_ned_enu(Eigen::Vector3d(home_position.approach_x, home_position.approach_y, home_position.approach_z));

		hp->header.stamp = rclcpp::Clock().now();
		hp->geo.latitude = home_position.latitude / 1E7;		// deg
		hp->geo.longitude = home_position.longitude / 1E7;		// deg
		hp->geo.altitude = home_position.altitude / 1E3 + m_uas->geoid_to_ellipsoid_height(&hp->geo);	// in meters
		tf2::convert(q, hp->orientation);
		tf2::convert(pos, hp->position);
		tf2::toMsg(hp_approach_enu, hp->approach);

		RCUTILS_LOG_DEBUG_NAMED("home_position", "HP: Home lat %f, long %f, alt %f", hp->geo.latitude, hp->geo.longitude, hp->geo.altitude);
		hp_pub->publish(*hp);
	}

	void home_position_cb(const mavros_msgs::msg::HomePosition::SharedPtr req)
	{
		mavlink::common::msg::SET_HOME_POSITION hp {};

		Eigen::Vector3d pos, approach;
		Eigen::Quaterniond q;

		tf2::convert(req->position, pos);
		pos = ftf::transform_frame_enu_ned(pos);

		tf2::fromMsg(req->orientation, q);
		q = ftf::transform_orientation_enu_ned(q);

		tf2::fromMsg(req->approach, approach);
		approach = ftf::transform_frame_enu_ned(approach);

		hp.target_system = m_uas->get_tgt_system();
		ftf::quaternion_to_mavlink(q, hp.q);

		hp.altitude = req->geo.altitude * 1e3 + m_uas->ellipsoid_to_geoid_height(&req->geo);
		// [[[cog:
		// for f, m in (('latitude', '1e7'), ('longitude', '1e7')):
		//     cog.outl("hp.{f} = req->geo.{f} * {m};".format(**locals()))
		// for a, b in (('', 'pos'), ('approach_', 'approach')):
		//     for f in "xyz":
		//         cog.outl("hp.{a}{f} = {b}.{f}();".format(**locals()))
		// ]]]
		hp.latitude = req->geo.latitude * 1e7;
		hp.longitude = req->geo.longitude * 1e7;
		hp.x = pos.x();
		hp.y = pos.y();
		hp.z = pos.z();
		hp.approach_x = approach.x();
		hp.approach_y = approach.y();
		hp.approach_z = approach.z();
		// [[[end]]] (checksum: 9c40c5b3ac06b3b82016b4f07a8e12b2)

		UAS_FCU(m_uas)->send_message_ignore_drop(hp);
	}

	bool req_update_cb(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
	{
		res->success = call_get_home_position();
		return true;
	}

	void timeout_cb()
	{
		RCUTILS_LOG_INFO_NAMED("home_position", "HP: requesting home position");
		call_get_home_position();
	}

	void connection_cb(bool connected) override
	{
		if (connected)
			poll_timer->reset();
		else
			poll_timer->cancel();
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HomePositionPlugin, mavros::plugin::PluginBase)
