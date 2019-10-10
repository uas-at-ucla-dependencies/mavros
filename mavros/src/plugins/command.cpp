/**
 * @brief Command plugin
 * @file command.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
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

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_int.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_trigger_control.hpp>
#include <mavros_msgs/srv/command_trigger_interval.hpp>

namespace mavros {
namespace std_plugins {
using utils::enum_value;
using lock_guard = std::lock_guard<std::mutex>;
using unique_lock = std::unique_lock<std::mutex>;

class CommandTransaction {
public:
	std::mutex cond_mutex;
	std::condition_variable ack;
	uint16_t expected_command;
	uint8_t result;

	explicit CommandTransaction(uint16_t command) :
		ack(),
		expected_command(command),
		// Default result if wait ack timeout
		result(enum_value(mavlink::common::MAV_RESULT::FAILED))
	{ }
};

/**
 * @brief Command plugin.
 *
 * Send any command via COMMAND_LONG
 */
class CommandPlugin : public plugin::PluginBase {
public:
	CommandPlugin() : PluginBase(),
		use_comp_id_system_control(false),
		command_ack_timeout_dt(0, 0)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		
		cmd_nh = uas_.mavros_node->create_sub_node("~cmd");

		double command_ack_timeout;

		command_ack_timeout = cmd_nh->declare_parameter<double>("command_ack_timeout", ACK_TIMEOUT_DEFAULT);
		use_comp_id_system_control = cmd_nh->declare_parameter<bool>("use_comp_id_system_control", false);

		command_ack_timeout_dt = rclcpp::Duration(command_ack_timeout);

		command_long_srv = cmd_nh->create_service<mavros_msgs::srv::CommandLong>("command", std::bind(&CommandPlugin::command_long_cb, this, std::placeholders::_1));
		command_int_srv = cmd_nh->create_service<mavros_msgs::srv::CommandInt>("command_int", std::bind(&CommandPlugin::command_int_cb, this, std::placeholders::_1));
		arming_srv = cmd_nh->create_service<mavros_msgs::srv::CommandBool>("arming", std::bind(&CommandPlugin::arming_cb, this, std::placeholders::_1));
		set_home_srv = cmd_nh->create_service<mavros_msgs::srv::CommandHome>("set_home", std::bind(&CommandPlugin::set_home_cb, this, std::placeholders::_1));
		takeoff_srv = cmd_nh->create_service<mavros_msgs::srv::CommandTOL>("takeoff", std::bind(&CommandPlugin::takeoff_cb, this, std::placeholders::_1));
		land_srv = cmd_nh->create_service<mavros_msgs::srv::CommandTOL>("land", std::bind(&CommandPlugin::land_cb, this, std::placeholders::_1));
		trigger_control_srv = cmd_nh->create_service<mavros_msgs::srv::CommandTriggerControl>("trigger_control", std::bind(&CommandPlugin::trigger_control_cb, this, std::placeholders::_1));
		trigger_interval_srv = cmd_nh->create_service<mavros_msgs::srv::CommandTriggerInterval>("trigger_interval", std::bind(&CommandPlugin::trigger_interval_cb, this, std::placeholders::_1));
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&CommandPlugin::handle_command_ack)
		};
	}

private:
	using L_CommandTransaction = std::list<CommandTransaction>;

	std::mutex mutex;

	rclcpp::Node::SharedPtr cmd_nh;
	rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_long_srv;
	rclcpp::Service<mavros_msgs::srv::CommandInt>::SharedPtr command_int_srv;
	rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv;
	rclcpp::Service<mavros_msgs::srv::CommandHome>::SharedPtr set_home_srv;
	rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_srv;
	rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr land_srv;
	rclcpp::Service<mavros_msgs::srv::CommandTriggerControl>::SharedPtr trigger_control_srv;
	rclcpp::Service<mavros_msgs::srv::CommandTriggerInterval>::SharedPtr trigger_interval_srv;

	bool use_comp_id_system_control;

	L_CommandTransaction ack_waiting_list;
	static constexpr double ACK_TIMEOUT_DEFAULT = 5.0;
	rclcpp::Duration command_ack_timeout_dt;

	/* -*- message handlers -*- */

	void handle_command_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &ack)
	{
		lock_guard lock(mutex);

		// XXX(vooon): place here source ids check

		for (auto &tr : ack_waiting_list) {
			if (tr.expected_command == ack.command) {
				tr.result = ack.result;
				tr.ack.notify_all();
				return;
			}
		}

		RCUTILS_LOG_WARN_THROTTLE_NAMED(,10, "cmd", "CMD: Unexpected command %u, result %u",
			ack.command, ack.result);
	}

	/* -*- mid-level functions -*- */

	bool wait_ack_for(CommandTransaction &tr) {
		unique_lock lock(tr.cond_mutex);
		if (tr.ack.wait_for(lock, std::chrono::nanoseconds(command_ack_timeout_dt.nanoseconds())) == std::cv_status::timeout) {
			RCUTILS_LOG_WARN_NAMED("cmd", "CMD: Command %u -- wait ack timeout", tr.expected_command);
			return false;
		} else {
			return true;
		}
	}

	/**
	 * Common function for command service callbacks.
	 *
	 * NOTE: success is bool in messages, but has unsigned char type in C++
	 */
	bool send_command_long_and_wait(bool broadcast,
		uint16_t command, uint8_t confirmation,
		float param1, float param2,
		float param3, float param4,
		float param5, float param6,
		float param7,
		bool &success, uint8_t &result)
	{
		using mavlink::common::MAV_RESULT;

		unique_lock lock(mutex);

		L_CommandTransaction::iterator ack_it;

		/* check transactions */
		for (const auto &tr : ack_waiting_list) {
			if (tr.expected_command == command) {
				RCUTILS_LOG_WARN_THROTTLE_NAMED(,10, "cmd", "CMD: Command %u already in progress", command);
				return false;
			}
		}

		/**
		 * @note APM & PX4 master always send COMMAND_ACK. Old PX4 never.
		 * Don't expect any ACK in broadcast mode.
		 */
		bool is_ack_required = (confirmation != 0 || m_uas->is_ardupilotmega() || m_uas->is_px4()) && !broadcast;
		if (is_ack_required)
			ack_it = ack_waiting_list.emplace(ack_waiting_list.end(), command);

		command_long(broadcast,
			command, confirmation,
			param1, param2,
			param3, param4,
			param5, param6,
			param7);

		if (is_ack_required) {
			lock.unlock();
			bool is_not_timeout = wait_ack_for(*ack_it);
			lock.lock();

			success = is_not_timeout && ack_it->result == enum_value(MAV_RESULT::ACCEPTED);
			result = ack_it->result;

			ack_waiting_list.erase(ack_it);
		}
		else {
			success = true;
			result = enum_value(MAV_RESULT::ACCEPTED);
		}

		return true;
	}

	/**
	 * Common function for COMMAND_INT service callbacks.
	 */
	bool send_command_int(bool broadcast,
		uint8_t frame, uint16_t command,
		uint8_t current, uint8_t autocontinue,
		float param1, float param2,
		float param3, float param4,
		int32_t x, int32_t y,
		float z,
		bool &success)
	{
		/* Note: seems that COMMAND_INT don't produce COMMAND_ACK
		 * so wait don't needed.
		 */
		command_int(broadcast,
			frame, command, current, autocontinue,
			param1, param2,
			param3, param4,
			x, y, z);

		success = true;
		return true;
	}

	/* -*- low-level send -*- */

	template<typename MsgT>
	inline void set_target(MsgT &cmd, bool broadcast)
	{
		using mavlink::common::MAV_COMPONENT;

		const uint8_t tgt_sys_id = (broadcast) ? 0 : m_uas->get_tgt_system();
		const uint8_t tgt_comp_id = (broadcast) ? 0 :
			(use_comp_id_system_control) ?
			enum_value(MAV_COMPONENT::COMP_ID_SYSTEM_CONTROL) : m_uas->get_tgt_component();

		cmd.target_system = tgt_sys_id;
		cmd.target_component = tgt_comp_id;
	}

	void command_long(bool broadcast,
		uint16_t command, uint8_t confirmation,
		float param1, float param2,
		float param3, float param4,
		float param5, float param6,
		float param7)
	{
		const uint8_t confirmation_fixed = (broadcast) ? 0 : confirmation;

		mavlink::common::msg::COMMAND_LONG cmd {};
		set_target(cmd, broadcast);

		cmd.command = command;
		cmd.confirmation = confirmation_fixed;
		cmd.param1 = param1;
		cmd.param2 = param2;
		cmd.param3 = param3;
		cmd.param4 = param4;
		cmd.param5 = param5;
		cmd.param6 = param6;
		cmd.param7 = param7;

		UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
	}

	void command_int(bool broadcast,
		uint8_t frame, uint16_t command,
		uint8_t current, uint8_t autocontinue,
		float param1, float param2,
		float param3, float param4,
		int32_t x, int32_t y,
		float z)
	{
		mavlink::common::msg::COMMAND_INT cmd {};
		set_target(cmd, broadcast);

		cmd.frame = frame;
		cmd.command = command;
		cmd.current = current;
		cmd.autocontinue = autocontinue;
		cmd.param1 = param1;
		cmd.param2 = param2;
		cmd.param3 = param3;
		cmd.param4 = param4;
		cmd.x = x;
		cmd.y = y;
		cmd.z = z;

		UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
	}

	/* -*- callbacks -*- */

	bool command_long_cb(mavros_msgs::srv::CommandLong::Request &req,
		mavros_msgs::srv::CommandLong::Response &res)
	{
		return send_command_long_and_wait(req.broadcast,
			req.command, req.confirmation,
			req.param1, req.param2,
			req.param3, req.param4,
			req.param5, req.param6,
			req.param7,
			res.success, res.result);
	}

	bool command_int_cb(mavros_msgs::srv::CommandInt::Request &req,
		mavros_msgs::srv::CommandInt::Response &res)
	{
		return send_command_int(req.broadcast,
			req.frame, req.command,
			req.current, req.autocontinue,
			req.param1, req.param2,
			req.param3, req.param4,
			req.x, req.y, req.z,
			res.success);
	}

	bool arming_cb(mavros_msgs::srv::CommandBool::Request &req,
		mavros_msgs::srv::CommandBool::Response &res)
	{
		using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			enum_value(MAV_CMD::COMPONENT_ARM_DISARM), 1,
			(req.value) ? 1.0 : 0.0,
			0, 0, 0, 0, 0, 0,
			res.success, res.result);
	}

	bool set_home_cb(mavros_msgs::srv::CommandHome::Request &req,
		mavros_msgs::srv::CommandHome::Response &res)
	{
		using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			enum_value(MAV_CMD::DO_SET_HOME), 1,
			(req.current_gps) ? 1.0 : 0.0,
			0, 0, 0, req.latitude, req.longitude, req.altitude,
			res.success, res.result);
	}

	bool takeoff_cb(mavros_msgs::srv::CommandTOL::Request &req,
		mavros_msgs::srv::CommandTOL::Response &res)
	{
		using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			enum_value(MAV_CMD::NAV_TAKEOFF), 1,
			req.min_pitch,
			0, 0,
			req.yaw,
			req.latitude, req.longitude, req.altitude,
			res.success, res.result);
	}

	bool land_cb(mavros_msgs::srv::CommandTOL::Request &req,
		mavros_msgs::srv::CommandTOL::Response &res)
	{
		using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			enum_value(MAV_CMD::NAV_LAND), 1,
			0, 0, 0,
			req.yaw,
			req.latitude, req.longitude, req.altitude,
			res.success, res.result);
	}

	bool trigger_control_cb(mavros_msgs::srv::CommandTriggerControl::Request &req,
		mavros_msgs::srv::CommandTriggerControl::Response &res)
	{
		using mavlink::common::MAV_CMD;
		return send_command_long_and_wait(false,
			enum_value(MAV_CMD::DO_TRIGGER_CONTROL), 1,
			(req.trigger_enable) ? 1.0 : 0.0,
			(req.sequence_reset) ? 1.0 : 0.0,
			(req.trigger_pause) ? 1.0 : 0.0,
			0, 0, 0, 0,
			res.success, res.result);
	}

	bool trigger_interval_cb(mavros_msgs::srv::CommandTriggerInterval::Request &req,
		mavros_msgs::srv::CommandTriggerInterval::Response &res)
	{
		using mavlink::common::MAV_CMD;

		// trigger interval can only be set when triggering is disabled
		return send_command_long_and_wait(false,
			enum_value(MAV_CMD::DO_SET_CAM_TRIGG_INTERVAL), 1,
			req.cycle_time,
			req.integration_time,
			0, 0, 0, 0, 0,
			res.success, res.result);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::CommandPlugin, mavros::plugin::PluginBase)
