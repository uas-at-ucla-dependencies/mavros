/**
 * @brief MAVROS class
 * @file mavros.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <rclcpp/rclcpp.hpp>
#include <mavros/mavros.h>
#include <mavros/utils.h>
#include <fnmatch.h>

// MAVLINK_VERSION string
#include <mavlink/config.h>
#include <mavconn/mavlink_dialect.h>

using namespace mavros;
using mavconn::MAVConnInterface;
using mavconn::Framing;
using mavlink::mavlink_message_t;
using plugin::PluginBase;
using utils::enum_value;

rclcpp::Logger MavRos::logger = rclcpp::get_logger("mavros");


MavRos::MavRos(const rclcpp::NodeOptions& node_options) :
	rclcpp::Node("mavlink", "mavros", node_options),		// allow to namespace it
	clock(get_clock()),
	fcu_link_diag("FCU connection"),
	gcs_link_diag("GCS bridge"),
	plugin_loader("mavros", "mavros::plugin::PluginBase"),
	last_message_received_from_gcs(0),
	conn_timeout(0, 0),
	plugin_subscriptions{},
	mav_uas(this)
{
	std::string fcu_url, gcs_url;
	std::string fcu_protocol;
	int system_id, component_id;
	int tgt_system_id, tgt_component_id;
	bool px4_usb_quirk;
	double conn_timeout_d;
	std::vector<std::string> plugin_blacklist{}, plugin_whitelist{};
	MAVConnInterface::Ptr fcu_link;

	fcu_url = declare_parameter<std::string>("fcu_url", "serial:///dev/ttyACM0");
	gcs_url = declare_parameter<std::string>("gcs_url", "udp://@");
	gcs_quiet_mode = declare_parameter<bool>("gcs_quiet_mode", false);
	conn_timeout_d = declare_parameter<double>("conn/timeout", 30.0);

	fcu_protocol = declare_parameter<std::string>("fcu_protocol", "v2.0");
	system_id = declare_parameter<int>("system_id", 1);
	component_id = declare_parameter<int>("component_id", mavconn::MAV_COMP_ID_UDP_BRIDGE);
	tgt_system_id = declare_parameter<int>("target_system_id", 1);
	tgt_component_id = declare_parameter<int>("target_component_id", 1);
	px4_usb_quirk = declare_parameter<bool>("startup_px4_usb_quirk", false);
	plugin_blacklist = declare_parameter<std::vector<std::string>>("plugin_blacklist", {});
	plugin_whitelist = declare_parameter<std::vector<std::string>>("plugin_whitelist", {});

	conn_timeout = rclcpp::Duration(conn_timeout_d);

	// Now we use FCU URL as a hardware Id
	UAS_DIAG(&mav_uas).setHardwareID(fcu_url);

	RCLCPP_INFO_STREAM(logger, "FCU URL: " << fcu_url);
	try {
		fcu_link = MAVConnInterface::open_url(fcu_url, system_id, component_id);
		// may be overridden by URL
		system_id = fcu_link->get_system_id();
		component_id = fcu_link->get_component_id();

		fcu_link_diag.set_mavconn(fcu_link);
		UAS_DIAG(&mav_uas).add(fcu_link_diag);
	}
	catch (mavconn::DeviceError &ex) {
		RCLCPP_FATAL(logger, "FCU: %s", ex.what());
		rclcpp::shutdown();
		return;
	}

	if (fcu_protocol == "v1.0") {
		fcu_link->set_protocol_version(mavconn::Protocol::V10);
	}
	else if (fcu_protocol == "v2.0") {
		fcu_link->set_protocol_version(mavconn::Protocol::V20);
	}
	//else if (fcu_protocol == "auto") {	// XXX TODO
	//	fcu_link->set_protocol_version(mavconn::Protocol::V20);
	//}
	else {
		RCLCPP_WARN(logger, "Unknown FCU protocol: \"%s\", should be: \"v1.0\" or \"v2.0\". Used default v1.0.", fcu_protocol.c_str());
		fcu_link->set_protocol_version(mavconn::Protocol::V10);
	}

	if (gcs_url != "") {
		RCLCPP_INFO_STREAM(logger, "GCS URL: " << gcs_url);
		try {
			gcs_link = MAVConnInterface::open_url(gcs_url, system_id, component_id);

			gcs_link_diag.set_mavconn(gcs_link);
			UAS_DIAG(&mav_uas).setHardwareID(gcs_url);
			UAS_DIAG(&mav_uas).add(gcs_link_diag);
		}
		catch (mavconn::DeviceError &ex) {
			RCLCPP_FATAL(logger, "GCS: %s", ex.what());
			rclcpp::shutdown();
			return;
		}
	}
	else
		RCLCPP_INFO(logger, "GCS bridge disabled");

	// ROS mavlink bridge
	mavlink_pub = create_publisher<mavros_msgs::msg::Mavlink>("mavlink/from", 100);
	mavlink_sub = create_subscription<mavros_msgs::msg::Mavlink>("mavlink/to", rclcpp::QoS(100),
		std::bind(&MavRos::mavlink_sub_cb, this, std::placeholders::_1)); //,
		// ros::TransportHints()
		// 	.unreliable().maxDatagramSize(1024)
		// 	.reliable());

	// setup UAS and diag
	mav_uas.set_tgt(tgt_system_id, tgt_component_id);
	UAS_FCU(&mav_uas) = fcu_link;

	mav_uas.add_connection_change_handler(std::bind(&MavlinkDiag::set_connection_status, &fcu_link_diag, std::placeholders::_1));
	mav_uas.add_connection_change_handler(std::bind(&MavRos::log_connect_change, this, std::placeholders::_1));

	// prepare plugin lists
	// issue #257 2: assume that all plugins blacklisted
	if (plugin_blacklist.empty() and !plugin_whitelist.empty())
		plugin_blacklist.emplace_back("*");

	for (auto &name : plugin_loader.getDeclaredClasses())
		add_plugin(name, plugin_blacklist, plugin_whitelist);

	// connect FCU link

	// XXX TODO: move workers to ROS Spinner, let mavconn threads to do only IO
	fcu_link->message_received_cb = [this](const mavlink_message_t *msg, const Framing framing) {
		mavlink_pub_cb(msg, framing);
		plugin_route_cb(msg, framing);

		if (gcs_link) {
			if (this->gcs_quiet_mode && msg->msgid != mavlink::common::msg::HEARTBEAT::MSG_ID &&
				(clock->now() - this->last_message_received_from_gcs > this->conn_timeout)) {
				return;
			}

			gcs_link->send_message_ignore_drop(msg);
		}
	};

	fcu_link->port_closed_cb = []() {
		RCLCPP_ERROR(logger, "FCU connection closed, mavros will be terminated.");
		rclcpp::shutdown();
	};

	if (gcs_link) {
		// setup GCS link bridge
		gcs_link->message_received_cb = [this, fcu_link](const mavlink_message_t *msg, const Framing framing) {
			this->last_message_received_from_gcs = this->clock->now();
			fcu_link->send_message_ignore_drop(msg);
		};

		gcs_link_diag.set_connection_status(true);
	}

	if (px4_usb_quirk)
		startup_px4_usb_quirk();

	std::stringstream ss;
	for (auto &s : mavconn::MAVConnInterface::get_known_dialects())
		ss << " " << s;

	RCLCPP_INFO(logger, "Built-in SIMD instructions: %s", Eigen::SimdInstructionSetsInUse());
	RCLCPP_INFO(logger, "Built-in MAVLink package version: %s", MAVLINK_VERSION);
	RCLCPP_INFO(logger, "Known MAVLink dialects:%s", ss.str().c_str());
	RCLCPP_INFO(logger, "MAVROS started. MY ID %u.%u, TARGET ID %u.%u",
		system_id, component_id,
		tgt_system_id, tgt_component_id);
}

MavRos::~MavRos() {
	auto fcu_link = UAS_FCU(&mav_uas);
	if (fcu_link) {
		fcu_link->port_closed_cb = nullptr;
	}
}

void MavRos::spin()
{
	if (!rclcpp::ok()) {
		return;
	}
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(shared_from_this());
	executor.spin();

	RCLCPP_INFO(logger, "Stopping mavros...");
}

void MavRos::mavlink_pub_cb(const mavlink_message_t *mmsg, Framing framing)
{
	mavros_msgs::msg::Mavlink rmsg;

	if  (mavlink_pub->get_subscription_count() == 0)
		return;

	rmsg.header.stamp = clock->now();
	mavros_msgs::mavlink::convert(*mmsg, rmsg, enum_value(framing));
	mavlink_pub->publish(rmsg);
}

void MavRos::mavlink_sub_cb(const mavros_msgs::msg::Mavlink::UniquePtr rmsg)
{
	mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		UAS_FCU(&mav_uas)->send_message_ignore_drop(&mmsg);
	else
		RCLCPP_ERROR(logger, "Drop mavlink packet: convert error.");
}

void MavRos::plugin_route_cb(const mavlink_message_t *mmsg, const Framing framing)
{
	auto it = plugin_subscriptions.find(mmsg->msgid);
	if (it == plugin_subscriptions.end())
		return;

	for (auto &info : it->second)
		std::get<3>(info)(mmsg, framing);
}

static bool pattern_match(std::string &pattern, std::string &pl_name)
{
	int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
	if (cmp == 0)
		return true;
	else if (cmp != FNM_NOMATCH) {
		// never see that, i think that it is fatal error.
		RCLCPP_FATAL(MavRos::logger, "Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
				pattern.c_str(), pl_name.c_str(), cmp);
		rclcpp::shutdown();
	}

	return false;
}

/**
 * @brief Checks that plugin blacklisted
 *
 * Operation algo:
 *
 *  1. if blacklist and whitelist is empty: load all
 *  2. if blacklist is empty and whitelist non empty: assume blacklist is ["*"]
 *  3. if blacklist non empty: usual blacklist behavior
 *  4. if whitelist non empty: override blacklist
 *
 * @note Issue #257.
 */
static bool is_blacklisted(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist)
{
	for (auto &bl_pattern : blacklist) {
		if (pattern_match(bl_pattern, pl_name)) {
			for (auto &wl_pattern : whitelist) {
				if (pattern_match(wl_pattern, pl_name))
					return false;
			}

			return true;
		}
	}

	return false;
}

inline bool is_mavlink_message_t(const size_t rt)
{
	static const auto h = typeid(mavlink_message_t).hash_code();
	return h == rt;
}

/**
 * @brief Loads plugin (if not blacklisted)
 */
void MavRos::add_plugin(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist)
{
	if (is_blacklisted(pl_name, blacklist, whitelist)) {
		RCLCPP_INFO(logger, "Plugin %s blacklisted", pl_name.c_str());
		return;
	}

	try {
		auto plugin = plugin_loader.createSharedInstance(pl_name);

		RCLCPP_INFO(logger, "Plugin %s loaded", pl_name.c_str());

		for (auto &info : plugin->get_subscriptions()) {
			auto msgid = std::get<0>(info);
			auto msgname = std::get<1>(info);
			auto type_hash_ = std::get<2>(info);

			std::string log_msgname;

			if (is_mavlink_message_t(type_hash_))
				log_msgname = utils::format("MSG-ID (%u) <%zu>", msgid, type_hash_);
			else
				log_msgname = utils::format("%s (%u) <%zu>", msgname, msgid, type_hash_);

			RCLCPP_DEBUG(logger, "Route %s to %s", log_msgname.c_str(), pl_name.c_str());

			auto it = plugin_subscriptions.find(msgid);
			if (it == plugin_subscriptions.end()) {
				// new entry

				RCLCPP_DEBUG(logger, "%s - new element", log_msgname.c_str());
				plugin_subscriptions[msgid] = PluginBase::Subscriptions{{info}};
			}
			else {
				// existing: check handler message type

				bool append_allowed = is_mavlink_message_t(type_hash_);
				if (!append_allowed) {
					append_allowed = true;
					for (auto &e : it->second) {
						auto t2 = std::get<2>(e);
						if (!is_mavlink_message_t(t2) && t2 != type_hash_) {
							RCLCPP_ERROR(logger, "%s routed to different message type (hash: %z)",
								log_msgname.c_str(), t2);
							append_allowed = false;
						}
					}
				}

				if (append_allowed) {
					RCLCPP_DEBUG(logger, "%s - emplace", log_msgname.c_str());
					it->second.emplace_back(info);
				}
				else
					RCLCPP_ERROR(logger, "%s handler dropped because this ID are used for another message type", log_msgname.c_str());
			}
		}

		plugin->initialize(mav_uas);
		loaded_plugins.push_back(plugin);

		RCLCPP_INFO(logger, "Plugin %s initialized", pl_name.c_str());
	} catch (pluginlib::PluginlibException &ex) {
		RCLCPP_ERROR(logger, "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
	}
}

void MavRos::startup_px4_usb_quirk()
{
       /* sample code from QGC */
       const uint8_t init[] = {0x0d, 0x0d, 0x0d, 0};
       const uint8_t nsh[] = "sh /etc/init.d/rc.usb\n";

       RCLCPP_INFO(logger, "Autostarting mavlink via USB on PX4");
       UAS_FCU(&mav_uas)->send_bytes(init, 3);
       UAS_FCU(&mav_uas)->send_bytes(nsh, sizeof(nsh) - 1);
       UAS_FCU(&mav_uas)->send_bytes(init, 4); /* NOTE in original init[3] */
}

void MavRos::log_connect_change(bool connected)
{
	auto ap = utils::to_string(mav_uas.get_autopilot());

	/* note: sys_status plugin required */
	if (connected)
		RCLCPP_INFO(logger, "CON: Got HEARTBEAT, connected. FCU: %s", ap.c_str());
	else
		RCLCPP_WARN(logger, "CON: Lost connection, HEARTBEAT timed out.");
}
