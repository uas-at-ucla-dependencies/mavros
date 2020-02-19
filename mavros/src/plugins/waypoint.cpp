/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017,2018 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_pull.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>

using std::chrono_literals::operator""ms;

namespace mavros {
namespace std_plugins {
using utils::enum_value;
using mavlink::common::MAV_CMD;
using mavlink::common::MAV_FRAME;
using MRES = mavlink::common::MAV_MISSION_RESULT;


class WaypointItem : public mavlink::common::msg::MISSION_ITEM {
public:
	double x_lat;
	double y_long;
	double z_alt;

	mavros_msgs::msg::Waypoint to_msg()
	{
		mavros_msgs::msg::Waypoint ret;

		// [[[cog:
		// waypoint_item_msg = [(v, v) if isinstance(v, str) else v for v in (
		//     'frame',
		//     'command',
		//     ('is_current', 'current'),
		//     'autocontinue',
		//     'param1',
		//     'param2',
		//     'param3',
		//     'param4',
		//     'x_lat',
		//     'y_long',
		//     'z_alt',
		// )]
		// for a, b in waypoint_item_msg:
		//     cog.outl("ret.%s = %s;" % (a, b))
		// ]]]
		ret.frame = frame;
		ret.command = command;
		ret.is_current = current;
		ret.autocontinue = autocontinue;
		ret.param1 = param1;
		ret.param2 = param2;
		ret.param3 = param3;
		ret.param4 = param4;
		ret.x_lat = x_lat;
		ret.y_long = y_long;
		ret.z_alt = z_alt;
		// [[[end]]] (checksum: 371710cb8984352c8cc1b93eb8b04a2b)

		return ret;
	}

	static WaypointItem from_msg(mavros_msgs::msg::Waypoint &wp, uint16_t seq)
	{
		WaypointItem ret{};

		// [[[cog:
		// waypoint_coords = [
		//     ('x_lat', 'x'),
		//     ('y_long', 'y'),
		//     ('z_alt', 'z'),
		// ]
		// for a, b in waypoint_item_msg + waypoint_coords:
		//     cog.outl("ret.%s = wp.%s;" % (b, a))
		// ]]]
		ret.frame = wp.frame;
		ret.command = wp.command;
		ret.current = wp.is_current;
		ret.autocontinue = wp.autocontinue;
		ret.param1 = wp.param1;
		ret.param2 = wp.param2;
		ret.param3 = wp.param3;
		ret.param4 = wp.param4;
		ret.x_lat = wp.x_lat;
		ret.y_long = wp.y_long;
		ret.z_alt = wp.z_alt;
		ret.x = wp.x_lat;
		ret.y = wp.y_long;
		ret.z = wp.z_alt;
		// [[[end]]] (checksum: e277c889ab7a67085562bbd014283a78)

		ret.seq = seq;
		ret.mission_type = enum_value(mavlink::common::MAV_MISSION_TYPE::MISSION);

		return ret;
	}

	std::string to_string()
	{
		//return to_yaml();

		return utils::format("#%u%1s F:%u C:%3u p: %f %f %f %f x: %f y: %f z: %f",
			seq,
			(current) ? "*" : "",
			frame, command,
			param1, param2, param3, param4,
			x_lat, y_long, z_alt);
	}
};


/**
 * @brief Mission manupulation plugin
 */
class WaypointPlugin : public plugin::PluginBase {
public:
	WaypointPlugin() : PluginBase(),
		logger(rclcpp::get_logger("mavros.wp")),
		wp_state(WP::IDLE),
		wp_count(0),
		wp_retries(RETRIES_COUNT),
		wp_cur_id(0),
		wp_cur_active(0),
		wp_set_active(0),
		is_timedout(false),
		do_pull_after_gcs(false),
		enable_partial_push(false),
		reschedule_pull(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		wp_nh = std::make_shared<rclcpp::Node>("mission", "mavros");

		wp_state = WP::IDLE;

		wp_nh->get_parameter_or("~/pull_after_gcs", do_pull_after_gcs, true);

		wp_list_pub = wp_nh->create_publisher<mavros_msgs::msg::WaypointList>("~/waypoints", 
			rclcpp::QoS(2).reliable().transient_local());
		wp_reached_pub = wp_nh->create_publisher<mavros_msgs::msg::WaypointReached>("~/reached", 
			rclcpp::QoS(10).reliable().transient_local());
		pull_srv = wp_nh->create_service<mavros_msgs::srv::WaypointPull>("~/pull", 
			std::bind(&WaypointPlugin::pull_cb, this, std::placeholders::_1, std::placeholders::_2));
		push_srv = wp_nh->create_service<mavros_msgs::srv::WaypointPush>("~/push", 
			std::bind(&WaypointPlugin::push_cb, this, std::placeholders::_1, std::placeholders::_2));
		clear_srv = wp_nh->create_service<mavros_msgs::srv::WaypointClear>("~/clear", 
			std::bind(&WaypointPlugin::clear_cb, this, std::placeholders::_1, std::placeholders::_2));
		set_cur_srv = wp_nh->create_service<mavros_msgs::srv::WaypointSetCurrent>("~/set_current", 
			std::bind(&WaypointPlugin::set_cur_cb, this, std::placeholders::_1, std::placeholders::_2));

		wp_timer = wp_nh->create_wall_timer(WP_TIMEOUT_MS, 
			std::bind(&WaypointPlugin::timeout_cb, this));
		wp_timer->cancel();
		schedule_timer = wp_nh->create_wall_timer(BOOTUP_TIME_MS, 
			std::bind(&WaypointPlugin::scheduled_pull_cb, this));
		schedule_timer->cancel();
		enable_connection_cb();
	}

	Subscriptions get_subscriptions() {
		return {
			       make_handler(&WaypointPlugin::handle_mission_item),
			       make_handler(&WaypointPlugin::handle_mission_request),
			       make_handler(&WaypointPlugin::handle_mission_current),
			       make_handler(&WaypointPlugin::handle_mission_count),
			       make_handler(&WaypointPlugin::handle_mission_item_reached),
			       make_handler(&WaypointPlugin::handle_mission_ack),
		};
	}

	rclcpp::Node::SharedPtr get_ros_node() override {
		return wp_nh;
	}

private:
	using unique_lock = std::unique_lock<std::recursive_mutex>;
	using lock_guard = std::lock_guard<std::recursive_mutex>;

	std::recursive_mutex mutex;
	rclcpp::Node::SharedPtr wp_nh;
	rclcpp::Logger logger;

	rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr wp_list_pub;
	rclcpp::Publisher<mavros_msgs::msg::WaypointReached>::SharedPtr wp_reached_pub;
	rclcpp::Service<mavros_msgs::srv::WaypointPull>::SharedPtr pull_srv;
	rclcpp::Service<mavros_msgs::srv::WaypointPush>::SharedPtr push_srv;
	rclcpp::Service<mavros_msgs::srv::WaypointClear>::SharedPtr clear_srv;
	rclcpp::Service<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr set_cur_srv;


	std::vector<WaypointItem> waypoints;
	std::vector<WaypointItem> send_waypoints;
	enum class WP {
		IDLE,
		RXLIST,
		RXWP,
		TXLIST,
		TXPARTIAL,
		TXWP,
		CLEAR,
		SET_CUR
	};
	WP wp_state;

	size_t wp_count;
	size_t wp_start_id;
	size_t wp_end_id;
	size_t wp_cur_id;
	size_t wp_cur_active;
	size_t wp_set_active;
	size_t wp_retries;
	bool is_timedout;
	std::mutex recv_cond_mutex;
	std::mutex send_cond_mutex;
	std::condition_variable list_receiving;
	std::condition_variable list_sending;

	rclcpp::TimerBase::SharedPtr wp_timer;
	rclcpp::TimerBase::SharedPtr schedule_timer;
	bool do_pull_after_gcs;
	bool enable_partial_push;

	bool reschedule_pull;


	static constexpr std::chrono::milliseconds BOOTUP_TIME_MS = 15000ms;	//! system startup delay before start pull
	static constexpr std::chrono::milliseconds LIST_TIMEOUT_MS = 30000ms;	//! Timeout for pull/push operations
	static constexpr std::chrono::milliseconds WP_TIMEOUT_MS = 1000ms;
	static constexpr std::chrono::milliseconds RESCHEDULE_MS = 5000ms;
	static constexpr int RETRIES_COUNT = 3;

	/* -*- rx handlers -*- */

	/**
	 * @brief handle MISSION_ITEM mavlink msg
	 * handles and stores mission items when pulling waypoints
	 * @param msg		Received Mavlink msg
	 * @param wpi		WaypointItem from msg
	 */
	void handle_mission_item(const mavlink::mavlink_message_t *msg, WaypointItem &wpi)
	{
		unique_lock lock(mutex);

		// WaypointItem has wider fields for Lat/Long/Alt, set it
		// [[[cog:
		// for a, b in waypoint_coords:
		//     cog.outl("wpi.%s = wpi.%s;" % (a, b))
		// ]]]
		wpi.x_lat = wpi.x;
		wpi.y_long = wpi.y;
		wpi.z_alt = wpi.z;
		// [[[end]]] (checksum: b8f95ce9c7c9dbd4eb493bf1227f273f)

		/* receive item only in RX state */
		if (wp_state == WP::RXWP) {
			if (wpi.seq != wp_cur_id) {
				RCLCPP_WARN(logger, "WP: Seq mismatch, dropping item (%d != %zu)",
					wpi.seq, wp_cur_id);
				return;
			}

			RCLCPP_INFO_STREAM(logger, "WP: item " << wpi.to_string());

			waypoints.push_back(wpi);
			if (++wp_cur_id < wp_count) {
				restart_timeout_timer();
				mission_request(wp_cur_id);
			}
			else {
				request_mission_done();
				lock.unlock();
				publish_waypoints();
			}
		}
		else {
			RCLCPP_DEBUG(logger, "WP: rejecting item, wrong state %d", enum_value(wp_state));
			if (do_pull_after_gcs && reschedule_pull) {
				RCLCPP_DEBUG(logger, "WP: reschedule pull");
				schedule_pull(WP_TIMEOUT_MS);
			}
		}
	}

	/**
	 * @brief handle MISSION_REQUEST mavlink msg
	 * handles and acts on misison request from FCU
	 * @param msg		Received Mavlink msg
	 * @param mreq		MISSION_REQUEST from msg
	 */
	void handle_mission_request(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_REQUEST &mreq)
	{
		lock_guard lock(mutex);

		if ((wp_state == WP::TXLIST && mreq.seq == 0) || (wp_state == WP::TXPARTIAL && mreq.seq == wp_start_id) || (wp_state == WP::TXWP)) {
			if (mreq.seq != wp_cur_id && mreq.seq != wp_cur_id + 1) {
				RCLCPP_WARN(logger, "WP: Seq mismatch, dropping request (%d != %zu)",
					mreq.seq, wp_cur_id);
				return;
			}

			restart_timeout_timer();
			if (mreq.seq < wp_end_id) {
				RCLCPP_DEBUG(logger, "WP: FCU requested waypoint %d", mreq.seq);
				wp_state = WP::TXWP;
				wp_cur_id = mreq.seq;
				send_waypoint(wp_cur_id);
			}
			else
				RCLCPP_ERROR(logger, "WP: FCU require seq out of range");
		}
		else
			RCLCPP_DEBUG(logger, "WP: rejecting request, wrong state %d", enum_value(wp_state));
	}

	/**
	 * @brief handle MISSION_CURRENT mavlink msg
	 * This confirms a SET_CUR action
	 * @param msg		Received Mavlink msg
	 * @param mcur		MISSION_CURRENT from msg
	 */
	void handle_mission_current(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_CURRENT &mcur)
	{
		unique_lock lock(mutex);

		if (wp_state == WP::SET_CUR) {
			/* MISSION_SET_CURRENT ACK */
			RCLCPP_DEBUG(logger, "WP: set current #%d done", mcur.seq);
			go_idle();
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
		}
		else if (wp_state == WP::IDLE && wp_cur_active != mcur.seq) {
			/* update active */
			RCLCPP_DEBUG(logger, "WP: update current #%d", mcur.seq);
			wp_cur_active = mcur.seq;
			set_current_waypoint(wp_cur_active);

			lock.unlock();
			publish_waypoints();
		}
	}

	/**
	 * @brief handle MISSION_COUNT mavlink msg
	 * Handles a mission count from FCU in a Waypoint Pull
	 * Triggers a pull GCS seems to be requesting mission
	 * @param msg		Received Mavlink msg
	 * @param mcnt		MISSION_COUNT from msg
	 */
	void handle_mission_count(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_COUNT &mcnt)
	{
		unique_lock lock(mutex);

		if (wp_state == WP::RXLIST) {
			/* FCU report of MISSION_REQUEST_LIST */
			RCLCPP_DEBUG(logger, "WP: count %d", mcnt.count);

			wp_count = mcnt.count;
			wp_cur_id = 0;

			waypoints.clear();
			waypoints.reserve(wp_count);

			if (wp_count > 0) {
				wp_state = WP::RXWP;
				restart_timeout_timer();
				mission_request(wp_cur_id);
			}
			else {
				request_mission_done();
				lock.unlock();
				publish_waypoints();
			}
		}
		else {
			RCLCPP_INFO(logger, "WP: seems GCS requesting mission");
			/* schedule pull after GCS done */
			if (do_pull_after_gcs) {
				RCLCPP_INFO(logger, "WP: scheduling pull after GCS is done");
				reschedule_pull = true;
				schedule_pull(RESCHEDULE_MS);
			}
		}
	}

	/**
	 * @brief handle MISSION_ITEM_REACHED mavlink msg
	 * @param msg		Received Mavlink msg
	 * @param mitr		MISSION_ITEM_REACHED from msg
	 */
	void handle_mission_item_reached(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ITEM_REACHED &mitr)
	{
		/* in QGC used as informational message */
		RCLCPP_INFO(logger, "WP: reached #%d", mitr.seq);

		auto wpr = std::make_unique<mavros_msgs::msg::WaypointReached>();

		wpr->header.stamp = wp_nh->get_clock()->now();
		wpr->wp_seq = mitr.seq;

		wp_reached_pub->publish(std::move(wpr));
	}

	/**
	 * @brief handle MISSION_ACK mavlink msg
	 * Handles a MISSION_ACK which marks the end of a push, or a failure
	 * @param msg		Received Mavlink msg
	 * @param mack		MISSION_ACK from msg
	 */
	void handle_mission_ack(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MISSION_ACK &mack)
	{
		unique_lock lock(mutex);

		auto ack_type = static_cast<MRES>(mack.type);

		if ((wp_state == WP::TXLIST || wp_state == WP::TXPARTIAL || wp_state == WP::TXWP)
			&& (wp_cur_id == wp_end_id - 1)
			&& (ack_type == MRES::ACCEPTED)) {
			go_idle();
			waypoints = send_waypoints;
			send_waypoints.clear();

			lock.unlock();
			list_sending.notify_all();
			publish_waypoints();
			RCLCPP_INFO(logger, "WP: mission sended");
		}
		else if (wp_state == WP::TXWP && ack_type == MRES::INVALID_SEQUENCE) {
			// Mission Ack: INVALID_SEQUENCE received during TXWP
			// This happens when waypoint N was received by autopilot, but the request for waypoint N+1 failed.
			// This causes seq mismatch, ignore and eventually the request for n+1 will get to us and seq will sync up.
			RCLCPP_DEBUG(logger, "WP: Received INVALID_SEQUENCE ack");
		}
		else if (wp_state == WP::TXLIST || wp_state == WP::TXPARTIAL || wp_state == WP::TXWP) {
			go_idle();
			/* use this flag for failure report */
			is_timedout = true;
			lock.unlock();
			list_sending.notify_all();

			RCLCPP_ERROR_STREAM(logger, "WP: upload failed: " << utils::to_string(ack_type));
		}
		else if (wp_state == WP::CLEAR) {
			go_idle();
			if (ack_type != MRES::ACCEPTED) {
				is_timedout = true;
				lock.unlock();
				RCLCPP_ERROR_STREAM(logger, "WP: clear failed: " << utils::to_string(ack_type));
			}
			else {
				waypoints.clear();
				lock.unlock();
				publish_waypoints();
				RCLCPP_INFO(logger, "WP: mission cleared");
			}

			list_sending.notify_all();
		}
		else
			RCLCPP_DEBUG(logger, "WP: not planned ACK, type: %d", mack.type);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Act on a timeout
	 * Resend the message that may have been lost
	 */
	void timeout_cb()
	{
		unique_lock lock(mutex);
		if (wp_retries > 0) {
			wp_retries--;
			RCLCPP_WARN(logger, "WP: timeout, retries left %zu", wp_retries);

			switch (wp_state) {
			case WP::RXLIST:
				mission_request_list();
				break;
			case WP::RXWP:
				mission_request(wp_cur_id);
				break;
			case WP::TXLIST:
				mission_count(wp_count);
				break;
			case WP::TXPARTIAL:
				mission_write_partial_list(wp_start_id, wp_end_id);
				break;
			case WP::TXWP:
				send_waypoint(wp_cur_id);
				break;
			case WP::CLEAR:
				mission_clear_all();
				break;
			case WP::SET_CUR:
				mission_set_current(wp_set_active);
				break;

			case WP::IDLE:
				break;
			}

			restart_timeout_timer_int();
		}
		else {
			RCLCPP_ERROR(logger, "WP: timed out.");
			go_idle();
			is_timedout = true;
			/* prevent waiting cond var timeout */
			lock.unlock();
			list_receiving.notify_all();
			list_sending.notify_all();
		}
	}

	// Act on first heartbeat from FCU
	void connection_cb(bool connected) override
	{
		lock_guard lock(mutex);
		if (connected) {
			schedule_pull(BOOTUP_TIME_MS);

			if (wp_nh->has_parameter("enable_partial_push")) {
				wp_nh->get_parameter("enable_partial_push", enable_partial_push);
			}
			else {
				enable_partial_push = m_uas->is_ardupilotmega();
			}
		}
		else {
			schedule_timer->cancel();
		}
	}

	//! @brief Callback for scheduled waypoint pull
	void scheduled_pull_cb()
	{
		lock_guard lock(mutex);
		if (wp_state != WP::IDLE) {
			/* try later */
			RCLCPP_DEBUG(logger, "WP: busy, reschedule pull");
			schedule_pull(RESCHEDULE_MS);
			return;
		}

		RCLCPP_DEBUG(logger, "WP: start scheduled pull");
		wp_state = WP::RXLIST;
		wp_count = 0;
		restart_timeout_timer();
		mission_request_list();
	}

	//! @brief Send ACK back to FCU after pull
	void request_mission_done(void)
	{
		/* possibly not needed if count == 0 (QGC impl) */
		mission_ack(MRES::ACCEPTED);

		go_idle();
		list_receiving.notify_all();
		RCLCPP_INFO(logger, "WP: mission received");
	}

	void go_idle(void)
	{
		reschedule_pull = false;
		wp_state = WP::IDLE;
		wp_timer->cancel();
	}

	void restart_timeout_timer(void)
	{
		wp_retries = RETRIES_COUNT;
		restart_timeout_timer_int();
	}

	void restart_timeout_timer_int(void)
	{
		is_timedout = false;
		wp_timer->reset();
	}

	void schedule_pull(std::chrono::milliseconds dt)
	{
		schedule_timer = wp_nh->create_wall_timer(dt, 
			std::bind(&WaypointPlugin::scheduled_pull_cb, this));
	}

	//! @brief send a single waypoint to FCU
	void send_waypoint(size_t seq)
	{
		if (seq < send_waypoints.size()) {
			auto wpi = send_waypoints.at(seq);
			mission_item(wpi);

			RCLCPP_DEBUG_STREAM(logger, "WP: send item " << wpi.to_string());
		}
	}

	/**
	 * @brief wait until a waypoint pull is complete.
	 * Pull happens asyncronously, this function blocks until it is done.
	 */
	bool wait_fetch_all()
	{
		std::unique_lock<std::mutex> lock(recv_cond_mutex);
		return list_receiving.wait_for(lock, LIST_TIMEOUT_MS)
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	/**
	 * @brief wait until a waypoint push is complete.
	 * Push happens asyncronously, this function blocks until it is done.
	 */
	bool wait_push_all()
	{
		std::unique_lock<std::mutex> lock(send_cond_mutex);

		return list_sending.wait_for(lock, LIST_TIMEOUT_MS)
		       == std::cv_status::no_timeout
		       && !is_timedout;
	}

	//! @brief set the FCU current waypoint
	void set_current_waypoint(size_t seq)
	{
		for (auto &it : waypoints)
			it.current = (it.seq == seq) ? true : false;
	}

	//! @brief publish the updated waypoint list after operation
	void publish_waypoints()
	{
		auto wpl = std::make_unique<mavros_msgs::msg::WaypointList>();
		unique_lock lock(mutex);

		wpl->current_seq = wp_cur_active;
		wpl->waypoints.clear();
		wpl->waypoints.reserve(waypoints.size());
		for (auto &it : waypoints) {
			wpl->waypoints.push_back(it.to_msg());
		}

		lock.unlock();
		wp_list_pub->publish(std::move(wpl));
	}

	/* -*- low-level send functions -*- */

	void mission_item(WaypointItem &wp)
	{
		m_uas->msg_set_target(wp);
		// WaypointItem may be sent as MISSION_ITEM
		UAS_FCU(m_uas)->send_message_ignore_drop(wp);
	}

	void mission_request(uint16_t seq)
	{
		RCLCPP_DEBUG(logger, "WP:m: request #%u", seq);

		mavlink::common::msg::MISSION_REQUEST mrq {};
		m_uas->msg_set_target(mrq);
		mrq.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(mrq);
	}

	void mission_set_current(uint16_t seq)
	{
		RCLCPP_DEBUG(logger, "WP:m: set current #%u", seq);

		mavlink::common::msg::MISSION_SET_CURRENT msc {};
		m_uas->msg_set_target(msc);
		msc.seq = seq;

		UAS_FCU(m_uas)->send_message_ignore_drop(msc);
	}

	void mission_request_list()
	{
		RCLCPP_DEBUG(logger, "WP:m: request list");

		mavlink::common::msg::MISSION_REQUEST_LIST mrl {};
		m_uas->msg_set_target(mrl);

		UAS_FCU(m_uas)->send_message_ignore_drop(mrl);
	}

	void mission_count(uint16_t cnt)
	{
		RCLCPP_DEBUG(logger, "WP:m: count %u", cnt);

		mavlink::common::msg::MISSION_COUNT mcnt {};
		m_uas->msg_set_target(mcnt);
		mcnt.count = cnt;

		UAS_FCU(m_uas)->send_message_ignore_drop(mcnt);
	}

	void mission_write_partial_list(uint16_t start_index, uint16_t end_index)
	{
		RCLCPP_DEBUG(logger, "WP:m: write partial list %u - %u", start_index, end_index);

		mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST mwpl {};
		mwpl.start_index = start_index;
		mwpl.end_index = end_index;
		m_uas->msg_set_target(mwpl);

		UAS_FCU(m_uas)->send_message_ignore_drop(mwpl);
	}

	void mission_clear_all()
	{
		RCLCPP_DEBUG(logger, "WP:m: clear all");

		mavlink::common::msg::MISSION_CLEAR_ALL mclr {};
		m_uas->msg_set_target(mclr);

		UAS_FCU(m_uas)->send_message_ignore_drop(mclr);
	}

	void mission_ack(MRES type) {
		RCLCPP_DEBUG(logger, "WP:m: ACK %u", enum_value(type));

		mavlink::common::msg::MISSION_ACK mack {};
		m_uas->msg_set_target(mack);
		mack.type = enum_value(type);

		UAS_FCU(m_uas)->send_message_ignore_drop(mack);
	}

	/* -*- ROS callbacks -*- */

	bool pull_cb(mavros_msgs::srv::WaypointPull::Request::SharedPtr req,
		mavros_msgs::srv::WaypointPull::Response::SharedPtr res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		wp_state = WP::RXLIST;
		wp_count = 0;
		restart_timeout_timer();

		lock.unlock();
		mission_request_list();
		res->success = wait_fetch_all();
		lock.lock();

		res->wp_received = waypoints.size();
		go_idle();	// not nessessary, but prevents from blocking
		return true;
	}

	bool push_cb(mavros_msgs::srv::WaypointPush::Request::SharedPtr req,
		mavros_msgs::srv::WaypointPush::Response::SharedPtr res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			// Wrong initial state, other operation in progress?
			return false;

		if (req->start_index) {
			// Partial Waypoint update

			if (!enable_partial_push) {
				RCLCPP_WARN(logger, "WP: Partial Push not enabled. (Only supported on APM)");
				res->success = false;
				res->wp_transfered = 0;
				return true;
			}

			if (waypoints.size() < req->start_index + req->waypoints.size()) {
				RCLCPP_WARN(logger, "WP: Partial push out of range rejected.");
				res->success = false;
				res->wp_transfered = 0;
				return true;
			}

			wp_state = WP::TXPARTIAL;
			send_waypoints = waypoints;

			uint16_t seq = req->start_index;
			for (auto &it : req->waypoints) {
				send_waypoints[seq] = WaypointItem::from_msg(it, seq);
				seq++;
			}

			wp_count = req->waypoints.size();
			wp_start_id = req->start_index;
			wp_end_id = req->start_index + wp_count;
			wp_cur_id = req->start_index;
			restart_timeout_timer();

			lock.unlock();
			mission_write_partial_list(wp_start_id, wp_end_id);
			res->success = wait_push_all();
			lock.lock();

			res->wp_transfered = wp_cur_id - wp_start_id + 1;
		}
		else {
			// Full waypoint update
			wp_state = WP::TXLIST;

			send_waypoints.clear();
			send_waypoints.reserve(req->waypoints.size());
			uint16_t seq = 0;
			for (auto &it : req->waypoints) {
				send_waypoints.push_back(WaypointItem::from_msg(it, seq++));
			}

			wp_count = send_waypoints.size();
			wp_end_id = wp_count;
			wp_cur_id = 0;
			restart_timeout_timer();

			lock.unlock();
			mission_count(wp_count);
			res->success = wait_push_all();
			lock.lock();

			res->wp_transfered = wp_cur_id + 1;
		}

		go_idle();	// same as in pull_cb
		return true;
	}

	bool clear_cb(mavros_msgs::srv::WaypointClear::Request::SharedPtr req,
		mavros_msgs::srv::WaypointClear::Response::SharedPtr res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			return false;

		wp_state = WP::CLEAR;
		restart_timeout_timer();

		lock.unlock();
		mission_clear_all();
		res->success = wait_push_all();

		lock.lock();
		go_idle();	// same as in pull_cb
		return true;
	}

	bool set_cur_cb(mavros_msgs::srv::WaypointSetCurrent::Request::SharedPtr req,
		mavros_msgs::srv::WaypointSetCurrent::Response::SharedPtr res)
	{
		unique_lock lock(mutex);

		if (wp_state != WP::IDLE)
			return false;

		wp_state = WP::SET_CUR;
		wp_set_active = req->wp_seq;
		restart_timeout_timer();

		lock.unlock();
		mission_set_current(wp_set_active);
		res->success = wait_push_all();

		lock.lock();
		go_idle();	// same as in pull_cb
		return true;
	}
};

constexpr std::chrono::milliseconds WaypointPlugin::BOOTUP_TIME_MS;
constexpr std::chrono::milliseconds WaypointPlugin::LIST_TIMEOUT_MS;
constexpr std::chrono::milliseconds WaypointPlugin::WP_TIMEOUT_MS;
constexpr std::chrono::milliseconds WaypointPlugin::RESCHEDULE_MS;
constexpr int WaypointPlugin::RETRIES_COUNT;

}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WaypointPlugin, mavros::plugin::PluginBase)
