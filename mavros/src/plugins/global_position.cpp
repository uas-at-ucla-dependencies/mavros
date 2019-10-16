/**
 * @brief Global Position plugin
 * @file global_position.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2017 Nuno Marques.
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>
#include <GeographicLib/Geocentric.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>

#include <mavros_msgs/msg/home_position.hpp>

namespace mavros {
namespace std_plugins {
/**
 * @brief Global position plugin.
 *
 * Publishes global position. Conversion from GPS LLA to ECEF allows
 * publishing local position to TF and PoseWithCovarianceStamped.
 *
 */
class GlobalPositionPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	GlobalPositionPlugin() : PluginBase(),
		tf_send(false),
		rot_cov(99999.0),
		use_relative_alt(true),
		is_map_init(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		gp_nh = uas_.mavros_node->create_sub_node("global_position"),

		// general params
		frame_id = gp_nh->declare_parameter<std::string>("frame_id", "map");
		child_frame_id = gp_nh->declare_parameter<std::string>("child_frame_id", "base_link");
		rot_cov = gp_nh->declare_parameter("rot_covariance", 99999.0);
		gps_uere = gp_nh->declare_parameter("gps_uere", 1.0);
		use_relative_alt = gp_nh->declare_parameter("use_relative_alt", true);
		// tf subsection
		tf_send = gp_nh->declare_parameter("tf/send", false);
		tf_frame_id = gp_nh->declare_parameter<std::string>("tf/frame_id", "map");
		tf_global_frame_id = gp_nh->declare_parameter<std::string>("tf/global_frame_id", "earth");	// The global_origin should be represented as "earth" coordinate frame (ECEF) (REP 105)
		tf_child_frame_id = gp_nh->declare_parameter<std::string>("tf/child_frame_id", "base_link");

		UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		// gps data
		raw_fix_pub = gp_nh->create_publisher<sensor_msgs::msg::NavSatFix>("raw/fix", 10);
		raw_vel_pub = gp_nh->create_publisher<geometry_msgs::msg::TwistStamped>("raw/gps_vel", 10);
		raw_sat_pub = gp_nh->create_publisher<std_msgs::msg::UInt32>("raw/satellites", 10);

		// fused global position
		gp_fix_pub = gp_nh->create_publisher<sensor_msgs::msg::NavSatFix>("global", 10);
		gp_odom_pub = gp_nh->create_publisher<nav_msgs::msg::Odometry>("local", 10);
		gp_rel_alt_pub = gp_nh->create_publisher<std_msgs::msg::Float64>("rel_alt", 10);
		gp_hdg_pub = gp_nh->create_publisher<std_msgs::msg::Float64>("compass_hdg", 10);

		// global origin
		gp_global_origin_pub = gp_nh->create_publisher<geographic_msgs::msg::GeoPointStamped>("gp_origin", 10);
		gp_set_global_origin_sub = gp_nh->create_subscription<geographic_msgs::msg::GeoPointStamped>("set_gp_origin", 10, std::bind(&GlobalPositionPlugin::set_gp_origin_cb, this, std::placeholders::_1));

		// home position subscriber to set "map" origin
		// TODO use UAS
		hp_sub = gp_nh->create_subscription<mavros_msgs::msg::HomePosition>("home", 10, std::bind(&GlobalPositionPlugin::home_position_cb, this, std::placeholders::_1));

		// offset from local position to the global origin ("earth")
		gp_global_offset_pub = gp_nh->create_publisher<geometry_msgs::msg::PoseStamped>("gp_lp_offset", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
				make_handler(&GlobalPositionPlugin::handle_gps_raw_int),
				// GPS_STATUS: there no corresponding ROS message, and it is not supported by APM
				make_handler(&GlobalPositionPlugin::handle_global_position_int),
				make_handler(&GlobalPositionPlugin::handle_gps_global_origin),
				make_handler(&GlobalPositionPlugin::handle_lpned_system_global_offset)
		};
	}

private:
	rclcpp::Node::SharedPtr gp_nh;

	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr raw_fix_pub;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_vel_pub;
	rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr raw_sat_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gp_odom_pub;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gp_fix_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gp_hdg_pub;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gp_rel_alt_pub;
	rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_global_origin_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gp_global_offset_pub;

	rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_set_global_origin_sub;
	rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr hp_sub;

	std::string frame_id;		//!< origin frame for topic headers
	std::string child_frame_id;	//!< body-fixed frame for topic headers
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_global_frame_id;	//!< global origin for TF
	std::string tf_child_frame_id;	//!< frame for TF and Pose

	bool tf_send;
	bool use_relative_alt;
	bool is_map_init;

	double rot_cov;
	double gps_uere;

	Eigen::Vector3d map_origin {};	//!< geodetic origin of map frame [lla]
	Eigen::Vector3d ecef_origin {};	//!< geocentric origin of map frame [m]
	Eigen::Vector3d local_ecef {};	//!< local ECEF coordinates on map frame [m]

	template<typename MsgT>
	inline void fill_lla(MsgT &msg, sensor_msgs::msg::NavSatFix::Ptr fix)
	{
		fix->latitude = msg.lat / 1E7;		// deg
		fix->longitude = msg.lon / 1E7;		// deg
		fix->altitude = msg.alt / 1E3 + m_uas->geoid_to_ellipsoid_height(fix);	// in meters
	}

	inline void fill_unknown_cov(sensor_msgs::msg::NavSatFix::Ptr fix)
	{
		fix->position_covariance.fill(0.0);
		fix->position_covariance[0] = -1.0;
		fix->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
	}

	/* -*- message handlers -*- */

	void handle_gps_raw_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RAW_INT &raw_gps)
	{
		auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();

		fix->header = m_uas->synchronized_header(child_frame_id, raw_gps.time_usec);

		fix->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
		if (raw_gps.fix_type > 2)
			fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
		else {
			RCUTILS_LOG_WARN_THROTTLE_NAMED(,30, "global_position", "GP: No GPS fix");
			fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
		}

		fill_lla(raw_gps, fix);

		float eph = (raw_gps.eph != UINT16_MAX) ? raw_gps.eph / 1E2F : NAN;
		float epv = (raw_gps.epv != UINT16_MAX) ? raw_gps.epv / 1E2F : NAN;

		ftf::EigenMapCovariance3d gps_cov(fix->position_covariance.data());

		// With mavlink v2.0 use accuracies reported by sensor
		if (msg->magic == MAVLINK_STX &&
				raw_gps.h_acc > 0 && raw_gps.v_acc > 0) {
			gps_cov.diagonal() << std::pow(raw_gps.h_acc / 1E3, 2), std::pow(raw_gps.h_acc / 1E3, 2), std::pow(raw_gps.v_acc / 1E3, 2);
			fix->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
		}
		// With mavlink v1.0 approximate accuracies by DOP
		else if (!std::isnan(eph) && !std::isnan(epv)) {
			gps_cov.diagonal() << std::pow(eph * gps_uere, 2), std::pow(eph * gps_uere, 2), std::pow(epv * gps_uere, 2);
			fix->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		}
		else {
			fill_unknown_cov(fix);
		}

		// store & publish
		m_uas->update_gps_fix_epts(fix, eph, epv, raw_gps.fix_type, raw_gps.satellites_visible);
		raw_fix_pub->publish(*fix);

		if (raw_gps.vel != UINT16_MAX &&
					raw_gps.cog != UINT16_MAX) {
			double speed = raw_gps.vel / 1E2;				// m/s
			double course = angles::from_degrees(raw_gps.cog / 1E2);	// rad

			auto vel = std::make_shared<geometry_msgs::msg::TwistStamped>();

			vel->header.stamp = fix->header.stamp;
			vel->header.frame_id = frame_id;

			// From nmea_navsat_driver
			vel->twist.linear.x = speed * std::sin(course);
			vel->twist.linear.y = speed * std::cos(course);

			raw_vel_pub->publish(*vel);
		}

		// publish satellite count
		auto sat_cnt = std::make_shared<std_msgs::msg::UInt32>();
		sat_cnt->data = raw_gps.satellites_visible;
		raw_sat_pub->publish(*sat_cnt);
	}

	void handle_gps_global_origin(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_GLOBAL_ORIGIN &glob_orig)
	{
		auto g_origin = std::make_shared<geographic_msgs::msg::GeoPointStamped>();
		// auto header = m_uas->synchronized_header(frame_id, glob_orig.time_boot_ms);	#TODO: requires Mavlink msg update

		g_origin->header.frame_id = tf_global_frame_id;
		g_origin->header.stamp = rclcpp::Clock().now();

		g_origin->position.latitude = glob_orig.latitude / 1E7;
		g_origin->position.longitude = glob_orig.longitude / 1E7;
		g_origin->position.altitude = glob_orig.altitude / 1E3 + m_uas->geoid_to_ellipsoid_height(&g_origin->position);	// convert height amsl to height above the ellipsoid

		try {
			/**
			 * @brief Conversion from geodetic coordinates (LLA) to ECEF (Earth-Centered, Earth-Fixed)
			 * Note: "earth" frame, in ECEF, of the global origin
			 */
			GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
				GeographicLib::Constants::WGS84_f());

			earth.Forward(g_origin->position.latitude, g_origin->position.longitude, g_origin->position.altitude,
				g_origin->position.latitude, g_origin->position.longitude, g_origin->position.altitude);

			gp_global_origin_pub->publish(*g_origin);
		}
		catch (const std::exception& e) {
			RCUTILS_LOG_INFO("GP: Caught exception: %s", e.what());
		}
	}

	/** @todo Handler for GLOBAL_POSITION_INT_COV */

	void handle_global_position_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GLOBAL_POSITION_INT &gpos)
	{
		auto odom = std::make_shared<nav_msgs::msg::Odometry>();
		auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
		auto relative_alt = std::make_shared<std_msgs::msg::Float64>();
		auto compass_heading = std::make_shared<std_msgs::msg::Float64>();

		auto header = m_uas->synchronized_header(child_frame_id, gpos.time_boot_ms);

		// Global position fix
		fix->header = header;

		fill_lla(gpos, fix);

		// fill GPS status fields using GPS_RAW data
		auto raw_fix = m_uas->get_gps_fix();
		if (raw_fix) {
			fix->status.service = raw_fix->status.service;
			fix->status.status = raw_fix->status.status;
			fix->position_covariance = raw_fix->position_covariance;
			fix->position_covariance_type = raw_fix->position_covariance_type;
		}
		else {
			// no GPS_RAW_INT -> fix status unknown
			fix->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
			fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

			// we don't know covariance
			fill_unknown_cov(fix);
		}

		relative_alt->data = gpos.relative_alt / 1E3;	// in meters
		compass_heading->data = (gpos.hdg != UINT16_MAX) ? gpos.hdg / 1E2 : NAN;	// in degrees

		/**
		 * @brief Global position odometry:
		 *
		 * X: spherical coordinate X-axis (meters)
		 * Y: spherical coordinate Y-axis (meters)
		 * Z: spherical coordinate Z-axis (meters)
		 * VX: latitude vel (m/s)
		 * VY: longitude vel (m/s)
		 * VZ: altitude vel (m/s)
		 * Angular rates: unknown
		 * Pose covariance: computed, with fixed diagonal
		 * Velocity covariance: unknown
		 */
		odom->header.stamp = header.stamp;
		odom->header.frame_id = frame_id;
		odom->child_frame_id = child_frame_id;

		// Linear velocity
		tf::vectorEigenToMsg(Eigen::Vector3d(gpos.vy, gpos.vx, gpos.vz) / 1E2,
					odom->twist.twist.linear);

		// Velocity covariance unknown
		ftf::EigenMapCovariance6d vel_cov_out(odom->twist.covariance.data());
		vel_cov_out.fill(0.0);
		vel_cov_out(0) = -1.0;

		// Current fix in ECEF
		Eigen::Vector3d map_point;

		try {
			/**
			 * @brief Conversion from geodetic coordinates (LLA) to ECEF (Earth-Centered, Earth-Fixed)
			 *
			 * Note: "ecef_origin" is the origin of "map" frame, in ECEF, and the local coordinates are
			 * in spherical coordinates, with the orientation in ENU (just like what is applied
			 * on Gazebo)
			 */
			GeographicLib::Geocentric map(GeographicLib::Constants::WGS84_a(),
						GeographicLib::Constants::WGS84_f());

			/**
			 * @brief Checks if the "map" origin is set.
			 * - If not, and the home position is also not received, it sets the current fix as the origin;
			 * - If the home position is received, it sets the "map" origin;
			 * - If the "map" origin is set, then it applies the rotations to the offset between the origin
			 * and the current local geocentric coordinates.
			 */
			// Current fix to ECEF
			map.Forward(fix->latitude, fix->longitude, fix->altitude,
						map_point.x(), map_point.y(), map_point.z());

			// Set the current fix as the "map" origin if it's not set
			if (!is_map_init && fix->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
				map_origin.x() = fix->latitude;
				map_origin.y() = fix->longitude;
				map_origin.z() = fix->altitude;

				ecef_origin = map_point; // Local position is zero
				is_map_init = true;
			}
		}
		catch (const std::exception& e) {
			RCUTILS_LOG_INFO("GP: Caught exception: %s", e.what());
		}

		// Compute the local coordinates in ECEF
		local_ecef = map_point - ecef_origin;
		// Compute the local coordinates in ENU
		tf::pointEigenToMsg(ftf::transform_frame_ecef_enu(local_ecef, map_origin), odom->pose.pose.position);

		/**
		 * @brief By default, we are using the relative altitude instead of the geocentric
		 * altitude, which is relative to the WGS-84 ellipsoid
		 */
		if (use_relative_alt)
			odom->pose.pose.position.z = relative_alt->data;

		odom->pose.pose.orientation = m_uas->get_attitude_orientation_enu();

		// Use ENU covariance to build XYZRPY covariance
		ftf::EigenMapConstCovariance3d gps_cov(fix->position_covariance.data());
		ftf::EigenMapCovariance6d pos_cov_out(odom->pose.covariance.data());
		pos_cov_out.setZero();
		pos_cov_out.block<3, 3>(0, 0) = gps_cov;
		pos_cov_out.block<3, 3>(3, 3).diagonal() <<
							rot_cov,
								rot_cov,
									rot_cov;

		// publish
		gp_fix_pub->publish(*fix);
		gp_odom_pub->publish(*odom);
		gp_rel_alt_pub->publish(*relative_alt);
		gp_hdg_pub->publish(*compass_heading);

		// TF
		if (tf_send) {
			geometry_msgs::msg::TransformStamped transform;

			transform.header.stamp = odom->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			// setRotation()
			transform.transform.rotation = odom->pose.pose.orientation;

			// setOrigin()
			transform.transform.translation.x = odom->pose.pose.position.x;
			transform.transform.translation.y = odom->pose.pose.position.y;
			transform.transform.translation.z = odom->pose.pose.position.z;

			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	void handle_lpned_system_global_offset(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET &offset)
	{
		auto global_offset = std::make_shared<geometry_msgs::msg::PoseStamped>();
		global_offset->header = m_uas->synchronized_header(tf_global_frame_id, offset.time_boot_ms);

		auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(offset.x, offset.y, offset.z));
		auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
					ftf::transform_orientation_ned_enu(
						ftf::quaternion_from_rpy(offset.roll, offset.pitch, offset.yaw)));

		tf::pointEigenToMsg(enu_position, global_offset->pose.position);
		tf::quaternionEigenToMsg(enu_baselink_orientation, global_offset->pose.orientation);

		gp_global_offset_pub->publish(*global_offset);

		// TF
		if (tf_send) {
			geometry_msgs::msg::TransformStamped transform;

			transform.header.stamp = global_offset->header.stamp;
			transform.header.frame_id = tf_global_frame_id;
			transform.child_frame_id = tf_frame_id;

			// setRotation()
			transform.transform.rotation = global_offset->pose.orientation;

			// setOrigin()
			transform.transform.translation.x = global_offset->pose.position.x;
			transform.transform.translation.y = global_offset->pose.position.y;
			transform.transform.translation.z = global_offset->pose.position.z;

			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	/* -*- diagnostics -*- */
	void gps_diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		int fix_type, satellites_visible;
		float eph, epv;

		m_uas->get_gps_epts(eph, epv, fix_type, satellites_visible);

		if (satellites_visible <= 0)
			stat.summary(2, "No satellites");
		else if (fix_type < 2)
			stat.summary(1, "No fix");
		else if (fix_type == 2)
			stat.summary(0, "2D fix");
		else if (fix_type >= 3)
			stat.summary(0, "3D fix");

		stat.addf("Satellites visible", "%zd", satellites_visible);
		stat.addf("Fix type", "%d", fix_type);

		if (!std::isnan(eph))
			stat.addf("EPH (m)", "%.2f", eph);
		else
			stat.add("EPH (m)", "Unknown");

		if (!std::isnan(epv))
			stat.addf("EPV (m)", "%.2f", epv);
		else
			stat.add("EPV (m)", "Unknown");
	}

	/* -*- callbacks -*- */

	void home_position_cb(const mavros_msgs::msg::HomePosition::SharedPtr req)
	{
		map_origin.x() = req->geo.latitude;
		map_origin.y() = req->geo.longitude;
		map_origin.z() = req->geo.altitude;

		try {
			/**
			 * @brief Conversion from geodetic coordinates (LLA) to ECEF (Earth-Centered, Earth-Fixed)
			 */
			GeographicLib::Geocentric map(GeographicLib::Constants::WGS84_a(),
						GeographicLib::Constants::WGS84_f());

			// map_origin to ECEF
			map.Forward(map_origin.x(), map_origin.y(), map_origin.z(),
						ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
		}
		catch (const std::exception& e) {
			RCUTILS_LOG_INFO("GP: Caught exception: %s", e.what());
		}

		is_map_init = true;
	}

	void set_gp_origin_cb(const geographic_msgs::msg::GeoPointStamped::SharedPtr req)
	{
		mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN gpo;

		Eigen::Vector3d global_position;

		gpo.target_system = m_uas->get_tgt_system();
		// gpo.time_boot_ms = stamp.toNSec() / 1000;	#TODO: requires Mavlink msg update

		gpo.latitude = req->position.latitude * 1E7;
		gpo.longitude = req->position.longitude * 1E7;
		gpo.altitude = req->position.altitude * 1E3 + m_uas->ellipsoid_to_geoid_height(&req->position);

		UAS_FCU(m_uas)->send_message_ignore_drop(gpo);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::GlobalPositionPlugin, mavros::plugin::PluginBase)
