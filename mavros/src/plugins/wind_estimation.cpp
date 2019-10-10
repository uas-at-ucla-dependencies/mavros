/**
 * @brief Wind estimation plugin
 * @file wind_estimation.cpp
 * @author Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Thomas Stastny <thomas.stastny@mavt.ethz.ch>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace mavros {
namespace std_plugins {
/**
 * @brief Wind estimation plugin.
 */
class WindEstimationPlugin : public plugin::PluginBase {
public:
	WindEstimationPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		wind_pub = nh->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("wind_estimation", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&WindEstimationPlugin::handle_apm_wind),
			       make_handler(&WindEstimationPlugin::handle_px4_wind),
		};
	}

private:
	rclcpp::Node::SharedPtr nh;

	rclcpp::Publisher<>::SharedPtr wind_pub;

	/**
	 * Handle APM specific wind estimation message
	 */
	void handle_apm_wind(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::WIND &wind)
	{
		const double speed = wind.speed;
		const double course = -angles::from_degrees(wind.direction);	// direction "from" -> direction "to"

		auto twist_cov = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = rclcpp::Time::now();
		// TODO: check math's
		twist_cov->twist.twist.linear.x = speed * std::sin(course);	// E
		twist_cov->twist.twist.linear.y = speed * std::cos(course);	// N
		twist_cov->twist.twist.linear.z = -wind.speed_z;// D -> U

		// covariance matrix unknown in APM msg
		ftf::EigenMapCovariance6d cov_map(twist_cov->twist.covariance.data());
		cov_map.setZero();
		cov_map(0, 0) = -1.0;

		wind_pub.publish(twist_cov);
	}

	/**
	 * Handle PX4 specific wind estimation message
	 */
	void handle_px4_wind(const mavlink::mavlink_message_t *msg, mavlink::common::msg::WIND_COV &wind)
	{
		auto twist_cov = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = m_uas->synchronise_stamp(wind.time_usec);

		tf::vectorEigenToMsg(ftf::transform_frame_ned_enu(Eigen::Vector3d(wind.wind_x, wind.wind_y, wind.wind_z)),
					twist_cov->twist.twist.linear);

		// fill available covariance elements
		ftf::EigenMapCovariance6d cov_map(twist_cov->twist.covariance.data());
		cov_map.setZero();
		cov_map(0, 0) = wind.var_horiz;	// NOTE: this is a summed covariance for both x and y horizontal wind components
		cov_map(2, 2) = wind.var_vert;

		wind_pub.publish(twist_cov);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WindEstimationPlugin, mavros::plugin::PluginBase)
