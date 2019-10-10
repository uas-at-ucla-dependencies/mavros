/**
 * @brief MAVROS GCS proxy
 * @file gcs_bridge.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <rclcpp/rclcpp.hpp>

#include <mavros/utils.h>
#include <mavros/mavlink_diag.h>
#include <mavconn/interface.h>

using namespace mavros;
using namespace mavconn;

rclcpp::Logger logger = rclcpp::get_logger("gcs_bridge");
rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_pub;
rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_sub;
rclcpp::Clock::SharedPtr my_clock;
MAVConnInterface::Ptr gcs_link;


void mavlink_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
{
	mavros_msgs::msg::Mavlink rmsg;

	rmsg.header.stamp = my_clock->now();
	mavros_msgs::mavlink::convert(*mmsg, rmsg, mavros::utils::enum_value(framing));
	mavlink_pub->publish(rmsg);
}

void mavlink_sub_cb(const mavros_msgs::msg::Mavlink::UniquePtr rmsg)
{
	mavlink::mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		gcs_link->send_message(&mmsg);	// !!! queue exception -> fall of gcs_bridge. intentional.
	else
		RCLCPP_ERROR(logger, "Packet drop: convert error.");
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr mavlink_nh = rclcpp::Node::make_shared("gcs_bridge");
	my_clock = mavlink_nh->get_clock();
	diagnostic_updater::Updater updater(mavlink_nh, 0.5);
	mavros::MavlinkDiag gcs_link_diag("GCS bridge");

	std::string gcs_url;
	gcs_url = mavlink_nh->declare_parameter<std::string>("gcs_url", "udp://@");

	try {
		gcs_link = MAVConnInterface::open_url(gcs_url);
		gcs_link_diag.set_mavconn(gcs_link);
		gcs_link_diag.set_connection_status(true);
	}
	catch (mavconn::DeviceError &ex) {
		RCLCPP_FATAL(logger, "GCS: %s", ex.what());
		return 1;
	}

	mavlink_pub = mavlink_nh->create_publisher<mavros_msgs::msg::Mavlink>("~/to", 10);
	gcs_link->message_received_cb = mavlink_pub_cb;

	// prefer UDPROS, but allow TCPROS too
	mavlink_sub = mavlink_nh->create_subscription<mavros_msgs::msg::Mavlink>("~/from", 10, mavlink_sub_cb); //,
		// ros::TransportHints()
		// 	.unreliable().maxDatagramSize(1024)
		// 	.reliable());

	// setup updater
	updater.setHardwareID(gcs_url);
	updater.add(gcs_link_diag);

	rclcpp::spin(mavlink_nh);
	return 0;
}
