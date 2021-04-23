/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#include <fkie_iop_builder/util.h>
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_client_visual_sensor/VisualSensorClient.h>
#include <limits>
#include <string>


using namespace iop;
using namespace urn_jaus_jss_environmentSensing_VisualSensorClient;

VisualSensorClient::VisualSensorClient(std::shared_ptr<iop::Component> cmp, jUnsignedShortInteger id)
: logger(cmp->get_logger().get_child("VisualSensorClient"))
{
	this->cmp = cmp;
	p_id = id;
	p_name = "";
	p_switchable = false;
	p_switch_state = true;
	p_zoomable = false;
	p_zoom_level = 1;
	p_joint_nr = std::numeric_limits<uint8_t>::max();
	p_pose_valid = false;
	p_fov_horizontal = std::numeric_limits<float>::max();
	p_fov_vertical = std::numeric_limits<float>::max();
}

VisualSensorClient::VisualSensorClient(std::shared_ptr<iop::Component> cmp, CapabilityRec cap)
: logger(cmp->get_logger().get_child("VisualSensorClient"))
{
	this->cmp = cmp;
	p_id = cap.getSensorID();
	p_name = cap.getSensorName();
	p_switchable = false;
	p_switch_state = true;
	p_zoomable = false;
	p_zoom_level = 1;
	p_joint_nr = std::numeric_limits<uint8_t>::max();
	p_pose_valid = false;
	p_fov_horizontal = std::numeric_limits<float>::max();
	p_fov_vertical = std::numeric_limits<float>::max();
	apply_capability(cap);
}

VisualSensorClient::~VisualSensorClient()
{
}

bool VisualSensorClient::operator==(VisualSensorClient &value)
{
	return p_id == value.p_id;
}

bool VisualSensorClient::operator!=(VisualSensorClient &value)
{
	return !(*this == value);
}

void VisualSensorClient::apply_capability(CapabilityRec cap)
{
	if (p_id == 0 || p_id == cap.getSensorID()) {
		std::stringstream ss;
		ss << (int)p_id;
		std::string idstr("sensor_");
		idstr += ss.str();
		iop::Config cmpcfg(cmp, "VisualSensorClient");
		if (cap.isZoomModesValid()) {
			if (cap.getZoomModes()->getNone() == 0) {
				p_zoomable = true;
				p_pub_zoom_level = cmpcfg.create_publisher<std_msgs::msg::Float32>(idstr + "/zoom_level", 5);
				p_sub_zoom_level = cmpcfg.create_subscription<std_msgs::msg::Float32>(idstr + "/cmd_zoom_level", 5, std::bind(&VisualSensorClient::p_ros_zoom_level, this, std::placeholders::_1));
			}
		}
		RCLCPP_INFO(logger, "[visual sensor %d] zoomable: %d", p_id, p_zoomable);
		if (cap.isSupportedStatesValid()) {
			if ((cap.getSupportedStates()->getOff() == 1 || cap.getSupportedStates()->getStandby() == 1) && cap.getSupportedStates()->getActive() == 1) {
				p_switchable = true;
				p_pub_state = cmpcfg.create_publisher<std_msgs::msg::Bool>(idstr + "/pwr_state", 5);
				p_sub_state = cmpcfg.create_subscription<std_msgs::msg::Bool>(idstr + "/cmd_pwr_state", 5, std::bind(&VisualSensorClient::p_ros_state, this, std::placeholders::_1));
			}
		}
		RCLCPP_INFO(logger, "[visual sensor %d] switchable: %d", p_id, p_switchable);
		p_pub_fov_horizontal = cmpcfg.create_publisher<std_msgs::msg::Float32>(idstr + "/fov_horizontal", 5);
		p_pub_fov_vertical = cmpcfg.create_publisher<std_msgs::msg::Float32>(idstr + "/fov_vertical", 5);
	}
}

void VisualSensorClient::apply_configuration(ConfigurationRec conf)
{
	if (p_id != conf.getSensorID())
		return;
	if (conf.isHorizontalFieldOfViewValid()) {
		p_fov_horizontal = conf.getHorizontalFieldOfView();
		auto ros_msg = std_msgs::msg::Float32();
		ros_msg.data = p_fov_horizontal;
		p_pub_fov_horizontal->publish(ros_msg);
	}
	if (conf.isVerticalFieldOfViewValid()) {
		p_fov_vertical = conf.getVerticalFieldOfView();
		auto ros_msg = std_msgs::msg::Float32();
		ros_msg.data = p_fov_vertical;
		p_pub_fov_vertical->publish(ros_msg);
	}
	if (conf.isZoomLevelValid() && is_zoomable()) {
		p_zoom_level = pround(conf.getZoomLevel());
		auto ros_msg = std_msgs::msg::Float32();
		ros_msg.data = p_zoom_level;
		p_pub_zoom_level->publish(ros_msg);
	}
	if (conf.isSensorStateValid() && is_switchable()) {
		p_switch_state = (conf.getSensorState() == 0);
		std_msgs::msg::Bool ros_msg;
		ros_msg.data = p_switch_state;
		p_pub_state->publish(ros_msg);
	}
}

void VisualSensorClient::apply_geometric(GeometricSeq /* geo */)
{
	// TODO: publish tf or pose of the camera
}

SetConfigurationRec VisualSensorClient::get_configuration()
{
	SetConfigurationRec result;
	result.setSensorID(p_id);
	if (is_switchable()) {
		if (p_switch_state) {
			result.setSensorState(0);
		} else {
			result.setSensorState(2);
		}
	}
	if (is_zoomable()) {
		result.setZoomLevel(p_zoom_level + 1.0);  // HACK: because the JAUS message transports a wrong value
		result.setZoomMode(0);
	}
	return result;
}

void VisualSensorClient::p_ros_state(const std_msgs::msg::Bool::SharedPtr state)
{
	p_switch_state = state ->data;
	if (p_state_callback) {
		p_state_callback(p_id, get_configuration());
	}
}

void VisualSensorClient::p_ros_zoom_level(const std_msgs::msg::Float32::SharedPtr msg)
{
	p_zoom_level = msg->data;
	if (p_state_callback) {
		p_state_callback(p_id, get_configuration());
	}
}
