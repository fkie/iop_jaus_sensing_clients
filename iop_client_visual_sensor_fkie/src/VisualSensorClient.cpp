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

#include <iop_builder_fkie/util.h>
#include <iop_component_fkie/iop_config.h>
#include <iop_client_visual_sensor_fkie/VisualSensorClient.h>
#include <limits>
#include <ros/console.h>
#include <string>


using namespace iop;
using namespace urn_jaus_jss_environmentSensing_VisualSensorClient;

VisualSensorClient::VisualSensorClient(jUnsignedShortInteger id)
{
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

VisualSensorClient::VisualSensorClient(CapabilityRec cap)
{
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
		iop::Config cmpcfg("~VisualSensorClient");
		if (cap.isZoomModesValid()) {
			if (cap.getZoomModes()->getNone() == 0) {
				p_zoomable = true;
				p_pub_zoom_level = cmpcfg.advertise<std_msgs::Float32>(idstr + "/zoom_level", 5, true);
				p_sub_zoom_level = cmpcfg.subscribe<std_msgs::Float32>(idstr + "/cmd_zoom_level", 5, &VisualSensorClient::p_ros_zoom_level, this);
			}
		}
		ROS_INFO_NAMED("VisualSensorClient", "[visual sensor %d] zoomable: %d", p_id, p_zoomable);
		if (cap.isSupportedStatesValid()) {
			if ((cap.getSupportedStates()->getOff() == 1 || cap.getSupportedStates()->getStandby() == 1) && cap.getSupportedStates()->getActive() == 1) {
				p_switchable = true;
				p_pub_state = cmpcfg.advertise<std_msgs::Bool>(idstr + "/pwr_state", 5, true);
				p_sub_state = cmpcfg.subscribe<std_msgs::Bool>(idstr + "/cmd_pwr_state", 5, &VisualSensorClient::p_ros_state, this);
			}
		}
		ROS_INFO_NAMED("VisualSensorClient", "[visual sensor %d] switchable: %d", p_id, p_switchable);
		p_pub_fov_horizontal = cmpcfg.advertise<std_msgs::Float32>(idstr + "/fov_horizontal", 5, true);
		p_pub_fov_vertical = cmpcfg.advertise<std_msgs::Float32>(idstr + "/fov_vertical", 5, true);
	}
}

void VisualSensorClient::apply_configuration(ConfigurationRec conf)
{
	if (p_id != conf.getSensorID())
		return;
	if (conf.isHorizontalFieldOfViewValid()) {
		p_fov_horizontal = conf.getHorizontalFieldOfView();
		std_msgs::Float32 ros_msg;
		ros_msg.data = p_fov_horizontal;
		p_pub_fov_horizontal.publish(ros_msg);
	}
	if (conf.isVerticalFieldOfViewValid()) {
		p_fov_vertical = conf.getVerticalFieldOfView();
		std_msgs::Float32 ros_msg;
		ros_msg.data = p_fov_vertical;
		p_pub_fov_vertical.publish(ros_msg);
	}
	if (conf.isZoomLevelValid() && is_zoomable()) {
		p_zoom_level = pround(conf.getZoomLevel());
		std_msgs::Float32 ros_msg;
		ros_msg.data = p_zoom_level;
		p_pub_zoom_level.publish(ros_msg);
	}
	if (conf.isSensorStateValid() && is_switchable()) {
		p_switch_state = (conf.getSensorState() == 0);
		std_msgs::Bool ros_msg;
		ros_msg.data = p_switch_state;
		p_pub_state.publish(ros_msg);
	}
}

void VisualSensorClient::apply_geometric(GeometricSeq geo)
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

void VisualSensorClient::p_ros_state(const std_msgs::Bool::ConstPtr& state)
{
	p_switch_state = state ->data;
	if (p_state_callback) {
		p_state_callback(p_id, get_configuration());
	}
}

void VisualSensorClient::p_ros_zoom_level(const std_msgs::Float32::ConstPtr& msg)
{
	p_zoom_level = msg->data;
	if (p_state_callback) {
		p_state_callback(p_id, get_configuration());
	}
}
