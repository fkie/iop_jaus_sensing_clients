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


#ifndef RANGESENSORCLIENT_RECEIVEFSM_H
#define RANGESENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_RangeSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_RangeSensorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"


#include "RangeSensorClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_environmentSensing_RangeSensorClient
{

class DllExport RangeSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	RangeSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~RangeSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleConfirmSensorConfigurationAction(ConfirmSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorCapabilitiesAction(ReportRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorCompressedDataAction(ReportRangeSensorCompressedData msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorConfigurationAction(ReportRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorDataAction(ReportRangeSensorData msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);

	RangeSensorClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	QueryRangeSensorConfiguration p_query_cfg;
	QueryRangeSensorData p_query_sensor_data;
	QuerySensorGeometricProperties p_query_geo;
	QueryRangeSensorCapabilities p_query_cap;


	std::string p_tf_frame_robot;
	tf2_ros::TransformBroadcaster p_tf_broadcaster;
	std::map<unsigned int, geometry_msgs::msg::TransformStamped> p_tf_map;
	std::map<unsigned int, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr > p_publisher_map;
	std::map<unsigned int, std::string> p_sensor_names;
	std::map<unsigned int, int> p_sensor_max_range;
	std::map<unsigned int, int> p_sensor_min_range;
	int p_query_state;
	double p_hz;
};

}

#endif // RANGESENSORCLIENT_RECEIVEFSM_H
