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


#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <boost/thread/recursive_mutex.hpp>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>

#include "RangeSensorClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_environmentSensing_RangeSensorClient
{

class DllExport RangeSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface
{
public:
	RangeSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~RangeSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleConfirmSensorConfigurationAction(ConfirmSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorCapabilitiesAction(ReportRangeSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorCompressedDataAction(ReportRangeSensorCompressedData msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorConfigurationAction(ReportRangeSensorConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportRangeSensorDataAction(ReportRangeSensorData msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportSensorGeometricPropertiesAction(ReportSensorGeometricProperties msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

	RangeSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	QueryRangeSensorConfiguration p_query_cfg;
	QueryRangeSensorData p_query_sensor_data;
	QuerySensorGeometricProperties p_query_geo;
	QueryRangeSensorCapabilities p_query_cap;


	ros::NodeHandle p_nh;
	ros::NodeHandle p_pnh;
	ros::Timer p_query_timer;
	std::string p_tf_frame_robot;
	tf2_ros::TransformBroadcaster p_tf_broadcaster;
	std::map<unsigned int, geometry_msgs::TransformStamped> p_tf_map;
	std::map<unsigned int, ros::Publisher> p_publisher_map;
	std::map<unsigned int, std::string> p_sensor_names;
	std::map<unsigned int, int> p_sensor_max_range;
	std::map<unsigned int, int> p_sensor_min_range;
	int p_query_state;
	bool p_by_query;

	JausAddress p_remote_addr;
	bool p_has_access;
	void pHandleeventReportRangeSensorDataAction(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata);
	void pQueryCallback(const ros::TimerEvent& event);
};

};

#endif // RANGESENSORCLIENT_RECEIVEFSM_H
