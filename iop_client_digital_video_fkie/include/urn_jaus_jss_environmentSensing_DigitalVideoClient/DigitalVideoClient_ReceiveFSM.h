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




#ifndef DIGITALVIDEOCLIENT_RECEIVEFSM_H
#define DIGITALVIDEOCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_environmentSensing_DigitalVideoClient/Messages/MessageSet.h"
#include "urn_jaus_jss_environmentSensing_DigitalVideoClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>


#include "DigitalVideoClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_environmentSensing_DigitalVideoClient
{

class DllExport DigitalVideoClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface
{
public:
	DigitalVideoClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~DigitalVideoClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportDigitalVideoSensorCapabilitiesAction(ReportDigitalVideoSensorCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportDigitalVideoSensorConfigurationAction(ReportDigitalVideoSensorConfiguration msg, Receive::Body::ReceiveRec transportData);


	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	/// Guard Methods



	DigitalVideoClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	JausAddress p_control_addr;
	ros::NodeHandle p_nh;
	ros::NodeHandle p_pnh;
	ros::Subscriber p_sub_cur_dv_id;
	unsigned short p_current_resource_id;

	void p_dandle_current_ressource_id(const std_msgs::UInt16::ConstPtr msg);

};

};

#endif // DIGITALVIDEOCLIENT_RECEIVEFSM_H
